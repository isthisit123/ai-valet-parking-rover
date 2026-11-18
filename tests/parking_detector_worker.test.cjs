const {test} = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const read = name => fs.readFileSync(path.join(__dirname, '../scripts', name), 'utf8');

function proxyFixture() {
    const workers = [], jobs = [];
    let constructionFails = false, auto = true;
    class OriginalDetector {constructor(modelPath) {this.modelPath = modelPath;}}
    class FakeWorker {
        constructor(url) {if (constructionFails) {constructionFails = false; throw new Error('construction failed');}
            assert.equal(url, '/parking-detection-worker.js'); workers.push(this);}
        postMessage(message, transfer) {
            jobs.push({message, transfer});
            if (message.type === 'init') queueMicrotask(() => this.onmessage({data: {id: message.id, ready: true}}));
            else if (auto) queueMicrotask(() => this.onmessage({data: {id: message.id, inference_valid: true,
                detections: [0.35, 0.48, 0.6999, 0.70, 0.84].map(confidence => ({confidence}))}}));
        }
        terminate() {this.terminated = true;}
    }
    const context = {OriginalDetector, Worker: FakeWorker, setTimeout, clearTimeout};
    vm.runInNewContext(read('parking_detector_proxy.js').replace(/^import .*;\r?\n/, '')
        .replace('export class YOLODetector', 'globalThis.YOLODetector = class YOLODetector'), context);
    return {detector: new context.YOLODetector('/models/parking_cars.onnx'), workers, jobs,
        failConstruction: () => {constructionFails = true;}, manual: () => {auto = false;}};
}

test('proxy initializes one worker and filters display detections at exactly 70 percent', async () => {
    const f = proxyFixture();
    const initialized = await Promise.all([f.detector.init(), f.detector.init()]);
    assert.ok(initialized.every(result => result.success));
    assert.equal(f.workers.length, 1);
    const imageData = {data: new Uint8ClampedArray(16), width: 2, height: 2};
    const result = await f.detector.detect(imageData, 2, 2);
    assert.deepEqual(Array.from(result, d => d.confidence), [0.70, 0.84]);
    assert.equal(result.inference_valid, true);
    assert.equal(f.jobs[1].transfer[0], imageData.data.buffer);
});

test('proxy rejects overlapping inference rather than queuing old frames', async () => {
    const f = proxyFixture(); await f.detector.init(); f.manual();
    const first = f.detector.detect({data: new Uint8ClampedArray(16)}, 2, 2);
    await assert.rejects(f.detector.detect({}, 2, 2), /이전 차량 추론/);
    assert.equal(f.jobs.filter(job => job.message.type === 'detect').length, 1);
    f.workers[0].onmessage({data: {id: f.jobs[1].message.id, detections: [], inference_valid: true}});
    await first;
});

test('synchronous worker construction failure allows retry and never starts main thread inference', async () => {
    const f = proxyFixture(); f.failConstruction();
    assert.equal((await f.detector.init()).success, false);
    assert.equal((await f.detector.init()).success, true);
    assert.equal(f.workers.length, 1);
});

test('worker failure rejects pending result, terminates and allows a fresh worker', async () => {
    const f = proxyFixture(); await f.detector.init(); f.manual();
    const result = f.detector.detect({data: new Uint8ClampedArray(16)}, 2, 2);
    f.workers[0].onerror({message: 'WASM failure'});
    await assert.rejects(result, /WASM failure/);
    assert.equal(f.workers[0].terminated, true);
    assert.equal(f.detector.isInitialized, false);
    assert.equal((await f.detector.init()).success, true);
});

function workerFixture() {
    const messages = [], disposed = [];
    let fail = false, outputMissing = false, gate = null;
    class Original {
        async init() {this.isInitialized = true; this.outputNames = ['out']; this.session = {run: async () => {
            if (gate) await gate;
            if (fail) throw new Error('inference failed');
            return outputMissing ? {} : {out: {dispose: () => disposed.push('output')}};
        }};}
        preprocess() {return {tensor: {dispose: () => disposed.push('input')}, scale: 1, offsetX: 0, offsetY: 0};}
        postprocess() {return [0.35, 0.48, 0.70, 0.88].map(confidence => ({confidence}));}
    }
    const context = {importScripts() {}, ort: {env: {wasm: {}}},
        getOriginal: async () => ({YOLODetector: Original}), self: {postMessage: data => messages.push(data)}};
    vm.runInNewContext(read('parking_detection_worker.js').replace("await import('/rover-original-yolo.js')", 'await getOriginal()'), context);
    const send = (id, type = 'detect') => context.self.onmessage({data: {id, type, modelPath: 'model', width: 2, height: 2, imageData: {}}});
    return {messages, disposed, send, fail: () => {fail = true;}, missing: () => {outputMissing = true;},
        hold: promise => {gate = promise;}};
}

test('worker returns filtered detections and releases input/output tensors', async () => {
    const f = workerFixture(); await f.send(1);
    assert.deepEqual(Array.from(f.messages[0].detections, d => d.confidence), [0.70, 0.88]);
    assert.equal(f.messages[0].inference_valid, true);
    assert.deepEqual(f.disposed, ['input', 'output']);
});

test('worker inference errors and missing output are errors rather than confirmed empty frames', async () => {
    for (const option of ['fail', 'missing']) {
        const f = workerFixture(); f[option](); await f.send(1);
        assert.ok(f.messages[0].error);
        assert.equal(f.messages[0].inference_valid, undefined);
        assert.deepEqual(f.disposed, ['input']);
    }
});

test('worker refuses a second job while the first is running', async () => {
    const f = workerFixture(); let release;
    f.hold(new Promise(resolve => {release = resolve;}));
    const first = f.send(1); await f.send(2);
    assert.equal(f.messages[0].id, 2);
    assert.match(f.messages[0].error, /진행 중/);
    release(); await first;
    assert.equal(f.messages[1].id, 1);
});
