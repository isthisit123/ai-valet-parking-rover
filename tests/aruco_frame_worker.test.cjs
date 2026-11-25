const {test} = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

function fixture({failure = false} = {}) {
    const messages = [], canvases = [], options = [];
    let gate, clock = 0;
    class Canvas {
        constructor(width, height) {this.width = width; this.height = height; canvases.push(this);}
        getContext() {return {drawImage() {clock += 2;}};}
        async convertToBlob(config) {
            options.push(config);
            if (gate) await gate;
            clock += 5;
            if (failure) throw new Error('cannot encode');
            return {type: 'image/jpeg'};
        }
    }
    const context = {OffscreenCanvas: Canvas, performance: {timeOrigin: 1000, now: () => clock},
        self: {postMessage: data => messages.push(data)}};
    vm.runInNewContext(fs.readFileSync(path.join(__dirname, '../scripts/aruco_frame_worker.js'), 'utf8'), context);
    const send = (id, width = 1280, height = 960) => {
        const bitmap = {width, height, closes: 0, close() {this.closes++;}};
        return {bitmap, result: context.self.onmessage({data: {id, bitmap, sentAt: 995 + clock}})};
    };
    return {messages, canvases, options, send, hold: promise => {gate = promise;}};
}

test('worker preserves full frame size and JPEG quality and reports its own timings', async () => {
    const f = fixture(), first = f.send(1); await first.result;
    assert.equal(f.canvases[0].width, 1280);
    assert.equal(f.canvases[0].height, 960);
    assert.equal(f.options[0].type, 'image/jpeg');
    assert.equal(f.options[0].quality, 0.92);
    assert.equal(f.messages[0].timing.worker_queue_ms, 5);
    assert.equal(f.messages[0].timing.worker_encode_ms, 5);
    assert.equal(first.bitmap.closes, 1);
    await f.send(2).result;
    assert.equal(f.canvases.length, 1);
    await f.send(3, 800, 600).result;
    assert.equal(f.canvases[0].width, 800);
    assert.equal(f.canvases[0].height, 600);
});

test('worker rejects overlapping frames without a queue and releases every bitmap', async () => {
    const f = fixture(); let release;
    f.hold(new Promise(resolve => {release = resolve;}));
    const first = f.send(1), second = f.send(2);
    await second.result;
    assert.equal(f.messages[0].id, 2);
    assert.match(f.messages[0].error, /진행 중/);
    assert.equal(second.bitmap.closes, 1);
    release(); await first.result;
    assert.equal(f.messages[1].id, 1);
    assert.equal(first.bitmap.closes, 1);
});

test('encoding error produces no blob and releases the frame', async () => {
    const f = fixture({failure: true}), frame = f.send(1); await frame.result;
    assert.match(f.messages[0].error, /cannot encode/);
    assert.equal(f.messages[0].blob, undefined);
    assert.equal(frame.bitmap.closes, 1);
});
