const {test} = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

function fixture({encoding = 8, request = 25, status = 'detected', switchCamera = false, timeout = false,
                  worker = false, workerHang = false, workerError = false, switchDuringEncoding = false,
                  responseFields = {}} = {}) {
    let clock = 0;
    const requests = [], events = [], timers = [], encodingTypes = [], workers = [], bitmaps = [];
    const video = {readyState: 2, videoWidth: 1280, videoHeight: 960, paused: false,
        currentTime: 1, srcObject: {getVideoTracks: () => [{id: 'camera'}]}};
    const context = {performance: {now: () => clock}, AbortSignal: {timeout: value => value},
        window: {dispatchEvent: event => events.push(event.detail)},
        CustomEvent: class {constructor(type, options) {this.type = type; this.detail = options.detail;}},
        setTimeout: (callback, delay) => timers.push({callback, delay}),
        clearTimeout: id => {timers[id - 1].cancelled = true;},
        fetch: async (url, options) => {requests.push({url, options}); clock += request;
            if (timeout) throw Object.assign(new Error('signal timed out'), {name: 'TimeoutError'});
            if (switchCamera) video.srcObject = {getVideoTracks: () => [{id: 'other'}]};
            return {ok: true, json: async () => ({tracking_status: status, rover_marker_id: 7, width: 1280, height: 960,
                processing_ms: 18.9, rover: status === 'detected' ? {center_px: [400, 300], heading_deg: 0} : null,
                ...responseFields})};}};
    if (worker) {
        context.OffscreenCanvas = class {};
        context.createImageBitmap = async () => {
            clock += 2;
            const bitmap = {width: video.videoWidth, height: video.videoHeight, close() {this.closed = true;}};
            bitmaps.push(bitmap); return bitmap;
        };
        context.Worker = class {
            constructor(url) {assert.equal(url, '/aruco-frame-worker.js'); workers.push(this);}
            terminate() {this.terminated = true;}
            postMessage(message, transfer) {
                assert.equal(transfer[0], message.bitmap);
                if (workerHang) {this.message = message; return;}
                clock += encoding;
                if (switchDuringEncoding) video.srcObject = {getVideoTracks: () => [{id: 'other'}]};
                this.onmessage({data: {id: message.id, blob: {type: 'image/jpeg'},
                    timing: {worker_encode_ms: encoding, worker_queue_ms: 0, worker_draw_ms: 0},
                    ...(workerError ? {error: 'encoder failed'} : {})}});
            }
        };
    }
    const source = fs.readFileSync(path.join(__dirname, '../scripts/rover_aruco_bridge.js'), 'utf8');
    vm.runInNewContext(source.replace('export class ArUcoBridge', 'globalThis.ArUcoBridge = class ArUcoBridge'), context);
    const bridge = Object.create(context.ArUcoBridge.prototype);
    Object.assign(bridge, {media: {currentSourceType: 'webcam', videoElement: video}, ready: Promise.resolve(),
        periodMs: 100, frameId: 0, encoderId: 0, lastVideoTime: null, lastReceivedAt: null, lastResult: null,
        ctx: {drawImage() {}}, panel: {}, canvas: {toBlob(callback, mime, quality) {
            encodingTypes.push({mime, quality}); clock += encoding; callback({type: mime});}}});
    return {bridge, requests, events, timers, encodingTypes, video, workers, bitmaps,
        advance: ms => {clock += ms;}};
}

test('native size JPEG tracking reports separate timings and preserves a fresh ID 7 pose', async () => {
    const f = fixture(); await f.bridge.loop();
    assert.equal(f.encodingTypes[0].mime, 'image/jpeg');
    assert.equal(f.encodingTypes[0].quality, 0.92);
    assert.equal(f.bridge.canvas.width, 1280);
    assert.equal(f.bridge.canvas.height, 960);
    assert.equal(f.requests[0].options.headers['Content-Type'], 'image/jpeg');
    assert.equal(f.requests[0].options.headers['X-Source-Width'], '1280');
    assert.equal(f.events[0].tracking_status, 'detected');
    assert.equal(f.events[0].encoding_ms, 8);
    assert.equal(f.events[0].request_ms, 25);
    assert.match(f.bridge.panel.textContent, /인코딩 8.0ms/);
});

test('three second response is stale and never exposes a rover pose', async () => {
    const f = fixture({request: 3000}); await f.bridge.loop();
    assert.equal(f.events[0].tracking_status, 'stale');
    assert.equal(f.events[0].server_tracking_status, 'detected');
    assert.equal(f.events[0].rover, null);
});

test('slow encoder discards old image before sending HTTP', async () => {
    const f = fixture({encoding: 201}); await f.bridge.loop();
    assert.equal(f.requests.length, 0);
    assert.match(f.bridge.panel.textContent, /인코딩 지연/);
    assert.equal(f.events[0], null);
});

test('missing marker is distinct from slow transport', async () => {
    const f = fixture({status: 'lost'}); await f.bridge.loop();
    assert.equal(f.events[0].tracking_status, 'lost');
    assert.match(f.bridge.panel.textContent, /상단 ID 7/);
});

test('repeated video frame is not retransmitted and clears current pose', async () => {
    const f = fixture(); await f.bridge.loop(); await f.bridge.loop();
    assert.equal(f.requests.length, 1);
    assert.equal(f.bridge.lastResult, null);
    assert.equal(f.events.at(-1), null);
});

test('camera switch while HTTP is pending discards the old source result', async () => {
    const f = fixture({switchCamera: true}); await f.bridge.loop();
    assert.equal(f.events[0], null);
    assert.match(f.bridge.panel.textContent, /카메라가 변경/);
});

test('transport timeout clears current pose and reports timeout separately from missing marker', async () => {
    const f = fixture({timeout: true}); await f.bridge.loop();
    assert.equal(f.events[0], null);
    assert.equal(f.bridge.lastResult, null);
    assert.match(f.bridge.panel.textContent, /전송·응답 시간 초과/);
    assert.equal(f.timers.length, 1);
});

test('worker path transfers a native size frame and includes snapshot time in freshness', async () => {
    const f = fixture({worker: true}); await f.bridge.loop();
    assert.equal(f.bitmaps[0].width, 1280);
    assert.equal(f.bitmaps[0].height, 960);
    assert.equal(f.encodingTypes.length, 0);
    assert.equal(f.events[0].encoding_mode, 'worker');
    assert.equal(f.events[0].encoding_ms, 10);
    assert.equal(f.events[0].encoding_timing_ms.worker_encode_ms, 8);
    assert.equal(f.events[0].tracking_status, 'detected');
    assert.equal(f.requests[0].options.headers['X-Source-Width'], '1280');
});

test('slow worker cannot send an expired frame or reuse a previous pose', async () => {
    const f = fixture({worker: true, encoding: 199}); await f.bridge.loop();
    assert.equal(f.requests.length, 0);
    assert.equal(f.bridge.lastResult, null);
    assert.equal(f.events[0], null);
    assert.match(f.bridge.panel.textContent, /인코딩 만료 폐기 1/);
});

test('camera change during worker encoding discards the captured frame', async () => {
    const f = fixture({worker: true, switchDuringEncoding: true}); await f.bridge.loop();
    assert.equal(f.requests.length, 0);
    assert.equal(f.events[0], null);
});

test('worker failure clears pose and a later new frame can use the explicit main thread fallback', async () => {
    const f = fixture({worker: true, workerError: true}); await f.bridge.loop();
    assert.equal(f.workers[0].terminated, true);
    assert.equal(f.requests.length, 0);
    assert.equal(f.events[0], null);
    f.video.currentTime++;
    await f.bridge.loop();
    assert.equal(f.events[1].encoding_mode, 'main');
    assert.match(f.bridge.panel.textContent, /Worker 미지원·실패/);
});

test('worker timeout ignores a late reply and never starts HTTP for it', async () => {
    const f = fixture({worker: true, workerHang: true});
    const result = f.bridge.loop();
    await new Promise(resolve => setImmediate(resolve));
    f.timers.find(timer => timer.delay === 1000).callback();
    await result;
    f.workers[0].onmessage({data: {id: f.workers[0].message.id, blob: {type: 'image/jpeg'}}});
    assert.equal(f.requests.length, 0);
    assert.equal(f.events.at(-1), null);
    assert.equal(f.bridge.lastResult, null);
});

test('diagnostic summary updates once per second and expires old samples after ten seconds', () => {
    const f = fixture();
    assert.match(f.bridge.summarize({encoding_ms: 150, encode_expired: true}), /150.0\/150.0ms/);
    f.advance(100);
    assert.match(f.bridge.summarize({encoding_ms: 10}), /150.0\/150.0ms/);
    f.advance(1000);
    assert.match(f.bridge.summarize({encoding_ms: 20}), /60.0\/150.0ms/);
    f.advance(10000);
    const summary = f.bridge.summarize({encoding_ms: 5});
    assert.match(summary, /12.5\/20.0ms/);
    assert.match(summary, /인코딩 만료 폐기 0/);
    assert.match(summary, /cm 수학 변환 미측정/);
});

test('background file time is separate from response registration and storage failure remains visible', async () => {
    const f = fixture({responseFields: {
        server_timing_ms: {total_ms: 12, tracker_save_ms: .2, arrival_ms: 5, decode_ms: 3},
        persistence: {mode: 'async_latest', writing: true, pending: true, last_write_ms: 358},
        persistence_warning: '진단 JSON 저장 실패: file locked',
        arrival_test: {timing_ms: {save_ms: .1, cm_math_ms: .6, map_validation_ms: 2},
            persistence: {mode: 'async_latest', writing: false, pending: false, last_write_ms: 376}},
    }});
    await f.bridge.loop();
    assert.match(f.bridge.panel.textContent, /별도 스레드 · 대기 1개/);
    assert.match(f.bridge.panel.textContent, /응답 경로\) 0.3\/0.3ms/);
    assert.match(f.bridge.panel.textContent, /별도 기록 최근 소요: 추적 358.0ms \/ 도착 376.0ms/);
    assert.match(f.bridge.panel.textContent, /file locked/);
    assert.equal(f.events[0].tracking_status, 'detected');
});
