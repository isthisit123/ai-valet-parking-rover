const {test} = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

function fixture({encoding = 8, request = 25, status = 'detected', switchCamera = false, timeout = false} = {}) {
    let clock = 0;
    const requests = [], events = [], timers = [], encodingTypes = [];
    const video = {readyState: 2, videoWidth: 1280, videoHeight: 960, paused: false,
        currentTime: 1, srcObject: {getVideoTracks: () => [{id: 'camera'}]}};
    const context = {performance: {now: () => clock}, AbortSignal: {timeout: value => value},
        window: {dispatchEvent: event => events.push(event.detail)},
        CustomEvent: class {constructor(type, options) {this.type = type; this.detail = options.detail;}},
        setTimeout: (callback, delay) => timers.push({callback, delay}),
        fetch: async (url, options) => {requests.push({url, options}); clock += request;
            if (timeout) throw Object.assign(new Error('signal timed out'), {name: 'TimeoutError'});
            if (switchCamera) video.srcObject = {getVideoTracks: () => [{id: 'other'}]};
            return {ok: true, json: async () => ({tracking_status: status, rover_marker_id: 7, width: 1280, height: 960,
                processing_ms: 18.9, rover: status === 'detected' ? {center_px: [400, 300], heading_deg: 0} : null})};}};
    const source = fs.readFileSync(path.join(__dirname, '../scripts/rover_aruco_bridge.js'), 'utf8');
    vm.runInNewContext(source.replace('export class ArUcoBridge', 'globalThis.ArUcoBridge = class ArUcoBridge'), context);
    const bridge = Object.create(context.ArUcoBridge.prototype);
    Object.assign(bridge, {media: {currentSourceType: 'webcam', videoElement: video}, ready: Promise.resolve(),
        periodMs: 100, frameId: 0, lastVideoTime: null, lastReceivedAt: null, lastResult: null,
        ctx: {drawImage() {}}, panel: {}, canvas: {toBlob(callback, mime, quality) {
            encodingTypes.push({mime, quality}); clock += encoding; callback({type: mime});}}});
    return {bridge, requests, events, timers, encodingTypes, video};
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
