const {test} = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');

function fixture() {
    const free = {textContent: ''};
    const context = {window: {}, document: {getElementById: () => free}};
    vm.runInNewContext(fs.readFileSync(path.join(__dirname, '../scripts/parking_regions_bridge.js'), 'utf8'), context);
    const api = context.window.RoverParkingRegions;
    let regions = [{id: 'old'}];
    const renderer = {getRegions: () => regions, setRegions: value => {regions = value;}, syncCanvas() {},
        currentRegion: {}, selectedRegion: {}, selectedPointIndex: 1};
    const video = {srcObject: {getVideoTracks: () => [{id: 'track'}]}, videoWidth: 1000, videoHeight: 800, src: ''};
    const app = {currentSourceId: 'LIVE_WEBCAM', mainCanvas: {width: 250, height: 200},
        mediaHandler: {currentSourceType: 'webcam', videoElement: video, currentFile: null},
        overlayRenderer: renderer, setMode(mode) {this.mode = mode;}, updateStats() {}};
    const captured = api.snapshot(app);
    let saved = JSON.stringify({LIVE_WEBCAM: [{id: 'old'}], other_image: [{id: 'keep'}]});
    const storage = {getItem: () => saved, setItem: (_key, value) => {saved = value;}, removeItem: () => {saved = null;}};
    const report = {status: 'map_generated', published_as_latest: true, capture_id: 'capture', map_id: 'map',
        id_assignment_revision: 'revision', map_image_sha256: 'a'.repeat(64), source_sha256: 'b'.repeat(64),
        image_size_px: [500, 400]};
    report.parking_regions = {...report, schema_version: '1.0', status: 'generated', coordinate_frame: 'capture_image',
        unit: 'pixel', origin: 'top_left', axes: {x: 'right', y: 'down'},
        camera_source: {session_id: 'track', kind: 'webcam', width: 1000, height: 800},
        regions: Array.from({length: 6}, (_, i) => ({id: `P${i + 1}`, parking_space_id: `space_${i + 1}`,
            type: [2, 5].includes(i) ? 'accessible' : 'normal',
            points: [{x: 30, y: 30}, {x: 100, y: 40}, {x: 120, y: 100}, {x: 20, y: 100}]}))};
    return {api, app, captured, report, storage, free, context, stored: () => JSON.parse(saved)};
}

test('six polygons scale to current canvas and save only current source with unknown occupancy', () => {
    const f = fixture();
    assert.equal(f.api.apply(f.report, f.captured, f.app, f.storage), 6);
    const saved = f.stored();
    assert.deepEqual(saved.other_image, [{id: 'keep'}]);
    assert.deepEqual(saved.LIVE_WEBCAM[0].points[1], {x: 50, y: 20});
    assert.equal(saved.LIVE_WEBCAM[2].type, 'accessible');
    assert.equal(saved.LIVE_WEBCAM[0].map_image_sha256, f.report.map_image_sha256);
    assert.equal(saved.LIVE_WEBCAM[0].occupied, null);
    assert.equal(saved.LIVE_WEBCAM[0].occupancy_status, 'unknown');
    assert.equal(f.app.mode, 'setup');
    assert.equal(f.app.overlayRenderer.currentRegion, null);
    assert.equal(f.free.textContent, '미확인');
});

for (const [name, change] of Object.entries({
    'camera switched during OCR': f => {f.app.mediaHandler.videoElement.srcObject = {getVideoTracks: () => [{id: 'other'}]};},
    'file source switched': f => {f.app.currentSourceId = 'other_image';},
    'native resolution changed': f => {f.app.mediaHandler.videoElement.videoWidth = 1200;},
    'aspect changed': f => {f.app.mainCanvas.height = 100;},
    'stale map identity': f => {f.report.parking_regions.map_image_sha256 = 'old';},
    'wrong source hash': f => {f.report.parking_regions.source_sha256 = 'old';},
    'wrong coordinate frame': f => {f.report.parking_regions.unit = 'cm';},
    'partial map': f => {f.report.parking_regions.regions.pop();},
    'duplicate slot': f => {f.report.parking_regions.regions[1] = f.report.parking_regions.regions[0];},
    'wrong accessible slot': f => {f.report.parking_regions.regions[0].type = 'accessible';},
    'out of image': f => {f.report.parking_regions.regions[0].points[0].x = 500;},
    'self crossing polygon': f => {f.report.parking_regions.regions[0].points.reverse();
        const p = f.report.parking_regions.regions[0].points; [p[0], p[1]] = [p[1], p[0]];},
    'failed map': f => {f.report.status = 'failed';},
    'unpublished map': f => {f.report.published_as_latest = false;},
})) {
    test(`${name} preserves previous settings and visible regions`, () => {
        const f = fixture();
        const before = f.stored();
        const visible = f.app.overlayRenderer.getRegions();
        change(f);
        assert.throws(() => f.api.apply(f.report, f.captured, f.app, f.storage));
        assert.deepEqual(f.stored(), before);
        assert.equal(f.app.overlayRenderer.getRegions(), visible);
    });
}

test('localStorage quota failure preserves visible regions', () => {
    const f = fixture();
    const visible = f.app.overlayRenderer.getRegions();
    f.storage.setItem = () => {throw new Error('quota');};
    assert.throws(() => f.api.apply(f.report, f.captured, f.app, f.storage), /quota/);
    assert.equal(f.app.overlayRenderer.getRegions(), visible);
});

test('render failure rolls back localStorage and regions', () => {
    const f = fixture();
    const before = f.stored();
    const visible = f.app.overlayRenderer.getRegions();
    const original = f.app.overlayRenderer.setRegions;
    let once = true;
    f.app.overlayRenderer.setRegions = value => {if (once) {once = false; throw new Error('render');} original(value);};
    assert.throws(() => f.api.apply(f.report, f.captured, f.app, f.storage), /render/);
    assert.deepEqual(f.stored(), before);
    assert.equal(f.app.overlayRenderer.getRegions(), visible);
});

test('loaded image can apply without a webcam session', () => {
    const f = fixture();
    f.app.currentSourceId = 'map.png_123';
    f.app.mediaHandler.currentSourceType = 'image';
    f.app.mediaHandler.currentFile = {name: 'map.png'};
    f.report.parking_regions.camera_source = {kind: 'image', session_id: '', width: 0, height: 0};
    const captured = f.api.snapshot(f.app);
    f.api.apply(f.report, captured, f.app, f.storage);
    assert.equal(f.stored()['map.png_123'].length, 6);
});

function mapUploadFixture() {
    const f = fixture();
    const elements = [];
    const events = [];
    const requests = [];
    const element = tag => {
        const value = {tag, children: [], listeners: {}, style: {},
            append(...items) {this.children.push(...items);},
            replaceChildren() {this.children = [];},
            addEventListener(name, fn) {this.listeners[name] = fn;},
            getContext: () => ({drawImage() {}}), toBlob(callback) {callback({});}};
        elements.push(value);
        return value;
    };
    f.context.window.parkingApp = f.app;
    f.context.window.dispatchEvent = event => {events.push(event.type);};
    f.context.CustomEvent = class {constructor(type, options) {this.type = type; this.detail = options?.detail;}};
    f.context.localStorage = f.storage;
    f.context.document.createElement = element;
    f.context.document.body = element('body');
    f.context.document.getElementById = id => id === 'main-canvas' ? f.app.mainCanvas : f.free;
    Object.assign(f.report, {generation_method: 'ocr', output_directory: 'latest',
        graph: {nodes: 14, edges: 13, connected: true}, detected: {normal: 4, accessible: 2},
        files: {parking_regions: '/captures/latest/parking_regions.json'}});
    f.report.image_size_px = f.report.parking_regions.image_size_px = [250, 200];
    f.context.fetch = async (url, options) => {
        requests.push(options);
        assert.equal(url, '/api/map-capture');
        assert.equal(options.headers['X-Camera-Session'], 'track');
        assert.equal(options.headers['X-Source-Width'], '1000');
        return {ok: true, json: async () => f.report};
    };
    vm.runInNewContext(fs.readFileSync(path.join(__dirname, '../scripts/http_map_bridge.js'), 'utf8'), f.context);
    const button = elements.find(e => e.tag === 'button');
    return {...f, elements, events, requests, button,
        input: key => elements.find(e => e.id === 'rover-camera-' + key),
        status: elements.find(e => e.tag === 'pre')};
}

test('PNG upload button applies the response polygons and exposes JSON link', async () => {
    const f = mapUploadFixture();
    const {button, elements, events} = f;
    await button.listeners.click();
    assert.equal(button.disabled, false);
    assert.equal(f.stored().LIVE_WEBCAM.length, 6);
    assert.match(elements.find(e => e.tag === 'pre').textContent, /6개 구역 자동 적용/);
    assert.ok(elements.some(e => e.tag === 'a' && e.href === '/captures/latest/parking_regions.json'));
    assert.deepEqual(events, ['rover-map-start', 'rover-map-result', 'rover-map-end']);
});

test('web camera fields are captured with PNG and successful registration is shown', async () => {
    const f = mapUploadFixture();
    for (const [key, value] of Object.entries({x: '53', y: '151.7', z: '111.5'})) f.input(key).value = value;
    f.report.camera_position = {status: 'saved', camera_position_cm: [53, 151.7, 111.5], marker_height_cm: 10.5};
    f.report.files.camera_position = '/captures/latest/world/camera_position.json';
    await f.button.listeners.click();
    assert.deepEqual(JSON.parse(f.requests[0].headers['X-Rover-Camera-Position']),
        {camera_position_cm: [53, 151.7, 111.5], marker_height_cm: 10.5});
    assert.match(f.status.textContent, /카메라 좌표 등록 완료/);
    assert.ok(f.elements.some(e => e.tag === 'a' && e.href === f.report.files.camera_position));
    assert.equal(f.input('x').disabled, false);
});

for (const [name, values] of Object.entries({
    partial: {x: '53', y: '', z: '111.5'}, nonfinite: {x: 'NaN', y: '151.7', z: '111.5'},
    lowCamera: {x: '53', y: '151.7', z: '10.5'}, negativeHeight: {x: '53', y: '151.7', z: '111.5', height: '-1'},
})) {
    test(`invalid camera fields ${name} stop before PNG upload`, async () => {
        const f = mapUploadFixture();
        for (const [key, value] of Object.entries(values)) f.input(key).value = value;
        await f.button.listeners.click();
        assert.equal(f.requests.length, 0);
        assert.match(f.status.textContent, /Z는 마커 높이보다 커야/);
        assert.equal(f.button.disabled, false);
    });
}

test('failed cm validation reports camera registration as not saved', async () => {
    const f = mapUploadFixture();
    for (const [key, value] of Object.entries({x: '-10', y: '151.7', z: '111.5'})) f.input(key).value = value;
    f.report.camera_position = {status: 'not_saved', error: 'cm 보정 검증 필요'};
    await f.button.listeners.click();
    assert.match(f.status.textContent, /카메라 좌표 미등록/);
    assert.doesNotMatch(f.status.textContent, /카메라 좌표 등록 완료/);
    assert.equal(JSON.parse(f.requests[0].headers['X-Rover-Camera-Position']).camera_position_cm[0], -10);
});
