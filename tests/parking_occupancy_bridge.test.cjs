const {test} = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');

function fixture() {
    const listeners = {}, sent = [], messages = [];
    let clock = 0;
    const epoch = Date.now();
    class ClockDate extends Date {
        constructor(...args) {super(...(args.length ? args : [epoch + clock]));}
        static now() {return epoch + clock;}
    }
    const map = {capture_id: 'capture', map_id: 'map', map_image_sha256: 'a'.repeat(64),
        id_assignment_revision: 'v1', published_as_latest: true,
        camera_source: {kind: 'webcam', session_id: 'track', width: 800, height: 600}};
    let regions = Array.from({length: 6}, (_, i) => ({...map, parking_space_id: `space_${i + 1}`,
        image_size_px: [400, 300], occupied: null, points: [{x: i * 10, y: 30}]}));
    const source = {sourceId: 'LIVE_WEBCAM', kind: 'webcam', session_id: 'track', width: 800, height: 600};
    const detector = {fail: false, empty: false, car: false, confidence: 0.84,
        postprocess() {return this.vehicles || (this.car ? [{car: true, confidence: this.confidence}] : []);},
        async detect() {if (this.fail) return []; return this.postprocess();},
        checkParkingOccupancy(values, detections) {return values.map((r, i) => ({...r, occupied: i === 3 && detections.length > 0}));}};
    const app = {detector, currentMode: 'monitor', isDetecting: true, mainCanvas: {width: 400, height: 300},
        async detectAndRender() {return this.detector.detect();},
        overlayRenderer: {getRegions: () => regions}};
    const context = {performance: {now: () => clock},
        Date: ClockDate,
        AbortSignal: {timeout: value => value},
        window: {parkingApp: app, RoverParkingRegions: {snapshot: () => ({...source})},
            addEventListener: (type, callback) => {listeners[type] = callback;},
            dispatchEvent: event => {messages.push(event.detail);}},
        CustomEvent: class {constructor(type, options) {this.type = type; this.detail = options.detail;}},
        fetch: async (url, options) => {assert.equal(url, '/api/parking-occupancy');
            sent.push(JSON.parse(options.body)); return {ok: true, json: async () => ({occupancy: {states: {space_1: 'free'}}})};}};
    vm.runInNewContext(fs.readFileSync(path.join(__dirname, '../scripts/parking_occupancy_bridge.js'), 'utf8'), context);
    listeners['rover-map-result']({detail: map});
    async function detect() {await detector.detect(); await new Promise(resolve => setImmediate(resolve));}
    function aruco(overrides = {}) {
        const data = {tracking_status: 'detected', rover_marker_id: 7, width: 800, height: 600,
            camera_source: {...map.camera_source}, client_captured_at: new ClockDate().toISOString(), frame_id: String(clock),
            rover: {id: 7, center_px: [160, 160], corners_px: [[140, 140], [180, 140], [180, 180], [140, 180]]},
            ...overrides};
        listeners['aruco-result']({detail: data});
    }
    return {detector, app, source, map, sent, messages, listeners, detect,
        aruco, epoch, tick: value => {clock = value;}, regions: () => regions, setRegions: value => {regions = value;}};
}

test('publishes every newly inferred frame even when occupancy is unchanged', async () => {
    const f = fixture();
    await f.detect();
    assert.equal(f.sent.length, 1);
    assert.equal(f.sent[0].spaces[3].status, 'free');
    f.tick(500); await f.detect();
    assert.equal(f.sent.length, 2);
    f.detector.car = true; f.tick(600); await f.detect();
    assert.equal(f.sent.length, 3);
    assert.equal(f.sent[2].spaces[3].status, 'occupied');
    f.tick(1600); await f.detect();
    assert.equal(f.sent.length, 4);
    assert.equal(f.sent[3].capture_id, 'capture');
    assert.equal(f.sent[3].schema_version, '0.1-draft');
});

test('inference error returning [] produces unknown rather than free', async () => {
    const f = fixture();
    f.detector.fail = true;
    await f.detect();
    assert.ok(f.sent[0].spaces.every(space => space.status === 'unknown'));
});

for (const confidence of [0.35, 0.48, 0.6999, 0.70, 0.84]) {
    test(`confidence ${confidence} uses the same 70 percent threshold for display and occupancy`, async () => {
        const f = fixture(); f.detector.car = true; f.detector.confidence = confidence;
        const detections = await f.detector.detect();
        await new Promise(resolve => setImmediate(resolve));
        assert.equal(detections.length, confidence >= 0.70 ? 1 : 0);
        assert.equal(f.sent[0].spaces[3].status, confidence >= 0.70 ? 'occupied' : 'free');
    });
}

test('thrown inference error produces unknown', async () => {
    const f = fixture();
    // 원래 detect가 호출하는 postprocess의 예외를 모의한다.
    f.detector.postprocess = () => {throw new Error('model failed');};
    await f.detect();
    assert.ok(f.sent[0].spaces.every(space => space.status === 'unknown'));
});

for (const [name, change] of Object.entries({
    'different map': f => {f.regions()[0].capture_id = 'old';},
    'canvas resized': f => {f.app.mainCanvas.width = 500;},
    'setup mode': f => {f.app.currentMode = 'setup';},
    'detector stopped': f => {f.app.isDetecting = false;},
})) {
    test(`${name} does not publish unchecked free states`, async () => {
        const f = fixture(); change(f); await f.detect();
        assert.ok(f.sent[0].spaces.some(space => space.status === 'unknown'));
    });
}

test('camera changed or map generation started drops the old observation', async () => {
    const f = fixture();
    f.source.session_id = 'other'; await f.detect();
    assert.equal(f.sent.length, 0);
    f.source.session_id = 'track';
    f.listeners['rover-map-start'](); await f.detect();
    assert.equal(f.sent.length, 0);
});

test('partial ROI is shown as an error and not published', async () => {
    const f = fixture();
    f.setRegions(f.regions().slice(1));
    await f.detect();
    assert.equal(f.sent.length, 0);
    assert.ok(f.messages.some(item => /실패/.test(item.message)));
});

test('monitor interval skips overlap instead of queueing old frame inference', async () => {
    const f = fixture(); let release, calls = 0;
    f.detector.detect = () => {calls++; return new Promise(resolve => {release = resolve;});};
    const first = f.app.detectAndRender();
    await f.app.detectAndRender();
    assert.equal(calls, 1);
    release([]); await first;
    const next = f.app.detectAndRender();
    assert.equal(calls, 2);
    release([]); await next;
});

const roverBox = () => ({confidence: 0.90, bbox: {x: 50, y: 45, width: 60, height: 85}, center: {x: 80, y: 80}});
const otherCar = () => ({confidence: 0.86, bbox: {x: 180, y: 140, width: 60, height: 60}, center: {x: 210, y: 170}});

test('same-frame ArUco excludes rover body from display and parking occupancy', async () => {
    const f = fixture(); f.aruco(); f.detector.vehicles = [roverBox()];
    const result = await f.detector.detect(); await new Promise(resolve => setImmediate(resolve));
    assert.equal(result.length, 0);
    assert.equal(result.inference_valid, true);
    assert.equal(f.sent[0].spaces[3].status, 'free');
    assert.equal(f.sent[0].rover_exclusion.status, 'matched');
    assert.equal(f.sent[0].rover_exclusion.excluded_count, 1);
    assert.deepEqual(f.sent[0].rover_exclusion.marker_center_px, [80, 80]);
    assert.equal(f.sent[0].occupancy_scope, 'other_vehicles_excluding_controlled_rover');
});

test('another car in the same parking region remains occupied when rover body is excluded', async () => {
    const f = fixture(); f.aruco(); f.detector.vehicles = [roverBox(), otherCar()];
    const result = await f.detector.detect(); await new Promise(resolve => setImmediate(resolve));
    assert.equal(result.length, 1);
    assert.equal(result[0].bbox.x, 180);
    assert.equal(f.sent[0].spaces[3].status, 'occupied');
    assert.equal(f.sent[0].rover_exclusion.excluded_count, 1);
});

test('matches marker history at inference capture time instead of the rover position after 700ms', async () => {
    const f = fixture(); f.aruco(); f.detector.vehicles = [roverBox(), otherCar()];
    const request = f.detector.detect();
    f.tick(700);
    f.aruco({rover: {id: 7, center_px: [420, 340], corners_px: [[400, 320], [440, 320], [440, 360], [400, 360]]}});
    const result = await request; await new Promise(resolve => setImmediate(resolve));
    assert.equal(result.length, 1);
    assert.equal(result[0].bbox.x, 180);
    assert.equal(f.sent[0].rover_exclusion.marker_frame_id, '0');
    assert.equal(f.sent[0].rover_exclusion.frame_delta_ms, 0);
});

test('one missed marker frame can retain body identity only with decoded support on both sides', async () => {
    const f = fixture(); f.aruco(); f.tick(100); f.aruco({tracking_status: 'lost', rover: null});
    f.detector.vehicles = [roverBox(), otherCar()];
    const request = f.detector.detect(); f.tick(200); f.aruco();
    const result = await request; await new Promise(resolve => setImmediate(resolve));
    assert.equal(result.length, 1);
    assert.equal(result[0].bbox.x, 180);
    assert.equal(f.sent[0].rover_exclusion.association_method, 'bracketed_marker_observations');
    assert.equal(f.sent[0].spaces[3].status, 'occupied');
});

test('ambiguous ID never uses bracketing and rapid motion is not extrapolated', async () => {
    for (const ambiguous of [true, false]) {
        const f = fixture(); f.aruco(); f.tick(100);
        f.aruco({tracking_status: ambiguous ? 'ambiguous' : 'lost', rover: null});
        f.detector.vehicles = [roverBox()]; const request = f.detector.detect(); f.tick(200);
        f.aruco(ambiguous ? {} : {rover: {id: 7, center_px: [300, 160], corners_px: [[280, 140], [320, 140], [320, 180], [280, 180]]}});
        const result = await request; await new Promise(resolve => setImmediate(resolve));
        assert.equal(result.length, 1);
        assert.equal(f.sent[0].rover_exclusion.excluded_count, 0);
    }
});

for (const offset of [150, 151]) {
    test(`marker frame offset ${offset}ms respects the 150ms matching limit`, async () => {
        const f = fixture(); f.aruco(); f.tick(offset); f.detector.vehicles = [roverBox()];
        await f.detect();
        assert.equal(f.sent[0].rover_exclusion.excluded_count, offset === 150 ? 1 : 0);
    });
}

for (const [name, change] of Object.entries({
    'missing marker': f => {},
    'wrong marker ID': f => f.aruco({rover: {id: 8, center_px: [160, 160], corners_px: [[140, 140], [180, 140], [180, 180], [140, 180]]}}),
    'old marker': f => {f.aruco(); f.tick(201);},
    'stale tracking': f => f.aruco({tracking_status: 'stale'}),
    'wrong camera': f => f.aruco({camera_source: {...f.map.camera_source, session_id: 'other'}}),
    'native size mismatch': f => f.aruco({width: 640}),
    'lost frame nearer than old detection': f => {f.aruco(); f.tick(20); f.aruco({tracking_status: 'lost', rover: null});},
    'partial marker overlap': f => {f.aruco(); f.detector.vehicles[0].bbox = {x: 70, y: 70, width: 10, height: 10};},
    'oversized vehicle box': f => {f.aruco(); f.detector.vehicles[0].bbox = {x: 0, y: 0, width: 400, height: 300};},
    'ambiguous vehicle boxes': f => {f.aruco(); f.detector.vehicles.push({confidence: 0.9, bbox: {x: 40, y: 40, width: 80, height: 100}});},
})) {
    test(`${name} never hides an unconfirmed car`, async () => {
        const f = fixture(); f.detector.vehicles = [roverBox()]; change(f);
        const result = await f.detector.detect(); await new Promise(resolve => setImmediate(resolve));
        assert.equal(result.length, f.detector.vehicles.length);
        assert.equal(f.sent[0].spaces[3].status, 'occupied');
        assert.equal(f.sent[0].rover_exclusion.excluded_count, 0);
    });
}
