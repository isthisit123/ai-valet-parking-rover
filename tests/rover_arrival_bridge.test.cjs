const {test} = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');

async function fixture() {
    const elements = [], timers = [], requests = [], drawing = [], contexts = [];
    let state = {status: 'idle'};
    let clock = 10;
    const catalog = {ready: true, capture_id: 'map-capture', map_image_sha256: 'hash', image_size_px: [400, 400],
        occupancy: {fresh: true}, cases: [{case_id: '03_exit', available: true, parking_space_id: 'space_5', goal_px: [260, 200]}],
        spaces: [{parking_space_id: 'space_4', status: 'occupied', type: 'normal'},
                 {parking_space_id: 'space_5', status: 'free', type: 'normal'}]};
    function createElement(tag) {
        const e = {tag, value: '', style: {}, children: [], listeners: {}, replaceCount: 0,
            append(...items) {this.children.push(...items);}, replaceChildren() {this.children = []; this.replaceCount++;},
            addEventListener(type, callback) {this.listeners[type] = callback;},
            getContext() {
                const ctx = {segments: 0, clearRect() {drawing.length = 0; this.segments = 0;}, drawImage() {},
                    beginPath() {}, stroke() {}, fill() {}, moveTo() {}, lineTo() {this.segments++;},
                    arc: (...args) => drawing.push(args)};
                contexts.push(ctx); return ctx;
            }};
        elements.push(e); return e;
    }
    const context = {document: {createElement, createTextNode: text => ({textContent: text}), body: createElement('body')},
        window: {addEventListener() {}}, performance: {now: () => clock},
        Image: class {set src(value) {queueMicrotask(() => this.onload());}},
        setTimeout: callback => timers.push(callback),
        fetch: async (url, options) => {
            const data = options?.body ? JSON.parse(options.body) : null;
            requests.push({url, data});
            if (url === '/api/arrival-tests') return {ok: true, json: async () => catalog};
            if (url.endsWith('/stop')) state = {status: 'stopped'};
            if (url.endsWith('/start')) state = {...state, status: 'waiting_for_tracking', case_id: '03_exit',
                parking_space_id: data.parking_space_id || 'space_5', goal_px: [260, 200], radius_px: 15,
                guidance_available: false};
            return {ok: true, json: async () => state};
        }};
    vm.runInNewContext(fs.readFileSync(path.join(__dirname, '../scripts/rover_arrival_bridge.js'), 'utf8'), context);
    await new Promise(resolve => setImmediate(resolve));
    const [cases, spaces] = elements.filter(e => e.tag === 'select');
    cases.value = '03_exit';
    await cases.listeners.change();
    await new Promise(resolve => setImmediate(resolve));
    return {catalog, requests, drawing, contexts, timers, cases, spaces, elements, context, tick: value => {clock = value;},
        status: elements.find(e => e.tag === 'pre'), start: elements.find(e => e.textContent === '테스트 시작'),
        setState: next => {state = next;}};
}

test('occupied manual destination disables start and explains why', async () => {
    const f = await fixture();
    f.spaces.value = 'space_4';
    await f.spaces.listeners.change();
    assert.equal(f.start.disabled, true);
    assert.match(f.status.textContent, /이미 점유된 주차칸/);
    assert.equal(f.drawing.length, 0);
});

test('automatic selection submits the condition and retarget flag', async () => {
    const f = await fixture();
    assert.equal(f.start.disabled, false);
    await f.start.listeners.click();
    const request = f.requests.find(item => item.url.endsWith('/start'));
    assert.equal(request.data.case_id, '03_exit');
    assert.equal(request.data.parking_space_id, null);
    assert.equal(request.data.auto_retarget, true);
    assert.equal(request.data.arrival_area_ratio, 0.5);
    assert.equal(request.data.hold_seconds, 2);
    assert.equal(request.data.radius_px, undefined);
});

test('poll renders active retargeted goal rather than cached catalog destination', async () => {
    const f = await fixture();
    f.setState({status: 'running', case_id: '03_exit', parking_space_id: 'space_4', goal_px: [320, 200],
        radius_px: 15, guidance_available: true, waypoints: [[180, 150], [320, 200]],
        target_changes: [{from: 'space_5', to: 'space_4'}]});
    await f.timers.shift()();
    assert.match(f.status.textContent, /목표 P4/);
    assert.match(f.status.textContent, /P5 → P4/);
    assert.equal(f.drawing[0][0], 320);
    assert.equal(f.drawing[0][1], 200);
});

test('blocked occupancy clears route and target overlays', async () => {
    const f = await fixture();
    f.setState({status: 'blocked', case_id: '03_exit', parking_space_id: 'space_5', goal_px: [260, 200],
        guidance_available: false, reason: '최신 점유 정보가 없습니다.'});
    await f.timers.shift()();
    assert.match(f.status.textContent, /경로 안내 중단/);
    assert.equal(f.drawing.length, 0);
});

test('changing occupancy never rebuilds menus or replaces the selected condition or slot', async () => {
    const f = await fixture();
    f.spaces.value = 'space_5'; await f.spaces.listeners.change();
    const caseNodes = [...f.cases.children], spaceNodes = [...f.spaces.children];
    f.catalog.occupancy = {fresh: false};
    f.catalog.cases[0].available = false; f.catalog.cases[0].parking_space_id = null;
    f.catalog.spaces[1].status = 'unknown';
    f.tick(1010); await f.timers.shift()(); await new Promise(resolve => setImmediate(resolve));
    assert.equal(f.cases.value, '03_exit');
    assert.equal(f.spaces.value, 'space_5');
    assert.equal(f.cases.replaceCount, 1);
    assert.equal(f.spaces.replaceCount, 1);
    assert.ok(f.cases.children.every((node, i) => node === caseNodes[i]));
    assert.ok(f.spaces.children.every((node, i) => node === spaceNodes[i]));
    assert.equal(f.cases.children[1].textContent, '출구 가까운 일반석');
    assert.equal(f.start.disabled, true);
});

test('focused slot menu defers label changes while latest server state still blocks occupied targets', async () => {
    const f = await fixture(); f.spaces.value = 'space_5';
    const node = f.spaces.children.find(child => child.value === 'space_5');
    const previousLabel = node.textContent;
    f.context.document.activeElement = f.spaces;
    f.catalog.spaces[1].status = 'occupied';
    f.tick(1010); await f.timers.shift()(); await new Promise(resolve => setImmediate(resolve));
    assert.equal(node.textContent, previousLabel);
    assert.equal(f.start.disabled, true);
    f.context.document.activeElement = null;
    f.spaces.listeners.blur();
    assert.match(node.textContent, /점유/);
    assert.equal(f.spaces.value, 'space_5');
});

test('preview circle covers half of slot area by default and input uses an area percentage', async () => {
    const f = await fixture();
    f.catalog.cases[0].arrival_region = {parking_area_px2: 10000, max_radius_px: 50,
        actual_area_ratio: 0.5, radius_px: Math.sqrt(5000 / Math.PI)};
    const area = f.elements.find(e => e.tag === 'input' && e.type === 'number');
    assert.equal(area.value, '50');
    area.listeners.input();
    assert.ok(Math.abs(f.drawing[0][2] - Math.sqrt(5000 / Math.PI)) < 0.001);
    area.value = '25'; area.listeners.input();
    assert.ok(Math.abs(f.drawing[0][2] - Math.sqrt(2500 / Math.PI)) < 0.001);
    assert.match(f.status.textContent, /주차칸 중앙/);
});

test('arrival commitment and own-rover exclusion are shown without changing the goal', async () => {
    const f = await fixture();
    f.setState({status: 'holding', case_id: '03_exit', parking_space_id: 'space_4', goal_px: [320, 230],
        radius_px: 20, guidance_available: true, arrival_committed: true,
        occupancy: {fresh: true, states: {space_4: 'free'}, rover_exclusion: {status: 'matched', excluded_count: 1}}});
    await f.timers.shift()();
    assert.match(f.status.textContent, /목표 P4/);
    assert.match(f.status.textContent, /현재 목표 고정/);
    assert.ok(f.elements.some(e => e.tag === 'p' && /ArUco로 로버 본체 제외/.test(e.textContent || '')));
});

test('missing observations retain comparison anchor and draw only a gray reference point without guidance', async () => {
    const f = await fixture();
    f.setState({status: 'holding', case_id: '03_exit', parking_space_id: 'space_4', goal_px: [320, 230],
        radius_px: 20, guidance_available: false, tracking_paused: true, reason: 'comparison_waiting_for_observation',
        current_position_px: null, last_known_position_px: [321, 230], held_seconds: 0.4, hold_seconds: 2,
        waypoints: [[180, 150], [320, 230]], occupancy: {fresh: true, states: {space_4: 'free'}}});
    await f.timers.shift()();
    assert.match(f.status.textContent, /위치 비교 간격 0.4 \/ 2s/);
    assert.match(f.status.textContent, /새 마커 관측 대기/);
    assert.match(f.status.textContent, /새 인식 없이는 성공하지 않습니다/);
    assert.equal(f.contexts[0].fillStyle, '#9ca3af');
    assert.equal(f.contexts[0].segments, 0);
    assert.equal(f.drawing.at(-1)[0], 321);
});

test('height correction label and blue point use the corrected server position', async () => {
    const f = await fixture();
    f.setState({status: 'holding', case_id: '03_exit', parking_space_id: 'space_5', goal_px: [260, 200],
        radius_px: 20, guidance_available: true, height_correction_applied: true,
        current_position_px: [261, 202]});
    await f.timers.shift()();
    assert.match(f.status.textContent, /높이 보정 적용/);
    assert.match(f.status.textContent, /마커 아래 바닥 위치/);
    assert.deepEqual(f.drawing.at(-1), [261, 202, 5, 0, 2 * Math.PI]);
    assert.equal(f.contexts[0].fillStyle, '#22d3ee');
});

test('uncorrected position and changed calibration are explicitly explained', async () => {
    const f = await fixture();
    assert.match(f.status.textContent, /높이 보정 미적용/);
    f.setState({status: 'invalidated', case_id: '03_exit', reason: 'position_correction_changed'});
    await f.timers.shift()();
    assert.match(f.status.textContent, /카메라 실측값이 바뀌었습니다/);
});
