// 같은 카메라 구도의 도착 시험. 목표는 주차칸 중앙의 면적 기반 원이다.
(() => {
    const panel = document.createElement('section');
    panel.style.cssText = 'margin:20px;padding:16px;border:2px solid #a78bfa;border-radius:8px;background:#111827;color:white;';
    const title = document.createElement('h2');
    title.textContent = '로버 도착 판정 테스트';
    const note = document.createElement('p');
    note.textContent = '상단 「모드 선택 → 감시 모드」에서 차량 인식을 실행하세요. 노란 원 안의 첫 위치와 2초 뒤 새로 인식한 위치가 일치하면 도착입니다. 중간 미검출은 허용하며, 이동이 확인되면 다시 비교합니다. 카메라 좌표를 등록하면 마커 아래 바닥 위치를 사용합니다.';
    const select = document.createElement('select');
    select.style.cssText = 'color:white;background:#1f2937;padding:8px;width:28rem;max-width:100%;';
    const area = document.createElement('input');
    area.type = 'number'; area.min = '10'; area.max = '75'; area.step = '5'; area.value = '50';
    area.style.cssText = 'color:white;background:#1f2937;padding:8px;width:80px;';
    const areaLabel = document.createElement('label');
    areaLabel.textContent = '도착 영역(칸 면적 %) '; areaLabel.append(area);
    const spaceSelect = document.createElement('select');
    spaceSelect.style.cssText = 'color:white;background:#1f2937;padding:8px;width:22rem;max-width:100%;';
    const auto = document.createElement('input');
    auto.type = 'checkbox'; auto.checked = true;
    const autoLabel = document.createElement('label');
    autoLabel.style.cssText = 'display:inline-flex;align-items:center;gap:8px;';
    areaLabel.style.cssText = 'display:inline-flex;align-items:center;gap:8px;';
    autoLabel.append(auto, document.createTextNode(' 더 적합한 빈칸으로 자동 변경'));
    const occupancyStatus = document.createElement('p');
    occupancyStatus.textContent = '상단 「모드 선택 → 감시 모드」를 누르세요. 빈칸은 2초 연속 확인 후 사용합니다.';
    const start = document.createElement('button');
    start.textContent = '테스트 시작';
    start.style.cssText = 'padding:10px;margin:8px;background:#6d28d9;border-radius:6px;';
    const stop = document.createElement('button');
    stop.textContent = '테스트 중단';
    stop.style.cssText = 'padding:10px;margin:8px;background:#374151;border-radius:6px;';
    const status = document.createElement('pre');
    status.style.cssText = 'white-space:pre-wrap;';
    const canvas = document.createElement('canvas');
    canvas.style.cssText = 'max-width:100%;height:auto;display:block;margin-top:12px;';
    const ctx = canvas.getContext('2d');
    const controls = document.createElement('div');
    controls.style.cssText = 'display:flex;flex-wrap:wrap;align-items:center;gap:12px;margin:12px 0;';
    controls.append(select, spaceSelect, autoLabel, areaLabel, start, stop);
    panel.append(title, note, controls, occupancyStatus, status, canvas);
    document.body.append(panel);
    const names = {'01_building_a':'A 건물 가까운 일반석', '02_accessible':'장애인석 지정',
        '03_exit':'출구 가까운 일반석', '04_building_b_accessible':'B 건물 가까운 장애인석',
        '05_entrance_accessible':'입구 가까운 장애인석'};
    let catalog = {ready:false,cases:[]}, state = {status:'idle'}, busy = false, mapBusy = false, image = null, actionEpoch = 0;
    let lastCatalogPoll = -Infinity, catalogBusy = false;
    let caseKeys = null, spaceKeys = null, bridgeError = '';
    const spaceOptions = new Map();
    const parkingName = id => id ? id.replace(/^space_(\d+)$/, 'P$1') : '선택 가능한 빈칸 없음';
    const active = () => state.case_id === select.value && !['idle', 'stopped', 'invalidated', 'unavailable'].includes(state.status);
    const previewRadius = circle => Math.min(Math.sqrt(circle.parking_area_px2 * Number(area.value) / 100 / Math.PI), circle.max_radius_px);

    function renderSpaces() {
        const previous = spaceSelect.value;
        const keys = JSON.stringify((catalog.spaces || []).map(item => item.parking_space_id));
        if (spaceKeys === keys && document.activeElement === spaceSelect) return;
        if (spaceKeys !== keys) {
            spaceSelect.replaceChildren(); spaceOptions.clear(); spaceKeys = keys;
            const automatic = document.createElement('option');
            automatic.value = ''; automatic.textContent = '조건에 따라 빈칸 자동 선택'; spaceSelect.append(automatic);
            for (const item of catalog.spaces || []) {
                const option = document.createElement('option'); option.value = item.parking_space_id;
                spaceOptions.set(item.parking_space_id, option); spaceSelect.append(option);
            }
        }
        const accessible = ['02_accessible', '04_building_b_accessible', '05_entrance_accessible'].includes(select.value);
        for (const item of catalog.spaces || []) {
            const option = spaceOptions.get(item.parking_space_id);
            const label = {free:'빈칸', occupied:'점유', unknown:'미확인'}[item.status] || '미확인';
            const text = `${parkingName(item.parking_space_id)} · ${label}${item.type === 'accessible' ? ' · 장애인석' : ''}`;
            if (option.textContent !== text) option.textContent = text;
            option.disabled = item.type !== (accessible ? 'accessible' : 'normal');
        }
        const valid = (catalog.spaces || []).some(item => item.parking_space_id === previous
            && item.type === (accessible ? 'accessible' : 'normal'));
        const desired = valid ? previous : '';
        if (spaceSelect.value !== desired) spaceSelect.value = desired;
    }

    async function request(url, data) {
        const response = await fetch(url, data === undefined ? {cache:'no-store'} : {
            method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(data)});
        const result = await response.json();
        if (!response.ok) throw new Error(result.error || `서버 응답 ${response.status}`);
        return result;
    }

    function render() {
        const live = active() ? state.occupancy : catalog.occupancy;
        if (live?.fresh) {
            const free = Object.values(live.states || {}).filter(value => value === 'free').length;
            const age = Number.isFinite(live.age_seconds) ? ` · 관측 ${live.age_seconds.toFixed(1)}초 전` : '';
            occupancyStatus.textContent = `점유 연결 정상 · 확인된 빈칸 ${free}개${age} · 만료 ${live.ttl_seconds || 5}초`;
            if (live.rover_exclusion?.excluded_count) occupancyStatus.textContent += ' · ArUco로 로버 본체 제외';
            if (live.rover_exclusion?.status === 'ambiguous_vehicle') occupancyStatus.textContent += ' · 로버/차량 구분 확인 필요';
        } else {
            occupancyStatus.textContent = '점유 관측 대기/만료 · 상단 「모드 선택 → 감시 모드」와 연결 상태를 확인하세요.';
        }
        if (bridgeError) occupancyStatus.textContent += `\n${bridgeError}`;
        const requested = (catalog.spaces || []).find(item => item.parking_space_id === spaceSelect.value);
        const selectedCase = catalog.cases.find(item => item.case_id === select.value);
        const available = requested ? requested.status === 'free' : selectedCase?.available;
        start.disabled = busy || mapBusy || !catalog.ready || !select.value || !available || Boolean(live && !live.fresh)
            || !Number.isFinite(Number(area.value)) || Number(area.value)<10 || Number(area.value)>75;
        select.disabled = busy || mapBusy;
        spaceSelect.disabled = busy || mapBusy;
        auto.disabled = busy || mapBusy || active();
        stop.disabled = busy;
        area.disabled = busy || mapBusy || ['running','holding','waiting_for_tracking','blocked'].includes(state.status);
        if (mapBusy) status.textContent = '지도 생성 중: 기존 테스트가 중단됐습니다.';
        else if (!catalog.ready) status.textContent = catalog.error || '카메라를 켜고 지도를 한 번 생성하세요.';
        else {
            const labels = {idle:'케이스를 선택하고 테스트를 시작하세요.', running:'목표로 이동 중',
                holding:'도착 영역 안에서 2초 전후 위치 비교 중', waiting_for_tracking:'현재 마커 관측 대기 중',
                success:'성공 — 도착 판정 완료', stopped:'테스트 중단', invalidated:'지도/카메라 변경: 지도를 확인하고 다시 시작하세요.',
                blocked:'빈칸 조건을 충족하지 않아 경로 안내 중단', unavailable:'요청을 완료하지 못했습니다.'};
            status.textContent = labels[state.status] || state.status;
            const correction = active() ? state : catalog;
            status.textContent += correction.height_correction_applied
                ? '\n높이 보정 적용 · 파란 점과 도착 판정은 마커 아래 바닥 위치 기준입니다.'
                : '\n높이 보정 미적용 · 카메라 실측 좌표를 등록하고 테스트를 다시 시작하세요.';
            const selected = active() ? state : selectedCase;
            if (selected?.parking_space_id && selected.goal_px) status.textContent += `\n목표 ${parkingName(selected.parking_space_id)} · (${selected.goal_px.map(v => Number(v).toFixed(1)).join(', ')}) px`;
            if (selected?.arrival_region) {
                const circle = selected.arrival_region;
                const percent = active() ? circle.actual_area_ratio * 100
                    : Math.PI * previewRadius(circle) ** 2 / circle.parking_area_px2 * 100;
                status.textContent += `\n주차칸 중앙 도착 영역 · 칸 면적 약 ${percent.toFixed(0)}%`;
                if (percent + 0.01 < (active() ? state.arrival_area_ratio * 100 : Number(area.value))) status.textContent += ' · 칸 내부에 맞춰 크기 제한';
            }
            if (!active() && requested?.status === 'occupied') status.textContent += '\n이미 점유된 주차칸입니다. 다른 빈칸을 선택하세요.';
            else if (!active() && !available) status.textContent += `\n${selectedCase?.message || '빈칸 확인 중이거나 최신 점유 정보가 없습니다.'}`;
            if (state.target_changes?.length && state.case_id === select.value) {
                const change = state.target_changes.at(-1);
                status.textContent += `\n목적지 변경: ${parkingName(change.from)} → ${parkingName(change.to)} (빈칸 상태 변경)`;
            }
            if (active() && state.arrival_committed) status.textContent += '\n도착 영역 진입 · 현재 목표 고정';
            if (active() && state.hold_seconds != null) {
                const distance = state.distance_px != null ? `${state.distance_px.toFixed(1)} px` : '현재 마커 관측 대기';
                status.textContent += `\n목표까지 ${distance} · 위치 비교 간격 ${(state.held_seconds || 0).toFixed(1)} / ${state.hold_seconds}s`;
                if (state.comparison_displacement_px != null) status.textContent += ` · 기준점 이동 ${state.comparison_displacement_px.toFixed(1)} / ${state.stability_px}px`;
                if (state.tracking_paused) status.textContent += ' (새 마커 관측 대기)';
            }
            if (active() && state.hold_reset_count) {
                const reasons = {comparison_expired:'비교 기준점의 유효 기간 초과',
                    marker_identity_uncertain:'마커 ID 중복/불일치', position_unstable:'마커 위치 흔들림',
                    position_correction_failed:'높이 보정 위치 계산 실패',
                    outside_arrival_region:'도착 원 밖으로 이동', occupancy_blocked:'주차칸 점유/미확인',
                    target_changed:'목표 변경', parking_route_changed:'점유 확인 후 경로 갱신'};
                status.textContent += `\n도착 확인 재시작 ${state.hold_reset_count}회 · 마지막: ${reasons[state.last_hold_reset?.reason] || state.last_hold_reset?.reason || '상태 변경'}`;
            }
            if (state.reason) {
                const reasons = {user_stopped:'사용자가 중단했습니다.', map_regeneration:'지도 재생성으로 중단했습니다.',
                    map_changed:'기준 지도가 갱신됐습니다.', camera_changed:'카메라 연결이 바뀌었습니다.',
                    camera_resolution_changed:'카메라 해상도가 바뀌었습니다.', tracking_stale:'200ms 동안 새 관측이 없습니다.',
                    tracking_unavailable:'유효한 마커 관측이 없습니다.',
                    position_correction_changed:'지도 보정 또는 카메라 실측값이 바뀌었습니다. 테스트를 다시 시작하세요.',
                    position_correction_failed:'보정된 위치를 계산할 수 없어 도착 확인을 중단했습니다.',
                    comparison_waiting_for_observation:'비교 기준점을 보관하고 새 마커 관측을 기다립니다. 최근 위치는 회색 참고점이며 새 인식 없이는 성공하지 않습니다.'};
                status.textContent += `\n${reasons[state.reason] || state.reason}`;
            }
            if (state.position_correction_error) status.textContent += `\n${state.position_correction_error}`;
        }
        if (!image) return;
        ctx.clearRect(0,0,canvas.width,canvas.height);
        ctx.drawImage(image,0,0);
        const selected = active() ? state : selectedCase;
        if (!selected?.goal_px || state.status === 'blocked' || !available && !active()) return;
        if (!active() && spaceSelect.value && spaceSelect.value !== selected.parking_space_id) return;
        const canGuide = !active() || state.guidance_available || state.status === 'success';
        const points = canGuide ? selected.waypoints || [] : [];
        if (points.length) {
            ctx.strokeStyle = '#38bdf8'; ctx.lineWidth = 3; ctx.beginPath();
            points.forEach((point, index) => index ? ctx.lineTo(...point) : ctx.moveTo(...point)); ctx.stroke();
        }
        const circle = selected.arrival_region;
        const r = active() ? state.radius_px : circle ? previewRadius(circle) : 15;
        const [x,y] = selected.goal_px;
        ctx.strokeStyle = state.status === 'success' ? '#22c55e' : '#facc15'; ctx.lineWidth = 3;
        ctx.beginPath(); ctx.arc(x,y,Math.max(0,Number(r)||0),0,2*Math.PI);
        ctx.fillStyle = 'rgba(250,204,21,0.16)'; ctx.fill(); ctx.stroke();
        const current = state.case_id === select.value && state.current_position_px;
        const point = current || (active() && state.tracking_paused && state.last_known_position_px);
        if (point) {
            ctx.fillStyle = current ? '#22d3ee' : '#9ca3af';ctx.beginPath();ctx.arc(point[0],point[1],5,0,2*Math.PI);ctx.fill();
        }
    }

    async function loadCases() {
        if (catalogBusy || mapBusy) return;
        catalogBusy = true;
        const epoch = actionEpoch;
        try {
            const previous = select.value;
            const incomingCatalog = await request('/api/arrival-tests');
            if (epoch !== actionEpoch || mapBusy) return;
            catalog = incomingCatalog;
            lastCatalogPoll = performance.now();
            const keys = JSON.stringify(catalog.cases.map(item => item.case_id));
            if (caseKeys !== keys) {
                select.replaceChildren(); caseKeys = keys;
                const placeholder = document.createElement('option');
                placeholder.value = ''; placeholder.textContent = '테스트케이스 선택'; select.append(placeholder);
                for (const item of catalog.cases) {
                    const option = document.createElement('option'); option.value = item.case_id;
                    option.textContent = names[item.case_id] || item.case_id;
                    select.append(option);
                }
            }
            const desired = catalog.cases.some(item => item.case_id === previous) ? previous : '';
            if (select.value !== desired) select.value = desired;
            renderSpaces();
            if (!catalog.ready) image = null;
            if (catalog.ready && image?.captureId !== catalog.capture_id) {
                const incoming = new Image();
                incoming.onload = () => {
                    if (catalog.map_image_sha256 !== incoming.mapHash) return;
                    image = incoming; canvas.width = catalog.image_size_px[0]; canvas.height = catalog.image_size_px[1]; render();
                };
                incoming.mapHash = catalog.map_image_sha256;
                incoming.captureId = catalog.capture_id;
                incoming.src = `/captures/latest/source.png?version=${catalog.capture_id}`;
            }
        } catch(error) {catalog = {ready:false,cases:[],error:error.message};}
        finally {catalogBusy = false;}
        render();
    }

    start.addEventListener('click', async () => {
        actionEpoch++;
        busy = true;render();
        try {
            state = await request('/api/arrival-test/start', {case_id:select.value, capture_id:catalog.capture_id,
                arrival_area_ratio:Number(area.value)/100, hold_seconds:2.0,
                parking_space_id:spaceSelect.value || null, auto_retarget:auto.checked});
        } catch(error) {state = {status:'unavailable',reason:error.message};}
        finally {busy=false;render();}
    });
    async function stopTest() {
        actionEpoch++;
        busy=true;render();
        try {state=await request('/api/arrival-test/stop',{});}
        catch(error) {state={status:'unavailable',reason:error.message};}
        finally {busy=false;render();}
    }
    stop.addEventListener('click',stopTest);
    select.addEventListener('change',()=>{renderSpaces();stopTest();});
    spaceSelect.addEventListener('change',stopTest);
    spaceSelect.addEventListener('blur',()=>{renderSpaces();render();});
    area.addEventListener('input',render);
    window.addEventListener('rover-map-start',()=>{actionEpoch++;mapBusy=true;state={status:'stopped'};render();});
    window.addEventListener('rover-map-end',()=>{mapBusy=false;loadCases();});
    window.addEventListener('rover-occupancy-status',event=>{
        const message = event.detail.message;
        bridgeError = message.includes('실패') ? message : '';
        render();
    });
    async function poll() {
        const epoch=actionEpoch;
        try {
            if (!busy) {
                const incoming=await request('/api/arrival-test');
                if (!busy && actionEpoch===epoch) state=incoming;
            }
        }
        catch(error) {state={status:'unavailable',reason:error.message};}
        if (performance.now() - lastCatalogPoll >= 1000) loadCases();
        render();setTimeout(poll,250);
    }
    loadCases();poll();
})();
