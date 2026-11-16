// 월드 보정 전 도착 시험. 목표는 고정 지도에서 선택한 경로의 도로 진입점이다.
(() => {
    const panel = document.createElement('section');
    panel.style.cssText = 'margin:20px;padding:16px;border:2px solid #a78bfa;border-radius:8px;background:#111827;color:white;';
    const title = document.createElement('h2');
    title.textContent = '로버 도착 판정 테스트';
    const note = document.createElement('p');
    note.textContent = '카메라 구도를 고정하고 로버를 수동으로 이동하세요. 픽셀 기준 도로 진입점 도착 시험이며, 주차 완료·자동 주행 판정은 아닙니다.';
    const select = document.createElement('select');
    select.style.cssText = 'color:white;background:#1f2937;padding:8px;margin:8px;';
    const radius = document.createElement('input');
    radius.type = 'number'; radius.min = '3'; radius.max = '100'; radius.value = '15';
    radius.style.cssText = 'color:white;background:#1f2937;padding:8px;width:80px;';
    const radiusLabel = document.createElement('label');
    radiusLabel.textContent = '허용 반경(px) '; radiusLabel.append(radius);
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
    panel.append(title, note, select, radiusLabel, start, stop, status, canvas);
    document.body.append(panel);
    const names = {'01_building_a':'A 건물 가까운 일반석', '02_accessible':'장애인석 지정',
        '03_exit':'출구 가까운 일반석', '04_building_b_accessible':'B 건물 가까운 장애인석',
        '05_entrance_accessible':'입구 가까운 장애인석'};
    let catalog = {ready:false,cases:[]}, state = {status:'idle'}, busy = false, mapBusy = false, image = null, actionEpoch = 0;

    async function request(url, data) {
        const response = await fetch(url, data === undefined ? {cache:'no-store'} : {
            method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(data)});
        const result = await response.json();
        if (!response.ok) throw new Error(result.error || `서버 응답 ${response.status}`);
        return result;
    }

    function render() {
        start.disabled = busy || mapBusy || !catalog.ready || !select.value || !Number.isFinite(Number(radius.value)) || Number(radius.value)<3 || Number(radius.value)>100;
        select.disabled = busy || mapBusy;
        stop.disabled = busy;
        radius.disabled = busy || mapBusy || ['running','holding','waiting_for_tracking'].includes(state.status);
        if (mapBusy) status.textContent = '지도 생성 중: 기존 테스트가 중단됐습니다.';
        else if (!catalog.ready) status.textContent = catalog.error || '카메라를 켜고 지도를 한 번 생성하세요.';
        else {
            const labels = {idle:'케이스를 선택하고 테스트를 시작하세요.', running:'목표로 이동 중',
                holding:'도착 영역 안에서 위치 안정성 확인 중', waiting_for_tracking:'현재 마커 관측 대기 중',
                success:'성공 — 도착 판정 완료', stopped:'테스트 중단', invalidated:'지도/카메라 변경: 지도를 확인하고 다시 시작하세요.',
                unavailable:'서버 상태를 확인할 수 없습니다.'};
            status.textContent = labels[state.status] || state.status;
            const selected = catalog.cases.find(item => item.case_id === select.value);
            if (selected) status.textContent += `\n목표 ${selected.parking_space_id.replace(/^space_(\d+)$/, 'P$1')} · (${selected.goal_px.join(', ')}) px`;
            if (state.distance_px != null && state.case_id === select.value) {
                status.textContent += `\n목표까지 ${state.distance_px.toFixed(1)} px · 안정 유지 ${(state.held_seconds || 0).toFixed(1)} / ${state.hold_seconds}s`;
            }
            if (state.reason) {
                const reasons = {user_stopped:'사용자가 중단했습니다.', map_regeneration:'지도 재생성으로 중단했습니다.',
                    map_changed:'기준 지도가 갱신됐습니다.', camera_changed:'카메라 연결이 바뀌었습니다.',
                    camera_resolution_changed:'카메라 해상도가 바뀌었습니다.', tracking_stale:'200ms 동안 새 관측이 없습니다.',
                    tracking_unavailable:'유효한 마커 관측이 없습니다.'};
                status.textContent += `\n${reasons[state.reason] || state.reason}`;
            }
        }
        if (!image) return;
        ctx.clearRect(0,0,canvas.width,canvas.height);
        ctx.drawImage(image,0,0);
        const selected = catalog.cases.find(item => item.case_id === select.value);
        if (!selected) return;
        const r = state.case_id === select.value ? state.radius_px : Number(radius.value);
        const [x,y] = selected.goal_px;
        ctx.strokeStyle = state.status === 'success' ? '#22c55e' : '#facc15'; ctx.lineWidth = 3;
        ctx.beginPath(); ctx.arc(x,y,Math.max(0,Number(r)||0),0,2*Math.PI); ctx.stroke();
        const point = state.case_id === select.value && state.current_position_px;
        if (point) {
            ctx.fillStyle = '#22d3ee';ctx.beginPath();ctx.arc(point[0],point[1],5,0,2*Math.PI);ctx.fill();
        }
    }

    async function loadCases() {
        try {
            const previous = select.value;
            catalog = await request('/api/arrival-tests');
            select.replaceChildren();
            const placeholder = document.createElement('option');
            placeholder.value = ''; placeholder.textContent = '테스트케이스 선택'; select.append(placeholder);
            for (const item of catalog.cases) {
                const option = document.createElement('option'); option.value = item.case_id;
                option.textContent = `${names[item.case_id] || item.case_id} → ${item.parking_space_id.replace(/^space_(\d+)$/, 'P$1')}`;
                select.append(option);
            }
            select.value = catalog.cases.some(item => item.case_id === previous) ? previous : '';
            image = null;
            if (catalog.ready) {
                const incoming = new Image();
                incoming.onload = () => {
                    if (catalog.map_image_sha256 !== incoming.mapHash) return;
                    image = incoming; canvas.width = catalog.image_size_px[0]; canvas.height = catalog.image_size_px[1]; render();
                };
                incoming.mapHash = catalog.map_image_sha256;
                incoming.src = `/captures/latest/source.png?version=${catalog.capture_id}`;
            }
        } catch(error) {catalog = {ready:false,cases:[],error:error.message};}
        render();
    }

    start.addEventListener('click', async () => {
        actionEpoch++;
        busy = true;render();
        try {
            state = await request('/api/arrival-test/start', {case_id:select.value, capture_id:catalog.capture_id,
                radius_px:Number(radius.value), hold_seconds:1.0});
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
    select.addEventListener('change',stopTest);
    radius.addEventListener('input',render);
    window.addEventListener('rover-map-start',()=>{actionEpoch++;mapBusy=true;state={status:'stopped'};render();});
    window.addEventListener('rover-map-end',()=>{mapBusy=false;loadCases();});
    async function poll() {
        const epoch=actionEpoch;
        try {
            if (!busy) {
                const incoming=await request('/api/arrival-test');
                if (!busy && actionEpoch===epoch) state=incoming;
            }
        }
        catch(error) {state={status:'unavailable',reason:error.message};}
        render();setTimeout(poll,250);
    }
    loadCases();poll();
})();
