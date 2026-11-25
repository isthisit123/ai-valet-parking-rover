// ParkingHaeven 원본 파일을 변경하지 않고 수신 서버가 이 스크립트를 추가한다.
(() => {
    const panel = document.createElement('section');
    panel.style.cssText = 'margin:20px;padding:16px;border:2px solid #38bdf8;border-radius:8px;background:#111827;color:white;';
    const title = document.createElement('h2');
    title.textContent = 'Rover — HTTP PNG 지도 생성';
    const note = document.createElement('p');
    note.textContent = '차량을 모두 치우고 A/B, P1~P6, EN/EX가 보이게 전송하세요. 새 사진의 OCR로 지도를 만들며 기본 지도는 덮어쓰지 않습니다.';
    const cameraFields = document.createElement('fieldset');
    cameraFields.style.cssText = 'display:flex;gap:14px;flex-wrap:wrap;padding:12px;border:1px solid #64748b;';
    const legend = document.createElement('legend');
    legend.textContent = '높이 보정용 카메라 좌표 (cm)';
    cameraFields.append(legend);
    const inputs = {};
    for (const [key, caption, placeholder, value] of [
        ['x', '렌즈 X', '예: 53', ''], ['y', '렌즈 Y', '예: 151.7', ''],
        ['z', '렌즈 높이 Z', '예: 111.5', ''], ['height', '마커 높이', '10.5', '10.5'],
    ]) {
        const label = document.createElement('label');
        label.textContent = caption + ' ';
        const input = document.createElement('input');
        input.type = 'number'; input.step = 'any'; input.id = 'rover-camera-' + key;
        input.placeholder = placeholder; input.value = value;
        input.style.cssText = 'width:110px;padding:7px;background:#fff;color:#111827;';
        if (key === 'height') input.min = '0';
        inputs[key] = input;
        label.append(input); cameraFields.append(label);
    }
    const cameraNote = document.createElement('p');
    cameraNote.textContent = 'X·Y는 렌즈 바로 아래 점의 전지 기준 좌표, Z는 전지 표면부터 렌즈까지 수직 높이입니다. 전지 왼쪽 위가 원점이며 왼쪽·위쪽은 음수입니다. 입력한 값은 cm 보정 성공 시 이번 지도에 등록합니다. 카메라를 옮기면 다시 측정하세요. X·Y·Z를 모두 비우면 좌표 등록 없이 지도를 생성합니다.';
    const button = document.createElement('button');
    button.textContent = '현재 화면 PNG 전송 · 맵 생성';
    button.style.cssText = 'padding:10px;margin:10px 0;background:#0369a1;color:white;border-radius:6px;';
    const status = document.createElement('pre');
    status.style.cssText = 'white-space:pre-wrap;';
    status.textContent = '카메라를 시작하거나 빈 지도 사진을 불러오세요.';
    const links = document.createElement('div');
    const routes = document.createElement('div');
    panel.append(title, note, cameraFields, cameraNote, button, status, links, routes);
    document.body.append(panel);

    button.addEventListener('click', async () => {
        button.disabled = true;
        links.replaceChildren();
        routes.replaceChildren();
        try {
            const values = ['x', 'y', 'z'].map(key => inputs[key].value.trim());
            let measurement = null;
            if (values.some(value => value !== '')) {
                const position = values.map(Number);
                const height = Number(inputs.height.value);
                if (values.some(value => value === '') || inputs.height.value.trim() === ''
                        || !position.every(Number.isFinite) || !Number.isFinite(height)
                        || height < 0 || position[2] <= height) {
                    throw new Error('렌즈 X·Y·Z와 마커 높이를 모두 숫자로 입력하세요. Z는 마커 높이보다 커야 합니다.');
                }
                measurement = {camera_position_cm: position, marker_height_cm: height};
            }
            const source = document.getElementById('main-canvas');
            if (!source || source.width < 100 || source.height < 100) {
                throw new Error('원본 영상이 준비되지 않았습니다. 카메라를 먼저 시작하세요.');
            }
            // PNG 인코딩 중 라이브 캔버스가 바뀌지 않도록 한 프레임을 복사한다.
            const frame = document.createElement('canvas');
            const captured = window.RoverParkingRegions.snapshot();
            if (measurement && (captured.kind !== 'webcam' || !captured.session_id)) {
                throw new Error('카메라 좌표를 등록하려면 먼저 웹캠을 시작하세요.');
            }
            for (const input of Object.values(inputs)) input.disabled = true;
            const native = Boolean(window.parkingApp?.mediaHandler?.serverCamera);
            frame.width = native ? captured.width : source.width;
            frame.height = native ? captured.height : source.height;
            if (!native) frame.getContext('2d').drawImage(source, 0, 0);
            const capturedAt = new Date().toISOString();
            const blob = native ? null : await new Promise(resolve => frame.toBlob(resolve, 'image/png'));
            if (!native && !blob) throw new Error('PNG 인코딩 실패');
            window.dispatchEvent(new CustomEvent('rover-map-start'));
            status.textContent = `${frame.width}×${frame.height} PNG 전송 · OCR · 지도 및 경로 생성 중…`;
            const response = await fetch(native ? '/api/camera/map-capture' : '/api/map-capture', {
                method: 'POST', headers: {'Content-Type': 'image/png', 'X-Captured-At': capturedAt,
                    'X-Camera-Session': captured.session_id,
                    'X-Source-Type': captured.kind,
                    'X-Source-Width': String(captured.width),
                    'X-Source-Height': String(captured.height),
                    ...(measurement ? {'X-Rover-Camera-Position': JSON.stringify(measurement)} : {})}, body: blob,
            });
            const result = await response.json();
            window.dispatchEvent(new CustomEvent('rover-map-result', {detail: result}));
            status.textContent = response.ok
                ? `맵 생성 완료 (${result.generation_method === 'ocr' ? '새 사진 OCR' : '기존 힌트'})\n노드 ${result.graph.nodes}, 간선 ${result.graph.edges}, 연결 ${result.graph.connected}\n일반석 ${result.detected.normal}, 장애인석 ${result.detected.accessible}\n저장 위치: ${result.output_directory}`
                : `맵 생성 실패: ${result.error}\n${result.output_directory || ''}\n받은 PNG와 OCR 미리보기를 확인하세요.`;
            if (response.ok && result.published_as_latest) {
                try {
                    const count = window.RoverParkingRegions.apply(result, captured);
                    status.textContent += `\nP1~P6 ${count}개 구역 자동 적용·저장 완료. 설정 모드에서 꼭짓점을 확인하세요. 점유는 미확인입니다.`;
                } catch (error) {
                    status.textContent += `\n자동 구역 적용 실패 (기존 설정 유지): ${error.message}`;
                }
            }
            const unresolved = (result.ocr?.labels || []).filter(item => item.status !== 'recognized');
            if (unresolved.length) {
                status.textContent += '\n이름표 확인 필요: ' + unresolved.map(item => `${item.id} (${item.status})`).join(', ');
            }
            if (result.latest_preserved) {
                status.textContent += `\n직전 성공 결과 유지: ${result.latest_output_directory}`;
            }
            const world = result.world_calibration;
            if (world?.status === 'verified') {
                status.textContent += `\ncm 보정 검증 통과 · 확인점 오차 ${world.validation.max_held_out_error_cm.toFixed(2)} cm`;
            } else if (world?.status === 'estimated') {
                status.textContent += '\ncm 추정 미리보기 생성 · 독립 확인점 필요';
            } else if (world?.status === 'failed') {
                status.textContent += `\ncm 보정 실패: ${world.error}`;
            }
            if (result.camera_position?.status === 'saved') {
                status.textContent += `\n카메라 좌표 등록 완료: [${result.camera_position.camera_position_cm.join(', ')}] cm · 마커 높이 ${result.camera_position.marker_height_cm} cm. 위치 확인 프로그램에서 저장값을 사용할 수 있습니다.`;
            } else if (result.camera_position?.status === 'not_saved') {
                status.textContent += `\n카메라 좌표 미등록: ${result.camera_position.error}`;
            }
            const labels = {source: '받은 PNG', map: '처리된 지도', labels: 'OCR 힌트 JSON',
                ocr_report: 'OCR 보고서', ocr_preview: 'OCR 미리보기',
                preview: '그래프 미리보기', graph: '그래프 JSON', semantic: '의미 지도 JSON',
                calibration: '보정 보고서', floor_points: '자동 기준점 JSON', parking_regions: '주차칸 꼭짓점 JSON',
                camera_position: '카메라 좌표 설정'};
            for (const [key, url] of Object.entries({...result.files, ...world?.files})) {
                const link = document.createElement('a');
                link.href = url;
                link.target = '_blank';
                link.rel = 'noopener';
                link.textContent = labels[key] || key;
                link.style.cssText = 'display:inline-block;margin-right:16px;color:#7dd3fc;text-decoration:underline;';
                links.append(link);
            }
            const routeResult = result.route_generation;
            if (routeResult?.status === 'failed') {
                status.textContent += `\n지도는 생성됐지만 경로 검증에 실패했습니다: ${routeResult.error}`;
            } else if (routeResult?.status === 'generated') {
                status.textContent += `\n경로 ${routeResult.cases.length}개 생성 · 도로 영역 검사 통과`;
                const heading = document.createElement('h3');
                heading.textContent = '조건별 경로';
                const explanation = document.createElement('p');
                explanation.textContent = '정적 지도 시험 · 점유 미반영 · 픽셀 좌표. 경로는 주차칸의 도로 진입점까지이며, 실제 주행에는 cm 보정이 필요합니다.';
                routes.append(heading, explanation);
                const names = {
                    '01_building_a': 'A 건물 가까운 일반석',
                    '02_accessible': '장애인석 지정',
                    '03_exit': '출구 가까운 일반석',
                    '04_building_b_accessible': 'B 건물 가까운 장애인석',
                    '05_entrance_accessible': '입구 가까운 장애인석',
                };
                const worldCases = new Map((world?.cases || []).map(route => [route.case_id, route.files]));
                for (const route of routeResult.cases) {
                    const row = document.createElement('div');
                    row.style.cssText = 'display:flex;gap:14px;align-items:center;flex-wrap:wrap;margin:12px 0;';
                    const summary = document.createElement('span');
                    const space = route.parking_space_id.replace(/^space_(\d+)$/, 'P$1');
                    summary.textContent = `${names[route.case_id] || route.case_id} → ${space} · ${route.total_distance.toFixed(2)} px`;
                    row.append(summary);
                    const worldFiles = worldCases.get(route.case_id);
                    for (const [label, url, download] of [
                        ['PNG 보기', route.files.preview, false],
                        ['PNG 저장', route.files.preview, true],
                        ['waypoint JSON 저장', route.files.waypoints, true],
                        ['cm 지도 보기', worldFiles?.preview, false],
                        ['cm 경로 JSON 저장', worldFiles?.waypoints, true],
                    ]) {
                        if (!url) continue;
                        const link = document.createElement('a');
                        link.textContent = label;
                        link.href = url;
                        link.style.cssText = 'color:#7dd3fc;text-decoration:underline;';
                        if (download) link.download = url.split('/').pop();
                        else { link.target = '_blank'; link.rel = 'noopener'; }
                        row.append(link);
                    }
                    routes.append(row);
                }
            }
        } catch (error) {
            status.textContent = `실패: ${error.message}`;
        } finally {
            button.disabled = false;
            for (const input of Object.values(inputs)) input.disabled = false;
            window.dispatchEvent(new CustomEvent('rover-map-end'));
        }
    });
})();
