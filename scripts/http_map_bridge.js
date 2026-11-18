// ParkingHaeven 원본 파일을 변경하지 않고 수신 서버가 이 스크립트를 추가한다.
(() => {
    const panel = document.createElement('section');
    panel.style.cssText = 'margin:20px;padding:16px;border:2px solid #38bdf8;border-radius:8px;background:#111827;color:white;';
    const title = document.createElement('h2');
    title.textContent = 'Rover — HTTP PNG 지도 생성';
    const note = document.createElement('p');
    note.textContent = '차량을 모두 치우고 A/B, P1~P6, EN/EX가 보이게 전송하세요. 새 사진의 OCR로 지도를 만들며 기본 지도는 덮어쓰지 않습니다.';
    const button = document.createElement('button');
    button.textContent = '현재 화면 PNG 전송 · 맵 생성';
    button.style.cssText = 'padding:10px;margin:10px 0;background:#0369a1;color:white;border-radius:6px;';
    const status = document.createElement('pre');
    status.style.cssText = 'white-space:pre-wrap;';
    status.textContent = '카메라를 시작하거나 빈 지도 사진을 불러오세요.';
    const links = document.createElement('div');
    const routes = document.createElement('div');
    panel.append(title, note, button, status, links, routes);
    document.body.append(panel);

    button.addEventListener('click', async () => {
        button.disabled = true;
        links.replaceChildren();
        routes.replaceChildren();
        try {
            const source = document.getElementById('main-canvas');
            if (!source || source.width < 100 || source.height < 100) {
                throw new Error('원본 영상이 준비되지 않았습니다. 카메라를 먼저 시작하세요.');
            }
            // PNG 인코딩 중 라이브 캔버스가 바뀌지 않도록 한 프레임을 복사한다.
            const frame = document.createElement('canvas');
            frame.width = source.width;
            frame.height = source.height;
            frame.getContext('2d').drawImage(source, 0, 0);
            const capturedAt = new Date().toISOString();
            const blob = await new Promise(resolve => frame.toBlob(resolve, 'image/png'));
            if (!blob) throw new Error('PNG 인코딩 실패');
            window.dispatchEvent(new CustomEvent('rover-map-start'));
            status.textContent = `${frame.width}×${frame.height} PNG 전송 · OCR · 지도 및 경로 생성 중…`;
            const response = await fetch('/api/map-capture', {
                method: 'POST', headers: {'Content-Type': 'image/png', 'X-Captured-At': capturedAt,
                    'X-Camera-Session': window.parkingApp?.mediaHandler?.videoElement?.srcObject?.getVideoTracks?.()[0]?.id || '',
                    'X-Source-Type': window.parkingApp?.mediaHandler?.currentSourceType || '',
                    'X-Source-Width': String(window.parkingApp?.mediaHandler?.videoElement?.videoWidth || 0),
                    'X-Source-Height': String(window.parkingApp?.mediaHandler?.videoElement?.videoHeight || 0)}, body: blob,
            });
            const result = await response.json();
            window.dispatchEvent(new CustomEvent('rover-map-result', {detail: result}));
            status.textContent = response.ok
                ? `맵 생성 완료 (${result.generation_method === 'ocr' ? '새 사진 OCR' : '기존 힌트'})\n노드 ${result.graph.nodes}, 간선 ${result.graph.edges}, 연결 ${result.graph.connected}\n일반석 ${result.detected.normal}, 장애인석 ${result.detected.accessible}\n저장 위치: ${result.output_directory}`
                : `맵 생성 실패: ${result.error}\n${result.output_directory || ''}\n받은 PNG와 OCR 미리보기를 확인하세요.`;
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
            const labels = {source: '받은 PNG', map: '처리된 지도', labels: 'OCR 힌트 JSON',
                ocr_report: 'OCR 보고서', ocr_preview: 'OCR 미리보기',
                preview: '그래프 미리보기', graph: '그래프 JSON', semantic: '의미 지도 JSON',
                calibration: '보정 보고서', floor_points: '자동 기준점 JSON'};
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
            window.dispatchEvent(new CustomEvent('rover-map-end'));
        }
    });
})();
