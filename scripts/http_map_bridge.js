// ParkingHaeven 원본 파일을 변경하지 않고 수신 서버가 이 스크립트를 추가한다.
(() => {
    const panel = document.createElement('section');
    panel.style.cssText = 'margin:20px;padding:16px;border:2px solid #38bdf8;border-radius:8px;background:#111827;color:white;';
    const title = document.createElement('h2');
    title.textContent = 'Rover — HTTP PNG 지도 생성';
    const note = document.createElement('p');
    note.textContent = '차량을 모두 치우고 카메라 구도를 고정한 뒤 전송하세요. 원본 캔버스만 보내며 기본 지도는 덮어쓰지 않습니다.';
    const button = document.createElement('button');
    button.textContent = '현재 화면 PNG 전송 · 맵 생성';
    button.style.cssText = 'padding:10px;margin:10px 0;background:#0369a1;color:white;border-radius:6px;';
    const status = document.createElement('pre');
    status.style.cssText = 'white-space:pre-wrap;';
    status.textContent = '카메라를 시작하거나 빈 지도 사진을 불러오세요.';
    const links = document.createElement('div');
    panel.append(title, note, button, status, links);
    document.body.append(panel);

    button.addEventListener('click', async () => {
        button.disabled = true;
        links.replaceChildren();
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
            status.textContent = `${frame.width}×${frame.height} PNG 전송 및 맵 생성 중…`;
            const response = await fetch('/api/map-capture', {
                method: 'POST', headers: {'Content-Type': 'image/png', 'X-Captured-At': capturedAt}, body: blob,
            });
            const result = await response.json();
            status.textContent = response.ok
                ? `맵 생성 완료\n노드 ${result.graph.nodes}, 간선 ${result.graph.edges}, 연결 ${result.graph.connected}\n일반석 ${result.detected.normal}, 장애인석 ${result.detected.accessible}\n저장 위치: ${result.output_directory}`
                : `맵 생성 실패: ${result.error}\n${result.output_directory || ''}\n받은 사진과 힌트 위치를 확인하세요.`;
            const labels = {source: '받은 PNG', map: '처리된 지도', preview: '그래프 미리보기', graph: '그래프 JSON', semantic: '의미 지도 JSON'};
            for (const [key, url] of Object.entries(result.files || {})) {
                const link = document.createElement('a');
                link.href = url;
                link.target = '_blank';
                link.rel = 'noopener';
                link.textContent = labels[key] || key;
                link.style.cssText = 'display:inline-block;margin-right:16px;color:#7dd3fc;text-decoration:underline;';
                links.append(link);
            }
        } catch (error) {
            status.textContent = `실패: ${error.message}`;
        } finally {
            button.disabled = false;
        }
    });
})();
