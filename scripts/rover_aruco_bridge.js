// ParkingHaeven이 이미 연 카메라를 공유한다. OpenCV에서 카메라를 다시 열지 않는다.
export class ArUcoBridge {
    constructor(mediaHandler) {
        this.media = mediaHandler;
        this.canvas = document.createElement('canvas');
        this.ctx = this.canvas.getContext('2d');
        this.panel = document.createElement('pre');
        this.panel.style.cssText = 'white-space:pre-wrap;background:#111827;color:#a7f3d0;padding:12px;margin:12px;border:1px solid #475569;';
        this.panel.textContent = '로버 ArUco: 카메라 대기 중';
        document.body.appendChild(this.panel);
        this.periodMs = 100;
        this.frameId = 0;
        this.lastReceivedAt = null;
        this.lastVideoTime = null;
        this.lastResult = null;
        this.ready = this.configure();
        this.loop();
    }

    async configure() {
        const response = await fetch('/api/aruco-config');
        if (!response.ok) throw new Error('ArUco 서버 설정을 읽지 못했습니다. 서버를 재시작하세요.');
        const config = await response.json();
        this.periodMs = config.period_ms;
    }

    async loop() {
        const started = performance.now();
        try {
            await this.ready;
            const video = this.media.videoElement;
            if (this.media.currentSourceType === 'image' || !video || video.readyState < 2 || !video.videoWidth || video.paused) {
                this.lastResult = null;
                this.panel.textContent = '로버 ArUco: 실시간 웹캠/비디오를 시작하세요.';
                window.dispatchEvent(new CustomEvent('aruco-result', {detail: null}));
                return;
            }
            if (video.currentTime === this.lastVideoTime) {
                this.lastResult = null;
                this.panel.textContent = '로버 ArUco: 새 영상 프레임 대기 중. 이전 위치를 갱신하지 않습니다.';
                window.dispatchEvent(new CustomEvent('aruco-result', {detail: null}));
                return;
            }
            this.lastVideoTime = video.currentTime;
            const source = {kind: this.media.currentSourceType, width: video.videoWidth, height: video.videoHeight,
                session: video.srcObject?.getVideoTracks?.()[0]?.id || ''};
            const sameSource = () => this.media.videoElement === video && this.media.currentSourceType === source.kind
                && video.videoWidth === source.width && video.videoHeight === source.height
                && (video.srcObject?.getVideoTracks?.()[0]?.id || '') === source.session;
            this.canvas.width = video.videoWidth;
            this.canvas.height = video.videoHeight;
            this.ctx.drawImage(video, 0, 0);
            const capturedAt = new Date().toISOString();
            const captureStarted = performance.now();
            // 지도 생성은 PNG를 유지하고 추적 프레임만 용량이 작은 JPEG로 보낸다.
            const blob = await new Promise(resolve => this.canvas.toBlob(resolve, 'image/jpeg', 0.85));
            if (!blob) throw new Error('카메라 프레임 생성 실패');
            if (!sameSource()) throw new Error('프레임 생성 중 카메라가 변경됐습니다.');
            const encodingMs = performance.now() - captureStarted;
            if (encodingMs > 200) throw new Error(`영상 인코딩 지연 ${encodingMs.toFixed(0)}ms: 오래된 프레임을 버렸습니다.`);
            const requestStarted = performance.now();
            const response = await fetch('/api/aruco', {
                method: 'POST', body: blob,
                headers: {'Content-Type': blob.type || 'image/jpeg', 'X-Captured-At': capturedAt, 'X-Frame-Id': String(++this.frameId),
                    'X-Camera-Session': source.session,
                    'X-Source-Type': source.kind,
                    'X-Source-Width': String(source.width), 'X-Source-Height': String(source.height)},
                signal: AbortSignal.timeout(1000),
            });
            if (!response.ok) throw new Error(`ArUco 응답 오류 ${response.status}`);
            const data = await response.json();
            if (!sameSource()) throw new Error('응답 대기 중 카메라가 변경됐습니다.');
            const now = performance.now();
            const roundTripMs = now - captureStarted;
            data.server_tracking_status = data.tracking_status;
            data.server_detection_status = data.detection_status || data.tracking_status;
            if (roundTripMs > 200) {
                data.tracking_status = 'stale';
                data.rover = null;
            }
            data.client_round_trip_ms = roundTripMs;
            data.encoding_ms = encodingMs;
            data.request_ms = now - requestStarted;
            const interval = this.lastReceivedAt === null ? null : now - this.lastReceivedAt;
            this.lastReceivedAt = now;
            this.lastResult = {...data, receivedAt: Date.now()};
            const lines = [
                `로버 ArUco ID ${data.rover_marker_id} · ${data.tracking_status}`,
                `원본 영상 ${data.width}×${data.height} px`,
                `목표 ${this.periodMs}ms / 실제 응답 간격 ${interval === null ? '측정 중' : interval.toFixed(0) + 'ms'} / 검출 ${data.processing_ms.toFixed(1)}ms`,
                `JPEG 인코딩 ${encodingMs.toFixed(1)}ms / 전송·응답 ${data.request_ms.toFixed(1)}ms / 서버 검출 ${data.server_detection_status} · 유효성 ${data.server_tracking_status}`,
            ];
            if (data.rover) {
                lines.push(`마커 중심 x=${data.rover.center_px[0].toFixed(1)}, y=${data.rover.center_px[1].toFixed(1)} px`,
                    `로버 영상 방향각 ${data.rover.heading_deg.toFixed(1)}° (오른쪽 0°, 시계 방향 +)`);
            } else {
                lines.push('로버 마커 미검출/중복/만료: 현재 위치 없음');
                if (data.server_detection_status === 'lost') lines.push('마커를 찾지 못했습니다. 로버 상단 ID 7 마커가 가려지지 않았는지 확인하세요.');
                if (data.tracking_status === 'stale') lines.push('프레임 지연 200ms 초과: 오래된 좌표를 사용하지 않습니다.');
            }
            lines.push('카메라 영상 좌표 · 마커 상단 기준. 실제 cm 위치·바닥 기준 방향은 보정 전입니다.');
            if (interval !== null && interval > 200) lines.push('갱신 간격 200ms 초과: 목표 주기를 유지하지 못하고 있습니다.');
            this.panel.textContent = lines.join('\n');
            window.dispatchEvent(new CustomEvent('aruco-result', {detail: this.lastResult}));
        } catch (error) {
            this.lastResult = null;
            const message = ['TimeoutError', 'AbortError'].includes(error.name)
                ? '전송·응답 시간 초과(1초). 네트워크 또는 브라우저 지연을 확인하세요.' : error.message;
            this.panel.textContent = `로버 ArUco: ${message}\n현재 위치 없음. 서버와 카메라 상태를 확인하세요.`;
            window.dispatchEvent(new CustomEvent('aruco-result', {detail: null}));
        } finally {
            // 응답 완료 뒤 남은 시간만 기다려 목표 10Hz를 유지하고 요청 중첩·대기를 막는다.
            setTimeout(() => this.loop(), Math.max(0, this.periodMs - (performance.now() - started)));
        }
    }
}
