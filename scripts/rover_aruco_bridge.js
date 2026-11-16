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
            this.canvas.width = video.videoWidth;
            this.canvas.height = video.videoHeight;
            this.ctx.drawImage(video, 0, 0);
            const capturedAt = new Date().toISOString();
            const blob = await new Promise(resolve => this.canvas.toBlob(resolve, 'image/png'));
            if (!blob) throw new Error('카메라 프레임 생성 실패');
            const response = await fetch('/api/aruco', {
                method: 'POST', body: blob,
                headers: {'Content-Type': 'image/png', 'X-Captured-At': capturedAt, 'X-Frame-Id': String(++this.frameId),
                    'X-Camera-Session': video.srcObject?.getVideoTracks?.()[0]?.id || '',
                    'X-Source-Type': this.media.currentSourceType,
                    'X-Source-Width': String(video.videoWidth), 'X-Source-Height': String(video.videoHeight)},
                signal: AbortSignal.timeout(500),
            });
            if (!response.ok) throw new Error(`ArUco 응답 오류 ${response.status}`);
            const data = await response.json();
            const now = performance.now();
            const roundTripMs = now - started;
            if (roundTripMs > 200) {
                data.tracking_status = 'stale';
                data.rover = null;
            }
            data.client_round_trip_ms = roundTripMs;
            const interval = this.lastReceivedAt === null ? null : now - this.lastReceivedAt;
            this.lastReceivedAt = now;
            this.lastResult = {...data, receivedAt: Date.now()};
            const lines = [
                `로버 ArUco ID ${data.rover_marker_id} · ${data.tracking_status}`,
                `원본 영상 ${data.width}×${data.height} px`,
                `목표 ${this.periodMs}ms / 실제 응답 간격 ${interval === null ? '측정 중' : interval.toFixed(0) + 'ms'} / 검출 ${data.processing_ms.toFixed(1)}ms`,
            ];
            if (data.rover) {
                lines.push(`마커 중심 x=${data.rover.center_px[0].toFixed(1)}, y=${data.rover.center_px[1].toFixed(1)} px`,
                    `로버 영상 방향각 ${data.rover.heading_deg.toFixed(1)}° (오른쪽 0°, 시계 방향 +)`);
            } else {
                lines.push('로버 마커 미검출/중복/만료: 현재 위치 없음');
            }
            lines.push('카메라 영상 좌표 · 마커 상단 기준. 실제 cm 위치·바닥 기준 방향은 보정 전입니다.');
            if (interval !== null && interval > 200) lines.push('갱신 간격 200ms 초과: 목표 주기를 유지하지 못하고 있습니다.');
            this.panel.textContent = lines.join('\n');
            window.dispatchEvent(new CustomEvent('aruco-result', {detail: this.lastResult}));
        } catch (error) {
            this.lastResult = null;
            this.panel.textContent = `로버 ArUco: ${error.message}\n현재 위치 없음. 서버와 카메라 상태를 확인하세요.`;
            window.dispatchEvent(new CustomEvent('aruco-result', {detail: null}));
        } finally {
            // 응답 완료 뒤 남은 시간만 기다려 목표 10Hz를 유지하고 요청 중첩·대기를 막는다.
            setTimeout(() => this.loop(), Math.max(0, this.periodMs - (performance.now() - started)));
        }
    }
}
