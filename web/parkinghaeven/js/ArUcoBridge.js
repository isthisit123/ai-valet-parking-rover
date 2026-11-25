// The browser owns the webcam. Send full-resolution frames to local OpenCV.
// Pixel positions only; never present these as map coordinates or vehicle yaw.
export class ArUcoBridge {
    constructor(mediaHandler) {
        this.media = mediaHandler;
        this.canvas = document.createElement('canvas');
        this.ctx = this.canvas.getContext('2d');
        this.panel = document.createElement('pre');
        this.panel.style.cssText = 'white-space:pre-wrap;background:#111827;color:#a7f3d0;padding:12px;margin:12px;font-size:13px;border:1px solid #475569;';
        this.panel.textContent = 'ArUco: 카메라 대기 중';
        document.body.appendChild(this.panel);
        this.lastResult = null;
        this.loop();
    }

    async loop() {
        try {
            const video = this.media.videoElement;
            if (this.media.currentSourceType === 'image' || !video || video.readyState < 2) {
                this.panel.textContent = 'ArUco: 실시간 웹캠/비디오 대기 중';
                return;
            }
            this.canvas.width = video.videoWidth;
            this.canvas.height = video.videoHeight;
            this.ctx.drawImage(video, 0, 0);
            const blob = await new Promise(resolve => this.canvas.toBlob(resolve, 'image/png'));
            if (!blob) throw new Error('프레임 생성 실패');
            const response = await fetch('/api/aruco', {
                method: 'POST', headers: {'Content-Type': 'image/png'}, body: blob,
                signal: AbortSignal.timeout(3000)
            });
            if (!response.ok) throw new Error(`ArUco 서버 오류 ${response.status}`);
            const data = await response.json();
            this.lastResult = {...data, receivedAt: Date.now()};
            const lines = data.markers.map(m => `ID ${m.id} | u=${m.center_px[0].toFixed(1)}, v=${m.center_px[1].toFixed(1)} px`);
            this.panel.textContent = `ArUco 원본 ${data.width}×${data.height}\n` +
                (lines.join('\n') || '마커 미검출') + '\n지도 (x,y,θ): 카메라/지도 보정 필요';
            window.dispatchEvent(new CustomEvent('aruco-result', {detail: this.lastResult}));
        } catch (error) {
            this.lastResult = null;
            this.panel.textContent = `ArUco: ${error.message}\naruco_server.py로 실행했는지 확인합니다.`;
            window.dispatchEvent(new CustomEvent('aruco-result', {detail: null}));
        } finally {
            setTimeout(() => this.loop(), 300);
        }
    }
}
