// 웹캠/이미지/비디오 통합 미디어 처리
export class MediaHandler {
    constructor(canvas) {
        this.canvas = canvas;
        this.ctx = canvas.getContext('2d', { willReadFrequently: true });
        this.currentSourceType = 'webcam';
        this.currentFile = null;
        this.videoElement = null;
        this.animationFrameId = null;
        this.isProcessing = false;
        this.serverCamera = null;
        this.cameraControl = Promise.resolve();
    }

    // 시작·모드 변경·정지를 클릭한 순서대로 적용한다.
    cameraCommand(action, data = {}) {
        const request = this.cameraControl.then(async () => {
            const response = await fetch('/api/camera/' + action, {method: 'POST',
                headers: {'Content-Type': 'application/json'}, body: JSON.stringify(data)});
            const result = await response.json();
            if (!response.ok) throw new Error(result.error);
            return result;
        });
        this.cameraControl = request.catch(() => {});
        return request;
    }

    async showCameraFrame(source) {
        const response = await fetch('/api/camera/preview.jpg?session=' + encodeURIComponent(source.session_id),
            {cache: 'no-store', signal: AbortSignal.timeout(2000)});
        if (!response.ok) throw new Error((await response.json()).error);
        const bitmap = await createImageBitmap(await response.blob());
        try {
            if (source !== this.serverCamera) return;
            this.ctx.drawImage(bitmap, 0, 0, this.canvas.width, this.canvas.height);
        } finally { bitmap.close(); }
    }

    async previewCamera(source) {
        if (source !== this.serverCamera) return;
        try { await this.showCameraFrame(source); }
        catch (error) {
            if (source !== this.serverCamera) return;
            this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
            document.getElementById('canvas-status').textContent = '카메라 미리보기 대기 · ' + error.message;
        }
        if (source === this.serverCamera) setTimeout(() => this.previewCamera(source), 100);
    }

    async initWebcam(deviceId = null) {
        try {
            if (Number.isInteger(window.roverCameraIndex)) {
                const source = await this.cameraCommand('start');
                this.stopProcessing();
                this.videoElement?.pause();
                this.currentSourceType = 'webcam';
                this.currentFile = null;
                this.serverCamera = source;
                const scale = Math.min(this.canvas.parentElement.clientWidth / source.width, 1);
                this.canvas.width = Math.round(source.width * scale);
                this.canvas.height = Math.round(source.height * scale);
                await this.showCameraFrame(source);
                setTimeout(() => this.previewCamera(source), 100);
                return {success: true};
            }
            const constraints = {
                video: { width: { ideal: 1920 }, height: { ideal: 1080 }, ...(deviceId ? { deviceId: { exact: deviceId } } : {}) }
            };
            const stream = await navigator.mediaDevices.getUserMedia(constraints);

            if (!this.videoElement) {
                this.videoElement = document.createElement('video');
                this.videoElement.autoplay = true;
                this.videoElement.muted = true;
            }

            this.videoElement.srcObject = stream;
            await this.videoElement.play();

            this.currentSourceType = 'webcam';
            this.currentFile = null;
            this.startProcessing();

            return { success: true, stream };
        } catch (e) {
            if (this.serverCamera) this.stop();
            console.error('Webcam init failed:', e);
            return { success: false, error: e.message };
        }
    }

    async loadImage(file) {
        return new Promise((resolve, reject) => {
            const img = new Image();
            img.onload = () => {
                this.currentSourceType = 'image';
                this.currentFile = file;
                this.stopProcessing();
                this.renderImageToCanvas(img);
                resolve({ success: true, image: img });
            };
            img.onerror = () => reject(new Error('Failed to load image'));
            img.src = URL.createObjectURL(file);
        });
    }

    async loadVideo(file) {
        try {
            if (!this.videoElement) {
                this.videoElement = document.createElement('video');
                this.videoElement.loop = true;
                this.videoElement.muted = true;
                this.videoElement.playsInline = true;
            }

            this.videoElement.srcObject = null;
            this.videoElement.src = URL.createObjectURL(file);

            await new Promise((resolve, reject) => {
                this.videoElement.onloadedmetadata = resolve;
                this.videoElement.onerror = reject;
            });

            await this.videoElement.play();

            this.currentSourceType = 'video';
            this.currentFile = file;
            this.startProcessing();

            return { success: true };
        } catch (e) {
            console.error('Video load failed:', e);
            return { success: false, error: e.message };
        }
    }

    renderImageToCanvas(img) {
        const maxWidth = this.canvas.parentElement.clientWidth;
        const scale = Math.min(maxWidth / img.width, 1);

        this.canvas.width = img.width * scale;
        this.canvas.height = img.height * scale;

        this.ctx.drawImage(img, 0, 0, this.canvas.width, this.canvas.height);
    }

    startProcessing() {
        if (this.isProcessing) return;
        this.isProcessing = true;
        this.processFrame();
    }

    stopProcessing() {
        this.isProcessing = false;
        if (this.animationFrameId) {
            cancelAnimationFrame(this.animationFrameId);
            this.animationFrameId = null;
        }
    }

    processFrame() {
        if (!this.isProcessing) return;

        if (this.videoElement && this.videoElement.readyState >= 2) {
            const maxWidth = this.canvas.parentElement.clientWidth;
            const scale = Math.min(maxWidth / this.videoElement.videoWidth, 1);

            const width = Math.floor(this.videoElement.videoWidth * scale);
            const height = Math.floor(this.videoElement.videoHeight * scale);
            // 같은 크기 할당도 캔버스를 초기화하므로 해상도가 바뀔 때만 수행한다.
            if (this.canvas.width !== width) this.canvas.width = width;
            if (this.canvas.height !== height) this.canvas.height = height;

            this.ctx.drawImage(this.videoElement, 0, 0, this.canvas.width, this.canvas.height);
        }

        this.animationFrameId = requestAnimationFrame(() => this.processFrame());
    }

    getCurrentFrameData() {
        return this.ctx.getImageData(0, 0, this.canvas.width, this.canvas.height);
    }

    getCanvasSize() {
        return { width: this.canvas.width, height: this.canvas.height };
    }

    getOriginalVideoSize() {
        if (this.videoElement) {
            return { width: this.videoElement.videoWidth, height: this.videoElement.videoHeight };
        }
        return null;
    }

    getScale() {
        const original = this.getOriginalVideoSize();
        if (!original) return 1;
        return this.canvas.width / original.width;
    }

    stop() {
        if (this.serverCamera) {
            const source = this.serverCamera;
            this.serverCamera = null;
            this.cameraCommand('stop', {session_id: source.session_id}).catch(error => {
                document.getElementById('canvas-status').textContent = error.message;
            });
        }
        this.stopProcessing();

        if (this.videoElement && this.videoElement.srcObject) {
            const tracks = this.videoElement.srcObject.getTracks();
            tracks.forEach(track => track.stop());
            this.videoElement.srcObject = null;
        }
    }
}
