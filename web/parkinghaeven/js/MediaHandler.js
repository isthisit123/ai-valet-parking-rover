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
    }

    async initWebcam(deviceId = null) {
        try {
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

            this.canvas.width = this.videoElement.videoWidth * scale;
            this.canvas.height = this.videoElement.videoHeight * scale;

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
        this.stopProcessing();

        if (this.videoElement && this.videoElement.srcObject) {
            const tracks = this.videoElement.srcObject.getTracks();
            tracks.forEach(track => track.stop());
            this.videoElement.srcObject = null;
        }
    }
}
