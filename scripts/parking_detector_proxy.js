import {YOLODetector as OriginalDetector} from '/rover-original-yolo.js';

// 웹이 사용하는 기존 클래스 인터페이스를 유지하고 추론만 Worker로 보낸다.
export class YOLODetector extends OriginalDetector {
    constructor(modelPath) {
        super(modelPath);
        this.worker = null;
        this.pending = new Map();
        this.requestId = 0;
        this.initializing = null;
        this.detecting = false;
    }

    failWorker(message) {
        this.worker?.terminate();
        this.worker = null;
        this.isInitialized = false;
        for (const task of this.pending.values()) {clearTimeout(task.timer); task.reject(new Error(message));}
        this.pending.clear();
    }

    request(type, payload = {}) {
        if (!this.worker) return Promise.reject(new Error('차량 추론 Worker가 준비되지 않았습니다.'));
        const id = ++this.requestId;
        return new Promise((resolve, reject) => {
            const timer = setTimeout(() => this.failWorker('차량 추론 응답 시간이 초과됐습니다.'), type === 'init' ? 60000 : 10000);
            this.pending.set(id, {resolve, reject, timer});
            try {
                const message = {id, type, modelPath: this.modelPath, ...payload};
                const transfer = payload.imageData ? [payload.imageData.data.buffer] : [];
                this.worker.postMessage(message, transfer);
            } catch (error) {
                clearTimeout(timer); this.pending.delete(id); reject(error);
            }
        });
    }

    async init() {
        if (this.isInitialized) return {success: true};
        if (this.initializing) return this.initializing;
        this.initializing = Promise.resolve().then(async () => {
            try {
                this.worker = new Worker('/parking-detection-worker.js');
                this.worker.onmessage = ({data}) => {
                    const task = this.pending.get(data.id);
                    if (!task) return;
                    this.pending.delete(data.id); clearTimeout(task.timer);
                    if (data.error) task.reject(new Error(data.error)); else task.resolve(data);
                };
                this.worker.onerror = event => this.failWorker(event.message || '차량 추론 Worker 오류');
                await this.request('init');
                this.isInitialized = true;
                return {success: true};
            } catch (error) {
                this.failWorker(error.message);
                return {success: false, error: error.message};
            } finally {this.initializing = null;}
        });
        return this.initializing;
    }

    async detect(imageData, width, height) {
        if (this.detecting) throw new Error('이전 차량 추론이 진행 중입니다.');
        this.detecting = true;
        try {
            if (!this.isInitialized) {
                const result = await this.init();
                if (!result.success) throw new Error(result.error);
            }
            const result = await this.request('detect', {imageData, width, height});
            const detections = result.detections.filter(item => Number.isFinite(item.confidence) && item.confidence >= 0.70);
            Object.defineProperty(detections, 'inference_valid', {value: result.inference_valid === true});
            return detections;
        } finally {this.detecting = false;}
    }
}
