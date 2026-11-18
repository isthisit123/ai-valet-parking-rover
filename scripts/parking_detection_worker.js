// 기존 모델과 전/후처리를 작업 스레드에서 재사용해 카메라·ArUco UI를 막지 않는다.
importScripts('https://cdn.jsdelivr.net/npm/onnxruntime-web@1.22.0/dist/ort.min.js');
let detector, busy = false;
const MIN_VEHICLE_CONFIDENCE = 0.70;

async function initialize(modelPath) {
    if (detector) return;
    const {YOLODetector} = await import('/rover-original-yolo.js');
    const candidate = new YOLODetector(modelPath);
    // 다른 추론 요청이나 WASM 스레드가 계속 쌓이지 않도록 한 작업씩 수행한다.
    ort.env.wasm.numThreads = 1;
    await candidate.init();
    if (!candidate.isInitialized || !candidate.session) throw new Error('차량 모델 초기화 실패');
    detector = candidate;
}

self.onmessage = async ({data}) => {
    if (busy) {
        self.postMessage({id: data.id, error: '이전 차량 추론이 진행 중입니다.'});
        return;
    }
    busy = true;
    let tensor, outputs;
    try {
        await initialize(data.modelPath);
        if (data.type === 'init') {
            self.postMessage({id: data.id, ready: true});
            return;
        }
        if (data.type !== 'detect') throw new Error('알 수 없는 차량 추론 요청');
        const {imageData, width, height} = data;
        const prepared = detector.preprocess(imageData, width, height);
        tensor = prepared.tensor;
        outputs = await detector.session.run({images: tensor});
        const output = outputs[detector.outputNames[0]];
        if (!output) throw new Error('차량 모델 출력이 없습니다.');
        const detections = detector.postprocess(output, width, height, prepared.scale, prepared.offsetX, prepared.offsetY)
            .filter(item => Number.isFinite(item.confidence) && item.confidence >= MIN_VEHICLE_CONFIDENCE);
        self.postMessage({id: data.id, detections, inference_valid: true});
    } catch (error) {
        self.postMessage({id: data.id, error: error.message});
    } finally {
        tensor?.dispose?.();
        for (const value of Object.values(outputs || {})) value.dispose?.();
        busy = false;
    }
};
