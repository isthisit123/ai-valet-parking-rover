// 원본 크기와 JPEG 품질을 유지하면서 압축을 UI 스레드 밖에서 수행한다.
let canvas, context, busy = false;

self.onmessage = async ({data}) => {
    const {id, bitmap} = data;
    if (busy) {
        bitmap?.close();
        self.postMessage({id, error: '이전 ArUco 영상 인코딩이 진행 중입니다.'});
        return;
    }
    busy = true;
    const started = performance.now();
    try {
        const queueMs = Math.max(0, performance.timeOrigin + started - data.sentAt);
        if (!canvas) {
            canvas = new OffscreenCanvas(bitmap.width, bitmap.height);
            context = canvas.getContext('2d');
        }
        if (canvas.width !== bitmap.width) canvas.width = bitmap.width;
        if (canvas.height !== bitmap.height) canvas.height = bitmap.height;
        context.drawImage(bitmap, 0, 0);
        const drawn = performance.now();
        const blob = await canvas.convertToBlob({type: 'image/jpeg', quality: 0.92});
        self.postMessage({id, blob, timing: {worker_queue_ms: queueMs,
            worker_draw_ms: drawn - started, worker_encode_ms: performance.now() - drawn}});
    } catch (error) {
        self.postMessage({id, error: error.message});
    } finally {
        bitmap?.close();
        busy = false;
    }
};
