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
        const debug = document.createElement('a');
        debug.href = '/api/aruco-frame?miss=1'; debug.target = '_blank'; debug.rel = 'noopener';
        debug.textContent = '마커 누락 원본 영상 확인';
        debug.style.cssText = 'display:block;margin:8px 12px;color:#7dd3fc;text-decoration:underline;';
        document.body.appendChild(debug);
        this.periodMs = 100;
        this.frameId = 0;
        this.lastReceivedAt = null;
        this.lastVideoTime = null;
        this.lastResult = null;
        this.encoderWorker = null;
        this.encoderChecked = false;
        this.encoderPending = null;
        this.encoderId = 0;
        this.diagnosticHistory = [];
        this.longTasks = [];
        this.summaryAt = -Infinity;
        this.summaryText = '';
        this.scheduledFor = null;
        // 브라우저 콜백 실행 대기와 JPEG 압축 시간을 구별하기 위한 보조 진단이다.
        if (typeof PerformanceObserver !== 'undefined') {
            try {
                this.longTaskObserver = new PerformanceObserver(list => {
                    const now = performance.now();
                    this.longTasks = this.longTasks.filter(item => now - item.at <= 10000);
                    for (const entry of list.getEntries()) this.longTasks.push({at: now, ms: entry.duration});
                });
                this.longTaskObserver.observe({type: 'longtask'});
            } catch { /* 미지원 환경에서도 프레임 시각 검사는 유지한다. */ }
        }
        this.ready = this.configure();
        this.loop();
    }

    async configure() {
        const response = await fetch('/api/aruco-config');
        if (!response.ok) throw new Error('ArUco 서버 설정을 읽지 못했습니다. 서버를 재시작하세요.');
        const config = await response.json();
        this.periodMs = config.period_ms;
    }

    failEncoder(message) {
        this.encoderWorker?.terminate();
        this.encoderWorker = null;
        const pending = this.encoderPending;
        this.encoderPending = null;
        if (pending) {clearTimeout(pending.timer); pending.reject(new Error(message));}
    }

    async encodeFrame(video) {
        if (!this.encoderChecked) {
            this.encoderChecked = true;
            if (typeof Worker !== 'undefined' && typeof OffscreenCanvas !== 'undefined'
                    && typeof createImageBitmap !== 'undefined') {
                try {
                    this.encoderWorker = new Worker('/aruco-frame-worker.js');
                    this.encoderWorker.onmessage = ({data}) => {
                        const pending = this.encoderPending;
                        if (!pending || pending.id !== data.id) return;
                        this.encoderPending = null;
                        clearTimeout(pending.timer);
                        if (data.error) {this.failEncoder(data.error); pending.reject(new Error(data.error));}
                        else pending.resolve({blob: data.blob, mode: 'worker', ...data.timing});
                    };
                    this.encoderWorker.onerror = event => this.failEncoder(event.message || 'ArUco 인코딩 Worker 오류');
                } catch {this.encoderWorker = null;}
            }
        }
        if (!this.encoderWorker) {
            // Worker 미지원 시 기존 경로를 사용하되 프레임을 쌓거나 만료 시각을 늘리지 않는다.
            const started = performance.now();
            if (this.canvas.width !== video.videoWidth) this.canvas.width = video.videoWidth;
            if (this.canvas.height !== video.videoHeight) this.canvas.height = video.videoHeight;
            this.ctx.drawImage(video, 0, 0);
            const drawn = performance.now();
            const blob = await new Promise(resolve => this.canvas.toBlob(resolve, 'image/jpeg', 0.92));
            return {blob, mode: 'main', snapshot_ms: drawn - started};
        }
        const worker = this.encoderWorker;
        const started = performance.now();
        const bitmap = await createImageBitmap(video);
        const snapshotMs = performance.now() - started;
        if (worker !== this.encoderWorker) {bitmap.close(); throw new Error('ArUco 인코딩 Worker가 종료됐습니다.');}
        if (this.encoderPending) {bitmap.close(); throw new Error('이전 ArUco 영상 인코딩이 진행 중입니다.');}
        const result = await new Promise((resolve, reject) => {
            const id = ++this.encoderId;
            const timer = setTimeout(() => this.failEncoder('ArUco 인코딩 응답 시간 초과'), 1000);
            this.encoderPending = {id, resolve, reject, timer};
            try {worker.postMessage({id, bitmap, sentAt: performance.timeOrigin + performance.now()}, [bitmap]);}
            catch (error) {
                bitmap.close();
                this.failEncoder(error.message);
            }
        });
        return {...result, snapshot_ms: snapshotMs};
    }

    summarize(sample) {
        const now = performance.now();
        this.diagnosticHistory = (this.diagnosticHistory || []).filter(item => now - item.at <= 10000);
        this.diagnosticHistory.push({at: now, ...sample});
        this.diagnosticHistory = this.diagnosticHistory.slice(-128);
        if (this.summaryText && now - this.summaryAt < 1000) return this.summaryText;
        this.summaryAt = now;
        const stats = key => {
            const values = this.diagnosticHistory.map(item => item[key]).filter(Number.isFinite);
            if (!values.length) return '미측정';
            return `${(values.reduce((a, b) => a + b, 0) / values.length).toFixed(1)}/${Math.max(...values).toFixed(1)}ms`;
        };
        const count = key => this.diagnosticHistory.filter(item => item[key]).length;
        const tasks = (this.longTasks || []).filter(item => now - item.at <= 10000);
        const storage = [...this.diagnosticHistory].reverse().find(item => item.persistence) || {};
        const writeTime = value => Number.isFinite(value) ? `${value.toFixed(1)}ms` : '미기록';
        this.summaryText = [
            '최근 10초 요약 · 평균/최대 · 1초마다 갱신',
            `응답 간격 ${stats('interval_ms')} / 예약 실행 지연 ${stats('schedule_ms')}`,
            `JPEG 생성 전체 ${stats('encoding_ms')} / Worker 압축 ${stats('worker_encode_ms')} / 영상 복사 ${stats('snapshot_ms')}`,
            `Worker 시작 대기 ${stats('worker_queue_ms')} / Worker 그리기 ${stats('worker_draw_ms')}`,
            `전송·응답 ${stats('request_ms')} / 서버 전체 ${stats('server_ms')} / 영상 디코딩 ${stats('decode_ms')}`,
            `cm 수학 변환 ${stats('cm_math_ms')} / 변환 전 지도 검증 ${stats('map_validation_ms')}`,
            `추적·도착 파일 저장·등록(응답 경로) ${stats('save_ms')} / 도착 처리 전체 ${stats('arrival_ms')}`,
            storage.persistence ? `별도 기록 최근 소요: 추적 ${writeTime(storage.persistence.last_write_ms)} / 도착 ${writeTime(storage.arrival_persistence?.last_write_ms)} · 평균 아님` : '',
            `유효 응답 ${count('fresh')} / 만료 응답 ${count('stale')} / 인코딩 만료 폐기 ${count('encode_expired')} / 새 영상 대기 ${count('no_frame')}`,
            `주 스레드 긴 작업(50ms 이상) ${tasks.length}회 · 최대 ${Math.max(0, ...tasks.map(item => item.ms)).toFixed(1)}ms`,
        ].filter(Boolean).join('\n');
        return this.summaryText;
    }

    async loop() {
        const started = performance.now();
        const sample = {schedule_ms: Math.max(0, started - (this.scheduledFor ?? started))};
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
                sample.no_frame = true;
                this.panel.textContent = '로버 ArUco: 새 영상 프레임 대기 중. 이전 위치를 갱신하지 않습니다.\n'
                    + this.summarize(sample);
                window.dispatchEvent(new CustomEvent('aruco-result', {detail: null}));
                return;
            }
            this.lastVideoTime = video.currentTime;
            const source = {kind: this.media.currentSourceType, width: video.videoWidth, height: video.videoHeight,
                session: video.srcObject?.getVideoTracks?.()[0]?.id || ''};
            const sameSource = () => this.media.videoElement === video && this.media.currentSourceType === source.kind
                && video.videoWidth === source.width && video.videoHeight === source.height
                && (video.srcObject?.getVideoTracks?.()[0]?.id || '') === source.session;
            // 복사·압축 대기 시간도 위치의 나이에 포함한다.
            const capturedAt = new Date().toISOString();
            const captureStarted = performance.now();
            const encoded = await this.encodeFrame(video);
            const {blob} = encoded;
            if (!blob) throw new Error('카메라 프레임 생성 실패');
            if (!sameSource()) throw new Error('프레임 생성 중 카메라가 변경됐습니다.');
            const encodingMs = performance.now() - captureStarted;
            Object.assign(sample, {encoding_ms: encodingMs, snapshot_ms: encoded.snapshot_ms,
                worker_encode_ms: encoded.worker_encode_ms, worker_queue_ms: encoded.worker_queue_ms,
                worker_draw_ms: encoded.worker_draw_ms});
            if (encodingMs > 200) {
                sample.encode_expired = true;
                throw new Error(`영상 인코딩 지연 ${encodingMs.toFixed(0)}ms: 오래된 프레임을 버렸습니다.`);
            }
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
            data.encoding_mode = encoded.mode;
            data.encoding_timing_ms = {snapshot_ms: encoded.snapshot_ms, worker_queue_ms: encoded.worker_queue_ms,
                worker_encode_ms: encoded.worker_encode_ms, worker_draw_ms: encoded.worker_draw_ms};
            data.request_ms = now - requestStarted;
            const interval = this.lastReceivedAt === null ? null : now - this.lastReceivedAt;
            this.lastReceivedAt = now;
            this.lastResult = {...data, receivedAt: Date.now()};
            const server = data.server_timing_ms || {}, arrival = data.arrival_test?.timing_ms || {};
            Object.assign(sample, {request_ms: data.request_ms, interval_ms: interval,
                fresh: data.tracking_status === 'detected', stale: data.tracking_status === 'stale',
                server_ms: server.total_ms, decode_ms: server.decode_ms, arrival_ms: server.arrival_ms,
                cm_math_ms: arrival.cm_math_ms, map_validation_ms: arrival.map_validation_ms,
                save_ms: Number.isFinite(server.tracker_save_ms) && Number.isFinite(arrival.save_ms)
                    ? server.tracker_save_ms + arrival.save_ms : undefined});
            sample.persistence = data.persistence;
            sample.arrival_persistence = data.arrival_test?.persistence;
            const lines = [
                `로버 ArUco ID ${data.rover_marker_id} · ${data.tracking_status}`,
                `원본 영상 ${data.width}×${data.height} px`,
                `목표 ${this.periodMs}ms / 실제 응답 간격 ${interval === null ? '측정 중' : interval.toFixed(0) + 'ms'} / 검출 ${data.processing_ms.toFixed(1)}ms`,
                `JPEG 인코딩 ${encodingMs.toFixed(1)}ms / 전송·응답 ${data.request_ms.toFixed(1)}ms / 서버 검출 ${data.server_detection_status} · 유효성 ${data.server_tracking_status}`,
                `JPEG 생성 방식: ${encoded.mode === 'worker' ? '별도 Worker' : '브라우저 UI 스레드(Worker 미지원·실패)'}`,
            ];
            if (data.persistence?.mode === 'async_latest') {
                const stores = [data.persistence, data.arrival_test?.persistence].filter(Boolean);
                lines.push(`진단 JSON 기록: 별도 스레드 · 대기 ${stores.filter(item => item.pending).length}개 / 기록 중 ${stores.filter(item => item.writing).length}개`);
                for (const warning of [data.persistence_warning, data.arrival_test?.persistence_warning]) {
                    if (warning) lines.push(warning);
                }
            }
            if (data.recent_detection) lines.push(`최근 ${data.recent_detection.frames}프레임 ID 검출률 ${(data.recent_detection.ratio * 100).toFixed(0)}% · 방법 ${data.detection_method}`);
            if (data.fallback_budget_exhausted) lines.push('재검출 시간 예산 소진: 일부 보완 검출을 실행하지 못했습니다. 누락 원본과 서버 처리 시간을 확인하세요.');
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
            lines.push('', this.summarize(sample));
            this.panel.textContent = lines.join('\n');
            window.dispatchEvent(new CustomEvent('aruco-result', {detail: this.lastResult}));
        } catch (error) {
            this.lastResult = null;
            const message = ['TimeoutError', 'AbortError'].includes(error.name)
                ? '전송·응답 시간 초과(1초). 네트워크 또는 브라우저 지연을 확인하세요.' : error.message;
            this.panel.textContent = `로버 ArUco: ${message}\n현재 위치 없음. 서버와 카메라 상태를 확인하세요.\n`
                + this.summarize(sample);
            window.dispatchEvent(new CustomEvent('aruco-result', {detail: null}));
        } finally {
            // 응답 완료 뒤 남은 시간만 기다려 목표 10Hz를 유지하고 요청 중첩·대기를 막는다.
            const delay = Math.max(0, this.periodMs - (performance.now() - started));
            this.scheduledFor = performance.now() + delay;
            setTimeout(() => this.loop(), delay);
        }
    }
}
