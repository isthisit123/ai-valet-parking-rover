// 기존 모델 추론·점유 판정을 재사용한다. 오류 시 반환되는 []를 빈칸으로 전달하지 않는다.
(() => {
    const MIN_VEHICLE_CONFIDENCE = 0.70;
    const MARKER_MATCH_MS = 150;
    const MARKER_HISTORY_MS = 5000;
    const MAX_ROVER_BOX_AREA_RATIO = 30;
    const MARKER_CENTER_MARGIN = 0.1;
    let markerHistory = [];
    let activeMap = null, attached = null, sending = false;
    function announce(message) {
        window.dispatchEvent(new CustomEvent('rover-occupancy-status', {detail: {message}}));
    }
    function sourceEqual(a, b) {
        return ['sourceId', 'kind', 'session_id', 'width', 'height', 'file', 'stream', 'videoUrl'].every(k => a[k] === b[k]);
    }
    function recordMarker(data) {
        if (!data?.camera_source || !Number.isInteger(data.rover_marker_id)) return;
        const capturedMs = Date.parse(data.client_captured_at), age = Date.now() - capturedMs;
        if (!Number.isFinite(capturedMs) || age < -50 || age > MARKER_HISTORY_MS) return;
        const corners = data.rover?.corners_px, center = data.rover?.center_px;
        const valid = data.tracking_status === 'detected' && age <= 200 && data.rover?.id === data.rover_marker_id
            && Array.isArray(corners) && corners.length === 4
            && corners.every(p => Array.isArray(p) && p.length === 2 && p.every(Number.isFinite))
            && Array.isArray(center) && center.length === 2 && center.every(Number.isFinite)
            && data.width === data.camera_source.width && data.height === data.camera_source.height;
        markerHistory = markerHistory.filter(item => Date.now() - item.capturedMs <= MARKER_HISTORY_MS
            && item.id === data.rover_marker_id);
        markerHistory.push({capturedMs, source: {...data.camera_source}, id: data.rover_marker_id,
            frameId: data.frame_id, status: data.tracking_status, valid, corners: valid ? corners.map(p => [...p]) : null,
            center: valid ? [...center] : null});
        markerHistory = markerHistory.slice(-64);
    }
    function excludeRover(detections, captured, startedAt, size) {
        const time = Date.parse(startedAt);
        const matches = markerHistory.filter(item => Number.isFinite(time)
            && Date.now() - item.capturedMs <= MARKER_HISTORY_MS
            && ['kind', 'session_id', 'width', 'height'].every(key => item.source[key] === captured[key])
            && Math.abs(item.capturedMs - time) <= MARKER_MATCH_MS);
        matches.sort((a, b) => Math.abs(a.capturedMs - time) - Math.abs(b.capturedMs - time));
        let pose = matches[0], support = null;
        if (pose?.status === 'lost') {
            const before = matches.filter(p => p.valid && p.capturedMs < time).sort((a, b) => b.capturedMs - a.capturedMs)[0];
            const after = matches.filter(p => p.valid && p.capturedMs > time).sort((a, b) => a.capturedMs - b.capturedMs)[0];
            if (before && after && before.id === after.id) {pose = before; support = after;}
        }
        if (!pose?.valid || captured.width <= 0 || captured.height <= 0) {
            return {detections, evidence: {status: 'no_matching_marker', excluded_count: 0}};
        }
        const scale = [size[0] / captured.width, size[1] / captured.height];
        const corners = pose.corners.map(p => [p[0] * scale[0], p[1] * scale[1]]);
        const center = pose.center.map((v, i) => v * scale[i]);
        const markerArea = Math.abs(corners.reduce((area, p, i) => {
            const q = corners[(i + 1) % 4]; return area + p[0] * q[1] - q[0] * p[1];
        }, 0)) / 2;
        const supportCorners = support?.corners.map(p => [p[0] * scale[0], p[1] * scale[1]]) || [];
        const supportCenter = support?.center.map((v, i) => v * scale[i]);
        if (supportCenter && Math.hypot(center[0] - supportCenter[0], center[1] - supportCenter[1]) > 0.5 * Math.sqrt(markerArea)) {
            return {detections, evidence: {status: 'uncertain_motion', excluded_count: 0}};
        }
        const markerCorners = [...corners, ...supportCorners];
        const candidates = [];
        for (const [index, detection] of detections.entries()) {
            const b = detection.bbox;
            if (!b || ![b.x, b.y, b.width, b.height].every(Number.isFinite) || b.width <= 0 || b.height <= 0) continue;
            if (markerArea <= 1 || b.width * b.height > markerArea * MAX_ROVER_BOX_AREA_RATIO) continue;
            if (!markerCorners.every(p => p[0] >= b.x && p[0] <= b.x + b.width && p[1] >= b.y && p[1] <= b.y + b.height)) continue;
            const rx = (center[0] - b.x) / b.width, ry = (center[1] - b.y) / b.height;
            if (rx < MARKER_CENTER_MARGIN || rx > 1 - MARKER_CENTER_MARGIN
                || ry < MARKER_CENTER_MARGIN || ry > 1 - MARKER_CENTER_MARGIN) continue;
            if (supportCenter) {
                const sx = (supportCenter[0] - b.x) / b.width, sy = (supportCenter[1] - b.y) / b.height;
                if (sx < MARKER_CENTER_MARGIN || sx > 1 - MARKER_CENTER_MARGIN
                    || sy < MARKER_CENTER_MARGIN || sy > 1 - MARKER_CENTER_MARGIN) continue;
            }
            candidates.push(index);
        }
        if (candidates.length !== 1) return {detections,
            evidence: {status: candidates.length ? 'ambiguous_vehicle' : 'no_matching_vehicle', excluded_count: 0}};
        const ignored = candidates[0];
        return {detections: detections.filter((_, i) => i !== ignored), evidence: {
            status: 'matched', excluded_count: 1, rover_marker_id: pose.id, marker_frame_id: pose.frameId,
            association_method: support ? 'bracketed_marker_observations' : 'direct_marker_observation',
            support_marker_frame_id: support?.frameId || null,
            marker_captured_at: new Date(pose.capturedMs).toISOString(), frame_delta_ms: Math.abs(pose.capturedMs - time),
            marker_center_px: center, marker_corners_px: corners, image_size_px: [...size],
            bbox: {...detections[ignored].bbox}, confidence: detections[ignored].confidence}};
    }
    async function publish(app, context, captured, regions, valid, startedAt, size, geometry, exclusion) {
        if (!context || context !== activeMap || sending) return;
        const current = window.RoverParkingRegions.snapshot(app);
        const camera = context.camera_source;
        if (!sourceEqual(captured, current) || captured.kind !== 'webcam' || !camera
            || camera.session_id !== captured.session_id || camera.width !== captured.width
            || camera.height !== captured.height || camera.kind !== captured.kind) return;
        valid = valid && app.currentMode === 'monitor' && app.isDetecting
            && app.mainCanvas.width === size[0] && app.mainCanvas.height === size[1]
            && JSON.stringify(app.overlayRenderer.getRegions().map(r => [r.parking_space_id, r.points])) === geometry;
        const seen = new Set();
        const spaces = regions.map(region => {
            const key = region.parking_space_id;
            if (!/^space_[1-6]$/.test(key) || seen.has(key)) throw new Error('P1~P6 구역을 다시 자동 지정하세요.');
            seen.add(key);
            const sameMap = ['map_id', 'map_image_sha256', 'id_assignment_revision', 'capture_id'].every(k => region[k] === context[k]);
            const sameSize = region.image_size_px?.[0] === size[0] && region.image_size_px?.[1] === size[1];
            return {parking_space_id: key,
                status: valid && sameMap && sameSize && typeof region.occupied === 'boolean'
                    ? (region.occupied ? 'occupied' : 'free') : 'unknown',
                // 기존 boolean 판정의 소비 기준값이다. 모델 정확도 측정값이 아니다.
                confidence: 0.5};
        });
        if (seen.size !== 6) throw new Error('P1~P6 구역 여섯 개가 필요합니다.');
        spaces.sort((a, b) => a.parking_space_id.localeCompare(b.parking_space_id));
        // 같은 상태여도 새 추론 결과를 생략하지 않는다. 과거 관측의 시각을 갱신하지 않는다.
        sending = true;
        try {
            const data = {schema_version: '0.1-draft', capture_id: context.capture_id,
                map_id: context.map_id, map_image_sha256: context.map_image_sha256,
                id_assignment_revision: context.id_assignment_revision, camera_source: camera,
                observed_at: startedAt, source: 'parking_heaven_vehicle_center', spaces,
                occupancy_scope: 'other_vehicles_excluding_controlled_rover', rover_exclusion: exclusion};
            const response = await fetch('/api/parking-occupancy', {method: 'POST',
                headers: {'Content-Type': 'application/json'}, body: JSON.stringify(data), signal: AbortSignal.timeout(2000)});
            const result = await response.json();
            if (!response.ok) throw new Error(result.error || '점유 전송 실패');
            if (context !== activeMap) return;
            const free = Object.values(result.occupancy.states).filter(value => value === 'free').length;
            announce(`점유 연결 정상 · 확인된 빈칸 ${free}개 · 새 관측마다 갱신`);
        } finally {
            sending = false;
        }
    }
    function attach(app = window.parkingApp) {
        if (!app?.detector || attached === app.detector) return;
        const detector = app.detector;
        const postprocess = detector.postprocess;
        const detect = detector.detect;
        if (typeof postprocess !== 'function' || typeof detect !== 'function') return;
        attached = detector;
        if (typeof app.detectAndRender === 'function') {
            const detectAndRender = app.detectAndRender;
            let rendering = false;
            app.detectAndRender = async function (...args) {
                if (rendering) return;
                rendering = true;
                try {return await detectAndRender.apply(this, args);}
                finally {rendering = false;}
            };
        }
        const validated = Symbol('successful_inference');
        detector.postprocess = function (...args) {
            const detections = postprocess.apply(this, args)
                .filter(item => Number.isFinite(item.confidence) && item.confidence >= MIN_VEHICLE_CONFIDENCE);
            Object.defineProperty(detections, validated, {value: true});
            return detections;
        };
        detector.detect = async function (...args) {
            const context = activeMap;
            const captured = window.RoverParkingRegions.snapshot(app);
            const regions = JSON.parse(JSON.stringify(app.overlayRenderer.getRegions()));
            const geometry = JSON.stringify(regions.map(r => [r.parking_space_id, r.points]));
            const startedAt = new Date().toISOString();
            const size = [app.mainCanvas.width, app.mainCanvas.height];
            let detections, valid = false;
            try {
                detections = await detect.apply(this, args);
                valid = Boolean(detections?.[validated] || detections?.inference_valid === true);
            } catch (error) {
                announce(`추론 실패 · 점유 미확인: ${error.message}`);
                detections = [];
            }
            const association = valid ? excludeRover(detections, captured, startedAt, size)
                : {detections, evidence: {status: 'inference_unavailable', excluded_count: 0}};
            if (association.detections !== detections) {
                Object.defineProperty(association.detections, 'inference_valid', {value: valid});
            }
            detections = association.detections;
            const observed = valid ? this.checkParkingOccupancy(regions, detections) : regions;
            // 기존 웹 화면 갱신은 HTTP 요청을 기다리지 않는다.
            publish(app, context, captured, observed, valid, startedAt, size, geometry, association.evidence)
                .catch(error => announce(`점유 연결 실패: ${error.message}`));
            return detections;
        };
    }
    window.addEventListener('load', () => attach());
    window.addEventListener('aruco-result', event => recordMarker(event.detail));
    window.addEventListener('rover-map-start', () => {activeMap = null; markerHistory = [];});
    window.addEventListener('rover-map-result', event => {
        activeMap = event.detail?.published_as_latest ? event.detail : null;
        markerHistory = [];
        attach();
    });
    window.RoverParkingOccupancy = {attach};
})();
