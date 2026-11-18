// 원본 웹의 points·localStorage 계약을 유지하며 현재 영상에만 자동 구역을 적용한다.
(() => {
    const storageKey = 'PARKING_SYSTEM_CONFIG';
    function snapshot(app = window.parkingApp) {
        const media = app?.mediaHandler;
        const video = media?.videoElement;
        const kind = media?.currentSourceType || '';
        const track = kind === 'webcam' ? video?.srcObject?.getVideoTracks?.()[0] : null;
        return {sourceId: app?.currentSourceId, kind,
            session_id: track?.id || '', width: kind === 'image' ? 0 : video?.videoWidth || 0,
            height: kind === 'image' ? 0 : video?.videoHeight || 0,
            file: media?.currentFile, stream: video?.srcObject, videoUrl: video?.src};
    }
    function sameSource(before, after) {
        return before.sourceId && ['webcam', 'image', 'video'].includes(before.kind)
            && ['sourceId', 'kind', 'session_id', 'width', 'height', 'file', 'stream', 'videoUrl']
                .every(key => before[key] === after[key])
            && (before.kind !== 'webcam' || Boolean(before.session_id));
    }
    function validPolygon(points, width, height) {
        if (!Array.isArray(points) || points.length < 3 || points.length > 12) return false;
        if (!points.every(p => p && Number.isFinite(p.x) && Number.isFinite(p.y)
            && p.x >= 0 && p.x < width && p.y >= 0 && p.y < height)) return false;
        const turns = points.map((a, i) => {
            const b = points[(i + 1) % points.length], c = points[(i + 2) % points.length];
            return (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
        });
        return turns.every(t => t > 0) || turns.every(t => t < 0);
    }
    function apply(result, captured, app = window.parkingApp, storage = localStorage) {
        const data = result.parking_regions;
        if (result.status !== 'map_generated' || !result.published_as_latest || data?.status !== 'generated') {
            throw new Error(data?.error || '검증된 새 지도에 자동 구역 정보가 없습니다.');
        }
        if (!sameSource(captured, snapshot(app))) throw new Error('촬영 후 영상 소스가 바뀌었습니다. 다시 맵을 생성하세요.');
        if (data.schema_version !== '1.0' || data.coordinate_frame !== 'capture_image' || data.unit !== 'pixel'
            || data.origin !== 'top_left' || data.axes?.x !== 'right' || data.axes?.y !== 'down') {
            throw new Error('자동 구역의 픽셀 좌표 규약이 다릅니다.');
        }
        for (const key of ['capture_id', 'map_id', 'id_assignment_revision', 'map_image_sha256', 'source_sha256']) {
            if (!data[key] || data[key] !== result[key]) throw new Error('자동 구역과 현재 지도의 식별자가 다릅니다.');
        }
        const camera = data.camera_source;
        if (!camera || camera.kind !== captured.kind || camera.session_id !== captured.session_id
            || camera.width !== captured.width || camera.height !== captured.height) {
            throw new Error('자동 구역의 카메라 정보가 다릅니다.');
        }
        const [width, height] = data.image_size_px || [];
        const canvas = app.mainCanvas;
        if (![width, height, canvas.width, canvas.height].every(v => Number.isInteger(v) && v > 0)
            || width !== result.image_size_px?.[0] || height !== result.image_size_px?.[1]
            || Math.abs(canvas.width * height - canvas.height * width) > 2 * Math.max(width, height)) {
            throw new Error('영상 크기·비율이 달라졌습니다. 다시 맵을 생성하세요.');
        }
        const expected = new Set(Array.from({length: 6}, (_, i) => `space_${i + 1}`));
        if (!Array.isArray(data.regions) || data.regions.length !== 6) throw new Error('주차칸 여섯 개가 필요합니다.');
        const regions = data.regions.map(region => {
            const number = Number(region.parking_space_id?.replace('space_', ''));
            if (!expected.delete(region.parking_space_id) || region.id !== `P${number}`
                || region.type !== ([3, 6].includes(number) ? 'accessible' : 'normal')
                || !validPolygon(region.points, width, height)) throw new Error('주차칸 ID·외곽 꼭짓점이 잘못되었습니다.');
            return {...region, occupied: null, occupancy_status: 'unknown',
                map_id: data.map_id, id_assignment_revision: data.id_assignment_revision,
                map_image_sha256: data.map_image_sha256, capture_id: data.capture_id,
                coordinate_frame: 'parking_heaven_canvas', unit: 'pixel',
                image_size_px: [canvas.width, canvas.height],
                points: region.points.map(p => ({x: p.x * canvas.width / width, y: p.y * canvas.height / height}))};
        });
        const previous = storage.getItem(storageKey);
        const config = JSON.parse(previous || '{}');
        if (!config || Array.isArray(config) || typeof config !== 'object') throw new Error('기존 구역 설정 JSON을 확인하세요.');
        config[captured.sourceId] = regions;
        const renderer = app.overlayRenderer;
        const oldRegions = renderer.getRegions();
        // 저장 실패 시 기존 구역을 유지한다. 다른 이미지/카메라 키도 보존한다.
        storage.setItem(storageKey, JSON.stringify(config));
        try {
            app.setMode('setup');
            renderer.currentRegion = null;
            renderer.selectedRegion = null;
            renderer.selectedPointIndex = -1;
            renderer.isDragging = false;
            renderer.syncCanvas();
            renderer.setRegions(regions);
            app.updateStats();
            const free = document.getElementById('free-spaces');
            if (free) free.textContent = '미확인';
            app.log?.('P1~P6 외곽 자동 적용·저장 완료. 점유 상태는 아직 미확인입니다.');
        } catch (error) {
            if (previous === null) storage.removeItem(storageKey);
            else storage.setItem(storageKey, previous);
            renderer.setRegions(oldRegions);
            throw error;
        }
        return regions.length;
    }
    window.RoverParkingRegions = {snapshot, apply};
})();
