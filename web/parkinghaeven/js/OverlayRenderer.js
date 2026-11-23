// 캔버스 위에 주차 구역 및 탐지 결과 오버레이 렌더링
export class OverlayRenderer {
    constructor(overlayCanvas, mainCanvas) {
        this.overlayCanvas = overlayCanvas;
        this.ctx = overlayCanvas.getContext('2d');
        this.mainCanvas = mainCanvas;
        this.regions = [];
        this.detectionResults = [];
        this.mode = 'setup';
        this.editingMode = 'add'; // 'add' or 'edit'
        this.currentRegion = null;
        this.selectedRegion = null;
        this.selectedPointIndex = -1;
        this.isDragging = false;
    }

    setEditingMode(mode) {
        this.editingMode = mode;
        this.render();
    }

    syncCanvas() {
        this.overlayCanvas.width = this.mainCanvas.width;
        this.overlayCanvas.height = this.mainCanvas.height;
    }

    setRegions(regions) {
        this.regions = regions;
        this.render();
    }

    setDetectionResults(results) {
        this.detectionResults = results;
        this.render();
    }

    setMode(mode) {
        this.mode = mode;
        this.render();
    }

    getMousePos(evt) {
        const rect = this.overlayCanvas.getBoundingClientRect();
        const scaleX = this.overlayCanvas.width / rect.width;
        const scaleY = this.overlayCanvas.height / rect.height;
        return {
            x: (evt.clientX - rect.left) * scaleX,
            y: (evt.clientY - rect.top) * scaleY
        };
    }

    addPoint(point) {
        if (!this.currentRegion) {
            this.currentRegion = { id: Date.now(), points: [point], occupied: false };
        } else {
            this.currentRegion.points.push(point);
        }
        this.render();
        return this.currentRegion;
    }

    finishRegion() {
        if (this.currentRegion && this.currentRegion.points.length >= 3) {
            this.regions.push(this.currentRegion);
            this.currentRegion = null;
            this.render();
            return true;
        }
        this.currentRegion = null;
        this.render();
        return false;
    }

    cancelRegion() {
        this.currentRegion = null;
        this.render();
    }

    findPointAt(pos) {
        const threshold = 10;
        for (let i = 0; i < this.regions.length; i++) {
            for (let j = 0; j < this.regions[i].points.length; j++) {
                const point = this.regions[i].points[j];
                const dist = Math.sqrt(Math.pow(pos.x - point.x, 2) + Math.pow(pos.y - point.y, 2));
                if (dist <= threshold) {
                    return { regionIndex: i, pointIndex: j };
                }
            }
        }
        return null;
    }

    movePoint(regionIndex, pointIndex, newPos) {
        if (this.regions[regionIndex] && this.regions[regionIndex].points[pointIndex]) {
            this.regions[regionIndex].points[pointIndex] = newPos;
            this.render();
        }
    }

    deleteRegion(index) {
        this.regions.splice(index, 1);
        this.render();
    }

    clearRegions() {
        this.regions = [];
        this.currentRegion = null;
        this.selectedRegion = null;
        this.selectedPointIndex = -1;
        this.render();
    }

    isPointInPolygon(point, polygon) {
        let inside = false;
        for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
            const xi = polygon[i].x, yi = polygon[i].y;
            const xj = polygon[j].x, yj = polygon[j].y;

            const intersect = ((yi > point.y) !== (yj > point.y)) &&
                (point.x < (xj - xi) * (point.y - yi) / (yj - yi) + xi);
            if (intersect) inside = !inside;
        }
        return inside;
    }

    render() {
        this.ctx.clearRect(0, 0, this.overlayCanvas.width, this.overlayCanvas.height);

        // Draw completed regions
        this.regions.forEach((region, index) => {
            this.drawRegion(region, index === this.selectedRegion?.index);
        });

        // Draw current region being created
        if (this.currentRegion && this.currentRegion.points.length > 0) {
            this.drawCurrentRegion();
        }

        // Draw detection results in monitor mode
        if (this.mode === 'monitor') {
            this.drawDetectionResults();
        }

        // Debug: always show detection count in monitor mode
        if (this.mode === 'monitor' && this.detectionResults.length > 0) {
            this.ctx.fillStyle = '#fff';
            this.ctx.font = 'bold 14px Arial';
            this.ctx.fillText(`탐지: ${this.detectionResults.length}대`, 10, 20);
        }
    }

    drawRegion(region, isSelected = false) {
        const points = region.points;
        if (points.length < 2) return;

        this.ctx.beginPath();
        this.ctx.moveTo(points[0].x, points[0].y);
        for (let i = 1; i < points.length; i++) {
            this.ctx.lineTo(points[i].x, points[i].y);
        }
        this.ctx.closePath();

        // Fill based on occupancy status
        if (this.mode === 'monitor') {
            this.ctx.fillStyle = region.occupied ? 'rgba(239, 68, 68, 0.5)' : 'rgba(34, 197, 94, 0.5)';
            this.ctx.fill();
        } else {
            this.ctx.fillStyle = 'rgba(59, 130, 246, 0.2)';
            this.ctx.fill();
        }

        // Stroke
        const strokeColor = this.mode === 'monitor'
            ? (region.occupied ? '#ef4444' : '#22c55e')
            : (isSelected ? '#fbbf24' : '#3b82f6');
        this.ctx.strokeStyle = strokeColor;
        this.ctx.lineWidth = isSelected ? 3 : 2;
        this.ctx.stroke();

        // Draw points in setup mode
        if (this.mode === 'setup') {
            points.forEach((point, i) => {
                this.ctx.beginPath();

                // In edit mode, make points larger and more visible
                const pointRadius = this.editingMode === 'edit' ? 8 : 6;
                this.ctx.arc(point.x, point.y, pointRadius, 0, Math.PI * 2);

                // In edit mode, use different color to indicate points are clickable
                if (this.editingMode === 'edit') {
                    this.ctx.fillStyle = isSelected ? '#fbbf24' : '#60a5fa';
                    this.ctx.fill();
                    this.ctx.strokeStyle = '#fff';
                    this.ctx.lineWidth = 3;
                    this.ctx.stroke();

                    // Add hover effect ring
                    this.ctx.beginPath();
                    this.ctx.arc(point.x, point.y, pointRadius + 3, 0, Math.PI * 2);
                    this.ctx.strokeStyle = 'rgba(96, 165, 250, 0.5)';
                    this.ctx.lineWidth = 2;
                    this.ctx.stroke();
                } else {
                    this.ctx.fillStyle = isSelected ? '#fbbf24' : '#3b82f6';
                    this.ctx.fill();
                    this.ctx.strokeStyle = '#fff';
                    this.ctx.lineWidth = 2;
                    this.ctx.stroke();
                }

                // Draw point number
                this.ctx.fillStyle = '#fff';
                this.ctx.font = '10px Arial';
                this.ctx.fillText(i + 1, point.x + 8, point.y - 8);
            });
        }

        // Draw region ID
        const center = this.getPolygonCenter(points);
        this.ctx.fillStyle = this.mode === 'monitor' ? '#fff' : '#3b82f6';
        this.ctx.font = 'bold 12px Arial';
        this.ctx.fillText(`#${region.id}`, center.x - 10, center.y);
    }

    drawCurrentRegion() {
        const points = this.currentRegion.points;
        this.ctx.beginPath();
        this.ctx.moveTo(points[0].x, points[0].y);
        for (let i = 1; i < points.length; i++) {
            this.ctx.lineTo(points[i].x, points[i].y);
        }
        this.ctx.strokeStyle = '#22c55e';
        this.ctx.lineWidth = 2;
        this.ctx.setLineDash([5, 5]);
        this.ctx.stroke();
        this.ctx.setLineDash([]);

        points.forEach((point, i) => {
            this.ctx.beginPath();
            this.ctx.arc(point.x, point.y, 5, 0, Math.PI * 2);
            this.ctx.fillStyle = '#22c55e';
            this.ctx.fill();
        });
    }

    drawDetectionResults() {
        // Only draw detections that are related to parking regions
        this.detectionResults.forEach(detection => {
            // Check if this detection is inside any parking region
            const isInRegion = this.regions.some(region =>
                this.isPointInPolygon(detection.center, region.points)
            );

            // Skip drawing if not in any region
            if (!isInRegion) return;

            const { x, y, width, height } = detection.bbox;
            const { label, confidence } = detection;
            const color = label === 'car' ? '#3b82f6' : '#8b5cf6';

            this.ctx.strokeStyle = color;
            this.ctx.lineWidth = 2;
            this.ctx.strokeRect(x, y, width, height);

            this.ctx.fillStyle = color;
            this.ctx.fillRect(x, y - 20, width, 20);

            this.ctx.fillStyle = '#fff';
            this.ctx.font = '11px Arial';
            this.ctx.fillText(`${label} ${(confidence * 100).toFixed(0)}%`, x + 4, y - 6);
        });
    }

    getPolygonCenter(points) {
        let cx = 0, cy = 0;
        points.forEach(p => { cx += p.x; cy += p.y; });
        return { x: cx / points.length, y: cy / points.length };
    }

    getRegions() {
        return [...this.regions];
    }
}
