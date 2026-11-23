// YOLOv8 ONNX 추론 엔진
export class YOLODetector {
    constructor(modelPath = '/models/parking_cars.onnx') {
        this.modelPath = modelPath;
        this.session = null;
        this.inputSize = 640;
        this.labels = ['car'];
        this.isInitialized = false;
    }

    async init() {
        if (this.isInitialized) return;

        try {
            ort.env.wasm.wasmPaths = 'https://cdn.jsdelivr.net/npm/onnxruntime-web@1.22.0/dist/';

            this.session = await ort.InferenceSession.create(this.modelPath);

            // Get output names
            this.outputNames = this.session.outputNames;
            console.log('YOLOv8 model loaded successfully');
            console.log('Model outputs:', this.outputNames);

            this.isInitialized = true;
            return { success: true };
        } catch (e) {
            console.error('Failed to load YOLOv8 model:', e);
            return { success: false, error: e.message };
        }
    }

    preprocess(imageData, canvasWidth, canvasHeight) {
        const { data, width, height } = imageData;

        // Calculate scale to maintain aspect ratio
        const scale = Math.min(this.inputSize / width, this.inputSize / height);
        const newWidth = Math.round(width * scale);
        const newHeight = Math.round(height * scale);

        // Create resized image data
        const resized = new Float32Array(3 * this.inputSize * this.inputSize);

        // Center offset
        const offsetX = Math.floor((this.inputSize - newWidth) / 2);
        const offsetY = Math.floor((this.inputSize - newHeight) / 2);

        // Resize and normalize (RGB, 0-1)
        for (let c = 0; c < 3; c++) {
            for (let y = 0; y < this.inputSize; y++) {
                for (let x = 0; x < this.inputSize; x++) {
                    const inputIdx = c * this.inputSize * this.inputSize + y * this.inputSize + x;

                    if (x < offsetX || x >= offsetX + newWidth || y < offsetY || y >= offsetY + newHeight) {
                        resized[inputIdx] = 114 / 255;
                    } else {
                        const srcX = Math.floor((x - offsetX) / scale);
                        const srcY = Math.floor((y - offsetY) / scale);
                        const srcIdx = (srcY * width + srcX) * 4;

                        // RGB normalization
                        const val = data[srcIdx + c] / 255.0;
                        resized[inputIdx] = val;
                    }
                }
            }
        }

        const inputTensor = new ort.Tensor('float32', resized, [1, 3, this.inputSize, this.inputSize]);

        return {
            tensor: inputTensor,
            scale: scale,
            offsetX: offsetX,
            offsetY: offsetY
        };
    }

    async detect(imageData, canvasWidth, canvasHeight) {
        if (!this.isInitialized) {
            await this.init();
        }

        const { tensor, scale, offsetX, offsetY } = this.preprocess(imageData, canvasWidth, canvasHeight);

        try {
            const outputs = await this.session.run({ images: tensor });

            // Get the first output by name
            const outputName = this.outputNames[0];
            const output = outputs[outputName];

            if (!output) {
                console.error('No output found. Available keys:', Object.keys(outputs));
                return [];
            }

            const detections = this.postprocess(output, canvasWidth, canvasHeight, scale, offsetX, offsetY);
            return detections;
        } catch (e) {
            console.error('Inference failed:', e);
            return [];
        }
    }

    postprocess(output, canvasWidth, canvasHeight, scale, offsetX, offsetY) {
        const dims = output.dims;
        const data = output.data;

        // YOLOv8 output format: [1, 84, 8400] or [1, number_of_classes + 4, 8400]
        // where 84 = 4(bbox) + 80(classes), 8400 = number of predictions
        const [batch, features, predictions] = dims;

        const numDetections = predictions;
        const numClasses = features - 4;

        const detections = [];

        for (let i = 0; i < numDetections; i++) {
            let maxScore = 0;
            let maxClass = 0;

            // Find class with max score (skip first 4 values which are bbox)
            for (let c = 0; c < numClasses; c++) {
                const score = data[(4 + c) * numDetections + i];
                if (score > maxScore) {
                    maxScore = score;
                    maxClass = c;
                }
            }

            // Apply confidence threshold (keep vehicle types)
            if (maxScore > 0.3 && (maxClass === 0)) {
                // Custom single class: 0 = car
                const x = data[(0) * numDetections + i];
                const y = data[(1) * numDetections + i];
                const w = data[(2) * numDetections + i];
                const h = data[(3) * numDetections + i];

                // Convert from center format to corner format and scale back
                const x1 = (x - w / 2 - offsetX) / scale;
                const y1 = (y - h / 2 - offsetY) / scale;
                const x2 = (x + w / 2 - offsetX) / scale;
                const y2 = (y + h / 2 - offsetY) / scale;

                // Clip to canvas bounds
                const clippedX1 = Math.max(0, Math.min(canvasWidth, x1));
                const clippedY1 = Math.max(0, Math.min(canvasHeight, y1));
                const clippedX2 = Math.max(0, Math.min(canvasWidth, x2));
                const clippedY2 = Math.max(0, Math.min(canvasHeight, y2));

                if (clippedX2 > clippedX1 && clippedY2 > clippedY1) {
                    const label = 'car';
                    detections.push({
                        bbox: {
                            x: clippedX1,
                            y: clippedY1,
                            width: clippedX2 - clippedX1,
                            height: clippedY2 - clippedY1
                        },
                        label: label,
                        className: this.labels[maxClass] || 'unknown',
                        classIndex: maxClass,
                        confidence: maxScore,
                        center: {
                            x: (clippedX1 + clippedX2) / 2,
                            y: (clippedY1 + clippedY2) / 2
                        }
                    });
                }
            }
        }

        return this.nonMaximumSuppression(detections, 0.45);
    }

    nonMaximumSuppression(detections, threshold) {
        const sorted = [...detections].sort((a, b) => b.confidence - a.confidence);
        const kept = [];
        while (sorted.length) {
            const best = sorted.shift();
            kept.push(best);
            for (let i = sorted.length - 1; i >= 0; i--) {
                const a = best.bbox, b = sorted[i].bbox;
                const w = Math.max(0, Math.min(a.x+a.width, b.x+b.width)-Math.max(a.x,b.x));
                const h = Math.max(0, Math.min(a.y+a.height, b.y+b.height)-Math.max(a.y,b.y));
                const overlap = w*h;
                const union = a.width*a.height + b.width*b.height - overlap;
                if (union > 0 && overlap/union > threshold) sorted.splice(i, 1);
            }
        }
        return kept;
    }

    checkParkingOccupancy(regions, detections) {
        return regions.map(region => {
            const regionPoints = region.points;

            let occupied = false;

            // Check if any vehicle center is inside the parking region
            for (const detection of detections) {
                if (this.isPointInPolygon(detection.center, regionPoints)) {
                    occupied = true;
                    break;
                }
            }

            return {
                ...region,
                occupied: occupied
            };
        });
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

    calculatePolygonBBoxIntersection(polygon, bbox) {
        const { x, y, width, height } = bbox;
        const bboxCenter = { x: x + width / 2, y: y + height / 2 };

        if (!this.isPointInPolygon(bboxCenter, polygon)) {
            return 0;
        }

        let intersection = 0;
        const steps = 20;

        for (let i = 0; i <= steps; i++) {
            for (let j = 0; j <= steps; j++) {
                const px = x + (width * i / steps);
                const py = y + (height * j / steps);
                if (this.isPointInPolygon({ x: px, y: py }, polygon)) {
                    intersection++;
                }
            }
        }

        return intersection / ((steps + 1) * (steps + 1));
    }
}
