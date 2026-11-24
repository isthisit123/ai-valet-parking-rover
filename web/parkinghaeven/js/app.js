import { StorageManager } from './StorageManager.js';
import { MediaHandler } from './MediaHandler.js';
import { OverlayRenderer } from './OverlayRenderer.js';
import { YOLODetector } from './YOLODetector.js';
import { ArUcoBridge } from './ArUcoBridge.js';

class ParkingApp {
    constructor() {
        this.mainCanvas = document.getElementById('main-canvas');
        this.overlayCanvas = document.getElementById('overlay-canvas');

        this.mediaHandler = new MediaHandler(this.mainCanvas);
        this.overlayRenderer = new OverlayRenderer(this.overlayCanvas, this.mainCanvas);
        this.detector = new YOLODetector('/models/parking_cars.onnx');

        this.currentSourceType = 'webcam';
        this.currentSourceId = 'LIVE_WEBCAM';
        this.currentMode = 'setup';
        this.editingMode = 'add'; // 'add' or 'edit'
        this.currentFile = null;
        this.webcamStream = null;

        this.detectInterval = null;
        this.isDetecting = false;
        this.displayGeneration = 0;
        this.lastDetectionCount = 0;
        this.detectionDebugCounter = 0;

        // Initialize UI state
        setTimeout(() => this.setEditingMode('add'), 100);

        this.init();
        this.arucoBridge = new ArUcoBridge(this.mediaHandler);
    }

    init() {
        this.setupEventListeners();
        this.setupCanvasResizing();
        this.enumerateWebcams();
        if (!Number.isInteger(window.roverCameraIndex)) this.initModel();
    }

    async initModel() {
        this.log('YOLOv8 모델 초기화 중...');
        const result = await this.detector.init();
        if (result.success) {
            this.log('YOLOv8 모델 로드 완료');
        } else {
            this.log(`모델 로드 실패: ${result.error}`, 'error');
        }
    }

    setupEventListeners() {
        // Tab buttons
        document.querySelectorAll('.tab-btn').forEach(btn => {
            btn.addEventListener('click', (e) => this.handleTabChange(e));
        });

        // Mode buttons
        document.getElementById('mode-setup').addEventListener('click', () => this.setMode('setup'));
        document.getElementById('mode-monitor').addEventListener('click', () => this.setMode('monitor'));

        // Setup sub-mode buttons
        document.getElementById('submode-add').addEventListener('click', () => this.setEditingMode('add'));
        document.getElementById('submode-edit').addEventListener('click', () => this.setEditingMode('edit'));

        // File input
        document.getElementById('file-input').addEventListener('change', (e) => this.handleFileUpload(e));

        // Webcam controls
        document.getElementById('webcam-start-btn').addEventListener('click', () => this.startWebcam());

        // Region controls
        document.getElementById('add-region-btn').addEventListener('click', () => this.startAddingRegion());
        document.getElementById('clear-regions-btn').addEventListener('click', () => this.clearAllRegions());
        document.getElementById('save-config-btn').addEventListener('click', () => this.saveConfiguration());

        // Import/Export
        document.getElementById('export-btn').addEventListener('click', () => this.exportCoordinates());
        document.getElementById('export-all-btn').addEventListener('click', () => this.exportAllCoordinates());
        document.getElementById('import-btn').addEventListener('click', () => document.getElementById('import-file').click());
        document.getElementById('import-file').addEventListener('change', (e) => this.importCoordinates(e));

        // Canvas events
        this.overlayCanvas.style.pointerEvents = 'auto';
        this.overlayCanvas.addEventListener('click', (e) => this.handleCanvasClick(e));
        this.overlayCanvas.addEventListener('mousemove', (e) => this.handleCanvasMouseMove(e));
        this.overlayCanvas.addEventListener('mouseup', (e) => this.handleCanvasMouseUp(e));

        // Keyboard shortcuts
        document.addEventListener('keydown', (e) => this.handleKeyDown(e));
    }

    setupCanvasResizing() {
        const resizeObserver = new ResizeObserver(() => {
            this.overlayRenderer.syncCanvas();
            this.overlayRenderer.render();
        });
        resizeObserver.observe(this.mainCanvas.parentElement);
    }

    async enumerateWebcams() {
        if (Number.isInteger(window.roverCameraIndex)) {
            const select = document.getElementById('webcam-select');
            select.replaceChildren(new Option('Python 카메라 ' + window.roverCameraIndex, String(window.roverCameraIndex)));
            return;
        }
        try {
            const devices = await navigator.mediaDevices.enumerateDevices();
            const videoDevices = devices.filter(device => device.kind === 'videoinput');
            const select = document.getElementById('webcam-select');
            select.innerHTML = '<option value="">웹캠 선택...</option>';
            videoDevices.forEach(device => {
                const option = document.createElement('option');
                option.value = device.deviceId;
                option.textContent = device.label || `Camera ${select.options.length}`;
                select.appendChild(option);
            });
        } catch (e) {
            this.log('웹캠 열거 실패: ' + e.message, 'error');
        }
    }

    handleTabChange(e) {
        const source = e.target.dataset.source;

        document.querySelectorAll('.tab-btn').forEach(btn => btn.classList.remove('tab-active'));
        e.target.classList.add('tab-active');

        this.currentSourceType = source;
        this.stopDetection();

        const fileUploadArea = document.getElementById('file-upload-area');
        const webcamControls = document.getElementById('webcam-controls');

        if (source === 'webcam') {
            fileUploadArea.classList.add('hidden');
            webcamControls.classList.remove('hidden');
        } else {
            fileUploadArea.classList.remove('hidden');
            webcamControls.classList.add('hidden');
            this.stopWebcam();
            this.loadSampleFiles(source);
        }

        this.updateCanvasStatus(`${source.toUpperCase()} 소스를 선택하세요`);
    }

    async loadSampleFiles(type) {
        const sampleList = document.getElementById('sample-files-list');
        const folder = type === 'image' ? 'samples/images' : 'samples/videos';

        sampleList.innerHTML = '<p class="text-xs text-gray-500">로딩 중...</p>';

        try {
            const response = await fetch(`${folder}/`);
            if (!response.ok) throw new Error('폴더를 찾을 수 없습니다');

            const html = await response.text();
            const parser = new DOMParser();
            const doc = parser.parseFromString(html, 'text/html');
            const links = doc.querySelectorAll('a');
            const files = [];

            links.forEach(link => {
                const href = link.getAttribute('href');
                if (href && href !== '../' && !href.startsWith('?')) {
                    const fileName = href;
                    const ext = fileName.split('.').pop().toLowerCase();
                    const isImage = ['jpg', 'jpeg', 'png', 'gif', 'webp'].includes(ext);
                    const isVideo = ['mp4', 'webm', 'mov', 'avi'].includes(ext);

                    if ((type === 'image' && isImage) || (type === 'video' && isVideo)) {
                        files.push(fileName);
                    }
                }
            });

            if (files.length === 0) {
                sampleList.innerHTML = '<p class="text-xs text-gray-500">샘플 파일이 없습니다</p>';
                return;
            }

            sampleList.innerHTML = files.map(file => `
                <button class="sample-file-btn w-full text-left px-2 py-1 text-xs bg-gray-700 hover:bg-gray-600 rounded truncate"
                        data-file="${file}" data-folder="${folder}">
                    ${file}
                </button>
            `).join('');

            // Add click handlers
            sampleList.querySelectorAll('.sample-file-btn').forEach(btn => {
                btn.addEventListener('click', (e) => this.loadSampleFile(e));
            });

        } catch (e) {
            sampleList.innerHTML = '<p class="text-xs text-gray-500">샘플 파일을 불러올 수 없습니다</p>';
        }
    }

    async loadSampleFile(e) {
        const btn = e.target;
        const fileName = btn.dataset.file;
        const folder = btn.dataset.folder;
        const url = `${folder}/${fileName}`;

        this.log(`샘플 파일 로드 중: ${fileName}`);

        try {
            const isVideo = ['mp4', 'webm', 'mov', 'avi'].includes(fileName.split('.').pop().toLowerCase());

            // Create a mock file object for Source ID generation
            const response = await fetch(url);
            const blob = await response.blob();
            const file = new File([blob], fileName, { type: isVideo ? 'video/mp4' : 'image/jpeg' });

            this.currentFile = file;
            this.currentSourceId = StorageManager.getSourceId(file);
            document.getElementById('source-id').textContent = this.currentSourceId;
            document.getElementById('file-name').textContent = fileName;

            let result;
            if (isVideo) {
                result = await this.mediaHandler.loadVideo(file);
            } else {
                result = await this.mediaHandler.loadImage(file);
            }

            if (result.success) {
                this.overlayRenderer.syncCanvas();
                this.loadRegionsForSource();
                this.updateCanvasStatus('샘플 파일 로드 완료');
                this.log(`로드 완료: ${fileName}`);
            }

        } catch (err) {
            this.log(`파일 로드 실패: ${err.message}`, 'error');
        }
    }

    async handleFileUpload(e) {
        const file = e.target.files[0];
        if (!file) return;

        document.getElementById('file-name').textContent = file.name;
        this.currentFile = file;
        this.currentSourceId = StorageManager.getSourceId(file);
        document.getElementById('source-id').textContent = this.currentSourceId;

        this.log(`파일 로드 중: ${file.name}`);

        let result;
        if (file.type.startsWith('image/')) {
            result = await this.mediaHandler.loadImage(file);
        } else if (file.type.startsWith('video/')) {
            result = await this.mediaHandler.loadVideo(file);
        } else {
            this.log('지원하지 않는 파일 형식입니다', 'error');
            return;
        }

        if (result.success) {
            this.overlayRenderer.syncCanvas();
            this.loadRegionsForSource();
            this.updateCanvasStatus('파일 로드 완료');
            this.log(`로드 완료: ${file.name}`);
        } else {
            this.log(`로드 실패: ${result.error}`, 'error');
        }
    }

    async startWebcam() {
        const select = document.getElementById('webcam-select');
        const deviceId = select.value || null;
        const button = document.getElementById('webcam-start-btn');
        if (button.disabled) return;
        if (this.mediaHandler.serverCamera) {
            this.stopWebcam();
            this.updateCanvasStatus('Python 카메라 중지');
            return;
        }
        button.disabled = true;
        this.log('웹캠 시작 중...');
        const result = await this.mediaHandler.initWebcam(deviceId);
        button.disabled = false;
        if (this.currentSourceType !== 'webcam') {
            this.stopWebcam();
            return;
        }

        if (result.success) {
            if (this.mediaHandler.serverCamera) button.textContent = '카메라 중지';
            this.webcamStream = result.stream;
            this.currentSourceId = 'LIVE_WEBCAM';
            document.getElementById('source-id').textContent = this.currentSourceId;

            this.overlayRenderer.syncCanvas();
            this.loadRegionsForSource();

            this.updateCanvasStatus('웹캠 실행 중');
            this.log('웹캠 시작 완료');

            if (this.currentMode === 'monitor') {
                this.startDetection();
            }
        } else {
            this.log(`웹캠 시작 실패: ${result.error}`, 'error');
        }
    }

    stopWebcam() {
        if (this.mediaHandler.serverCamera) {
            this.overlayRenderer.setDetectionResults([]);
            this.overlayRenderer.setRegions(this.overlayRenderer.getRegions().map(r => ({...r, occupied:null, occupancy_status:'unknown'})));
            this.updateStats();
            this.mainCanvas.getContext('2d').clearRect(0, 0, this.mainCanvas.width, this.mainCanvas.height);
            document.getElementById('webcam-start-btn').textContent = '카메라 시작';
        }
        this.mediaHandler.stop();
        this.webcamStream = null;
        this.stopDetection();
    }

    loadRegionsForSource() {
        let regions = StorageManager.loadRegions(this.currentSourceId);
        if (this.mediaHandler.serverCamera) {
            regions = regions.map(r => ({...r, occupied: null, occupancy_status: 'unknown'}));
        }
        this.overlayRenderer.setRegions(regions);
        this.log(`${regions.length}개 주차 구역 로드됨`);
        this.updateStats();
    }

    setMode(mode) {
        this.currentMode = mode;

        document.getElementById('mode-setup').classList.toggle('bg-yellow-600', mode === 'setup');
        document.getElementById('mode-setup').classList.toggle('bg-gray-700', mode !== 'setup');
        document.getElementById('mode-monitor').classList.toggle('bg-green-600', mode === 'monitor');
        document.getElementById('mode-monitor').classList.toggle('bg-gray-700', mode !== 'monitor');

        const desc = mode === 'setup' ? '주차 구역을 다각형으로 그려서 설정하세요' : 'AI가 주차 상태를 실시간 감지합니다';
        document.getElementById('mode-description').textContent = desc;

        // Show/hide setup submodes
        const setupSubmodes = document.getElementById('setup-submodes');
        if (mode === 'setup') {
            setupSubmodes.classList.remove('hidden');
        } else {
            setupSubmodes.classList.add('hidden');
        }

        this.overlayRenderer.setMode(mode);
        this.overlayCanvas.style.pointerEvents = mode === 'setup' ? 'auto' : 'none';

        if (mode === 'monitor') {
            this.startDetection();
        } else {
            this.stopDetection();
        }
    }

    setEditingMode(mode) {
        this.editingMode = mode;
        this.overlayRenderer.setEditingMode(mode);
        this.overlayRenderer.selectedRegion = null;
        this.overlayRenderer.selectedPointIndex = -1;
        this.overlayRenderer.isDragging = false;

        const addBtn = document.getElementById('submode-add');
        const editBtn = document.getElementById('submode-edit');
        const statusText = document.getElementById('submode-status');

        if (mode === 'add') {
            addBtn.classList.remove('bg-gray-700');
            addBtn.classList.add('bg-green-600');
            editBtn.classList.remove('bg-blue-600');
            editBtn.classList.add('bg-gray-700');
            statusText.textContent = '추가 모드: 새 구역을 그립니다';
            statusText.className = 'text-xs text-green-400';
            this.log('추가 모드로 전환');
        } else {
            addBtn.classList.remove('bg-green-600');
            addBtn.classList.add('bg-gray-700');
            editBtn.classList.remove('bg-gray-700');
            editBtn.classList.add('bg-blue-600');
            statusText.textContent = '수정 모드: 기존 구역의 점을 수정합니다';
            statusText.className = 'text-xs text-blue-400';
            this.log('수정 모드로 전환');
        }

        this.overlayRenderer.render();
    }

    startDetection() {
        if (this.isDetecting) return;

        this.isDetecting = true;
        if (this.mediaHandler.serverCamera) {
            const source = this.mediaHandler.serverCamera;
            const regions = this.overlayRenderer.getRegions();
            const generation = this.displayGeneration;
            this.mediaHandler.cameraCommand('monitor', {enabled: true, session_id: source.session_id,
                capture_id: regions[0]?.capture_id, regions, canvas_size: [this.mainCanvas.width, this.mainCanvas.height]})
                .then(() => this.refreshCameraDetections(source, generation))
                .catch(error => {
                    this.log('감시 시작 실패: ' + error.message, 'error');
                    if (this.mediaHandler.serverCamera === source && generation === this.displayGeneration) this.setMode('setup');
                });
            return;
        }
        if (Number.isInteger(window.roverCameraIndex) && this.currentSourceType === 'webcam') {
            this.isDetecting = false;
            return;
        }
        this.log('감지 시작...');

        this.detectInterval = setInterval(() => {
            this.detectAndRender();
        }, 500);
    }

    stopDetection() {
        if (this.isDetecting && this.mediaHandler.serverCamera) {
            this.mediaHandler.cameraCommand('monitor', {enabled: false, session_id: this.mediaHandler.serverCamera.session_id})
                .catch(error => this.log('감시 중단 실패: ' + error.message, 'error'));
        }
        this.displayGeneration++;
        this.isDetecting = false;
        if (this.detectInterval) {
            clearInterval(this.detectInterval);
            this.detectInterval = null;
        }
    }

    async detectAndRender() {
        const generation = this.displayGeneration;
        const frameData = this.mediaHandler.getCurrentFrameData();
        const { width, height } = this.mediaHandler.getCanvasSize();

        const detections = await this.detector.detect(frameData, width, height);
        if (this.displayGeneration !== generation || !this.isDetecting) return;
        this.overlayRenderer.setDetectionResults(detections);

        // Debug logging
        this.detectionDebugCounter++;

        // Log detection count changes
        if (detections.length !== this.lastDetectionCount) {
            this.log(`차량 탐지: ${detections.length}대 detected`);
            this.lastDetectionCount = detections.length;
        }

        // Detailed debug every 10 frames
        if (this.detectionDebugCounter % 10 === 0) {
            console.log(`[Detection] Frame ${this.detectionDebugCounter}: ${detections.length} objects`);
            detections.forEach((d, i) => {
                console.log(`  [${i+1}] ${d.label} at (${d.center.x.toFixed(0)}, ${d.center.y.toFixed(0)}) conf=${d.confidence.toFixed(2)}`);
            });
        }

        const regionsWithOccupancy = this.detector.checkParkingOccupancy(
            this.overlayRenderer.getRegions(),
            detections
        );

        // Log occupancy changes
        const occupiedCount = regionsWithOccupancy.filter(r => r.occupied).length;
        if (this.detectionDebugCounter % 10 === 0) {
            console.log(`[Occupancy] ${occupiedCount}/${regionsWithOccupancy.length} regions occupied`);
        }

        this.overlayRenderer.setRegions(regionsWithOccupancy);
        this.updateStats();
    }

    async refreshCameraDetections(source, generation = this.displayGeneration) {
        const active = () => this.isDetecting && this.mediaHandler.serverCamera === source && generation === this.displayGeneration;
        if (!active()) return;
        let result;
        try {
            const response = await fetch('/api/camera/detections', {cache:'no-store', signal:AbortSignal.timeout(2000)});
            if (!response.ok) throw new Error('차량 검출 상태를 읽지 못했습니다.');
            result = await response.json();
        } catch (error) {result = {states:{}, boxes:[], error:error.message};}
        if (!active()) return;
        const sx = this.mainCanvas.width / source.width, sy = this.mainCanvas.height / source.height;
        const regions = this.overlayRenderer.getRegions().map(region => {
            const state = (result.capture_id === region.capture_id && result.states[region.parking_space_id]) || 'unknown';
            return {...region, occupied: state === 'unknown' ? null : state === 'occupied', occupancy_status: state};
        });
        const boxes = result.boxes.map(({bbox_xywh_px: [x,y,w,h], confidence}) => ({
            bbox: {x:x*sx, y:y*sy, width:w*sx, height:h*sy}, center: {x:(x+w/2)*sx, y:(y+h/2)*sy}, label:'car', confidence}));
        this.overlayRenderer.setDetectionResults(boxes);
        this.overlayRenderer.setRegions(regions);
        this.updateStats();
        this.updateCanvasStatus(result.error || 'Python 카메라 · 감시 중');
        setTimeout(() => this.refreshCameraDetections(source, generation), 350);
    }

    handleCanvasClick(e) {
        if (this.currentMode !== 'setup') return;

        const pos = this.overlayRenderer.getMousePos(e);

        // Check if clicking on existing point
        const pointData = this.overlayRenderer.findPointAt(pos);
        if (pointData) {
            // Only allow point selection in edit mode
            if (this.editingMode === 'edit') {
                this.overlayRenderer.selectedRegion = { index: pointData.regionIndex };
                this.overlayRenderer.selectedPointIndex = pointData.pointIndex;
                this.overlayRenderer.isDragging = true;
                this.log(`점 선택됨 (구역 ${pointData.regionIndex + 1}, 점 ${pointData.pointIndex + 1})`);
            } else {
                this.log('수정 모드에서만 점을 수정할 수 있습니다', 'error');
            }
            return;
        }

        // Only allow adding points in add mode
        if (this.editingMode === 'add') {
            // Add point to current region
            if (e.detail === 1) { // Single click
                this.overlayRenderer.addPoint(pos);
            }
        }
    }

    handleCanvasMouseMove(e) {
        if (!this.overlayRenderer.isDragging) return;

        const pos = this.overlayRenderer.getMousePos(e);
        const { index: regionIndex } = this.overlayRenderer.selectedRegion;
        const pointIndex = this.overlayRenderer.selectedPointIndex;

        this.overlayRenderer.movePoint(regionIndex, pointIndex, pos);
    }

    handleCanvasMouseUp(e) {
        if (e.detail === 2 && this.currentMode === 'setup') { // Double click
            const finished = this.overlayRenderer.finishRegion();
            if (finished) {
                this.log('주차 구역 추가 완료');
            }
        }

        this.overlayRenderer.isDragging = false;
        this.overlayRenderer.selectedPointIndex = -1;
    }

    handleKeyDown(e) {
        if (this.currentMode !== 'setup') return;

        switch(e.key) {
            case 'Enter':
                const finished = this.overlayRenderer.finishRegion();
                if (finished) {
                    this.log('주차 구역 추가 완료');
                }
                e.preventDefault();
                break;
            case 'Escape':
                this.overlayRenderer.cancelRegion();
                this.log('주차 구역 추가 취소');
                e.preventDefault();
                break;
            case 'Delete':
                if (this.overlayRenderer.selectedRegion) {
                    const index = this.overlayRenderer.selectedRegion.index;
                    this.overlayRenderer.deleteRegion(index);
                    this.overlayRenderer.selectedRegion = null;
                    this.log(`주차 구역 ${index + 1} 삭제됨`);
                }
                e.preventDefault();
                break;
            case 's':
                if (e.ctrlKey || e.metaKey) {
                    e.preventDefault();
                    this.saveConfiguration();
                }
                break;
        }
    }

    startAddingRegion() {
        this.setMode('setup');
        this.setEditingMode('add');
        this.log('추가 모드: 캔버스를 클릭하여 점을 추가하고, 더블클릭으로 완료하세요');
    }

    clearAllRegions() {
        if (confirm('모든 주차 구역을 삭제하시겠습니까?')) {
            this.overlayRenderer.clearRegions();
            StorageManager.clearRegions(this.currentSourceId);
            this.updateStats();
            this.log('모든 주차 구역 삭제됨');
        }
    }

    saveConfiguration() {
        const regions = this.overlayRenderer.getRegions();
        const success = StorageManager.saveRegions(this.currentSourceId, regions);
        if (success) {
            this.log(`설정 저장됨: ${regions.length}개 구역`);
        } else {
            this.log('저장 실패', 'error');
        }
    }

    updateStats() {
        const regions = this.overlayRenderer.getRegions();
        const total = regions.length;
        const occupied = regions.filter(r => r.occupied).length;
        const free = this.mediaHandler.serverCamera ? regions.filter(r => r.occupied === false).length : total - occupied;

        document.getElementById('total-spaces').textContent = total;
        document.getElementById('occupied-spaces').textContent = occupied;
        document.getElementById('free-spaces').textContent = free;
    }

    updateCanvasStatus(message) {
        document.getElementById('canvas-status').textContent = message;
    }

    exportCoordinates() {
        const config = localStorage.getItem('PARKING_SYSTEM_CONFIG');

        if (!config) {
            this.log('내보낼 좌표가 없습니다', 'error');
            alert('저장된 주차 구역 좌표가 없습니다.');
            return;
        }

        const data = JSON.parse(config);

        // If current source has regions, export only that source
        let exportData = data;
        let fileName = 'parking_regions.json';

        if (this.currentSourceId && data[this.currentSourceId]) {
            // Export only current source
            exportData = { [this.currentSourceId]: data[this.currentSourceId] };

            // Generate filename from source (keep extension)
            if (this.currentSourceId === 'LIVE_WEBCAM') {
                fileName = 'webcam_regions.json';
            } else if (this.currentFile) {
                // Keep the full filename including extension
                fileName = `${this.currentFile.name}_regions.json`;
            }
        }

        const blob = new Blob([JSON.stringify(exportData, null, 2)], { type: 'application/json' });
        const url = URL.createObjectURL(blob);

        const a = document.createElement('a');
        a.href = url;
        a.download = fileName;
        a.click();
        URL.revokeObjectURL(url);

        const sourceCount = Object.keys(exportData).length;
        const totalRegions = Object.values(exportData).reduce((sum, regions) => sum + regions.length, 0);
        this.log(`좌표 내보내기: ${fileName} (${totalRegions}개 구역)`);
    }

    exportAllCoordinates() {
        const config = localStorage.getItem('PARKING_SYSTEM_CONFIG');

        if (!config) {
            this.log('내보낼 좌표가 없습니다', 'error');
            alert('저장된 주차 구역 좌표가 없습니다.');
            return;
        }

        const data = JSON.parse(config);
        const blob = new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' });
        const url = URL.createObjectURL(blob);

        const a = document.createElement('a');
        a.href = url;
        a.download = 'all_parking_regions.json';
        a.click();
        URL.revokeObjectURL(url);

        const sourceCount = Object.keys(data).length;
        const totalRegions = Object.values(data).reduce((sum, regions) => sum + regions.length, 0);
        this.log(`전체 좌표 내보내기: ${sourceCount}개 파일, ${totalRegions}개 구역`);
    }

    importCoordinates(e) {
        const file = e.target.files[0];
        if (!file) return;

        const reader = new FileReader();
        reader.onload = (event) => {
            try {
                const data = JSON.parse(event.target.result);

                // Validate structure
                if (typeof data !== 'object' || data === null) {
                    throw new Error('잘못된 파일 형식');
                }

                // Merge with existing data
                const existing = JSON.parse(localStorage.getItem('PARKING_SYSTEM_CONFIG') || '{}');
                const merged = { ...existing, ...data };
                localStorage.setItem('PARKING_SYSTEM_CONFIG', JSON.stringify(merged));

                // Reload current source regions
                this.loadRegionsForSource();

                const sourceCount = Object.keys(data).length;
                const totalRegions = Object.values(data).reduce((sum, regions) => sum + regions.length, 0);
                this.log(`좌표 가져오기: ${sourceCount}개 파일, ${totalRegions}개 구역`);
                alert(`좌표를 성공적으로 가져왔습니다!\n${sourceCount}개 파일, ${totalRegions}개 구역`);
            } catch (err) {
                this.log('가져오기 실패: ' + err.message, 'error');
                alert('파일을 읽는 데 실패했습니다: ' + err.message);
            }
        };
        reader.readAsText(file);

        // Reset input
        e.target.value = '';
    }

    log(message, type = 'info') {
        const log = document.getElementById('detection-log');
        const time = new Date().toLocaleTimeString();
        const colorClass = type === 'error' ? 'text-red-400' : 'text-green-400';
        log.innerHTML = `<p class="${colorClass}">[${time}] ${message}</p>` + log.innerHTML;
    }
}

document.addEventListener('DOMContentLoaded', () => {
    window.parkingApp = new ParkingApp();
});
