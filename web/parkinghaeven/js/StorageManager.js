// LocalStorage를 통한 소스별 주차 구역 좌표 관리
export class StorageManager {
    static STORAGE_KEY = 'PARKING_SYSTEM_CONFIG';

    static getSourceId(file = null) {
        if (file) {
            return `${file.name}_${file.size}`;
        }
        return 'LIVE_WEBCAM';
    }

    static loadRegions(sourceId) {
        try {
            const config = JSON.parse(localStorage.getItem(this.STORAGE_KEY)) || {};
            return config[sourceId] || [];
        } catch (e) {
            console.error('Failed to load regions:', e);
            return [];
        }
    }

    static saveRegions(sourceId, regions) {
        try {
            const config = JSON.parse(localStorage.getItem(this.STORAGE_KEY)) || {};
            config[sourceId] = regions;
            localStorage.setItem(this.STORAGE_KEY, JSON.stringify(config));
            return true;
        } catch (e) {
            console.error('Failed to save regions:', e);
            return false;
        }
    }

    static clearRegions(sourceId) {
        try {
            const config = JSON.parse(localStorage.getItem(this.STORAGE_KEY)) || {};
            delete config[sourceId];
            localStorage.setItem(this.STORAGE_KEY, JSON.stringify(config));
            return true;
        } catch (e) {
            console.error('Failed to clear regions:', e);
            return false;
        }
    }

    static getAllSourceIds() {
        try {
            const config = JSON.parse(localStorage.getItem(this.STORAGE_KEY)) || {};
            return Object.keys(config);
        } catch (e) {
            return [];
        }
    }
}
