# 저장소에 포함한 ParkingHaeven 웹

기존 로컬 `ParkingHaeven/Source`의 웹 실행 파일과 실제 단일 car 모델을 함께 보관합니다. 저장소 루트에서 `python -m scripts.http_map_server --port 8001`로 실행하면 이 폴더를 기본으로 제공합니다. 지도 OCR·cm 보정은 저장소 루트 README의 환경 설치와 기준점 설정을 함께 따르세요.

- 모델: `models/parking_cars.onnx` (12,746,124바이트), SHA-256은 `models/manifest.json`에 기록합니다.
- 입출력: `images` `[1,3,640,640]` → `[1,5,8400]`, 클래스 `car` 하나입니다.
- ONNX Runtime Web 1.22.0과 Tailwind는 기존 CDN을 사용합니다. 브라우저에서 CDN 접근이 필요합니다.
- 기존 Source의 `*.onnx` 제외 규칙은 가져오지 않았습니다. 모델은 일반 Git 바이너리로 clone할 때 함께 내려받습니다.
- 샘플 사진·영상, 학습 `.pt` 파일, 가상환경과 개인 카메라 보정은 포함하지 않습니다. 본인 환경에서 영상·지도와 보정을 생성하세요.
- 서버는 Worker 프록시, ArUco·점유·지도 생성 브리지를 제공하며 원본 `js/YOLODetector.js`의 전후처리를 재사용합니다.
