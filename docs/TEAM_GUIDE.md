# Q-ROVER 팀 통합 가이드

## 통합 기준 지도·점유·경로

| 항목 | 기준 |
| --- | --- |
| 기본 지도 | `parking_map_real`. OCR 원본과 같은 해상도의 종이 바깥 처리 PNG 및 대응 라벨을 사용합니다. |
| ID | A/B → `building_a`/`building_b`, EN/EX → `entrance`/`exit`, P1~P6 → `space_1`~`space_6`. P3·P6은 장애인석입니다. |
| 지도 일치 | `map_id`, `id_assignment_revision`, `map_image_sha256`을 비교합니다. 현재 ID 버전은 `ocr-p-labels-v1`입니다. SHA-256은 **기준 지도 PNG**의 값입니다. |
| 좌표 | 지도와 카메라 픽셀은 각각의 프레임에서 사용합니다. 주행의 원점은 전지 왼쪽 위, 단위는 cm, +x는 오른쪽, +y는 아래입니다. 각도는 오른쪽 0°, 시계 방향이 양수입니다. |
| 점유 | 실제 관측 시각 기준 기본 TTL 5초, `free` 신뢰도 0.5 이상인 칸만 선택합니다. 오래되거나 누락된 칸은 `unknown`입니다. |

같은 물리적 칸의 ID는 유지하고, 인쇄 번호를 다시 붙여 ID를 바꾸면 버전을 올립니다. 내부 도로 노드 번호 `N…`는 다시 생성될 수 있습니다. 기본 지도의 사진이나 ID 버전이 바뀌면 의미 지도·ROI·보정·경로를 함께 확인하세요.

## 담당과 전달물

| 담당 | 다음 작업 | 전달물 |
| --- | --- | --- |
| 맵 제작팀 | HTTP PNG에 OCR 자동 생성 연결. 실제 카메라 구도로 시설·주차칸과 점유 검증. 새 지도에서 기준점 4개 실측. | 기준 PNG·라벨·의미 지도, 카메라 ROI와 빈 기준 사진, 최신 점유 JSON, 검증된 cm 경로 |
| 주행팀 | 바퀴를 띄우고 모터·ToF·정지 시험. ArUco 위치를 같은 월드 좌표계로 연결한 뒤 짧은 직선부터 시험. | 위치/방향 입력, 모터 보정값, 장애물·통신·추적 유실 시 정지 기록, 도착 오차 |

## 카메라 점유 설정

먼저 카메라를 고정하고 빈 주차장 사진을 저장합니다. 아래 모델 경로와 카메라 번호는 장치에 맞춰 바꿉니다. ROI 설정 창에서 칸마다 네 모서리를 순서대로 클릭하고 Enter로 확정합니다. R은 다시 지정, Esc는 취소입니다.

```powershell
$python = ".\.venv\Scripts\python.exe"
$model = Join-Path ([Environment]::GetFolderPath('Desktop')) 'ParkingHaeven\Source\models\parking_cars.onnx'
& $python -m src.parking_detection.onnx_occupancy --model $model --camera 1 --save-frame archives/occupancy_setup/empty.png --output archives/occupancy_setup/initial.json
& $python -m src.parking_detection.configure_regions --image archives/occupancy_setup/empty.png --output archives/occupancy_setup/regions.json --verified-empty-reference
& $python -m src.parking_detection.onnx_occupancy --model $model --camera 1 --regions archives/occupancy_setup/regions.json --empty-reference archives/occupancy_setup/empty.png --output archives/occupancy_setup/current.json
& $python -m src.rover_control --points floor_points.json --occupancy archives/occupancy_setup/current.json --case 01_building_a
```

`--verified-empty-reference`는 실제로 빈 사진을 확인한 뒤 사용합니다. 마지막 두 명령은 TTL 안에 이어 실행합니다. 현재 ROI와 일치하는 빈 배경이 없으면 차량 미검출을 `free`로 처리하지 않습니다. 카메라 위치·해상도·크롭이 바뀌면 ROI를 다시 잡습니다.

## 점유 JSON

```json
{
  "schema_version": "0.1-draft",
  "map_id": "parking_map_real",
  "id_assignment_revision": "ocr-p-labels-v1",
  "map_image_sha256": "11c848278e4840493db5a799cd71a1f48cc1418daf47c02bbdc8f8b5e60b50cd",
  "observed_at": "2026-11-12T10:00:00+09:00",
  "spaces": [
    {"parking_space_id": "space_1", "status": "occupied", "confidence": 0.96},
    {"parking_space_id": "space_2", "status": "free", "confidence": 0.91}
  ]
}
```

시각은 예시입니다. 생산자가 실제 관측 시각을 넣고, 차량 탐지 점수와 칸 상태의 신뢰도를 구분합니다. 조건에 맞는 `free` 칸이 없으면 경로 생성이 중단됩니다. ~~ParkingHaeven의 이전 4:3 구역 매핑과 저장된 점유 boolean을 그대로 실시간 입력으로 사용~~하는 방식은 폐기했습니다. 새 웹 구역 JSON을 받아 현재 칸 ID와 다시 대조해야 합니다. 과거 저장 파일 재생은 오프라인 시험입니다.

## 경로와 로버 위치

- 지도에서 경로를 선택할 때는 픽셀 그래프를 사용합니다. 로버 경로 생성 시 웨이포인트와 간선·경로 길이를 cm로 변환하고, 최종 점을 선택한 주차칸 중앙으로 둡니다.
- 주행 파일은 `coordinate_frame: world`, `unit: cm`, `origin: paper_top_left`, `calibration_verified: true`와 `waypoints: [[x, y], ...]`를 사용합니다. `world_estimate`는 확인용 미리보기입니다.
- 로버 위치는 같은 노트북의 UDP `127.0.0.1:8766`에 약 0.10초마다 보냅니다. `captured_monotonic_s`는 그 노트북에서 프레임을 받은 시점의 `time.monotonic()`입니다. 0.20초를 넘긴 위치는 사용하지 않습니다.

```json
{"type":"pose","map_id":"parking_map_real","coordinate_frame":"world","unit":"cm","origin":"paper_top_left","x_cm":8.95,"y_cm":40.06,"heading_deg":0.0,"captured_monotonic_s":12345.6}
```

추적을 잃으면 `{"type":"lost","map_id":"parking_map_real"}`을 보냅니다. UNO Q의 TCP 기본 포트는 `8765`입니다. 명령과 응답은 줄바꿈으로 구분하고, 로버는 명령 `seq`를 `applied_seq`로 확인합니다.

```jsonl
{"type":"drive","seq":1,"linear_cm_s":6.0,"angular_deg_s":-20.0}
{"type":"status","applied_seq":1,"state":"moving"}
{"type":"stop","seq":2,"reason":"goal_reached"}
{"type":"status","applied_seq":2,"state":"stopped"}
```

배선·모터 보정·정지 조건은 [UNO Q 수신 앱](../apps/q_rover_receiver/README.md)을 참고합니다. 실주행은 실측 보정과 위치·정지 시험 후 출발점 근처의 짧은 직선부터 진행합니다.

## Git 작업

담당 기능 브랜치는 최신 `main`에서 분기하고, 기능별로 테스트한 뒤 PR을 올립니다. 팀원에게는 변경 목적, 실행 명령·결과, 영향받는 데이터 규칙, 남은 실물 확인을 전달합니다. 모델·카메라 원본·실행 로그·`archives/`는 로컬에서 관리합니다.
