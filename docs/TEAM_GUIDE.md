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
| 맵 제작팀 | HTTP OCR 자동 생성은 연결됨. 실제 카메라 구도로 시설·주차칸과 점유 검증. 새 지도에서 기준점 4개 실측. | 기준 PNG·라벨·의미 지도, 카메라 ROI와 빈 기준 사진, 최신 점유 JSON, 검증된 cm 경로 |
| 주행팀 | 바퀴를 띄우고 모터·ToF·정지 시험. ArUco 위치를 같은 월드 좌표계로 연결한 뒤 짧은 직선부터 시험. | 위치/방향 입력, 모터 보정값, 장애물·통신·추적 유실 시 정지 기록, 도착 오차 |

## 카메라 점유 설정

HTTP 지도 생성은 `scripts.http_map_server`가 수신 PNG에서 OCR 힌트를 새로 생성합니다. 기존 카메라 종횡비나 수동 힌트·전지 외곽을 기본으로 재사용하지 않습니다. 검증된 결과는 고정 폴더 `data/http_map_captures/latest/`에 덮어쓰고, 실패 진단은 `last_failed/`만 갱신합니다. 실패 시 직전 성공 지도를 보존합니다. `capture_id`는 기록용이고 실제 폴더는 응답의 `output_directory`를 사용합니다. 생성한 지도를 기본 지도·ROI·보정·점유와 연결할 때는 위 지도 식별자와 사진 해시 계약을 별도로 맞춰야 합니다. 카메라 이동 후 기존 ROI를 그대로 사용하면 안 됩니다.

기본 OCR 모드는 동일 캡처의 지도에서 5개 조건별 픽셀 경로도 생성합니다. 응답의 `route_generation`에 별도 성공/실패 상태와 결과 URL이 추가되며 기존 지도 필드는 유지합니다. 다운로드 JSON에도 동일한 지도 ID·사진 해시·ID 버전이 들어갑니다. `planning_mode: static_map`, `occupancy_checked: false`이며 실제 빈칸을 보장하지 않습니다. 끝점은 주차칸에 연결된 도로 노드입니다. 경로 실패 시 지도는 보존하고 미완성 경로는 게시하지 않습니다.

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

로버 실측 치수는 폭 15cm, 전체 길이 19cm입니다. 주행 제어 기준점은 좌우 구동 바퀴 축의 중간점이며 앞끝까지 9.5cm, 뒤끝까지 9.5cm입니다. 좌우 폭이 대칭이라고 가정하면 기준점에서 좌우 각각 7.5cm를 차지합니다. 마커 면 높이는 약 10.5cm이고 마커 중심은 기준점보다 앞쪽 2.5cm입니다. 이 치수는 통로·주차칸·회전 공간 검사의 입력 기준으로 기록하며, 현재 경로 추종의 차체 외곽 충돌 검사는 후속 구현 대상입니다.

- 자동 보정에서는 빨간 십자 A~D의 실측 cm 좌표를 한 번 등록하고 같은 배치의 새 카메라 프레임에서 픽셀을 다시 검출합니다. 실물 주행에는 독립 확인점 E의 검증을 통과한 경로를 전달합니다. 기본 지도의 E는 A~D 안쪽에 있는 `space_5` 진입 도로와 EN–EX 도로의 교점입니다. 보정 보고서의 `validation`에는 교점의 시설 ID, 픽셀, 실측 cm, 변환한 cm, 오차가 기록됩니다. 캡처 PNG와 라벨 및 보정 보고서를 주행 경로와 함께 전달합니다. [자동 보정 실행 방법](../README.md#카메라에서-빨간-십자-자동-검출)
- HTTP에서 `--marker-layout`를 사용하면 같은 캡처의 픽셀 경로를 재사용해 `latest/world/`에 보정 결과를 저장합니다. API의 `world_calibration.status`가 `verified`이면 주차칸 중앙까지의 cm 경로를 제공합니다. 픽셀 도착 시험은 `route_generation`의 진입점을 사용합니다. [HTTP 자동 보정 실행 방법](../README.md#같은-http-사진으로-cm-지도-만들기)
- `POST /api/aruco`, `GET /api/rover-pose`에서는 ID 7의 `rover.center_px`·`rover.heading_deg`를 카메라 픽셀과 영상 방향각으로 제공합니다. 목표 전송 주기는 100ms이며, 수신 후 200ms가 지나거나 마커를 잃으면 위치를 비웁니다. 로버 상단 마커를 월드 cm로 전달하려면 마커 높이와 로버 중심 오프셋의 실측 검증이 필요합니다.

- 지도에서 경로를 선택할 때는 픽셀 그래프를 사용합니다. 로버 경로 생성 시 웨이포인트와 간선·경로 길이를 cm로 변환하고, 최종 점을 선택한 주차칸 중앙으로 둡니다.
- 주행 파일은 `coordinate_frame: world`, `unit: cm`, `origin: paper_top_left`, `calibration_verified: true`와 `waypoints: [[x, y], ...]`를 사용합니다. `world_estimate`는 확인용 미리보기입니다.
- 로버 위치는 같은 노트북의 UDP `127.0.0.1:8766`에 약 0.10초마다 보냅니다. `captured_monotonic_s`는 그 노트북에서 프레임을 받은 시점의 `time.monotonic()`입니다. 0.20초를 넘긴 위치는 사용하지 않습니다.
- `src.rover_control.pose_sender`는 주행 전에 검증·저장한 바닥 변환식을 읽고, ArUco API가 보내는 로버 마커의 원본 픽셀 위치·방향을 월드로 바꿉니다. 웹캠 원본과 지도 PNG의 같은 비율 크기 차이를 반영합니다. 같은 구도에서 십자가 가려져도 변환식을 재사용합니다. 위치 메시지에는 `map_image_sha256`과 `id_assignment_revision`도 포함하며 주행기가 경로와 대조합니다. `--preview-only`로 위치를 확인할 수 있습니다.
- 카메라 위치·높이를 수동 입력하지 않고 상단 마커의 바닥 투영 근사 좌표를 사용합니다. 주행 전 카메라 위치를 정해 보정하고 주행 중 구도를 유지합니다. 위치·줌·전지 배치가 달라지면 다시 보정합니다. 현재 마커 높이 실측은 약 10.5cm이며 높이 보정은 미적용입니다. `--center-offset-cm`은 마커에서 구동 바퀴 축 중간점까지 전방·오른쪽 거리이고 실행 기본값은 `[-2.5, 0]cm`입니다. 좌우 편차는 0으로 가정하며 실제 중심 위치와 대조합니다. 로버 마커 유실·중복, 관측 만료, 통신 오류, 카메라 세션·해상도 또는 기준 지도 불일치, 전송기 종료에는 `lost`를 보냅니다. [실행·설정](../README.md#보정된-cm-위치를-주행-프로그램에-보내기)

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

픽셀 도착 시험은 `GET /api/arrival-tests`, `POST /api/arrival-test/start`, `POST /api/arrival-test/stop`, `GET /api/arrival-test`를 사용합니다. 생성한 5개 경로의 마지막 도로 진입점을 선택하며 기본 반경 15px·안정 유지 1초·최소 5관측으로 판정합니다. `X-Camera-Session`, `X-Source-Type`, `X-Source-Width`, `X-Source-Height`를 지도 생성과 마커 입력에 함께 기록해 같은 웹캠 연결/해상도인지 확인합니다. 실제 물리적 구도는 별도 고정해야 합니다. 지도 버전/연결 변경 시 무효화하며 상단 마커 위치를 지도 픽셀로만 비율 변환합니다. 결과는 `data/rover_runs/arrival/latest.json`, `planning_mode: image_pixel_test`입니다. 이 성공은 주차 완료·경로 주행 성공·월드 cm 도착을 뜻하지 않으며 주행팀의 보정/제어 코드는 변경하지 않습니다.

담당 기능 브랜치는 최신 `main`에서 분기하고, 기능별로 테스트한 뒤 PR을 올립니다. 팀원에게는 변경 목적, 실행 명령·결과, 영향받는 데이터 규칙, 남은 실물 확인을 전달합니다. 모델·카메라 원본·실행 로그·`archives/`는 로컬에서 관리합니다.
