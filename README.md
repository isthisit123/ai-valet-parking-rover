# Q-ROVER — AI Valet Parking Rover

지도 사진에서 주차칸과 도로를 찾고, 빈칸까지 경로를 계산해 Q-ROVER에 전달합니다. 현재 **로컬·HTTP PNG → OCR 힌트 → 의미 지도 → 조건별 픽셀 경로**와 기본 지도의 **cm 경로 미리보기**를 검증했습니다. 실제 카메라 점유 연동과 로버 주행은 [로드맵](docs/ROADMAP.md)에 남아 있습니다.

## 기본 지도

- `assets/maps/parking_map_real.png`가 기본 지도입니다. `parking_map_ocr.png`에서 종이 바깥만 흰색 처리했습니다. 원본 해상도와 내부 픽셀은 같습니다. 외곽 설정은 [parking_map_real_preparation.json](assets/maps/parking_map_real_preparation.json)에 있습니다.
- 이름표는 건물 **A/B**, 입구 **EN**, 출구 **EX**, 주차칸 **P1~P6**입니다. `P3`·`P6`은 파란색 장애인석이고 나머지는 초록색 일반석입니다. 도로는 빨강, 건물과 출입구는 검정입니다.
- OCR 라벨, 카메라 구역, 점유 JSON, 경로는 같은 지도 ID·칸 ID 버전·기준 사진 해시를 사용합니다. [팀 통합 규칙](docs/TEAM_GUIDE.md#통합-기준-지도점유경로)

## 설치 및 지도 생성

저장소 루트에서 최초 한 번 실행합니다. 이미 준비된 가상환경이 있으면 해당 Python을 사용하세요.

```powershell
py -m venv .venv
$python = ".\.venv\Scripts\python.exe"
& $python -X utf8 -m pip install -r requirements.txt

& $python -m venv .venv-ocr
& .\.venv-ocr\Scripts\python.exe -X utf8 -m pip install -r requirements-ocr.txt
```

새 사진을 `assets/maps/parking_map_new.png`로 저장한 뒤 실행합니다.

```powershell
& $python -m scripts.generate_map --map-id parking_map_new --replace
```

OCR이 시설 10개의 좌표를 추출해 `assets/maps/parking_map_new_labels.json`을 만듭니다. 색상 영역과의 매칭이 성공하면 `data/maps/parking_map_new/`에 `graph.json`, `graph_preview.png`, `semantic.json`을 저장합니다. 같은 ID로 재실행하면 같은 위치의 결과를 갱신합니다. OCR 실패나 매칭 오류가 나면 이전 결과를 보존하고, 상세 기록은 `data/ocr_building_tests/`에 남깁니다.

기존 기본 지도에서 시험하려면 `--map-id parking_map_real --replace`를 사용합니다. 종이 바깥과 검은 시설이 붙어 보이는 사진은 원본과 해시가 일치하는 외곽 설정을 자동 사용하거나 `--paper-preparation <파일>`을 지정합니다.

### 웹캠으로 촬영하고 지도 생성

주차장 전체와 시설 이름표가 카메라에 보이도록 맞춘 뒤 실행합니다. `--camera`만 입력하면 1번 카메라를 엽니다. 다른 카메라를 사용하려면 `--camera 0`처럼 번호를 지정합니다.

```powershell
& $python -m scripts.generate_map --camera --map-id parking_map_capture --replace
```

OpenCV가 촬영한 PNG로 OCR과 지도 생성을 실행합니다. Windows에서는 DirectShow로 카메라를 열고, 15초 동안 촬영이 완료되지 않으면 오류를 표시합니다. 검증에 성공하면 `assets/maps/parking_map_capture.png`, 대응 라벨 JSON과 `data/maps/parking_map_capture/`를 같은 이름으로 갱신합니다. 촬영이나 검증이 실패하면 직전 결과를 보존합니다.

## cm 경로 미리보기

### 카메라에서 빨간 십자 자동 검출

[기준점 설정](assets/maps/parking_map_real_markers.json)의 `markers_cm`에 십자 교차점의 실측 좌표를 한 번 입력합니다. 전지 왼쪽 위에서 오른쪽·아래쪽으로 잰 `[x, y]` cm를 씁니다. 전지와 십자의 배치가 그대로면 카메라를 바꿔도 이 값을 재사용합니다.

- 현재 사진의 A는 입구 위, B는 출구 쪽 도로 위, C는 P4 오른쪽, D는 입구 쪽 도로 아래의 십자입니다. A~D가 외곽에서 왼쪽 위부터 시계 방향으로 보이도록 촬영하세요. 전지 전체가 화면에 들어오고 십자는 빨간 도로와 떨어져 있어야 합니다.
- 기본 지도에서 확인점 E는 **P5로 연결되는 도로와 EN–EX 도로의 교점**입니다. 두 도로 중심선이 만나는 지점의 실측 `[x, y]` cm를 `junction_check.world_cm`에 입력합니다. `junction_check.parking_space_id`는 `space_5`입니다. 프로그램은 시설 ID와 도로 그래프로 교점의 픽셀 좌표를 찾고, A~D로 계산한 변환의 오차를 기본 1 cm 기준으로 확인합니다.
- 다른 지도의 확인점은 `markers_cm.E`에 A~D 안쪽에 붙인 독립 빨간 십자의 실측 좌표로 지정할 수 있습니다. 확인점 설정을 생략하면 cm 추정 미리보기를 생성합니다.

```powershell
$python = ".\.venv\Scripts\python.exe"
& $python -X utf8 -m src.rover_control --marker-layout assets/maps/parking_map_real_markers.json --camera 1
```

카메라 번호는 연결한 장치에 맞춥니다. 저장한 사진으로 실행할 때는 `--camera 1` 대신 `--image <카메라 PNG 경로>`를 지정합니다. `--calibration-only`로 보정 결과까지만 확인할 수 있고, 도로 교점 확인에는 이 옵션을 사용해도 OCR과 도로 그래프 처리가 필요합니다. 실행 폴더에는 촬영 PNG, 자동 검출한 `floor_points.json`, 보정 결과가 저장됩니다. 전체 실행에서 E 검증을 통과하면 같은 폴더의 `routes/`에 주행용 `*_cm.json`이 생성됩니다. 사진과 라벨 해시 검증에 필요한 파일들이므로 실행 폴더 구조를 함께 유지하세요.

이번 촬영의 cm 지도는 터미널에 출력된 실행별 `cm_preview` 경로에서 확인합니다. 모든 경로가 생성되면 공용 cm 지도 폴더도 갱신됩니다. 기본 시설 인식에 실패하면 같은 사진에 어두운 영상용 색상 기준을 적용해 한 번 더 시도합니다. 실패 원인과 각 시도의 OCR 결과는 터미널과 실행별 `ocr` 폴더에 남습니다.

### 등록된 PNG의 기존 보정 입력 사용

```powershell
& $python -m src.rover_control --points floor_points.json
```

현재 기본 지도 결과는 `data/rover_runs/cm_preview/parking_map_real/`에 나옵니다. `floor_points.json`은 현재 기본 PNG에 대응합니다. 다른 사진으로 바꿨다면 해당 사진의 기준점 파일을 사용하세요.

전지 모서리만 쓴 결과는 `world_estimate` 미리보기입니다. 같은 PNG에서 실측 기준점 4개를 등록하고 검증을 통과하면 실행별 폴더에 주행용 `*_cm.json`도 생성합니다. 로버 설치·배선은 [수신 앱 가이드](apps/q_rover_receiver/README.md), 위치와 경로 규약은 [팀 가이드](docs/TEAM_GUIDE.md#경로와-로버-위치)를 참고하세요.

## 웹에서 PNG 전송

```powershell
$webRoot = Join-Path ([Environment]::GetFolderPath('Desktop')) 'ParkingHaeven\Source'
& $python -m scripts.http_map_server --web-root $webRoot --port 8001
```

[localhost:8001](http://localhost:8001)에서 **현재 화면 PNG 전송 · 맵 생성**을 누릅니다. 서버는 `POST /api/map-capture`로 원본 PNG를 받아 **로컬 명령과 같은 OCR 지도 생성**을 실행합니다. 고정 폴더 `data/http_map_captures/latest/`의 `source.png`, `labels.json`, `graph.json`, `graph_preview.png`, `semantic.json`, `report.json`을 검증 성공 시 덮어씁니다. OCR 기록도 `ocr/report.json`, `ocr/preview.png`의 고정 파일명으로 저장하며 회차 폴더를 누적하지 않습니다. 실패 원본·진단 자료는 고정 폴더 `last_failed/`만 갱신하고 직전 성공 `latest/`와 기본 지도는 보존합니다. 과거에 생성한 회차 폴더는 삭제하지 않습니다.

새 사진의 A/B, P1~P6, EN/EX를 읽으므로 이전 카메라 매핑·종횡비·외곽 좌표를 기본으로 재사용하지 않습니다. 입력 PNG의 크기·방향·바이트를 보존하며 원근 보정은 별도입니다. 주차장 전체와 이름표가 보여야 하고, 색상 영역 매칭도 통과해야 합니다. OCR Python은 `.venv-ocr`를 자동 선택하며 다른 환경은 `--ocr-python <Python 경로>`로 지정합니다. 예전 고정 구도 흐름이 필요한 경우에만 `--legacy-hints`를 명시합니다.

전체 프레임에서 건물 이름표를 놓치면 검은 시설을 더 어두운 기준으로 재검출해 영역·글자 확대 OCR을 시도합니다. 회색 바닥과 시설이 합쳐져 B 영역이 누락되는 경우를 보완하며, 신뢰도 기준을 낮추거나 `8`을 B로 바꾸지는 않습니다. 실물 사진의 어두운 초록색 검출 기준은 디지털 지도와 별도로 관리합니다.

서버 코드 변경 후에는 실행 중인 서버를 **Ctrl+C로 종료한 뒤 같은 명령으로 재실행**하고, 웹에서 **Ctrl+F5**를 누릅니다. 터미널에 `Generation: OCR (...)`, 웹에 새 사진의 OCR 안내가 표시되는지 확인하세요. 기본 OCR 모드에서는 지도 생성 후 아래 5개 픽셀 경로도 자동 생성합니다.

### 같은 HTTP 사진으로 cm 지도 만들기

전지 위 A~D와 독립 확인점 E를 실측해 [기준점 설정](assets/maps/parking_map_real_markers.json)에 입력하고 서버를 실행합니다. 설정의 `map_id`는 서버의 `--map-id`와 같아야 합니다.

```powershell
& $python -m scripts.http_map_server --web-root $webRoot --port 8001 --marker-layout assets/maps/parking_map_real_markers.json
```

웹에서 맵 생성 버튼을 누르면 같은 `source.png`와 이미 만든 픽셀 경로로 보정을 수행합니다. 화면에서 확인점 오차, cm 지도, 검증된 cm 경로를 확인할 수 있습니다. `latest/world/`에 자동 기준점과 보정 보고서를 저장합니다. E 없이 실행하면 cm 추정 미리보기를 제공하고, E 검증을 통과하면 주차칸 중앙까지 이어지는 `world/routes/*_cm.json`도 제공합니다. 검증에 실패한 회차는 픽셀 결과와 보정 오류를 표시합니다. cm 경로는 함께 생성한 `source.png`와 `labels.json`이 있는 캡처 폴더 구조로 보관하세요.

### 웹에서 조건별 경로 확인

같은 캡처의 PNG·라벨·의미 지도로 다음 조건을 계산합니다. 웹의 **조건별 경로**에서 선택된 주차칸과 거리(px)를 확인하고 **PNG 보기 / PNG 저장 / waypoint JSON 저장**을 누릅니다.

| 조건 | 대상 주차칸 |
| --- | --- |
| A 건물 가까운 일반석 | 일반석 |
| 장애인석 지정 | 장애인석 |
| 출구 가까운 일반석 | 일반석 |
| B 건물 가까운 장애인석 | 장애인석 |
| 입구 가까운 장애인석 | 장애인석 |

파일은 `data/http_map_captures/latest/routes/`의 같은 10개 PNG·JSON에 덮어쓰며, 5개 모두 도로 영역·좌표 연결 검사를 통과한 뒤 게시합니다. 경로 검증이 실패하면 이번 의미 지도는 `last_failed/`에서 확인할 수 있고 직전 성공 `latest/`는 유지합니다. 웹에 **지도 생성 성공 / 경로 검증 실패**를 구분해 표시하며 일부만 생성된 경로는 게시하지 않습니다. API 응답의 `route_generation.status`는 `generated` 또는 `failed`이며, 성공 시 `cases`에 조건·선택 칸·거리·waypoint 수·고정 PNG/JSON URL을 제공합니다. `capture_id`는 기록 식별자이며 폴더명으로 사용하지 않습니다. 실제 위치는 `output_directory`, 저장 구분은 `storage_key`를 사용하세요.

이 경로는 **점유 미반영 정적 지도 시험**입니다. JSON의 `planning_mode`는 `static_map`, `occupancy_checked`는 `false`, 좌표는 원본 지도 픽셀입니다. 끝점은 주차칸에 연결된 도로 진입점(`waypoint_target: parking_road_node`)입니다. 실제 빈칸 선택과 주차칸 중앙 도착을 위한 점유·cm 보정·주행 검증은 후속 단계입니다.

### 1단계: 수신 PNG로 로컬 OCR 지도 생성 검사

웹 버튼이 OCR까지 자동 실행하므로 별도 검사 명령은 필수가 아닙니다. 실패한 회차를 로컬에서 다시 검사하려면 웹에 표시된 저장 폴더를 아래 `--capture-dir`에 넣습니다. 과거 힌트 매핑이 실패했더라도 `source.png`와 `report.json`이 저장됐다면 검사할 수 있습니다.

```powershell
& $python -m scripts.check_http_capture --capture-dir "data/http_map_captures/latest"
```

수신 보고서와 원본 PNG의 SHA-256·해상도를 대조한 뒤, 크기 변경이나 재인코딩 없이 같은 이미지로 기존 로컬 OCR 지도 생성을 실행합니다. 이름표 10개와 색상 시설 매핑을 모두 통과해야 성공입니다. 실제 영상에 주차장 전체와 A/B, P1~P6, EN/EX가 보이는지는 저장된 `source.png`와 OCR 미리보기로 확인하세요. 종이 바깥과 시설이 붙어 검출되면 **해당 PNG에서 확인한** 외곽 설정을 `--paper-preparation <파일>`로 지정할 수 있습니다.

결과는 해당 캡처 폴더의 `ocr_check_<고유값>/`에 저장합니다. `report.json`에 입력 정보·실패 단계·오류 또는 시설 10개 매핑 결과가 기록되고, 성공하면 `labels.json`, `map/graph.json`, `map/graph_preview.png`, `map/semantic.json`이 생성됩니다. OCR 상세 기록·미리보기는 `ocr/`에 남습니다. 수신 파일과 기본 지도, 이전 검사 결과는 덮어쓰지 않습니다.

## 로버 ArUco 위치·영상 방향각

같은 HTTP 서버를 재시작하고 웹을 Ctrl+F5로 새로고침한 뒤 카메라를 켭니다. 기존 웹캠을 공유하며 별도 OpenCV 카메라 창은 열지 않습니다. 기본 사전은 `DICT_4X4_50`, 로버 ID는 사용자 사진에서 확인한 **7**, 전방은 마커의 **오른쪽 축(+x)**입니다. 웹의 ArUco 모듈은 서버에서 제공하므로 ParkingHaeven 원본 파일을 수정할 필요가 없습니다.

```powershell
& $python -m scripts.http_map_server --port 8001 --rover-marker-id 7 --aruco-period-ms 100
```

웹 아래 패널에 마커 중심 `(x, y)` 픽셀과 영상 방향각을 표시합니다. 원점은 원본 카메라 영상 왼쪽 위, +x 오른쪽·+y 아래, 각도는 오른쪽 0°·시계 방향 양수입니다. 마커를 다시 붙여 전방이 바뀌면 `--marker-front-offset-deg`로 마커 오른쪽 축에서 전방까지의 보정각을 지정하세요. 로버 ID·사전은 `--rover-marker-id`, `--aruco-dictionary`로 지정합니다.

추천 기본 주기는 **100ms(10Hz)**입니다. PNG 전송·응답 시간을 포함해 다음 요청까지 남은 시간만 기다리며 요청은 중첩하지 않습니다. 새 비디오 프레임이 없으면 전송하지 않습니다. 실제 응답 간격과 검출 시간을 화면에 표시하며, 브라우저 왕복 지연이 200ms를 넘으면 현재 위치로 표시하지 않습니다. 노트북의 기존 주행 위치 만료 기준도 200ms입니다. 첨부 사진 한 장에서 검출 약 19ms를 확인했으며 실제 웹캠·YOLO 병행 시 목표 주기는 별도 실측해야 합니다.

```powershell
Invoke-RestMethod http://localhost:8001/api/rover-pose | ConvertTo-Json -Depth 6
```

`POST /api/aruco`가 검출하고 `GET /api/rover-pose`가 최신 상태를 반환합니다. `rover.center_px`, `rover.heading_deg`, `tracking_status`를 확인하세요. `detected`일 때만 현재 로버 값이 있으며 `lost`·`ambiguous`·`stale`에는 `rover: null`입니다. 서버가 프레임을 받은 지 200ms를 넘기면 GET에서 만료 처리합니다. JSON은 고정 파일 `data/rover_runs/aruco/latest.json`에 덮어쓰며 프레임 PNG를 누적 저장하지 않습니다. 파일을 직접 읽는 소비자는 같은 노트북의 `time.monotonic()`과 `received_monotonic_s`, `valid_for_ms`로 만료 여부를 직접 확인해야 합니다.

현재 결과는 `coordinate_frame: camera_image`, `unit: pixel`인 **로버 상단 마커 중심·영상 방향각**입니다. `map_pose`는 `null`이며 월드 cm나 바닥 기준 로버 중심으로 사용하지 않습니다. 마커 높이·마커와 로버 중심의 오프셋·카메라/지도 보정을 확인한 뒤 기존 주행용 월드 cm·UDP 계약에 연결해야 합니다. 이 단계는 모터 명령을 보내지 않습니다. 마커 모서리 순서는 [OpenCV 문서](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)를 따릅니다.

## 픽셀 도착 판정 테스트

서버 재시작·웹 Ctrl+F5 후 카메라를 켜고, 로버를 치운 상태에서 **맵을 1회 생성**합니다. 이번 기능은 카메라 연결 정보가 함께 저장돼야 하므로 업데이트 전의 지도는 한 번 다시 생성해야 합니다. 이후에는 지도 재생성 없이 마커 추적과 케이스 변경만 수행합니다.

1. **로버 도착 판정 테스트**의 드롭다운에서 5개 조건 중 하나를 선택합니다. 조건 이름과 선택 주차칸(P1~P6)을 표시합니다.
2. 지도 픽셀 기준 허용 반경을 지정하고 **테스트 시작**을 누릅니다. 기본 반경은 15px입니다. 노란 원이 목표 영역이며 하늘색 점은 판정용 마커 위치입니다.
3. 로버를 수동으로 이동합니다. 선택한 경로의 **마지막 도로 진입점** 반경 안에서 1초 동안 위치가 안정되면 **성공 — 도착 판정 완료**가 표시됩니다. 안정성은 최초 관측점으로부터 `min(5px, 반경/3)` 이내이며, 최소 5개 관측이 필요합니다.
4. 다음 케이스를 선택하면 기존 시험을 중단합니다. 새로 **테스트 시작**을 눌러 목표를 고정합니다. 성공은 다음 시작·중단·지도 변경까지 해당 시험의 완료 결과로 유지합니다.

마커 미검출·중복·지연·200ms 관측 공백에는 유지 시간을 초기화합니다. 같은 프레임의 반복으로 시간을 채우지 않습니다. 새 지도 생성, 카메라 연결·해상도 변경은 기존 시험을 중단/무효화합니다. 영상과 지도 해상도가 달라도 동일 웹캠 세션·동일 구도·원본 전체 프레임인 경우 지도 픽셀로 비율 변환합니다. 카메라를 물리적으로 움직인 것은 자동 검출하지 못하므로 구도·줌·크롭은 유지하세요.

`GET /api/arrival-tests`에서 케이스 목록, `POST /api/arrival-test/start`에서 `{case_id, capture_id, radius_px, hold_seconds}`로 시작, `POST /api/arrival-test/stop`에서 중단, `GET /api/arrival-test`에서 상태를 조회합니다. 웹은 상태를 약 250ms마다 표시하며 ArUco 입력은 기존 목표 100ms 주기를 사용합니다. 결과는 고정 JSON `data/rover_runs/arrival/latest.json`에 덮어씁니다. `coordinate_frame: map_image`, `unit: pixel`, `planning_mode: image_pixel_test`입니다.

이 시험은 로버 상단 마커의 영상상 도착을 표시합니다. 주행에 쓰려면 마커 높이와 로버 중심 오프셋을 실측해 월드 cm 위치 입력을 검증해야 합니다.

## 대체된 작업 방식

| 이전 | 현재 |
| --- | --- |
| ~~`parking_map_v1.png`를 기본 지도로 사용~~ | `parking_map_real.png`와 OCR 라벨을 사용 |
| ~~새 사진마다 시설별 픽셀 힌트를 손으로 입력~~ | `scripts.generate_map`으로 자동 생성. 픽셀 `hint`와 실측 `hint_cm`은 필요한 경우 직접 지정 가능 |
| ~~숫자 `1~6`만 인쇄해 OCR로 읽기~~ | `P1~P6`으로 표기 |
| ~~이전 카메라 구역과 점유 파일 재사용~~ | 새 구도의 카메라 영역을 다시 지정. 이전 기록은 `archives/`에 보관 |

점유 인식 설정은 [카메라 점유 설정](docs/TEAM_GUIDE.md#카메라-점유-설정)을 참고하세요. 테스트 명령은 `& $python -m unittest discover -s tests -v`입니다. 남은 팀별 작업은 [로드맵](docs/ROADMAP.md), 이전 구현과 실행 로그는 [인수인계 기록](docs/PROJECT_HANDOFF.md)에 있습니다.
