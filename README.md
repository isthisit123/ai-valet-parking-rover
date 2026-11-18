# Q-ROVER — AI Valet Parking Rover

지도 사진에서 주차칸과 도로를 찾고, 빈칸까지 경로를 계산해 Q-ROVER에 전달합니다. 현재 **로컬·HTTP PNG → OCR 힌트 → 의미 지도 → 조건별 경로**, 검증된 cm 경로 생성과 **ArUco 픽셀 → 월드 cm 위치·방향 → 주행기 UDP 입력**을 구현했습니다. 실시간 점유는 웹의 픽셀 경로·도착 시험에 연결되어 있습니다. 실시간 경로 변경의 모터 주행 연결과 실물 주행 검증은 [로드맵](docs/ROADMAP.md)에 남아 있습니다.

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

OCR은 P1~P6을 읽었지만 색상 주차칸 수가 맞지 않으면 같은 사진에서 주차칸만 어두운 영상 기준으로 한 번 재검출합니다. 기존 OCR 힌트와 유형·고유 ID가 각각 유일하게 대응해야 성공하며, 성공한 의미 지도에는 `parking_detection_profile: low_light_retry`를 기록합니다. 재검출 후에도 칸이 부족하거나 매핑이 모호하면 실패로 표시하고 직전 지도를 보존합니다.

Windows에서 결과 폴더를 교체할 때 `WinError 5/32/33`이 발생하면 짧게 기다려 최대 6회 시도합니다. 계속 실패하면 이전 결과를 복원합니다. 복원도 잠겨 실패하면 출력 폴더의 `.previous-<고유값>`에 이전 지도를 보존하고 오류에 위치를 표시합니다. 이 백업은 복원 확인 전 삭제하지 마세요. 새 지도 게시 후 백업 정리만 실패한 경우에는 성공 결과를 유지하고 서버에 정리 대기 경고를 남깁니다. `latest`의 PNG를 연 뷰어·탐색기 미리보기 등 폴더를 사용하는 프로그램과 OneDrive의 동기화 잠금을 확인하세요.

### 주차칸 외곽을 ParkingHaeven에 자동 적용

HTTP 서버에서 차량 박스와 점유 판정은 **차량 신뢰도 70% 이상**인 검출만 사용합니다. 예를 들어 P2 35%·P3 48% 검출은 표시·점유에서 제외하고 70%부터 포함합니다. 기존 모델의 추론·전후처리는 Worker에서 수행하고, 이전 추론이 끝나기 전에는 다음 프레임 추론을 시작하지 않습니다. ParkingHaeven 원본 파일과 모델은 수정하지 않으며 기존 ONNX Runtime Web 1.22.0을 사용합니다. Worker 초기화·추론 실패는 미확인으로 전달하고 경로 안내에 빈칸으로 사용하지 않습니다.

제어 중인 로버가 car로 검출되는 경우에는 같은 카메라·해상도이고 추론 영상 촬영 시각에서 150ms 이내인 유효한 ArUco 관측으로 본체를 구분합니다. 마커 네 모서리를 포함하는 차량 박스가 하나이며 크기·위치 조건이 맞을 때만 그 박스를 표시·점유에서 제외합니다. 같은 칸에 다른 차량 검출이 있으면 점유로 유지합니다. 마커가 없거나 오래됐거나 박스가 모호하면 제외하지 않습니다. 화면에 **ArUco로 로버 본체 제외** 또는 **로버/차량 구분 확인 필요**를 표시합니다. 이 점유 정보는 제어 중인 로버를 제외한 다른 차량 기준이며 `occupancy_scope`와 `rover_exclusion`에 적용 범위·근거를 기록합니다.

서버 재시작 후 웹에서 **Ctrl+F5 → 카메라 시작 → 빈 주차장 PNG 전송 · 맵 생성**을 실행합니다. 기본 OCR 모드에서 새 지도·경로 게시에 성공하면 색상 영역의 실제 외곽 꼭짓점을 P1~P6 구역으로 자동 적용하고 현재 영상의 설정을 저장합니다. 웹은 설정 모드로 전환됩니다. 표시된 경계가 실물 칸을 따르는지 확인하고 필요한 꼭짓점만 수동 수정한 뒤 **설정 저장**을 누르세요. ParkingHaeven 원본 파일을 바꾸지 않고 이 서버가 제공하는 연결 스크립트를 사용합니다.

꼭짓점은 동일 사진의 픽셀 좌표이며 웹 캔버스 크기에 맞춰 확대·축소합니다. cm 변환은 구역 설정에 사용하지 않습니다. 보통 네 꼭짓점이며, 네 점으로 외곽을 충분히 보존하지 못하면 추가 꼭짓점을 유지합니다. 촬영 후 영상 소스·카메라 세션·원본 해상도·종횡비가 바뀌거나 칸 ID·외곽 검증이 실패하면 기존 설정을 유지합니다. 카메라를 물리적으로 움직이거나 적용 이후 화면 캔버스 크기를 바꿨다면 다시 생성하세요. 물리적 카메라 이동의 자동 감지는 포함하지 않습니다.

고정 파일 `data/http_map_captures/latest/parking_regions.json`과 웹의 **주차칸 꼭짓점 JSON** 링크에서 원본 좌표와 지도 식별자를 확인할 수 있습니다. 이 파일은 서버 전달 규약입니다. 기존 웹 가져오기에 쓸 파일은 자동 적용 후 웹의 좌표 내보내기로 저장하세요. 초기 `occupied: null`, `occupancy_status: unknown`이며 자동 구역 생성만으로 빈칸이 확인되는 것은 아닙니다. 이후 웹의 기존 차량 검출 기능으로 점유를 확인하세요. 실시간 점유는 아래 픽셀 도착 테스트에 연결되며 cm 주행 경로·모터 제어 연결은 별도입니다.

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

추천 기본 주기는 **100ms(10Hz)**입니다. 원본 해상도의 추적 프레임만 JPEG 품질 0.85로 전송하고, 지도 생성은 PNG를 유지합니다. 인코딩·전송·응답 시간을 포함해 다음 요청까지 남은 시간만 기다리며 요청은 중첩하지 않습니다. 새 비디오 프레임이 없으면 전송하지 않습니다. 인코딩만 200ms를 넘긴 프레임은 전송 전에 버립니다. 전송 요청 자체는 1초 제한으로 종료하지만 좌표 유효 시간은 계속 **200ms**입니다. 서버도 촬영 시각과 수신 시각을 모두 확인해 오래된 위치를 비웁니다.

ArUco 패널은 JPEG 인코딩, 전송·응답, 서버 검출 시간을 따로 표시합니다. `서버 검출 detected · 유효성 stale`은 마커는 읽었지만 프레임이 늦었다는 뜻이고, `서버 검출 lost`는 ID 7 마커를 찾지 못했다는 뜻입니다. `signal timed out`에 해당하는 경우에는 전송·응답 시간 초과를 표시합니다. 재시작·Ctrl+F5 후 로버 상단 마커가 보이도록 놓고 웹 탭을 전면에 둔 상태에서 응답 간격을 확인하세요. 브라우저 비활성 탭 제한·기기 부하 때문에 10Hz를 보장하지 않습니다. 약 19ms 검출 대비 약 3초 응답 지연이 보고돼 Worker 분리·중복 추론 방지·JPEG 전송을 적용했으며, 실제 카메라에서 개선된 간격은 재확인이 필요합니다.

```powershell
Invoke-RestMethod http://localhost:8001/api/rover-pose | ConvertTo-Json -Depth 6
```

`POST /api/aruco`가 검출하고 `GET /api/rover-pose`가 최신 상태를 반환합니다. `rover.center_px`, `rover.heading_deg`, `tracking_status`를 확인하세요. `detected`일 때만 현재 로버 값이 있으며 `lost`·`ambiguous`·`stale`에는 `rover: null`입니다. 서버가 프레임을 받은 지 200ms를 넘기면 GET에서 만료 처리합니다. JSON은 고정 파일 `data/rover_runs/aruco/latest.json`에 덮어쓰며 프레임 PNG를 누적 저장하지 않습니다. 파일을 직접 읽는 소비자는 같은 노트북의 `time.monotonic()`과 `received_monotonic_s`, `valid_for_ms`로 만료 여부를 직접 확인해야 합니다.

이 API의 결과는 `coordinate_frame: camera_image`, `unit: pixel`인 로버 상단 마커 중심·영상 방향각입니다. 월드 cm 변환과 주행 입력 전송은 아래 `pose_sender` 명령으로 실행합니다. 마커 모서리 순서는 [OpenCV 문서](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)를 따릅니다.

### 보정된 cm 위치를 주행 프로그램에 보내기

같은 노트북에서 서버를 실행합니다. 주행 시작 전에 카메라 위치를 정하고, 로버를 치운 상태에서 지도 생성 버튼을 눌러 확인점 검증을 통과한 `latest/world/map_calibration.json`과 cm 경로를 준비합니다. 이때 A~D 십자의 픽셀과 실측 cm 좌표로 구한 변환식을 저장합니다. 주행 중에는 같은 카메라 구도에서 저장한 변환식을 계속 사용하므로 십자가 가려져도 로버 위치를 변환할 수 있습니다. 카메라 위치·방향·줌이나 전지 배치를 바꾼 다음에는 주행 전에 다시 보정합니다.

```powershell
$python = ".\.venv\Scripts\python.exe"
& $python -X utf8 -m scripts.http_map_server --port 8001 --marker-layout assets/maps/parking_map_real_markers.json
```

높이 보정에는 현재 카메라 렌즈 중심의 월드 좌표 `[X, Y, Z]cm`를 실측해 입력합니다. 카메라 내부 보정 파일 대신 이 실측 위치와 기존 바닥 보정식을 사용합니다. 측정 기준은 다음과 같습니다.

| 입력 | 측정 방법 |
| --- | --- |
| 카메라 X·Y | 렌즈 중심에서 전지 평면으로 수직으로 내린 점의 좌표. 전지 왼쪽 위가 원점, 오른쪽 +X·아래쪽 +Y입니다. 영상 중앙이 바라보는 점과 다릅니다. 전지 밖도 가능하며 왼쪽·위쪽은 음수입니다. |
| 카메라 Z | 전지 표면에서 렌즈 중심까지의 수직 높이. 렌즈에서 전지까지의 비스듬한 거리가 아닙니다. 마커 높이보다 커야 합니다. |
| 마커 높이 | 전지 표면에서 로버 상단 마커 면 중심까지의 수직 높이. 현재 실측 기본값 10.5cm입니다. |
| 중심 오프셋 | 마커 중심에서 구동 바퀴 축 중간점까지 로버 전방·오른쪽 거리. 현재 기본값은 `[-2.5, 0]cm`입니다. |

추나 수직 기준을 사용해 렌즈 바로 아래 점을 찾고 X·Y를 재세요. 카메라 배치가 바뀌면 X·Y·Z를 다시 측정하고 현재 구도로 지도를 다시 생성합니다. 카메라 각도나 마커 한 변 길이는 이 방식의 입력값이 아닙니다. 마커는 전지와 평행하게 고정하고 마커의 +x가 로버 전방을 향하는지 확인하세요.

현재 구도의 사용자 실측값은 렌즈 `[53, 151.7, 111.5]cm`, 마커 높이 10.5cm입니다. `data/http_map_captures/latest/world/camera_position.json`에 현재 캡처·카메라 세션·보정 해시와 함께 저장했습니다. 같은 지도에서는 아래 명령만으로 높이 보정된 위치를 확인합니다. 이 파일은 로컬 실행 설정이며 Git 추적 대상이 아닙니다.

```powershell
.\.venv\Scripts\python.exe -X utf8 -m src.rover_control.pose_sender --preview-only
```

새 지도·보정으로 바뀌면 저장값을 거부합니다. 아래처럼 실측값을 명시하면 현재 캡처에 묶어 설정을 다시 저장합니다. 카메라가 물리적으로 이동한 것은 자동 검출하지 못하므로 이동 후에는 반드시 재측정·재보정하세요.

`world/map_calibration.json`이 없으면 위치 표시기는 캡처 보고서의 cm 보정 실패 원인을 표시합니다. 픽셀 지도·경로 생성 성공만으로 cm 보정 성공을 뜻하지 않습니다. 전지 외곽은 실제 종이 경계를 검출한 값이 아니라 십자 실측값에서 계산한 값입니다. 작은 외삽 오차는 영상 가로·세로 각각 2% 여백까지 허용하고 보정 보고서의 `paper_bounds`에 원래 계산 좌표·허용 여백·이탈 여부를 기록합니다. 좌표를 강제로 잘라 맞추지 않으며 독립 확인점 E의 1cm 검증도 유지합니다. 여백을 넘으면 촬영 범위와 A~D 실측값·전지 크기를 함께 확인하세요. 웹의 cm 보정 검증 통과 후 위치 표시를 실행하며, 카메라·전지를 이동했다면 렌즈 X·Y·Z도 재측정합니다.

새 PowerShell에서 저장소 루트로 이동한 뒤 아래를 실행하고 실측값을 입력합니다. 기존 웹캠 API를 사용하므로 카메라를 추가로 열지 않습니다. 터미널에는 약 1초마다 cm 위치와 월드 방향각이 표시됩니다.

```powershell
$cx = [double](Read-Host '렌즈 바로 아래 점의 X 좌표(cm)')
$cy = [double](Read-Host '렌즈 바로 아래 점의 Y 좌표(cm)')
$cz = [double](Read-Host '전지 표면부터 렌즈까지 수직 높이(cm)')
.\.venv\Scripts\python.exe -X utf8 -m src.rover_control.pose_sender --preview-only --camera-position-cm $cx $cy $cz --marker-height-cm 10.5 --center-offset-cm -2.5 0
```

시작 메시지의 `높이 보정 적용`, 카메라 좌표와 마커 높이를 확인합니다. 코드가 실측값을 추정하거나 기존 한 점의 오차를 상수로 더하지 않습니다. 저장된 값과 명시적 카메라 입력이 모두 없으면 실행을 중단합니다. 높이 미보정 진단은 명시적으로 선택해야 하며 `--preview-only`에서만 허용합니다. 바닥에 놓은 마커 자체의 좌표를 확인할 때는 중심 오프셋도 0으로 둡니다.

```powershell
.\.venv\Scripts\python.exe -X utf8 -m src.rover_control.pose_sender --preview-only --floor-projection-only --center-offset-cm 0 0
```

높이 보정은 렌즈 위치 C와 바닥 투영점 Q 사이의 광선에서 마커 높이 h의 교점을 계산합니다. XY 계산은 `C_xy + (1 - h / C_z) × (Q_xy - C_xy)`이며 그 뒤 바퀴 축 오프셋을 적용합니다. 지도·경로는 기존 바닥 cm 좌표를 유지합니다. 마커 중심과 전방 두 점에 동일 높이 보정을 적용하므로 전지와 평행한 마커의 방향각을 보존하며, 기존 16.4° 같은 방향 오차를 임의로 없애지는 않습니다. 마커의 기울기·부착 방향, 렌즈 왜곡과 실측 오차는 별도로 검증해야 합니다. 원점은 전지 왼쪽 위, +x 오른쪽·+y 아래, 오른쪽 0°·시계 방향 양수입니다. P5 교점 또는 독립 십자 E의 검증은 초기 지도·경로 생성 때 수행합니다.

위치를 주행기에 전달하려면 위 명령에서 `--preview-only`를 빼고 실행합니다. 목표 주기는 0.10초이고 UDP `127.0.0.1:8766`으로 `pose`를 전송합니다. `captured_monotonic_s`는 원래 서버 수신 시각을 유지합니다. 같은 프레임을 반복 조회해 관측 시각을 새로 만들지 않습니다. 로버 마커 미검출·중복, 200ms 만료, 통신 오류, 카메라 세션·원본 해상도·기준 지도 불일치에는 `lost`를 보냅니다. 새 기준 지도를 생성하면 새 경로와 위치 전송기를 다시 실행합니다. 실패 뒤 주행기는 정지 상태로 종료하므로 문제를 해결하고 다시 실행합니다.

좌우 PWM과 정지 시험을 마친 뒤, 별도 PowerShell에서 같은 캡처의 주행용 경로를 지정합니다.

```powershell
.\.venv\Scripts\python.exe -X utf8 -m src.rover_control.drive --usb --route data/http_map_captures/latest/world/routes/01_building_a_cm.json --max-speed-cm-s 4
```

ADB가 PATH에 없으면 `--adb '<adb.exe 경로>'`를 추가합니다. 위치·주행 명령의 세부 규약은 [팀 가이드](docs/TEAM_GUIDE.md#경로와-로버-위치)에 있습니다.

### 실제 주행 경로 선택

`drive --route`에 같은 캡처의 `world/routes/`에 있는 cm 경로 JSON을 지정합니다. `01_building_a_cm.json`은 A 건물 가까운 일반석, `02_accessible_cm.json`은 장애인석, `03_exit_cm.json`은 출구 가까운 일반석, `04_building_b_accessible_cm.json`은 B 건물 가까운 장애인석, `05_entrance_accessible_cm.json`은 입구 가까운 장애인석입니다. 선택된 주차칸 중앙이 최종 목적지입니다. 주행기는 시작할 때 지정한 파일을 읽으며, 웹의 실시간 `route_revision`이나 점유 상태를 받아 경로를 교체하는 연결은 아직 없습니다. 웹의 픽셀 도착 시험 드롭다운은 수동 이동 시험에 사용합니다.

실제 주행기는 남은 경로 길이와 목표까지의 직선거리가 모두 2cm 이내이면 정지 명령을 보내고, 정지 응답 뒤 새 위치 3개가 목표 2cm 이내에서 서로 0.5cm 이내로 유지되는지 확인합니다. 아래 웹의 픽셀 도착 성공은 이 정지 확인과 별도입니다.

## 픽셀 도착 판정 테스트

서버 재시작·웹 Ctrl+F5 후 카메라를 켜고, 로버를 치운 상태에서 **맵을 1회 생성**합니다. 이번 기능은 카메라 연결 정보가 함께 저장돼야 하므로 업데이트 전의 지도는 한 번 다시 생성해야 합니다. 이후에는 지도 재생성 없이 마커 추적과 케이스 변경만 수행합니다.

1. 자동 지정된 칸 경계를 확인한 뒤 웹 상단 **모드 선택 → 감시 모드**를 누릅니다. 앞서 “AI 모니터링”이라고 안내했던 차량 검출 모드의 실제 버튼명입니다. 기존 차량 검출 결과를 사용하며, 빈칸은 최소 3개의 새 관측이 2초 이상에 걸쳐 확인돼야 선택할 수 있습니다. 점유 중인 칸은 즉시 후보에서 제외합니다.
2. **로버 도착 판정 테스트**에서 5개 조건 중 하나를 선택합니다. 기본은 조건에 맞는 빈칸 자동 선택입니다. 옆 목록에서 P1~P6을 직접 지정할 수도 있습니다. 일반 조건에는 일반석만, 장애인 조건에는 장애인석만 허용합니다. 점유 칸을 지정하면 **이미 점유된 주차칸입니다**를 표시하고 시작을 막습니다. 미확인 칸이나 최신 점유가 없는 상태도 시작할 수 없습니다.
3. **도착 영역(칸 면적 %)**을 지정하고 **테스트 시작**을 누릅니다. 기본 50%, 조정 범위 10~75%입니다. 노란 원의 중심은 **주차칸 외곽의 중심**이며 원 면적이 지정한 비율이 되도록 칸마다 반지름을 자동 계산합니다. 원이 칸을 벗어나지 않도록 필요한 경우 크기를 제한합니다. 유효한 마커 관측이 들어오면 현재 로버 위치에서 도로를 따라 칸 입구까지 연결하고, 중앙까지 직선 예시 구간을 표시합니다. 하늘색 점은 판정용 마커 위치입니다.
4. 로버를 수동으로 이동합니다. **주차칸 중앙의 노란 원 안**에서 ArUco 마커 중심이 1초 동안 안정되면 **성공 — 도착 판정 완료**가 표시됩니다. 입구에 머무르는 것으로는 중앙 도착이 완료되지 않습니다. 안정성은 최초 관측점으로부터 `min(5px, 자동 반경/3)` 이내이며, 최소 5개 관측이 필요합니다.
5. **더 적합한 빈칸으로 자동 변경**은 기본으로 켜져 있습니다. 예를 들어 출구 조건에서 P4가 점유돼 P5로 이동하다 P4가 2초 연속 빈칸으로 확인되면 현재 위치부터 P4로 다시 경로를 계산합니다. 목표 변경 시 도착 유지 시간은 초기화합니다. 직접 지정한 칸은 자동 변경하지 않습니다. 자동 변경을 끄면 처음 선택된 칸을 유지합니다. 도착 원 진입 전 현재 목표가 점유되면 자동 조건은 대체 빈칸을 찾고, 직접 지정/자동 변경 해제 상태 또는 대체 빈칸이 없으면 안내를 중단합니다. **원에 진입하면 현재 목표를 고정**합니다. 이후 다른 차량의 점유나 미확인이 발생하면 다른 칸으로 바꾸거나 성공 처리하지 않고 안내·안정 유지 시간을 중단합니다. 확인된 로버 본체만 점유에서 제외하고 실제 다른 차량은 계속 반영합니다. 목표 고정은 테스트를 새로 시작할 때 해제됩니다.
6. 케이스·직접 지정 칸을 바꾸면 기존 시험을 중단합니다. 자동 변경 옵션은 중단 후 바꿀 수 있습니다. 목표가 아직 빈칸이고 도착 반경의 두 배 이내에 접근했다면 더 좋은 칸으로 변경하지 않습니다. 반복 변경은 최소 2초 간격으로 제한하며, 현재 목표가 점유되면 이 제한을 적용하지 않습니다. 성공 이후에는 목적지를 변경하지 않습니다.

마커 미검출·중복·지연·200ms 관측 공백에는 유지 시간을 초기화합니다. 같은 프레임의 반복으로 시간을 채우지 않습니다. 새 지도 생성, 카메라 연결·해상도 변경은 기존 시험을 중단/무효화합니다. 영상과 지도 해상도가 달라도 동일 웹캠 세션·동일 구도·원본 전체 프레임인 경우 지도 픽셀로 비율 변환합니다. 카메라를 물리적으로 움직인 것은 자동 검출하지 못하므로 구도·줌·크롭은 유지하세요.

`GET /api/arrival-tests`에서 현재 빈칸 기준 케이스·칸 상태를 조회하고, `POST /api/arrival-test/start`에 `{case_id, capture_id, arrival_area_ratio:0.5, hold_seconds:1.0, parking_space_id:null, auto_retarget:true}`로 시작합니다. 중단은 `POST /api/arrival-test/stop`, 상태는 `GET /api/arrival-test`입니다. 웹은 상태를 약 250ms마다, 후보 목록은 1초마다 표시하며 ArUco 입력은 기존 목표 100ms 주기를 사용합니다. 상태의 `goal_px`는 칸 중앙, `road_goal_px`는 도로 진입점, `arrival_region`은 화면·판정이 공유하는 원 정보입니다. `radius_px`는 자동 계산한 반지름입니다. `waypoints`, `route_revision`, `target_changes`, `guidance_available`이 현재 안내 결과이며 고정 JSON `data/rover_runs/arrival/latest.json`에 덮어씁니다. `coordinate_frame: map_image`, `unit: pixel`, `planning_mode: image_pixel_test`입니다. 마지막 `parking_approach_px`는 중앙까지의 직선 미리보기로 주차 기동·충돌 회피를 보장하지 않습니다.

기존 모델 추론 결과만 사용해 `POST /api/parking-occupancy`로 **새 추론이 끝날 때마다** 전송합니다. 동일 상태도 생략하지 않으며 추가 추론이나 과거 관측의 시각 갱신은 없습니다. `GET /api/parking-occupancy`로 현재 상태를 조회하고 고정 파일 `data/rover_runs/occupancy/latest.json`에 최신 관측을 저장합니다. TTL은 공통 점유 기준인 **촬영 후 5초**입니다(ArUco 위치의 200ms 기준은 유지). 추론 실패·미관측·점유 만료는 `unknown`으로 처리하고 안내·도착 유지 시간을 중단합니다. 연속 빈칸 확인은 촬영 시각끼리 비교하고, 새 프레임 없이 기다린 시간으로 완료하지 않습니다. 같은 빈칸 후보·시작점의 경로 계산은 캐시를 재사용합니다. 페이지를 새로 열거나 카메라를 다시 연결하면 지도를 새로 한 번 생성하세요. 상단의 기존 5개 PNG/JSON은 계속 점유 미반영 정적 예시이며 실시간 안내는 도착 테스트 패널에서 확인합니다.

도착 테스트의 조건 목록은 고정된 이름을 사용하고, 목표 칸·빈칸 상태는 아래 상태 영역에 표시합니다. 갱신마다 목록을 새로 만들지 않아 선택한 조건과 수동 지정 칸을 유지합니다. 칸 목록을 조작하는 동안에는 표시 글자 변경도 보류하고, 최신 점유에 따른 시작 차단은 계속 적용합니다. 상태 줄에는 관측 나이와 5초 만료 기준을 표시합니다.

이 시험은 로버 상단 마커의 영상상 도착을 표시합니다. 주행에 쓰려면 마커 높이와 로버 중심 오프셋을 실측해 월드 cm 위치 입력을 검증해야 합니다.

## 대체된 작업 방식

| 이전 | 현재 |
| --- | --- |
| ~~`parking_map_v1.png`를 기본 지도로 사용~~ | `parking_map_real.png`와 OCR 라벨을 사용 |
| ~~새 사진마다 시설별 픽셀 힌트를 손으로 입력~~ | `scripts.generate_map`으로 자동 생성. 픽셀 `hint`와 실측 `hint_cm`은 필요한 경우 직접 지정 가능 |
| ~~숫자 `1~6`만 인쇄해 OCR로 읽기~~ | `P1~P6`으로 표기 |
| ~~이전 카메라 구역과 점유 파일 재사용~~ | 새 구도의 카메라 영역을 다시 지정. 이전 기록은 `archives/`에 보관 |

점유 인식 설정은 [카메라 점유 설정](docs/TEAM_GUIDE.md#카메라-점유-설정)을 참고하세요. 테스트 명령은 `& $python -m unittest discover -s tests -v`입니다. 남은 팀별 작업은 [로드맵](docs/ROADMAP.md), 이전 구현과 실행 로그는 [인수인계 기록](docs/PROJECT_HANDOFF.md)에 있습니다.
