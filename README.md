# Q-ROVER — AI Valet Parking Rover

지도 사진에서 주차칸과 도로를 찾고, 빈칸까지 경로를 계산해 Q-ROVER에 전달합니다. 현재 **로컬 PNG → OCR 힌트 → 의미 지도 → cm 경로 미리보기**를 검증했습니다. 실제 카메라 점유 연동과 로버 주행은 [로드맵](docs/ROADMAP.md)에 남아 있습니다.

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

## cm 경로 미리보기

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

[localhost:8001](http://localhost:8001)에서 **현재 화면 PNG 전송 · 맵 생성**을 누릅니다. 서버는 `POST /api/map-capture`로 원본 PNG를 받아, 기존 픽셀 힌트로 그래프·의미 지도·보고서를 `data/http_map_captures/<capture_id>/`에 저장합니다. 기준 사진과 같은 카메라 위치·줌·크롭이 필요합니다.

HTTP의 현재 범위는 **기존 힌트로 의미 지도 생성까지**입니다. 새 캡처의 OCR 힌트 생성과 cm 경로 생성은 로컬 명령에 구현돼 있고, 웹 버튼에는 연결할 예정입니다.

## 대체된 작업 방식

| 이전 | 현재 |
| --- | --- |
| ~~`parking_map_v1.png`를 기본 지도로 사용~~ | `parking_map_real.png`와 OCR 라벨을 사용 |
| ~~새 사진마다 시설별 픽셀 힌트를 손으로 입력~~ | `scripts.generate_map`으로 자동 생성. 픽셀 `hint`와 실측 `hint_cm`은 필요한 경우 직접 지정 가능 |
| ~~숫자 `1~6`만 인쇄해 OCR로 읽기~~ | `P1~P6`으로 표기 |
| ~~이전 카메라 구역과 점유 파일 재사용~~ | 새 구도의 카메라 영역을 다시 지정. 이전 기록은 `archives/`에 보관 |

점유 인식 설정은 [카메라 점유 설정](docs/TEAM_GUIDE.md#카메라-점유-설정)을 참고하세요. 테스트 명령은 `& $python -m unittest discover -s tests -v`입니다. 남은 팀별 작업은 [로드맵](docs/ROADMAP.md), 이전 구현과 실행 로그는 [인수인계 기록](docs/PROJECT_HANDOFF.md)에 있습니다.
