# Q-ROVER — AI Valet Parking Rover

> **Arduino UNO Q 기반 모형 주차장 자율 발렛파킹 프로젝트**  
> **현재 단계:** 지도 인식, 주차칸 선택, 다익스트라 경로 계산, 경로 PNG와 픽셀 waypoint JSON 생성까지 두 지도에서 검증

## 1. 프로젝트 소개

천장 고정 카메라와 **Arduino UNO Q(Q-ROVER)**를 이용해 모형 주차장의 주차 가능 공간을 파악하고, 사용자가 원하는 위치 조건에 맞는 주차칸을 선택한 뒤 로버가 해당 위치로 이동하도록 하는 프로젝트입니다.

전체 시스템은 다음 흐름을 목표로 합니다.

```text
주차장 배치도 이미지
    ↓ OpenCV 기반 시설/도로 검출
도로 중심선 및 가중치 그래프 생성
    ↓
건물·입구·출구·주차칸 정보가 포함된 의미 지도(JSON)
    ↓                     ↖ 향후 YOLO 점유 결과(별도 팀원)
조건에 맞는 주차칸 선택
    ↓ Dijkstra
최단 경로 및 픽셀 waypoint 생성·시각화
    ↓ 향후 실제 좌표 변환
천장 카메라 + ArUco 기반 위치 추정 → Q-ROVER 주행 제어 [예정]
```

현재 **지도 인식은 AI 학습이 아닌 OpenCV 색상·형태 규칙 기반**입니다. YOLO를 이용한 차량 검출 및 주차칸 점유 판정은 **YOLO 담당 팀원과 병렬 개발**합니다. LLM/VLM 연동은 핵심 기능을 완료한 뒤 여유가 있을 때 검토하는 **선택적 확장**입니다.

## 2. 기술 스택

| 영역 | 기술 | 현재 용도 |
| --- | --- | --- |
| 지도 이미지 처리 | Python, OpenCV, NumPy | HSV 색상 분할, 연결 영역 검출 |
| 도로 중심선 | scikit-image | Skeletonization(세선화) |
| 그래프 및 경로 탐색 | NetworkX | 픽셀 그래프, 가중치 그래프, 다익스트라 |
| 데이터 저장 | JSON | 노드, 간선, 주차 시설, 위치 힌트, 중심선 좌표 |
| 실물 하드웨어(예정) | Arduino UNO Q / Q-ROVER | 모터 제어, 경로 추종 |
| 위치 추정(예정) | 천장 카메라, ArUco, Homography | 픽셀 좌표→월드 좌표 변환 및 로버 위치 추정 |
| 점유 상태(별도 담당) | YOLO | 각 주차칸의 점유 여부 전달 |

**확인된 개발 환경:** Windows PowerShell, Python 3.12.2, `opencv-python`, `numpy`, `networkx`, `scikit-image`. 실제 설치 버전은 `requirements.txt` 및 로컬 환경을 기준으로 확인해 주세요.

## 3. 현재 구현된 기능

- **도로 추출:** HSV 색상 분할로 빨간 도로를 찾고, 가장 큰 연결 영역만 남겨 범례 등 분리된 빨간 영역을 제거합니다.
- **도로 중심선 생성:** 도로를 약 1픽셀 두께로 세선화하고 8-이웃 관계로 끝점·교차점 후보를 검출합니다.
- **가중치 그래프 생성:** 도로 끝점·교차점을 노드로, 중심선 구간을 간선으로 변환합니다. 간선마다 픽셀 단위 `weight`와 순서 있는 `path` 좌표를 저장합니다.
- **의미 지도 생성:** 일반/장애인 주차칸, 건물, 입구/출구를 색상과 영역 기준으로 검출합니다. 주차칸과 출입구는 도로 끝점에 연결하고, 건물은 별도 사각형 영역으로 저장합니다.
- **시설 이름 부여:** `parking_map_v1_labels.json`의 위치 힌트로 건물 A/B와 입구/출구를 식별합니다. OCR로 글자를 읽는 방식은 아닙니다.
- **주차칸 선택:** `entrance`, `exit`, `building_a`, `building_b` 네 가지 선호 조건으로 일반 주차칸 중 목적지를 선택합니다.
- **다익스트라:** 선택된 목적지까지 입구에서 이동할 그래프 노드 경로와 거리를 계산합니다.
- **경로 시각화:** 지도 위에 출발점·교차점·목적지를 표시하고, 같은 경로를 픽셀 waypoint JSON으로 저장합니다.

**후속 통합 항목**은 실제 cm 단위 좌표 변환, 실시간 주차 점유 반영, ArUco 위치 추정과 로버 주행 제어입니다.

## 4. 프로젝트 구조

아래는 대화에서 확인된 **현재 코드 구조 + 프로젝트 문서**입니다. 실제 저장소를 수정하기 전 로컬 파일을 먼저 확인해 주세요.

```text
ai-valet-parking-rover/
├── README.md                         # 프로젝트 개요 및 실행 안내
├── AGENTS.md                         # Codex 작업 지침
├── docs/
│   ├── TEAM_GUIDE.md                  # 팀 역할·브랜치·데이터 계약
│   ├── ROADMAP.md                     # 팀별 단계/진행 현황
│   └── PROJECT_HANDOFF.md             # 지도 코드 상세 이력·검증 로그
├── assets/
│   └── maps/
│       ├── parking_map_v1.png         # 입력 지도 이미지
│       └── parking_map_v1_labels.json # 건물/출입구 이름과 위치 힌트
├── src/
│   └── map_processing/
│       ├── map_reader.py              # 빨간 도로 마스크 추출
│       ├── graph_builder.py           # 세선화, 끝점·교차점 검출
│       ├── graph_exporter.py          # 그래프 생성 및 JSON/미리보기 저장
│       ├── semantic_mapper.py         # 시설 검출 및 의미 정보 연결
│       ├── parking_selector.py        # 조건별 주차칸 선택 및 다익스트라
│       └── path_visualizer.py         # 최단 경로 PNG와 픽셀 waypoint JSON 저장
├── data/
│   └── graphs/
│       ├── road_skeleton.png          # 세선화 확인 이미지
│       ├── graph_preview.png          # 노드·간선 미리보기
│       ├── parking_map_v1.json         # 도로 그래프
│       └── parking_map_v1_semantic.json # 의미 정보가 추가된 그래프
├── requirements.txt
└── .gitignore
```

## 5. 설치 및 실행

### 5.1 환경 준비 (Windows PowerShell)

프로젝트 **최상위 폴더**에서 실행합니다. 처음 가져온 팀원은 아래 명령을 사용하면 됩니다.

```powershell
git clone https://github.com/isthisit123/ai-valet-parking-rover.git
cd ai-valet-parking-rover
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```

이미 프로젝트와 가상환경이 있다면 해당 폴더에서 `.\.venv\Scripts\Activate.ps1`만 실행하면 됩니다. Python 패키지/모듈을 확실히 같은 가상환경으로 실행하려면 활성화 후 `python -m ...` 형식을 권장합니다.

### 5.2 실행 순서

```powershell
# 1. 도로 중심선 및 그래프 생성
python -m src.map_processing.graph_exporter

# 2. 주차칸·건물·입출구 의미 정보 추가
python -m src.map_processing.semantic_mapper

# 3. 조건별 주차칸 선택 및 다익스트라 경로 계산
python -m src.map_processing.parking_selector

# 4. 네 조건의 경로 PNG와 waypoint JSON 저장
python -m src.map_processing.path_visualizer
```

각 단계의 입출력은 다음과 같습니다.

| 단계 | 입력 | 생성·출력 |
| --- | --- | --- |
| `graph_exporter` | `assets/maps/parking_map_v1.png` | `data/graphs/parking_map_v1.json`, `graph_preview.png` |
| `semantic_mapper` | 지도 이미지, 그래프 JSON, `parking_map_v1_labels.json` | `data/graphs/parking_map_v1_semantic.json` |
| `parking_selector` | 의미 지도 JSON | 선택 주차칸, 조건 점수, 노드 경로, 입구부터의 거리(터미널) |
| `path_visualizer` | 지도 이미지, 의미 지도 JSON | `data/graphs/routes/`의 조건별 PNG와 waypoint JSON |

현재 파이프라인은 상대 import를 사용하므로 `.py` 파일을 개별 경로로 직접 실행하기보다 **`python -m src.map_processing.<모듈명>`** 형식으로 실행합니다.

특정 조건만 코드에서 시험하려면:

```python
from src.map_processing.parking_selector import select_parking

result = select_parking("building_a")
print(result)
```

경로 한 개만 저장하려면 `python -m src.map_processing.path_visualizer --preference exit`를 실행합니다. 출력 이름은 `<map_id>_route_<preference>.png`와 `<map_id>_route_<preference>.json`입니다. 다른 지도를 처리할 때는 그래프 생성, 의미 지도 생성, 시각화 명령에 같은 `--map-id`와 `--output-dir`을 전달합니다.

경로 JSON은 지도·주차칸 ID, 노드 순서, 그래프 이동 거리인 `total_distance`와 픽셀 `waypoints`를 담습니다. 좌표계는 `map_image`이며 좌상단 원점에서 x는 오른쪽, y는 아래로 증가합니다. 경로는 입구 도로 노드에서 선택 주차칸에 연결된 도로 노드까지 이어집니다. 두 좌표 사이 간격이 1픽셀보다 큰 축에서는 선분을 따라 점을 채우고, 각 점이 검출된 도로 안에 있는지 검사합니다.

중간 처리 이미지를 확인할 때:

```powershell
# 도로 마스크 미리보기
python -m src.map_processing.map_reader

# 도로 중심선·노드 검출 미리보기
python -m src.map_processing.graph_builder
```

미리보기 창은 아무 키나 누르면 종료됩니다. `graph_builder`는 `road_skeleton.png`도 저장합니다.

## 6. 지도 이미지 작성 규칙 및 좌표계

현재 지도 분석기는 일정한 색상 및 도형 규칙을 전제로 합니다.

| 시설 | 표현 규칙 |
| --- | --- |
| 도로 | 빨간색 선 또는 영역 |
| 일반 주차칸 | 초록색 바탕, `P` 표기 |
| 장애인 주차칸 | 밝은 파란색 바탕, `P` 표기 |
| 건물 | 어두운 회색/검은색 사각형 |
| 입구·출구 | 밝은 회색 사각형 |

기본 지도는 초록색 일반 주차칸을 사용합니다. 기존 테스트 지도(`--map-id parking_map_v1_test`)는 흰색 주차칸 설정으로 실행합니다. HSV와 최소 영역 면적은 `src/map_processing/map_config.py`에서 관리합니다.

건물 A/B 및 입구/출구의 이름은 `assets/maps/parking_map_v1_labels.json`에서 `id`와 `hint`를 지정해 매칭합니다. `hint`는 **이미지 좌상단 원점의 `[x, y]` 픽셀 좌표**로, 정확한 건물 모서리 좌표가 아니라 시설을 구별하기 위한 위치 힌트입니다.

현재 데이터 단위는 **`pixel`**입니다. 지도상의 `weight`도 도로 중심선을 따른 픽셀 거리이므로 **실제 cm/m 거리로 해석하면 안 됩니다.** 실물 제작 후 카메라 캘리브레이션·월드 좌표계 정의·Homography를 적용하고, 변환된 좌표에서 간선 길이를 다시 계산할 예정입니다.

### 다른 지도 테스트

- 테스트 지도는 `--map-id parking_map_v1_test`로 실행합니다. 결과는 `data/graphs/parking_map_v1_test/` 아래에 저장됩니다.
- 지도의 배치·크기가 바뀌면 해당 라벨 JSON의 위치 힌트를 갱신한 뒤 그래프를 다시 생성합니다.

## 7. 검증 결과

2026년 11월 기준, **초록색 일반 주차칸을 사용하는 기본 지도와 흰색 일반 주차칸을 사용하는 테스트 지도**에서 기본 기능을 실행해 확인했습니다.

| 검증 항목 | 원본 지도 | 두 번째 지도 |
| --- | --- | --- |
| 그래프 노드 | 12 | 12 |
| 그래프 간선 | 11 | 11 |
| 그래프 연결 | `True` | `True` |
| 일반/장애인 주차칸 | 4 / 2 | 4 / 2 |
| 건물/입출구 영역 | 2 / 2 | 2 / 2 |
| 입구 노드 / 출구 노드 | `N4` / `N3` | `N3` / `N4` |
| 조건별 다익스트라 실행 | 성공 | 성공 |

실행 결과 예시(두 번째 지도, `building_b` 조건):

```text
Preference: building_b
Selected: N5
Score: 144.0
Route: N3 -> N10 -> N11 -> N5
Route distance: 833.95
```

두 지도에서 네 가지 조건의 경로 PNG와 JSON을 각각 생성하고, waypoint가 도로 영역 안에 있으며 인접 점 사이가 최대 대각선 1픽셀 간격인지 확인했습니다. 복잡한 곡선·고리형 도로·밀집 교차점은 추가 검증이 필요합니다.

## 8. 개발 로드맵

상세 체크리스트와 완료 기준은 **[docs/ROADMAP.md](docs/ROADMAP.md)**, 팀원별 책임은 **[docs/TEAM_GUIDE.md](docs/TEAM_GUIDE.md)**를 참고합니다. 지도·YOLO·로버 제어는 일부 작업을 **병렬 진행**합니다.

| 작업 영역 | 다음 주요 목표 | 상태 |
| --- | --- | --- |
| 지도·경로 | 픽셀 경로에서 실측 좌표로 변환 | 경로 시각화와 픽셀 waypoint 기본 지도 2종 검증 |
| YOLO 점유 인식 | 모델 학습·검증 → `parking_space_id`별 `free/occupied/unknown` 출력 | 팀원 담당 / 완료 여부 미확인 |
| 로버 주행 | UNO Q 모터·ToF 제어 → waypoint 추종 및 안전 정지 | 팀원 담당 / 완료 여부 미확인 |
| 실물·카메라 | 천장 카메라 보정 → ArUco → 공통 월드 좌표 | 협업 진행 예정 |
| 시스템 통합 | 점유 상태 반영해 최적 빈칸 선택 → 실물 주행·도착 판정 | 후속 통합 |
| 선택 확장 | OCR/Segmentation/VLM/LLM·칼만 필터 | 필수 기능 이후 |

**지도·경로 담당 모듈의 실물 통합 이전 1차 완료 기준**은 지도에서 시설·도로를 분석하여 의미 그래프를 생성하고, 선택 조건에 따른 최단 경로를 **시각화 및 연속 waypoint 데이터로 제공**하는 것입니다.

## 9. 현재 한계 및 개선 계획

- **검출 규칙 의존:** 기존 지도에서 과다 검출을 줄이기 위해 연결 영역의 색상 픽셀 비율(최소 60%), 파란색 HSV 밝기 하한(150), 파란 영역 최소 면적(3,000px) 등의 보정을 적용했습니다. 해상도/색상/지도 형태 변화에 대한 일반화는 추가 검증이 필요합니다.
- **도로/교차점 처리:** `31×31` 교차점 병합, 노드 주변 반경 등 경험적인 설정을 사용합니다. 밀접 교차점, 루프, 짧은 가지, 연결이 끊긴 지도에서는 오류가 날 수 있습니다.
- **경로 좌표 연속성:** 픽셀 경로를 내보낼 때 노드 주변 간격을 직선으로 보간하고 도로 영역을 검사합니다. 실물 주행에는 월드 좌표 변환과 회전반경 검증이 필요합니다.
- **주차칸 고유 ID:** 주차칸은 `(map_id, parking_space_id)`로 식별합니다. `N0`, `N1` 등의 그래프 노드 ID는 지도나 그래프를 다시 생성할 때 달라질 수 있습니다.
- **점유 정보:** 모든 일반 주차칸이 비었다는 **테스트 가정**으로 선택합니다. YOLO 결과는 아직 반영하지 않았습니다. 장애인 주차칸은 일반 주차 조건에서 제외합니다.
- **거리 의미 차이:** 출입구까지는 그래프 **도로 최단거리**, 건물까지는 건물 사각형 외곽의 **직선거리**를 사용합니다. 실제 출입문까지의 보행 거리는 아직 모델링하지 않았습니다.
- **실물 좌표:** 월드 좌표와 로버 `(x, y, θ)` 추적/주행 제어는 아직 구현 전입니다.

## 10. 협업 및 문서

**GitHub 저장소:** <https://github.com/isthisit123/ai-valet-parking-rover>  
**팀원 공통 시작 문서:** **[docs/TEAM_GUIDE.md](docs/TEAM_GUIDE.md)**  
**Codex 공통 지침:** [AGENTS.md](AGENTS.md)

### 기능별 역할

| 담당 | 개발 범위 | 브랜치 예시 |
| --- | --- | --- |
| 지도·경로 | 의미 지도, 주차칸 고유 ID, 최단 경로, 시각화/waypoint | `feature/map-to-graph`, 후속 기능별 브랜치 |
| YOLO 점유 인식 | 데이터 수집·학습, 차량/주차칸 인식, ID별 점유 상태 | `feature/parking-detection` |
| 로버 주행 | UNO Q 모터·ToF 제어, 경로 추종, 안전 정지 | `feature/rover-control` |
| 팀 공통 | 카메라·ArUco, Homography, 인터페이스, 통합 시연 | 협의한 기능 브랜치 |

협업자는 초대 수락 후 저장소를 Clone하고, `main` 최신 상태에서 **자기 기능 브랜치**를 생성합니다. 변경 후 테스트·커밋·Push하고 **Pull Request로 `main`에 병합**하는 것을 권장합니다. 다른 담당자의 파일을 임의로 대폭 수정하거나 로컬 변경을 삭제하지 않습니다.

### 팀 간 데이터 연결(협의용)

1. **지도 팀 → YOLO 팀:** `(map_id, parking_space_id)`, 주차칸 bbox/중심, 연결 도로 노드. `P01`은 영구 주차칸 ID이고 `N2`는 변동 가능한 그래프 노드입니다.
2. **YOLO 팀 → 경로 선택:** 주차칸별 `free / occupied / unknown` 상태, 관측 시각, 신뢰도. `unknown`과 오래된 데이터는 빈칸으로 단정하지 않습니다.
3. **경로 팀 → 로버 팀:** 선택한 주차칸 ID, 월드 좌표 프레임/단위가 명시된 연속 waypoint. 현재 생성된 그래프의 `pixel` 거리 데이터를 실제 cm로 사용하면 안 됩니다.

데이터 계약의 상세 **JSON 예시와 담당별 완료 기준**은 `TEAM_GUIDE.md`에 있습니다. 문서의 계약 예시는 아직 합의·구현이 완료된 API가 아닙니다.

### 문서 역할

- **[README.md](README.md)** — 전체 목표, 실제 구현 상태, 지도 규칙, 설치·실행법
- **[docs/TEAM_GUIDE.md](docs/TEAM_GUIDE.md)** — 팀원 역할, 협업 절차, 모듈 연결 JSON 계약 초안
- **[docs/ROADMAP.md](docs/ROADMAP.md)** — 지도/YOLO/로버/실물/통합의 병렬 개발 로드맵
- **[docs/PROJECT_HANDOFF.md](docs/PROJECT_HANDOFF.md)** — 지도 처리 구현·과거 실행 로그·설계 이력
- **[AGENTS.md](AGENTS.md)** — 팀 전체의 Codex 개발·Git 안전 규칙

> **주의:** 2026-11-09 이전 지도 코드 기록과 이후 팀 협업 계획은 구분해야 합니다. Codex는 작업 시작 전에 반드시 실제 로컬 Git 상태/소스/JSON을 확인하고, 문서에만 근거하여 완료를 주장하거나 다른 팀원의 파일을 덮어쓰지 마세요.
