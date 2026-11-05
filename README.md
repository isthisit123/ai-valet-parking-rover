# AI Valet Parking Rover

컴퓨터 비전과 경로 탐색을 이용하는 자율 발렛 주차 로버 프로젝트입니다.
현재는 주차장 배치도 이미지에서 도로와 시설을 추출하고, 조건에 맞는 주차칸과 입구에서의 이동 경로를 계산하는 단계까지 구현했습니다.

## 구현된 기능

- **도로 추출:** OpenCV의 HSV 색상 분할로 빨간색 도로를 검출하고, 가장 큰 연결 영역만 남겨 분리된 범례를 제거합니다.
- **도로 중심선 생성:** 도로를 1픽셀 두께로 세선화하고 끝점과 교차점을 검출합니다.
- **도로 그래프 생성:** 끝점과 교차점을 노드로, 도로 구간을 간선으로 변환합니다. 간선에는 도로 중심선을 따른 거리와 경로 좌표를 저장합니다.
- **의미 지도 생성:** 일반 주차칸, 장애인 주차칸, 건물, 출입구를 색상으로 구분합니다. 주차칸과 출입구는 가까운 도로 끝점에 일대일로 연결하고, 건물은 별도 영역으로 저장합니다.
- **시설 이름 부여:** 위치 힌트로 건물 A/B와 입구/출구를 구분합니다.
- **조건별 주차 선택:** 출구, 입구, 건물 A 또는 건물 B에 가까운 일반 주차칸을 선택합니다.
- **최단 경로 계산:** Dijkstra 알고리즘으로 입구에서 선택한 주차칸까지의 경로와 이동 거리를 계산합니다.

## 실행 환경

- Windows PowerShell
- Python 3.12.2에서 실행 확인
- 주요 라이브러리: OpenCV, NumPy, NetworkX, scikit-image

최초 설치 시 프로젝트 폴더에서 실행합니다.

```powershell
cd C:\Users\7008j\Desktop\ai-valet-parking-rover
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```

이미 가상환경을 설치했다면 폴더 이동과 활성화만 하면 됩니다.

```powershell
cd C:\Users\7008j\Desktop\ai-valet-parking-rover
.\.venv\Scripts\Activate.ps1
```

## 실행 순서

모든 명령은 프로젝트 최상위 폴더에서 실행합니다. 모듈 간 상대 import를 사용하므로 `python -m ...` 형식으로 실행합니다.

### 1. 도로 그래프 생성

```powershell
python -m src.map_processing.graph_exporter
```

원본 지도에서 도로 추출, 세선화, 노드 검출, 그래프 생성을 수행합니다.

생성 파일:

- `data/graphs/parking_map_v1.json`: 도로 노드, 간선, 거리, 경로 좌표
- `data/graphs/graph_preview.png`: 노드가 표시된 도로 그래프 이미지

### 2. 의미 지도 생성

```powershell
python -m src.map_processing.semantic_mapper
```

도로 그래프에 주차칸, 건물, 입구, 출구 정보를 추가하고 `data/graphs/parking_map_v1_semantic.json`에 저장합니다.

현재 제공된 `parking_map_v1.png`에서 확인한 검출 결과:

```text
Normal spaces detected: 4
Accessible spaces detected: 2
Buildings detected: 2
Gate regions detected: 2
```

일반 주차칸 4개, 장애인 주차칸 2개, 출입구 2개가 서로 다른 8개의 도로 끝점에 연결됩니다. 건물 2개는 도로 노드와 별도로 저장됩니다.

### 3. 조건별 주차 선택 및 경로 확인

```powershell
python -m src.map_processing.parking_selector
```

현재 실행 명령은 아래 네 가지 조건을 차례로 확인합니다.

| 조건 값 | 주차칸 선택 기준 |
| --- | --- |
| `exit` | 주차칸에서 출구까지의 도로 최단 거리가 가장 짧은 곳 |
| `entrance` | 주차칸에서 입구까지의 도로 최단 거리가 가장 짧은 곳 |
| `building_a` | 주차칸에 연결된 노드에서 건물 A의 사각형 영역까지 직선거리가 가장 짧은 곳 |
| `building_b` | 주차칸에 연결된 노드에서 건물 B의 사각형 영역까지 직선거리가 가장 짧은 곳 |

각 조건에서 선택한 노드(`Selected`), 조건에 대한 거리 점수(`Score`), 입구부터의 경로(`Route`), 입구부터의 이동 거리(`Route distance`)를 출력합니다.
거리 단위는 이미지의 **픽셀**이며, 실제 미터 단위로 환산하는 기능은 아직 없습니다.

코드에서 조건 하나만 사용하려면 다음과 같이 호출합니다.

```python
from src.map_processing.parking_selector import select_parking

result = select_parking("building_a")
print(result)
```

### 선택 사항: 중간 처리 결과 확인

```powershell
# 빨간색 도로 마스크 확인
python -m src.map_processing.map_reader

# 도로 중심선 및 끝점/교차점 확인
python -m src.map_processing.graph_builder
```

두 명령은 OpenCV 미리보기 창을 엽니다. 창에서 아무 키나 누르면 종료됩니다.
`graph_builder`는 `data/graphs/road_skeleton.png`도 저장합니다.

## 프로젝트 구조

```text
ai-valet-parking-rover/
├── assets/maps/
│   ├── parking_map_v1.png                # 원본 주차장 배치도
│   └── parking_map_v1_labels.json        # 건물 및 출입구의 ID와 위치 힌트
├── data/graphs/
│   ├── road_skeleton.png                # 중심선 확인용 이미지
│   ├── graph_preview.png                # 그래프 확인용 이미지
│   ├── parking_map_v1.json               # 도로 그래프
│   └── parking_map_v1_semantic.json      # 시설 및 주차칸 정보가 추가된 그래프
├── src/map_processing/
│   ├── map_reader.py                    # 도로 색상 마스크 추출
│   ├── graph_builder.py                 # 세선화 및 끝점/교차점 검출
│   ├── graph_exporter.py                # 그래프 생성 및 저장
│   ├── semantic_mapper.py               # 시설 검출 및 노드 연결
│   └── parking_selector.py              # 주차 조건 평가 및 최단 경로 계산
├── requirements.txt
└── README.md
```

## 지도와 라벨 설정

현재 처리 대상은 `assets/maps/parking_map_v1.png`입니다.

| 지도 요소 | 현재 사용하는 색상 |
| --- | --- |
| 도로 | 빨간색 |
| 일반 주차칸 | 흰색 |
| 장애인 주차칸 | 밝은 파란색 |
| 건물 | 어두운 회색 또는 검은색 |
| 입구·출구 | 밝은 회색 |

`assets/maps/parking_map_v1_labels.json`의 `id`와 `hint`로 건물과 출입구의 이름을 지정합니다.
`hint`는 이미지 왼쪽 위를 원점으로 하는 `[x, y]` 픽셀 좌표이며, 각 힌트에 가까운 아직 사용하지 않은 검출 영역에 이름을 부여합니다. 이미지의 글자를 읽어 이름을 판단하는 방식은 아닙니다.

지도의 위치나 크기를 변경하면 위치 힌트도 함께 확인하고, 도로 그래프 생성부터 의미 지도 생성까지 다시 실행해야 합니다. 생성 명령은 같은 이름의 결과 파일을 덮어씁니다.

## 현재 검출 보정과 한계

현재 지도에서 발생한 과다 검출은 다음 기준으로 보정했습니다.

- 연결 영역의 실제 색상 픽셀 수가 경계 사각형 면적의 60% 미만이면 제외하여 가느다란 테두리 조각을 제거합니다.
- 파란색 마스크의 HSV 밝기 최솟값을 150으로 지정하여 어두운 청회색 건물이 장애인 주차칸으로 검출되는 것을 줄입니다.
- 파란색 영역의 최소 면적을 3,000픽셀로 지정하여 하단 범례 아이콘을 제외합니다.
- 연결할 주차칸과 출입구 영역 수가 도로 끝점 수보다 많으면, 연결 전에 두 개수를 포함한 오류를 표시합니다.

이 기준은 현재 지도에서 검증한 값입니다. 색상, 해상도, 배경 또는 배치가 달라진 지도에 대한 일반화는 아직 완료하지 않았습니다.

## 다음 개발 항목

- 건물과 주차칸이 명확히 구분되도록 지도 색상 규칙 정리
- 지도 크기와 색상 변화에 대응하도록 검출 기준과 설정 일반화
- YOLO 등으로 주차칸 점유 여부를 확인하고 빈 주차칸 선택에 반영
- 실제 로버 주행 제어 및 주차 동작 연동
- 주요 기능 완성 후 자연어 조건 처리를 위한 LLM 연동 검토

현재 주차 선택은 **모든 일반 주차칸이 비어 있다고 가정**하며, 장애인 주차칸은 검출·저장만 수행하고 선택 후보에서는 제외합니다.
