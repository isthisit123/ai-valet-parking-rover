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

### HTTP 지도에서 웹 구역 자동 등록

지도 생성 패널의 렌즈 X·Y·Z와 마커 높이는 PNG 요청의 선택 헤더 `X-Rover-Camera-Position`으로 전달합니다. 값은 `{"camera_position_cm":[53,151.7,111.5],"marker_height_cm":10.5}` 형식입니다. 전체 생략 시 기존 지도 API 동작을 유지합니다. 유한 숫자 3개, 0 이상 마커 높이, Z>마커 높이를 검사하고 현재 웹캠 세션·유효한 원본 크기 및 서버 `--marker-layout`이 있어야 등록할 수 있습니다.

cm 검증과 지도·경로 게시가 모두 성공하면 응답 `camera_position.status: saved` 및 `files.camera_position`을 추가하고 `world/camera_position.json`을 같은 폴더 교체 작업으로 게시합니다. 최종 캡처 보고서·보정 파일 해시에 묶으므로 기존 위치 전송기가 그대로 읽습니다. 검증 실패 시 `camera_position.status: not_saved`와 이유를 표시하고 설정을 생성하지 않습니다. 기존 ParkingHaeven 원본·YOLO 모델·픽셀 좌표 API는 변경하지 않습니다.

결과 게시의 Windows 디렉터리 잠금(5/32/33)은 최대 6회 재시도합니다. 지속 실패 시 이전 지도를 복원하며, 복원까지 실패하면 임시 빌드 폴더 외부의 `.previous-<고유값>`에 이전 결과를 보존하고 위치를 오류로 알립니다. 새 지도 게시 뒤 백업 정리만 실패한 경우 응답의 선택 필드 `cleanup_warning`과 서버 로그로 알립니다. 복원 실패 백업을 실행 산출물 정리 대상으로 자동 삭제하지 않습니다.

실물 지도에서 OCR 힌트 수와 색상 주차칸 수가 다르면 공통 의미 지도 생성 단계가 같은 PNG의 어두운 영상 기준으로 주차칸만 한 번 재검출합니다. 기존 OCR 결과·ID·장애인석 유형·검은 시설 검출은 유지하고, 각 힌트가 동일 유형 영역 하나에 유일하게 매칭되는지 검사합니다. 성공한 의미 지도에는 선택 필드 `parking_detection_profile: low_light_retry`를 추가합니다. 실패 시 6개를 임의로 채우지 않으며 기존 게시 지도를 보존합니다.

기본 OCR 지도 생성은 `semantic.json`의 주차칸에 `polygon_px: [[x,y], ...]`를 추가합니다. 기존 `bbox`, `center`, `area`, 시설·칸 ID와 도로 연결 계약은 유지합니다. 글자 내부 구멍을 제외한 색상 영역의 볼록 외곽을 단순화하며, 네 점 근사가 외곽 면적의 93% 이상을 보존할 때 사각형을 사용합니다. 그 외에는 외곽의 추가 꼭짓점을 유지합니다. cm 지도에서는 같은 점을 변환한 `polygon_cm`을 사용하고 `polygon_px`를 섞지 않습니다.

`POST /api/map-capture`의 추가 응답 `parking_regions`와 고정 `parking_regions.json`은 `schema_version: "1.0"`, `capture_id`, 지도 식별자 3개, `source_sha256`, `camera_source`, `image_size_px`, `coordinate_frame: "capture_image"`, `unit: "pixel"`, 왼쪽 위 원점·오른쪽/아래 축, `regions`를 포함합니다. 구역은 `{id:"P1", parking_space_id:"space_1", type:"normal", points:[{x,y},...], occupied:null, occupancy_status:"unknown"}` 형식입니다. 외곽 내보내기 실패는 `parking_regions.status: "failed"`로 구분하고 지도·경로 결과는 보존합니다. 구버전 JSON 소비자는 새 필드를 무시할 수 있습니다.

수신 웹은 정상 게시된 동일 응답 안의 여섯 칸과 식별자를 검증한 뒤 현재 캔버스 픽셀로 크기를 맞춥니다. `PARKING_SYSTEM_CONFIG`의 현재 소스 키에만 저장하며 다른 소스 설정은 유지합니다. 촬영 시점과 적용 시점의 소스·카메라 세션·원본 크기·종횡비가 달라지면 적용을 중단합니다. 웹 설정에는 지도 식별자와 `image_size_px`, `coordinate_frame: "parking_heaven_canvas"`를 함께 저장합니다. 고정 URL의 과거 결과를 다시 읽어 자동 적용하지 않습니다. 이후 창 크기나 카메라 구도가 바뀌면 다시 맵을 생성하고 외곽을 확인하세요.

자동 등록은 구역 설정만 완료합니다. 웹은 설정 모드로 전환하고 빈칸 통계는 미확인으로 표시합니다. 저장된 boolean·좌표 내보내기만으로 최신 빈칸을 보장하지 않습니다. 상단 **모드 선택 → 감시 모드**에서 차량 검출을 시작하면 아래 실시간 브리지가 관측 시각·지도 식별자·카메라 세션을 붙여 픽셀 도착 테스트에 전달합니다. 점유 모듈의 `free/occupied/unknown` 규약은 유지합니다.

### 웹 점유와 픽셀 도착 테스트의 실시간 연결

`scripts/parking_occupancy_bridge.js`는 기존 `YOLODetector.detect/postprocess/checkParkingOccupancy` 결과를 재사용합니다. 성공한 postprocess와 오류 반환 `[]`를 구별하고, 모델 오류·다른 구역/소스·캔버스 크기 변경은 빈칸으로 보내지 않습니다. 브라우저의 한 추론을 추가로 실행하지 않습니다. 동일 상태라도 새 추론 결과가 나올 때마다 전송하고 전송 중 중첩은 방지합니다. 전송이 2초 안에 끝나지 않으면 요청을 해제해 다음 새 관측을 보낼 수 있게 합니다. 과거 관측으로 heartbeat를 만들지 않습니다. 감시 중단·통신 유실 시 서버는 TTL로 무효화합니다.

`aruco-result`의 원본 카메라 픽셀 관측을 최대 64개·5초 동안 보관하고, YOLO 입력의 **촬영 시각**에서 가장 가까운 ±150ms 관측과 비교합니다. 추론 완료 후의 최신 위치를 그대로 쓰지 않습니다. 관측 당시 유효한 `detected`·설정된 로버 ID·동일 카메라 세션/종류/원본 크기를 요구하고, 네 모서리를 YOLO 캔버스로 비율 변환합니다. 마커 전체를 포함하고 마커 면적의 30배 이하이며 마커 중심이 박스 내부 10~90%에 있는 후보가 정확히 하나일 때만 그 검출을 로버로 제외합니다. 더 가까운 lost/stale 관측·ID 불일치·부분 겹침·큰 박스·복수 후보에는 제외하지 않습니다. 남은 모든 차량 검출로 기존 점유 계산을 다시 수행해 같은 칸의 다른 차량은 보존합니다. 화면 표시도 같은 필터 결과를 사용합니다.

스냅샷의 `occupancy_scope: other_vehicles_excluding_controlled_rover`는 제어 중인 로버를 제외한 다른 차량 기준임을 명시합니다. `rover_exclusion`은 적용 상태·제외 수·마커 ID/프레임/촬영 시각·시각 차이·캔버스 좌표·제외 박스를 기록하고 GET 점유 상태에도 전달합니다. 일반 차량 전체의 공용 빈칸 정보와 이 로버의 계획용 후보 정보를 스코프로 구분합니다. 모델·카메라·추론을 추가로 실행하지 않습니다.

일시적인 lost 프레임은 양쪽 ±150ms에 실제로 읽은 동일 로버 ID가 있고, 두 관측의 모서리가 같은 유일한 차량 박스 안에 있으며 이동량이 마커 한 변의 약 절반 이내일 때만 본체 분류를 유지합니다. `association_method: bracketed_marker_observations`와 지원 프레임 ID로 구분합니다. 한쪽 관측뿐이거나 ID 중복·빠른 이동·박스 모호함에는 적용하지 않습니다. 이 처리는 차량 역할의 연결에만 사용하며 누락 프레임의 ArUco 현재 좌표나 도착 관측을 생성하지 않습니다.

차량 검출의 최솟값은 **0.70(포함)**입니다. 서버는 `/js/YOLODetector.js`에 인터페이스가 같은 Worker 프록시를 제공하고, `/rover-original-yolo.js`로 기존 소스의 전후처리를 재사용합니다. 모델 추론은 `/parking-detection-worker.js` 안에서 한 번씩 수행하며 WASM 스레드는 하나를 사용합니다. 입력/출력 Tensor를 작업 종료 때 정리하고 모델 오류·누락 출력을 성공한 빈 프레임으로 반환하지 않습니다. 웹의 500ms 반복 호출도 이전 표시·추론이 끝나기 전이면 건너뜁니다. 원본 ParkingHaeven 소스를 덮어쓰거나 메인 스레드 추론으로 자동 대체하지 않습니다. 브라우저가 Worker를 로드하지 못하면 초기화 오류를 해결한 뒤 다시 시험해야 합니다.

`POST /api/parking-occupancy`는 기존 `schema_version: "0.1-draft"` 점유 규약에 필수 `capture_id`, `camera_source`를 추가합니다. 지도 식별자 3개와 관측 시각도 필수입니다. `spaces`의 상태는 `free/occupied/unknown`이며 누락은 unknown입니다. 웹 boolean을 명시적 상태로 변환할 때 `confidence: 0.5`를 소비 기준값으로 사용하며 측정된 모델 정확도/차량 검출 확률을 뜻하지 않습니다. 서버는 과거·미래·역순·중복 관측과 다른 지도/세션을 거부합니다. 최신 스냅샷은 `data/rover_runs/occupancy/latest.json`에 원자적으로 갱신합니다. GET은 원시 상태 `raw_states`와 연속 확인된 `states`, `observed_at`, TTL·나이·fresh를 반환합니다.

픽셀 테스트도 공통 기본 **5초 TTL**을 적용합니다. 촬영 시각으로부터 만료시키며 수신/조회 시각으로 오래된 프레임의 수명을 늘리지 않습니다. free는 최소 3개의 새 관측이 촬영 시각 기준 2초 이상에 걸쳐 판정돼야 후보로 인정하고 occupied/unknown은 즉시 반영합니다. 연속성 검사는 `(이번 촬영 시각 - 이전 촬영 시각)`으로 수행해 추론 시간을 공백에 더하지 않습니다. 새 관측 없이 대기한 시간은 확인 기간에 포함하지 않습니다. 실제 관측 공백이 TTL을 넘으면 free 확인을 다시 시작합니다. ArUco 위치 만료는 별도 200ms를 유지합니다. 저장 파일 소비자도 관측 시각·TTL·현재 지도 식별자를 확인해야 합니다.

웹 조건/칸 목록은 고유 ID 목록이 바뀔 때만 항목을 생성합니다. 같은 ID에 대한 상태 변화에서는 기존 option·선택값을 유지하며 조건 이름은 목표와 분리해 고정 표시합니다. 칸 목록에 포커스가 있으면 표시 글자 갱신은 blur까지 보류하되, 최신 서버 상태에 따른 시작 차단은 계속 적용합니다. 점유 연결 표시는 서버 상태를 기준으로 통일하고 관측 나이·만료 시간을 함께 표시합니다.

`ArrivalTest`는 HTTP 서버에서 항상 `LiveParking`을 연결합니다. 조건별 후보는 기존 `parking_selector`의 순위와 유형 제한을 유지하며 최신 free만 넘깁니다. 시작 요청의 `parking_space_id`(생략/null이면 자동)와 `auto_retarget`(기본 true)이 추가됐습니다. 직접 지정한 occupied는 거부하고 안내하지 않습니다. 초기 경로는 유효한 ArUco 관측 후 현재 위치를 가장 가까운 도로 간선 픽셀에 투영해 시작합니다. 그 간선을 두 조각으로 나누고 평행 간선·방향·도로 마스크를 검증한 경로를 사용합니다. 카메라 실측값이 등록된 경우 높이 보정한 바닥 기준 위치를 사용하며, 미등록 시 상단 마커의 영상 좌표임을 표시합니다.

도착 목표는 **칸 중앙**입니다. `arrival_geometry.arrival_circle`이 `polygon_px`의 면적과 기하 중심을 사용하고, 구버전 지도에 외곽이 없으면 bbox를 사용합니다. 요청 `arrival_area_ratio`는 기본 0.5, 허용 0.1~0.75입니다. `sqrt(칸 면적 × 비율 / π)`로 반지름을 구하고, 중심부터 가장 가까운 변까지 거리로 제한해 원 전체를 칸 안에 둡니다. `arrival_region`은 `center_px`, `radius_px`, `parking_area_px2`, `requested_area_ratio`, `actual_area_ratio`, `max_radius_px`, `geometry_source`를 포함합니다. 화면과 서버가 같은 중심·반지름을 사용하며, 원 안에서 처음 관측한 위치와 최소 2초 뒤 새 유효 위치가 안정 반경 이내로 일치하면 판정합니다. 중간 이동·원 이탈 관측은 기준점을 초기화합니다. 목적지 변경 시 새 칸의 중심·반지름·도착 유지 시간을 함께 갱신합니다. HTTP 실시간 모드에서는 이전 요청의 `radius_px` 대신 면적 비율로 반지름을 결정합니다.

실시간 상태의 `target_reference: parking_space_center`, `waypoint_target: parking_center`, `goal_px`는 칸 중앙을 가리킵니다. `road_goal_px`와 앞의 `road_waypoint_count`개 좌표는 기존 도로 검증 구간입니다. `parking_approach_px`는 그 진입점부터 중앙까지 이어 붙인 직선 픽셀 미리보기(`parking_approach_mode: straight_line_preview`)입니다. 이 마지막 구간에는 도로 마스크 조건을 적용하지 않으며 실제 충돌 회피·회전반경·주차 기동 검증으로 해석하지 않습니다. 기존 상단 5개 정적 파일의 계약은 그대로입니다.

새 최적 빈칸으로 변경할 때 도착 유지 시간을 초기화하고 `route_revision`, `replan_count`, `target_changes`를 갱신합니다. 같은 상태의 경로는 캐시합니다. 현재 목표가 free이며 반경의 두 배 이내면 유지하고, 반복 변경은 2초 간격으로 제한합니다. 현재 목표 occupied에는 이 제한 없이 대체 빈칸을 찾습니다. 수동 지정·자동 변경 해제·후보 없음·점유 만료에는 `status: blocked`, `guidance_available: false`, 빈 `waypoints`로 안내를 차단합니다. 마커가 없으면 현재 위치 경로를 표시하지 않으며 success 이후에는 목표를 유지합니다. 실제 cm 경로·모터 제어·주차 기동·도로 위 차량 장애물 회피는 이번 변경 범위에 포함하지 않습니다.

단, 유효한 로버 위치가 현재 도착 원 안에 들어오면 `arrival_committed: true`로 그 목표를 고정합니다. 이 시점 이후에는 occupied/unknown도 다른 칸으로 재선택하지 않고 blocked로 확인을 기다립니다. 목표가 고정돼도 다른 차량을 free로 바꾸거나 도착 유지 시간을 우회하지 않습니다. 검증된 로버 제외 결과로 free가 유지될 때 두 시점 위치 비교를 수행합니다. 새 테스트 시작 시 고정을 초기화합니다.

도착 판정은 `arrival_method: two_observation_comparison`입니다. `hold_seconds`의 기본값을 2초로 바꾸고 기존 요청 필드와 0.5~5초 허용 범위는 유지합니다. 비교 기준점과 마지막 새 관측 사이의 촬영 간격을 기존 `held_seconds`로 반환하며 연속 유효 시간의 합계를 뜻하지 않습니다. `comparison_anchor_px`, `comparison_displacement_px`, `comparison_max_age_seconds`를 추가합니다. 안정 반경은 기존 `min(5px, radius_px/3)`이고 최소 2개의 서로 다른 유효 관측을 요구합니다. `client_captured_at`이 있으면 검증된 촬영 시각을 서버 단조 시각으로 옮겨 비교하고, 없으면 수신 시각을 사용합니다. 중복 프레임·중복 또는 역순 촬영 시각·오래된 관측은 새 비교점으로 인정하지 않습니다.

현재 좌표의 200ms 만료·일반 미검출에는 현재 좌표·안내를 비우되 기준점을 보관합니다. `tracking_paused: true`의 `last_known_position_px`는 회색 참고점으로만 사용합니다. 조회·대기만으로 성공하지 않으며 재검출된 새 관측이 필요합니다. 기준점 보관 한도는 `max(5초, 2 × hold_seconds)`이고, 만료되면 `comparison_expired`로 다시 잡습니다. 두 시점 사이 미관측 이동은 확인할 수 없으며, 관측된 이동·원 이탈·모호/상이 ID·점유 차단·목표/지도/카메라/보정 변경에는 비교를 초기화하거나 시험을 무효화합니다. 모터 주행의 200ms TTL과 안전정지는 바꾸지 않습니다.

도착 진단 필드 `hold_reset_count`, `hold_reset_counts`, `last_hold_reset`은 비교 초기화 횟수·사유별 합계·마지막 초기화 직전 촬영 간격/관측 수/기록 시각을 제공합니다. 현재 사유는 `comparison_expired`, `marker_identity_uncertain`, `position_unstable`, `outside_arrival_region`, `position_correction_failed`, `occupancy_blocked` 및 목표·카메라·지도·사용자 중단 등입니다. 이전 `tracking_gap_too_long`·`hold_pause_budget_exceeded` 초기화는 사용하지 않으며 호환용 `hold_pause_seconds`는 0입니다. 반복 GET은 이미 초기화된 비교를 다시 세지 않고 성공 뒤에도 기록을 유지합니다. 새 시험을 시작하면 비웁니다. 구간 수집기는 테스트별 마지막 누적값만 합산합니다.

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

웹 도착 시험은 같은 캡처의 `world/camera_position.json`이 있으면 기존 `CapturePose`의 높이 보정 결과를 바닥 지도 픽셀로 역투영해 `current_position_px`에 제공합니다. 파란 점·도착 거리·안정 유지·현재 위치 재탐색이 같은 보정 좌표를 사용하며, 목표 원과 거리 단위는 지도 픽셀을 유지합니다. `GET /api/arrival-tests`와 시험 상태에 선택 필드 `height_correction_applied`를 추가합니다. 적용 시 `position_reference`는 `height_corrected_marker_with_offset`이고 기본 오프셋은 `[0,0]cm`입니다. 원본 `/api/rover-pose`·`rover.corners_px`는 카메라 픽셀을 유지하므로 점유 박스 연결과 기존 cm 전송기에 보정이 중복 적용되지 않습니다.

저장값이 없으면 기존 영상 좌표 시험을 명시적으로 표시하고, 저장값이 잘못됐으면 시험 시작을 거부합니다. 시험 중 카메라 실측값·지도 보정 변경은 `invalidated / position_correction_changed`, 현재 관측의 보정 실패는 `position_correction_failed` 및 선택 필드 `position_correction_error`로 표시하고 현재 위치·판정 진행을 비웁니다. 관측 만료·반복 프레임·점유 차단 규약은 유지합니다.

통합 추적 계약: 보완 검출의 `rover.corners_px`는 ROI 확대를 되돌린 원본 카메라 좌표입니다. 기존 높이 보정·월드 cm 변환기는 이 현재 관측만 사용하며 `last_known_position_px`, `tracking_paused` 같은 웹 도착 참고 상태는 위치 입력으로 사용하지 않습니다. 웹의 두 시점 비교 기준점 보관이 모터 주행의 TTL을 연장하지 않습니다. `pose_sender`와 주행기의 0.20초 유효 시간 및 유실 시 정지는 그대로입니다.

십자로 계산한 전지 외곽은 영상 가로·세로 각각 2%의 작은 이탈만 허용합니다. 보정 보고서의 선택 필드 `paper_bounds`에는 원래 `projected_corners_px`, `tolerance_px`, `slightly_outside`를 기록합니다. 실제 종이 경계 검출이나 잘린 부분의 복원을 뜻하지 않으며, 독립 확인점 검증·사진 해시 검사는 그대로 적용합니다. 큰 범위 이탈은 계산 좌표·영상 크기를 포함한 오류로 차단합니다.

로버 실측 치수는 폭 15cm, 전체 길이 19cm입니다. 주행 제어 기준점은 좌우 구동 바퀴 축의 중간점이며 앞끝까지 9.5cm, 뒤끝까지 9.5cm입니다. 좌우 폭이 대칭이라고 가정하면 기준점에서 좌우 각각 7.5cm를 차지합니다. 마커 면 높이는 약 10.5cm이고 마커 중심의 수평 위치는 주행 제어 기준점과 일치하며 중심 오프셋은 0cm입니다. 이 치수는 통로·주차칸·회전 공간 검사의 입력 기준으로 기록하며, 현재 경로 추종의 차체 외곽 충돌 검사는 후속 구현 대상입니다.

- 자동 보정에서는 빨간 십자 A~D의 실측 cm 좌표를 한 번 등록하고 같은 배치의 새 카메라 프레임에서 픽셀을 다시 검출합니다. 실물 주행에는 독립 확인점 E의 검증을 통과한 경로를 전달합니다. 기본 지도의 E는 A~D 안쪽에 있는 `space_5` 진입 도로와 EN–EX 도로의 교점입니다. 보정 보고서의 `validation`에는 교점의 시설 ID, 픽셀, 실측 cm, 변환한 cm, 오차가 기록됩니다. 캡처 PNG와 라벨 및 보정 보고서를 주행 경로와 함께 전달합니다. [자동 보정 실행 방법](../README.md#카메라에서-빨간-십자-자동-검출)
- HTTP에서 `--marker-layout`를 사용하면 같은 캡처의 픽셀 경로를 재사용해 `latest/world/`에 보정 결과를 저장합니다. API의 `world_calibration.status`가 `verified`이면 주차칸 중앙까지의 cm 경로를 제공합니다. `route_generation`의 정적 픽셀 파일은 도로 진입점에서 끝나며, 웹의 실시간 도착 시험은 선택한 칸 중앙까지 별도 미리보기 구간을 연결합니다. [HTTP 자동 보정 실행 방법](../README.md#같은-http-사진으로-cm-지도-만들기)
- `POST /api/aruco`, `GET /api/rover-pose`에서는 ID 7의 `rover.center_px`·`rover.heading_deg`를 카메라 픽셀과 영상 방향각으로 제공합니다. 목표 전송 주기는 100ms이며, 수신 후 200ms가 지나거나 마커를 잃으면 위치를 비웁니다. 로버 상단 마커를 월드 cm로 전달하려면 마커 높이와 로버 중심 오프셋의 실측 검증이 필요합니다.
- ArUco 전송은 원본 해상도의 JPEG 품질 0.92를 기본으로 하고 기존 `image/png`도 수신합니다. 이미지 디코딩 전 JPEG 헤더/PNG IHDR의 해상도 제한을 검사합니다. 지도 캡처 API는 계속 PNG만 받습니다. 브라우저는 인코딩 200ms 초과 시 전송을 버리고 HTTP 요청은 1초에 종료합니다. 좌표 만료는 계속 200ms이며 촬영 시각도 확인합니다. `detection_status`는 실제 검출 결과이고 `tracking_status`는 만료를 반영한 상태입니다. 늦은 프레임에서는 마커를 검출했어도 `rover:null`입니다. UI는 인코딩·전송/응답·서버 검출 시간을 분리 표시하고, 카메라 변경 도중의 프레임과 중복 영상 프레임을 재사용하지 않습니다.
- 검출은 native → 최근 0.75초 유효 위치 주변의 원본 2배 확대 ROI(`roi_2x`) → 대비 보정/2배 확대 ROI → 전체 대비 보정 순서로 보완합니다. ROI 힌트는 카메라 세션·원본 크기가 같을 때만 사용하고, 현재 영상에서 ID를 실제 디코딩합니다. 확대 좌표를 원본 픽셀 중심 규약으로 역변환합니다. 추가 보완 검출은 약 100ms 예산으로 제한하며 사전·ID·오류 보정 기준은 완화하지 않습니다. `detection_method`, `detection_attempts`, `rejected_candidate_count`, `recent_detection`은 진단 필드입니다. 최근 20프레임 비율은 ID 판독률이며 현재 위치 유효율과 다릅니다. 세션·해상도 변경 시 이력과 ROI 힌트를 비웁니다. 추가 필드 `detection_attempt_details`는 실제 검출 호출별 방법·검출기 인덱스·시간·ID·거부 수를, `fallback_budget_exhausted`는 예산에 따른 보완 검출 생략 여부를 나타냅니다. 기존 필드와 소비 인터페이스는 유지합니다.
- `GET /api/aruco-debug`(최근 입력)와 `?miss=1`(최근 누락)은 프레임 해시·수신 정보와 `frame_url`을 반환합니다. 반환된 이미지 URL은 진단 시점 이미지를 메모리에서 고정해 후속 프레임과 바뀌지 않습니다. 다음 진단 조회는 그 고정본을 교체합니다. `GET /api/aruco-frame?miss=1`은 최근 누락 원본입니다. 입력·누락·진단 각 한 장만 보관하고 이미지 파일을 누적하지 않습니다. 카메라 연결이 바뀌면 이전 누락·진단 캐시를 비웁니다.
- 구간 수집은 `python -m scripts.diagnose_aruco_tracking --label stationary|moving|arrival --duration 10`을 사용합니다(라벨은 셋 중 하나 선택). 실제 새 프레임의 검출률·수신 간격과 반복 GET의 stale를 분리합니다. 현재 구간의 마지막 누락 원본을 해시 검증해 로컬에 저장하며 라벨별 결과를 덮어씁니다. 2026-11-18 정지 92/92, 이동 포함 553/559·556/558 ID 판독을 확인했습니다. 이는 원본 ROI 확대 추가 전의 실물 기록이며 수정 후 연속 검출률·카메라 이동 후 새 맵의 도착 시험은 별도 확인합니다. 도착 GET도 촬영 후 200ms의 남은 유효 시간만 사용하고 수신 시각으로 TTL을 늘리지 않습니다.

- 지도에서 경로를 선택할 때는 픽셀 그래프를 사용합니다. 로버 경로 생성 시 웨이포인트와 간선·경로 길이를 cm로 변환하고, 최종 점을 선택한 주차칸 중앙으로 둡니다.
- 주행 파일은 `coordinate_frame: world`, `unit: cm`, `origin: paper_top_left`, `calibration_verified: true`와 `waypoints: [[x, y], ...]`를 사용합니다. `world_estimate`는 확인용 미리보기입니다.
- 로버 위치는 같은 노트북의 UDP `127.0.0.1:8766`에 약 0.10초마다 보냅니다. `captured_monotonic_s`는 그 노트북에서 프레임을 받은 시점의 `time.monotonic()`입니다. 0.20초를 넘긴 위치는 사용하지 않습니다.
- `src.rover_control.pose_sender`는 주행 전에 검증·저장한 바닥 변환식을 읽고, ArUco API가 보내는 로버 마커의 원본 픽셀 위치·방향을 월드로 바꿉니다. 웹캠 원본과 지도 PNG의 같은 비율 크기 차이를 반영합니다. 같은 구도에서 십자가 가려져도 변환식을 재사용합니다. 위치 메시지에는 `map_image_sha256`과 `id_assignment_revision`도 포함하며 주행기가 경로와 대조합니다. `--preview-only`로 위치를 확인할 수 있습니다.
- `drive --route`는 시작할 때 지정한 검증된 cm 경로를 읽습니다. 웹의 실시간 픽셀 경로·`route_revision`·점유 상태를 모터 주행으로 전달하는 소비자는 아직 없습니다. 후속 연결에서는 같은 캡처의 보정값과 지도 식별자를 검사하고, 마커 중심과 구동 바퀴 축 중간점의 차이 및 경로 교체·안내 중단 시 정지 처리를 맞춰야 합니다. 웹의 마커 중심·칸 내부 원 판정은 실제 주행기의 목표 2cm 이내 정지·새 위치 3개 안정 확인과 별도입니다.
- 높이 보정에는 `--camera-position-cm X Y Z`로 전지 원점 기준 렌즈 중심의 실측 위치를 입력합니다. X·Y는 렌즈에서 수직으로 내린 점이며 카메라가 바라보는 화면 중심점과 다릅니다. Z는 전지 표면부터 렌즈까지 수직 높이로, 전지 밖 X·Y와 음수 좌표도 허용합니다. `--marker-height-cm` 기본 10.5cm보다 Z가 커야 합니다. 바닥 투영점 Q를 `C_xy + (1-h/C_z)*(Q_xy-C_xy)`로 높이 평면에 옮긴 뒤, `--center-offset-cm` 기본 `[0, 0]cm`를 적용해 구동 바퀴 축 중간점을 계산합니다. 카메라 이동 후 위치를 다시 측정하고 지도를 재보정합니다. 구도 변경의 자동 감지는 없습니다. [측정·실행](../README.md#보정된-cm-위치를-주행-프로그램에-보내기)
- 현재 렌즈 실측값 `[53, 151.7, 111.5]cm`와 마커 높이 10.5cm는 캡처별 `world/camera_position.json`에 저장합니다. 명시적 `--camera-position-cm` 입력은 검증 후 현재 캡처에 설정을 저장하고, 생략하면 저장값을 사용합니다. 캡처 보고서·보정 파일의 SHA-256이 달라지면 재입력을 요구합니다. 새 구도에 실측값을 자동 이식하지 않습니다.
- 높이 보정된 UDP 위치는 기존 월드 cm 계약을 유지하고 `position_reference: height_corrected_marker_with_offset`, `height_correction_applied: true`, `marker_height_cm`, `camera_position_cm`을 추가합니다. 주행기의 기존 좌표·지도 식별자 소비는 유지합니다. 웹의 `/api/rover-pose`는 상단 마커의 원본 카메라 픽셀을 제공하고, 도착 시험은 저장된 카메라 실측값이 있을 때 높이 보정 결과를 지도 픽셀로 역투영합니다. CLI의 높이 미보정 모드는 `--floor-projection-only --preview-only` 진단으로 제한하며 UDP를 전송하지 않습니다. 마커는 전지와 평행해야 하며 기울기·전방 부착 오차와 렌즈 왜곡은 별도로 확인합니다. 로버 마커 유실·중복, 관측 만료, 통신 오류, 카메라 세션·해상도 또는 기준 지도 불일치, 전송기 종료에는 기존대로 `lost`를 보냅니다.

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
