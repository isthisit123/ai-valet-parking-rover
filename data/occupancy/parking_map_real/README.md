# 현재 점유 연결 상태

기본 지도가 `ocr-p-labels-v1`로 변경됐습니다. 현재 P1/P2/P4/P5는 일반석,
P3/P6은 장애인석입니다. 이전 ID 배치의 점유 파일을 그대로 사용하지 않습니다.

기존 테스트 스냅샷/결과는
`archives/before_ocr_rule_update_20261111_221709/historical_occupancy_folder/`에 보관했습니다.
이전 경로는 같은 백업 아래 `historical_occupancy_test_routes/`에 있습니다.

새 카메라 영역 JSON으로 ID 대응·좌표 변환·IoU를 확인하기 전에는 현재 점유 상태가 확정되지
않습니다. `current.json`이나 새 `test_snapshot.json`을 아직 생성하지 않았습니다.
기존 매핑은 `requires_new_camera_regions` 상태여서 재현 스크립트가 사용을 거부합니다.
