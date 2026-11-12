# 현재 점유 연결 상태

현재 지도 ID 버전은 `ocr-p-labels-v1`입니다. P1/P2/P4/P5는 일반석, P3/P6은 장애인석입니다.
~~이전 구도의 ParkingHaeven 매핑과 점유 파일 재사용~~은 폐기했습니다.

기존 테스트 스냅샷/결과는
`archives/before_ocr_rule_update_20261111_221709/historical_occupancy_folder/`에 보관했습니다.
이전 경로는 같은 백업 아래 `historical_occupancy_test_routes/`에 있습니다.

현재 ParkingHaeven 매핑은 `requires_new_camera_regions` 상태입니다. 실제 카메라에서 칸 ID와 영역을 다시 대조한 뒤 점유 결과를 확인하세요. ONNX 경로의 카메라 설정과 JSON 형식은 [팀 가이드](../../../docs/TEAM_GUIDE.md#카메라-점유-설정)를 참고합니다.
