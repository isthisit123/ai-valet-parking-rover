"""ParkingHaeven의 과거 boolean 점유 파일로 경로 선택을 재현한다. 실시간 입력용이 아니다."""

import argparse
import copy
import hashlib
import json
from datetime import datetime, timezone
from math import dist, sqrt
from pathlib import Path

import cv2
import numpy as np

from src.map_processing.map_config import BASE_DIR, get_map_config
from src.map_processing.map_reader import detect_road_mask, load_map_image
from src.map_processing.graph_builder import remove_small_components
from src.map_processing.occupancy import occupancy_states
from src.map_processing.parking_selector import load_parking_map, select_parking_from_map
from src.map_processing.path_visualizer import DEFAULT_ROUTE_CASES, build_route_data, draw_route


TEST_CLOCK = datetime(2000, 1, 1, tzinfo=timezone.utc)


def save_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def convert_export(payload, mapping):
    # JSON의 점유 boolean은 테스트 정답으로만 사용한다. 미관측 실패를 구별하지 못한다.
    entries = payload[mapping["source"]["source_id"]]
    by_id = {}
    for entry in entries:
        source_id = str(entry["id"])
        if source_id in by_id:
            raise ValueError(f"중복된 원본 ID: {source_id}")
        if type(entry.get("occupied")) is not bool:
            raise ValueError(f"점유 boolean이 없습니다: {source_id}")
        by_id[source_id] = entry
    definitions = {str(entry["id"]): entry["points"] for entry in entries}
    geometry = json.dumps(definitions, sort_keys=True, separators=(",", ":")).encode("utf-8")
    if hashlib.sha256(geometry).hexdigest() != mapping["source"]["region_geometry_sha256"]:
        raise ValueError("원본 영역 ID/좌표가 매핑 생성 당시와 다릅니다. 매핑을 다시 확인하세요.")
    expected_ids = {item["source_region_id"] for item in mapping["spaces"]}
    if set(by_id) != expected_ids:
        raise ValueError("원본 점유 영역과 매핑의 ID 목록이 다릅니다.")
    target_ids = [item["parking_space_id"] for item in mapping["spaces"]]
    if len(set(target_ids)) != len(target_ids):
        raise ValueError("여러 원본 영역이 같은 주차칸 ID에 연결되어 있습니다.")
    return {
        "schema_version": "0.1-draft", "map_id": mapping["map_id"],
        "usage": "offline_test_only", "observed_at": None,
        "assumption": "내보낸 occupied boolean을 정적 테스트 정답으로 가정. 실제 관측 시각과 신뢰도는 알 수 없음.",
        "spaces": [
            {
                "parking_space_id": item["parking_space_id"],
                "source_region_id": item["source_region_id"],
                "status": "occupied" if by_id[item["source_region_id"]]["occupied"] else "free",
                "confidence": None,
            }
            for item in mapping["spaces"]
        ],
    }


def replay_fixture(snapshot):
    fixture = copy.deepcopy(snapshot)
    fixture["observed_at"] = TEST_CLOCK.isoformat()
    fixture["confidence_basis"] = "synthetic_test_truth_not_detector_confidence"
    for item in fixture["spaces"]:
        # 소비자의 신뢰도 조건을 시험하기 위한 명시적 fixture 값. 원본 파일에는 넣지 않는다.
        item["confidence"] = 1.0
    return fixture


def validate_mapping(mapping, map_data, image):
    if mapping["map_id"] != map_data["map_id"]:
        raise ValueError("지도와 매핑의 map_id가 다릅니다.")
    height, width = image.shape[:2]
    if [width, height] != mapping["target"]["image_size_px"]:
        raise ValueError("지도 해상도가 매핑과 다릅니다.")
    targets = {region["id"]: region for region in map_data["regions"]
               if region.get("role") in ("parking_normal", "parking_accessible")}
    mapped_ids = [item["parking_space_id"] for item in mapping["spaces"]]
    if set(mapped_ids) != set(targets):
        raise ValueError("매핑과 의미 지도의 주차칸 목록이 다릅니다.")
    matrix = np.asarray(mapping["transform"]["matrix_3x3"], dtype=float)
    masks = {}
    for space_id, region in targets.items():
        mask = np.zeros((height, width), dtype=np.uint8)
        x, y, w, h = region["bbox"]
        mask[y:y+h, x:x+w] = 1
        masks[space_id] = mask
    overlaps = []
    for item in mapping["spaces"]:
        points = np.asarray(item["source_polygon_px"], dtype=float)
        homogeneous = (matrix @ np.column_stack([points, np.ones(len(points))]).T).T
        points = homogeneous[:, :2] / homogeneous[:, 2:3]
        if not np.isfinite(points).all() or not np.allclose(points, item["map_polygon_px"]):
            raise ValueError("저장된 변환 다각형이 행렬 계산 결과와 다릅니다.")
        if (points < 0).any() or (points[:, 0] >= width).any() or (points[:, 1] >= height).any():
            raise ValueError("변환 영역이 지도 밖에 있습니다.")
        mask = np.zeros((height, width), dtype=np.uint8)
        cv2.fillPoly(mask, [np.round(points).astype(np.int32)], 1)
        scores = {
            space_id: float(np.count_nonzero(mask & target) / np.count_nonzero(mask | target))
            for space_id, target in masks.items()
        }
        space_id = item["parking_space_id"]
        if max(scores, key=scores.get) != space_id or scores[space_id] < 0.5:
            raise ValueError(f"매핑 겹침 검증 실패: {space_id}, {scores}")
        if item["parking_type"] != targets[space_id]["type"]:
            raise ValueError(f"주차칸 유형 불일치: {space_id}")
        overlaps.append({"parking_space_id": space_id, "bbox_iou": scores[space_id]})
    return overlaps


def run(source_path, mapping_path, output_dir, routes_dir):
    mapping = json.loads(mapping_path.read_text(encoding="utf-8-sig"))
    config = get_map_config(mapping["map_id"])
    if hashlib.sha256(config.image_path.read_bytes()).hexdigest() != mapping["target"]["image_sha256"]:
        raise ValueError("지도가 매핑 생성 당시와 다릅니다. 변환을 재확인하세요.")
    map_data, graph = load_parking_map(config.semantic_path)
    image = load_map_image(config.image_path)
    overlaps = validate_mapping(mapping, map_data, image)
    payload = json.loads(source_path.read_text(encoding="utf-8-sig"))
    snapshot = convert_export(payload, mapping)
    snapshot["source_file_name"] = source_path.name
    snapshot["source_file_sha256"] = hashlib.sha256(source_path.read_bytes()).hexdigest()
    fixture = replay_fixture(snapshot)
    states = occupancy_states(map_data, fixture, now=TEST_CLOCK)
    road_mask = remove_small_components(detect_road_mask(image))
    cases = DEFAULT_ROUTE_CASES + (
        ("06_entrance_normal", "entrance", "normal"),
        ("07_building_b_normal", "building_b", "normal"),
    )
    report = {
        "map_id": config.map_id, "usage": "offline_test_only",
        "snapshot_observed_at": None, "simulation_clock": TEST_CLOCK.isoformat(),
        "confidence_basis": fixture["confidence_basis"],
        "iou_threshold": 0.5, "iou_target_geometry": "semantic_region_bbox",
        "mapping_overlap": overlaps, "cases": [],
    }
    save_json(output_dir / "test_snapshot.json", snapshot)
    routes_dir.mkdir(parents=True, exist_ok=True)
    for case_id, preference, parking_type in cases:
        baseline = select_parking_from_map(map_data, graph, preference, parking_type=parking_type)
        baseline_space = graph.nodes[baseline["selected"]]["region_id"]
        row = {"case_id": case_id, "preference": preference, "parking_type": parking_type,
               "baseline_parking_space_id": baseline_space}
        free_ids = {r["id"] for r in map_data["regions"]
                    if r.get("type") == parking_type and states.get(r["id"]) == "free"}
        if not free_ids:
            try:
                select_parking_from_map(map_data, graph, preference, parking_type=parking_type,
                                        occupancy=fixture, now=TEST_CLOCK)
            except ValueError as exc:
                if "free" not in str(exc):
                    raise
                row.update(outcome="no_free_spaces", selected=None, message=str(exc))
            else:
                raise AssertionError("빈칸이 없는데 주차칸을 선택했습니다.")
            # 같은 출력 폴더를 재사용할 때 과거 성공 경로가 남지 않도록 정리한다.
            (routes_dir / f"{case_id}.png").unlink(missing_ok=True)
            save_json(routes_dir / f"{case_id}.json", {**row, "usage": "offline_test_only"})
        else:
            route = build_route_data(config.map_id, map_data, graph, preference, road_mask,
                                     parking_type=parking_type, occupancy=fixture, now=TEST_CLOCK)
            assert route["parking_space_id"] in free_ids
            assert all(0 < dist(a, b) <= sqrt(2) for a, b in zip(route["waypoints"], route["waypoints"][1:]))
            route.update(case_id=case_id, usage="offline_test_only",
                         source_observed_at=None, confidence_basis=fixture["confidence_basis"])
            preview = draw_route(image, graph, route)
            for region in map_data["regions"]:
                if states.get(region["id"]) == "occupied":
                    x, y, w, h = region["bbox"]
                    cv2.rectangle(preview, (x, y), (x+w, y+h), (0, 0, 255), 2)
                    cv2.putText(preview, region["id"] + " OCCUPIED", (x, y+20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            ok, encoded = cv2.imencode(".png", preview)
            if not ok:
                raise OSError("경로 PNG 인코딩 실패")
            (routes_dir / f"{case_id}.png").write_bytes(encoded.tobytes())
            save_json(routes_dir / f"{case_id}.json", route)
            row.update(outcome="route_created", selected=route["parking_space_id"],
                       destination_changed=baseline_space != route["parking_space_id"],
                       total_distance=route["total_distance"], unit=route["unit"])
        report["cases"].append(row)
        print(case_id, baseline_space, "->", row["selected"], row["outcome"])
    # 고정 테스트 시각에서만 사용한다. 실제 현재 시각에서는 만료되는지도 확인한다.
    assert all(status == "unknown" for status in occupancy_states(map_data, fixture).values())
    all_occupied = copy.deepcopy(fixture)
    for item in all_occupied["spaces"]:
        item["status"] = "occupied"
    for parking_type in ("normal", "accessible"):
        try:
            select_parking_from_map(map_data, graph, "any", parking_type=parking_type,
                                    occupancy=all_occupied, now=TEST_CLOCK)
        except ValueError as exc:
            assert "free" in str(exc)
        else:
            raise AssertionError("전체 점유 시 선택이 중단되지 않았습니다.")
    report["additional_checks"] = {"stale_replay_rejected_by_live_clock": True,
                                   "all_occupied_rejected_for_both_types": True}
    save_json(output_dir / "test_results.json", report)
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--mapping", type=Path, default=BASE_DIR / "assets/maps/parking_map_real_parkinghaeven_mapping.json")
    parser.add_argument("--output-dir", type=Path, default=BASE_DIR / "data/occupancy/parking_map_real")
    parser.add_argument("--routes-dir", type=Path, default=BASE_DIR / "data/maps/parking_map_real/occupancy_test_routes")
    args = parser.parse_args()
    run(args.source, args.mapping, args.output_dir, args.routes_dir)
