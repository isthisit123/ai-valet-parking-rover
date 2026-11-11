import json
import math
from pathlib import Path

import cv2
import numpy as np

from .map_config import (
    ACCESSIBLE_REGION,
    BUILDING_REGION,
    DEFAULT_MAP,
    GATE_REGION,
    REAL_ACCESSIBLE_REGION,
    REAL_DARK_MIN_FILL_RATIO,
    REAL_DARK_OPEN_KERNEL_SIZE,
    REAL_MAP_ID,
    REGION_BORDER_MARGIN,
    REGION_CLOSE_KERNEL_SIZE,
    REGION_MAX_IMAGE_RATIO,
    REGION_MIN_FILL_RATIO,
    REGION_MIN_SIDE,
    create_map_parser,
    get_map_config,
    normal_region_for_map,
)
from .map_reader import load_map_image
from .planar_calibration import load_world_to_image, project_points


def extract_regions(mask, min_area, image_shape, *, allow_border=False,
                    min_fill_ratio=REGION_MIN_FILL_RATIO):
    # 같은 색상으로 연결된 영역을 각각 검출
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(
        mask, connectivity=8
    )

    height, width = image_shape[:2]
    regions = []

    for i in range(1, count):
        x, y, w, h, area = stats[i]

        # 너무 작거나 이미지 대부분을 차지하는 영역은 제외
        if area < min_area:
            continue

        if area > width * height * REGION_MAX_IMAGE_RATIO:
            continue

        # 실물 검은 시설 후보를 제외하고 이미지 경계에 닿은 영역은 거른다.
        if not allow_border and (
            x <= REGION_BORDER_MARGIN
            or y <= REGION_BORDER_MARGIN
            or x + w >= width - REGION_BORDER_MARGIN
            or y + h >= height - REGION_BORDER_MARGIN
        ):
            continue

        if w < REGION_MIN_SIDE or h < REGION_MIN_SIDE:
            continue

        # 테두리나 배경의 가느다란 조각은 실제 사각형 영역에서 제외
        if area / (w * h) < min_fill_ratio:
            continue

        cx, cy = centroids[i]

        regions.append({
            "bbox": [int(x), int(y), int(w), int(h)],
            "center": [round(float(cx), 2), round(float(cy), 2)],
            "area": int(area)
        })

    return regions


def detect_regions(image, *, map_id=DEFAULT_MAP.map_id):
    # 색상 검출을 위해 BGR 이미지를 HSV로 변환
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    kernel = np.ones(REGION_CLOSE_KERNEL_SIZE, np.uint8)
    is_real_map = map_id == REAL_MAP_ID
    accessible_region = REAL_ACCESSIBLE_REGION if is_real_map else ACCESSIBLE_REGION
    region_settings = (
        (normal_region_for_map(map_id), False),
        (accessible_region, False),
        (BUILDING_REGION, is_real_map),
        (GATE_REGION, False),
    )
    regions = []
    for (lower, upper, min_area), is_real_dark in region_settings:
        mask = cv2.inRange(hsv, lower, upper)
        if is_real_dark:
            # 얇게 이어진 그림자를 끊고, 경계에 닿은 시설도 힌트 매칭 후보로 남긴다.
            opening_kernel = np.ones(REAL_DARK_OPEN_KERNEL_SIZE, np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, opening_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        regions.append(
            extract_regions(
                mask, min_area, image.shape, allow_border=is_real_dark,
                min_fill_ratio=REAL_DARK_MIN_FILL_RATIO if is_real_dark else REGION_MIN_FILL_RATIO,
            )
        )
    return tuple(regions)


def assign_labels(regions, label_specs):
    # 위치 힌트와 가까운 검출 영역에 고유 이름 부여
    result = []
    used = set()

    if len(regions) < len(label_specs):
        raise ValueError(
            f"검출 영역이 라벨 힌트보다 적습니다. "
            f"검출 {len(regions)}개, 힌트 {len(label_specs)}개."
        )

    for spec in label_specs:
        hint = spec["hint"]

        candidates = [
            (math.dist(region["center"], hint), i)
            for i, region in enumerate(regions)
            if i not in used
        ]

        _, index = min(candidates)
        used.add(index)

        region = regions[index].copy()
        region["id"] = spec["id"]
        region["role"] = spec["id"]

        result.append(region)

    return result


def is_pixel_hint(hint):
    return (
        isinstance(hint, list)
        and len(hint) == 2
        and all(type(value) in (int, float) for value in hint)
    )


def resolve_cm_hints(label_data, image_size, calibration_path):
    resolved = label_data.copy()
    cm_specs = []
    for group in ("buildings", "gates", "parking_spaces"):
        specs = label_data.get(group)
        if not isinstance(specs, list):
            continue
        resolved[group] = specs.copy()
        for index, spec in enumerate(specs):
            if not isinstance(spec, dict) or "hint_cm" not in spec:
                continue
            if "hint" in spec:
                raise ValueError(f"{spec.get('id')!r}의 hint와 hint_cm 중 하나만 지정하세요.")
            copy = spec.copy()
            resolved[group][index] = copy
            cm_specs.append(copy)

    if not cm_specs:
        return label_data
    if calibration_path is None:
        raise ValueError("hint_cm을 사용하려면 --calibration으로 Homography JSON을 지정하세요.")
    matrix, paper_size = load_world_to_image(
        calibration_path, map_id=label_data.get("map_id"), image_size=image_size
    )

    # 실제 위치를 지도 픽셀로 역변환하고, 기존 영역 매칭에는 픽셀 힌트를 넘긴다.
    for spec in cm_specs:
        hint_cm = spec.pop("hint_cm")
        if not is_pixel_hint(hint_cm) or not np.isfinite(hint_cm).all():
            raise ValueError(f"{spec.get('id')!r}의 hint_cm은 유효한 [x, y] 좌표여야 합니다.")
        point = np.asarray(hint_cm, dtype=float)
        if (point < 0).any() or (point > paper_size).any():
            raise ValueError(f"{spec.get('id')!r}의 hint_cm이 전지 범위를 벗어났습니다.")
        denominator = matrix[2, :2] @ point + matrix[2, 2]
        if not np.isfinite(denominator) or np.isclose(denominator, 0):
            raise ValueError(f"{spec.get('id')!r}의 hint_cm을 지도 픽셀로 변환할 수 없습니다.")
        pixel = project_points([point], matrix)[0]
        if not np.isfinite(pixel).all() or (pixel < 0).any() or (pixel >= image_size).any():
            raise ValueError(f"{spec.get('id')!r}의 hint_cm을 변환한 위치가 지도 이미지를 벗어났습니다.")
        spec["hint"] = pixel.tolist()
    return resolved


def matching_region_indices(regions, hint):
    hint_x, hint_y = hint
    matches = []
    for index, region in enumerate(regions):
        x, y, width, height = region["bbox"]
        if x <= hint_x < x + width and y <= hint_y < y + height:
            matches.append(index)
    return matches


def assign_dark_facilities(regions, label_specs):
    # 실물 지도의 건물과 입출구는 검은색 후보를 공유한다.
    named_regions = []
    used_regions = set()
    used_ids = set()
    for spec in label_specs:
        facility_id, hint = spec["id"], spec.get("hint")
        if facility_id in used_ids:
            raise ValueError(f"건물 또는 입출구 ID {facility_id!r}가 중복됩니다.")
        if not is_pixel_hint(hint):
            raise ValueError(f"시설 {facility_id!r}의 hint는 [x, y] 좌표여야 합니다.")
        matches = matching_region_indices(regions, hint)
        if len(matches) != 1:
            raise ValueError(
                f"시설 {facility_id!r}의 위치 힌트 {hint}에 해당하는 검은색 영역이 "
                f"{len(matches)}개입니다. 정확히 1개가 필요합니다."
            )
        index = matches[0]
        if index in used_regions:
            raise ValueError(f"시설 {facility_id!r}의 힌트가 이미 매핑된 검은색 영역을 가리킵니다.")
        used_ids.add(facility_id)
        used_regions.add(index)
        named_regions.append({**regions[index], "id": facility_id, "role": facility_id})
    return named_regions


def assign_parking_ids(normal, accessible, label_specs):
    regions_by_type = {"normal": normal, "accessible": accessible}
    detected_count = len(normal) + len(accessible)
    if not detected_count:
        raise ValueError("지도에서 주차칸을 검출하지 못했습니다.")
    if not isinstance(label_specs, list) or not label_specs:
        raise ValueError("지도 라벨에 parking_spaces 위치 힌트가 필요합니다.")
    if detected_count != len(label_specs):
        raise ValueError(
            "주차칸 검출 수와 위치 힌트 수가 다릅니다. "
            f"검출 {detected_count}개, 힌트 {len(label_specs)}개."
        )

    parking_regions = []
    used_ids = set()
    used_regions = set()
    for spec in label_specs:
        if not isinstance(spec, dict):
            raise ValueError("parking_spaces의 각 항목은 JSON 객체여야 합니다.")
        space_id = spec.get("id")
        space_type = spec.get("type")
        hint = spec.get("hint")
        if not isinstance(space_id, str) or not space_id.strip():
            raise ValueError("주차칸 id는 비어 있지 않은 문자열이어야 합니다.")
        if space_id in used_ids:
            raise ValueError(f"중복 주차칸 ID {space_id!r}.")
        if space_type not in ("normal", "accessible"):
            raise ValueError(f"주차칸 {space_id!r}의 type이 잘못되었습니다.")
        if not is_pixel_hint(hint):
            raise ValueError(f"주차칸 {space_id!r}의 hint는 [x, y] 좌표여야 합니다.")

        # 같은 유형의 검출 상자 안에 힌트가 있어야 ID를 붙인다.
        # 후보가 없거나 여러 개면 가까운 다른 칸으로 대신 매핑하지 않는다.
        detected = regions_by_type[space_type]
        matches = matching_region_indices(detected, hint)
        if len(matches) != 1:
            raise ValueError(
                f"주차칸 {space_id!r}({space_type})의 위치 힌트 {hint}에 "
                f"해당하는 검출 영역이 {len(matches)}개입니다. 정확히 1개가 필요합니다."
            )
        region_index = matches[0]
        region_key = (space_type, region_index)
        if region_key in used_regions:
            raise ValueError(f"주차칸 {space_id!r}의 위치 힌트가 이미 매핑된 영역을 가리킵니다.")

        used_ids.add(space_id)
        used_regions.add(region_key)
        parking_regions.append({
            **detected[region_index],
            "id": space_id,
            "role": f"parking_{space_type}",
            "type": space_type,
        })

    return parking_regions


def point_to_bbox_distance(px, py, bbox):
    # 한 점에서 사각형 영역까지의 최단 거리
    x, y, w, h = bbox

    dx = max(x - px, 0, px - (x + w))
    dy = max(y - py, 0, py - (y + h))

    return math.hypot(dx, dy)


def assign_regions_to_nodes(nodes, regions):
    # 주차칸 및 출입구 영역을 가장 가까운 끝점에 연결
    endpoints = [
        node for node in nodes
        if node["type"] == "endpoint"
    ]

    # 연결을 시작하기 전에 검출 과다 여부를 확인
    if len(regions) > len(endpoints):
        raise ValueError(
            f"연결할 시설이 도로 끝점보다 많습니다. 시설 {len(regions)}개, "
            f"끝점 {len(endpoints)}개. 색상 마스크와 영역 검출 기준을 확인하세요."
        )

    candidates = []

    for region_index, region in enumerate(regions):
        for node in endpoints:
            distance = point_to_bbox_distance(
                node["x"],
                node["y"],
                region["bbox"]
            )

            candidates.append((
                distance,
                region_index,
                node["id"]
            ))

    # 가까운 영역과 노드 조합부터 일대일 연결
    candidates.sort()

    used_regions = set()
    used_nodes = set()
    node_lookup = {node["id"]: node for node in nodes}

    for distance, region_index, node_id in candidates:
        if region_index in used_regions:
            continue

        if node_id in used_nodes:
            continue

        region = regions[region_index]
        node = node_lookup[node_id]

        node["role"] = region["role"]
        node["region_id"] = region["id"]

        region["node_id"] = node_id

        used_regions.add(region_index)
        used_nodes.add(node_id)

    if len(used_regions) != len(regions):
        raise ValueError("일부 주차칸 또는 출입구를 도로 끝점에 연결하지 못했습니다.")


def map_semantics(graph_data, label_data, detected_regions, *, expected_map_id=None):
    map_id = label_data.get("map_id")
    if not isinstance(map_id, str) or not map_id.strip():
        raise ValueError("지도 라벨에 비어 있지 않은 map_id가 필요합니다.")
    if expected_map_id is not None and map_id != expected_map_id:
        raise ValueError(f"선택한 지도 {expected_map_id!r}와 라벨의 map_id {map_id!r}가 다릅니다.")

    normal, accessible, buildings, gates = detected_regions
    parking_regions = assign_parking_ids(
        normal, accessible, label_data.get("parking_spaces")
    )

    # 건물 A/B 및 입구/출구에 의미 부여
    building_specs, gate_specs = label_data["buildings"], label_data["gates"]
    if map_id == REAL_MAP_ID:
        facilities = assign_dark_facilities(buildings, building_specs + gate_specs)
        named_buildings = facilities[:len(building_specs)]
        named_gates = facilities[len(building_specs):]
    else:
        named_buildings = assign_labels(buildings, building_specs)
        named_gates = assign_labels(gates, gate_specs)

    facility_ids = {region["id"] for region in named_gates + named_buildings}
    for region in parking_regions:
        if region["id"] in facility_ids:
            raise ValueError(f"주차칸 ID {region['id']!r}가 건물 또는 출입구 ID와 중복됩니다.")

    # 원본 그래프의 노드를 보존하고 복사본에 주차칸과 출입구를 연결한다.
    nodes = [node.copy() for node in graph_data["nodes"]]
    assign_regions_to_nodes(nodes, parking_regions + named_gates)

    # 건물은 도로 노드가 아니므로 별도 영역으로 저장
    return {
        **graph_data,
        "nodes": nodes,
        "regions": parking_regions + named_gates + named_buildings,
        "map_id": map_id,
    }


def create_semantic_map(
    *,
    image_path=DEFAULT_MAP.image_path,
    graph_path=DEFAULT_MAP.graph_path,
    label_path=DEFAULT_MAP.label_path,
    output_path=DEFAULT_MAP.semantic_path,
    expected_map_id=None,
    calibration_path=None,
):
    image = load_map_image(image_path)
    with open(graph_path, "r", encoding="utf-8") as f:
        graph_data = json.load(f)
    with open(label_path, "r", encoding="utf-8-sig") as f:
        label_data = json.load(f)
    height, width = image.shape[:2]
    label_data = resolve_cm_hints(label_data, (width, height), calibration_path)

    map_id = label_data.get("map_id")
    detected = detect_regions(image, map_id=map_id)
    normal, accessible, buildings, gates = detected
    print("Normal spaces detected:", len(normal))
    print("Accessible spaces detected:", len(accessible))
    if map_id == REAL_MAP_ID:
        print("검은색 시설 후보:", len(buildings))
    else:
        print("Buildings detected:", len(buildings))
        print("Gate regions detected:", len(gates))
    semantic_data = map_semantics(
        graph_data, label_data, detected, expected_map_id=expected_map_id
    )
    if map_id == REAL_MAP_ID:
        print("매칭된 건물:", len(label_data["buildings"]))
        print("매칭된 입출구:", len(label_data["gates"]))

    # 의미가 추가된 지도 저장
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with output_path.open("w", encoding="utf-8") as f:
        json.dump(
            semantic_data,
            f,
            indent=2,
            ensure_ascii=False
        )

    # 노드의 의미 출력
    for node in semantic_data["nodes"]:
        if "role" in node:
            print(node["id"], "->", node["role"])

    print("Saved:", output_path)

    return semantic_data


if __name__ == "__main__":
    parser = create_map_parser("도로 그래프에 주차칸과 시설 정보를 연결합니다.")
    parser.add_argument("--calibration", type=Path, help="hint_cm에 사용할 Homography JSON")
    args = parser.parse_args()
    config = get_map_config(args.map_id, output_root=args.output_dir)
    try:
        create_semantic_map(
            image_path=config.image_path,
            graph_path=config.graph_path,
            label_path=config.label_path,
            output_path=config.semantic_path,
            expected_map_id=config.map_id,
            calibration_path=args.calibration,
        )
    except (OSError, ValueError) as error:
        parser.error(str(error))
