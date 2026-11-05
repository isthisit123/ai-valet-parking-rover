import json
import math
from pathlib import Path

import cv2
import numpy as np

from .map_reader import BASE_DIR


IMAGE_PATH = BASE_DIR / "assets" / "maps" / "parking_map_v1.png"
GRAPH_PATH = BASE_DIR / "data" / "graphs" / "parking_map_v1.json"
LABEL_PATH = BASE_DIR / "assets" / "maps" / "parking_map_v1_labels.json"
OUTPUT_PATH = BASE_DIR / "data" / "graphs" / "parking_map_v1_semantic.json"


def extract_regions(mask, min_area, image_shape):
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

        if area > width * height * 0.08:
            continue

        # 이미지 경계와 연결된 배경 영역 제외
        if x <= 1 or y <= 1:
            continue

        if x + w >= width - 1 or y + h >= height - 1:
            continue

        if w < 20 or h < 20:
            continue

        # 테두리나 배경의 가느다란 조각은 실제 사각형 영역에서 제외
        if area / (w * h) < 0.6:
            continue

        cx, cy = centroids[i]

        regions.append({
            "bbox": [int(x), int(y), int(w), int(h)],
            "center": [round(float(cx), 2), round(float(cy), 2)],
            "area": int(area)
        })

    return regions


def detect_regions(image):
    # 색상 검출을 위해 BGR 이미지를 HSV로 변환
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # 일반 주차칸: 흰색
    white_mask = cv2.inRange(
        hsv,
        np.array([0, 0, 245]),
        np.array([179, 55, 255])
    )

    # 장애인 주차칸: 밝은 파란색 (어두운 청회색 건물 제외)
    blue_mask = cv2.inRange(
        hsv,
        np.array([90, 70, 150]),
        np.array([135, 255, 255])
    )

    # 건물: 어두운 회색 또는 검은색
    building_mask = cv2.inRange(
        hsv,
        np.array([0, 0, 0]),
        np.array([179, 150, 120])
    )

    # 입구와 출구: 밝은 회색
    gate_mask = cv2.inRange(
        hsv,
        np.array([0, 0, 150]),
        np.array([179, 85, 238])
    )

    # 작은 구멍이나 색상 잡음 보정
    kernel = np.ones((3, 3), np.uint8)

    masks = [white_mask, blue_mask, building_mask, gate_mask]
    masks = [
        cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        for mask in masks
    ]

    white_mask, blue_mask, building_mask, gate_mask = masks

    # 색상별 연결 영역 추출
    normal = extract_regions(white_mask, 1000, image.shape)
    # 하단 범례의 작은 파란색 아이콘은 제외
    accessible = extract_regions(blue_mask, 3000, image.shape)
    buildings = extract_regions(building_mask, 1500, image.shape)
    gates = extract_regions(gate_mask, 1500, image.shape)

    return normal, accessible, buildings, gates


def assign_labels(regions, label_specs):
    # 위치 힌트와 가까운 검출 영역에 고유 이름 부여
    result = []
    used = set()

    if len(regions) < len(label_specs):
        raise ValueError(
            f"Not enough regions: detected={len(regions)}, "
            f"expected={len(label_specs)}"
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
            f"Too many regions for endpoints: regions={len(regions)}, "
            f"endpoints={len(endpoints)}. Check color masks and region filters."
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
        raise ValueError("Some regions could not be assigned to nodes")


def create_semantic_map():
    # 원본 지도와 기존 그래프 불러오기
    image = cv2.imread(str(IMAGE_PATH))

    if image is None:
        raise FileNotFoundError(IMAGE_PATH)

    with open(GRAPH_PATH, "r", encoding="utf-8") as f:
        graph_data = json.load(f)

    with open(LABEL_PATH, "r", encoding="utf-8") as f:
        label_data = json.load(f)

    # 주차칸, 건물, 출입구 영역 자동 검출
    normal, accessible, buildings, gates = detect_regions(image)

    print("Normal spaces detected:", len(normal))
    print("Accessible spaces detected:", len(accessible))
    print("Buildings detected:", len(buildings))
    print("Gate regions detected:", len(gates))

    # 건물 A/B 및 입구/출구에 의미 부여
    named_buildings = assign_labels(
        buildings, label_data["buildings"]
    )

    named_gates = assign_labels(
        gates, label_data["gates"]
    )

    # 주차칸 영역에 고유 ID 및 종류 부여
    parking_regions = []

    for role, detected in [
        ("parking_normal", normal),
        ("parking_accessible", accessible)
    ]:
        for region in detected:
            parking_regions.append({
                **region,
                "id": f"space_{len(parking_regions) + 1}",
                "role": role
            })

    # 주차칸과 출입구를 그래프 끝점에 연결
    assign_regions_to_nodes(
        graph_data["nodes"],
        parking_regions + named_gates
    )

    # 건물은 도로 노드가 아니므로 별도 영역으로 저장
    graph_data["regions"] = (
        parking_regions + named_gates + named_buildings
    )

    # 의미가 추가된 지도 저장
    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)

    with open(OUTPUT_PATH, "w", encoding="utf-8") as f:
        json.dump(
            graph_data,
            f,
            indent=2,
            ensure_ascii=False
        )

    # 노드의 의미 출력
    for node in graph_data["nodes"]:
        if "role" in node:
            print(node["id"], "->", node["role"])

    print("Saved:", OUTPUT_PATH)

    return graph_data


if __name__ == "__main__":
    create_semantic_map()
