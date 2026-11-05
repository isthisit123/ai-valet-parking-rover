import json
import math

import networkx as nx

from .map_reader import BASE_DIR


MAP_PATH = (
    BASE_DIR / "data" / "graphs" / "parking_map_v1_semantic.json"
)


def load_parking_map():
    # 의미 정보가 포함된 지도 불러오기
    with open(MAP_PATH, "r", encoding="utf-8") as f:
        map_data = json.load(f)

    # NetworkX 그래프 생성
    graph = nx.MultiGraph()

    for node in map_data["nodes"]:
        graph.add_node(node["id"], **node)

    for edge in map_data["edges"]:
        graph.add_edge(
            edge["from"],
            edge["to"],
            weight=edge["weight"],
            path=edge["path"]
        )

    return map_data, graph


def get_node_by_role(map_data, role):
    # 지정한 역할을 가진 노드 찾기
    for node in map_data["nodes"]:
        if node.get("role") == role:
            return node

    raise ValueError(f"Node not found: {role}")


def get_region_by_id(map_data, region_id):
    # 건물 등 특정 영역 찾기
    for region in map_data["regions"]:
        if region["id"] == region_id:
            return region

    raise ValueError(f"Region not found: {region_id}")


def distance_to_building(node, building):
    # 주차칸 노드에서 건물 사각형까지의 직선거리
    x, y = node["x"], node["y"]

    bx, by, bw, bh = building["bbox"]

    dx = max(bx - x, 0, x - (bx + bw))
    dy = max(by - y, 0, y - (by + bh))

    return math.hypot(dx, dy)


def select_parking(preference):
    map_data, graph = load_parking_map()

    # 입구 노드 확인
    entrance = get_node_by_role(map_data, "entrance")
    start_id = entrance["id"]

    # 현재는 일반 주차칸만 선택
    # YOLO 연결 전이므로 모든 일반 주차칸이 비었다고 가정
    parking_nodes = [
        node for node in map_data["nodes"]
        if node.get("role") == "parking_normal"
    ]

    if not parking_nodes:
        raise ValueError("No normal parking spaces found")

    candidates = []

    for parking in parking_nodes:
        parking_id = parking["id"]

        # 출구 또는 입구까지의 도로 최단 거리
        if preference in ("exit", "entrance"):
            target = get_node_by_role(map_data, preference)

            score = nx.dijkstra_path_length(
                graph,
                parking_id,
                target["id"],
                weight="weight"
            )

        # 건물까지의 직선거리
        elif preference in ("building_a", "building_b"):
            building = get_region_by_id(
                map_data, preference
            )

            score = distance_to_building(
                parking, building
            )

        else:
            raise ValueError(
                f"Unknown preference: {preference}"
            )

        candidates.append((score, parking_id))

    # 점수가 가장 작은 주차칸 선택
    best_score, best_parking = min(candidates)

    # 입구에서 선택된 주차칸까지 최단 경로 계산
    route = nx.dijkstra_path(
        graph,
        start_id,
        best_parking,
        weight="weight"
    )

    route_distance = nx.dijkstra_path_length(
        graph,
        start_id,
        best_parking,
        weight="weight"
    )

    return {
        "preference": preference,
        "selected": best_parking,
        "score": round(best_score, 2),
        "route": route,
        "route_distance": round(route_distance, 2),
        "unit": map_data["unit"]
    }


if __name__ == "__main__":
    # 조건별 주차칸 선택 테스트
    preferences = [
        "exit",
        "entrance",
        "building_a",
        "building_b"
    ]

    for preference in preferences:
        result = select_parking(preference)

        print()
        print("Preference:", result["preference"])
        print("Selected:", result["selected"])
        print("Score:", result["score"])
        print("Route:", " -> ".join(result["route"]))
        print("Route distance:", result["route_distance"])