import json
import math

import networkx as nx

from .map_config import DEFAULT_MAP, create_map_parser, get_map_config


PARKING_PREFERENCES = ("exit", "entrance", "building_a", "building_b")
PARKING_TYPES = ("normal", "accessible")


def load_parking_map(map_path=DEFAULT_MAP.semantic_path):
    # 의미 정보가 포함된 지도 불러오기
    with open(map_path, "r", encoding="utf-8") as f:
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


def select_parking(preference, *, map_path=DEFAULT_MAP.semantic_path, parking_type="normal"):
    map_data, graph = load_parking_map(map_path)
    return select_parking_from_map(map_data, graph, preference, parking_type=parking_type)


def select_parking_from_map(map_data, graph, preference, *, parking_type="normal"):
    # 입구 노드 확인
    entrance = get_node_by_role(map_data, "entrance")
    start_id = entrance["id"]

    if parking_type not in PARKING_TYPES:
        raise ValueError(f"Unknown parking type: {parking_type}")

    # 요청한 유형의 주차칸만 후보에 넣는다. 기본 유형은 일반 주차칸이다.
    # 점유 정보 연동 전에는 해당 후보가 비어 있다고 가정한다.
    parking_role = f"parking_{parking_type}"
    parking_nodes = [
        node for node in map_data["nodes"]
        if node.get("role") == parking_role
    ]

    if not parking_nodes:
        raise ValueError(f"No {parking_type} parking spaces found")

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

        elif preference == "any":
            score = 0.0

        else:
            raise ValueError(
                f"Unknown preference: {preference}"
            )

        candidates.append((score, parking_id))

    # 조건이 없는 경우에는 고유 주차칸 ID로 결정해 재실행 결과를 일정하게 유지한다.
    if preference == "any":
        best_score, best_parking = min(
            candidates,
            key=lambda candidate: graph.nodes[candidate[1]].get("region_id", candidate[1]),
        )
    else:
        best_score, best_parking = min(candidates)

    # 입구에서 선택된 주차칸까지 최단 경로 계산
    route_distance, route = nx.single_source_dijkstra(
        graph, start_id, target=best_parking, weight="weight"
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
    parser = create_map_parser("조건별 주차칸과 최단 경로를 계산합니다.")
    parser.add_argument(
        "--parking-type", choices=PARKING_TYPES, default="normal",
        help="선택할 주차칸 유형 (기본값 normal)",
    )
    args = parser.parse_args()
    config = get_map_config(args.map_id, output_root=args.output_dir)
    map_data, graph = load_parking_map(config.semantic_path)

    for preference in PARKING_PREFERENCES:
        result = select_parking_from_map(map_data, graph, preference, parking_type=args.parking_type)

        print()
        print("Preference:", result["preference"])
        print("Selected:", result["selected"])
        print("Score:", result["score"])
        print("Route:", " -> ".join(result["route"]))
        print("Route distance:", result["route_distance"])
