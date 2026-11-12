import json
import math
from pathlib import Path

import networkx as nx

from .map_config import DEFAULT_MAP, create_map_parser, get_map_config
from .map_identity import configured_map_identity, require_map_identity
from .occupancy import DEFAULT_OCCUPANCY_TTL, load_occupancy, occupancy_states


PARKING_PREFERENCES = ("exit", "entrance", "building_a", "building_b")
PARKING_TYPES = ("normal", "accessible")


def load_parking_map(map_path=DEFAULT_MAP.semantic_path, *, config=None):
    # 의미 정보가 포함된 지도 불러오기
    with open(map_path, "r", encoding="utf-8") as f:
        map_data = json.load(f)
    if config is None and Path(map_path).resolve() == DEFAULT_MAP.semantic_path:
        config = DEFAULT_MAP
    if config is not None:
        require_map_identity(configured_map_identity(config), map_data, "의미 지도")

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


def select_parking(preference, *, map_path=DEFAULT_MAP.semantic_path, parking_type="normal",
                   occupancy=None, max_age_seconds=DEFAULT_OCCUPANCY_TTL, now=None):
    map_data, graph = load_parking_map(map_path)
    return select_parking_from_map(
        map_data, graph, preference, parking_type=parking_type,
        occupancy=occupancy, max_age_seconds=max_age_seconds, now=now,
    )


def select_parking_from_map(map_data, graph, preference, *, parking_type="normal",
                            occupancy=None, max_age_seconds=DEFAULT_OCCUPANCY_TTL, now=None):
    # 입구 노드 확인
    entrance = get_node_by_role(map_data, "entrance")
    start_id = entrance["id"]

    if parking_type not in PARKING_TYPES:
        raise ValueError(f"Unknown parking type: {parking_type}")

    # 요청한 유형의 주차칸만 후보에 넣는다. 기본 유형은 일반 주차칸이다.
    # 점유 정보가 없으면 기존 정적 지도 시뮬레이션으로 동작한다.
    parking_role = f"parking_{parking_type}"
    parking_nodes = [
        node for node in map_data["nodes"]
        if node.get("role") == parking_role
    ]

    if occupancy is not None:
        states = occupancy_states(map_data, occupancy, max_age_seconds=max_age_seconds, now=now)
        parking_nodes = [node for node in parking_nodes if states[node["region_id"]] == "free"]
        if not parking_nodes:
            raise ValueError(f"최신 free 상태의 {parking_type} 주차칸이 없습니다.")

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

    result = {
        "preference": preference,
        "selected": best_parking,
        "score": round(best_score, 2),
        "route": route,
        "route_distance": round(route_distance, 2),
        "unit": map_data["unit"]
    }
    if occupancy is not None:
        result.update(
            parking_space_id=graph.nodes[best_parking]["region_id"],
            occupancy_status="free", occupancy_observed_at=occupancy["observed_at"],
        )
    return result


if __name__ == "__main__":
    parser = create_map_parser("조건별 주차칸과 최단 경로를 계산합니다.")
    parser.add_argument(
        "--parking-type", choices=PARKING_TYPES, default="normal",
        help="선택할 주차칸 유형 (기본값 normal)",
    )
    parser.add_argument("--occupancy", help="주차칸 ID 기반 점유 JSON 경로")
    parser.add_argument("--occupancy-ttl", type=float, default=DEFAULT_OCCUPANCY_TTL, help="점유 유효 시간(초)")
    args = parser.parse_args()
    config = get_map_config(args.map_id, output_root=args.output_dir)
    map_data, graph = load_parking_map(config.semantic_path, config=config)
    occupancy = load_occupancy(args.occupancy) if args.occupancy else None
    if occupancy is None:
        print("정적 지도 시뮬레이션: 실제 빈자리 여부는 확인하지 않습니다.")

    for preference in PARKING_PREFERENCES:
        result = select_parking_from_map(
            map_data, graph, preference, parking_type=args.parking_type,
            occupancy=occupancy, max_age_seconds=args.occupancy_ttl,
        )

        print()
        print("Preference:", result["preference"])
        print("Selected:", result["selected"])
        if occupancy is not None:
            print("Parking space:", result["parking_space_id"], result["occupancy_status"])
        print("Score:", result["score"])
        print("Route:", " -> ".join(result["route"]))
        print("Route distance:", result["route_distance"])
