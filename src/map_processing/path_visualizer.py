import json

import cv2
import numpy as np

from .graph_builder import remove_small_components
from .map_config import create_map_parser, get_map_config
from .map_reader import detect_road_mask, load_map_image
from .parking_selector import (
    PARKING_PREFERENCES,
    PARKING_TYPES,
    load_parking_map,
    select_parking_from_map,
)


ROUTE_COLOR = (255, 232, 68)
OUTLINE_COLOR = (32, 28, 20)

# 요청한 다섯 가지 경로 조합을 기본 실행에서 순서대로 생성한다.
DEFAULT_ROUTE_CASES = (
    ("01_building_a", "building_a", "normal"),
    ("02_accessible", "any", "accessible"),
    ("03_exit", "exit", "normal"),
    ("04_building_b_accessible", "building_b", "accessible"),
    ("05_entrance_accessible", "entrance", "accessible"),
)


def node_point(graph, node_id):
    node = graph.nodes[node_id]
    return node["x"], node["y"]


def connect_waypoints(graph, node_route, road_mask):
    if not node_route:
        raise ValueError("연결할 노드 경로가 비어 있습니다.")

    waypoints = [node_point(graph, node_route[0])]
    for start, end in zip(node_route, node_route[1:]):
        edges = graph.get_edge_data(start, end)
        if not edges:
            raise ValueError(f"경로의 도로 간선을 찾을 수 없습니다. {start} → {end}")

        # 다익스트라와 같은 최소 가중치의 간선을 사용한다. 동률이면 저장 순서가 앞선 간선을 고른다.
        edge = min(edges.values(), key=lambda edge: edge["weight"])
        path = [tuple(point) for point in edge["path"]]
        if path and path[0] != waypoints[-1]:
            path.reverse()
        if not path or path[0] != waypoints[-1] or path[-1] != node_point(graph, end):
            raise ValueError(f"간선의 좌표 양끝이 도로 노드와 일치하지 않습니다. {start} → {end}")

        # 간선 사이의 공통점은 한 번만 기록하고 빈 픽셀 구간은 선분을 따라 채운다.
        for (x, y), (next_x, next_y) in zip(path, path[1:]):
            dx, dy = next_x - x, next_y - y
            steps = max(abs(dx), abs(dy))
            for step in range(1, steps + 1):
                waypoints.append((
                    round(x + dx * step / steps),
                    round(y + dy * step / steps),
                ))

    height, width = road_mask.shape
    for x, y in waypoints:
        if not (0 <= x < width and 0 <= y < height and road_mask[y, x]):
            raise ValueError(f"경로 좌표가 검출된 도로 영역을 벗어났습니다. ({x}, {y})")
    return waypoints


def build_route_data(map_id, map_data, graph, preference, road_mask, *, parking_type="normal"):
    if map_data["unit"] != "pixel":
        raise ValueError("지도 위 경로는 픽셀 단위의 도로 그래프가 필요합니다.")

    result = select_parking_from_map(map_data, graph, preference, parking_type=parking_type)
    node_route = result["route"]
    destination = result["selected"]
    space_id = graph.nodes[destination].get("region_id")
    if not space_id:
        raise ValueError(f"목적지 노드 {destination}의 주차칸 ID가 없습니다.")

    return {
        "map_id": map_id,
        "preference": preference,
        "parking_type": parking_type,
        "parking_space_id": space_id,
        "start_node_id": node_route[0],
        "destination_node_id": destination,
        "node_route": node_route,
        "coordinate_frame": "map_image",
        "origin": "top_left",
        "axis_directions": {"x": "right", "y": "down"},
        "unit": result["unit"],
        "total_distance": result["route_distance"],
        "waypoints": [list(point) for point in connect_waypoints(graph, node_route, road_mask)],
    }


def draw_marker(image, point, label, color, route_points):
    cv2.circle(image, point, 10, OUTLINE_COLOR, -1, cv2.LINE_AA)
    cv2.circle(image, point, 7, color, -1, cv2.LINE_AA)

    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.65
    (label_width, label_height), baseline = cv2.getTextSize(label, font, scale, 2)
    height, width = image.shape[:2]
    x, y = point
    label_x = x + 18
    if label_x + label_width > width - 12:
        label_x = max(8, x - label_width - 18)
    label_y = max(label_height + 6, min(y - 16, height - baseline - 6))
    label_left = label_x - 6
    label_right = label_x + label_width + 6
    route_x, route_y = route_points.T
    nearby_y = route_y[(route_x >= label_left - 5) & (route_x <= label_right + 5)]
    # 라벨이 경로를 가리면 점 아래쪽에도 놓아 본다.
    for candidate_y in (label_y, y + label_height + 16):
        if candidate_y + baseline + 3 >= height:
            continue
        overlaps = (nearby_y >= candidate_y - label_height - 10) & (
            nearby_y <= candidate_y + baseline + 8
        )
        if not np.any(overlaps):
            label_y = candidate_y
            break
    cv2.rectangle(
        image,
        (label_left, label_y - label_height - 5),
        (label_right, label_y + baseline + 3),
        OUTLINE_COLOR, -1,
    )
    cv2.putText(
        image, label, (label_x, label_y), font, scale, (255, 255, 255), 2, cv2.LINE_AA
    )


def draw_route(image, graph, route_data):
    preview = image.copy()
    waypoints = route_data["waypoints"]
    points = np.asarray(waypoints, dtype=np.int32)
    cv2.polylines(preview, [points], False, OUTLINE_COLOR, 10, cv2.LINE_AA)
    cv2.polylines(preview, [points], False, ROUTE_COLOR, 5, cv2.LINE_AA)
    for node_id in route_data["node_route"][1:-1]:
        cv2.circle(preview, node_point(graph, node_id), 4, OUTLINE_COLOR, -1, cv2.LINE_AA)
    draw_marker(preview, tuple(waypoints[0]), "START", (255, 255, 255), points)
    goal_label = f"GOAL {route_data['parking_space_id']}"
    draw_marker(preview, tuple(waypoints[-1]), goal_label, (45, 205, 255), points)
    caption = (
        f"{route_data.get('case_id', route_data['preference'])} | "
        f"{route_data['parking_type']} | {route_data['total_distance']:.2f} px"
    )
    (text_width, text_height), baseline = cv2.getTextSize(
        caption, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2
    )
    cv2.rectangle(preview, (8, 8), (text_width + 20, text_height + baseline + 20), OUTLINE_COLOR, -1)
    cv2.putText(
        preview, caption, (14, text_height + 12), cv2.FONT_HERSHEY_SIMPLEX,
        0.65, (255, 255, 255), 2, cv2.LINE_AA,
    )
    return preview


def export_routes(config, preferences=None, *, parking_type="normal"):
    map_data, graph = load_parking_map(config.semantic_path)
    if map_data.get("map_id", config.map_id) != config.map_id:
        raise ValueError(f"선택한 지도와 의미 지도 JSON의 map_id가 다릅니다. {config.map_id}")
    image = load_map_image(config.image_path)
    road_mask = remove_small_components(detect_road_mask(image))
    output_dir = config.output_dir / "routes"
    output_dir.mkdir(parents=True, exist_ok=True)
    outputs = []
    cases = DEFAULT_ROUTE_CASES
    if preferences is not None:
        cases = tuple(
            (f"{preference}_{parking_type}", preference, parking_type) for preference in preferences
        )
    for case_id, preference, case_parking_type in cases:
        route_data = build_route_data(
            config.map_id, map_data, graph, preference, road_mask, parking_type=case_parking_type
        )
        route_data["case_id"] = case_id
        preview = draw_route(image, graph, route_data)
        png_path = output_dir / f"{case_id}.png"
        json_path = output_dir / f"{case_id}.json"
        success, encoded = cv2.imencode(".png", preview)
        if not success:
            raise OSError(f"경로 미리보기 PNG를 인코딩하지 못했습니다. {png_path}")
        route_json = json.dumps(route_data, ensure_ascii=False, indent=2)
        png_path.write_bytes(encoded.tobytes())
        json_path.write_text(route_json + "\n", encoding="utf-8")
        outputs.append((png_path, json_path))
    return outputs


if __name__ == "__main__":
    parser = create_map_parser("최단 경로를 지도 PNG와 픽셀 waypoint JSON으로 저장합니다.")
    parser.add_argument(
        "--preference", choices=PARKING_PREFERENCES + ("any",), help="생략하면 기본 다섯 조합을 저장"
    )
    parser.add_argument("--parking-type", choices=PARKING_TYPES, help="단일 조건의 주차칸 유형")
    args = parser.parse_args()
    if args.parking_type and not args.preference:
        parser.error("--parking-type은 --preference와 함께 지정하세요.")
    config = get_map_config(args.map_id, output_root=args.output_dir)
    preferences = (args.preference,) if args.preference else None
    for png_path, json_path in export_routes(
        config, preferences, parking_type=args.parking_type or "normal"
    ):
        print(f"PNG  {png_path}")
        print(f"JSON {json_path}")
