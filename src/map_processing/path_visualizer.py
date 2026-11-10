import json

import cv2
import numpy as np

from .graph_builder import remove_small_components
from .map_config import create_map_parser, get_map_config
from .map_reader import detect_road_mask, load_map_image
from .parking_selector import (
    PARKING_PREFERENCES,
    load_parking_map,
    select_parking_from_map,
)


ROUTE_COLOR = (255, 232, 68)
OUTLINE_COLOR = (32, 28, 20)


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


def build_route_data(map_id, map_data, graph, preference, road_mask):
    if map_data["unit"] != "pixel":
        raise ValueError("지도 위 경로는 픽셀 단위의 도로 그래프가 필요합니다.")

    result = select_parking_from_map(map_data, graph, preference)
    node_route = result["route"]
    destination = result["selected"]
    space_id = graph.nodes[destination].get("region_id")
    if not space_id:
        raise ValueError(f"목적지 노드 {destination}의 주차칸 ID가 없습니다.")

    return {
        "map_id": map_id,
        "preference": preference,
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


def draw_marker(image, point, label, color):
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
    label_y = y + label_height + 16 if y < height / 2 else y - 16
    cv2.rectangle(
        image,
        (label_x - 6, label_y - label_height - 5),
        (label_x + label_width + 6, label_y + baseline + 3),
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
    draw_marker(preview, tuple(waypoints[0]), "START", (255, 255, 255))
    goal_label = (
        f"GOAL {route_data['parking_space_id']} | {route_data['preference']} | "
        f"{route_data['total_distance']:.2f} px"
    )
    draw_marker(preview, tuple(waypoints[-1]), goal_label, (45, 205, 255))
    return preview


def export_routes(config, preferences=PARKING_PREFERENCES):
    map_data, graph = load_parking_map(config.semantic_path)
    if map_data.get("map_id", config.map_id) != config.map_id:
        raise ValueError(f"선택한 지도와 의미 지도 JSON의 map_id가 다릅니다. {config.map_id}")
    image = load_map_image(config.image_path)
    road_mask = remove_small_components(detect_road_mask(image))
    output_dir = config.output_dir / "routes"
    output_dir.mkdir(parents=True, exist_ok=True)
    outputs = []
    for preference in preferences:
        route_data = build_route_data(config.map_id, map_data, graph, preference, road_mask)
        preview = draw_route(image, graph, route_data)
        stem = f"{config.map_id}_route_{preference}"
        png_path = output_dir / f"{stem}.png"
        json_path = output_dir / f"{stem}.json"
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
    parser.add_argument("--preference", choices=PARKING_PREFERENCES, help="생략하면 네 조건을 모두 저장")
    args = parser.parse_args()
    config = get_map_config(args.map_id, output_root=args.output_dir)
    preferences = (args.preference,) if args.preference else PARKING_PREFERENCES
    for png_path, json_path in export_routes(config, preferences):
        print(f"PNG  {png_path}")
        print(f"JSON {json_path}")
