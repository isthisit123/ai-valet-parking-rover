import argparse
from hashlib import sha256
import json
from math import dist
from pathlib import Path

import cv2
import networkx as nx
import numpy as np

from src.map_processing.floor_markers import calibrate_markers, evaluate_check
from src.map_processing.map_identity import configured_map_identity, map_identity, require_map_identity
from src.map_processing.parking_selector import get_region_by_id, load_parking_map
from src.map_processing.planar_calibration import fit_floor_transform, load_world_to_image, project_points, quadrilateral
from .route_preview import route_segments, save_world_route
from .tracking import AXES, WORLD_ORIGIN, Route, parking_center_pixel, route_map_config


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8-sig"))


def read_floor_points(path, map_id, image_bytes):
    spec = read_json(path)
    if spec["map_id"] != map_id:
        raise ValueError("기준점과 지도의 map_id가 다릅니다.")
    image_hash = spec.get("image_sha256")
    if image_hash is not None and image_hash != sha256(image_bytes).hexdigest():
        raise ValueError("기준점을 지정한 사진과 현재 지도 이미지가 다릅니다.")
    return spec


def capture_floor_points(config, image, layout, *, max_error_cm):
    if layout["map_id"] != config.map_id:
        raise ValueError("기준점 설정과 촬영 지도의 map_id가 다릅니다.")
    if "junction_check" in layout and "E" in layout["markers_cm"]:
        raise ValueError("확인점은 빨간 십자 E 또는 도로 교점 중 하나를 지정하세요.")
    points, report = calibrate_markers(image, layout, max_error_cm=max_error_cm)
    digest = sha256(config.image_path.read_bytes()).hexdigest()
    points["image_sha256"] = digest
    report["source_frame"].update(image=config.image_path.name, sha256=digest)
    return points, report


def parking_junction_pixel(config, parking_space_id):
    semantic, graph = load_parking_map(config.semantic_path, config=config)
    entrance, exit_node, parking = (
        get_region_by_id(semantic, facility)["node_id"]
        for facility in ("entrance", "exit", parking_space_id)
    )
    try:
        main_road = nx.shortest_path(graph, entrance, exit_node, weight="weight")
        from_parking = [nx.shortest_path(graph, parking, gate, weight="weight")
                        for gate in (entrance, exit_node)]
    except (nx.NetworkXNoPath, nx.NodeNotFound) as error:
        raise ValueError(f"{parking_space_id}에서 EN–EX 도로로 연결되는 경로가 없습니다.") from error
    main_nodes = set(main_road)
    joins = [next(node for node in path if node in main_nodes) for path in from_parking]
    junction = joins[0]
    if (junction != joins[1] or junction == parking or junction not in main_road[1:-1]
            or len(graph[junction]) < 3 or graph.nodes[junction]["type"] != "junction"):
        raise ValueError(f"{parking_space_id} 도로가 EN–EX 도로에 합류하는 교점을 하나로 찾을 수 없습니다.")
    node = graph.nodes[junction]
    return [node["x"], node["y"]]


def verify_junction(config, spec, report, check):
    if not isinstance(check, dict) or not isinstance(check.get("parking_space_id"), str):
        raise ValueError("도로 확인점에 parking_space_id와 실측 world_cm을 지정하세요.")
    parking_space_id = check["parking_space_id"]
    if not parking_space_id.startswith("space_"):
        raise ValueError("도로 확인점에는 주차칸 ID(space_…)가 필요합니다.")
    world_cm = np.asarray(check["world_cm"], dtype=float)
    outer_cm = np.float32([point["world_cm"] for point in spec["control_points"]])
    if (world_cm.shape != (2,) or not np.isfinite(world_cm).all()
            or cv2.pointPolygonTest(outer_cm, tuple(world_cm), False) <= 0):
        raise ValueError("도로 확인점의 실측 [x, y] cm는 A~D 안쪽이어야 합니다.")
    pixel = parking_junction_pixel(config, parking_space_id)
    outer_px = np.float32([point["pixel"] for point in spec["control_points"]])
    if cv2.pointPolygonTest(outer_px, tuple(pixel), False) <= 0:
        raise ValueError("사진에서 찾은 도로 확인점이 십자 A~D 밖에 있습니다.")
    candidate = report["candidate_image_to_world"]
    validation = evaluate_check(pixel, world_cm, np.asarray(candidate),
                                tolerance_cm=report["validation"]["tolerance_cm"],
                                method="independent_road_junction")
    validation.update(parking_space_id=parking_space_id, pixel=pixel, world_cm=world_cm.tolist())
    report["validation"] = validation
    if validation["passed"]:
        report["image_to_world"] = report.pop("candidate_image_to_world")


def floor_transform(map_id, image_bytes, image_size, *, points=None, calibration=None):
    if calibration:
        inverse, size = load_world_to_image(
            calibration, map_id=map_id, image_size=image_size,
            image_sha256=sha256(image_bytes).hexdigest(),
        )
        return np.linalg.inv(inverse), size.tolist()

    spec = read_floor_points(points, map_id, image_bytes)
    size = np.asarray(spec["paper_size_cm"], dtype=float)
    if size.shape != (2,) or not np.isfinite(size).all() or (size <= 0).any():
        raise ValueError("전지의 가로·세로 실측값을 입력하세요.")
    corners = quadrilateral(spec["paper_corners_px"], "전지 모서리")
    if cv2.contourArea(corners, oriented=True) <= 0 or (corners < 0).any() or (corners >= image_size).any():
        raise ValueError("사진 안의 전지 모서리를 왼쪽 위부터 시계 방향으로 입력하세요.")
    width, height = size
    world = np.float32([[0, 0], [width, 0], [width, height], [0, height]])
    return fit_floor_transform(corners, world, corners), size.tolist()


def build_map(semantic, source_route, matrix, size, *, verified):
    if semantic["map_id"] != source_route["map_id"] or semantic["unit"] != "pixel":
        raise ValueError("경로에 해당하는 픽셀 의미 지도가 필요합니다.")
    require_map_identity(semantic, source_route, "픽셀 경로")
    if (source_route["coordinate_frame"], source_route["unit"], source_route["origin"],
            source_route["axis_directions"]) != ("map_image", "pixel", "top_left", AXES):
        raise ValueError("지도 픽셀 경로를 지정하세요.")

    def convert(points):
        projected = project_points(points, matrix)
        if not np.isfinite(projected).all():
            raise ValueError("변환 후 유한하지 않은 좌표가 있습니다.")
        return projected.tolist()

    nodes = []
    for node in semantic["nodes"]:
        xy, = convert([(node["x"], node["y"])])
        nodes.append({**{k: v for k, v in node.items() if k not in ("x", "y")}, "xy_cm": xy})
    edges = []
    for edge in semantic["edges"]:
        points = convert(edge["path"])
        length = sum(dist(a, b) for a, b in zip(points, points[1:]))
        edges.append({"from": edge["from"], "to": edge["to"], "length_cm": length, "path_cm": points})
    regions = []
    for region in semantic["regions"]:
        x, y, w, h = region["bbox"]
        center, = convert([region["center"]])
        regions.append({
            **{k: v for k, v in region.items() if k not in ("bbox", "center", "area", "polygon_px")},
            "center_cm": center,
            "box_corners_cm": convert([(x, y), (x + w, y), (x + w, y + h), (x, y + h)]),
            **({"polygon_cm": convert(region["polygon_px"])} if "polygon_px" in region else {}),
        })
    road_points = source_route["waypoints"]
    center_px = parking_center_pixel(source_route, semantic)
    points = list(map(tuple, convert([*road_points, center_px])))
    road_end, parking_center = points[-2:]
    route = Route(source_route["map_id"], points)
    width, height = size
    if not all(0 <= x <= width and 0 <= y <= height for x, y in route.points):
        raise ValueError("변환한 경로가 전지 범위를 벗어났습니다.")
    selected = next(region for region in regions if region["id"] == source_route["parking_space_id"])
    return {
        **map_identity(source_route),
        "coordinate_frame": "world" if verified else "world_estimate",
        "unit": "cm",
        "origin": WORLD_ORIGIN,
        "axis_directions": AXES,
        "paper_size_cm": size,
        "calibration_verified": verified,
        "nodes": nodes,
        "edges": edges,
        "regions": regions,
        "route": {
            "parking_space_id": selected["id"],
            **{key: source_route[key] for key in ("occupancy_status", "occupancy_observed_at")
               if key in source_route},
            "node_route": source_route["node_route"],
            "destination_node_id": source_route["destination_node_id"],
            "start_cm": route.points[0],
            "road_end_cm": road_end,
            "goal_cm": route.points[-1],
            "parking_space_center_cm": parking_center,
            "parking_entry_distance_cm": dist(road_end, parking_center),
            "total_distance_cm": route.lengths[-1],
            "waypoints_cm": route.points,
            "segments": [
                {"distance_cm": s["distance"], "end_cm": s["end"], "heading_deg": s["heading_deg"],
                 "turn_after_deg": s["turn_after_deg"]}
                for s in route_segments(route, 1)
            ],
        },
    }


def draw_map(image, matrix, data, route_name):
    width, height = data["paper_size_cm"]
    scale = min(10, 1250 / width, 900 / height)
    left, top = 60, 94
    w, h = round(width * scale), round(height * scale)
    warp = np.diag([scale, scale, 1]) @ matrix
    floor = cv2.warpPerspective(image, warp, (w + 1, h + 1))
    canvas = np.full((top + h + 52, left + w + 24, 3), 249, dtype=np.uint8)
    canvas[top:top + h + 1, left:left + w + 1] = floor

    def pixel(point):
        return left + round(point[0] * scale), top + round(point[1] * scale)

    def label(value, point, size=0.52, color=(40, 40, 40)):
        cv2.putText(canvas, value, point, cv2.FONT_HERSHEY_SIMPLEX, size, color, 1, cv2.LINE_AA)

    for x in range(0, int(width) + 1, 10):
        a, b = pixel((x, 0)), pixel((x, height))
        cv2.line(canvas, a, b, (169, 169, 169), 1, cv2.LINE_AA)
        label(str(x), (a[0] - 6, a[1] - 8), 0.43)
    for y in range(0, int(height) + 1, 10):
        a, b = pixel((0, y)), pixel((width, y))
        cv2.line(canvas, a, b, (169, 169, 169), 1, cv2.LINE_AA)
        label(str(y), (a[0] - 30, a[1] + 5), 0.43)
    cv2.rectangle(canvas, pixel((0, 0)), pixel((width, height)), (65, 65, 65), 1)

    route = data["route"]
    line = np.asarray([pixel(p) for p in route["waypoints_cm"]], dtype=np.int32)
    cv2.polylines(canvas, [line], False, (255, 255, 255), 8, cv2.LINE_AA)
    cv2.polylines(canvas, [line], False, (110, 40, 205), 4, cv2.LINE_AA)
    markers = [
        ("Start", route["start_cm"], (185, 100, 0)),
        ("Road end " + route["destination_node_id"], route["road_end_cm"], (75, 25, 180)),
        ("Goal: space center", route["goal_cm"], (45, 110, 25)),
    ]
    for name, point, color in markers:
        x, y = pixel(point)
        cv2.circle(canvas, (x, y), 8, (255, 255, 255), -1, cv2.LINE_AA)
        cv2.circle(canvas, (x, y), 5, color, -1, cv2.LINE_AA)
        cv2.rectangle(canvas, (x + 10, y - 17), (x + 18 + len(name) * 10, y + 4), (249, 249, 249), -1)
        label(name, (x + 13, y - 2), color=color)
    status = "VERIFIED" if data["calibration_verified"] else "ESTIMATE - NOT FOR DRIVING"
    label(f"{route_name} | {status}", (left, 30), 0.72)
    label(f"{width:g} x {height:g} cm | Route {route['total_distance_cm']:.2f} cm | Grid 10 cm", (left, 55))
    label("x right (+), y down (+), clockwise angles positive", (left, top + h + 32))
    return canvas


def export_cm_map(image, matrix, data, route_name, output_dir):
    canvas = draw_map(image, matrix, data, route_name)
    ok, png = cv2.imencode(".png", canvas)
    if not ok:
        raise ValueError("미리보기 이미지를 인코딩할 수 없습니다.")
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / f"{route_name}_map_cm.json"
    png_path = json_path.with_suffix(".png")
    if json_path.exists() or png_path.exists():
        raise ValueError("출력 파일이 이미 있습니다. 다른 --output-dir을 지정하세요.")
    with json_path.open("x", encoding="utf-8") as file:
        file.write(json.dumps(data, ensure_ascii=False, allow_nan=False, indent=2) + "\n")
    with png_path.open("xb") as file:
        file.write(png.tobytes())
    return json_path, png_path


def export_cm_routes(semantic, image, matrix, size, pixel_routes, preview_dir, world_dir, *, verified):
    if verified:
        world_dir.mkdir(parents=True, exist_ok=True)
    for source in pixel_routes:
        data = build_map(semantic, read_json(source), matrix, size, verified=verified)
        json_path, png_path = export_cm_map(image, matrix, data, source.stem, preview_dir)
        print(f"{source.stem}: {data['route']['total_distance_cm']:.2f} cm")
        print(f"  PNG: {png_path}")
        print(f"  JSON: {json_path}")
        if verified:
            route_path = world_dir / f"{source.stem}_cm.json"
            route = Route(semantic["map_id"], data["route"]["waypoints_cm"])
            save_world_route(source, route_path, route, verified=True, waypoint_target="parking_center")
            print(f"  검증된 cm 경로: {route_path}")
    return preview_dir


def main():
    parser = argparse.ArgumentParser(description="cm 좌표계의 지도 PNG와 확인용 JSON을 저장합니다.")
    parser.add_argument("--route", type=Path, required=True)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--points", type=Path, help="전지 모서리와 실측 크기로 추정")
    source.add_argument("--calibration", type=Path, help="검증을 통과한 보정 JSON")
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    try:
        route = read_json(args.route)
        config = route_map_config(args.route, route["map_id"])
        source_image = config.image_path.read_bytes()
        image = cv2.imdecode(np.frombuffer(source_image, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("지도 이미지를 읽을 수 없습니다.")
        matrix, size = floor_transform(
            route["map_id"], source_image, image.shape[1::-1],
            points=args.points, calibration=args.calibration,
        )
        semantic = read_json(config.semantic_path)
        require_map_identity(configured_map_identity(config), semantic, "의미 지도")
        data = build_map(semantic, route, matrix, size, verified=args.calibration is not None)
        json_path, png_path = export_cm_map(image, matrix, data, args.route.stem, args.output_dir)
    except (OSError, KeyError, TypeError, ValueError, StopIteration, cv2.error) as error:
        parser.error(str(error))
    print(f"{json_path}\n{png_path}")


if __name__ == "__main__":
    main()
