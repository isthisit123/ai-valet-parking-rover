import argparse
from hashlib import sha256
import json
from math import dist
from pathlib import Path

import cv2
import numpy as np

from src.map_processing.map_config import get_map_config
from src.map_processing.map_identity import configured_map_identity, map_identity, require_map_identity
from src.map_processing.planar_calibration import fit_floor_transform, load_world_to_image, project_points, quadrilateral
from .route_preview import route_segments
from .tracking import AXES, WORLD_ORIGIN, Route, parking_center_pixel


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
            **{k: v for k, v in region.items() if k not in ("bbox", "center", "area")},
            "center_cm": center,
            "box_corners_cm": convert([(x, y), (x + w, y), (x + w, y + h), (x, y + h)]),
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
        config = get_map_config(route["map_id"])
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
