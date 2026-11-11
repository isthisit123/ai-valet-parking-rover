import argparse
import json
from math import atan2, degrees, dist, isfinite
from pathlib import Path

from .tracking import AXES, WORLD_ORIGIN, Route, load_route, number


def save_world_route(source, output, route):
    original = json.loads(source.read_text(encoding="utf-8-sig"))
    metadata = ("case_id", "preference", "parking_type", "parking_space_id",
                "start_node_id", "destination_node_id", "node_route")
    data = {key: original[key] for key in metadata if key in original}
    data.update(
        map_id=route.map_id,
        coordinate_frame="world",
        unit="cm",
        origin=WORLD_ORIGIN,
        axis_directions=AXES,
        total_distance=route.lengths[-1],
        waypoints=route.points,
    )
    # 원본 경로, 보정 파일, 이미 만든 결과를 덮어쓰지 않는다.
    serialized = json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    with output.open("x", encoding="utf-8") as file:
        file.write(serialized)


def corner_indices(points, tolerance):
    """미리보기용으로 직선에서 크게 벗어나는 원본 waypoint를 남긴다."""
    kept = {0, len(points) - 1}
    pending = [(0, len(points) - 1)]
    while pending:
        start, end = pending.pop()
        x, y = points[start]
        dx, dy = points[end][0] - x, points[end][1] - y
        length_squared = dx * dx + dy * dy
        furthest = tolerance
        split = None
        for i in range(start + 1, end):
            px, py = points[i]
            along = ((px - x) * dx + (py - y) * dy) / length_squared if length_squared else 0
            fraction = min(1, max(0, along))
            error = dist((px, py), (x + fraction * dx, y + fraction * dy))
            if error > furthest:
                furthest, split = error, i
        if split is not None:
            kept.add(split)
            pending.extend(((start, split), (split, end)))
    return sorted(kept)


def route_segments(route, tolerance):
    corners = corner_indices(route.points, tolerance)
    spans = list(zip(corners, corners[1:]))
    headings = []
    for start, end in spans:
        x, y = route.points[start]
        next_x, next_y = route.points[end]
        headings.append(degrees(atan2(next_y - y, next_x - x)))
    segments = []
    for i, (start, end) in enumerate(spans):
        turn = None
        if i + 1 < len(headings):
            turn = (headings[i + 1] - headings[i] + 180) % 360 - 180
        segments.append({
            "distance": route.lengths[end] - route.lengths[start],
            "end": route.points[end],
            "heading_deg": headings[i],
            "turn_after_deg": turn,
        })
    return segments


def main():
    parser = argparse.ArgumentParser(description="경로의 구간별 거리와 회전을 모터 연결 없이 표시합니다.")
    parser.add_argument("--route", type=Path, required=True)
    units = parser.add_mutually_exclusive_group()
    units.add_argument("--calibration", type=Path, help="지도 픽셀 경로를 cm로 변환할 검증된 보정 JSON")
    units.add_argument("--pixels", action="store_true", help="보정 없이 지도 픽셀 단위로 확인")
    parser.add_argument("--output", type=Path, help="월드 cm 경로를 새 JSON 파일로 저장")
    parser.add_argument("--tolerance", type=float, help="회전점 정리 허용 오차 (기본 8 px 또는 1 cm)")
    args = parser.parse_args()

    try:
        if args.pixels:
            if args.output:
                raise ValueError("--output은 cm 경로만 저장합니다. 픽셀 경로에는 --calibration을 지정하세요.")
            data = json.loads(args.route.read_text(encoding="utf-8-sig"))
            if (data["coordinate_frame"], data["unit"], data["origin"], data["axis_directions"]) != (
                "map_image", "pixel", "top_left", AXES
            ):
                raise ValueError("--pixels는 지도 픽셀 경로에 사용하세요.")
            route = Route(data["map_id"], [(number(x), number(y)) for x, y in data["waypoints"]])
        else:
            route = load_route(args.route, args.calibration)
        unit = "px" if args.pixels else "cm"
        tolerance = args.tolerance if args.tolerance is not None else (8 if args.pixels else 1)
        if not isfinite(tolerance) or tolerance <= 0:
            raise ValueError("--tolerance는 양수여야 합니다.")
        if args.output:
            save_world_route(args.route, args.output, route)
    except (OSError, KeyError, TypeError, ValueError) as error:
        parser.error(str(error))

    segments = route_segments(route, tolerance)
    print(f"{args.route.stem} | 전체 {route.lengths[-1]:.2f} {unit} | 회전점 허용 오차 {tolerance:g} {unit}")
    print(f"출발 방향 {segments[0]['heading_deg']:+.1f}° (오른쪽 0°, 시계 방향 +)")
    for i, segment in enumerate(segments, start=1):
        distance = segment["distance"]
        x, y = segment["end"]
        action = "목표점"
        turn = segment["turn_after_deg"]
        if turn is not None:
            action = f"{'우회전' if turn > 0 else '좌회전'} {abs(turn):.1f}°" if abs(turn) >= 0.05 else "직진 계속"
        print(f"{i:2d}. {distance:7.2f} {unit} 이동 → ({x:.2f}, {y:.2f}) → {action}")
    if args.output:
        print(f"cm 경로 저장: {args.output}")


if __name__ == "__main__":
    main()
