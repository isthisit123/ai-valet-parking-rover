from bisect import bisect_right
from dataclasses import dataclass
from hashlib import sha256
import json
from math import atan2, cos, degrees, dist, isfinite, radians
from pathlib import Path

from src.map_processing.map_identity import AXES, WORLD_ORIGIN, configured_map_identity, require_map_identity

CONTROL_PERIOD_S = 0.10
POSE_MAX_AGE_S = 0.20
COMMAND_TIMEOUT_S = 0.30
GOAL_RADIUS_CM = 2.0
LOOKAHEAD_CM = 4.0
START_RADIUS_CM = 3.0
MAX_TRACK_ERROR_CM = 5.0
MAX_SPEED_CM_S = 8.0
MAX_ANGULAR_DEG_S = 45.0


def number(value):
    if type(value) not in (int, float) or not isfinite(value):
        raise ValueError("좌표·시각·속도는 유한한 숫자여야 합니다.")
    return float(value)


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    heading_deg: float
    captured_at: float

    @property
    def point(self):
        return self.x, self.y


@dataclass
class Route:
    map_id: str
    points: list

    def __post_init__(self):
        self.points = [p for i, p in enumerate(self.points) if i == 0 or p != self.points[i - 1]]
        if len(self.points) < 2:
            raise ValueError("서로 다른 waypoint가 두 개 이상 필요합니다.")
        self.lengths = [0.0]
        for start, end in zip(self.points, self.points[1:]):
            self.lengths.append(self.lengths[-1] + dist(start, end))

    def point_at(self, distance):
        if distance >= self.lengths[-1]:
            return self.points[-1]
        i = min(bisect_right(self.lengths, distance) - 1, len(self.points) - 2)
        fraction = (distance - self.lengths[i]) / (self.lengths[i + 1] - self.lengths[i])
        x, y = self.points[i]
        next_x, next_y = self.points[i + 1]
        return x + fraction * (next_x - x), y + fraction * (next_y - y)

    def project(self, point, start, end):
        """경로의 지정된 구간에서 가장 가까운 투영점의 거리와 진행량을 구한다."""
        best = (dist(point, self.point_at(start)), start)
        first = bisect_right(self.lengths, start) - 1
        for i in range(first, len(self.points) - 1):
            offset = self.lengths[i]
            if offset >= end:
                break
            length = self.lengths[i + 1] - offset
            x, y = self.points[i]
            next_x, next_y = self.points[i + 1]
            dx, dy = (next_x - x) / length, (next_y - y) / length
            along = (point[0] - x) * dx + (point[1] - y) * dy
            progress = min(max(offset + along, start), min(offset + length, end))
            projected = (x + (progress - offset) * dx, y + (progress - offset) * dy)
            best = min(best, (dist(point, projected), progress))
        return best

    def to_goal(self, goal_cm):
        error, progress = self.project(goal_cm, 0, self.lengths[-1])
        if error > 0.5 or progress <= 0:
            raise ValueError("--goal-cm은 출발점 이후 경로에서 0.5 cm 이내여야 합니다.")
        points = self.points[:bisect_right(self.lengths, progress)] + [self.point_at(progress)]
        return Route(self.map_id, points)


def parking_center_pixel(route_data, semantic):
    if semantic["map_id"] != route_data["map_id"] or semantic["unit"] != "pixel":
        raise ValueError("주차칸 진입에 사용할 픽셀 의미 지도가 경로와 다릅니다.")
    space_id = route_data["parking_space_id"]
    for region in semantic["regions"]:
        if region["id"] == space_id and region.get("node_id") == route_data["destination_node_id"]:
            x, y = region["center"]
            return number(x), number(y)
    raise ValueError(f"도착 도로 노드에 연결된 주차칸 중심을 찾을 수 없습니다. {space_id}")


def load_route(path, calibration=None, goal_cm=None, *, require_verified=False):
    data = json.loads(path.read_text(encoding="utf-8-sig"))
    map_id = data["map_id"]
    if not isinstance(map_id, str) or not map_id.strip():
        raise ValueError("경로에 map_id가 필요합니다.")
    points = [(number(x), number(y)) for x, y in data["waypoints"]]
    if not points:
        raise ValueError("경로에 waypoint가 없습니다.")
    frame = data["coordinate_frame"], data["unit"], data["origin"]
    if data["axis_directions"] != AXES:
        raise ValueError("경로 좌표축은 x 오른쪽, y 아래 방향이어야 합니다.")
    if require_verified and "parking_space_id" in data:
        from src.map_processing.map_config import get_map_config

        require_map_identity(configured_map_identity(get_map_config(map_id)), data, "주차 경로")
        if frame == ("world", "cm", WORLD_ORIGIN) and data.get("calibration_verified") is not True:
            raise ValueError("보정을 검증한 주차 경로 JSON이 필요합니다.")
    if frame == ("map_image", "pixel", "top_left"):
        if calibration is None:
            raise ValueError("픽셀 경로에는 검증된 --calibration JSON이 필요합니다.")
        if "parking_space_id" in data:
            from src.map_processing.map_config import get_map_config

            semantic_path = path.parent.parent / "semantic.json"
            if path.parent.name != "routes" or not semantic_path.is_file():
                semantic_path = get_map_config(map_id).semantic_path
            semantic = json.loads(semantic_path.read_text(encoding="utf-8-sig"))
            require_map_identity(semantic, data, "픽셀 경로")
            points.append(parking_center_pixel(data, semantic))
        points = pixel_to_world(map_id, points, calibration)
    elif frame != ("world", "cm", WORLD_ORIGIN):
        raise ValueError("지원하는 경로는 지도 픽셀 또는 전지 좌상단 원점의 월드 cm입니다.")
    route = Route(map_id, points)
    return route if goal_cm is None else route.to_goal(goal_cm)


def pixel_to_world(map_id, points, calibration):
    import cv2
    import numpy as np

    from src.map_processing.map_config import get_map_config
    from src.map_processing.planar_calibration import load_world_to_image, project_points

    image_bytes = get_map_config(map_id).image_path.read_bytes()
    image = cv2.imdecode(np.frombuffer(image_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("지도 이미지를 읽을 수 없습니다.")
    height, width = image.shape[:2]
    pixels = np.asarray(points)
    if (pixels < 0).any() or (pixels >= (width, height)).any():
        raise ValueError("경로 waypoint가 지도 이미지 범위를 벗어났습니다.")
    world_to_image, paper_size = load_world_to_image(
        calibration, map_id=map_id, image_size=(width, height),
        image_sha256=sha256(image_bytes).hexdigest(),
    )
    image_to_world = np.linalg.inv(world_to_image)
    denominators = pixels @ image_to_world[2, :2] + image_to_world[2, 2]
    if np.any(np.abs(denominators) < 1e-9):
        raise ValueError("월드 좌표로 변환할 수 없는 waypoint가 있습니다.")
    world_points = project_points(points, image_to_world)
    if not np.isfinite(world_points).all() or (
        (world_points < -1e-6).any() or (world_points > paper_size + 1e-6).any()
    ):
        raise ValueError("변환된 waypoint가 전지 범위를 벗어났습니다.")
    return [tuple(map(float, point)) for point in world_points]


class Follower:
    def __init__(self, route, max_speed):
        self.route = route
        self.max_speed = max_speed
        self.progress = 0.0
        self.started = False

    def drive(self, pose):
        if not self.started:
            if dist(pose.point, self.route.points[0]) > START_RADIUS_CM:
                raise ValueError("로버를 경로 출발점에서 3 cm 이내에 놓으세요.")
            self.started = True

        total = self.route.lengths[-1]
        # 현재 구간 주변만 확인해 교차로의 뒤쪽 경로로 건너뛰는 것을 제한한다.
        error, progress = self.route.project(pose.point, self.progress, min(total, self.progress + LOOKAHEAD_CM))
        if error > MAX_TRACK_ERROR_CM:
            raise ValueError("로버가 경로를 벗어났습니다.")
        self.progress = progress
        remaining = total - progress
        goal_distance = dist(pose.point, self.route.points[-1])
        if remaining <= GOAL_RADIUS_CM and goal_distance <= GOAL_RADIUS_CM:
            return None

        x, y = self.route.point_at(min(total, progress + LOOKAHEAD_CM))
        heading = degrees(atan2(y - pose.y, x - pose.x))
        error_deg = (heading - pose.heading_deg + 180) % 360 - 180
        speed = min(self.max_speed, 1.5 * max(remaining, goal_distance) ** 0.5)
        speed *= max(0.0, cos(radians(error_deg)))
        turn = min(MAX_ANGULAR_DEG_S, max(-MAX_ANGULAR_DEG_S, 2 * error_deg))
        return {"linear_cm_s": round(speed, 3), "angular_deg_s": round(turn, 3)}
