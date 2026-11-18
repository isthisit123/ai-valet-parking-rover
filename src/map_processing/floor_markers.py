"""고정 빨간 십자를 검출해 카메라 픽셀을 전지 cm 좌표에 연결한다."""

import cv2
import numpy as np

from .map_identity import AXES, WORLD_ORIGIN
from .planar_calibration import fit_floor_transform, project_points, quadrilateral


MARKER_RED_RANGES = (((0, 65, 65), (12, 255, 255)), ((168, 65, 65), (179, 255, 255)))
CROSS_APPROXIMATION_FRACTIONS = (0.012, 0.018, 0.025, *np.linspace(0.012, 0.025, 27)[1:-1])
# 실측·십자 검출 오차를 외곽까지 외삽할 때의 작은 이탈만 허용한다.
# 보정점이나 외곽을 강제로 화면 안으로 옮기지 않으며 독립점 검증도 유지한다.
PROJECTED_PAPER_BORDER_TOLERANCE_RATIO = 0.02


def cross_center(contour):
    # 십자의 안쪽 꼭짓점 네 개는 세 꼭짓점 간격으로 나타난다.
    # 안쪽 사각형의 대각선 교점은 원근 변환 뒤에도 십자의 중심과 일치한다.
    perimeter = cv2.arcLength(contour, True)
    # 테이프 가장자리와 저해상도 계단 때문에 꼭짓점 수가 건너뛰면 중간 오차도 검사한다.
    for fraction in CROSS_APPROXIMATION_FRACTIONS:
        polygon = cv2.approxPolyDP(contour, perimeter * fraction, True)
        if len(polygon) != 12:
            continue
        points = polygon.reshape(-1, 2).astype(float)
        incoming = points - np.roll(points, 1, axis=0)
        outgoing = np.roll(incoming, -1, axis=0)
        turns = incoming[:, 0] * outgoing[:, 1] - incoming[:, 1] * outgoing[:, 0]
        inner = np.flatnonzero(turns * cv2.contourArea(polygon, oriented=True) < 0)
        if len(inner) != 4 or not np.all(np.diff(np.r_[inner, inner[0] + 12]) == 3):
            continue
        a, b, c, d = points[inner]
        diagonal = np.column_stack((c - a, b - d))
        if abs(np.linalg.det(diagonal)) < 1e-6:
            continue
        t, u = np.linalg.solve(diagonal, b - a)
        if 0 < t < 1 and 0 < u < 1:
            return a + t * (c - a)
    return None


def clockwise_corners(four_points, name):
    points = sorted(four_points, key=lambda p: p[1])
    top_left, top_right = sorted(points[:2], key=lambda p: p[0])
    bottom_left, bottom_right = sorted(points[2:], key=lambda p: p[0])
    corners = quadrilateral([top_left, top_right, bottom_right, bottom_left], name)
    if cv2.contourArea(corners, oriented=True) <= 0:
        raise ValueError(f"{name}의 왼쪽 위부터 시계 방향 배치를 확인하세요.")
    return corners


def evaluate_check(pixel, world_cm, matrix, *, tolerance_cm, method):
    estimated, = project_points([pixel], matrix)
    error = float(np.linalg.norm(estimated - world_cm))
    return {
        "passed": error <= tolerance_cm,
        "method": method,
        "tolerance_cm": tolerance_cm,
        "max_held_out_error_cm": error,
        "check_id": "E",
        "estimated_cm": estimated.tolist(),
    }


def calibrate_markers(image, layout, *, max_error_cm):
    size = np.asarray(layout["paper_size_cm"], dtype=float)
    if size.shape != (2,) or not np.isfinite(size).all() or (size <= 0).any():
        raise ValueError("전지 가로·세로 실측 cm가 필요합니다.")
    if not np.isfinite(max_error_cm) or max_error_cm <= 0:
        raise ValueError("허용 오차는 양수(cm)로 지정하세요.")
    controls = layout["markers_cm"]
    keys = set(controls)
    if keys not in (set("ABCD"), set("ABCDE")):
        raise ValueError("실측 기준점 A~D와 선택 확인점 E를 지정하세요.")
    world = quadrilateral([controls[key] for key in "ABCD"], "실측 십자")
    if not np.allclose(world, clockwise_corners(world, "실측 십자")):
        raise ValueError("A~D는 왼쪽 위부터 시계 방향 순서로 지정하세요.")
    measured = np.asarray([controls[key] for key in sorted(keys)], dtype=float)
    if measured.shape != (len(keys), 2) or not np.isfinite(measured).all() or (
        (measured < 0).any() or (measured > size).any()
    ):
        raise ValueError("실측 기준점은 전지 안의 유효한 [x, y] cm여야 합니다.")
    if "E" in keys and cv2.pointPolygonTest(world, tuple(measured[-1]), False) <= 0:
        raise ValueError("확인점 E는 A~D 안쪽에 배치하세요.")

    height, width = image.shape[:2]
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_red, upper_red = [cv2.inRange(hsv, low, high) for low, high in MARKER_RED_RANGES]
    mask = cv2.morphologyEx(lower_red | upper_red, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for contour in contours:
        if not 50 <= cv2.contourArea(contour) <= width * height * 0.02:
            continue
        point = cross_center(contour)
        if point is not None:
            candidates.append(point)
    if len(candidates) != len(keys):
        raise ValueError(f"독립된 빨간 십자 {len(keys)}개가 필요합니다. 검출: {len(candidates)}개.")
    pixels = np.float32(candidates)
    hull = cv2.convexHull(pixels, returnPoints=False).ravel()
    if len(hull) != 4:
        raise ValueError("사진에서 A~D가 외곽을 이루고 E가 안쪽에 있는지 확인하세요.")
    outer = clockwise_corners(pixels[hull], "사진 속 십자")
    check_pixels = list(outer)
    validation = {"passed": False, "method": "no_independent_check", "tolerance_cm": max_error_cm}
    paper = np.float32([[0, 0], [size[0], 0], size, [0, size[1]]])
    # 전지 전체에서 변환식이 유효한지 확인하도록 cm → 픽셀을 먼저 구한다.
    inverse = fit_floor_transform(world, outer, paper)
    matrix = np.linalg.inv(inverse)
    corners = project_points(paper, inverse)
    tolerance = np.asarray([width, height]) * PROJECTED_PAPER_BORDER_TOLERANCE_RATIO
    if (not np.isfinite(corners).all() or (corners < -tolerance).any()
            or (corners > np.asarray([width - 1, height - 1]) + tolerance).any()):
        raise ValueError("계산한 전지 외곽이 화면 허용 여백(가로·세로 2%)을 벗어납니다. "
                         f"계산 모서리={corners.round(1).tolist()}, 영상={width}x{height}. "
                         "현재 촬영과 A~D 실측 좌표·전지 크기를 함께 확인하세요.")
    bounds = {"projected_corners_px": corners.tolist(), "tolerance_px": tolerance.tolist(),
              "slightly_outside": bool((corners < 0).any() or (corners >= [width, height]).any())}
    if "E" in keys:
        index, = sorted(set(range(len(pixels))) - set(hull))
        point = pixels[index]
        check_pixels.append(point)
        validation = evaluate_check(point, measured[-1], matrix, tolerance_cm=max_error_cm,
                                    method="independent_marker_E")

    spec = {
        "map_id": layout["map_id"],
        "paper_size_cm": size.tolist(),
        "paper_corners_px": corners.tolist(),
        "paper_corner_source": "projected_from_red_crosses",
        "control_points": [
            {"id": key, "pixel": point.tolist(), "world_cm": controls[key]}
            for key, point in zip(sorted(keys), check_pixels, strict=True)
        ],
    }
    report = {
        "map_id": spec["map_id"],
        "paper_size_cm": size.tolist(),
        "source_frame": {"coordinate_frame": "camera_image", "size_px": [width, height]},
        "target_frame": {
            "coordinate_frame": "world", "unit": "cm", "origin": WORLD_ORIGIN, "axis_directions": AXES,
        },
        "lens_distortion_corrected": False,
        "paper_bounds": bounds,
        "validation": validation,
        ("image_to_world" if validation["passed"] else "candidate_image_to_world"): matrix.tolist(),
    }
    return spec, report
