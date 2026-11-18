"""주차칸 외곽에서 중앙 도착 원을 계산한다. 좌표와 면적은 지도 픽셀 기준이다."""

from math import isfinite, pi, sqrt

import cv2
import numpy as np


def arrival_circle(region, *, area_ratio=0.5, image_size=None):
    if type(area_ratio) not in (int, float) or not isfinite(area_ratio) or not 0.1 <= area_ratio <= 0.75:
        raise ValueError("도착 영역은 주차칸 면적의 10~75%여야 합니다.")
    polygon = region.get("polygon_px")
    source = "polygon_px"
    if polygon is None:
        x, y, width, height = region["bbox"]
        polygon = [[x, y], [x + width, y], [x + width, y + height], [x, y + height]]
        source = "bbox"
    contour = np.asarray(polygon, dtype=np.float32)
    if (contour.ndim != 2 or contour.shape[1] != 2 or len(contour) < 3
            or not np.isfinite(contour).all() or not cv2.isContourConvex(contour)):
        raise ValueError("도착 원을 계산할 주차칸 외곽이 올바르지 않습니다.")
    if image_size is not None and ((contour < 0).any() or (contour > image_size).any()):
        raise ValueError("주차칸 외곽이 지도 범위를 벗어났습니다.")
    moments = cv2.moments(contour)
    area = abs(moments["m00"])
    if area <= 0:
        raise ValueError("주차칸 면적이 0입니다.")
    center = (moments["m10"] / moments["m00"], moments["m01"] / moments["m00"])
    limit = cv2.pointPolygonTest(contour, center, True)
    radius = min(sqrt(area * area_ratio / pi), limit)
    if radius <= 0:
        raise ValueError("주차칸 안에 도착 원을 만들 수 없습니다.")
    return {"type": "circle", "center_px": list(center), "radius_px": radius,
            "parking_area_px2": area, "requested_area_ratio": area_ratio,
            "actual_area_ratio": pi * radius ** 2 / area, "max_radius_px": limit,
            "limited_to_parking_space": radius < sqrt(area * area_ratio / pi),
            "geometry_source": source, "coordinate_frame": "map_image", "unit": "pixel"}
