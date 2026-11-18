"""의미 지도의 실제 외곽을 ParkingHaeven의 points 형식으로 내보낸다."""

import cv2
import numpy as np


def export_parking_regions(semantic, report):
    width, height = report["image_size_px"]
    spaces = [r for r in semantic["regions"] if r.get("type") in ("normal", "accessible")]
    expected = {f"space_{i}" for i in range(1, 7)}
    if len(spaces) != 6 or {r["id"] for r in spaces} != expected:
        raise ValueError("P1~P6의 고유 주차칸 여섯 개가 필요합니다.")
    regions = []
    for space in sorted(spaces, key=lambda r: r["id"]):
        expected_type = "accessible" if space["id"] in ("space_3", "space_6") else "normal"
        if space["type"] != expected_type:
            raise ValueError(f"{space['id']}의 일반석·장애인석 구분을 확인하세요.")
        points = np.asarray(space.get("polygon_px", []), dtype=np.float32)
        if (points.ndim != 2 or points.shape[1] != 2 or not 3 <= len(points) <= 12
                or not np.isfinite(points).all() or (points < 0).any()
                or (points >= (width, height)).any() or not cv2.isContourConvex(points)
                or cv2.contourArea(points) <= 0):
            raise ValueError(f"{space['id']}의 외곽 꼭짓점을 확인하세요.")
        regions.append({"id": space["id"].replace("space_", "P"),
                        "parking_space_id": space["id"], "type": space["type"],
                        "points": [{"x": float(x), "y": float(y)} for x, y in points],
                        "occupied": None, "occupancy_status": "unknown"})
    return {"schema_version": "1.0", "status": "generated", "capture_id": report["capture_id"],
            **{key: semantic[key] for key in ("map_id", "id_assignment_revision", "map_image_sha256")},
            "coordinate_frame": "capture_image", "unit": "pixel", "origin": "top_left",
            "axes": {"x": "right", "y": "down"}, "image_size_px": [width, height],
            "source_sha256": report["source_sha256"], "camera_source": report.get("camera_source"),
            "regions": regions}
