"""웹 카메라 프레임에서 로버 상단 마커의 픽셀 위치와 영상 방향각을 구한다."""

from copy import deepcopy
from datetime import datetime, timezone
import json
from math import atan2, degrees, isfinite
from pathlib import Path
from tempfile import NamedTemporaryFile
import threading
import time

import cv2
import numpy as np


MAX_AGE_SECONDS = 0.20


def marker_geometry(corners, front_offset_deg=0.0):
    points = np.asarray(corners, dtype=np.float32).reshape(4, 2)
    if not np.isfinite(points).all() or not cv2.isContourConvex(points) or cv2.contourArea(points) < 1:
        raise ValueError("마커 모서리 네 점이 올바른 사각형이 아닙니다.")
    canonical = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], np.float32)
    transform = cv2.getPerspectiveTransform(canonical, points)
    # 대각선 교차에 해당하는 실제 정사각형 중심과 중심에서 오른쪽을 향하는 축을 투영한다.
    center, right = cv2.perspectiveTransform(np.array([[[0.5, 0.5], [0.75, 0.5]]], np.float32), transform)[0]
    direction = right - center
    if not np.isfinite(center).all() or not np.isfinite(direction).all() or np.linalg.norm(direction) < 1e-6:
        raise ValueError("마커의 중심 또는 방향을 계산할 수 없습니다.")
    heading = (degrees(atan2(float(direction[1]), float(direction[0]))) + front_offset_deg) % 360
    return center.tolist(), heading


class ArucoTracker:
    def __init__(self, *, marker_id=7, dictionary="DICT_4X4_50", front_offset_deg=0.0, output_path=None):
        if not dictionary.startswith("DICT_") or not hasattr(cv2.aruco, dictionary):
            raise ValueError(f"지원하지 않는 ArUco 사전: {dictionary}")
        self.dictionary = dictionary
        codebook = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
        if type(marker_id) is not int or not 0 <= marker_id < len(codebook.bytesList):
            raise ValueError("마커 ID가 선택한 사전의 범위를 벗어났습니다.")
        if not isfinite(front_offset_deg):
            raise ValueError("전방 각도 보정값은 유한한 숫자여야 합니다.")
        self.marker_id = marker_id
        self.front_offset_deg = front_offset_deg
        self.output_path = Path(output_path) if output_path is not None else None
        self.detectors = []
        for threshold in (7, 5):
            parameters = cv2.aruco.DetectorParameters()
            parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            parameters.adaptiveThreshConstant = threshold
            self.detectors.append(cv2.aruco.ArucoDetector(codebook, parameters))
        self._latest = None
        self._lock = threading.RLock()

    def observe(self, image, *, received_monotonic_s=None, client_captured_at=None, frame_id=None, camera_source=None):
        received = time.monotonic() if received_monotonic_s is None else received_monotonic_s
        start = time.perf_counter()
        with self._lock:
            found_corners, found_ids = [], []
            for detector in self.detectors:
                corners, ids, _ = detector.detectMarkers(image)
                ids = [] if ids is None else ids.flatten().tolist()
                if self.marker_id in ids or len(ids) > len(found_ids):
                    found_corners, found_ids = corners, ids
                if self.marker_id in ids:
                    break
            markers = []
            for identity, corners in zip(found_ids, found_corners, strict=True):
                points = corners.reshape(4, 2)
                center, heading = marker_geometry(points, self.front_offset_deg)
                markers.append({"id": int(identity), "corners_px": points.tolist(),
                                "center_px": center, "heading_deg": heading})
            matches = [marker for marker in markers if marker["id"] == self.marker_id]
            status = "detected" if len(matches) == 1 else "ambiguous" if matches else "lost"
            self._latest = {
                "tracking_status": status, "dictionary": self.dictionary, "rover_marker_id": self.marker_id,
                "detection_status": status,
                "width": image.shape[1], "height": image.shape[0], "markers": markers,
                "rover": matches[0] if status == "detected" else None,
                "coordinate_frame": "camera_image", "unit": "pixel", "origin": "top_left",
                "axis_directions": {"x": "right", "y": "down"}, "angle_unit": "degree",
                "heading_zero": "right", "positive_rotation": "clockwise",
                "marker_front_axis": "+x", "front_offset_deg": self.front_offset_deg,
                "position_reference": "marker_center_on_rover_top",
                "pose_status": "calibration_required", "map_pose": None,
                "received_monotonic_s": received, "valid_for_ms": MAX_AGE_SECONDS * 1000,
                "observed_at": datetime.now(timezone.utc).isoformat(), "client_captured_at": client_captured_at,
                "frame_id": frame_id, "processing_ms": (time.perf_counter() - start) * 1000,
                "camera_source": camera_source,
            }
            result = self.current()
            if self.output_path is not None:
                self.output_path.parent.mkdir(parents=True, exist_ok=True)
                pending = None
                try:
                    with NamedTemporaryFile(mode="w", encoding="utf-8", dir=self.output_path.parent,
                                            prefix=".aruco-", delete=False) as stream:
                        pending = Path(stream.name)
                        json.dump(result, stream, ensure_ascii=False, allow_nan=False)
                        stream.write("\n")
                    pending.replace(self.output_path)
                finally:
                    if pending is not None:
                        pending.unlink(missing_ok=True)
            return result

    def current(self):
        with self._lock:
            if self._latest is None:
                return {"tracking_status": "not_started", "rover": None, "markers": [],
                        "pose_status": "calibration_required", "map_pose": None}
            result = deepcopy(self._latest)
            result["age_ms"] = max(0, time.monotonic() - result["received_monotonic_s"]) * 1000
            capture_valid = True
            if result.get("client_captured_at"):
                try:
                    captured = datetime.fromisoformat(result["client_captured_at"].replace("Z", "+00:00"))
                    if captured.utcoffset() is None:
                        raise ValueError("촬영 시각에 시간대가 필요합니다.")
                    result["capture_age_ms"] = (datetime.now(timezone.utc) - captured).total_seconds() * 1000
                    capture_valid = -50 <= result["capture_age_ms"] <= result["valid_for_ms"]
                except (ValueError, TypeError):
                    capture_valid = False
            if result["age_ms"] > result["valid_for_ms"] or not capture_valid:
                result.update(tracking_status="stale", rover=None, markers=[])
            return result
