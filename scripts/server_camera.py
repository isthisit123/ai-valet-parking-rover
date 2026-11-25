"""서버 소유 웹캠. 영상 표시 요청과 독립적으로 최신 프레임만 분석한다."""
from dataclasses import dataclass
from datetime import datetime, timezone
import logging
import sys
import threading
import time
import uuid

import cv2
import numpy as np

from src.map_processing.map_identity import map_identity, require_map_identity
from src.parking_detection.onnx_occupancy import CarDetector


@dataclass(frozen=True)
class Frame:
    number: int
    stamp: float
    captured_at: str
    source: dict
    image: np.ndarray


def encode_image(image, extension):
    ok, buffer = cv2.imencode(extension, image)
    if not ok:
        raise ValueError("카메라 영상 인코딩 실패")
    return buffer.tobytes()


def separate_rover(boxes, pose):
    """같은 프레임의 유일하고 크기가 맞는 박스에서만 제어 중인 로버를 제외한다."""
    rover = pose.get("rover")
    if pose["tracking_status"] != "detected" or not rover:
        return boxes, {"status": "no_matching_marker", "excluded_count": 0}
    corners = np.asarray(rover["corners_px"], np.float32)
    area = cv2.contourArea(corners)
    cx, cy = rover["center_px"]
    matches = []
    for i, box in enumerate(boxes):
        x, y, w, h = box["bbox_xywh_px"]
        if (area > 1 and w*h <= 30*area and np.all(corners >= (x, y))
                and np.all(corners <= (x+w, y+h)) and .1 <= (cx-x)/w <= .9 and .1 <= (cy-y)/h <= .9):
            matches.append(i)
    if len(matches) != 1:
        return boxes, {"status": "ambiguous_vehicle" if matches else "no_matching_vehicle", "excluded_count": 0}
    return [box for i, box in enumerate(boxes) if i != matches[0]], {
        "status": "matched", "excluded_count": 1, "marker_frame_id": pose["frame_id"],
        "rover_marker_id": rover["id"], "marker_captured_at": pose["client_captured_at"], "frame_delta_ms": 0}


def monitor_regions(data, frame, report):
    if report.get("camera_source") != frame.source or data.get("capture_id") != report["capture_id"]:
        raise ValueError("현재 카메라에서 맵을 다시 생성하세요.")
    width, height = frame.source["width"], frame.source["height"]
    polygons = {}
    for region in data["regions"]:
        require_map_identity(report, region, "주차칸")
        key = region["parking_space_id"]
        if region.get("capture_id") != report["capture_id"] or key in polygons:
            raise ValueError("주차칸의 캡처 ID 또는 중복을 확인하세요.")
        cw, ch = data["canvas_size"]
        if (not all(type(v) is int and v > 0 for v in (cw, ch)) or region["image_size_px"] != [cw, ch]
                or abs(cw*height-ch*width) > 2*max(width, height)):
            raise ValueError("주차칸 설정과 영상의 크기가 다릅니다.")
        points = np.array([[p["x"], p["y"]] for p in region["points"]], np.float32)
        if (not 3 <= len(points) <= 12 or not np.isfinite(points).all() or (points < 0).any()
                or (points >= (cw, ch)).any() or not cv2.isContourConvex(points)):
            raise ValueError("주차칸 외곽을 확인하세요.")
        polygons[key] = points * np.float32([width/cw, height/ch])
    if set(polygons) != {f"space_{i}" for i in range(1, 7)}:
        raise ValueError("같은 지도의 P1~P6 여섯 칸이 필요합니다.")
    return polygons


class ServerCamera:
    def __init__(self, index, model, tracker, arrival, parking, period):
        self.index, self.model, self.period = index, model, period
        self.tracker, self.arrival, self.parking = tracker, arrival, parking
        self.lifecycle = threading.RLock()
        self.lock = threading.RLock()
        self.frame_changed = threading.Condition(self.lock)
        self.monitor_lock = threading.RLock()
        self.stop = threading.Event()
        self.ready = threading.Event()
        self.threads = []
        self.frame = self.tracked = self.miss = self.monitor = None
        self.error = None
        self.display = {}

    def start(self):
        with self.lifecycle:
            if not self.threads or self.stop.is_set():
                self.close()
                if any(t.is_alive() for t in self.threads):
                    raise ValueError("이전 카메라 드라이버가 종료 중입니다.")
                self.ready.clear()
                self.stop.clear()
                self.error = None
                self.threads = [threading.Thread(target=self._worker, args=(work,), daemon=True)
                                for work in (self._capture, self._track, self._detect)]
                for thread in self.threads:
                    thread.start()
            if not self.ready.wait(10):
                self.stop.set()
                raise ValueError("카메라 프레임 대기 시간 초과")
            return self.latest().source

    def close(self):
        with self.lifecycle:
            self.stop.set()
            for thread in self.threads:
                thread.join(timeout=2)
            self.disable()
            with self.lock:
                self.frame = self.tracked = self.miss = None

    def latest(self):
        with self.lock:
            if self.stop.is_set() or self.frame is None or time.monotonic() - self.frame.stamp > .5:
                raise ValueError(self.error or "새 카메라 프레임이 없습니다.")
            return self.frame

    def require_session(self, session_id):
        with self.lock:
            if self.frame is None or session_id != self.frame.source["session_id"]:
                raise ValueError("카메라 세션이 변경됐습니다.")

    def _worker(self, work):
        try:
            work()
        except Exception as error:
            logging.exception("카메라 작업 중단")
            with self.lock:
                self.error = str(error)
                self.stop.set()
                self.ready.set()
            self.disable()

    def _capture(self):
        cap = cv2.VideoCapture(self.index, cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_ANY)
        try:
            if not cap.isOpened():
                raise ValueError(f"카메라 {self.index} 열기 실패. --camera-index를 확인하세요.")
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            session = uuid.uuid4().hex
            number, source = 0, None
            while not self.stop.is_set():
                ok, image = cap.read()
                if self.stop.is_set():
                    return
                stamp = time.monotonic()
                at = datetime.now(timezone.utc).isoformat()
                if not ok or image is None:
                    raise ValueError("카메라 프레임 수신 중단. 카메라를 다시 시작하세요.")
                h, w = image.shape[:2]
                if source and (w, h) != (source["width"], source["height"]):
                    raise ValueError("카메라 해상도가 변경됐습니다. 다시 시작하세요.")
                source = {"kind": "webcam", "session_id": session, "width": w, "height": h}
                number += 1
                with self.lock:
                    self.frame = Frame(number, stamp, at, source, image)
                    self.frame_changed.notify_all()
                    self.ready.set()
        finally:
            cap.release()

    def _track(self):
        previous = 0
        while not self.stop.is_set():
            with self.frame_changed:
                self.frame_changed.wait_for(
                    lambda: self.stop.is_set() or (self.frame is not None and self.frame.number != previous),
                    timeout=.1)
                if self.stop.is_set():
                    return
                frame = self.frame
            if frame is None or frame.number == previous:
                continue
            start = time.monotonic()
            previous = frame.number
            pose = self.tracker.observe(frame.image, received_monotonic_s=frame.stamp,
                                        client_captured_at=frame.captured_at, frame_id=str(frame.number),
                                        camera_source=frame.source)
            self.arrival.observe(pose)
            with self.lock:
                self.tracked = (frame, pose)
                if pose["detection_status"] != "detected":
                    self.miss = self.tracked
            self.stop.wait(max(.001, self.period - (time.monotonic()-start)))

    def diagnostic(self, miss):
        with self.lock:
            pair = self.miss if miss else self.tracked
        if pair:
            frame, pose = pair
            return encode_image(frame.image, ".png"), "image/png", pose

    def disable(self):
        with self.monitor_lock:
            self.monitor = None
            self.display = {}
            self.parking.clear_observations()

    def configure(self, data, report):
        with self.monitor_lock:
            self.disable()
            frame = self.latest()
            polygons = monitor_regions(data, frame, report)
            self.monitor = ({"schema_version": "0.1-draft", **map_identity(report),
                             "capture_id": report["capture_id"], "camera_source": frame.source},
                            polygons, time.monotonic())

    def results(self):
        with self.monitor_lock:
            if self.display and time.monotonic() - self.display["stamp"] < self.parking.TTL_SECONDS:
                return self.display
            return {"boxes": [], "states": {}, "error": self.error or "새 점유 관측 대기"}

    def _detect(self):
        detector, previous = None, None
        while not self.stop.wait(.15):
            with self.lock:
                pair = self.tracked
            with self.monitor_lock:
                monitor = self.monitor
            if not monitor or not pair or pair is previous:
                continue
            previous = pair
            frame, pose = pair
            identity, polygons, enabled_at = monitor
            if (frame.source != identity["camera_source"] or frame.stamp < enabled_at
                    or time.monotonic()-frame.stamp >= self.parking.TTL_SECONDS):
                continue
            error = None
            try:
                if detector is None:
                    detector = CarDetector(self.model)
                boxes = [b for b in detector.detect(frame.image) if b["confidence"] >= .70]
                boxes, evidence = separate_rover(boxes, pose)
            except (OSError, ValueError, cv2.error) as exc:
                boxes, evidence, error = [], {"status": "inference_unavailable", "excluded_count": 0}, str(exc)
            centers = [(x+w/2, y+h/2) for x, y, w, h in (b["bbox_xywh_px"] for b in boxes)]
            spaces = []
            for key, polygon in polygons.items():
                occupied = any(cv2.pointPolygonTest(polygon, center, False) >= 0 for center in centers)
                spaces.append({"parking_space_id": key, "confidence": .5,
                               "status": "unknown" if error else "occupied" if occupied else "free"})
            with self.monitor_lock:
                if self.stop.is_set() or self.monitor is not monitor:
                    continue
                try:
                    result = self.parking.update({**identity, "observed_at": frame.captured_at, "spaces": spaces,
                        "occupancy_scope": "other_vehicles_excluding_controlled_rover", "rover_exclusion": evidence})
                    self.display = {"boxes": boxes, "states": result["states"], "capture_id": identity["capture_id"],
                                    "stamp": frame.stamp, "error": error}
                except (OSError, ValueError) as exc:
                    self.parking.clear_observations()
                    self.display = {"boxes": [], "states": {}, "stamp": frame.stamp, "error": str(exc)}
