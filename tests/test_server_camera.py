"""브라우저의 프레임 전송 없이 실제 HTTP 관측·점유 API가 갱신되는지 검증한다."""
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock, patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import cv2
import numpy as np

from scripts.http_map_server import BUNDLED_WEB_ROOT, handler_for
from scripts.server_camera import Frame, ServerCamera
from src.map_processing.map_identity import map_identity
from test_aruco_tracker import marker_image
from test_live_parking import write_live_map


class CameraInput:
    def __init__(self):
        self.image = marker_image()
        self.released = False
        self.disconnected = False

    def isOpened(self):
        return True

    def set(self, *args):
        return True

    def read(self):
        time.sleep(.025)
        return not self.disconnected, self.image

    def release(self):
        self.released = True


class CameraTrackingTimingTest(unittest.TestCase):
    def test_new_frame_wakes_tracking_after_idle_period(self):
        observed = threading.Event()
        tracker = Mock()

        def observe(*args, **kwargs):
            observed.set()
            return {"detection_status": "detected"}

        tracker.observe.side_effect = observe
        camera = ServerCamera(0, None, tracker, Mock(), Mock(), period=.6)
        image = np.zeros((10, 10, 3), np.uint8)
        camera.frame = Frame(1, time.monotonic(), "", {}, image)
        thread = threading.Thread(target=camera._track, daemon=True)
        thread.start()
        try:
            self.assertTrue(observed.wait(1))
            observed.clear()
            # 분석 주기가 지난 뒤 도착한 프레임을 다음 고정 틱까지 기다리지 않는다.
            time.sleep(.75)
            self.assertEqual(tracker.observe.call_count, 1)
            with camera.frame_changed:
                camera.frame = Frame(2, time.monotonic(), "", {}, image)
                camera.frame_changed.notify_all()
            self.assertTrue(observed.wait(.25), "새 프레임이 분석을 즉시 깨워야 합니다.")
            self.assertEqual(tracker.observe.call_args.kwargs["frame_id"], "2")
        finally:
            camera.stop.set()
            with camera.frame_changed:
                camera.frame_changed.notify_all()
            thread.join(timeout=1)
        self.assertFalse(thread.is_alive())


class ServerCameraTest(unittest.TestCase):
    def setUp(self):
        self.root = Path(self.enterContext(tempfile.TemporaryDirectory()))
        self.maps = self.root / "maps"
        self.report = write_live_map(self.maps)
        self.device = CameraInput()
        self.open = self.enterContext(patch("scripts.server_camera.cv2.VideoCapture", return_value=self.device))
        self.detector = self.enterContext(patch("scripts.server_camera.CarDetector")).return_value
        # 첫 박스는 로버 마커, 두 번째 박스는 P1에 있는 다른 차다.
        self.detector.detect.return_value = [
            {"bbox_xywh_px": [97, 97, 106, 106], "confidence": .9},
            {"bbox_xywh_px": [73, 63, 14, 14], "confidence": .8}]
        handler = handler_for(BUNDLED_WEB_ROOT, self.maps, self.root/"labels", self.root/"mapping",
                              camera_index=3, async_diagnostics=True, aruco_period_ms=55,
                              aruco_output_path=self.root/"pose", arrival_output_path=self.root/"arrival",
                              occupancy_output_path=self.root/"occupancy")
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
        self.url = f"http://127.0.0.1:{self.server.server_port}"
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.addCleanup(handler.close_diagnostics)
        self.addCleanup(self.server.server_close)
        self.addCleanup(self.server.shutdown)

    def request(self, path, data=None, headers=None):
        request = Request(self.url + path, None if data is None else json.dumps(data).encode(),
                          {"Content-Type": "application/json", **(headers or {})})
        with urlopen(request, timeout=5) as response:
            if response.headers.get_content_type() == "application/json":
                return json.load(response)
            return response.read()

    def enable(self):
        source = self.request("/api/camera/start", {})
        self.report["camera_source"] = source
        (self.maps/"latest/report.json").write_text(json.dumps(self.report))
        regions = json.loads((self.maps/"latest/semantic.json").read_text())["regions"]
        config = []
        for region in regions[:6]:
            x, y, w, h = region["bbox"]
            config.append({"parking_space_id": region["id"], **map_identity(self.report),
                           "capture_id": self.report["capture_id"], "image_size_px": [400, 400],
                           "points": [{"x": a, "y": b} for a,b in ((x,y),(x+w,y),(x+w,y+h),(x,y+h))]})
        self.request("/api/camera/monitor", {"enabled": True, "session_id": source["session_id"],
                     "regions": config, "canvas_size": [400, 400], "capture_id": self.report["capture_id"]})
        return source

    def test_background_capture_keeps_pose_and_occupancy_current(self):
        source = self.enable()
        self.assertEqual(self.request("/api/camera/start", {}), source)
        self.assertEqual(self.open.call_count, 1)
        # 화면 조회·폴링·업로드가 전혀 없는 구간도 새 free 관측을 쌓아야 한다.
        time.sleep(2.85)
        pose = self.request("/api/rover-pose")
        self.assertEqual(pose["rover"]["id"], 7)
        live = self.request("/api/parking-occupancy")
        self.assertTrue(live["fresh"])
        self.assertEqual(live["states"]["space_1"], "occupied")
        self.assertEqual(live["states"]["space_2"], "free")
        self.assertEqual(live["rover_exclusion"]["excluded_count"], 1)
        time.sleep(.3)
        self.assertNotEqual(pose["frame_id"], self.request("/api/rover-pose")["frame_id"])
        self.assertEqual(len(self.request("/api/camera/detections")["boxes"]), 1)
        with self.assertRaises(HTTPError):
            self.request("/api/parking-occupancy", {})
        self.detector.detect.side_effect = ValueError("inference failure")
        time.sleep(.55)
        self.assertEqual(set(self.request("/api/parking-occupancy")["states"].values()), {"unknown"})
        self.request("/api/camera/stop", {"session_id": source["session_id"]})
        time.sleep(.22)
        self.assertIsNone(self.request("/api/rover-pose")["rover"])
        self.assertFalse(self.request("/api/parking-occupancy")["fresh"])
        self.assertTrue(self.device.released)

    def test_map_uses_native_image_and_own_session(self):
        source = self.request("/api/camera/start", {})
        headers = {"X-Source-Type": "webcam", "X-Source-Width": "400", "X-Source-Height": "400",
                   "X-Camera-Session": source["session_id"]}
        self.assertTrue(self.request("/api/camera/preview.jpg").startswith(b"\xff\xd8"))
        with patch("scripts.http_map_server.generate_map", return_value={"status": "map_generated"}) as generate:
            self.request("/api/camera/map-capture", {}, headers)
            args, kwargs = generate.call_args
            np.testing.assert_array_equal(cv2.imdecode(np.frombuffer(args[0], np.uint8), cv2.IMREAD_COLOR),
                                          self.device.image)
            self.assertEqual(kwargs["camera_source"], source)
            self.assertIsNotNone(kwargs["captured_at"])
            headers["X-Camera-Session"] = "old"
            with self.assertRaises(HTTPError):
                self.request("/api/camera/map-capture", {}, headers)
            self.assertEqual(generate.call_count, 1)

    def test_disconnect_invalidates_observations_and_reopen_requires_new_map(self):
        source = self.enable()
        self.device.disconnected = True
        deadline = time.monotonic() + 2
        while not self.device.released and time.monotonic() < deadline:
            time.sleep(.02)
        self.assertTrue(self.device.released)
        time.sleep(.25)
        self.assertIsNone(self.request("/api/rover-pose")["rover"])
        self.assertFalse(self.request("/api/parking-occupancy")["fresh"])
        with self.assertRaises(HTTPError):
            self.request("/api/camera/preview.jpg")
        self.device.disconnected = False
        new_source = self.request("/api/camera/start", {})
        self.assertNotEqual(source["session_id"], new_source["session_id"])
        with self.assertRaises(HTTPError):
            self.request("/api/camera/monitor", {"enabled": True, "session_id": new_source["session_id"],
                         "capture_id": self.report["capture_id"]})
        with self.assertRaises(HTTPError):
            self.request("/api/camera/stop", {"session_id": source["session_id"]})
