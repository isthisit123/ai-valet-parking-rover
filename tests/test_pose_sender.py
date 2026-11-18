from copy import deepcopy
from hashlib import sha256
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
from pathlib import Path
import socket
from tempfile import TemporaryDirectory
import threading
import time
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from src.map_processing.map_identity import AXES, WORLD_ORIGIN
from src.map_processing.planar_calibration import project_points
from src.rover_control.drive import PoseReceiver
from src.rover_control.pose_sender import CapturePose, stream_pose
from src.rover_control.tracking import Follower, Route


class PoseSenderTest(unittest.TestCase):
    def setUp(self):
        self.folder = Path(self.enterContext(TemporaryDirectory()))
        image = np.full((300, 400, 3), 255, np.uint8)
        ok, png = cv2.imencode(".png", image)
        self.assertTrue(ok)
        (self.folder / "source.png").write_bytes(png.tobytes())
        self.identity = {"map_id": "test_map", "id_assignment_revision": "v1",
                         "map_image_sha256": sha256(png.tobytes()).hexdigest()}
        (self.folder / "labels.json").write_text(json.dumps({
            "map_id": "test_map", "id_assignment_revision": "v1", "image_sha256": self.identity["map_image_sha256"],
        }), encoding="utf-8")
        self.camera = {"kind": "webcam", "session_id": "current-camera", "width": 800, "height": 600}
        self.report = {**self.identity, "published_as_latest": True, "status": "map_generated",
                       "capture_id": "capture-1", "camera_source": self.camera, "image_size_px": [400, 300]}
        (self.folder / "report.json").write_text(json.dumps(self.report), encoding="utf-8")
        world = np.float32([[0, 0], [100, 0], [100, 80], [0, 80]])
        image_corners = np.float32([[30, 20], [360, 60], [390, 270], [15, 260]])
        self.matrix = cv2.getPerspectiveTransform(image_corners, world)
        (self.folder / "world").mkdir()
        self.calibration = {
            "map_id": "test_map", "validation": {"passed": True}, "paper_size_cm": [100, 80],
            "source_frame": {"size_px": [400, 300], "sha256": self.identity["map_image_sha256"]},
            "target_frame": {"coordinate_frame": "world", "unit": "cm", "origin": WORLD_ORIGIN, "axis_directions": AXES},
            "image_to_world": self.matrix.tolist(),
        }
        self.calibration_path = self.folder / "world/map_calibration.json"
        self.calibration_path.write_text(json.dumps(self.calibration), encoding="utf-8")

    def observation(self, *, center=(60, 40), heading=30, captured=None):
        angle = np.deg2rad(heading)
        forward = np.array([np.cos(angle), np.sin(angle)])
        right = np.array([-forward[1], forward[0]])
        corners_cm = np.array([center - 2 * forward - 2 * right, center + 2 * forward - 2 * right,
                               center + 2 * forward + 2 * right, center - 2 * forward + 2 * right])
        corners_px = 2 * project_points(corners_cm, np.linalg.inv(self.matrix))
        return {"tracking_status": "detected", "coordinate_frame": "camera_image", "unit": "pixel",
                "origin": "top_left", "axis_directions": AXES, "camera_source": self.camera,
                "width": 800, "height": 600, "received_monotonic_s": time.monotonic() if captured is None else captured,
                "frame_id": "one", "front_offset_deg": 0, "rover": {"id": 7, "corners_px": corners_px.tolist()}}

    def test_scaled_perspective_position_and_world_heading(self):
        pose = CapturePose(self.folder).convert(self.observation())
        np.testing.assert_allclose([pose["x_cm"], pose["y_cm"]], [60, 40], atol=0.001)
        self.assertAlmostEqual(pose["heading_deg"], 30, places=3)
        self.assertEqual(pose["unit"], "cm")
        observation = self.observation()
        observation["front_offset_deg"] = 90
        turned = CapturePose(self.folder).convert(observation)
        self.assertAlmostEqual(turned["heading_deg"], 120, places=3)

    def test_center_offset_is_applied_in_rover_axes(self):
        converter = CapturePose(self.folder, center_offset_cm=(2, 3))
        pose = converter.convert(self.observation(heading=90))
        np.testing.assert_allclose([pose["x_cm"], pose["y_cm"]], [57, 42], atol=0.001)
        self.assertEqual(pose["position_reference"], "floor_projected_marker_with_offset")

    def test_loss_staleness_session_mismatch_and_duplicate_frames(self):
        converter = CapturePose(self.folder)
        observation = self.observation(captured=100)
        self.assertIsNotNone(converter.convert(observation, now=100))
        self.assertIsNone(converter.convert(observation, now=100))
        for changes in ({"tracking_status": "lost"}, {"received_monotonic_s": 99}, {"width": 640},
                        {"camera_source": {**self.camera, "session_id": "other"}}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                converter.convert({**observation, **changes}, now=100)

    def test_saved_transform_keeps_working_when_floor_crosses_are_occluded(self):
        converter = CapturePose(self.folder)
        matrix = converter.matrix.copy()
        for index, center in enumerate(((60, 40), (61, 41), (62, 42))):
            observation = self.observation(center=center, captured=100 + index * 0.1)
            observation["frame_id"] = str(index)
            # 주행 프레임에는 십자 검출값을 보내지 않고 로버 마커만 제공한다.
            self.assertNotIn("floor_calibration", observation)
            pose = converter.convert(observation, now=100 + index * 0.1)
            np.testing.assert_allclose([pose["x_cm"], pose["y_cm"]], center, atol=0.001)
        np.testing.assert_array_equal(converter.matrix, matrix)

    def test_unverified_calibration_and_map_changes_are_rejected(self):
        converter = CapturePose(self.folder)
        changed = deepcopy(self.report)
        changed["capture_id"] = "capture-2"
        (self.folder / "report.json").write_text(json.dumps(changed), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "갱신"):
            converter.convert(self.observation())
        self.calibration["validation"]["passed"] = False
        self.calibration_path.write_text(json.dumps(self.calibration), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "검증"):
            CapturePose(self.folder)

    def test_http_observation_reaches_drive_receiver_over_udp_and_stops_on_exit(self):
        observation = self.observation()

        class Handler(BaseHTTPRequestHandler):
            def do_GET(handler):
                self.assertEqual(handler.path, "/api/rover-pose")
                body = json.dumps({**observation, "received_monotonic_s": time.monotonic()}).encode()
                handler.send_response(200)
                handler.end_headers()
                handler.wfile.write(body)

            def log_message(self, *args):
                pass

        server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
                udp.bind(("127.0.0.1", 0))
                udp.setblocking(False)
                receiver = PoseReceiver(udp, "test_map", self.identity)
                received = []

                def after_cycle(_):
                    received.append(receiver.read())
                    raise KeyboardInterrupt

                with patch("src.rover_control.pose_sender.time.sleep", side_effect=after_cycle), \
                        patch("sys.stdout", io.StringIO()), self.assertRaises(KeyboardInterrupt):
                    stream_pose(CapturePose(self.folder), f"http://127.0.0.1:{server.server_port}",
                                pose_port=udp.getsockname()[1])
                pose, = received
                np.testing.assert_allclose(pose.point, [60, 40], atol=0.001)
                follower = Follower(Route("test_map", [(60, 40), (70, 40)]), 4)
                self.assertGreater(follower.drive(pose)["linear_cm_s"], 0)
                with self.assertRaisesRegex(ValueError, "추적이 끊겼"):
                    receiver.read()
                bad_pose = {**CapturePose(self.folder).convert(self.observation()), "map_image_sha256": "other"}
                udp.sendto(json.dumps(bad_pose).encode(), udp.getsockname())
                with self.assertRaisesRegex(ValueError, "map_image_sha256"):
                    receiver.read()
        finally:
            server.shutdown()
            thread.join()
            server.server_close()
