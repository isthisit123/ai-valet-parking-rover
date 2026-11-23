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
from src.rover_control.aruco_tracker import ArucoTracker
from src.rover_control.pose_sender import CapturePose, PoseUnavailable, load_camera_position, main, save_camera_position, stream_pose
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
        converter = CapturePose(self.folder)
        pose = converter.convert(self.observation())
        for field in ('map_validation_ms', 'cm_math_ms', 'total_ms'):
            self.assertGreaterEqual(converter.last_timing_ms[field], 0)
        with self.assertRaises(PoseUnavailable):
            converter.convert({**self.observation(), 'tracking_status': 'lost'})
        self.assertNotIn('cm_math_ms', converter.last_timing_ms)
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

    def test_recovered_aruco_corners_keep_height_corrected_cm_and_strict_pose_expiry(self):
        expected_corners = np.float32(self.observation()['rover']['corners_px'])
        square = np.float32([[0, 0], [99, 0], [99, 99], [0, 99]])
        marker = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 7, 100)
        warp = cv2.getPerspectiveTransform(square, expected_corners)
        image = cv2.warpPerspective(marker, warp, (800, 600), flags=cv2.INTER_NEAREST, borderValue=255)
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        tracker = ArucoTracker()
        baseline = tracker.observe(image, camera_source=self.camera, frame_id='first')
        self.assertEqual(baseline['tracking_status'], 'detected')
        camera = [53, 151.7, 111.5]
        expected = CapturePose(self.folder, camera_position_cm=camera).convert(baseline)
        native = tracker.detectors[0]

        class NativeMiss:
            def detectMarkers(self, view):
                if view.shape[:2] == (600, 800):
                    return [], None, []
                return native.detectMarkers(view)

        tracker.detectors = [NativeMiss(), NativeMiss()]
        recovered = tracker.observe(image, camera_source=self.camera, frame_id='recovered')
        self.assertEqual(recovered['detection_method'], 'roi_2x')
        converter = CapturePose(self.folder, camera_position_cm=camera)
        pose = converter.convert(recovered)
        np.testing.assert_allclose([pose['x_cm'], pose['y_cm']], [expected['x_cm'], expected['y_cm']], atol=0.15)
        self.assertLess(abs((pose['heading_deg'] - expected['heading_deg'] + 180) % 360 - 180), 2.5)
        self.assertTrue(pose['height_correction_applied'])
        self.assertEqual(converter.offset, (0, 0))
        expired = {**recovered, 'received_monotonic_s': time.monotonic() - 0.21,
                   'arrival_test': {'tracking_paused': True, 'last_known_position_px': [1, 2]}}
        with self.assertRaisesRegex(ValueError, '0.20초'):
            CapturePose(self.folder, camera_position_cm=camera).convert(expired)

    def test_height_correction_recovers_projected_3d_marker_and_axle_position(self):
        # 별도의 3D 핀홀 카메라로 영상을 합성해 높이·오프셋의 적용 순서를 검증한다.
        intrinsic = np.array([[450., 0, 400], [0, 450., 300], [0, 0, 1]])
        ground = np.float32([[0, 0, 0], [100, 0, 0], [100, 80, 0], [0, 80, 0]])
        for camera in ([50., -70., 110.], [130., 40., 130.]):
            camera = np.asarray(camera)
            z = np.array([50., 40., 0.]) - camera
            z /= np.linalg.norm(z)
            x = np.cross(z, [0, 0, 1.])
            x /= np.linalg.norm(x)
            rotation = np.array([x, np.cross(z, x), z])
            rvec, _ = cv2.Rodrigues(rotation)
            tvec = -rotation @ camera

            def pixels(points):
                return cv2.projectPoints(np.asarray(points, dtype=float), rvec, tvec, intrinsic, None)[0].reshape(-1, 2)

            self.matrix = cv2.getPerspectiveTransform(np.float32(pixels(ground) / 2), ground[:, :2].copy())
            self.calibration['image_to_world'] = self.matrix.tolist()
            self.calibration_path.write_text(json.dumps(self.calibration), encoding='utf-8')
            for heading, axle in ((0, [33.7, 39.5]), (90, [70., 55.]), (215, [25., 25.])):
                with self.subTest(camera=camera, heading=heading):
                    angle = np.deg2rad(heading)
                    forward = np.array([np.cos(angle), np.sin(angle), 0])
                    right = np.array([-forward[1], forward[0], 0])
                    center = np.array([*axle, 10.5])
                    corners = [center - 2 * forward - 2 * right, center + 2 * forward - 2 * right,
                               center + 2 * forward + 2 * right, center - 2 * forward + 2 * right]
                    observation = self.observation()
                    observation['rover']['corners_px'] = pixels(corners).tolist()
                    corrected = CapturePose(self.folder, camera_position_cm=camera.tolist()).convert(observation)
                    np.testing.assert_allclose([corrected['x_cm'], corrected['y_cm']], axle, atol=0.001)
                    self.assertLess(abs((corrected['heading_deg'] - heading + 180) % 360 - 180), 0.001)
                    self.assertTrue(corrected['height_correction_applied'])
                    self.assertEqual(corrected['marker_height_cm'], 10.5)
                    self.assertEqual(corrected['position_reference'], 'height_corrected_marker_with_offset')
                    raw = CapturePose(self.folder).convert(observation)
                    self.assertGreater(np.linalg.norm(np.array([raw['x_cm'], raw['y_cm']]) - axle), 1)

    def test_invalid_height_and_camera_measurements_are_rejected(self):
        for camera, height in (([0, 0], 10.5), ([0, 0, 10.5], 10.5), ([0, 0, -1], 0),
                               ([0, float('nan'), 100], 10.5), ([0, 0, 100], -1),
                               ([0, 0, 100], float('inf'))):
            with self.subTest(camera=camera, height=height), self.assertRaises(ValueError):
                CapturePose(self.folder, camera_position_cm=camera, marker_height_cm=height)

    def test_cli_requires_camera_or_explicit_preview_diagnostic(self):
        with patch('sys.argv', ['pose_sender', '--capture-dir', str(self.folder), '--preview-only']), \
                patch('sys.stderr', io.StringIO()) as error:
            self.assertEqual(main(), 1)
        self.assertIn('저장된 카메라 실측값이 없습니다', error.getvalue())
        with patch('sys.argv', ['pose_sender', '--floor-projection-only']), patch('sys.stderr', io.StringIO()), \
                patch('src.rover_control.pose_sender.CapturePose') as constructor:
            self.assertEqual(main(), 1)
            constructor.assert_not_called()
        arguments = ['pose_sender', '--capture-dir', str(self.folder), '--preview-only',
                     '--camera-position-cm', '50', '-70', '110']
        with patch('sys.argv', arguments), patch('sys.stdout', io.StringIO()) as output, \
                patch('src.rover_control.pose_sender.stream_pose') as stream:
            self.assertEqual(main(), 0)
            converter = stream.call_args.args[0]
            np.testing.assert_array_equal(converter.camera_position, [50, -70, 110])
            self.assertEqual(converter.marker_height, 10.5)
            self.assertEqual(converter.offset, (0, 0))
            self.assertIn('중심 오프셋: 전방 0 cm / 오른쪽 0 cm', output.getvalue())
            self.assertIn('높이 보정 적용', output.getvalue())

    def test_saved_camera_measurements_are_reused_only_for_the_same_capture(self):
        converter = CapturePose(self.folder, camera_position_cm=[53, 151.7, 111.5])
        save_camera_position(converter)
        self.assertEqual(load_camera_position(self.folder)['camera_position_cm'], [53, 151.7, 111.5])
        path = self.folder / 'world/camera_position.json'
        original_settings = path.read_bytes()
        malformed = json.loads(original_settings)
        malformed['camera_position_cm'] = None
        path.write_text(json.dumps(malformed), encoding='utf-8')
        with self.assertRaisesRegex(ValueError, '실측'):
            load_camera_position(self.folder)
        path.write_bytes(original_settings)
        with patch('sys.argv', ['pose_sender', '--capture-dir', str(self.folder), '--preview-only']), \
                patch('sys.stdout', io.StringIO()), patch('src.rover_control.pose_sender.stream_pose') as stream:
            self.assertEqual(main(), 0)
            np.testing.assert_array_equal(stream.call_args.args[0].camera_position, [53, 151.7, 111.5])
            self.assertEqual(stream.call_args.args[0].marker_height, 10.5)
            self.assertEqual(stream.call_args.args[0].offset, (0, 0))
        original_report = (self.folder / 'report.json').read_bytes()
        changed = {**self.report, 'camera_source': {**self.camera, 'session_id': 'new-camera'}}
        (self.folder / 'report.json').write_text(json.dumps(changed), encoding='utf-8')
        with self.assertRaisesRegex(ValueError, '현재 지도·보정'):
            load_camera_position(self.folder)
        (self.folder / 'report.json').write_bytes(original_report)
        changed_calibration = deepcopy(self.calibration)
        changed_calibration['image_to_world'][0][2] += 1
        self.calibration_path.write_text(json.dumps(changed_calibration), encoding='utf-8')
        with self.assertRaisesRegex(ValueError, '현재 지도·보정'):
            load_camera_position(self.folder)

    def test_loss_staleness_session_mismatch_and_duplicate_frames(self):
        converter = CapturePose(self.folder)
        observation = self.observation(captured=100)
        self.assertIsNotNone(converter.convert(observation, now=100))
        self.assertIsNone(converter.convert(observation, now=100))
        for changes in ({"tracking_status": "lost"}, {"received_monotonic_s": 99}, {"width": 640},
                        {"camera_source": {**self.camera, "session_id": "other"}}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                converter.convert({**observation, **changes}, now=100)

    def test_only_unavailable_observations_have_retryable_exception_type(self):
        converter = CapturePose(self.folder)
        observation = self.observation(captured=100)
        for changes in ({'tracking_status': 'lost'}, {'tracking_status': 'stale'},
                        {'tracking_status': 'not_started'}, {'received_monotonic_s': 99}):
            with self.subTest(changes=changes), self.assertRaises(PoseUnavailable):
                converter.convert({**observation, **changes}, now=100)
        for changes in ({'tracking_status': 'ambiguous'},
                        {'tracking_status': 'stale', 'detection_status': 'ambiguous'},
                        {'tracking_status': 'lost', 'camera_source': {**self.camera, 'session_id': 'other'}},
                        {'tracking_status': 'lost', 'width': 640}, {'received_monotonic_s': 101}):
            with self.subTest(changes=changes), self.assertRaises(ValueError) as raised:
                converter.convert({**observation, **changes}, now=100)
            self.assertNotIsInstance(raised.exception, PoseUnavailable)

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

    def test_missing_calibration_explains_the_recorded_world_failure(self):
        self.calibration_path.unlink()
        report = {**self.report, 'world_calibration': {
            'status': 'failed', 'error': '계산한 전지 외곽이 화면을 벗어납니다.'}}
        (self.folder / 'report.json').write_text(json.dumps(report), encoding='utf-8')
        with self.assertRaisesRegex(ValueError, '전지 외곽이 화면을 벗어납니다'):
            CapturePose(self.folder)
        report.pop('world_calibration')
        (self.folder / 'report.json').write_text(json.dumps(report), encoding='utf-8')
        with self.assertRaisesRegex(ValueError, '--marker-layout'):
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
