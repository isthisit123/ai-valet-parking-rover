from copy import deepcopy
from datetime import datetime, timedelta, timezone
from hashlib import sha256
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

import numpy as np

from src.map_processing.map_identity import AXES, WORLD_ORIGIN
from src.map_processing.planar_calibration import project_points
from src.map_processing.path_visualizer import DEFAULT_ROUTE_CASES
from src.rover_control.arrival_test import ArrivalTest
from src.rover_control.pose_sender import CapturePose, save_camera_position


def write_test_map(root, *, size=(400, 300), native=(800, 600), goal=(100, 80)):
    folder = Path(root) / "latest"
    (folder / "routes").mkdir(parents=True)
    body = b"fixed map fixture"
    (folder / "source.png").write_bytes(body)
    source = {"kind": "webcam", "session_id": "camera-A", "width": native[0], "height": native[1]}
    identity = {"map_id": "test_map", "id_assignment_revision": "v1", "map_image_sha256": sha256(body).hexdigest()}
    report = {**identity, "capture_id": "version-1", "source_sha256": identity["map_image_sha256"],
              "published_as_latest": True, "route_generation": {"status": "generated"},
              "camera_source": source, "image_size_px": list(size)}
    (folder / "report.json").write_text(json.dumps(report), encoding="utf-8")
    for index, (case, _, _) in enumerate(DEFAULT_ROUTE_CASES):
        data = {**identity, "case_id": case, "parking_space_id": f"space_{index+1}",
                "coordinate_frame": "map_image", "unit": "pixel", "waypoints": [[10, 10], list(goal)]}
        (folder / "routes" / f"{case}.json").write_text(json.dumps(data), encoding="utf-8")
    return report


class ArrivalTestCase(unittest.TestCase):
    def setUp(self):
        self.root = Path(self.enterContext(TemporaryDirectory()))
        self.report = write_test_map(self.root)
        self.output = self.root / "results/latest.json"
        self.test = ArrivalTest(self.root, output_path=self.output)
        self.test.start(DEFAULT_ROUTE_CASES[0][0], capture_id="version-1")

    def observation(self, t, *, position=(200, 160), **overrides):
        return {"tracking_status": "detected", "rover": {"id": 7, "center_px": list(position)},
                "camera_source": self.report["camera_source"], "width": 800, "height": 600,
                "received_monotonic_s": t, "frame_id": str(t), **overrides}

    def test_two_new_observations_two_seconds_apart_complete_arrival(self):
        self.assertFalse(self.test.catalog()["height_correction_applied"])
        first = self.test.observe(self.observation(10), now=10)
        self.assertEqual(first["arrival_method"], "two_observation_comparison")
        self.assertEqual(first["hold_seconds"], 2)
        self.assertEqual(first["comparison_anchor_px"], [100, 80])
        self.assertNotEqual(self.test.observe(self.observation(11.99), now=11.99)["status"], "success")
        result = self.test.observe(self.observation(12), now=12)
        self.assertEqual(result["status"], "success")
        self.assertEqual(result["current_position_px"], [100, 80])
        self.assertEqual(result["comparison_displacement_px"], 0)
        self.assertEqual(json.loads(self.output.read_text())["status"], "success")
        self.assertEqual([p.name for p in self.output.parent.iterdir()], ["latest.json"])

    def test_get_expires_at_capture_deadline_but_preserves_comparison_anchor(self):
        wall = datetime.now(timezone.utc)
        captured = (wall - timedelta(milliseconds=150)).isoformat()
        with patch('src.rover_control.arrival_test.datetime') as clock:
            clock.now.return_value = wall
            clock.fromisoformat.side_effect = datetime.fromisoformat
            self.test.observe(self.observation(10, client_captured_at=captured), now=10)
        self.assertIsNotNone(self.test.current(now=10.04)['current_position_px'])
        expired = self.test.current(now=10.06)
        self.assertIsNone(expired['current_position_px'])
        self.assertFalse(expired['guidance_available'])
        self.assertTrue(expired['tracking_paused'])
        self.assertEqual(expired['held_seconds'], 0)
        self.assertEqual(expired['comparison_anchor_px'], [100, 80])
        self.assertEqual(expired['hold_reset_count'], 0)

    def test_pass_through_or_moving_inside_radius_does_not_succeed(self):
        for index in range(30):
            position = (200, 160) if index % 2 == 0 else (224, 160)
            t = 10 + index * .1
            self.assertNotEqual(self.test.observe(self.observation(t, position=position), now=t)["status"], "success")
        outside = self.test.observe(self.observation(13, position=(300, 200)), now=13)
        self.assertEqual(outside["status"], "running")
        self.assertEqual(outside["held_seconds"], 0)
        self.assertIsNone(outside["comparison_anchor_px"])

    def test_multiple_missing_frames_do_not_reset_a_matching_endpoint_comparison(self):
        self.test.observe(self.observation(10), now=10)
        for t in (10.1, 10.7, 11.6):
            lost = self.test.observe(self.observation(t, tracking_status='lost', rover=None), now=t)
            self.assertIsNone(lost['current_position_px'])
            self.assertFalse(lost['guidance_available'])
            self.assertEqual(lost['held_seconds'], 0)
            self.assertEqual(lost['hold_reset_count'], 0)
        returned = self.test.observe(self.observation(12), now=12)
        self.assertEqual(returned['status'], 'success')
        self.assertEqual(returned['sample_count'], 2)
        self.assertEqual(returned['hold_reset_count'], 0)

    def test_gets_or_duplicate_frames_alone_cannot_complete_arrival(self):
        self.test.observe(self.observation(10), now=10)
        for t in (10.25, 10.7, 11.5, 12.1):
            state = self.test.current(now=t)
            self.assertNotEqual(state['status'], 'success')
            self.assertEqual(state['held_seconds'], 0)
        duplicate = self.test.observe(self.observation(12.2, frame_id='10'), now=12.2)
        self.assertEqual(duplicate['sample_count'], 1)
        self.assertNotEqual(duplicate['status'], 'success')
        self.assertEqual(self.test.observe(self.observation(12.3), now=12.3)['status'], 'success')

    def test_capture_latency_and_slow_updates_no_longer_reset_stationary_comparison(self):
        base = datetime.now(timezone.utc)
        for interval, latency in ((.1, .12), (.25, 0)):
            with self.subTest(interval=interval, latency=latency):
                self.test.start(DEFAULT_ROUTE_CASES[0][0], capture_id='version-1')
                for index in range(30):
                    t = 10 + index * interval
                    wall = base + timedelta(seconds=index * interval)
                    captured = (wall - timedelta(seconds=latency)).isoformat()
                    with patch('src.rover_control.arrival_test.datetime') as clock:
                        clock.now.return_value = wall
                        clock.fromisoformat.side_effect = datetime.fromisoformat
                        state = self.test.observe(self.observation(t, client_captured_at=captured), now=t)
                    if state['status'] == 'success':
                        break
                self.assertEqual(state['status'], 'success')
                self.assertGreaterEqual(state['held_seconds'] + 1e-6, 2)
                self.assertEqual(state['hold_reset_count'], 0)

    def test_comparison_uses_capture_times_instead_of_receive_delay(self):
        base = datetime.now(timezone.utc)
        for t, latency in ((10, 0), (12, .15), (12.2, .15)):
            wall = base + timedelta(seconds=t - 10)
            with patch('src.rover_control.arrival_test.datetime') as clock:
                clock.now.return_value = wall
                clock.fromisoformat.side_effect = datetime.fromisoformat
                state = self.test.observe(self.observation(t, client_captured_at=(wall - timedelta(seconds=latency)).isoformat()), now=t)
            if t == 12:
                self.assertNotEqual(state['status'], 'success')
                self.assertAlmostEqual(state['held_seconds'], 1.85)
        self.assertEqual(state['status'], 'success')

    def test_repeated_or_reversed_capture_time_is_not_a_second_observation(self):
        wall = datetime.now(timezone.utc)
        for received, capture_offset in ((10, 0), (10.1, 0), (10.11, -.02)):
            with patch('src.rover_control.arrival_test.datetime') as clock:
                # 시계 샘플링 오차로 단조 시각 환산값이 미세하게 증가해도 같은 촬영 시각은 중복이다.
                sampling_skew = .001 if received > 10 else 0
                clock.now.return_value = wall + timedelta(seconds=received - 10 - sampling_skew)
                clock.fromisoformat.side_effect = datetime.fromisoformat
                state = self.test.observe(self.observation(received,
                    client_captured_at=(wall + timedelta(seconds=capture_offset)).isoformat()), now=received)
        self.assertEqual(state['sample_count'], 1)
        self.assertEqual(state['held_seconds'], 0)

    def test_observed_movement_and_leaving_the_circle_restart_the_baseline(self):
        for moved_position in ((222, 160), (400, 400)):
            with self.subTest(position=moved_position):
                self.test.start(DEFAULT_ROUTE_CASES[0][0], capture_id='version-1')
                self.test.observe(self.observation(10), now=10)
                self.test.observe(self.observation(11, position=moved_position), now=11)
                returned = self.test.observe(self.observation(12), now=12)
                self.assertNotEqual(returned['status'], 'success')
                self.assertEqual(returned['held_seconds'], 0)
                self.assertEqual(self.test.observe(self.observation(14), now=14)['status'], 'success')

    def test_expired_baseline_resets_once_and_diagnostics_survive_success(self):
        self.test.observe(self.observation(10), now=10)
        state = self.test.current(now=15.01)
        self.assertEqual(state['hold_reset_counts'], {'comparison_expired': 1})
        self.assertEqual(self.test.current(now=15.5)['hold_reset_count'], 1)
        returned = self.test.observe(self.observation(16), now=16)
        self.assertNotEqual(returned['status'], 'success')
        self.assertEqual(returned['sample_count'], 1)
        state = self.test.observe(self.observation(18), now=18)
        self.assertEqual(state['status'], 'success')
        self.assertEqual(state['hold_reset_count'], 1)
        self.test.start(DEFAULT_ROUTE_CASES[0][0], capture_id='version-1')
        self.assertEqual(self.test.state['hold_reset_count'], 0)

    def test_missing_reference_point_is_gray_only_and_expires_after_five_seconds(self):
        self.test.observe(self.observation(10), now=10)
        brief = self.test.current(now=10.25)
        self.assertEqual(brief['status'], 'holding')
        self.assertEqual(brief['last_known_position_px'], [100, 80])
        self.assertIsNone(brief['current_position_px'])
        self.assertFalse(brief['guidance_available'])
        expired = self.test.current(now=15.01)
        self.assertEqual(expired['status'], 'waiting_for_tracking')
        self.assertIsNone(expired['last_known_position_px'])
        self.assertIsNone(expired['comparison_anchor_px'])

    def test_ambiguous_or_wrong_marker_resets_the_comparison(self):
        for overrides in ({'tracking_status':'ambiguous', 'rover':None},
                          {'rover':{'id':6, 'center_px':[200,160]}}):
            with self.subTest(overrides=overrides):
                self.test.start(DEFAULT_ROUTE_CASES[0][0], capture_id='version-1')
                self.test.observe(self.observation(10), now=10)
                state = self.test.observe(self.observation(10.5, **overrides), now=10.5)
                self.assertEqual(state['hold_reset_counts'], {'marker_identity_uncertain':1})
                self.assertIsNone(state['comparison_anchor_px'])
                self.assertFalse(state['tracking_paused'])
                self.assertNotEqual(self.test.observe(self.observation(12), now=12)['status'], 'success')

    def test_camera_or_map_change_invalidates_the_target(self):
        for change in ("camera", "size", "map"):
            with self.subTest(change=change):
                self.test.start(DEFAULT_ROUTE_CASES[0][0], capture_id="version-1")
                observation = self.observation(10)
                if change == "camera":
                    observation["camera_source"] = {**self.report["camera_source"], "session_id": "camera-B"}
                elif change == "size":
                    observation["width"] = 1600
                else:
                    path = self.root / "latest/report.json"
                    path.write_text(json.dumps({**self.report, "capture_id": "new-map"}), encoding="utf-8")
                self.assertEqual(self.test.observe(observation, now=10)["status"], "invalidated")
                (self.root / "latest/report.json").write_text(json.dumps(self.report), encoding="utf-8")

    def test_wrong_case_version_or_camera_metadata_is_rejected(self):
        for options in ({"case_id": "../bad", "capture_id": "version-1"},
                        {"case_id": DEFAULT_ROUTE_CASES[0][0], "capture_id": "old"},
                        {"case_id": DEFAULT_ROUTE_CASES[0][0], "capture_id": "version-1", "radius_px": float("nan")}):
            with self.assertRaises(ValueError):
                self.test.start(**options)
        report = deepcopy(self.report)
        report["camera_source"] = None
        (self.root / "latest/report.json").write_text(json.dumps(report), encoding="utf-8")
        self.assertFalse(self.test.catalog()["ready"])

    def test_delayed_client_frame_or_wrong_marker_does_not_count_as_arrival(self):
        delayed = (datetime.now(timezone.utc) - timedelta(seconds=1)).isoformat()
        for overrides in ({"client_captured_at": delayed}, {"rover": {"id": 6, "center_px": [200, 160]}}):
            result = self.test.observe(self.observation(10, **overrides), now=10)
            self.assertEqual(result["status"], "waiting_for_tracking")
            self.assertEqual(result["sample_count"], 0)


class HeightCorrectedArrivalTest(unittest.TestCase):
    def setUp(self):
        self.root = Path(self.enterContext(TemporaryDirectory()))
        # 원근 변환과 원본/지도 해상도 차이가 있는 구도에서 높이 보정을 왕복한다.
        self.matrix = np.array([[.25, .02, 0], [.01, .3, 0], [.0003, .0001, 1]])
        self.camera = [60, 100, 100]
        self.target_cm = [44, 36]  # 바닥 투영 [40,20]을 높이 20cm로 보정한 실제 위치.
        self.goal = project_points([self.target_cm], np.linalg.inv(self.matrix))[0]
        self.report = write_test_map(self.root, goal=self.goal.tolist())
        self.report["status"] = "map_generated"
        self.folder = self.root / "latest"
        (self.folder / "report.json").write_text(json.dumps(self.report), encoding="utf-8")
        (self.folder / "labels.json").write_text(json.dumps({
            "map_id": "test_map", "id_assignment_revision": "v1",
            "image_sha256": self.report["map_image_sha256"],
        }), encoding="utf-8")
        (self.folder / "world").mkdir()
        self.calibration_path = self.folder / "world/map_calibration.json"
        self.calibration_path.write_text(json.dumps({
            "map_id": "test_map", "validation": {"passed": True}, "paper_size_cm": [150, 120],
            "source_frame": {"size_px": [400, 300], "sha256": self.report["map_image_sha256"]},
            "target_frame": {"coordinate_frame": "world", "unit": "cm", "origin": WORLD_ORIGIN,
                             "axis_directions": AXES}, "image_to_world": self.matrix.tolist(),
        }), encoding="utf-8")
        save_camera_position(CapturePose(self.folder, camera_position_cm=self.camera, marker_height_cm=20))
        self.settings_path = self.folder / "world/camera_position.json"
        self.test = ArrivalTest(self.root)
        self.test.start(DEFAULT_ROUTE_CASES[0][0], capture_id="version-1")

    def observation(self, t, *, floor_cm=(40, 20)):
        corners = np.asarray(floor_cm) + np.array([[-2, -2], [2, -2], [2, 2], [-2, 2]])
        pixels = 2 * project_points(corners, np.linalg.inv(self.matrix))
        center = 2 * project_points([floor_cm], np.linalg.inv(self.matrix))[0]
        return {"tracking_status": "detected", "coordinate_frame": "camera_image", "unit": "pixel",
                "origin": "top_left", "axis_directions": AXES, "camera_source": self.report["camera_source"],
                "width": 800, "height": 600, "received_monotonic_s": t, "frame_id": str(t),
                "rover": {"id": 7, "center_px": center.tolist(), "corners_px": pixels.tolist()}}

    def test_corrected_position_drives_both_blue_point_and_success(self):
        self.assertTrue(self.test.catalog()["height_correction_applied"])
        raw = np.array(self.observation(10)["rover"]["center_px"]) / 2
        self.assertGreater(np.linalg.norm(raw - self.goal), 15)
        self.test.observe(self.observation(10), now=10)
        lost = {**self.observation(11), "tracking_status": "lost", "rover": None}
        self.assertIsNone(self.test.observe(lost, now=11)["current_position_px"])
        state = self.test.observe(self.observation(12), now=12)
        self.assertEqual(state["status"], "success")
        self.assertTrue(state["height_correction_applied"])
        self.assertEqual(state["position_reference"], "height_corrected_marker_with_offset")
        np.testing.assert_allclose(state["current_position_px"], self.goal, atol=.001)
        self.assertLess(state["distance_px"], .001)

    def test_raw_marker_at_goal_cannot_produce_false_success(self):
        for index in range(8):
            t = 10 + index * .1
            state = self.test.observe(self.observation(t, floor_cm=self.target_cm), now=t)
        self.assertEqual(state["status"], "running")
        self.assertEqual(state["held_seconds"], 0)
        self.assertGreater(state["distance_px"], state["radius_px"])

    def test_repeated_stale_or_invalid_observation_cannot_reuse_raw_position(self):
        observation = self.observation(10)
        state = self.test.observe(observation, now=10)
        self.assertEqual(self.test.observe(observation, now=10.1)["sample_count"], 1)
        stale = self.test.current(now=10.25)
        self.assertIsNone(stale["current_position_px"])
        np.testing.assert_allclose(stale["last_known_position_px"], self.goal, atol=.001)
        invalid = self.observation(10.3)
        invalid["rover"]["corners_px"] = []
        state = self.test.observe(invalid, now=10.3)
        self.assertEqual(state["reason"], "position_correction_failed")
        self.assertIsNone(state["current_position_px"])
        self.assertEqual(state["sample_count"], 0)
        self.assertFalse(state["guidance_available"])

    def test_settings_or_calibration_changes_invalidate_instead_of_mixing_coordinates(self):
        for path in (self.settings_path, self.calibration_path):
            original = path.read_bytes()
            self.test.start(DEFAULT_ROUTE_CASES[0][0], capture_id="version-1")
            self.test.observe(self.observation(10), now=10)
            path.write_bytes(original + b"\n")
            state = self.test.current(now=10.1)
            self.assertEqual(state["status"], "invalidated")
            self.assertEqual(state["reason"], "position_correction_changed")
            self.assertIsNone(state["current_position_px"])
            path.write_bytes(original)
        self.settings_path.unlink()
        self.assertFalse(self.test.catalog()["height_correction_applied"])
        self.test.start(DEFAULT_ROUTE_CASES[0][0], capture_id="version-1")
        save_camera_position(CapturePose(self.folder, camera_position_cm=self.camera))
        self.assertEqual(self.test.current()["status"], "invalidated")

    def test_invalid_saved_settings_block_start_instead_of_falling_back(self):
        settings = json.loads(self.settings_path.read_text())
        settings["calibration_sha256"] = "old-calibration"
        self.settings_path.write_text(json.dumps(settings), encoding="utf-8")
        self.assertFalse(self.test.catalog()["ready"])
        with self.assertRaisesRegex(ValueError, "현재 지도·보정"):
            self.test.start(DEFAULT_ROUTE_CASES[0][0], capture_id="version-1")


if __name__ == "__main__":
    unittest.main()
