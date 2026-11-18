from copy import deepcopy
from datetime import datetime, timedelta, timezone
from hashlib import sha256
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

from src.map_processing.path_visualizer import DEFAULT_ROUTE_CASES
from src.rover_control.arrival_test import ArrivalTest


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

    def test_scaled_camera_position_only_succeeds_after_stable_hold(self):
        self.assertEqual(self.test.observe(self.observation(10, position=(400, 400)), now=10)["status"], "running")
        for index in range(12):
            t = 10.1 + index * 0.1
            result = self.test.observe(self.observation(t), now=t)
            if index < 10:
                self.assertNotEqual(result["status"], "success")
        self.assertEqual(result["status"], "success")
        self.assertEqual(result["current_position_px"], [100, 80])
        self.assertEqual(json.loads(self.output.read_text())["status"], "success")
        self.assertEqual([p.name for p in self.output.parent.iterdir()], ["latest.json"])

    def test_get_expires_at_capture_deadline_without_restarting_ttl_at_receive(self):
        wall = datetime.now(timezone.utc)
        captured = (wall - timedelta(milliseconds=150)).isoformat()
        with patch('src.rover_control.arrival_test.datetime') as clock:
            clock.now.return_value = wall
            clock.fromisoformat.side_effect = datetime.fromisoformat
            state = self.test.observe(self.observation(10, client_captured_at=captured), now=10)
        self.assertIsNotNone(state['current_position_px'])
        self.assertIsNotNone(self.test.current(now=10.04)['current_position_px'])
        expired = self.test.current(now=10.06)
        self.assertIsNone(expired['current_position_px'])
        self.assertFalse(expired['guidance_available'])
        self.assertTrue(expired['tracking_paused'])
        self.assertEqual(expired['held_seconds'], 0)

    def test_pass_through_or_moving_inside_radius_does_not_succeed(self):
        for index in range(20):
            position = (200, 160) if index % 2 == 0 else (224, 160)
            t = 10 + index * 0.1
            self.assertNotEqual(self.test.observe(self.observation(t, position=position), now=t)["status"], "success")
        outside = self.test.observe(self.observation(12, position=(300, 200)), now=12)
        self.assertEqual(outside["status"], "running")
        self.assertEqual(outside["held_seconds"], 0)

    def test_loss_gap_staleness_and_repeated_frames_reset_or_do_not_advance_hold(self):
        for t in (10, 10.1, 10.2):
            self.test.observe(self.observation(t), now=t)
        lost = self.test.observe(self.observation(10.3, tracking_status="lost", rover=None), now=10.3)
        self.assertEqual(lost["status"], "holding")
        self.assertTrue(lost["tracking_paused"])
        self.assertIsNone(lost["current_position_px"])
        self.assertAlmostEqual(lost["held_seconds"], 0.2)
        self.assertFalse(lost["guidance_available"])
        self.test.observe(self.observation(10.4), now=10.4)
        gap = self.test.observe(self.observation(11), now=11)
        self.assertEqual(gap["held_seconds"], 0)
        duplicate = self.test.observe(self.observation(11.1, frame_id="11"), now=11.1)
        self.assertEqual(duplicate["sample_count"], 1)
        self.assertIsNone(self.test.current(now=11.4)["current_position_px"])

    def test_short_loss_pauses_hold_without_counting_unseen_time(self):
        for t in (10.0, 10.1, 10.2, 10.3, 10.4):
            self.test.observe(self.observation(t), now=t)
        missing = self.test.observe(self.observation(10.5, tracking_status="lost", rover=None), now=10.5)
        self.assertAlmostEqual(missing['held_seconds'], 0.4)
        returned = self.test.observe(self.observation(10.6), now=10.6)
        self.assertAlmostEqual(returned['held_seconds'], 0.4)
        self.assertAlmostEqual(returned['hold_pause_seconds'], 0.2)
        for index in range(7):
            t = 10.7 + index * 0.1
            returned = self.test.observe(self.observation(t), now=t)
        self.assertEqual(returned['status'], 'success')
        self.assertEqual(returned['hold_reset_count'], 0)

    def test_reset_diagnostics_survive_success_without_counting_repeated_gets(self):
        for t in (10, 10.1, 10.2):
            self.test.observe(self.observation(t), now=t)
        state = self.test.current(now=10.65)
        self.assertEqual(state['hold_reset_counts'], {'tracking_gap_too_long': 1})
        self.assertAlmostEqual(state['last_hold_reset']['held_seconds'], .2)
        self.assertEqual(self.test.current(now=10.8)['hold_reset_count'], 1)
        self.test.observe(self.observation(11), now=11)
        moved = self.test.observe(self.observation(11.1, position=(222, 160)), now=11.1)
        self.assertEqual(moved['hold_reset_counts'], {'tracking_gap_too_long': 1, 'position_unstable': 1})
        for index in range(12):
            t = 11.2 + index * .1
            state = self.test.observe(self.observation(t, position=(222, 160)), now=t)
        self.assertEqual(state['status'], 'success')
        self.assertEqual(state['hold_reset_count'], 2)
        self.test.start(DEFAULT_ROUTE_CASES[0][0], capture_id='version-1')
        self.assertEqual(self.test.state['hold_reset_count'], 0)

    def test_cumulative_pause_and_ambiguous_id_have_distinct_reset_reasons(self):
        for t in (10, 10.1, 10.2):
            self.test.observe(self.observation(t), now=t)
        for t in (10.3, 10.5, 10.7):
            self.test.observe(self.observation(t, tracking_status='lost', rover=None), now=t)
            state = self.test.observe(self.observation(t + .1), now=t + .1)
        self.assertEqual(state['hold_reset_counts'], {'hold_pause_budget_exceeded': 1})
        state = self.test.observe(self.observation(10.9, tracking_status='ambiguous', rover=None), now=10.9)
        self.assertEqual(state['hold_reset_counts']['marker_identity_uncertain'], 1)

    def test_pause_budget_and_different_reacquired_position_reset_hold(self):
        for t in (10.0, 10.1, 10.2):
            self.test.observe(self.observation(t), now=t)
        for t in (10.3, 10.5, 10.7):
            self.test.observe(self.observation(t, tracking_status='lost', rover=None), now=t)
            result = self.test.observe(self.observation(t + 0.1), now=t + 0.1)
        self.assertEqual(result['held_seconds'], 0)
        self.test.observe(self.observation(10.9), now=10.9)
        self.test.observe(self.observation(11.0, tracking_status='lost', rover=None), now=11.0)
        moved = self.test.observe(self.observation(11.1, position=(222, 160)), now=11.1)
        self.assertEqual(moved['held_seconds'], 0)

    def test_recent_position_is_display_only_and_long_loss_is_not_current(self):
        self.test.observe(self.observation(10, position=(400, 400)), now=10)
        brief = self.test.current(now=10.25)
        self.assertEqual(brief['status'], 'running')
        self.assertEqual(brief['last_known_position_px'], [200, 200])
        self.assertIsNone(brief['current_position_px'])
        self.assertFalse(brief['guidance_available'])
        expired = self.test.current(now=10.41)
        self.assertEqual(expired['status'], 'waiting_for_tracking')
        self.assertIsNone(expired['last_known_position_px'])

    def test_ambiguous_marker_resets_instead_of_using_short_gap_grace(self):
        for t in (10, 10.1, 10.2):
            self.test.observe(self.observation(t), now=t)
        state = self.test.observe(self.observation(10.3, tracking_status='ambiguous', rover=None), now=10.3)
        self.assertEqual(state['held_seconds'], 0)
        self.assertFalse(state['tracking_paused'])
        self.assertIsNone(state['last_known_position_px'])

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


if __name__ == "__main__":
    unittest.main()
