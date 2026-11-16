from copy import deepcopy
from datetime import datetime, timedelta, timezone
from hashlib import sha256
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

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
        self.assertEqual(lost["status"], "waiting_for_tracking")
        self.assertEqual(lost["held_seconds"], 0)
        self.test.observe(self.observation(10.4), now=10.4)
        gap = self.test.observe(self.observation(11), now=11)
        self.assertEqual(gap["held_seconds"], 0)
        duplicate = self.test.observe(self.observation(11.1, frame_id="11"), now=11.1)
        self.assertEqual(duplicate["sample_count"], 1)
        self.assertIsNone(self.test.current(now=11.4)["current_position_px"])

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
