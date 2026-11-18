import json
from pathlib import Path
from tempfile import TemporaryDirectory
import time
import unittest
from datetime import datetime, timedelta, timezone

import cv2
import numpy as np

from src.rover_control.aruco_tracker import ArucoTracker, marker_geometry


def marker_image(marker_id=7):
    image = np.full((400, 400, 3), 255, np.uint8)
    marker = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), marker_id, 100)
    image[100:200, 100:200] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    return image


class ArucoTrackerTest(unittest.TestCase):
    def test_delayed_or_future_capture_never_exposes_detected_marker_as_current_pose(self):
        image = marker_image()
        for seconds in (-3, 3):
            with self.subTest(seconds=seconds):
                captured = (datetime.now(timezone.utc) + timedelta(seconds=seconds)).isoformat()
                result = ArucoTracker().observe(image, client_captured_at=captured)
                self.assertEqual(result['detection_status'], 'detected')
                self.assertEqual(result['tracking_status'], 'stale')
                self.assertIsNone(result['rover'])

    def test_invalid_or_timezone_missing_capture_time_cannot_supply_current_pose(self):
        for captured in ('bad-time', '2026-11-18T10:00:00'):
            result = ArucoTracker().observe(marker_image(), client_captured_at=captured)
            self.assertEqual(result['tracking_status'], 'stale')
            self.assertIsNone(result['rover'])

    def test_detects_id_and_four_rotations_with_image_heading(self):
        tracker = ArucoTracker()
        for turns in range(4):
            with self.subTest(turns=turns):
                result = tracker.observe(np.rot90(marker_image(), -turns).copy())
                self.assertEqual(result["tracking_status"], "detected")
                self.assertEqual(result["rover"]["id"], 7)
                expected = turns * 90
                error = (result["rover"]["heading_deg"] - expected + 180) % 360 - 180
                self.assertAlmostEqual(error, 0, places=1)
                self.assertEqual(result["coordinate_frame"], "camera_image")
                self.assertIsNone(result["map_pose"])

    def test_projective_center_and_heading_offset(self):
        corners = np.array([[20, 10], [120, 20], [100, 100], [30, 80]], np.float32)
        center, heading = marker_geometry(corners)
        reference = cv2.perspectiveTransform(np.array([[[0.5, 0.5]]], np.float32),
            cv2.getPerspectiveTransform(np.array([[0, 0], [1, 0], [1, 1], [0, 1]], np.float32), corners))[0, 0]
        np.testing.assert_allclose(center, reference)
        self.assertFalse(np.allclose(center, corners.mean(axis=0)))
        self.assertAlmostEqual(marker_geometry(corners, 90)[1], (heading + 90) % 360)

    def test_missing_duplicate_or_stale_marker_clears_current_rover(self):
        tracker = ArucoTracker()
        self.assertEqual(tracker.current()["tracking_status"], "not_started")
        self.assertEqual(tracker.observe(marker_image())["tracking_status"], "detected")
        missing = tracker.observe(marker_image(6))
        self.assertEqual(missing["tracking_status"], "lost")
        self.assertIsNone(missing["rover"])
        duplicate = marker_image()
        duplicate[100:200, 250:350] = duplicate[100:200, 100:200]
        result = tracker.observe(duplicate)
        self.assertEqual(result["tracking_status"], "ambiguous")
        self.assertIsNone(result["rover"])
        stale = tracker.observe(marker_image(), received_monotonic_s=time.monotonic() - 1)
        self.assertEqual(stale["tracking_status"], "stale")
        self.assertIsNone(stale["rover"])
        self.assertFalse(stale["markers"])

    def test_persists_only_fixed_json_and_replaces_on_loss(self):
        with TemporaryDirectory() as directory:
            output = Path(directory) / "latest.json"
            tracker = ArucoTracker(output_path=output)
            tracker.observe(marker_image())
            self.assertEqual(json.loads(output.read_text())["rover"]["id"], 7)
            tracker.observe(np.full((400, 400, 3), 255, np.uint8))
            self.assertIsNone(json.loads(output.read_text())["rover"])
            self.assertEqual([p.name for p in Path(directory).iterdir()], ["latest.json"])


if __name__ == "__main__":
    unittest.main()
