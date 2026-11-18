import json
from pathlib import Path
from tempfile import TemporaryDirectory
import time
import unittest
from unittest.mock import patch
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
    def test_diagnostics_count_only_executed_passes_and_report_budget_exhaustion(self):
        tracker = ArucoTracker()
        with patch('src.rover_control.aruco_tracker.FALLBACK_BUDGET_SECONDS', -1):
            state = tracker.observe(np.full((400, 400, 3), 255, np.uint8))
        self.assertTrue(state['fallback_budget_exhausted'])
        self.assertEqual(state['detection_attempts'], ['native'])
        self.assertEqual(len(state['detection_attempt_details']), 2)
        for detail in state['detection_attempt_details']:
            self.assertEqual(detail['decoded_ids'], [])
            self.assertGreaterEqual(detail['duration_ms'], 0)
        self.assertIsNone(state['rover'])

    def test_camera_session_or_resolution_change_resets_detection_history(self):
        tracker = ArucoTracker()
        tracker.observe(marker_image(), camera_source={'session_id': 'A'})
        state = tracker.observe(marker_image(6), camera_source={'session_id': 'A'})
        self.assertEqual(state['recent_detection']['frames'], 2)
        state = tracker.observe(marker_image(6), camera_source={'session_id': 'B'})
        self.assertEqual(state['recent_detection'], {'frames': 1, 'detected': 0, 'ratio': 0})
        state = tracker.observe(cv2.resize(marker_image(), (800, 800)), camera_source={'session_id': 'B'})
        self.assertEqual(state['recent_detection'], {'frames': 1, 'detected': 1, 'ratio': 1})

    def test_roi_fallback_decodes_current_frame_and_maps_corners_to_native_pixels(self):
        image = np.full((1000, 1000, 3), 255, np.uint8)
        marker = marker_image()[100:200, 100:200]
        image[700:740, 600:640] = cv2.resize(marker, (40, 40), interpolation=cv2.INTER_NEAREST)
        source = {'session_id': 'A'}
        tracker = ArucoTracker()
        baseline = tracker.observe(image, camera_source=source)
        self.assertEqual(baseline['tracking_status'], 'detected')
        native = tracker.detectors[0]

        class NativeMiss:
            def detectMarkers(self, view):
                if view.shape[:2] == (1000, 1000):
                    return [], None, []
                return native.detectMarkers(view)

        tracker.detectors = [NativeMiss(), NativeMiss()]
        recovered = tracker.observe(image, camera_source=source)
        self.assertEqual(recovered['detection_method'], 'roi_2x')
        np.testing.assert_allclose(recovered['rover']['center_px'], [619.5, 719.5], atol=0.8)
        wrong = image.copy()
        marker6 = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 6, 40)
        wrong[700:740, 600:640] = cv2.cvtColor(marker6, cv2.COLOR_GRAY2BGR)
        self.assertIsNone(tracker.observe(wrong, camera_source=source)['rover'])
        changed = tracker.observe(image, camera_source={'session_id': 'B'})
        self.assertIsNone(changed['rover'])
        self.assertNotIn('roi_2x', changed['detection_attempts'])
        self.assertNotIn('roi_contrast_2x', changed['detection_attempts'])

    def test_roi_contrast_remains_available_when_raw_enlargement_cannot_decode(self):
        tracker = ArucoTracker()
        tracker.observe(marker_image(), camera_source={'session_id': 'A'})
        native = tracker.detectors[0]
        calls = []

        class ContrastOnly:
            def detectMarkers(self, view):
                calls.append(view.shape)
                if len(calls) <= 4:  # native 2회와 원본 확대 ROI 2회 실패
                    return [], None, []
                return native.detectMarkers(view)

        tracker.detectors = [ContrastOnly(), ContrastOnly()]
        result = tracker.observe(marker_image(), camera_source={'session_id': 'A'})
        self.assertEqual(result['detection_method'], 'roi_contrast_2x')
        self.assertEqual(result['rover']['id'], 7)

    def test_contrast_fallback_still_requires_a_real_decoded_id(self):
        tracker = ArucoTracker()
        native = tracker.detectors[0]
        calls = []

        class FirstPassMiss:
            def detectMarkers(self, view):
                calls.append(view.shape)
                if len(calls) <= 2:
                    return [], None, []
                return native.detectMarkers(view)

        tracker.detectors = [FirstPassMiss(), FirstPassMiss()]
        recovered = tracker.observe(marker_image())
        self.assertEqual(recovered['tracking_status'], 'detected')
        self.assertEqual(recovered['detection_method'], 'contrast')
        self.assertEqual(recovered['rover']['id'], 7)

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
