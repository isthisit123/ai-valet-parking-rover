import unittest
from copy import deepcopy

import cv2
import numpy as np

from src.map_processing.planar_calibration import calibrate_floor, project_points


class PlanarCalibrationTest(unittest.TestCase):
    def setUp(self):
        paper_px = np.float32([[120, 90], [510, 120], [570, 410], [60, 380]])
        paper_cm = np.float32([[0, 0], [100, 0], [100, 80], [0, 80]])
        marker_cm = [[10, 12], [82, 19], [91, 66], [14, 55]]
        cm_to_pixel = cv2.getPerspectiveTransform(paper_cm, paper_px)
        marker_px = project_points(marker_cm, cm_to_pixel)
        self.spec = {
            "map_id": "test_map",
            "paper_size_cm": [100, 80],
            "paper_corners_px": paper_px.tolist(),
            "control_points": [
                {"id": label, "pixel": px.tolist(), "world_cm": cm}
                for label, px, cm in zip("ABCD", marker_px, marker_cm, strict=True)
            ],
        }

    def test_held_out_points_agree_with_floor_transform(self):
        result = calibrate_floor(self.spec, (640, 480), max_error_cm=0.1)
        self.assertTrue(result["validation"]["passed"])
        self.assertIn("image_to_world", result)
        self.assertEqual(len(result["validation"]["held_out_checks"]), 8)
        self.assertLess(result["validation"]["max_held_out_error_cm"], 0.01)

    def test_marker_typo_fails_the_held_out_check(self):
        wrong_spec = deepcopy(self.spec)
        wrong_spec["control_points"][1]["world_cm"][0] -= 8
        result = calibrate_floor(wrong_spec, (640, 480), max_error_cm=3)
        self.assertFalse(result["validation"]["passed"])
        self.assertNotIn("image_to_world", result)
        worst = max(result["validation"]["marker_checks_against_paper"], key=lambda check: check["error_cm"])
        self.assertEqual(worst["id"], "B")
