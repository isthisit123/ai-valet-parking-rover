import unittest
from copy import deepcopy

import cv2
import numpy as np

from src.map_processing.floor_markers import calibrate_markers
from src.map_processing.map_config import DEFAULT_MAP
from src.map_processing.map_reader import load_map_image
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


class CameraMarkerCalibrationTest(unittest.TestCase):
    def setUp(self):
        self.layout = {
            "map_id": "test_map",
            "paper_size_cm": [100, 80],
            "markers_cm": {"A": [12, 12], "B": [86, 14], "C": [89, 67], "D": [14, 64], "E": [61, 29]},
        }
        image = np.full((400, 500, 3), 235, np.uint8)
        # 두 갈래가 있는 도로와 표지판도 함께 둔다.
        cv2.line(image, (55, 204), (445, 204), (0, 0, 210), 9)
        cv2.line(image, (150, 204), (150, 260), (0, 0, 210), 9)
        cv2.rectangle(image, (75, 233), (97, 254), (0, 0, 210), -1)
        for x, y in self.layout["markers_cm"].values():
            x, y = 5 * x, 5 * y
            cv2.rectangle(image, (x - 13, y - 4), (x + 13, y + 4), (0, 0, 210), -1)
            cv2.rectangle(image, (x - 4, y - 13), (x + 4, y + 13), (0, 0, 210), -1)
        self.transform = cv2.getPerspectiveTransform(
            np.float32([[0, 0], [500, 0], [500, 400], [0, 400]]),
            np.float32([[122, 60], [625, 95], [690, 509], [75, 472]]),
        )
        self.image = cv2.warpPerspective(image, self.transform, (760, 560))

    def test_calibrates_new_camera_perspective_and_checks_independent_point(self):
        points, report = calibrate_markers(self.image, self.layout, max_error_cm=0.5)
        self.assertTrue(report["validation"]["passed"])
        expected = project_points(np.array(list(self.layout["markers_cm"].values())) * 5, self.transform)
        np.testing.assert_allclose([point["pixel"] for point in points["control_points"]], expected, atol=2)
        moved = deepcopy(self.layout)
        moved["markers_cm"]["E"] = [65, 29]
        _, rejected = calibrate_markers(self.image, moved, max_error_cm=0.5)
        self.assertFalse(rejected["validation"]["passed"])
        self.assertNotIn("image_to_world", rejected)

    def test_current_photo_detects_four_crosses_without_counting_road(self):
        # 실측 대신 테스트 전용 근삿값을 주고, 검출된 픽셀과 미리보기 제한을 확인한다.
        self.layout["markers_cm"] = {
            "A": [7.68, 19.76], "B": [86.87, 31.64], "C": [103.63, 67.87], "D": [16.67, 49.75],
        }
        self.layout["paper_size_cm"] = [109.5, 79]
        image = load_map_image(DEFAULT_MAP.image_path)
        points, report = calibrate_markers(image, self.layout, max_error_cm=1)
        np.testing.assert_allclose(
            [point["pixel"] for point in points["control_points"]],
            [[345, 232], [921, 306], [1123, 505], [390, 375]], atol=4,
        )
        self.assertFalse(report["validation"]["passed"])
        self.assertIn("candidate_image_to_world", report)
        with self.assertRaisesRegex(ValueError, "독립 기준점"):
            calibrate_floor(points, image.shape[1::-1], max_error_cm=1)

    def test_rejects_wrong_marker_count(self):
        with self.assertRaisesRegex(ValueError, "검출: 0개"):
            calibrate_markers(np.zeros_like(self.image), self.layout, max_error_cm=1)
