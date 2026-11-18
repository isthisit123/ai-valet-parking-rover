import copy
import unittest

import cv2
import numpy as np

from scripts.parking_regions import export_parking_regions
from src.map_processing.semantic_mapper import extract_regions


class ParkingRegionGeometryTest(unittest.TestCase):
    def setUp(self):
        self.semantic = {"map_id": "test_map", "id_assignment_revision": "ocr-p-labels-v1",
                         "map_image_sha256": "a" * 64, "regions": []}
        self.report = {"capture_id": "capture", "image_size_px": [500, 400], "source_sha256": "b" * 64,
                       "camera_source": {"session_id": "camera", "kind": "webcam", "width": 1000, "height": 800}}
        for i in range(1, 7):
            self.semantic["regions"].append({"id": f"space_{i}",
                "type": "accessible" if i in (3, 6) else "normal",
                "polygon_px": [[30, 30], [100, 40], [120, 100], [20, 100]]})

    def test_trapezoid_with_white_label_keeps_real_outer_corners(self):
        mask = np.zeros((400, 500), np.uint8)
        expected = np.array([[100, 100], [190, 110], [210, 190], [80, 180]])
        cv2.fillConvexPoly(mask, expected, 255)
        cv2.putText(mask, "P1", (110, 155), cv2.FONT_HERSHEY_SIMPLEX, 0.7, 0, 3)
        region, = extract_regions(mask, 100, mask.shape, include_polygon=True)
        points = np.asarray(region["polygon_px"])
        self.assertEqual(len(points), 4)
        for corner in expected:
            self.assertLessEqual(np.linalg.norm(points - corner, axis=1).min(), 1)
        self.assertEqual(len(set(points[:, 0])), 4)
        self.assertEqual(region["bbox"], [80, 100, 131, 91])

    def test_exports_stable_ids_camera_frame_and_unknown_occupancy(self):
        self.semantic["regions"].reverse()
        data = export_parking_regions(self.semantic, self.report)
        self.assertEqual([r["id"] for r in data["regions"]], [f"P{i}" for i in range(1, 7)])
        self.assertEqual(data["coordinate_frame"], "capture_image")
        self.assertEqual(data["camera_source"], self.report["camera_source"])
        self.assertEqual(data["regions"][0]["points"][1], {"x": 100, "y": 40})
        self.assertTrue(all(r["occupied"] is None and r["occupancy_status"] == "unknown" for r in data["regions"]))

    def test_rejects_partial_duplicate_wrong_type_and_invalid_polygon(self):
        changes = [lambda s: s["regions"].pop(),
                   lambda s: s["regions"][1].update(id="space_1"),
                   lambda s: s["regions"][0].update(type="accessible"),
                   lambda s: s["regions"][0].pop("polygon_px"),
                   lambda s: s["regions"][0].update(polygon_px=[[30, 30], [500, 40], [120, 100], [20, 100]]),
                   lambda s: s["regions"][0].update(polygon_px=[[30, 30], [120, 100], [100, 40], [20, 100]])]
        for change in changes:
            semantic = copy.deepcopy(self.semantic)
            change(semantic)
            with self.subTest(change=change), self.assertRaises(ValueError):
                export_parking_regions(semantic, self.report)
