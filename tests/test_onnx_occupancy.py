import unittest

import numpy as np

from src.map_processing.map_identity import map_identity
from src.map_processing.occupancy import occupancy_states
from src.parking_detection.onnx_occupancy import CarDetector, build_snapshot


class OnnxOccupancyTest(unittest.TestCase):
    def setUp(self):
        self.image = np.full((100, 200, 3), 120, np.uint8)
        self.map_data = {"map_id": "test", "id_assignment_revision": "test-v1",
                         "map_image_sha256": "a" * 64, "nodes": [
            {"region_id": "space_1", "role": "parking_normal"},
            {"region_id": "space_2", "role": "parking_accessible"},
        ]}
        self.regions = {
            **map_identity(self.map_data), "coordinate_frame": "camera_image",
            "image_size_px": [200, 100], "empty_reference_verified": True,
            "spaces": [{"parking_space_id": "space_1",
                        "polygon_px": [[10, 10], [90, 10], [90, 90], [10, 90]]}],
        }

    def snapshot(self, detections=(), **kwargs):
        return build_snapshot(self.map_data, self.image, detections, "2026-11-11T12:00:00+00:00",
                              regions=self.regions, **kwargs)

    def test_absence_and_missing_roi_are_unknown(self):
        self.assertEqual([s["status"] for s in self.snapshot()["spaces"]], ["unknown", "unknown"])

    def test_car_overrides_matching_background_and_weak_detection_blocks_free(self):
        for confidence, expected in ((0.95, "occupied"), (0.15, "unknown")):
            detection = {"bbox_xywh_px": [20, 20, 40, 40], "confidence": confidence}
            snapshot = self.snapshot([detection], empty_reference=self.image)
            self.assertEqual(snapshot["spaces"][0]["status"], expected)

    def test_free_requires_verified_background_match(self):
        snapshot = self.snapshot(empty_reference=self.image)
        self.assertEqual(snapshot["spaces"][0]["status"], "free")
        self.assertEqual(map_identity(snapshot), map_identity(self.map_data))
        from datetime import datetime, timezone
        states = occupancy_states(self.map_data, snapshot, now=datetime(2026, 11, 11, 12, tzinfo=timezone.utc))
        self.assertEqual(states, {"space_1": "free", "space_2": "unknown"})
        self.image[20:80, 20:80] = 10  # 차량이 아닌 장애물도 빈칸으로 처리하지 않는다.
        self.assertEqual(self.snapshot(empty_reference=np.full_like(self.image, 120))["spaces"][0]["status"], "unknown")
        self.regions["empty_reference_verified"] = False
        with self.assertRaisesRegex(ValueError, "확인"):
            self.snapshot(empty_reference=self.image)

    def test_reject_wrong_map_resolution_and_duplicate_regions(self):
        for key, value in (("map_id", "other"), ("image_size_px", [100, 200]),
                           ("id_assignment_revision", "old"), ("map_image_sha256", "b" * 64)):
            original = self.regions[key]
            self.regions[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                self.snapshot()
            self.regions[key] = original
        self.regions["spaces"].append(self.regions["spaces"][0])
        with self.assertRaisesRegex(ValueError, "중복"):
            self.snapshot()

    def test_letterbox_roundtrip_and_nms(self):
        class Network:
            def setInput(inner, blob):
                self.assertEqual(blob.shape, (1, 3, 640, 640))
                self.assertAlmostEqual(float(blob[0, 0, 0, 0]), 114/255, places=6)

            def forward(inner):
                output = np.zeros((1, 5, 8400), np.float32)
                # 200x100 영상을 640 정사각형으로 변환: 배율 3.2, y 패딩 160.
                output[0, :, 0] = [160, 320, 128, 192, 0.95]
                output[0, :, 1] = [160, 320, 128, 192, 0.8]
                return output

        detector = CarDetector.__new__(CarDetector)
        detector.net = Network()
        detections = detector.detect(self.image)
        self.assertEqual(len(detections), 1)
        np.testing.assert_allclose(detections[0]["bbox_xywh_px"], [30, 20, 40, 60], atol=0.001)


if __name__ == "__main__":
    unittest.main()
