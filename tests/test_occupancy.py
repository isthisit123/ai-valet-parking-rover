import copy
import unittest
from datetime import datetime, timedelta, timezone

from src.map_processing.occupancy import occupancy_states
from src.map_processing.parking_selector import load_parking_map, select_parking_from_map


class OccupancyTest(unittest.TestCase):
    def setUp(self):
        self.map_data, self.graph = load_parking_map()
        self.now = datetime(2026, 11, 11, 12, tzinfo=timezone.utc)
        self.snapshot = {
            "schema_version": "0.1-draft",
            "map_id": self.map_data["map_id"],
            "observed_at": self.now.isoformat(),
            "spaces": [
                {"parking_space_id": "space_1", "status": "occupied", "confidence": 0.9},
                {"parking_space_id": "space_2", "status": "free", "confidence": 0.9},
            ],
        }

    def select(self, **kwargs):
        return select_parking_from_map(
            self.map_data, self.graph, "entrance", occupancy=self.snapshot,
            now=self.now, **kwargs,
        )

    def test_only_explicit_free_space_is_selected(self):
        result = self.select()
        self.assertEqual(result["parking_space_id"], "space_2")
        self.assertEqual(self.graph.nodes[result["selected"]]["region_id"], "space_2")
        self.assertEqual(result["occupancy_status"], "free")
        self.assertEqual(result["route"][0], "N3")

    def test_stale_future_missing_unknown_and_low_confidence_are_not_free(self):
        for age in (6, -1):
            with self.subTest(age=age):
                self.snapshot["observed_at"] = (self.now - timedelta(seconds=age)).isoformat()
                with self.assertRaisesRegex(ValueError, "free"):
                    self.select()
        self.snapshot["observed_at"] = self.now.isoformat()
        for status, confidence in (("unknown", None), ("occupied", 0.9),
                                   ("free", None), ("free", 0.2)):
            with self.subTest(status=status, confidence=confidence):
                self.snapshot["spaces"][1].update(status=status, confidence=confidence)
                with self.assertRaisesRegex(ValueError, "free"):
                    self.select()
        self.snapshot["spaces"] = []
        with self.assertRaisesRegex(ValueError, "free"):
            self.select()

    def test_invalid_contract_is_rejected(self):
        for key, value in (("map_id", "another_map"), ("schema_version", "2"),
                           ("observed_at", "2026-11-11T12:00:00")):
            snapshot = {**self.snapshot, key: value}
            with self.subTest(key=key), self.assertRaises(ValueError):
                occupancy_states(self.map_data, snapshot, now=self.now)
        for change in ({"parking_space_id": "N0"}, {"status": "FREE"},
                       {"confidence": float("nan")}, {"confidence": True}):
            snapshot = copy.deepcopy(self.snapshot)
            snapshot["spaces"][0].update(change)
            with self.subTest(change=change), self.assertRaises(ValueError):
                occupancy_states(self.map_data, snapshot, now=self.now)
        self.snapshot["spaces"].append(self.snapshot["spaces"][0])
        with self.assertRaisesRegex(ValueError, "중복"):
            self.select()

    def test_accessible_is_not_a_normal_candidate(self):
        self.snapshot["spaces"] = [
            {"parking_space_id": "space_5", "status": "free", "confidence": 0.9}
        ]
        with self.assertRaisesRegex(ValueError, "free"):
            self.select()
        self.assertEqual(self.select(parking_type="accessible")["parking_space_id"], "space_5")

    def test_route_metadata_and_legacy_selection(self):
        import cv2
        import numpy as np
        from src.map_processing.map_config import DEFAULT_MAP
        from src.map_processing.path_visualizer import build_route_data

        image = cv2.imdecode(np.fromfile(DEFAULT_MAP.image_path, dtype=np.uint8), cv2.IMREAD_COLOR)
        # 실제 도로 마스크로 선택된 경로의 연속성까지 검사한다.
        from src.map_processing.map_reader import detect_road_mask
        from src.map_processing.graph_builder import remove_small_components
        route = build_route_data(
            self.map_data["map_id"], self.map_data, self.graph, "entrance",
            remove_small_components(detect_road_mask(image)),
            occupancy=self.snapshot, now=self.now,
        )
        self.assertEqual(route["parking_space_id"], "space_2")
        self.assertEqual(route["occupancy_observed_at"], self.snapshot["observed_at"])
        self.assertGreater(len(route["waypoints"]), 1)
        legacy = select_parking_from_map(self.map_data, self.graph, "entrance")
        self.assertEqual(legacy["selected"], "N0")
        self.assertNotIn("occupancy_status", legacy)


if __name__ == "__main__":
    unittest.main()
