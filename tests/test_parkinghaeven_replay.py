import copy
import hashlib
import json
import unittest

from scripts.parkinghaeven_replay import convert_export, replay_fixture
from src.map_processing.occupancy import occupancy_states


class ParkingHaevenReplayTest(unittest.TestCase):
    def setUp(self):
        self.entries = [
            {"id": 11, "points": [{"x": 10, "y": 10}], "occupied": True},
            {"id": 22, "points": [{"x": 30, "y": 10}], "occupied": False},
        ]
        geometry = {str(r["id"]): r["points"] for r in self.entries}
        digest = hashlib.sha256(json.dumps(geometry, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
        self.payload = {"LIVE_WEBCAM": self.entries}
        self.mapping = {
            "map_id": "test_map",
            "source": {"source_id": "LIVE_WEBCAM", "region_geometry_sha256": digest},
            "spaces": [
                {"source_region_id": "11", "parking_space_id": "space_1"},
                {"source_region_id": "22", "parking_space_id": "space_2"},
            ],
        }

    def test_saved_boolean_translates_by_id_and_keeps_missing_metadata(self):
        snapshot = convert_export(self.payload, self.mapping)
        self.assertEqual([s["status"] for s in snapshot["spaces"]], ["occupied", "free"])
        self.assertIsNone(snapshot["observed_at"])
        self.assertTrue(all(s["confidence"] is None for s in snapshot["spaces"]))
        self.entries.reverse()
        self.assertEqual(convert_export(self.payload, self.mapping), snapshot)

    def test_geometry_drift_is_rejected(self):
        self.entries[0]["points"][0]["x"] += 1
        with self.assertRaisesRegex(ValueError, "좌표"):
            convert_export(self.payload, self.mapping)

    def test_invalidated_mapping_is_rejected_before_using_saved_states(self):
        self.mapping["status"] = "requires_new_camera_regions"
        with self.assertRaisesRegex(ValueError, "재설정"):
            convert_export(self.payload, self.mapping)

    def test_duplicates_and_missing_boolean_are_rejected(self):
        original = copy.deepcopy(self.payload)
        for malformed in (self.entries + [self.entries[0]],
                          [{**self.entries[0], "occupied": None}, self.entries[1]]):
            with self.subTest(entries=malformed), self.assertRaises(ValueError):
                convert_export({"LIVE_WEBCAM": malformed}, self.mapping)
        self.assertEqual(self.payload, original)
        self.mapping["spaces"][1]["parking_space_id"] = "space_1"
        with self.assertRaisesRegex(ValueError, "같은 주차칸"):
            convert_export(self.payload, self.mapping)

    def test_replay_does_not_invent_observation_metadata_in_saved_snapshot(self):
        snapshot = convert_export(self.payload, self.mapping)
        original = copy.deepcopy(snapshot)
        fixture = replay_fixture(snapshot)
        self.assertEqual(snapshot, original)
        self.assertEqual(fixture["confidence_basis"], "synthetic_test_truth_not_detector_confidence")
        self.assertTrue(fixture["observed_at"].startswith("2000-01-01"))
        map_data = {"map_id": "test_map", "nodes": []}
        with self.assertRaisesRegex(ValueError, "observed_at"):
            occupancy_states(map_data, snapshot)


if __name__ == "__main__":
    unittest.main()
