from contextlib import redirect_stdout
from datetime import datetime, timezone
import io
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from src.map_processing.map_config import BASE_DIR, DEFAULT_MAP
from src.map_processing.map_identity import configured_map_identity, map_identity
from src.map_processing.planar_calibration import project_points
from src.rover_control.__main__ import prepare_map
from src.rover_control.map_preview import floor_transform
from src.rover_control.tracking import load_route


class RoverOccupancyTest(unittest.TestCase):
    def setUp(self):
        self.run_dir = Path(self.enterContext(tempfile.TemporaryDirectory()))
        self.points = BASE_DIR / "floor_points.json"
        self.snapshot = {
            "schema_version": "0.1-draft",
            **configured_map_identity(DEFAULT_MAP),
            "observed_at": datetime.now(timezone.utc).isoformat(),
            "spaces": [
                {"parking_space_id": "space_1", "status": "occupied", "confidence": 0.95},
                {"parking_space_id": "space_2", "status": "free", "confidence": 0.95},
            ],
        }

    def test_selected_free_space_is_preserved_in_cm_preview(self):
        with redirect_stdout(io.StringIO()):
            preview = prepare_map(DEFAULT_MAP.map_id, self.points, self.run_dir,
                                  "01_building_a", 1.0, occupancy=self.snapshot, max_age_seconds=60)
        result = json.loads((preview / "01_building_a_map_cm.json").read_text(encoding="utf-8"))
        route = result["route"]
        self.assertEqual(route["parking_space_id"], "space_2")
        self.assertEqual(route["occupancy_status"], "free")
        self.assertEqual(route["occupancy_observed_at"], self.snapshot["observed_at"])
        self.assertEqual(route["waypoints_cm"][-1], route["parking_space_center_cm"])
        self.assertFalse(result["calibration_verified"])
        self.assertEqual(map_identity(result), map_identity(self.snapshot))
        routes = preview.parent / "routes"
        self.assertEqual([path.name for path in routes.glob("*.json")], ["01_building_a.json"])

    def test_reject_old_id_revision_and_other_map_photo(self):
        old_snapshot = {**self.snapshot, "id_assignment_revision": "old"}
        with self.assertRaisesRegex(ValueError, "id_assignment_revision"):
            prepare_map(DEFAULT_MAP.map_id, self.points, self.run_dir, None, 1.0, occupancy=old_snapshot)

        wrong_points = self.run_dir / "wrong_points.json"
        spec = json.loads(self.points.read_text(encoding="utf-8"))
        wrong_points.write_text(json.dumps({**spec, "image_sha256": "0" * 64}), encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "현재 지도 이미지가 다릅니다"):
            prepare_map(DEFAULT_MAP.map_id, wrong_points, self.run_dir, None, 1.0)
        self.assertFalse((self.run_dir / DEFAULT_MAP.map_id).exists())

    def test_drive_accepts_current_calibrated_route_and_rejects_other_photo(self):
        # 수학 fixture로 검증 경로의 메타데이터와 주행 파일 읽기만 시험한다.
        image_bytes = DEFAULT_MAP.image_path.read_bytes()
        matrix, _ = floor_transform(DEFAULT_MAP.map_id, image_bytes, (1280, 720), points=self.points)
        points_cm = [[15, 15], [90, 15], [90, 65], [15, 65]]
        points_px = project_points(points_cm, np.linalg.inv(matrix))
        spec = json.loads(self.points.read_text(encoding="utf-8"))
        spec["control_points"] = [
            {"id": label, "pixel": pixel.tolist(), "world_cm": world}
            for label, pixel, world in zip("ABCD", points_px, points_cm, strict=True)
        ]
        points = self.run_dir / "test_points.json"
        points.write_text(json.dumps(spec), encoding="utf-8")
        with redirect_stdout(io.StringIO()):
            preview = prepare_map(DEFAULT_MAP.map_id, points, self.run_dir,
                                  "01_building_a", 1.0, occupancy=self.snapshot, max_age_seconds=60)
        route_path = preview.parent / "routes" / "01_building_a_cm.json"
        data = json.loads(route_path.read_text(encoding="utf-8"))
        self.assertTrue(data["calibration_verified"])
        self.assertEqual(map_identity(data), map_identity(self.snapshot))
        self.assertEqual(load_route(route_path, require_verified=True).points[-1], tuple(data["waypoints"][-1]))

        for fields, error in (
            ({"map_image_sha256": "0" * 64}, "map_image_sha256"),
            ({"calibration_verified": False}, "보정을 검증한"),
        ):
            with self.subTest(fields=fields):
                route_path.write_text(json.dumps({**data, **fields}), encoding="utf-8")
                with self.assertRaisesRegex(ValueError, error):
                    load_route(route_path, require_verified=True)
