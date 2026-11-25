"""실주행 시작 전에 잘못된 경로·출발 위치를 차단하는 회귀시험."""
from contextlib import contextmanager
import io
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import node_follower as follower


def route():
    return {"map_id": "test", "coordinate_frame": "world", "unit": "cm",
            "origin": "paper_top_left", "axis_directions": {"x": "right", "y": "down"},
            "calibration_verified": True, "map_image_sha256": "a" * 64,
            "id_assignment_revision": "labels-v1", "waypoints": [[0, 0], [20, 0]]}


def pose(x=0, y=0):
    return {"x_cm": x, "y_cm": y, "heading_deg": 0, "position_samples": [(x, y)] * 3}


@contextmanager
def startup(data, observations):
    argv = ["node_follower.py", "--route", "unused.json", "--host", "test-rover",
            "--track-width-cm", "12", "--drive"]
    with patch.object(sys, "argv", argv), \
            patch("builtins.open", return_value=io.StringIO(json.dumps(data))), \
            patch.object(follower, "Poses") as poses, \
            patch.object(follower, "Rover") as rover, \
            patch.object(follower, "wait_settled", side_effect=observations), \
            patch.object(follower, "follow") as follow, \
            patch("sys.stdout", io.StringIO()):
        yield SimpleNamespace(poses=poses, rover=rover, follow=follow)


class DriveGuardTests(unittest.TestCase):
    def test_accepts_verified_team_route(self):
        nodes, heading = follower.load_route(route(), require_verified=True)
        self.assertEqual(nodes, [(0, 0), (20, 0)])
        self.assertIsNone(heading)

    def test_legacy_nodes_remain_diagnostic_only(self):
        data = {"map_id": "test", "unit": "cm", "nodes": [[20, 0]]}
        self.assertEqual(follower.load_route(data)[0], [(20, 0)])
        with self.assertRaises(ValueError):
            follower.load_route(data, require_verified=True)

    def test_missing_or_wrong_drive_metadata_is_rejected(self):
        for key in ("map_image_sha256", "id_assignment_revision", "axis_directions",
                    "calibration_verified", "coordinate_frame", "unit", "origin"):
            data = route()
            del data[key]
            with self.subTest(missing=key), self.assertRaises(ValueError):
                follower.load_route(data, require_verified=True)
        for changes in ({"calibration_verified": False}, {"unit": "pixel"},
                        {"coordinate_frame": "world_estimate"}, {"origin": "other"},
                        {"map_image_sha256": "not-a-hash"}, {"map_image_sha256": "g" * 64},
                        {"id_assignment_revision": " "}, {"map_image_sha256": 123},
                        {"axis_directions": {"x": "left", "y": "up"}}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                follower.load_route(dict(route(), **changes), require_verified=True)

    def test_wrong_explicit_axes_are_rejected_in_diagnostics_too(self):
        data = dict(route(), axis_directions={"x": "left", "y": "up"})
        with self.assertRaises(ValueError):
            follower.load_route(data)

    def test_drive_needs_distinct_start_and_goal(self):
        for points in ([], [[0, 0]], [[0, 0], [0, 0]]):
            with self.subTest(points=points), self.assertRaises(ValueError):
                follower.load_route(dict(route(), waypoints=points), require_verified=True)

    def test_start_radius_boundary_and_all_observations(self):
        follower.require_start_position(pose(3, 0), (0, 0))
        for data in (pose(3.01, 0), dict(pose(), position_samples=[(0, 0), (0, 0), (3.01, 0)]),
                     dict(pose(), position_samples=[]), dict(pose(), position_samples=[(0, 0)])):
            with self.subTest(pose=data), self.assertRaises(ValueError):
                follower.require_start_position(data, (0, 0))

    def test_invalid_route_is_rejected_before_any_socket(self):
        data = route()
        del data["map_image_sha256"]
        with startup(data, []) as calls:
            with self.assertRaises(ValueError):
                follower.main()
            calls.poses.assert_not_called()
            calls.rover.assert_not_called()
            calls.follow.assert_not_called()

    def test_far_start_never_connects_to_rover(self):
        with startup(route(), [pose(100, 100)]) as calls:
            with self.assertRaises(ValueError):
                follower.main()
            calls.rover.assert_not_called()
            calls.follow.assert_not_called()
            calls.poses.return_value.close.assert_called_once()

    def test_position_change_during_connection_prevents_steps_and_closes(self):
        with startup(route(), [pose(), pose(3.01, 0)]) as calls:
            with self.assertRaises(ValueError):
                follower.main()
            calls.follow.assert_not_called()
            calls.rover.return_value.request.assert_called_once_with("stop")
            calls.rover.return_value.close.assert_called_once()
            calls.poses.return_value.close.assert_called_once()

    def test_valid_start_reaches_follower(self):
        with startup(route(), [pose(), pose()]) as calls:
            follower.main()
            calls.rover.assert_called_once_with("test-rover")
            calls.follow.assert_called_once()
            calls.rover.return_value.close.assert_called_once()
            calls.poses.return_value.close.assert_called_once()


if __name__ == "__main__":
    unittest.main(verbosity=2)
