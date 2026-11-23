"""코너 보존, 경로 이탈, 방향 수렴, 차체 외곽과 합격 판정 회귀시험."""
from copy import deepcopy
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from node_follower import PathGuard, follow, plan, simplify_path
from vehicle_geometry import VehicleGeometry
from simulate import validate_report


class NavigationTests(unittest.TestCase):
    def test_dense_corner_is_preserved_and_not_accepted_from_start(self):
        path = [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2), (2, 4)]
        self.assertEqual(simplify_path(path), [(0, 0), (2, 0), (2, 4)])
        actions = []
        def step(action):
            actions.append(action)
            raise RuntimeError('first action')
        with self.assertRaisesRegex(RuntimeError, 'first action'):
            follow(path, None, 12, step, {'x_cm': 0, 'y_cm': 0, 'heading_deg': 0},
                   path_mode=True, check=PathGuard(), log=lambda *a, **k: None)
        self.assertEqual(actions[0][0], 1)

    def test_collinear_reverse_is_not_shortcut(self):
        path = [(0, 0), (10, 0), (5, 0), (15, 0)]
        self.assertEqual(simplify_path(path), path)

    def test_active_segment_blocks_other_branch(self):
        guard = PathGuard()
        with self.assertRaises(RuntimeError):
            guard({'x_cm': 0, 'y_cm': 3}, ((0, 0), (20, 0)))
        guard({'x_cm': 5, 'y_cm': 1}, ((0, 0), (20, 0)))

    def test_last_position_uses_two_cm_goal(self):
        self.assertIsNotNone(plan({'x_cm': 2.5, 'y_cm': 0, 'heading_deg': 180}, (0, 0))[0])

    def test_heading_progress_can_take_more_than_thirty_seconds(self):
        pose = {'x_cm': 0, 'y_cm': 0, 'heading_deg': -80}
        elapsed = [0]
        def step(action):
            elapsed[0] += 1
            pose['heading_deg'] += 2
            return dict(pose)
        result = follow([(0, 0)], 0, 12, step, dict(pose), clock=lambda: elapsed[0], log=lambda *a, **k: None)
        self.assertGreater(elapsed[0], 30)
        self.assertLessEqual(abs(result['heading_deg']), 4.7)

    def test_no_heading_progress_still_stops(self):
        pose = {'x_cm': 0, 'y_cm': 0, 'heading_deg': -80}
        elapsed = [0]
        def step(action):
            elapsed[0] += 1
            return pose
        with self.assertRaises(TimeoutError):
            follow([(0, 0)], 0, 12, step, pose, clock=lambda: elapsed[0], log=lambda *a, **k: None)

    def test_body_hits_obstacle_even_when_center_is_clear(self):
        geometry = VehicleGeometry([100, 100], [('wall', [(30, 10), (35, 10), (35, 40), (30, 40)])])
        with self.assertRaisesRegex(ValueError, 'wall'):
            geometry.check((25, 25), 0)

    def test_turn_sweep_and_small_parking_are_rejected(self):
        target = [(15, 15), (25, 15), (25, 25), (15, 25)]
        geometry = VehicleGeometry([100, 100], target=target)
        with self.assertRaises(ValueError):
            geometry.check((20, 20), 0, parked=True)
        geometry = VehicleGeometry([100, 100])
        geometry.check((10, 20), 90)
        with self.assertRaises(ValueError):
            geometry.pivot((10, 20), 90, 90)

    def test_model_failures_cannot_return_a_passing_report(self):
        report = {'routes': {}, 'rotation': {}, 'simulated_fault_injection': {'stop': True}}
        for width in ('12', '15'):
            report['routes'][width] = {'runs': 25, 'completed': 25, 'within_position_and_angle': 25,
                'max_node_error_cm': 1.9, 'max_final_angle_error_deg': 4.9, 'failures': []}
            report['rotation'][f'low_pwm_noisy_pose_{width}cm'] = {'runs': 150,
                'true_error_within_5deg': 150, 'max_true_error_deg': 4.9, 'failures': []}
        validate_report(report)
        for changes in ({'completed': 24}, {'max_node_error_cm': 2.1}, {'max_final_angle_error_deg': 5.1}):
            bad = deepcopy(report)
            bad['routes']['12'].update(changes)
            with self.subTest(changes=changes), self.assertRaises(AssertionError):
                validate_report(bad)


if __name__ == '__main__':
    unittest.main(verbosity=2)
