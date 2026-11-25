"""실제 웹 진입점을 모의 시계·엔코더·카메라로 실행한다."""
from contextlib import nullcontext, redirect_stdout
from copy import deepcopy
import io
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

APP = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(APP), str(APP.parents[1]), str(Path(__file__).parent)]
from simulate import RoverModel, DT, tuning
from vehicle_geometry import VehicleGeometry, overlaps
import web_drive as web
from web_navigation import RoadMap, WebPath
from node_follower import segment_distance
from src.map_processing.planar_calibration import project_points
from src.rover_control.pose_sender import CapturePose, load_camera_position
from src.rover_control.arrival_geometry import arrival_circle


class Replay:
    def __init__(self, state, start, heading=0):
        self.state = deepcopy(state)
        self.model = RoverModel(width=15, x=start[0], y=start[1], heading=heading)
        self.now = 10.0
        self.frame = None
        self.commands = []
        self.pending = None
        self.stopped = self.now
        self.loss_until = 0
        self.loss_count = 0
        self.on_step = lambda: None
        self.on_finish = lambda: None

    def sleep(self, duration):
        self.now += duration
        if self.now > 300:
            raise AssertionError('모의 주행 시간 초과')

    def current(self):
        # 같은 카메라 프레임을 여러 번 조회할 수 있다.
        stamp = math.floor(self.now * 12.5) / 12.5
        if self.frame is None or stamp != self.frame['captured_monotonic_s']:
            self.frame = dict(x_cm=self.model.x, y_cm=self.model.y,
                              heading_deg=self.model.heading, captured_monotonic_s=stamp)
        lost = self.now < self.loss_until
        state = dict(self.state, **({'status': 'waiting_for_tracking', 'guidance_available': False,
                                     'tracking_available': False} if lost else {}))
        occupancy = {**state, 'fresh': True, 'ttl_seconds': 5, 'age_seconds': 0.02,
                     'states': {state['parking_space_id']: 'free'}}
        return dict(state=state, occupancy=occupancy, pose=None if lost else self.frame, loss_count=self.loss_count)

    def start(self, timeout):
        return self.current()

    def request(self, kind, **params):
        if kind in ('timed_step', 'step'):
            assert self.pending is None and self.now >= self.loss_until
            assert self.now >= self.stopped + web.MCU_BRAKE_S
            self.pending_kind = kind
            amount = params['duration_ms'] if kind == 'timed_step' else params['pulses']
            self.pending = params['mode'], amount
            if kind == 'timed_step':
                assert 40 <= amount <= 200
            else:
                assert 1 <= amount <= (5 if params['mode'] == 1 else 2)
            self.commands.append(('step', self.frame['captured_monotonic_s'], self.pending))
            self.on_step()
            return 1
        if kind == 'stop':
            self.commands.append((kind, self.now, self.pending))
            self.pending, self.stopped = None, self.now
        elif self.pending:
            if self.pending_kind == 'step':
                self.model.run_job(self.pending)
            else:
                mode, duration = self.pending
                self.model.jobs.append(self.pending)
                pwm = tuning('TIMED_FORWARD_PWM') if mode == 1 else tuning('TIMED_TURN_PWM')
                direction = {1: (1, 1), 2: (1, -1), 3: (-1, 1)}[mode]
                for _ in range(round(duration / 1000 / DT)):
                    self.model.advance((pwm, pwm), direction)
                for _ in range(round(0.60 / DT)):
                    self.model.advance((0, 0))
            self.pending, self.stopped = None, self.now
            self.on_finish()
        return 0

    def close(self):
        pass


class PointDriveTests(unittest.TestCase):
    def test_progress_stays_on_forward_route_near_logged_p1_branch(self):
        start, goal = (9.32, 35.36), (38, 66)
        points = [start, (9, 39), (14, 39), (19, 39), (24, 39), (28, 39), (35, 39), (38, 40), goal]
        roads = SimpleNamespace(path=lambda s, g: points)
        path = WebPath(roads, start, goal, 2)
        for x, y in [(9.32, 35.36), (9, 39), (14, 39), (19, 37), (24, 38), (28, 39), (25, 36)]:
            progress = path.progress
            path.target(dict(x_cm=x, y_cm=y, heading_deg=40))
            self.assertGreaterEqual(path.progress, progress)
            self.assertEqual(path.points[0], start)
        self.assertGreater(path.point_at(path.progress + 5)[0], 30)

    def test_progress_window_does_not_jump_to_near_parallel_return_road(self):
        roads = SimpleNamespace(path=lambda s, g: [s, (30, 0), (30, 2), g])
        path = WebPath(roads, (0, 0), (0, 2), 0.5)
        path.target(dict(x_cm=1, y_cm=2, heading_deg=0))
        self.assertEqual(path.index, 1)
        self.assertLess(path.progress, 8)

    def test_encoder_jobs_use_more_pulses_on_straight_and_one_at_goal(self):
        roads = SimpleNamespace(path=lambda s, g: [s, g])
        path = WebPath(roads, (0, 0), (20, 0), 2)
        self.assertEqual(path.encoder_action(dict(x_cm=0, y_cm=0, heading_deg=0), 15), (1, 5))
        self.assertEqual(path.encoder_action(dict(x_cm=0, y_cm=0, heading_deg=90), 15), (3, 2))
        path = WebPath(roads, (17.5, 0), (20, 0), 2)
        self.assertEqual(path.encoder_action(dict(x_cm=17.5, y_cm=0, heading_deg=0), 15), (1, 1))

    def test_intersection_target_stays_at_corner_until_approach(self):
        roads = SimpleNamespace(path=lambda s, g: [s, (10, 0), g])
        path = WebPath(roads, (0, 0), (10, 20), 2)
        error, distance = path.target(dict(x_cm=5, y_cm=0, heading_deg=0))
        self.assertAlmostEqual(error, 0)
        self.assertAlmostEqual(distance, 5)
        action = path.encoder_action(dict(x_cm=9, y_cm=0, heading_deg=0), 15)
        self.assertEqual(action, (2, 2))
        action = path.encoder_action(dict(x_cm=10, y_cm=3, heading_deg=90), 15)
        self.assertEqual(action, (1, 5))

    def test_crossing_and_parallel_edges_preserve_topology_and_direction(self):
        def road_map(paths):
            nodes = {}
            edges = []
            for u, v, pixels in paths:
                nodes.update({u: pixels[0], v: pixels[-1]})
                edges.append({'from': u, 'to': v, 'path': pixels})
            semantic = dict(nodes=[dict(id=k, x=x, y=y) for k, (x, y) in nodes.items()],
                            edges=edges, regions=[])
            converter = SimpleNamespace(world_to_image=np.eye(3), image_size=np.array([100, 100]),
                                        paper_size=np.array([100, 100]))
            return RoadMap(semantic, converter)
        roads = road_map([('a', 'b', [(0, 20), (20, 20), (40, 20)]),
                          ('c', 'd', [(20, 0), (20, 20), (20, 40)])])
        with self.assertRaisesRegex(ValueError, '연결되지'):
            roads.path((5, 20), (20, 35))
        roads = road_map([('a', 'b', [(0, 10), (0, 35), (40, 35), (40, 10)]),
                          ('a', 'b', [(0, 10), (20, 10), (40, 10)])])
        self.assertEqual(roads.path((39, 10), (3, 10)), [(39, 10), (3, 10)])

    def test_arrival_boundary_is_inclusive_with_any_heading(self):
        for heading in (-180, -90, 0, 90, 179):
            path = WebPath(None, (12, 10), (10, 10), 2)
            self.assertIsNone(path.action(dict(x_cm=12, y_cm=10, heading_deg=heading), 15))
            self.assertFalse(path.arrived((12.0001, 10)))

    def test_retarget_brakes_and_stop_while_tracking_is_lost_ends_drive(self):
        identity = dict(map_id='test', map_image_sha256='a' * 64, id_assignment_revision='v1')
        state = dict(identity, status='running', capture_id='capture', case_id='accessible',
                     started_at='session', parking_space_id='left', height_correction_applied=True,
                     coordinate_frame='map_image', unit='pixel', route_revision=1, guidance_available=True)
        roads = SimpleNamespace(parking_center={'left': (30, 10), 'right': (10, 30)}.__getitem__,
                                path=lambda start, goal: [start, goal])
        for scenario in ('retarget', 'stop_during_loss'):
            with self.subTest(scenario=scenario):
                replay = Replay(state, (10, 10))
                replay.stopped -= 1
                initial = replay.current()
                path = web.make_plan(initial, roads, 2)
                def interrupt():
                    if len(replay.commands) == 1:
                        if scenario == 'retarget':
                            replay.state.update(route_revision=2, parking_space_id='right')
                        else:
                            replay.loss_until = replay.now + 1
                            replay.loss_count = 1
                replay.on_step = interrupt
                def observe():
                    bundle = replay.current()
                    if scenario == 'stop_during_loss' and replay.now > 10.25:
                        bundle['state']['status'] = 'stopped'
                    return bundle
                clock = SimpleNamespace(monotonic=lambda: replay.now, sleep=replay.sleep)
                def drive():
                    web.drive_web(SimpleNamespace(current=observe), replay, roads, initial, path, 15,
                                  identity, log=lambda *args, **kwargs: None, encoder_drive=True)
                with (patch.object(web, 'time', clock),
                      patch.object(web, 'braked_work', side_effect=lambda rover, work: work())):
                    if scenario == 'retarget':
                        drive()
                        self.assertLessEqual(math.dist((replay.model.x, replay.model.y), (10, 30)), 2)
                    else:
                        with self.assertRaisesRegex(RuntimeError, '중단'):
                            drive()
                        self.assertEqual(sum(k == 'step' for k, _, _ in replay.commands), 1)
                self.assertEqual([k for k, _, _ in replay.commands[:2]], ['step', 'stop'])
                self.assertEqual(replay.commands[-1][0], 'stop')


class SavedCaptureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        repo = APP.parents[1]
        folder = repo / 'data/http_map_captures/latest'
        arrival = repo / 'data/rover_runs/arrival/latest.json'
        if not arrival.is_file() or not (folder / 'world/camera_position.json').is_file():
            raise unittest.SkipTest('로컬 저장 캡처가 없습니다.')
        cls.state = json.loads(arrival.read_bytes())
        camera = load_camera_position(folder)
        cls.converter = CapturePose(folder, camera_position_cm=camera['camera_position_cm'],
                                    marker_height_cm=camera['marker_height_cm'])
        if cls.state.get('capture_id') != json.loads(cls.converter.report_bytes)['capture_id']:
            raise unittest.SkipTest('도착 관측과 저장 캡처가 다릅니다.')
        cls.semantic = json.loads((folder / 'semantic.json').read_bytes())
        cls.roads = RoadMap(cls.semantic, cls.converter)
        first = cls.semantic['nodes'][0]
        observed = cls.state.get('current_position_px') or cls.state.get('last_known_position_px')
        origin_px = observed or [first['x'], first['y']]
        cls.origin = cls.roads.to_world([origin_px])[0]
        # 실행 중인 웹의 선택·마커 유실 상태에 영향받지 않도록 같은 캡처에 모의 선택을 만든다.
        region = next(r for r in cls.semantic['regions'] if r['id'] == 'space_6')
        cls.state.update(status='running', guidance_available=True, tracking_available=True,
                         parking_space_id='space_6', case_id='02_accessible',
                         goal_px=arrival_circle(region)['center_px'], current_position_px=origin_px,
                         waypoints=cls.state.get('waypoints') or [[first['x'], first['y']], region['center']])

    def run_cli(self, replay, drive, encoder=True):
        plans = []
        def record(bundle, roads, radius):
            path = make_plan(bundle, roads, radius)
            replay.path = path
            plans.append(path)
            return path
        make_plan = web.make_plan
        args = ['web_drive.py', '--usb', '--track-width-cm', '15']
        if drive:
            args.append('--drive')
        if encoder:
            args.append('--encoder-drive')
        clock = SimpleNamespace(monotonic=lambda: replay.now, sleep=replay.sleep)
        with (patch.object(sys, 'argv', args), patch.object(web, 'WebFeed', return_value=replay),
              patch.object(web, 'Rover', return_value=replay) as motor,
              patch('src.rover_control.connection.rover_endpoint', return_value=nullcontext(('mock', 0))) as usb,
              patch.object(web, 'time', clock), patch.object(web, 'braked_work', side_effect=lambda r, f: f()),
              patch.object(web, 'make_plan', side_effect=record), redirect_stdout(io.StringIO())):
            self.assertEqual(web.main(), 0)
        self.assertEqual(motor.called, drive)
        self.assertEqual(usb.called, drive)
        return plans

    def test_encoder_mode_reaches_three_spaces_with_multi_pulse_forward_jobs(self):
        for space, case in [('space_6', self.state['case_id']), ('space_1', '01_building_a'), ('space_5', '01_building_a')]:
            with self.subTest(space=space):
                replay = Replay(dict(self.state, parking_space_id=space, case_id=case), self.origin)
                plans = self.run_cli(replay, True, encoder=True)
                self.assertTrue(any(mode == 1 and pulses > 1 for mode, pulses in replay.model.jobs))
                self.assertLessEqual(math.dist((replay.model.x, replay.model.y), plans[-1].goal), 2)

    def test_same_non_entrance_start_three_selected_spaces_preflight_and_drive(self):
        x, y, w, h = next(r['bbox'] for r in self.semantic['regions'] if r['id'] == 'building_a')
        wall = project_points([(x, y), (x+w, y), (x+w, y+h), (x, y+h)], self.roads.matrix).tolist()
        # 현재 실물 로버가 어디에 있든 건물 가장자리에서 가상 차체가 겹치는 조건을 재현한다.
        start = self.roads.to_world([(x + 0.75*w, y)])[0]
        box = VehicleGeometry(self.converter.paper_size.tolist()).rectangle(*start, 0)
        self.assertTrue(overlaps(box, wall))
        old_start = self.roads.to_world([self.state['waypoints'][0]])[0]
        self.assertGreater(math.dist(old_start, start), 5)
        entrance = next(r for r in self.semantic['regions'] if r['id'] == 'entrance')
        self.assertGreater(math.dist(self.roads.to_world([entrance['center']])[0], start), 5)
        for space, case in [('space_6', self.state['case_id']), ('space_1', '01_building_a'), ('space_5', '01_building_a')]:
            with self.subTest(space=space):
                # 고의로 저장 경로를 그대로 둔다. 목적지는 현재 선택 ID로만 정해져야 한다.
                state = dict(self.state, parking_space_id=space, case_id=case)
                dry = Replay(state, start)
                path, = self.run_cli(dry, False)
                self.assertEqual(path.points[0], start)
                region = next(r for r in self.semantic['regions'] if r['id'] == space)
                center_px = arrival_circle(region)['center_px']
                expected = project_points([center_px], np.linalg.inv(self.converter.world_to_image))[0]
                np.testing.assert_allclose(path.points[-1], expected, rtol=0, atol=1e-9)
                if space == self.state['parking_space_id']:
                    np.testing.assert_allclose(center_px, self.state['goal_px'], rtol=0, atol=1e-9)
                replay = Replay(state, start)
                actual, = self.run_cli(replay, True)
                self.assertEqual(path.points, actual.points)
                frames = [stamp for kind, stamp, _ in replay.commands if kind == 'step']
                self.assertGreater(len(frames), 0)
                self.assertEqual(frames, sorted(set(frames)))
                self.assertLessEqual(math.dist((replay.model.x, replay.model.y), expected), 2)
                self.assertEqual(replay.commands[-1][0], 'stop')

    def test_large_disturbance_and_tracking_loss_resume_in_same_drive(self):
        replay = Replay(self.state, self.origin)
        deviations = []
        def disturb():
            if len(replay.model.jobs) == 5:
                path = replay.path
                a, b = path.points[path.index-1:path.index+1]
                dx, dy = b[0]-a[0], b[1]-a[1]
                length = math.hypot(dx, dy)
                replay.model.x -= 10 * dy / length
                replay.model.y += 10 * dx / length
                deviations.append(segment_distance((replay.model.x, replay.model.y), a, b))
        def lose():
            if len(replay.model.jobs) == 11 and not replay.loss_count:
                replay.loss_count = 1
                replay.loss_until = replay.now + 0.8
        replay.on_finish, replay.on_step = disturb, lose
        plans = self.run_cli(replay, True)
        self.assertGreater(deviations[0], 5)
        self.assertNotEqual(plans[0].points[0], self.origin)  # 이탈 위치에서 같은 목적지로 다시 탐색
        self.assertTrue(any(k == 'stop' and p is not None and t < replay.loss_until
                            for k, t, p in replay.commands))
        self.assertTrue(any(k == 'step' and t > replay.loss_until for k, t, _ in replay.commands))
        self.assertLessEqual(math.dist((replay.model.x, replay.model.y), plans[-1].goal), 2)

    def test_arrival_on_first_frame_and_during_active_job(self):
        goal = self.roads.parking_center(self.state['parking_space_id'])
        replay = Replay(self.state, goal, heading=-175)
        self.run_cli(replay, True)
        self.assertEqual({k for k, _, _ in replay.commands}, {'stop'})
        replay = Replay(self.state, self.origin)
        def enter_goal():
            replay.model.x, replay.model.y = goal[0], goal[1] + 1
            replay.model.heading = -175
            replay.now += 0.08
        replay.on_step = enter_goal
        self.run_cli(replay, True)
        self.assertEqual([k for k, _, _ in replay.commands], ['stop', 'step', 'stop'])
        self.assertIsNotNone(replay.commands[-1][2])
