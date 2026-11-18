from copy import deepcopy
from datetime import datetime, timedelta, timezone
from hashlib import sha256
import json
from math import dist
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from src.map_processing.parking_selector import load_parking_map, select_parking_from_map
from src.map_processing.path_visualizer import DEFAULT_ROUTE_CASES, build_route_data
from src.rover_control.arrival_test import ArrivalTest
from src.rover_control.live_parking import LiveParking


def write_live_map(root):
    folder = Path(root) / 'latest'
    (folder / 'routes').mkdir(parents=True)
    image = np.full((400, 400, 3), 255, np.uint8)
    cv2.line(image, (30, 150), (370, 150), (0, 0, 255), 7)
    nodes = [{'id': 'en', 'x': 30, 'y': 150, 'role': 'entrance'},
             {'id': 'ex', 'x': 370, 'y': 150, 'role': 'exit'}]
    regions, edges = [], []
    positions = {1: (80, 100), 2: (140, 100), 3: (200, 100),
                 4: (320, 200), 5: (260, 200), 6: (230, 200)}
    for number, (x, y) in positions.items():
        kind = 'accessible' if number in (3, 6) else 'normal'
        nodes += [{'id': f'j{number}', 'x': x, 'y': 150},
                  {'id': f'p{number}', 'x': x, 'y': y, 'role': f'parking_{kind}', 'region_id': f'space_{number}'}]
        edges.append({'from': f'j{number}', 'to': f'p{number}', 'weight': 50, 'path': [[x, 150], [x, y]]})
        cv2.line(image, (x, 150), (x, y), (0, 0, 255), 7)
        center_y = y + (30 if y > 150 else -30)
        width = 60 if number == 4 else 40
        regions.append({'id': f'space_{number}', 'type': kind, 'node_id': f'p{number}',
                        'bbox': [x - width / 2, center_y - 25, width, 50], 'center': [x, center_y], 'area': width * 50})
    line = sorted((n for n in nodes if n['y'] == 150), key=lambda n: n['x'])
    for a, b in zip(line, line[1:]):
        edges.append({'from': a['id'], 'to': b['id'], 'weight': b['x'] - a['x'],
                      'path': [[a['x'], 150], [b['x'], 150]]})
    regions += [{'id': 'building_a', 'bbox': [60, 40, 30, 30]},
                {'id': 'building_b', 'bbox': [320, 40, 30, 30]}]
    _, encoded = cv2.imencode('.png', image)
    body = encoded.tobytes()
    (folder / 'source.png').write_bytes(body)
    identity = {'map_id': 'live_map', 'id_assignment_revision': 'v1', 'map_image_sha256': sha256(body).hexdigest()}
    semantic = {**identity, 'unit': 'pixel', 'nodes': nodes, 'edges': edges, 'regions': regions}
    (folder / 'semantic.json').write_text(json.dumps(semantic))
    report = {**identity, 'capture_id': 'version-live', 'published_as_latest': True,
              'source_sha256': identity['map_image_sha256'], 'image_size_px': [400, 400],
              'camera_source': {'kind': 'webcam', 'session_id': 'camera-A', 'width': 400, 'height': 400},
              'route_generation': {'status': 'generated'}}
    (folder / 'report.json').write_text(json.dumps(report))
    _, graph = load_parking_map(folder / 'semantic.json')
    mask = np.any(image != 255, axis=2).astype(np.uint8) * 255
    for case_id, preference, kind in DEFAULT_ROUTE_CASES:
        route = build_route_data(identity['map_id'], semantic, graph, preference, mask, parking_type=kind)
        (folder / 'routes' / f'{case_id}.json').write_text(json.dumps(route))
    return report


class LiveParkingTest(unittest.TestCase):
    def setUp(self):
        self.root = Path(self.enterContext(TemporaryDirectory())) / '한글 지도'
        self.report = write_live_map(self.root)
        self.clock = 10.0
        self.enterContext(patch('time.monotonic', side_effect=lambda: self.clock))
        self.wall_epoch = datetime.now(timezone.utc) - timedelta(seconds=self.clock)
        for module in ('src.rover_control.live_parking', 'src.map_processing.occupancy',
                       'src.rover_control.arrival_test'):
            clock_datetime = self.enterContext(patch(f'{module}.datetime', wraps=datetime))
            clock_datetime.now.side_effect = lambda tz=None: self.wall_now().astimezone(tz)
        self.parking = LiveParking(self.root)
        self.arrival = ArrivalTest(self.root, parking=self.parking)
        self.sequence = 0

    def wall_now(self):
        return self.wall_epoch + timedelta(seconds=self.clock)

    def snapshot(self, **states):
        self.sequence += 1
        # 촬영·수신 시계를 함께 진행하고 같은 시험 시각의 새 프레임도 구별한다.
        observed = self.wall_now() - timedelta(milliseconds=20) + timedelta(microseconds=self.sequence)
        return {**{k: self.report[k] for k in ('map_id', 'id_assignment_revision', 'map_image_sha256', 'capture_id', 'camera_source')},
                'schema_version': '0.1-draft', 'observed_at': observed.isoformat(),
                'spaces': [{'parking_space_id': f'space_{i}', 'status': states.get(f'space_{i}', 'free'), 'confidence': 0.5}
                           for i in range(1, 7)]}

    def send(self, **states):
        data = self.snapshot(**states)
        return self.parking.update(data)

    def confirm(self, **states):
        for _ in range(3):
            self.send(**states)
            self.clock += 1.01
        self.clock -= 1.01

    def pose(self, x=100, y=150):
        return {'tracking_status': 'detected', 'rover': {'id': 7, 'center_px': [x, y]},
                'camera_source': self.report['camera_source'], 'width': 400, 'height': 400,
                'received_monotonic_s': self.clock, 'frame_id': str(self.clock)}

    def start(self, **options):
        return self.arrival.start('03_exit', capture_id=self.report['capture_id'], **options)

    def test_no_snapshot_unknown_and_two_second_free_confirmation(self):
        with self.assertRaisesRegex(ValueError, '최신 점유'):
            self.start()
        self.send()
        self.assertEqual(self.parking.current()['states']['space_4'], 'unknown')
        self.clock += 1
        self.send()
        self.assertEqual(self.parking.current()['states']['space_4'], 'unknown')
        self.clock += 1.01
        self.send()
        self.assertEqual(self.start()['parking_space_id'], 'space_4')

    def test_manual_occupied_rejected_and_automatic_selects_next_free(self):
        self.confirm(space_4='occupied')
        with self.assertRaisesRegex(ValueError, '이미 점유된 주차칸'):
            self.start(parking_space_id='space_4')
        self.assertEqual(self.start()['parking_space_id'], 'space_5')
        with self.assertRaisesRegex(ValueError, '유형'):
            self.start(parking_space_id='space_3')

    def test_reroutes_from_current_road_pixel_when_better_slot_becomes_free(self):
        self.confirm(space_4='occupied')
        initial = self.start()
        state = self.arrival.observe(self.pose())
        self.assertEqual(state['route_start_px'], [100, 150])
        self.assertTrue(state['guidance_available'])
        for _ in range(3):
            self.clock += 1.01
            self.send()
            state = self.arrival.observe(self.pose(x=180))
        self.assertEqual(state['parking_space_id'], 'space_4')
        self.assertEqual(state['route_start_px'], [180, 150])
        self.assertEqual(state['target_changes'][-1]['from'], 'space_5')
        self.assertEqual(state['waypoints'][0], [180, 150])
        self.assertEqual(state['waypoints'][-1], [320, 230])
        self.assertEqual(state['road_goal_px'], [320, 200])
        self.assertGreater(state['radius_px'], initial['radius_px'])
        self.assertEqual(state['target_reference'], 'parking_space_center')
        for a, b in zip(state['waypoints'], state['waypoints'][1:]):
            self.assertLessEqual(dist(a, b), 2 ** 0.5)
        self.assertLess(state['held_seconds'], 0.01)

    def test_occupied_current_target_blocks_or_changes_immediately(self):
        self.confirm()
        self.start(parking_space_id='space_4')
        self.arrival.observe(self.pose())
        self.clock += 0.1
        self.send(space_4='occupied')
        state = self.arrival.current()
        self.assertEqual(state['status'], 'blocked')
        self.assertFalse(state['guidance_available'])
        self.assertEqual(state['waypoints'], [])
        self.start()
        state = self.arrival.observe(self.pose())
        self.assertEqual(state['parking_space_id'], 'space_5')

    def test_stale_observation_blocks_and_one_new_frame_does_not_restore_free(self):
        self.confirm()
        self.start()
        self.arrival.observe(self.pose())
        self.clock += self.parking.TTL_SECONDS + 0.1
        blocked = self.arrival.current()
        self.assertEqual(blocked['status'], 'blocked')
        self.assertFalse(blocked['guidance_available'])
        self.send()
        self.assertEqual(self.parking.current()['states']['space_4'], 'unknown')

    def test_disabling_retarget_and_near_goal_keep_current_free_target(self):
        self.confirm(space_4='occupied')
        self.start(auto_retarget=False)
        for _ in range(3):
            self.clock += 1.01
            self.send()
            state = self.arrival.observe(self.pose())
        self.assertEqual(state['parking_space_id'], 'space_5')
        self.start(parking_space_id='space_5')
        self.assertEqual(self.arrival.observe(self.pose())['parking_space_id'], 'space_5')

    def test_near_goal_keeps_free_target_and_success_is_not_retargeted(self):
        self.confirm(space_4='occupied')
        self.start()
        self.arrival.observe(self.pose(x=260, y=210))
        for _ in range(3):
            self.clock += 1.01
            self.send()
            state = self.arrival.observe(self.pose(x=260, y=210))
        self.assertEqual(state['parking_space_id'], 'space_5')
        self.start(parking_space_id='space_5', hold_seconds=0.5)
        for _ in range(7):
            self.clock += 0.1
            state = self.arrival.observe(self.pose(x=260, y=230))
        self.assertEqual(state['status'], 'success')
        self.clock += 0.1
        self.send(space_5='occupied')
        self.assertEqual(self.arrival.current()['status'], 'success')

    def test_brief_false_free_and_missing_tracking_do_not_switch_to_an_entrance_route(self):
        self.confirm(space_4='occupied')
        self.start()
        self.arrival.observe(self.pose(x=180))
        self.clock += 0.1
        self.send()
        self.assertEqual(self.arrival.current()['parking_space_id'], 'space_5')
        self.clock += 0.1
        self.send(space_4='occupied')
        self.assertEqual(self.arrival.current()['parking_space_id'], 'space_5')
        for _ in range(3):
            self.clock += 1.01
            self.send()
        state = self.arrival.current()
        self.assertEqual(state['parking_space_id'], 'space_4')
        self.assertFalse(state['guidance_available'])
        self.assertEqual(state['waypoints'], [])
        self.clock += 0.1
        state = self.arrival.observe(self.pose(x=190))
        self.assertEqual(state['waypoints'][0], [190, 150])

    def test_identical_states_reuse_plan_without_extending_observation_lifetime(self):
        self.confirm()
        with patch('src.rover_control.live_parking.select_parking_from_map', wraps=select_parking_from_map) as select:
            self.parking.plan('03_exit')
            self.clock += 0.1
            self.send()
            self.parking.plan('03_exit')
            self.assertEqual(select.call_count, 1)
            self.clock += self.parking.TTL_SECONDS + 0.1
            with self.assertRaisesRegex(ValueError, '최신 점유'):
                self.parking.plan('03_exit')

    def test_wrong_camera_map_old_and_duplicate_updates_rejected(self):
        data = self.snapshot()
        self.parking.update(data)
        changes = [lambda d: d.update(capture_id='old'),
                   lambda d: d.update(map_image_sha256='old'),
                   lambda d: d['camera_source'].update(session_id='other'),
                   lambda d: d.update(observed_at=(self.wall_now() - timedelta(seconds=self.parking.TTL_SECONDS + 1)).isoformat()),
                   lambda d: d['spaces'].append(d['spaces'][0])]
        for change in changes:
            altered = deepcopy(self.snapshot())
            change(altered)
            with self.assertRaises(ValueError):
                self.parking.update(altered)
        with self.assertRaisesRegex(ValueError, '중복'):
            self.parking.update(data)

    def test_inference_delay_is_not_added_to_free_observation_gap(self):
        self.parking.TTL_SECONDS = 2.0
        base = self.wall_now()
        for index in range(3):
            wall = base + timedelta(seconds=1.3 * index)
            data = self.snapshot()
            data['observed_at'] = (wall - timedelta(seconds=0.8)).isoformat()
            now = 10 + index * 1.3
            state = self.parking.update(data, now=now, wall_now=wall)
        self.assertTrue(state['fresh'])
        self.assertEqual(state['states']['space_4'], 'free')
        expired = self.parking.current(now=now + 1.21, wall_now=wall + timedelta(seconds=1.21))
        self.assertFalse(expired['fresh'])
        self.assertEqual(expired['states']['space_4'], 'unknown')

    def test_default_ttl_covers_observed_camera_jitter_but_never_extends_old_frames(self):
        self.confirm()
        self.clock += 2.2
        self.assertTrue(self.parking.current()['fresh'])
        self.assertEqual(self.parking.current()['states']['space_4'], 'free')
        self.clock += 3.0
        self.assertFalse(self.parking.current()['fresh'])
        self.assertEqual(self.parking.current()['states']['space_4'], 'unknown')

    def test_waiting_without_new_frames_cannot_complete_free_confirmation(self):
        for _ in range(3):
            self.send()
            self.clock += 0.1
        self.clock += 2.0
        self.assertTrue(self.parking.current()['fresh'])
        self.assertEqual(self.parking.current()['states']['space_4'], 'unknown')
        self.send()
        self.assertEqual(self.parking.current()['states']['space_4'], 'free')

    def test_road_entrance_does_not_complete_arrival_but_central_half_area_does(self):
        self.confirm()
        state = self.start()
        self.assertEqual(state['goal_px'], [320, 230])
        self.assertAlmostEqual(state['arrival_region']['actual_area_ratio'], 0.5)
        for _ in range(12):
            self.clock += 0.1
            state = self.arrival.observe(self.pose(x=320, y=200))
            self.assertEqual(state['status'], 'running')
        for _ in range(12):
            self.clock += 0.1
            state = self.arrival.observe(self.pose(x=320, y=230))
        self.assertEqual(state['status'], 'success')

    def test_inside_arrival_circle_keeps_target_but_other_vehicle_still_blocks_success(self):
        self.confirm()
        self.start()
        state = self.arrival.observe(self.pose(x=320, y=230))
        self.assertTrue(state['arrival_committed'])
        self.clock += 0.1
        self.send(space_4='occupied')
        state = self.arrival.observe(self.pose(x=320, y=230))
        self.assertEqual(state['parking_space_id'], 'space_4')
        self.assertEqual(state['status'], 'blocked')
        self.assertFalse(state['guidance_available'])
        self.assertEqual(state['target_changes'], [])
        self.assertEqual(state['held_seconds'], 0)

    def test_controlled_rover_exclusion_evidence_is_visible_without_ignoring_other_occupied_slots(self):
        self.confirm()
        data = self.snapshot(space_5='occupied')
        data.update(occupancy_scope='other_vehicles_excluding_controlled_rover',
                    rover_exclusion={'status': 'matched', 'excluded_count': 1, 'rover_marker_id': 7})
        live = self.parking.update(data)
        self.assertEqual(live['rover_exclusion']['rover_marker_id'], 7)
        self.assertEqual(live['states']['space_5'], 'occupied')
        self.start()
        for _ in range(12):
            self.clock += 0.1
            state = self.arrival.observe(self.pose(x=320, y=230))
        self.assertEqual(state['parking_space_id'], 'space_4')
        self.assertEqual(state['status'], 'success')

    def test_arrival_area_ratio_is_validated_and_cache_keeps_each_size_separate(self):
        self.confirm()
        default = self.parking.plan('03_exit')
        smaller = self.parking.plan('03_exit', arrival_area_ratio=0.25)
        self.assertLess(smaller['arrival_region']['radius_px'], default['arrival_region']['radius_px'])
        for ratio in (True, float('nan'), 0.01, 0.9):
            with self.assertRaises(ValueError):
                self.start(arrival_area_ratio=ratio)

    def test_roi_dropout_and_occupied_all_never_offer_a_free_destination(self):
        self.confirm()
        self.start()
        self.clock += 0.1
        self.send(**{f'space_{i}': 'unknown' for i in range(1, 7)})
        self.assertEqual(self.arrival.current()['status'], 'blocked')
        self.send(**{f'space_{i}': 'occupied' for i in range(1, 7)})
        self.assertTrue(all(not case['available'] for case in self.arrival.catalog()['cases']))
