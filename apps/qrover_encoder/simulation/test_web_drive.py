"""기존 웹 API 모양의 HTTP 입력부터 실제 엔코더 TCP 수신기까지 루프백 검증."""
from copy import deepcopy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import socket
import sys
import tempfile
import threading
import time
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))
for repository in (ROOT.parent / 'ai-valet-parking-rover', ROOT.parents[1]):
    if (repository / 'src/rover_control/pose_sender.py').is_file():
        sys.path.insert(0, str(repository))
        break
from node_follower import Rover, execute, wait_settled
from receiver_source import read_receiver
from web_drive import RouteChanged, WebFeed, braked_work, check_revision, default_repo, drive_web, make_plan, route_key, validate_guidance

IDENTITY = {'map_id': 'test', 'map_image_sha256': 'a' * 64, 'id_assignment_revision': 'v1'}


def state():
    return {**IDENTITY, 'capture_id': 'capture', 'started_at': 'session-one', 'case_id': '01_building_a',
            'status': 'running', 'guidance_available': True, 'height_correction_applied': True,
            'coordinate_frame': 'map_image', 'unit': 'pixel', 'route_revision': 1,
            'parking_space_id': 'space_1', 'waypoints': [[20, 20], [20, 50]]}


def occupancy():
    return {**IDENTITY, 'capture_id': 'capture', 'fresh': True, 'age_seconds': 0.01,
            'ttl_seconds': 5, 'states': {'space_1': 'free'}}


class WebDriveTests(unittest.TestCase):
    def test_repository_location_is_independent_of_clone_folder_name(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / 'different-clone-name'
            module = root / 'src/rover_control/pose_sender.py'
            module.parent.mkdir(parents=True)
            module.write_text('')
            script = root / 'apps/qrover_encoder/web_drive.py'
            with patch('web_drive.__file__', str(script)):
                self.assertEqual(default_repo(), root.resolve())

    def test_stopped_expired_unknown_and_wrong_map_are_blocked(self):
        validate_guidance(state(), occupancy(), IDENTITY, 'capture')
        for bad_state, bad_occupancy in ((dict(state(), status='stopped'), occupancy()),
                (state(), dict(occupancy(), age_seconds=6)),
                (state(), dict(occupancy(), fresh=False)),
                (state(), dict(occupancy(), states={'space_1': 'unknown'})),
                (dict(state(), map_image_sha256='b' * 64), occupancy()),
                (dict(state(), height_correction_applied=False), occupancy())):
            with self.subTest(state=bad_state, occupancy=bad_occupancy), self.assertRaises((RuntimeError, ValueError)):
                validate_guidance(bad_state, bad_occupancy, IDENTITY, 'capture')

    def test_route_change_requests_braking_but_new_session_aborts(self):
        expected = route_key(state())
        bundle = {'state': dict(state(), route_revision=2), 'occupancy': occupancy()}
        with self.assertRaises(RouteChanged):
            check_revision(bundle, expected, IDENTITY, 'capture')
        for change in ({'started_at': 'new-test'}, {'capture_id': 'other'}, {'route_revision': 0}):
            bundle['state'] = dict(state(), **change)
            with self.assertRaises(RuntimeError) as raised:
                check_revision(bundle, expected, IDENTITY, 'capture')
            self.assertNotIsInstance(raised.exception, RouteChanged)

    def test_ground_route_uses_homography_without_marker_height_shift(self):
        import numpy as np
        converter = SimpleNamespace(identity=IDENTITY, report_bytes=b'{"capture_id":"capture"}',
            world_to_image=np.diag([2., 2., 1.]), paper_size=np.asarray([100., 100.]),
            image_size=np.asarray([200., 200.]))
        guidance = dict(state(), waypoints=[[40, 40], [40, 100]])
        semantic = {'regions': [{'id': 'space_1', 'type': 'normal',
                                'polygon_px': [[20, 80], [60, 80], [60, 120], [20, 120]]}]}
        bundle = {'state': guidance, 'occupancy': occupancy(),
                  'pose': {'x_cm': 20, 'y_cm': 20, 'heading_deg': 90}}
        route, geometry = make_plan(bundle, converter, semantic)
        self.assertEqual(route['waypoints'], [[20, 20], [20, 50]])
        self.assertEqual(route['final_heading_deg'], 90)

    def test_old_http_snapshot_and_old_pose_cannot_refresh_ttl(self):
        feed = WebFeed.__new__(WebFeed)
        feed.lock, feed.error = threading.Lock(), None
        now = time.monotonic()
        for requested, captured in ((now - 1, now), (now, now - 1)):
            feed.latest = {'requested_at': requested, 'pose': {'captured_monotonic_s': captured}}
            with self.assertRaises(TimeoutError):
                feed.current()

    def test_time_after_http_response_counts_against_occupancy_ttl(self):
        feed = WebFeed.__new__(WebFeed)
        feed.lock, feed.error = threading.Lock(), None
        now = time.monotonic()
        feed.latest = {'requested_at': now - 0.15, 'pose': {'captured_monotonic_s': now},
                       'state': state(), 'occupancy': dict(occupancy(), age_seconds=4.9)}
        bundle = feed.current()
        with self.assertRaises(RuntimeError):
            validate_guidance(bundle['state'], bundle['occupancy'], IDENTITY, 'capture')

    def test_route_replacement_brakes_before_rebuilding(self):
        rover = MagicMock()
        pose = {'x_cm': 20, 'y_cm': 20, 'heading_deg': 90, 'position_samples': [(20, 20)] * 3}
        feed = MagicMock()
        feed.current.return_value = {'pose': pose, 'state': state(), 'occupancy': occupancy()}
        feed.update.return_value = dict(pose, x_cm=20, y_cm=50)
        route = {**IDENTITY, 'coordinate_frame': 'world', 'unit': 'cm', 'origin': 'paper_top_left',
                 'axis_directions': {'x': 'right', 'y': 'down'}, 'calibration_verified': True,
                 'waypoints': [[20, 20], [20, 50]], 'final_heading_deg': 90}
        events = []
        rover.request.side_effect = lambda kind: (events.append(kind), 0)[1]
        def build(*args):
            events.append('build')
            return route, MagicMock()
        with patch('web_drive.wait_settled', return_value=pose), \
                patch('web_drive.make_plan', side_effect=build), \
                patch('web_drive.follow', side_effect=[RouteChanged('replan'), None]):
            drive_web(feed, SimpleNamespace(identity=IDENTITY), {}, rover, 12, log=lambda *a, **k: None)
        self.assertEqual([event for event in events if event != 'poll'], ['stop', 'build', 'stop', 'build', 'stop'])

    def test_preflight_keeps_idle_connection_alive(self):
        rover = MagicMock()
        rover.request.return_value = 0
        braked_work(rover, lambda: time.sleep(0.12))
        self.assertGreaterEqual(rover.request.call_count, 2)
        self.assertTrue(all(call.args == ('poll',) for call in rover.request.call_args_list))

    def test_mcu_fault_blocks_listening_before_any_client(self):
        receiver = ModuleType('receiver')
        exec(compile(read_receiver('python/main.py'), 'receiver.py', 'exec'), receiver.__dict__)
        arduino, utils = ModuleType('arduino'), ModuleType('arduino.app_utils')
        utils.Bridge = SimpleNamespace(call=lambda *a, **k: -4)
        with patch.dict(sys.modules, {'arduino': arduino, 'arduino.app_utils': utils}), \
                patch.object(receiver.socket, 'create_server') as listen:
            with self.assertRaisesRegex(RuntimeError, 'stop not ready'):
                receiver.main()
            listen.assert_not_called()

    def test_http_web_stop_reaches_encoder_receiver(self):
        receiver = ModuleType('receiver')
        exec(compile(read_receiver('python/main.py'), 'receiver.py', 'exec'), receiver.__dict__)
        web_state, calls, errors = state(), [], []
        class Handler(BaseHTTPRequestHandler):
            def do_GET(handler):
                if handler.path == '/api/rover-pose':
                    data = {'arrival_test': deepcopy(web_state), 'received_monotonic_s': time.monotonic(),
                            'x_cm': 20, 'y_cm': 20, 'heading_deg': 90}
                else:
                    data = occupancy()
                body = json.dumps(data).encode()
                handler.send_response(200)
                handler.send_header('Content-Length', str(len(body)))
                handler.end_headers()
                handler.wfile.write(body)
            def log_message(self, *args):
                pass
        class Converter:
            def require_current(self):
                pass
            def convert(self, observation):
                return {**observation, 'captured_monotonic_s': observation['received_monotonic_s']}
        def bridge(name, *args, timeout):
            calls.append(name)
            if name == 'encoder_step':
                web_state['status'] = 'stopped'
                web_state['guidance_available'] = False
                return 1
            return 0
        server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        http_thread = threading.Thread(target=server.serve_forever, daemon=True)
        http_thread.start()
        listener = socket.create_server(('127.0.0.1', 0))
        def receive():
            with listener.accept()[0] as connection:
                try:
                    receiver.serve(connection, bridge)
                except Exception as error:
                    errors.append(error)
        tcp_thread = threading.Thread(target=receive, daemon=True)
        tcp_thread.start()
        with tempfile.TemporaryDirectory() as temporary:
            settings = Path(temporary) / 'settings.json'
            settings.write_text('{}')
            feed = WebFeed(f'http://127.0.0.1:{server.server_port}', Converter(), settings)
            rover = None
            try:
                feed.start()
                pose = wait_settled(feed, None, initial=True)
                self.assertEqual(pose['x_cm'], 20)
                rover = Rover('127.0.0.1', listener.getsockname()[1])
                expected = route_key(web_state)
                def check(pose):
                    check_revision(feed.current(), expected, IDENTITY, 'capture')
                with self.assertRaises(RuntimeError):
                    execute(rover, feed, (1, 1), check=check)
            finally:
                if rover:
                    rover.close()
                feed.close()
                tcp_thread.join(1)
                listener.close()
                server.shutdown()
                http_thread.join(1)
                server.server_close()
        self.assertIn('encoder_step', calls)
        self.assertEqual(calls[-1], 'encoder_stop')
        self.assertFalse(errors)


if __name__ == '__main__':
    unittest.main(verbosity=2)
