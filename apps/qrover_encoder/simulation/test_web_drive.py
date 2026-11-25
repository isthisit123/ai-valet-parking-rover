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
from node_follower import MCUFault, Rover
from receiver_source import read_receiver
from web_drive import (RouteChanged, WebFeed, braked_work, check_revision, default_repo, drive_web,
                       make_plan, route_key, validate_guidance)
from web_navigation import RoadMap

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
    def test_encoder_mode_uses_legacy_step_and_stops_on_arrival(self):
        receiver = ModuleType('receiver')
        exec(compile(read_receiver('python/main.py'), 'receiver.py', 'exec'), receiver.__dict__)
        bundle = dict(state=state(), occupancy=occupancy(), loss_count=0,
                      pose=dict(x_cm=20, y_cm=20, heading_deg=90, captured_monotonic_s=time.monotonic()))
        path = MagicMock(radius=2)
        path.arrived.return_value = False
        path.action.return_value = (1, 120)
        path.encoder_action.return_value = (1, 3)
        feed = MagicMock()
        feed.current.return_value = bundle
        calls, errors = [], []
        def bridge(name, *args, timeout):
            calls.append((name, *args))
            self.assertNotEqual(name, 'timed_step')
            if name == 'encoder_step':
                path.arrived.return_value = True
                return 1
            return 0
        with socket.create_server(('127.0.0.1', 0)) as listener:
            def serve():
                with listener.accept()[0] as connection:
                    try:
                        receiver.serve(connection, bridge)
                    except Exception as error:
                        errors.append(error)
            thread = threading.Thread(target=serve, daemon=True)
            thread.start()
            rover = Rover(*listener.getsockname())
            try:
                drive_web(feed, rover, None, bundle, path, 15, IDENTITY, encoder_drive=True,
                          log=lambda *args, **kwargs: None)
            finally:
                rover.close()
                thread.join(1)
        self.assertFalse(thread.is_alive())
        self.assertFalse(errors)
        self.assertEqual(calls[0], ('encoder_step', 1, 3))
        self.assertTrue(all(call == ('encoder_stop',) for call in calls[1:]))

    def test_first_step_reports_tof_fault_after_receiver_closes(self):
        receiver = ModuleType('receiver')
        exec(compile(read_receiver('python/main.py'), 'receiver.py', 'exec'), receiver.__dict__)
        bundle = dict(state=state(), occupancy=occupancy(), loss_count=0,
                      pose=dict(x_cm=20, y_cm=20, heading_deg=90, captured_monotonic_s=time.monotonic()))
        path = MagicMock()
        path.arrived.return_value = False
        path.action.return_value = (1, 100)
        feed = MagicMock()
        feed.current.return_value = bundle
        bridge = MagicMock(return_value=-5)
        with socket.create_server(('127.0.0.1', 0)) as listener:
            def serve():
                with listener.accept()[0] as connection:
                    receiver.serve(connection, bridge)
            thread = threading.Thread(target=serve, daemon=True)
            thread.start()
            rover = Rover(*listener.getsockname())
            try:
                with self.assertRaisesRegex(MCUFault, r'MCU fault -5 \(timed_step\).*ToF'):
                    drive_web(feed, rover, None, bundle, path, 15, IDENTITY)
            finally:
                rover.close()
                thread.join(1)
        self.assertFalse(thread.is_alive())
        self.assertEqual([c.args[0] for c in bridge.call_args_list], ['timed_step', 'encoder_stop'])

    def test_failed_brake_does_not_mask_web_stop(self):
        initial = dict(state=state(), occupancy=occupancy(), pose={}, loss_count=0)
        feed, rover = MagicMock(), MagicMock()
        feed.current.return_value = dict(initial, state=dict(state(), status='stopped'))
        rover.request.side_effect = ConnectionResetError('reset')
        with self.assertRaisesRegex(RuntimeError, '웹 경로 안내가 중단'):
            drive_web(feed, rover, None, initial, None, 15, IDENTITY)
        rover.request.assert_called_once_with('stop')

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
        bundle = {'state': dict(state(), route_revision=2), 'occupancy': occupancy(), 'pose': {}}
        with self.assertRaises(RouteChanged):
            check_revision(bundle, expected, IDENTITY, 'capture')
        for change in ({'started_at': 'new-test'}, {'capture_id': 'other'}, {'route_revision': 0}):
            bundle['state'] = dict(state(), **change)
            with self.assertRaises(RuntimeError) as raised:
                check_revision(bundle, expected, IDENTITY, 'capture')
            self.assertNotIsInstance(raised.exception, RouteChanged)

    def test_same_revision_preview_clear_waits_for_tracking(self):
        expected = route_key(state())
        bundle = dict(state=dict(state(), waypoints=[], guidance_available=False,
                                 tracking_available=False, status='waiting_for_tracking'),
                      occupancy=occupancy(), pose=None)
        check_revision(bundle, expected, IDENTITY, 'capture')

    def test_same_revision_target_change_is_still_rejected(self):
        expected = route_key(state())
        bundle = dict(state=dict(state(), parking_space_id='space_2'), occupancy=occupancy(), pose={})
        with self.assertRaisesRegex(RuntimeError, '동일하거나 오래된'):
            check_revision(bundle, expected, IDENTITY, 'capture')

    def test_occupied_target_after_loss_stops_with_occupancy_reason(self):
        expected = route_key(state())
        bundle = dict(state=dict(state(), waypoints=[], guidance_available=False, status='blocked',
                                 reason='tracking_unavailable'),
                      occupancy=dict(occupancy(), states={'space_1': 'occupied'}), pose=None)
        with self.assertRaisesRegex(RuntimeError, '점유=occupied'):
            check_revision(bundle, expected, IDENTITY, 'capture')

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
        semantic.update(nodes=[{'id': 'a', 'x': 40, 'y': 40}, {'id': 'b', 'x': 40, 'y': 96}],
                        edges=[{'from': 'a', 'to': 'b', 'path': [[40, 40], [40, 96]]}])
        path = make_plan(bundle, RoadMap(semantic, converter), 2)
        self.assertEqual(path.points, [(20, 20), (20, 48), (20, 50)])
        self.assertIsNone(path.action(dict(bundle['pose'], y_cm=48, heading_deg=-90), 15))

    def test_old_http_snapshot_and_old_pose_cannot_refresh_ttl(self):
        feed = WebFeed.__new__(WebFeed)
        feed.lock, feed.error = threading.Lock(), None
        now = time.monotonic()
        for requested, captured in ((now - 1, now), (now, now - 1)):
            feed.latest = {'requested_at': requested, 'pose': {'captured_monotonic_s': captured}, 'occupancy': occupancy()}
            self.assertIsNone(feed.current()['pose'])

    def test_time_after_http_response_counts_against_occupancy_ttl(self):
        feed = WebFeed.__new__(WebFeed)
        feed.lock, feed.error = threading.Lock(), None
        now = time.monotonic()
        feed.latest = {'requested_at': now - 0.15, 'pose': {'captured_monotonic_s': now},
                       'state': state(), 'occupancy': dict(occupancy(), age_seconds=4.9)}
        bundle = feed.current()
        with self.assertRaises(RuntimeError):
            validate_guidance(bundle['state'], bundle['occupancy'], IDENTITY, 'capture')

    def test_preflight_keeps_idle_connection_alive(self):
        rover = MagicMock()
        rover.request.return_value = 0
        braked_work(rover, lambda: time.sleep(0.12))
        self.assertGreaterEqual(rover.request.call_count, 2)
        self.assertTrue(all(call.args == ('poll',) for call in rover.request.call_args_list))

    def test_preflight_joins_pending_heartbeat_within_board_reply_timeout(self):
        rover = MagicMock()
        def reply(kind):
            time.sleep(0.7)
            return 0
        rover.request.side_effect = reply
        self.assertEqual(braked_work(rover, lambda: 'ready'), 'ready')

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
            if name == 'timed_step':
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
                bundle = feed.start()
                self.assertEqual(bundle['pose']['x_cm'], 20)
                rover = Rover('127.0.0.1', listener.getsockname()[1])
                path = MagicMock(radius=2)
                path.arrived.return_value = False
                path.action.return_value = (1, 100)
                with self.assertRaisesRegex(RuntimeError, '중단'):
                    drive_web(feed, rover, None, bundle, path, 15, IDENTITY)
            finally:
                if rover:
                    rover.close()
                feed.close()
                tcp_thread.join(1)
                listener.close()
                server.shutdown()
                http_thread.join(1)
                server.server_close()
        self.assertIn('timed_step', calls)
        self.assertEqual(calls[-1], 'encoder_stop')
        self.assertFalse(errors)


if __name__ == '__main__':
    unittest.main(verbosity=2)
