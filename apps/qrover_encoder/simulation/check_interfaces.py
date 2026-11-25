"""Loopback tests for the packaged laptop client and UNO Q Python receiver."""
import json
from pathlib import Path
import socket
import sys
import threading
import time
from types import ModuleType
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from node_follower import Poses, Rover, execute, load_route, plan, settled_pose, wait_settled
from receiver_source import read_receiver

receiver = ModuleType("receiver")
exec(compile(read_receiver("python/main.py"), "python/main.py", "exec"), receiver.__dict__)

ROUTE = {"map_id": "test", "unit": "cm", "nodes": [[1, 2]],
         "map_image_sha256": "test-image", "id_assignment_revision": "test-revision"}
POSE = {"type": "pose", "map_id": "test", "unit": "cm", "origin": "paper_top_left",
        "coordinate_frame": "world", "map_image_sha256": "test-image",
        "id_assignment_revision": "test-revision", "height_correction_applied": True,
        "x_cm": 0, "y_cm": 0, "heading_deg": 0}


class MapAndPoseTests(unittest.TestCase):
    def test_legacy_and_team_route(self):
        self.assertEqual(load_route(ROUTE), ([(1, 2)], None))
        route = dict(ROUTE, waypoints=ROUTE["nodes"], calibration_verified=True,
                     coordinate_frame="world", origin="paper_top_left", final_heading_deg=90)
        del route["nodes"]
        self.assertEqual(load_route(route), ([(1, 2)], 90))
        for override in ({"calibration_verified": False}, {"coordinate_frame": "world_estimate"},
                         {"unit": "pixel"}, {"origin": "wrong"}, {"final_heading_deg": float("nan")}):
            with self.subTest(override=override), self.assertRaises(ValueError):
                load_route(dict(route, **override))

    def test_wrap_gate_directions_and_final_heading(self):
        stable = settled_pose([dict(POSE, heading_deg=a) for a in (359.5, 0.3, 0.1)])
        self.assertEqual(plan(stable, (20, 0))[0][0], 1)
        self.assertEqual(plan(stable, (0, 20))[0][0], 2)
        self.assertEqual(plan(stable, (0, -20))[0][0], 3)
        self.assertEqual(plan(stable, (0, 0), final_heading_deg=90)[0][0], 2)
        self.assertIsNone(plan(stable, (0, 0), final_heading_deg=0)[0])
        self.assertIsNone(settled_pose([dict(POSE, heading_deg=a) for a in (0, 4, 6)]))
        spread = settled_pose([dict(POSE, heading_deg=a) for a in (3, 4, 5)])
        self.assertNotEqual(plan(spread, (20, 0))[0][0], 1)

    def test_udp_freshness_identity_and_loss(self):
        for override, raises in (({}, None), ({"type": "lost"}, RuntimeError),
                                 ({"map_image_sha256": "other"}, ValueError),
                                 ({"id_assignment_revision": "other"}, ValueError),
                                 ({"origin": "other"}, ValueError),
                                 ({"height_correction_applied": False}, ValueError),
                                 ({"captured_monotonic_s": time.monotonic()-1}, ValueError)):
            with self.subTest(override=override), socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
                poses = Poses(0, ROUTE)
                self.addCleanup(poses.close)
                msg = dict(POSE, captured_monotonic_s=time.monotonic())
                sender.sendto(json.dumps(dict(msg, **override)).encode(), poses.sock.getsockname())
                if raises:
                    with self.assertRaises(raises):
                        poses.update()
                else:
                    self.assertEqual(poses.update()["x_cm"], 0)

    def test_wait_ignores_frames_before_settling(self):
        class FakeTime:
            now = 0.0
            def monotonic(self):
                return self.now
            def sleep(self, seconds):
                self.now += seconds
        clock = FakeTime()
        class FakePoses:
            def update(self, required=True):
                stamp = int(clock.now / 0.1) * 0.1
                return dict(POSE, captured_monotonic_s=stamp,
                            heading_deg=90 if stamp <= 0.30 else 0)
        class BrakedRover:
            def request(self, kind):
                return 0
        with patch("node_follower.time", clock):
            pose = wait_settled(FakePoses(), BrakedRover())
        self.assertGreaterEqual(pose["captured_monotonic_s"], 0.5)
        self.assertEqual(pose["heading_samples"], [0, 0, 0])


class ReceiverTests(unittest.TestCase):
    def wire(self, fault=0, delay=0, stop_delay=0, bridge_failure=False):
        listener = socket.create_server(("127.0.0.1", 0))
        self.addCleanup(listener.close)
        calls, errors = [], []
        def fake_bridge(name, *args, timeout):
            calls.append((name, *args))
            if bridge_failure and name in ('encoder_poll', 'encoder_stop'):
                raise TimeoutError('simulated Bridge timeout')
            if name == 'timed_step' and delay:
                time.sleep(delay)
            if name == 'encoder_stop' and stop_delay:
                time.sleep(stop_delay)
            return {"encoder_step": 1, "timed_step": 1, "encoder_poll": fault, "encoder_stop": 0}[name]
        def run():
            with listener.accept()[0] as sock:
                try:
                    receiver.serve(sock, fake_bridge)
                except (OSError, ValueError, RuntimeError) as error:
                    errors.append(error)
        thread = threading.Thread(target=run, daemon=True)
        thread.start()
        client = Rover("127.0.0.1", listener.getsockname()[1])
        self.addCleanup(client.sock.close)
        return client, thread, calls, errors

    def test_step_poll_and_disconnect(self):
        client, thread, calls, errors = self.wire()
        self.assertEqual(client.request("step", mode=2, pulses=1), 1)
        self.assertEqual(client.request("poll"), 0)
        client.sock.close()
        thread.join(1)
        self.assertEqual(calls, [("encoder_step", 2, 1), ("encoder_poll",), ("encoder_stop",)])
        self.assertFalse(errors)

    def test_timed_step_forwards_duration_and_disconnect_brakes(self):
        client, thread, calls, errors = self.wire()
        self.assertEqual(client.request("timed_step", mode=2, duration_ms=180), 1)
        self.assertEqual(client.request("stop"), 0)
        client.sock.close()
        thread.join(1)
        self.assertEqual(calls, [("timed_step", 2, 180), ("encoder_stop",), ("encoder_stop",)])
        self.assertFalse(errors)

    def test_late_reply_closes_connection_without_resending_stop(self):
        client, thread, calls, errors = self.wire(delay=1.70)
        with self.assertRaisesRegex(TimeoutError, '1500ms'):
            client.request('timed_step', mode=2, duration_ms=120)
        self.assertEqual(client.sock.fileno(), -1)
        with self.assertRaisesRegex(ConnectionError, '연결이 종료'):
            client.request('stop')
        client.close()
        thread.join(1)
        self.assertFalse(thread.is_alive())
        self.assertEqual(calls, [('timed_step', 2, 120), ('encoder_stop',)])

    def test_delayed_stop_and_motion_replies_are_accepted(self):
        client, thread, calls, errors = self.wire(delay=0.70, stop_delay=0.30)
        self.assertEqual(client.request('stop'), 0)
        self.assertEqual(client.request('timed_step', mode=2, duration_ms=120), 1)
        client.sock.close()
        thread.join(1)
        self.assertFalse(thread.is_alive())
        self.assertFalse(errors)
        self.assertEqual(calls, [('encoder_stop',), ('timed_step', 2, 120), ('encoder_stop',)])

    def test_bridge_timeout_is_reported_without_masking_it_with_stop_failure(self):
        client, thread, calls, errors = self.wire(bridge_failure=True)
        with self.assertRaisesRegex(RuntimeError, '보드 내부 통신 중단: MCU Bridge encoder_poll'):
            client.request('poll')
        client.close()
        thread.join(1)
        self.assertFalse(thread.is_alive())
        self.assertEqual(calls, [('encoder_poll',), ('encoder_stop',)])
        self.assertIsInstance(errors[0], receiver.MCUBridgeUnavailable)

    def test_connect_timeout_is_distinguished_from_reply_timeout(self):
        with patch('node_follower.socket.create_connection', side_effect=TimeoutError) as connect:
            with self.assertRaisesRegex(TimeoutError, 'TCP 연결이 2초'):
                Rover('test-board')
        connect.assert_called_once_with(('test-board', 8765), timeout=2.0)

    def test_bad_timed_duration_brakes_without_starting_motor(self):
        for duration in (0, 39, 201, True, 100.5, "100"):
            with self.subTest(duration=duration):
                client, thread, calls, errors = self.wire()
                with self.assertRaises((ConnectionError, OSError)):
                    client.request("timed_step", mode=1, duration_ms=duration)
                thread.join(1)
                self.assertEqual(calls, [("encoder_stop",)])
                self.assertIsInstance(errors[0], ValueError)

    def test_timed_job_connection_timeout_brakes(self):
        client, thread, calls, errors = self.wire()
        client.request("timed_step", mode=1, duration_ms=180)
        thread.join(receiver.TIMEOUT + 0.5)
        self.assertFalse(thread.is_alive())
        self.assertEqual(calls[-1], ("encoder_stop",))
        self.assertIsInstance(errors[0], TimeoutError)

    def test_heartbeat_times_out_and_brakes(self):
        client, thread, calls, errors = self.wire()
        client.request("step", mode=1, pulses=5)
        thread.join(receiver.TIMEOUT + 0.5)
        self.assertFalse(thread.is_alive())
        self.assertEqual(calls[-1], ("encoder_stop",))
        self.assertIsInstance(errors[0], TimeoutError)

    def test_tof_fault_prevents_more_jobs(self):
        for fault in (-4, -5):
            with self.subTest(fault=fault):
                client, thread, calls, errors = self.wire(fault)
                client.request("step", mode=1, pulses=5)
                with self.assertRaisesRegex(RuntimeError, str(fault)):
                    client.request("poll")
                thread.join(1)
                self.assertEqual(calls, [("encoder_step", 1, 5), ("encoder_poll",), ("encoder_stop",)])
                self.assertFalse(errors)

    def test_lost_during_job_sends_stop(self):
        client, thread, calls, errors = self.wire()
        class LostPoses:
            count = 0
            def update(self):
                self.count += 1
                if self.count > 1:
                    raise RuntimeError("ArUco lost")
        with self.assertRaisesRegex(RuntimeError, "ArUco lost"):
            try:
                execute(client, LostPoses(), (1, 5))
            finally:
                client.close()
        thread.join(1)
        self.assertEqual(calls, [("encoder_step", 1, 5), ("encoder_stop",), ("encoder_stop",)])
        self.assertFalse(errors)


if __name__ == "__main__":
    unittest.main(verbosity=2)
