from contextlib import contextmanager
import socket
from threading import Thread
import unittest

from apps.q_rover_receiver.python.main import serve_client
from src.rover_control.drive import RoverLink
from src.rover_control.probe import probe


class RoverReceiverTest(unittest.TestCase):
    @contextmanager
    def connected(self, drive_result=2, mode=0):
        calls = []
        errors = []

        def bridge_call(method, *args, timeout):
            calls.append((method, args))
            return {"rover_mode": mode, "rover_drive": drive_result, "rover_stop": 0, "rover_bench": 1}[method]

        def serve(connection):
            try:
                with connection:
                    serve_client(connection, bridge_call)
            except (OSError, ValueError) as error:
                errors.append(error)

        client, server = socket.socketpair()
        worker = Thread(target=serve, args=(server,), daemon=True)
        worker.start()
        try:
            with client:
                yield RoverLink(client), calls, errors, worker
        finally:
            worker.join(timeout=2)
            self.assertFalse(worker.is_alive())

    def test_probe_reaches_mcu_in_dry_run(self):
        with self.connected() as (client, calls, errors, _):
            probe(client)
        self.assertEqual(calls, [
            ("rover_mode", ()),
            *[("rover_drive", (2000, 0))] * 5,
            ("rover_stop", ()),
        ])
        self.assertEqual(errors, [])

    def test_existing_controller_can_drive_and_stop(self):
        with self.connected(drive_result=1, mode=1) as (client, calls, errors, _):
            client.command("drive", linear_cm_s=2.5, angular_deg_s=-15.25)
            client.command("stop", reason="goal_reached")
        self.assertEqual(calls, [("rover_drive", (2500, -15250)), ("rover_stop", ())])
        self.assertEqual(errors, [])

    def test_bench_probe_sends_one_pulse_and_stop(self):
        with self.connected(mode=2) as (client, calls, errors, _):
            probe(client, bench_drive=True)
        self.assertEqual(calls, [("rover_mode", ()), ("rover_bench", ()), ("rover_stop", ())])
        self.assertEqual(errors, [])

    def test_normal_probe_refuses_bench_mode(self):
        with self.connected(mode=2) as (client, calls, errors, _):
            with self.assertRaisesRegex(ValueError, "dry_run"):
                probe(client)
        self.assertEqual(calls, [("rover_mode", ())])
        self.assertEqual(errors, [])

    def test_idle_connection_stops_active_drive(self):
        with self.connected(drive_result=1, mode=1) as (client, calls, errors, worker):
            client.command("drive", linear_cm_s=2, angular_deg_s=0)
            worker.join(timeout=1)
            self.assertFalse(worker.is_alive())
        self.assertEqual(calls[-1], ("rover_stop", ()))
        self.assertIsInstance(errors[0], TimeoutError)

    def test_disconnect_stops_active_drive(self):
        with self.connected(drive_result=1, mode=1) as (client, calls, errors, _):
            client.command("drive", linear_cm_s=2, angular_deg_s=0)
        self.assertEqual(calls[-1], ("rover_stop", ()))
        self.assertEqual(errors, [])

    def test_mcu_obstacle_reason_is_reported_to_laptop(self):
        with self.connected(drive_result=4, mode=1) as (client, calls, errors, _):
            with self.assertRaisesRegex(ValueError, "tof_obstacle"):
                client.command("drive", linear_cm_s=2, angular_deg_s=0)
        self.assertEqual(calls, [("rover_drive", (2000, 0))])
        self.assertEqual(errors, [])
