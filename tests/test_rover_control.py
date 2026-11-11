from hashlib import sha256
from contextlib import redirect_stdout
import importlib.util
import io
import json
from math import cos, dist, radians, sin
from pathlib import Path
import socket
from tempfile import TemporaryDirectory
from threading import Thread
import time
import unittest
from unittest.mock import patch

from src.rover_control.drive import PoseReceiver, RoverLink, encode, run
from src.rover_control.route_preview import main as preview_route
from src.rover_control.tracking import AXES, WORLD_ORIGIN, Follower, Pose, Route, load_route


REAL_ROUTE = Path("data/maps/parking_map_real/routes/01_building_a.json")


class RouteTest(unittest.TestCase):
    def test_world_route_can_stop_before_last_waypoint(self):
        with TemporaryDirectory() as folder:
            path = Path(folder) / "test_route.json"
            path.write_text(json.dumps({
                "map_id": "test", "unit": "cm", "coordinate_frame": "world",
                "origin": WORLD_ORIGIN, "axis_directions": AXES,
                "waypoints": [[0, 0], [5, 0], [10, 0], [10, 10]],
            }), encoding="utf-8")
            route = load_route(path, goal_cm=(10.2, 6))
            self.assertEqual(route.points, [(0, 0), (5, 0), (10, 0), (10, 6)])
            with self.assertRaisesRegex(ValueError, "0.5 cm"):
                load_route(path, goal_cm=(6, 6))

    def test_pixel_route_requires_calibration(self):
        with self.assertRaisesRegex(ValueError, "--calibration"):
            load_route(REAL_ROUTE)

    @unittest.skipUnless(importlib.util.find_spec("cv2"), "OpenCV가 필요합니다.")
    def test_pixel_route_is_converted_using_the_matching_map_image(self):
        import cv2
        import numpy as np
        from src.map_processing.map_config import get_map_config

        image = get_map_config("parking_map_real").image_path.read_bytes()
        height, width = cv2.imdecode(np.frombuffer(image, np.uint8), cv2.IMREAD_COLOR).shape[:2]
        with TemporaryDirectory() as folder:
            calibration = Path(folder) / "calibration.json"
            report = {
                "map_id": "parking_map_real", "validation": {"passed": True},
                "source_frame": {"size_px": [width, height], "sha256": sha256(image).hexdigest()},
                "target_frame": {"coordinate_frame": "world", "unit": "cm",
                                 "origin": WORLD_ORIGIN, "axis_directions": AXES},
                "paper_size_cm": [width / 10, height / 10],
                "image_to_world": [[0.1, 0, 0], [0, 0.1, 0], [0, 0, 1]],
            }
            calibration.write_text(json.dumps(report), encoding="utf-8")
            route = load_route(REAL_ROUTE, calibration)
            self.assertAlmostEqual(route.points[0][0], 24.1)
            self.assertAlmostEqual(route.points[0][1], 41.2)
            output = Path(folder) / "01_building_a_cm.json"
            original = REAL_ROUTE.read_bytes()
            with patch("sys.argv", [
                "route_preview", "--route", str(REAL_ROUTE), "--calibration", str(calibration),
                "--output", str(output),
            ]), redirect_stdout(io.StringIO()) as stdout:
                preview_route()
            saved = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual(saved["unit"], "cm")
            self.assertEqual(saved["parking_space_id"], json.loads(original)["parking_space_id"])
            self.assertEqual(saved["total_distance"], route.lengths[-1])
            self.assertEqual(load_route(output).points, route.points)
            self.assertEqual(REAL_ROUTE.read_bytes(), original)
            self.assertIn("cm 이동", stdout.getvalue())
            report["source_frame"]["sha256"] = "different"
            calibration.write_text(json.dumps(report), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "이미지와 일치"):
                load_route(REAL_ROUTE, calibration)

    def test_right_turn_has_positive_rotation(self):
        follower = Follower(Route("test", [(0, 0), (0, 10)]), 8)
        command = follower.drive(Pose(0, 0, 0, 0))
        self.assertEqual(command["linear_cm_s"], 0)
        self.assertGreater(command["angular_deg_s"], 0)

    def test_follows_a_right_angle_to_the_last_waypoint(self):
        route = Route("test", [(0, 0), (20, 0), (20, 20)])
        follower = Follower(route, 8)
        pose = Pose(0, 0, 0, 0)
        for _ in range(150):
            command = follower.drive(pose)
            if command is None:
                break
            heading = pose.heading_deg + command["angular_deg_s"] * 0.1
            step = command["linear_cm_s"] * 0.1
            pose = Pose(pose.x + cos(radians(heading)) * step,
                        pose.y + sin(radians(heading)) * step, heading, pose.captured_at + 0.1)
        else:
            self.fail("모의 로버가 최종 waypoint에 도착하지 않았습니다.")
        self.assertLessEqual(dist(pose.point, route.points[-1]), 2)


class DriveIntegrationTest(unittest.TestCase):
    def exercise(self, *, stream_poses, confirm_commands=True):
        commands = []
        errors = []
        with socket.create_server(("127.0.0.1", 0)) as server, socket.socket(
            socket.AF_INET, socket.SOCK_DGRAM
        ) as receiver, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            receiver.bind(("127.0.0.1", 0))
            receiver.setblocking(False)
            pose_address = receiver.getsockname()

            def send_pose(x):
                sender.sendto(encode({
                    "type": "pose", "map_id": "test", "coordinate_frame": "world", "unit": "cm",
                    "origin": WORLD_ORIGIN, "x_cm": x, "y_cm": 0, "heading_deg": 0,
                    "captured_monotonic_s": time.monotonic(),
                }), pose_address)

            def fake_rover():
                try:
                    connection, _ = server.accept()
                    with connection, connection.makefile("rb") as messages:
                        connection.settimeout(3)
                        send_pose(0)
                        for line in messages:
                            command = json.loads(line)
                            commands.append(command)
                            is_stop = command["type"] == "stop"
                            if confirm_commands:
                                connection.sendall(encode({
                                    "type": "status", "applied_seq": command["seq"],
                                    "state": "stopped" if is_stop else "moving",
                                }))
                            if stream_poses and is_stop:
                                for _ in range(3):
                                    time.sleep(0.07)
                                    send_pose(6)
                            if is_stop:
                                break
                            if stream_poses:
                                send_pose(min(6, len(commands) * 1.5))
                except (OSError, ValueError) as error:
                    errors.append(error)

            worker = Thread(target=fake_rover, daemon=True)
            worker.start()
            try:
                with socket.create_connection(server.getsockname(), timeout=2) as rover:
                    follower = Follower(Route("test", [(0, 0), (6, 0)]), 8)
                    action = lambda: run(follower, PoseReceiver(receiver, "test"), RoverLink(rover))
                    if not confirm_commands:
                        with self.assertRaises(TimeoutError):
                            action()
                    elif not stream_poses:
                        with self.assertRaisesRegex(ValueError, "갱신되지"):
                            action()
                    else:
                        action()
            finally:
                worker.join(timeout=4)
            self.assertFalse(worker.is_alive())
            self.assertEqual(errors, [])
        self.assertTrue(all(command["type"] == "drive" for command in commands[:-1]))
        return commands

    def test_stream_sends_drive_commands_and_stops_at_goal(self):
        commands = self.exercise(stream_poses=True)
        self.assertGreaterEqual(len(commands), 3)
        self.assertEqual([item["seq"] for item in commands], list(range(1, len(commands) + 1)))
        self.assertEqual(commands[-1]["reason"], "goal_reached")

    def test_stale_pose_stops_motion(self):
        commands = self.exercise(stream_poses=False)
        self.assertEqual(commands[-1]["type"], "stop")
        self.assertEqual(commands[-1]["reason"], "controller_exit")

    def test_missing_ack_does_not_queue_more_drive_commands(self):
        commands = self.exercise(stream_poses=False, confirm_commands=False)
        self.assertEqual([command["type"] for command in commands], ["drive", "stop"])
