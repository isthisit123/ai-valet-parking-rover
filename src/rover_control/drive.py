import argparse
from collections import deque
import json
from math import dist
from pathlib import Path
import socket
import sys
import time

from .connection import add_connection_arguments, rover_endpoint

from .tracking import (
    COMMAND_TIMEOUT_S,
    CONTROL_PERIOD_S,
    GOAL_RADIUS_CM,
    MAX_SPEED_CM_S,
    POSE_MAX_AGE_S,
    WORLD_ORIGIN,
    Follower,
    Pose,
    load_route,
    number,
)


ACK_TIMEOUT_S = 0.08
FIRST_POSE_TIMEOUT_S = 5.0
ARRIVAL_TIMEOUT_S = 1.0
STATIONARY_SPAN_CM = 0.5
MESSAGE_LIMIT = 4096


def encode(message):
    return json.dumps(message, allow_nan=False, separators=(",", ":")).encode() + b"\n"


class PoseReceiver:
    def __init__(self, sock, map_id):
        self.sock = sock
        self.map_id = map_id
        self.latest = None

    def read(self):
        # 큐의 최신 관측을 쓰되 한 주기의 입력 처리 시간은 제한한다.
        for _ in range(64):
            try:
                payload, _ = self.sock.recvfrom(MESSAGE_LIMIT)
            except BlockingIOError:
                break
            data = json.loads(payload)
            if not isinstance(data, dict):
                raise ValueError("위치 입력은 JSON 객체여야 합니다.")
            if data["map_id"] != self.map_id:
                raise ValueError("위치 입력과 경로의 map_id가 다릅니다.")
            if data.get("type") == "lost":
                raise ValueError("로버 위치 추적이 끊겼습니다.")
            if (data.get("type"), data.get("coordinate_frame"), data.get("unit"), data.get("origin")) != (
                "pose", "world", "cm", WORLD_ORIGIN
            ):
                raise ValueError("전지 좌상단 원점의 월드 cm 위치 입력이 필요합니다.")
            captured = number(data["captured_monotonic_s"])
            if captured > time.monotonic():
                raise ValueError("촬영 시각은 노트북의 time.monotonic()으로 기록해야 합니다.")
            if self.latest is None or captured > self.latest.captured_at:
                self.latest = Pose(
                    number(data["x_cm"]), number(data["y_cm"]), number(data["heading_deg"]), captured
                )
        if self.latest is not None and time.monotonic() - self.latest.captured_at <= POSE_MAX_AGE_S:
            return self.latest
        return None


class RoverLink:
    def __init__(self, sock):
        self.sock = sock
        self.seq = 0
        self.buffer = bytearray()

    def send(self, kind, **fields):
        self.seq += 1
        self.sock.settimeout(ACK_TIMEOUT_S)
        self.sock.sendall(encode({"type": kind, "seq": self.seq, **fields}))
        return self.seq

    def command(self, kind, **fields):
        deadline = time.monotonic() + ACK_TIMEOUT_S
        seq = self.send(kind, **fields)
        while b"\n" not in self.buffer:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("로버의 명령 적용 응답 시간이 초과됐습니다.")
            self.sock.settimeout(remaining)
            chunk = self.sock.recv(MESSAGE_LIMIT - len(self.buffer))
            if not chunk:
                raise ConnectionError("로버 연결이 끊겼습니다.")
            self.buffer.extend(chunk)
            if len(self.buffer) >= MESSAGE_LIMIT and b"\n" not in self.buffer:
                raise ValueError("로버 응답 한 줄이 너무 깁니다.")
        line, _, self.buffer = self.buffer.partition(b"\n")
        status = json.loads(line)
        if not isinstance(status, dict) or status.get("type") != "status" or status.get("applied_seq") != seq:
            raise ValueError("로버 응답의 applied_seq가 보낸 명령과 다릅니다.")
        expected = "moving" if kind == "drive" else "stopped"
        if status.get("state") != expected:
            raise ValueError(f"로버가 {kind} 명령을 적용하지 못했습니다. {status.get('stop_reason', status.get('state'))}")


def wait_for_first_pose(receiver):
    deadline = time.monotonic() + FIRST_POSE_TIMEOUT_S
    while time.monotonic() < deadline:
        pose = receiver.read()
        if pose is not None:
            return pose
        time.sleep(CONTROL_PERIOD_S)
    raise TimeoutError("5초 동안 유효한 위치를 받지 못했습니다.")


def confirm_arrival(receiver, goal):
    stopped_at = time.monotonic()
    last_capture = stopped_at
    samples = deque(maxlen=3)
    while time.monotonic() - stopped_at < ARRIVAL_TIMEOUT_S:
        pose = receiver.read()
        if pose is None:
            raise ValueError("정지 후 위치 갱신이 끊겼습니다.")
        if pose.captured_at > last_capture:
            last_capture = pose.captured_at
            if dist(pose.point, goal) > GOAL_RADIUS_CM:
                raise ValueError("정지 후 로버가 목표 반경을 벗어났습니다.")
            samples.append(pose.point)
            if len(samples) == 3 and all(dist(a, b) <= STATIONARY_SPAN_CM for a in samples for b in samples):
                return dist(pose.point, goal)
        time.sleep(CONTROL_PERIOD_S / 2)
    raise TimeoutError("정지 후 연속 3개의 위치 관측으로 도착을 확인하지 못했습니다.")


def run(follower, receiver, rover):
    stop_confirmed = False
    try:
        pose = receiver.read()
        while True:
            cycle_started = time.monotonic()
            if pose is None:
                raise ValueError("로버 위치가 0.20초 안에 갱신되지 않았습니다.")
            drive = follower.drive(pose)
            if drive is None:
                rover.command("stop", reason="goal_reached")
                stop_confirmed = True
                error_cm = confirm_arrival(receiver, follower.route.points[-1])
                print(f"arrived: 최종 오차 {error_cm:.2f} cm", flush=True)
                return
            rover.command("drive", **drive)
            time.sleep(max(0.0, CONTROL_PERIOD_S - (time.monotonic() - cycle_started)))
            pose = receiver.read()
    finally:
        if not stop_confirmed:
            # 응답을 기다리다 실패한 연결에서는 로버 자체 타임아웃도 동작해야 한다.
            try:
                rover.send("stop", reason="controller_exit")
            except OSError:
                pass


def main():
    parser = argparse.ArgumentParser(description="경로 JSON과 실시간 위치로 Q-ROVER에 주행 명령을 보냅니다.")
    parser.add_argument("--route", type=Path, required=True, help="선택한 경로 JSON")
    parser.add_argument("--calibration", type=Path, help="픽셀 경로용 검증된 지도 이미지 Homography")
    parser.add_argument("--goal-cm", nargs=2, type=float, metavar=("X", "Y"), help="경로 위 월드 cm 목표점")
    add_connection_arguments(parser)
    parser.add_argument("--pose-port", type=int, default=8766, help="로컬 위치 입력 UDP 포트")
    parser.add_argument(
        "--max-speed-cm-s", type=float, default=MAX_SPEED_CM_S,
        help=f"전진 속도 상한 (최대 {MAX_SPEED_CM_S:g})",
    )
    args = parser.parse_args()
    try:
        if not 0 < number(args.max_speed_cm_s) <= MAX_SPEED_CM_S:
            raise ValueError(f"전진 속도 상한은 0 초과 {MAX_SPEED_CM_S:g} cm/s 이하로 설정하세요.")
        if not 0 < args.pose_port <= 65535:
            raise ValueError("위치 입력 포트는 1~65535로 지정하세요.")
        goal = tuple(map(number, args.goal_cm)) if args.goal_cm is not None else None
        route = load_route(args.route, args.calibration, goal, require_verified=True)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as pose_sock:
            pose_sock.bind(("127.0.0.1", args.pose_port))
            pose_sock.setblocking(False)
            receiver = PoseReceiver(pose_sock, route.map_id)
            print(f"route={args.route.name} goal_cm={route.points[-1]} pose_port={args.pose_port}", flush=True)
            with rover_endpoint(args) as endpoint:
                # 위치를 기다리는 동안 로버의 명령 타임아웃을 소모하지 않는다.
                wait_for_first_pose(receiver)
                with socket.create_connection(endpoint, timeout=2) as rover_sock:
                    rover_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    run(Follower(route, args.max_speed_cm_s), receiver, RoverLink(rover_sock))
    except (OSError, ValueError, KeyError, TypeError) as error:
        print(f"stopped: {error} (로버 명령 타임아웃 {COMMAND_TIMEOUT_S:.2f}초 필요)", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("stopped: 수동 중단", file=sys.stderr)
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
