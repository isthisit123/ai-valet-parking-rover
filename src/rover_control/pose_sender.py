"""기존 웹캠의 ArUco 관측을 검증된 월드 cm 위치로 바꿔 주행기에 전달한다."""

import argparse
from datetime import datetime, timezone
import json
from math import atan2, cos, degrees, radians, sin
from pathlib import Path
import socket
import sys
import time
from urllib.parse import urlsplit
from urllib.request import urlopen

import cv2
import numpy as np

from src.map_processing.map_config import MapConfig
from src.map_processing.map_identity import AXES, WORLD_ORIGIN, configured_map_identity, require_map_identity
from src.map_processing.planar_calibration import load_world_to_image, project_points
from .drive import encode
from .tracking import CONTROL_PERIOD_S, POSE_MAX_AGE_S, number


class CapturePose:
    def __init__(self, folder, *, marker_id=7, center_offset_cm=(0.0, 0.0)):
        self.folder = Path(folder)
        self.marker_id = marker_id
        self.offset = tuple(number(value) for value in center_offset_cm)
        if len(self.offset) != 2:
            raise ValueError("중심 오프셋 [전방, 오른쪽] cm가 필요합니다.")

        self.report_bytes = (self.folder / "report.json").read_bytes()
        report = json.loads(self.report_bytes)
        if report.get("status") != "map_generated" or report.get("published_as_latest") is not True:
            raise ValueError("현재 웹캠으로 지도와 cm 보정을 먼저 생성하세요.")
        self.image_size = np.asarray([number(value) for value in report["image_size_px"]])
        if self.image_size.shape != (2,) or (self.image_size <= 0).any():
            raise ValueError("기준 지도 PNG의 유효한 크기가 필요합니다.")
        self.camera_source = report.get("camera_source") or {}
        if self.camera_source.get("kind") != "webcam" or not self.camera_source.get("session_id"):
            raise ValueError("보정할 때 사용한 웹캠 세션 정보가 필요합니다. 웹에서 지도를 다시 생성하세요.")
        self.native_size = np.asarray([number(self.camera_source[key]) for key in ("width", "height")])
        if ((self.native_size <= 0).any()
                or abs(self.native_size[0] / self.native_size[1] - self.image_size[0] / self.image_size[1]) >= 0.005):
            raise ValueError("기준 지도와 웹캠의 종횡비가 다릅니다.")
        self.config = MapConfig(report["map_id"], self.folder / "source.png", self.folder / "labels.json", self.folder)
        self.identity = configured_map_identity(self.config)
        require_map_identity(self.identity, report, "캡처 보고서")
        self.calibration_path = self.folder / "world/map_calibration.json"
        self.calibration_bytes = self.calibration_path.read_bytes()
        inverse, self.paper_size = load_world_to_image(
            self.calibration_path, map_id=self.identity["map_id"], image_size=self.image_size.tolist(),
            image_sha256=self.identity["map_image_sha256"],
        )
        # 최초 보정식을 보관하고, 원본 웹캠과 지도 PNG의 크기 차이만 반영한다.
        self.matrix = np.linalg.inv(inverse) @ np.diag([*(self.image_size / self.native_size), 1.0])
        self.last_capture = -1.0
        self.last_frame = None

    def require_current(self):
        if ((self.folder / "report.json").read_bytes() != self.report_bytes
                or self.calibration_path.read_bytes() != self.calibration_bytes):
            raise ValueError("지도나 보정이 갱신됐습니다. 주행을 중단하고 새 경로와 위치 전송기를 다시 실행하세요.")
        require_map_identity(self.identity, configured_map_identity(self.config), "현재 지도")

    def convert(self, observation, *, now=None):
        now = time.monotonic() if now is None else now
        self.require_current()
        if observation.get("tracking_status") != "detected":
            raise ValueError("ArUco 마커가 미검출·중복·만료 상태입니다.")
        native_size = np.asarray([number(observation[key]) for key in ("width", "height")])
        if (observation.get("camera_source") != self.camera_source
                or native_size.tolist() != self.native_size.tolist()):
            raise ValueError("보정 당시의 웹캠 세션·해상도와 다릅니다. 현재 카메라로 지도를 다시 생성하세요.")
        if (observation.get("coordinate_frame"), observation.get("unit"), observation.get("origin"),
                observation.get("axis_directions")) != ("camera_image", "pixel", "top_left", AXES):
            raise ValueError("원본 카메라 픽셀 좌표의 ArUco 관측이 필요합니다.")
        captured = number(observation["received_monotonic_s"])
        if not 0 <= now - captured <= POSE_MAX_AGE_S:
            raise ValueError("ArUco 위치가 0.20초 안에 갱신되지 않았습니다.")
        if observation.get("client_captured_at"):
            client_time = datetime.fromisoformat(observation["client_captured_at"].replace("Z", "+00:00"))
            age = (datetime.now(timezone.utc) - client_time).total_seconds()
            if not -0.05 <= age <= POSE_MAX_AGE_S:
                raise ValueError("웹캠 프레임의 촬영 시각이 오래됐습니다.")
        rover = observation.get("rover") or {}
        if rover.get("id") != self.marker_id:
            raise ValueError("추적한 마커 ID가 설정과 다릅니다.")
        frame_id = observation.get("frame_id")
        frame = ((observation.get("camera_source") or {}).get("session_id"), frame_id) if frame_id is not None else None
        if captured <= self.last_capture or (frame is not None and frame == self.last_frame):
            return None

        corners = np.asarray(rover["corners_px"], dtype=np.float32)
        if (corners.shape != (4, 2) or not np.isfinite(corners).all() or not cv2.isContourConvex(corners)
                or abs(cv2.contourArea(corners)) < 1 or (corners < 0).any() or (corners >= native_size).any()):
            raise ValueError("화면 안의 유효한 마커 모서리 네 점이 필요합니다.")
        square = np.float32([[0, 0], [1, 0], [1, 1], [0, 1]])
        marker_to_pixel = cv2.getPerspectiveTransform(square, corners)
        angle = radians(number(observation.get("front_offset_deg", 0.0)))
        axis = np.asarray([[0.5, 0.5], [0.5 + 0.25 * cos(angle), 0.5 + 0.25 * sin(angle)]])
        # 중심과 전방의 두 점을 모두 월드로 옮겨 원근에 따른 방향각 변화도 반영한다.
        transform = self.matrix @ marker_to_pixel
        denominators = axis @ transform[2, :2] + transform[2, 2]
        if (np.abs(denominators) < 1e-9).any() or np.prod(denominators) <= 0:
            raise ValueError("마커 위치·방향을 월드 좌표로 투영할 수 없습니다.")
        center, forward = project_points(axis, transform)
        direction = forward - center
        length = np.linalg.norm(direction)
        if not np.isfinite(center).all() or not np.isfinite(length) or length < 1e-9:
            raise ValueError("변환한 위치·방향이 유효하지 않습니다.")
        direction /= length
        right = np.asarray([-direction[1], direction[0]])
        center += self.offset[0] * direction + self.offset[1] * right
        if (center < 0).any() or (center > self.paper_size).any():
            raise ValueError("변환한 로버 중심이 전지 범위를 벗어났습니다.")
        self.last_capture, self.last_frame = captured, frame
        return {
            "type": "pose", **self.identity,
            "coordinate_frame": "world", "unit": "cm", "origin": WORLD_ORIGIN,
            "x_cm": float(center[0]), "y_cm": float(center[1]),
            "heading_deg": degrees(atan2(direction[1], direction[0])) % 360,
            "captured_monotonic_s": captured,
            "position_reference": "floor_projected_marker_with_offset",
        }


def stream_pose(converter, server_url, *, pose_port=8766, preview_only=False):
    last_status, last_print = None, 0.0
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
        try:
            while True:
                started = time.monotonic()
                try:
                    converter.require_current()
                    with urlopen(server_url.rstrip("/") + "/api/rover-pose", timeout=POSE_MAX_AGE_S) as response:
                        observation = json.load(response)
                    pose = converter.convert(observation)
                    if pose is not None:
                        if time.monotonic() - pose["captured_monotonic_s"] > POSE_MAX_AGE_S:
                            raise ValueError("위치 계산 중 관측 유효 시간이 지났습니다.")
                        if not preview_only:
                            udp.sendto(encode(pose), ("127.0.0.1", pose_port))
                        if started - last_print >= 1:
                            print(f"x={pose['x_cm']:.2f} cm y={pose['y_cm']:.2f} cm heading={pose['heading_deg']:.1f}°", flush=True)
                            last_print = started
                        last_status = None
                except (OSError, ValueError, KeyError, TypeError, cv2.error) as error:
                    if not preview_only:
                        udp.sendto(encode({"type": "lost", **converter.identity}), ("127.0.0.1", pose_port))
                    if str(error) != last_status:
                        print(f"위치 입력 중단: {error}", file=sys.stderr, flush=True)
                        last_status = str(error)
                time.sleep(max(0.0, CONTROL_PERIOD_S - (time.monotonic() - started)))
        finally:
            if not preview_only:
                udp.sendto(encode({"type": "lost", **converter.identity}), ("127.0.0.1", pose_port))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture-dir", type=Path, default=Path("data/http_map_captures/latest"))
    parser.add_argument("--server-url", default="http://127.0.0.1:8001")
    parser.add_argument("--pose-port", type=int, default=8766)
    parser.add_argument("--marker-id", type=int, default=7)
    parser.add_argument("--center-offset-cm", nargs=2, type=float, default=(-2.5, 0.0), metavar=("FORWARD", "RIGHT"),
                        help="마커에서 구동 바퀴 축 중간점까지 전방·오른쪽 거리(cm), 기본 -2.5 0")
    parser.add_argument("--preview-only", action="store_true", help="cm 위치를 터미널에서만 확인")
    args = parser.parse_args()
    try:
        url = urlsplit(args.server_url)
        if url.scheme != "http" or url.hostname not in ("localhost", "127.0.0.1", "::1"):
            raise ValueError("time.monotonic 시각을 공유하는 같은 노트북의 HTTP 서버 주소를 지정하세요.")
        if not 0 < args.pose_port <= 65535 or args.marker_id < 0:
            raise ValueError("유효한 UDP 포트와 0 이상 마커 ID를 지정하세요.")
        converter = CapturePose(args.capture_dir, marker_id=args.marker_id, center_offset_cm=args.center_offset_cm)
        print(f"map={converter.identity['map_id']} unit=cm period={CONTROL_PERIOD_S}s UDP={args.pose_port}", flush=True)
        print("시작 전 저장한 보정식을 재사용합니다. 위치는 상단 마커의 바닥 투영 근사이며 높이 오차가 남습니다.", flush=True)
        stream_pose(converter, args.server_url, pose_port=args.pose_port, preview_only=args.preview_only)
    except KeyboardInterrupt:
        return 130
    except (OSError, ValueError, TypeError, KeyError, cv2.error, np.linalg.LinAlgError) as error:
        print(f"위치 전송 중단: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
