"""같은 카메라 구도의 픽셀 도착 시험. 저장된 실측값으로 높이 보정을 재사용한다."""

from copy import deepcopy
from datetime import datetime, timezone
from hashlib import sha256
import json
from math import dist, isfinite
from pathlib import Path
from tempfile import NamedTemporaryFile
import threading
import time

from src.map_processing.map_identity import require_map_identity
from src.map_processing.path_visualizer import DEFAULT_ROUTE_CASES
from .pose_sender import CapturePose, load_camera_position

COMPARISON_MAX_AGE_SECONDS = 5.0


class ArrivalTest:
    def __init__(self, map_root, *, marker_id=7, output_path=None, map_lock=None, parking=None):
        self.map_root = Path(map_root).resolve()
        self.marker_id = marker_id
        self.output_path = Path(output_path) if output_path is not None else None
        self.map_lock = map_lock or threading.RLock()
        self.lock = threading.RLock()
        self.state = {"status": "idle", "planning_mode": "image_pixel_test", "motor_commands": False,
                      "coordinate_frame": "map_image", "unit": "pixel"}
        self.last_received = None
        self.last_pose_expires = None
        self.last_frame = None
        self.last_capture = None
        self.last_client_capture = None
        self.hold_start = None
        self.anchor = None
        self.samples = 0
        self.parking = parking
        self.parking_signature = None
        self.last_retarget = None
        self.hold_observed = 0.0
        self.hold_pause_total = 0.0
        self.last_known_position = None
        self.position_converter = None
        self.camera_settings_bytes = None

    def _load_position_converter(self, folder):
        path = folder / "world/camera_position.json"
        if not path.is_file():
            return None, None
        settings_bytes = path.read_bytes()
        settings = load_camera_position(folder)
        converter = CapturePose(folder, marker_id=self.marker_id,
                                camera_position_cm=settings["camera_position_cm"],
                                marker_height_cm=settings["marker_height_cm"])
        return converter, settings_bytes

    def catalog(self, *, live=True):
        try:
            with self.map_lock:
                folder = self.map_root / "latest"
                report = json.loads((folder / "report.json").read_text(encoding="utf-8"))
                if not report.get("published_as_latest") or report.get("route_generation", {}).get("status") != "generated":
                    raise ValueError("검증된 지도와 5개 경로를 먼저 생성하세요.")
                source = report.get("camera_source") or {}
                if source.get("kind") != "webcam" or not source.get("session_id"):
                    raise ValueError("현재 웹캠에서 지도를 한 번 생성하세요. 이전 저장 사진에는 카메라 연결 정보가 없습니다.")
                size = report["image_size_px"]
                if source.get("width", 0) <= 0 or source.get("height", 0) <= 0:
                    raise ValueError("원본 카메라 해상도를 확인할 수 없습니다.")
                if abs(size[0] / size[1] - source["width"] / source["height"]) >= 0.005:
                    raise ValueError("지도와 원본 카메라 영상의 비율이 다릅니다.")
                identity = {"map_id": report["map_id"], "map_image_sha256": report["source_sha256"],
                            "id_assignment_revision": report["id_assignment_revision"]}
                if sha256((folder / "source.png").read_bytes()).hexdigest() != identity["map_image_sha256"]:
                    raise ValueError("기준 지도 PNG와 보고서의 해시가 다릅니다.")
                cases = []
                for case_id, _, _ in DEFAULT_ROUTE_CASES:
                    route = json.loads((folder / "routes" / f"{case_id}.json").read_text(encoding="utf-8"))
                    require_map_identity(identity, route, "시험 경로")
                    if route["unit"] != "pixel" or route["coordinate_frame"] != "map_image":
                        raise ValueError("지도 픽셀 경로가 필요합니다.")
                    goal = route["waypoints"][-1]
                    if len(goal) != 2 or not all(isfinite(v) and 0 <= v < limit for v, limit in zip(goal, size)):
                        raise ValueError("도착점이 지도 범위를 벗어났습니다.")
                    cases.append({"case_id": case_id, "parking_space_id": route["parking_space_id"],
                                  "goal_px": goal, "preview": f"/captures/latest/routes/{case_id}.png"})
                result = {"ready": True, "capture_id": report["capture_id"], **identity,
                          "image_size_px": size, "camera_source": source, "cases": cases}
                converter, _ = self._load_position_converter(folder)
                result["height_correction_applied"] = converter is not None
            # map_lock 밖에서 점유 잠금을 얻어 게시/점유 요청 간 잠금 역전을 피한다.
            if self.parking is not None and live:
                result["occupancy"] = self.parking.current()
                result["spaces"] = [{"parking_space_id": key, "status": value,
                                     "type": "accessible" if key in ("space_3", "space_6") else "normal"}
                                    for key, value in result["occupancy"]["states"].items()]
                for case in cases:
                    try:
                        case.update(self.parking.plan(case["case_id"]))
                    except ValueError as error:
                        case.update(available=False, parking_space_id=None, goal_px=None, waypoints=[], message=str(error))
            return result
        except FileNotFoundError:
            return {"ready": False, "error": "카메라를 켜고 지도와 5개 경로를 한 번 생성하세요.", "cases": []}
        except (OSError, ValueError, KeyError, TypeError, IndexError, ZeroDivisionError) as error:
            return {"ready": False, "error": str(error), "cases": []}

    def start(self, case_id, *, capture_id, radius_px=15, hold_seconds=2.0,
              parking_space_id=None, auto_retarget=True, arrival_area_ratio=0.5):
        if type(radius_px) not in (int, float) or not isfinite(radius_px) or not 3 <= radius_px <= 100:
            raise ValueError("도착 반경은 지도 기준 3~100px이어야 합니다.")
        if type(hold_seconds) not in (int, float) or not isfinite(hold_seconds) or not 0.5 <= hold_seconds <= 5:
            raise ValueError("위치 비교 간격은 0.5~5초여야 합니다.")
        if type(auto_retarget) is not bool:
            raise ValueError("자동 목적지 변경은 true/false여야 합니다.")
        with self.lock:
            catalog = self.catalog(live=False)
            if not catalog["ready"]:
                raise ValueError(catalog["error"])
            if catalog["capture_id"] != capture_id:
                raise ValueError("지도가 갱신됐습니다. 테스트 목록을 다시 불러오세요.")
            with self.map_lock:
                converter, settings_bytes = self._load_position_converter(self.map_root / "latest")
            matches = [case for case in catalog["cases"] if case["case_id"] == case_id]
            if len(matches) != 1:
                raise ValueError("생성된 5개 테스트케이스 중 하나를 선택하세요.")
            if self.parking is not None:
                plan = self.parking.plan(case_id, parking_space_id=parking_space_id, arrival_area_ratio=arrival_area_ratio)
                if plan["capture_id"] != capture_id:
                    raise ValueError("지도가 갱신됐습니다. 테스트 목록을 다시 불러오세요.")
                matches[0].update(plan)
                radius_px = plan["arrival_region"]["radius_px"]
            self.state = {"status": "waiting_for_tracking", "planning_mode": "image_pixel_test",
                          "coordinate_frame": "map_image", "unit": "pixel",
                          "position_reference": "marker_center_on_rover_top", "target_reference": "parking_road_node",
                          "motor_commands": False, **matches[0], "capture_id": capture_id,
                          "map_id": catalog["map_id"], "map_image_sha256": catalog["map_image_sha256"],
                          "id_assignment_revision": catalog["id_assignment_revision"],
                          "image_size_px": catalog["image_size_px"], "camera_source": catalog["camera_source"],
                          "radius_px": radius_px, "hold_seconds": hold_seconds,
                          "arrival_method": "two_observation_comparison",
                          "comparison_max_age_seconds": max(COMPARISON_MAX_AGE_SECONDS, 2 * hold_seconds),
                          "stability_px": min(5.0, radius_px / 3), "current_position_px": None,
                          "distance_px": None, "held_seconds": 0, "sample_count": 0,
                          "started_at": datetime.now(timezone.utc).isoformat()}
            self.position_converter, self.camera_settings_bytes = converter, settings_bytes
            self.state["height_correction_applied"] = converter is not None
            if converter is not None:
                self.state["position_reference"] = "height_corrected_marker_with_offset"
            if self.parking is not None:
                self.state.update(requested_parking_space_id=parking_space_id, auto_retarget=auto_retarget,
                                  arrival_area_ratio=arrival_area_ratio,
                                  arrival_committed=False,
                                  guidance_available=False, route_needs_position=True, route_revision=1,
                                  replan_count=0, target_changes=[])
                self.parking_signature = self.last_retarget = None
            self.last_received = self.last_frame = None
            self.last_capture = None
            self.last_client_capture = None
            self.last_pose_expires = None
            self.last_known_position = None
            self.state.update(tracking_paused=False, tracking_available=False, last_known_position_px=None)
            self._reset_hold()
            self._save()
            return self.current()

    def _reset_hold(self, reason=None):
        # 성공 후에도 지연 원인을 확인할 수 있도록 실제 진행 중인 판정의 초기화만 센다.
        self.state.setdefault("hold_reset_count", 0)
        self.state.setdefault("hold_reset_counts", {})
        self.state.setdefault("last_hold_reset", None)
        if reason is not None and self.samples:
            self.state["hold_reset_count"] += 1
            counts = self.state["hold_reset_counts"]
            counts[reason] = counts.get(reason, 0) + 1
            self.state["last_hold_reset"] = {
                "reason": reason, "held_seconds": self.hold_observed,
                "sample_count": self.samples, "hold_pause_seconds": self.hold_pause_total,
                "observed_at": datetime.now(timezone.utc).isoformat(),
            }
        self.hold_start = self.anchor = None
        self.samples = 0
        self.hold_observed = self.hold_pause_total = 0.0
        self.state.update(held_seconds=0, sample_count=0, hold_pause_seconds=0,
                          comparison_anchor_px=None, comparison_displacement_px=None)

    def _tracking_missing(self, now, reason, *, allow_grace=True):
        age = now - self.last_received if self.last_received is not None else float("inf")
        limit = self.state.get("comparison_max_age_seconds", COMPARISON_MAX_AGE_SECONDS)
        # 현재 위치의 만료는 비교 기준점의 유실과 구분한다. 새 유효 관측만 성공시킬 수 있다.
        if not allow_grace:
            self._reset_hold("marker_identity_uncertain")
        elif self.hold_start is not None and now - self.hold_start > limit:
            self._reset_hold("comparison_expired")
        reference_available = allow_grace and 0 <= age <= limit
        self.state.update(current_position_px=None, distance_px=None, guidance_available=False,
                          tracking_available=False, tracking_paused=reference_available,
                          tracking_gap_ms=round(age * 1000, 1) if isfinite(age) else None,
                          last_known_position_px=deepcopy(self.last_known_position) if reference_available else None)
        if self.anchor is not None:
            self.state.update(status="holding", reason="comparison_waiting_for_observation")
        else:
            self.state.update(status="waiting_for_tracking" if not reference_available else self.state["status"],
                              reason=reason)

    def _save(self):
        if self.output_path is None:
            return
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        pending = None
        try:
            with NamedTemporaryFile(mode="w", encoding="utf-8", dir=self.output_path.parent,
                                    prefix=".arrival-", delete=False) as stream:
                pending = Path(stream.name)
                json.dump(self.state, stream, ensure_ascii=False, allow_nan=False)
            pending.replace(self.output_path)
        finally:
            if pending is not None:
                pending.unlink(missing_ok=True)

    def stop(self, reason="user_stopped"):
        with self.lock:
            self._reset_hold(reason)
            self.state.update(status="stopped", reason=reason, current_position_px=None, distance_px=None)
            self.state.update(last_known_position_px=None, tracking_paused=False, tracking_available=False)
            self.state["guidance_available"] = False
            self._save()
            return deepcopy(self.state)

    def _check_map(self):
        if "capture_id" not in self.state or self.state["status"] in ("idle", "stopped", "invalidated"):
            return
        reason = "map_changed"
        with self.map_lock:
            try:
                report = json.loads((self.map_root / "latest/report.json").read_text(encoding="utf-8"))
                unchanged = (report["capture_id"] == self.state["capture_id"] and
                             report["source_sha256"] == self.state["map_image_sha256"] and
                             report["map_id"] == self.state["map_id"] and
                             report["id_assignment_revision"] == self.state["id_assignment_revision"])
            except (OSError, ValueError, KeyError):
                unchanged = False
            if unchanged:
                try:
                    path = self.map_root / "latest/world/camera_position.json"
                    settings_bytes = path.read_bytes() if path.is_file() else None
                    if settings_bytes != self.camera_settings_bytes:
                        raise ValueError("카메라 실측값이 변경됐습니다.")
                    if self.position_converter is not None:
                        self.position_converter.require_current()
                except (OSError, ValueError, KeyError):
                    unchanged = False
                    reason = "position_correction_changed"
        if not unchanged:
            self._reset_hold(reason)
            self.state.update(status="invalidated", reason=reason, current_position_px=None, distance_px=None)
            self.state.update(last_known_position_px=None, tracking_paused=False, tracking_available=False)
            self.state["guidance_available"] = False
            self._save()

    def current(self, *, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            self._check_map()
            if self.parking is not None:
                self._refresh_parking(now)
            if (self.state["status"] in ("running", "holding", "waiting_for_tracking") and
                    self.last_pose_expires is not None and now > self.last_pose_expires):
                self._tracking_missing(now, "tracking_stale")
                self._save()
            return deepcopy(self.state)

    def _refresh_parking(self, now):
        if self.state["status"] in ("idle", "stopped", "invalidated", "success"):
            return
        try:
            live = self.parking.current(now=now)
            self.state["occupancy"] = live
            self.state["occupancy_observed_at"] = live["observed_at"]
            signature = (live["capture_id"], tuple(sorted(live["states"].items())))
            fresh_pose = (self.last_received is not None and self.last_pose_expires is not None
                          and self.last_received <= now <= self.last_pose_expires)
            position = self.state.get("current_position_px") if fresh_pose else None
            pending = self.state.get("route_needs_position") and position is not None
            if signature == self.parking_signature and not pending:
                return
            self.parking_signature = signature
            requested = self.state.get("requested_parking_space_id")
            if self.state.get("arrival_committed") or requested is None and not self.state["auto_retarget"]:
                requested = self.state["parking_space_id"]
            plan = self.parking.plan(self.state["case_id"], parking_space_id=requested, position=position, now=now,
                                     arrival_area_ratio=self.state["arrival_area_ratio"])
            previous = self.state["parking_space_id"]
            changed = plan["parking_space_id"] != previous
            previous_free = live["states"].get(previous) == "free"
            if changed and previous_free:
                near_goal = self.state.get("distance_px") is not None and self.state["distance_px"] <= 2 * self.state["radius_px"]
                cooling = self.last_retarget is not None and now - self.last_retarget < 2.0
                if near_goal or cooling:
                    if cooling and not near_goal:
                        self.parking_signature = None
                    return
            if changed or pending or self.state["status"] == "blocked":
                self._reset_hold("target_changed" if changed else "parking_route_changed")
                self.state.update(plan, status="running" if position is not None else "waiting_for_tracking",
                                  radius_px=plan["arrival_region"]["radius_px"],
                                  stability_px=min(5.0, plan["arrival_region"]["radius_px"] / 3),
                                  guidance_available=position is not None, route_needs_position=position is None,
                                  route_revision=self.state["route_revision"] + 1,
                                  replan_count=self.state["replan_count"] + 1, distance_px=None)
                self.state.pop("reason", None)
                if changed:
                    self.last_retarget = now
                    self.state["target_changes"] = [*self.state["target_changes"],
                        {"from": previous, "to": plan["parking_space_id"],
                         "at": datetime.now(timezone.utc).isoformat(), "reason": "occupancy_changed"}][-20:]
                if position is None:
                    self.state["waypoints"] = []
                self._save()
        except (ValueError, OSError, KeyError) as error:
            self._reset_hold("occupancy_blocked")
            self.state.update(status="blocked", reason=str(error), guidance_available=False,
                              route_needs_position=True, waypoints=[], distance_px=None)
            self._save()

    def observe(self, observation, *, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            self.current(now=now)
            if self.state["status"] in ("idle", "stopped", "invalidated"):
                return deepcopy(self.state)
            if observation.get("camera_source") != self.state["camera_source"]:
                self._reset_hold("camera_changed")
                self.state.update(status="invalidated", reason="camera_changed", current_position_px=None, distance_px=None)
                self.state.update(last_known_position_px=None, tracking_paused=False, tracking_available=False)
                self.state["guidance_available"] = False
                self._save()
                return deepcopy(self.state)
            if self.state["status"] == "success":
                return deepcopy(self.state)
            received = observation.get("received_monotonic_s")
            rover = observation.get("rover")
            valid = (observation.get("tracking_status") == "detected" and rover and rover.get("id") == self.marker_id
                     and type(received) in (int, float) and isfinite(received) and 0 <= now - received <= 0.20)
            client_time = observation.get("client_captured_at")
            capture_remaining = 0.20
            captured = received
            client_capture = None
            if client_time:
                try:
                    parsed = datetime.fromisoformat(client_time.replace("Z", "+00:00"))
                    if parsed.utcoffset() is None:
                        raise ValueError("촬영 시각에 시간대가 필요합니다.")
                    age = (datetime.now(timezone.utc) - parsed).total_seconds()
                    valid = valid and -0.05 <= age <= 0.20
                    capture_remaining = min(0.20, 0.20 - age)
                    captured = now - age
                    client_capture = parsed.timestamp()
                except (ValueError, TypeError):
                    valid = False
            if not valid:
                identity_uncertain = observation.get("tracking_status") == "ambiguous" or bool(rover and rover.get("id") != self.marker_id)
                self._tracking_missing(now, "tracking_unavailable", allow_grace=not identity_uncertain)
                self._save()
                return deepcopy(self.state)
            source = self.state["camera_source"]
            if (observation.get("width"), observation.get("height")) != (source["width"], source["height"]):
                self._reset_hold("camera_resolution_changed")
                self.state.update(status="invalidated", reason="camera_resolution_changed", current_position_px=None, distance_px=None)
                self.state.update(last_known_position_px=None, tracking_paused=False, tracking_available=False)
                self.state["guidance_available"] = False
                self._save()
                return deepcopy(self.state)
            frame_key = observation.get("frame_id") or received
            if (frame_key == self.last_frame or (self.last_received is not None and received <= self.last_received)
                    or (self.last_capture is not None and captured <= self.last_capture)
                    or (client_capture is not None and self.last_client_capture is not None
                        and client_capture <= self.last_client_capture)):
                return deepcopy(self.state)
            if self.position_converter is not None:
                try:
                    with self.map_lock:
                        pose = self.position_converter.convert(observation, now=now)
                        if pose is None:
                            return deepcopy(self.state)
                        position = self.position_converter.world_to_map([pose["x_cm"], pose["y_cm"]])
                except (OSError, ValueError, KeyError, TypeError, IndexError) as error:
                    self._reset_hold("position_correction_failed")
                    self._tracking_missing(now, "position_correction_failed", allow_grace=False)
                    self.state["position_correction_error"] = str(error)
                    self._save()
                    return deepcopy(self.state)
                self.state.pop("position_correction_error", None)
            else:
                x, y = rover["center_px"]
                if not all(type(v) in (int, float) and isfinite(v) for v in (x, y)):
                    raise ValueError("유한한 마커 중심 좌표가 필요합니다.")
                position = [x * self.state["image_size_px"][0] / source["width"],
                            y * self.state["image_size_px"][1] / source["height"]]
            self.last_frame, self.last_received = frame_key, received
            self.last_capture = captured
            if client_capture is not None:
                self.last_client_capture = client_capture
            # GET에서도 촬영 후 200ms를 넘긴 좌표를 유지하지 않는다. 수신 시각으로 TTL을 새로 시작하지 않는다.
            self.last_pose_expires = min(received + 0.20, now + capture_remaining)
            self.last_known_position = list(position)
            self.state.update(tracking_available=True, tracking_paused=False, tracking_gap_ms=0,
                              last_known_position_px=list(position))
            distance = dist(position, self.state["goal_px"])
            self.state.update(current_position_px=position, distance_px=distance)
            if self.parking is not None:
                if distance <= self.state["radius_px"]:
                    # 진입한 칸에서 점유가 변하면 다른 칸으로 점프하지 않고 확인을 기다린다.
                    self.state["arrival_committed"] = True
                self._refresh_parking(now)
                if self.state["status"] == "blocked":
                    self._save()
                    return deepcopy(self.state)
                distance = dist(position, self.state["goal_px"])
                self.state.update(distance_px=distance, guidance_available=True)
            self.state.pop("reason", None)
            if distance > self.state["radius_px"]:
                self._reset_hold("outside_arrival_region")
                self.state["status"] = "running"
            else:
                if self.anchor is not None and captured - self.hold_start > self.state["comparison_max_age_seconds"]:
                    self._reset_hold("comparison_expired")
                if self.anchor is not None and dist(position, self.anchor) > self.state["stability_px"]:
                    self._reset_hold("position_unstable")
                if self.anchor is None:
                    self.anchor, self.hold_start = list(position), captured
                self.hold_observed = max(0.0, captured - self.hold_start)
                self.samples += 1
                self.state.update(status="holding", held_seconds=self.hold_observed, sample_count=self.samples,
                                  comparison_anchor_px=list(self.anchor),
                                  comparison_displacement_px=dist(position, self.anchor))
                if self.hold_observed + 1e-6 >= self.state["hold_seconds"] and self.samples >= 2:
                    self.state.update(status="success", completed_at=datetime.now(timezone.utc).isoformat())
            self._save()
            return deepcopy(self.state)
