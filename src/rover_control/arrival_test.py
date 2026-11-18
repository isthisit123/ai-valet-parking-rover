"""월드 보정 전, 같은 카메라 구도에서 수행하는 픽셀 도착 시험. 모터를 제어하지 않는다."""

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
        self.last_frame = None
        self.hold_start = None
        self.anchor = None
        self.samples = 0
        self.parking = parking
        self.parking_signature = None
        self.last_retarget = None

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

    def start(self, case_id, *, capture_id, radius_px=15, hold_seconds=1.0,
              parking_space_id=None, auto_retarget=True, arrival_area_ratio=0.5):
        if type(radius_px) not in (int, float) or not isfinite(radius_px) or not 3 <= radius_px <= 100:
            raise ValueError("도착 반경은 지도 기준 3~100px이어야 합니다.")
        if type(hold_seconds) not in (int, float) or not isfinite(hold_seconds) or not 0.5 <= hold_seconds <= 5:
            raise ValueError("유지 시간은 0.5~5초여야 합니다.")
        if type(auto_retarget) is not bool:
            raise ValueError("자동 목적지 변경은 true/false여야 합니다.")
        with self.lock:
            catalog = self.catalog(live=False)
            if not catalog["ready"]:
                raise ValueError(catalog["error"])
            if catalog["capture_id"] != capture_id:
                raise ValueError("지도가 갱신됐습니다. 테스트 목록을 다시 불러오세요.")
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
                          "stability_px": min(5.0, radius_px / 3), "current_position_px": None,
                          "distance_px": None, "held_seconds": 0, "sample_count": 0,
                          "started_at": datetime.now(timezone.utc).isoformat()}
            if self.parking is not None:
                self.state.update(requested_parking_space_id=parking_space_id, auto_retarget=auto_retarget,
                                  arrival_area_ratio=arrival_area_ratio,
                                  arrival_committed=False,
                                  guidance_available=False, route_needs_position=True, route_revision=1,
                                  replan_count=0, target_changes=[])
                self.parking_signature = self.last_retarget = None
            self.last_received = self.last_frame = None
            self._reset_hold()
            self._save()
            return self.current()

    def _reset_hold(self):
        self.hold_start = self.anchor = None
        self.samples = 0
        self.state.update(held_seconds=0, sample_count=0)

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
            self._reset_hold()
            self.state.update(status="stopped", reason=reason, current_position_px=None, distance_px=None)
            self.state["guidance_available"] = False
            self._save()
            return deepcopy(self.state)

    def _check_map(self):
        if "capture_id" not in self.state or self.state["status"] in ("idle", "stopped", "invalidated"):
            return
        with self.map_lock:
            try:
                report = json.loads((self.map_root / "latest/report.json").read_text(encoding="utf-8"))
                unchanged = (report["capture_id"] == self.state["capture_id"] and
                             report["source_sha256"] == self.state["map_image_sha256"] and
                             report["map_id"] == self.state["map_id"] and
                             report["id_assignment_revision"] == self.state["id_assignment_revision"])
            except (OSError, ValueError, KeyError):
                unchanged = False
        if not unchanged:
            self._reset_hold()
            self.state.update(status="invalidated", reason="map_changed", current_position_px=None, distance_px=None)
            self.state["guidance_available"] = False
            self._save()

    def current(self, *, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            self._check_map()
            if self.parking is not None:
                self._refresh_parking(now)
            if (self.state["status"] in ("running", "holding", "waiting_for_tracking") and
                    self.last_received is not None and now - self.last_received > 0.20):
                self._reset_hold()
                self.state.update(status="waiting_for_tracking", reason="tracking_stale", current_position_px=None, distance_px=None)
                if self.parking is not None:
                    self.state["guidance_available"] = False
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
            fresh_pose = self.last_received is not None and 0 <= now - self.last_received <= 0.20
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
                self._reset_hold()
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
            self._reset_hold()
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
                self._reset_hold()
                self.state.update(status="invalidated", reason="camera_changed", current_position_px=None, distance_px=None)
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
            if client_time:
                try:
                    parsed = datetime.fromisoformat(client_time.replace("Z", "+00:00"))
                    age = (datetime.now(timezone.utc) - parsed).total_seconds()
                    valid = valid and -0.05 <= age <= 0.20
                except (ValueError, TypeError):
                    valid = False
            if not valid:
                self._reset_hold()
                self.state.update(status="waiting_for_tracking", reason="tracking_unavailable", current_position_px=None, distance_px=None)
                self.state["guidance_available"] = False
                self._save()
                return deepcopy(self.state)
            source = self.state["camera_source"]
            if (observation.get("width"), observation.get("height")) != (source["width"], source["height"]):
                self._reset_hold()
                self.state.update(status="invalidated", reason="camera_resolution_changed", current_position_px=None, distance_px=None)
                self.state["guidance_available"] = False
                self._save()
                return deepcopy(self.state)
            frame_key = observation.get("frame_id") or received
            if frame_key == self.last_frame or (self.last_received is not None and received <= self.last_received):
                return deepcopy(self.state)
            if self.last_received is not None and received - self.last_received > 0.20:
                self._reset_hold()
            self.last_frame, self.last_received = frame_key, received
            x, y = rover["center_px"]
            if not all(type(v) in (int, float) and isfinite(v) for v in (x, y)):
                raise ValueError("유한한 마커 중심 좌표가 필요합니다.")
            position = [x * self.state["image_size_px"][0] / source["width"],
                        y * self.state["image_size_px"][1] / source["height"]]
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
                self._reset_hold()
                self.state["status"] = "running"
            else:
                if self.anchor is None or dist(position, self.anchor) > self.state["stability_px"]:
                    self._reset_hold()
                    self.anchor, self.hold_start = position, received
                self.samples += 1
                self.state.update(status="holding", held_seconds=max(0, received - self.hold_start), sample_count=self.samples)
                if self.state["held_seconds"] >= self.state["hold_seconds"] and self.samples >= 5:
                    self.state.update(status="success", completed_at=datetime.now(timezone.utc).isoformat())
            self._save()
            return deepcopy(self.state)
