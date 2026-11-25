"""동일 웹캠의 최신 점유로 후보를 고르고 도로 위 현재 위치부터 경로를 만든다."""

from copy import deepcopy
from datetime import datetime, timezone
import json
from math import dist, isfinite
from pathlib import Path
import threading
import time

import networkx as nx
import numpy as np

from src.map_processing.graph_builder import line_pixels, remove_small_components
from src.rover_control.arrival_geometry import arrival_circle
from src.map_processing.map_identity import map_identity, require_map_identity
from src.map_processing.map_reader import detect_road_mask, load_map_image
from src.map_processing.occupancy import DEFAULT_OCCUPANCY_TTL, occupancy_states
from src.map_processing.parking_selector import load_parking_map, select_parking_from_map
from src.map_processing.path_visualizer import DEFAULT_ROUTE_CASES, connect_waypoints


class LiveParking:
    TTL_SECONDS = DEFAULT_OCCUPANCY_TTL
    FREE_CONFIRM_SECONDS = 2.0

    def __init__(self, map_root, *, map_lock=None, output_path=None):
        self.map_root = Path(map_root)
        self.map_lock = map_lock or threading.RLock()
        self.lock = threading.RLock()
        self.output_path = Path(output_path) if output_path else None
        self.context = None
        self.snapshot = None
        self.received = None
        self.free_since = {}
        self.samples = {}
        self.last_observed = None
        self.plan_cache = {}
        self.cache_states = None

    def clear_observations(self):
        """감시 중단·구역 변경 직후 이전 free를 더 이상 사용하지 않는다."""
        with self.lock:
            self.snapshot = self.received = self.last_observed = None
            self.free_since.clear()
            self.samples.clear()
            self.plan_cache.clear()

    def _context(self):
        with self.map_lock:
            root = self.map_root / "latest"
            report = json.loads((root / "report.json").read_text(encoding="utf-8"))
            if not report.get("published_as_latest"):
                raise ValueError("새 지도를 먼저 생성하세요.")
            if self.context is None or self.context[0]["capture_id"] != report["capture_id"]:
                semantic, graph = load_parking_map(root / "semantic.json")
                require_map_identity(report, semantic, "실시간 의미 지도")
                if not graph or not nx.is_connected(graph):
                    raise ValueError("연결된 도로 그래프가 필요합니다.")
                try:
                    image = load_map_image(root / "source.png")
                except OSError as exc:
                    raise ValueError("기준 지도 PNG를 읽을 수 없습니다.") from exc
                mask = remove_small_components(detect_road_mask(image))
                self.context = (report, semantic, graph, mask)
                self.snapshot = self.received = self.last_observed = None
                self.free_since.clear()
                self.samples.clear()
                self.plan_cache.clear()
                self.cache_states = None
            return self.context

    def update(self, data, *, now=None, wall_now=None):
        now = time.monotonic() if now is None else now
        wall_now = wall_now or datetime.now(timezone.utc)
        with self.lock:
            report, semantic, _, _ = self._context()
            require_map_identity(semantic, data, "웹 점유 정보")
            if data.get("capture_id") != report["capture_id"] or data.get("camera_source") != report.get("camera_source"):
                raise ValueError("점유 정보의 지도 또는 카메라 연결이 다릅니다.")
            states = occupancy_states(semantic, data, max_age_seconds=self.TTL_SECONDS, now=wall_now)
            observed = datetime.fromisoformat(data["observed_at"])
            age = (wall_now - observed).total_seconds()
            if not 0 <= age <= self.TTL_SECONDS:
                raise ValueError("오래되거나 미래 시각인 점유 관측입니다.")
            if self.last_observed is not None and observed <= self.last_observed:
                raise ValueError("중복되거나 순서가 뒤바뀐 점유 관측입니다.")
            captured_monotonic = now - age
            # 연속성은 촬영 시각끼리 비교한다. 추론 지연을 관측 공백에 더하지 않는다.
            if self.received is None or captured_monotonic - self.received > self.TTL_SECONDS:
                self.free_since.clear()
                self.samples.clear()
            for space_id, status in states.items():
                if status == "free":
                    self.free_since.setdefault(space_id, captured_monotonic)
                    self.samples[space_id] = self.samples.get(space_id, 0) + 1
                else:
                    self.free_since.pop(space_id, None)
                    self.samples.pop(space_id, None)
            self.snapshot = deepcopy(data)
            self.received, self.last_observed = captured_monotonic, observed
            if self.output_path:
                self.output_path.parent.mkdir(parents=True, exist_ok=True)
                temporary = self.output_path.with_suffix(".pending")
                temporary.write_text(json.dumps(data, ensure_ascii=False, allow_nan=False), encoding="utf-8")
                temporary.replace(self.output_path)
            return self.current(now=now, wall_now=wall_now)

    def current(self, *, now=None, wall_now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            report, semantic, _, _ = self._context()
            raw = {r["id"]: "unknown" for r in semantic["regions"] if r.get("type") in ("normal", "accessible")}
            fresh = self.received is not None and 0 <= now - self.received <= self.TTL_SECONDS
            if fresh:
                raw.update(occupancy_states(semantic, self.snapshot, max_age_seconds=self.TTL_SECONDS, now=wall_now))
            states = {key: ("unknown" if value == "free" and
                       (self.received - self.free_since.get(key, self.received) < self.FREE_CONFIRM_SECONDS or self.samples.get(key, 0) < 3)
                       else value) for key, value in raw.items()}
            return {"capture_id": report["capture_id"], **map_identity(semantic), "fresh": fresh,
                    "ttl_seconds": self.TTL_SECONDS, "free_confirm_seconds": self.FREE_CONFIRM_SECONDS,
                    "observed_at": self.snapshot.get("observed_at") if self.snapshot else None,
                    "age_seconds": max(0, now - self.received) if self.received is not None else None,
                    "states": states, "raw_states": raw,
                    "occupancy_scope": self.snapshot.get("occupancy_scope", "all_vehicles") if self.snapshot else "all_vehicles",
                    "rover_exclusion": deepcopy(self.snapshot.get("rover_exclusion")) if self.snapshot else None}

    def plan(self, case_id, *, parking_space_id=None, position=None, now=None, wall_now=None, arrival_area_ratio=0.5):
        with self.lock:
            if type(arrival_area_ratio) not in (int, float) or not isfinite(arrival_area_ratio) or not 0.1 <= arrival_area_ratio <= 0.75:
                raise ValueError("도착 영역은 주차칸 면적의 10~75%여야 합니다.")
            if parking_space_id is not None and not isinstance(parking_space_id, str):
                raise ValueError("주차칸 ID는 문자열이어야 합니다.")
            report, semantic, graph, road_mask = self._context()
            live = self.current(now=now, wall_now=wall_now)
            case = next((c for c in DEFAULT_ROUTE_CASES if c[0] == case_id), None)
            if case is None:
                raise ValueError("생성된 5개 테스트케이스 중 하나를 선택하세요.")
            if not live["fresh"]:
                raise ValueError("최신 점유 정보가 없습니다. 상단의 감시 모드와 연결 상태를 확인하세요.")
            state_key = tuple(sorted(live["states"].items()))
            if self.cache_states != state_key:
                self.plan_cache.clear()
                self.cache_states = state_key
            if position is not None and (len(position) != 2 or
                    not all(type(v) in (int, float) and isfinite(v) for v in position)):
                raise ValueError("유효한 현재 로버 위치가 필요합니다.")
            cache_key = (case_id, parking_space_id, tuple(position) if position is not None else None, arrival_area_ratio)
            if cache_key in self.plan_cache:
                return {**deepcopy(self.plan_cache[cache_key]), "occupancy_observed_at": live["observed_at"]}
            _, preference, parking_type = case
            if parking_space_id is not None:
                region = next((r for r in semantic["regions"] if r["id"] == parking_space_id), None)
                if region is None or region.get("type") != parking_type:
                    raise ValueError("선택 조건에 맞는 주차칸 유형을 지정하세요.")
                status = live["states"].get(parking_space_id, "unknown")
                if status == "occupied":
                    raise ValueError("이미 점유된 주차칸입니다. 다른 빈칸을 선택하세요.")
                if status != "free":
                    raise ValueError("빈칸 확인 중이거나 점유 정보가 없습니다.")
            snapshot = {**self.snapshot, "spaces": [
                {"parking_space_id": key, "status": value if parking_space_id in (None, key) else "unknown", "confidence": 1.0}
                for key, value in live["states"].items()]}
            result = select_parking_from_map(semantic, graph, preference, parking_type=parking_type,
                                             occupancy=snapshot, max_age_seconds=self.TTL_SECONDS, now=wall_now)
            target = result["selected"]
            if position is None:
                node_route = result["route"]
                points = connect_waypoints(graph, node_route, road_mask)
                projected = points[0]
            else:
                # 가장 가까운 실제 간선을 나눠 입구가 아닌 현재 도로 위치에서 재탐색한다.
                candidates = []
                for u, v, key, edge in graph.edges(keys=True, data=True):
                    single = nx.MultiGraph()
                    single.add_nodes_from([(u, graph.nodes[u]), (v, graph.nodes[v])])
                    single.add_edge(u, v, **edge)
                    pixels = connect_waypoints(single, [u, v], road_mask)
                    distances = np.sum((np.asarray(pixels) - position) ** 2, axis=1)
                    index = int(np.argmin(distances))
                    candidates.append((float(distances[index]), u, v, key, pixels, index))
                if not candidates:
                    raise ValueError("현재 위치를 연결할 도로 간선이 없습니다.")
                _, u, v, key, pixels, index = min(candidates, key=lambda c: c[0])
                projected = pixels[index]
                work = graph.copy()
                if index == 0:
                    start = u
                elif index == len(pixels) - 1:
                    start = v
                else:
                    start = "__live_rover_start__"
                    work.remove_edge(u, v, key)
                    work.add_node(start, x=projected[0], y=projected[1])
                    for node, path in ((u, list(reversed(pixels[:index + 1]))), (v, pixels[index:])):
                        work.add_edge(start, node, weight=sum(dist(a, b) for a, b in zip(path, path[1:])), path=path)
                node_route = nx.dijkstra_path(work, start, target, weight="weight")
                points = connect_waypoints(work, node_route, road_mask)
            region = next(r for r in semantic["regions"] if r["id"] == result["parking_space_id"])
            circle = arrival_circle(region, area_ratio=arrival_area_ratio, image_size=report["image_size_px"])
            road_goal = list(points[-1])
            goal = circle["center_px"]
            rounded_goal = tuple(int(round(value)) for value in goal)
            approach = [list(point) for point in line_pixels(tuple(road_goal), rounded_goal)]
            if approach[-1] != goal:
                approach.append(goal)
            plan = {"capture_id": report["capture_id"], **map_identity(semantic),
                    "case_id": case_id, "parking_space_id": result["parking_space_id"],
                    "goal_px": goal, "road_goal_px": road_goal,
                    "arrival_region": circle, "target_reference": "parking_space_center",
                    "waypoint_target": "parking_center", "road_waypoint_count": len(points),
                    "waypoints": [*[list(p) for p in points], *approach[1:]],
                    "parking_approach_px": approach,
                    "parking_approach_mode": "straight_line_preview",
                    "node_route": node_route, "route_start_px": list(projected),
                    "route_start_reference": "nearest_road_pixel" if position is not None else "entrance",
                    "projection_distance_px": dist(position, projected) if position is not None else 0,
                    "score": result["score"], "occupancy_checked": True,
                    "occupancy_observed_at": live["observed_at"], "available": True}
            if len(self.plan_cache) >= 32:
                self.plan_cache.clear()
            self.plan_cache[cache_key] = deepcopy(plan)
            return plan
