"""실측 cm 도로 그래프와 마커 중심을 사용하는 웹 주행 경로."""
import math
from bisect import bisect_right
from itertools import accumulate

import networkx as nx
import numpy as np

from node_follower import CM_PER_PULSE, projected_point, segment_distance, simplify_path, wrap

ARRIVAL_RADIUS_CM = 2.0
LOOKAHEAD_CM = 6.5
PROGRESS_WINDOW_CM = 8.0
MAX_FORWARD_PULSES = 5
CORNER_REACH_CM = 1.5
REPLAN_DISTANCE_CM = 5.0
# 실물 속도·회전율 측정 전 초기값. 매 동작 뒤 새 ArUco 관측으로 보정한다.
FORWARD_CM_S = 50.0
TURN_DEG_S = 400.0
MIN_ACTION_MS = 40
MAX_ACTION_MS = 120


def position(pose):
    return pose['x_cm'], pose['y_cm']


class RoadMap:
    def __init__(self, semantic, converter):
        self.spaces = {r['id']: r for r in semantic['regions'] if r.get('type') in ('normal', 'accessible')}
        self.matrix = np.linalg.inv(converter.world_to_image)
        self.image_size, self.paper_size = converter.image_size, converter.paper_size
        self.graph = nx.Graph()
        self.points = {}
        nodes = {n['id']: (n['x'], n['y']) for n in semantic['nodes']}
        for index, edge in enumerate(semantic['edges']):
            u, v = edge['from'], edge['to']
            pixels = edge['path']
            if tuple(pixels[0]) != nodes[u] or tuple(pixels[-1]) != nodes[v]:
                raise ValueError(f'도로 간선의 좌표와 노드가 다릅니다: {u} → {v}')
            coords = self.to_world(pixels)
            # 같은 픽셀을 지나는 별개 도로는 그래프 노드 ID를 공유할 때만 연결한다.
            keys = [u, *((index, i) for i in range(1, len(coords) - 1)), v]
            self.points.update(zip(keys, coords))
            for a, b in zip(keys, keys[1:]):
                self.graph.add_edge(a, b, weight=math.dist(self.points[a], self.points[b]))
        self.links = list(self.graph.edges)
        if not self.links:
            raise ValueError('합류할 도로 간선이 없습니다.')

    def to_world(self, pixels):
        from src.map_processing.planar_calibration import project_points
        pixels = np.asarray(pixels, dtype=float)
        if not np.isfinite(pixels).all() or (pixels < 0).any() or (pixels >= self.image_size).any():
            raise ValueError('도로 또는 목적지 좌표가 캡처 범위를 벗어났습니다.')
        denominators = pixels @ self.matrix[2, :2] + self.matrix[2, 2]
        if not ((denominators > 1e-9).all() or (denominators < -1e-9).all()):
            raise ValueError('보정 경계를 가로지르는 도로 좌표입니다.')
        world = project_points(pixels, self.matrix)
        if not np.isfinite(world).all() or (world < 0).any() or (world > self.paper_size).any():
            raise ValueError('도로 또는 목적지가 월드 지도 범위를 벗어났습니다.')
        return [tuple(row) for row in world.tolist()]

    def parking_center(self, space_id):
        from src.rover_control.arrival_geometry import arrival_circle
        if space_id not in self.spaces:
            raise ValueError(f'지도에서 선택한 주차칸을 찾을 수 없습니다: {space_id}')
        center = arrival_circle(self.spaces[space_id])['center_px']
        return self.to_world([center])[0]

    def nearest(self, point):
        def candidate(link):
            a, b = link
            foot = projected_point(point, self.points[a], self.points[b])
            return math.dist(point, foot), link, foot
        _, link, foot = min(map(candidate, self.links), key=lambda item: item[0])
        return link, foot

    def path(self, start, goal):
        graph = self.graph.copy()
        points = self.points.copy()
        first, last = object(), object()
        entry, points[first] = self.nearest(start)
        exit_link, points[last] = self.nearest(goal)
        for key, link in ((first, entry), (last, exit_link)):
            for end in link:
                graph.add_edge(key, end, weight=math.dist(points[key], points[end]))
        if entry == exit_link:
            graph.add_edge(first, last, weight=math.dist(points[first], points[last]))
        try:
            route = nx.dijkstra_path(graph, first, last, weight='weight')
        except nx.NetworkXNoPath as error:
            raise ValueError('현재 위치와 목적지의 도로가 연결되지 않았습니다.') from error
        road = [points[key] for key in route]
        road = simplify_path(road) if any(p != road[0] for p in road) else road[:1]
        # 직선 정리는 도로 구간 안에서만 한다. 실제 출발점·합류점·이탈점·칸 중심을 보존한다.
        path = [start]
        for point in [*road, goal]:
            if point != path[-1]:
                path.append(point)
        return path


class WebPath:
    def __init__(self, roads, start, goal, radius):
        self.roads, self.goal, self.radius = roads, goal, radius
        self.reset(start)

    def reset(self, start):
        self.points = ([start, self.goal] if self.arrived(start)
                       else self.roads.path(start, self.goal))
        self.index = 1
        self.lengths = [0.0, *accumulate(math.dist(a, b) for a, b in zip(self.points, self.points[1:]))]
        self.progress = 0.0
        self.turning = False
        self.corners = []
        self.cleared_corners = set()
        for index in range(1, len(self.points) - 1):
            a, b, c = self.points[index - 1:index + 2]
            incoming = b[0] - a[0], b[1] - a[1]
            outgoing = c[0] - b[0], c[1] - b[1]
            length = math.hypot(*incoming) * math.hypot(*outgoing)
            if length and sum(x * y for x, y in zip(incoming, outgoing)) / length <= math.cos(math.radians(45)):
                self.corners.append(index)

    def arrived(self, point):
        return math.dist(point, self.goal) <= self.radius

    def point_at(self, distance):
        distance = min(self.lengths[-1], max(0.0, distance))
        index = min(len(self.points) - 2, bisect_right(self.lengths, distance) - 1)
        start, end = self.points[index:index + 2]
        length = self.lengths[index + 1] - self.lengths[index]
        fraction = (distance - self.lengths[index]) / length if length else 0
        return tuple(a + fraction * (b - a) for a, b in zip(start, end))

    def corner_limit(self, point):
        corner_limit = self.lengths[-1]
        for index in self.corners:
            if self.lengths[index] < self.progress or index in self.cleared_corners:
                continue
            a, b, c = self.points[index - 1:index + 2]
            if math.dist(point, b) <= CORNER_REACH_CM:
                self.cleared_corners.add(index)
                continue
            # 코너를 지난 뒤 작은 초과 이동 때문에 뒤쪽 꼭짓점으로 돌아가지 않는다.
            dx, dy = b[0] - a[0], b[1] - a[1]
            passed = (point[0] - a[0]) * dx + (point[1] - a[1]) * dy >= dx * dx + dy * dy
            if passed and segment_distance(point, b, c) <= REPLAN_DISTANCE_CM:
                self.cleared_corners.add(index)
                continue
            corner_limit = self.lengths[index]
            break
        return corner_limit

    def target(self, pose):
        point = position(pose)
        corner_limit = self.corner_limit(point)
        limit = min(self.lengths[-1], self.progress + PROGRESS_WINDOW_CM, corner_limit)
        best = math.dist(point, self.point_at(self.progress)), self.progress
        # 같은 경로의 현재 위치부터 가까운 앞 구간만 검사한다.
        for index in range(self.index - 1, len(self.points) - 1):
            if self.lengths[index] > limit:
                break
            a, b = self.points[index:index + 2]
            foot = projected_point(point, a, b)
            along = self.lengths[index] + math.dist(a, foot)
            along = min(limit, max(self.progress, along))
            best = min(best, (math.dist(point, self.point_at(along)), along))
        if best[0] > REPLAN_DISTANCE_CM:
            self.reset(point)
            corner_limit = self.corner_limit(point)
        else:
            self.progress = best[1]
        self.index = min(len(self.points) - 1, bisect_right(self.lengths, self.progress))
        target = self.point_at(min(self.lengths[-1], self.progress + LOOKAHEAD_CM, corner_limit))
        dx, dy = target[0] - point[0], target[1] - point[1]
        error = wrap(math.degrees(math.atan2(dy, dx)) - pose['heading_deg'])
        # 작은 방향 흔들림으로 전진과 회전을 번갈아 명령하지 않도록 한다.
        self.turning = abs(error) > (9.0 if self.turning else 15.0)
        distance = min(math.hypot(dx, dy), max(0, math.dist(point, self.goal) - self.radius))
        return error, distance

    def action(self, pose, width):
        if self.arrived(position(pose)):
            return None
        error, distance = self.target(pose)
        if self.turning:
            return (2 if error > 0 else 3), min(MAX_ACTION_MS, max(MIN_ACTION_MS, round(abs(error) / TURN_DEG_S * 1000)))
        return 1, min(MAX_ACTION_MS, max(MIN_ACTION_MS, round(distance / FORWARD_CM_S * 1000)))

    def encoder_action(self, pose, width):
        if self.arrived(position(pose)):
            return None
        error, distance = self.target(pose)
        if self.turning:
            degrees_per_pulse = math.degrees(2 * CM_PER_PULSE / width)
            return (2 if error > 0 else 3), min(2, max(1, int(abs(error) / degrees_per_pulse)))
        return 1, min(MAX_FORWARD_PULSES, max(1, int(distance / CM_PER_PULSE)))
