"""Laptop: consume the repository pose_sender UDP stream; run small encoder jobs.
Run from repository root. This script itself uses Python standard library only.
Default is DRY RUN (no TCP connection). Pass --drive only after checks.
"""
import argparse
import json
import math
import socket
from statistics import median
import time

M_PER_PULSE = math.pi * 0.065 / 20
CM_PER_PULSE = M_PER_PULSE * 100
DEFAULT_TRACK_WIDTH_CM = 15.0
WORLD_FRAME = ("world", "cm", "paper_top_left")
WORLD_AXES = {"x": "right", "y": "down"}
START_RADIUS_CM = 3.0
IDENTITY_KEYS = ("map_image_sha256", "id_assignment_revision")
POSE_AGE = 0.20
ROVER_REPLY_TIMEOUT_S = 1.5
GOAL_CM = 2.0
CORNER_CM = 1.0
MAX_TRACK_ERROR_CM = 2.0
SIMPLIFY_CM = 0.20
HEADING_TOLERANCE_DEG = 5.0
# Leave a small margin for measurement jitter at the acceptance boundary.
POSITION_MARGIN_CM = 0.15
HEADING_MARGIN_DEG = 0.30


def number(value):
    if type(value) not in (int, float):
        raise ValueError('finite numeric value required')
    try:
        value = float(value)
    except OverflowError as error:
        raise ValueError('finite numeric value required') from error
    if not math.isfinite(value):
        raise ValueError('finite numeric value required')
    return value


def wrap(angle):
    return (angle + 180) % 360 - 180


def frame_of(data):
    return data.get('coordinate_frame'), data.get('unit'), data.get('origin')


def load_route(route, *, require_verified=False):
    if not isinstance(route, dict) or not isinstance(route.get('map_id'), str) or not route['map_id'].strip():
        raise ValueError('map_id required')
    # 실주행은 출발점을 포함한 검증된 지도팀 경로만 허용한다.
    if require_verified:
        if 'nodes' in route or 'waypoints' not in route:
            raise ValueError('실주행에는 출발점을 포함한 검증된 waypoints 경로가 필요합니다.')
        if route.get('axis_directions') != WORLD_AXES:
            raise ValueError('실주행 좌표축은 x 오른쪽, y 아래여야 합니다.')
        for key in IDENTITY_KEYS:
            if not isinstance(route.get(key), str) or not route[key].strip():
                raise ValueError(f'실주행 경로에 {key}가 필요합니다.')
        digest = route['map_image_sha256']
        if len(digest) != 64 or any(char not in '0123456789abcdef' for char in digest):
            raise ValueError('map_image_sha256은 소문자 16진수 64자리여야 합니다.')
    if 'axis_directions' in route and route['axis_directions'] != WORLD_AXES:
        raise ValueError('경로 좌표축은 x 오른쪽, y 아래여야 합니다.')
    if 'waypoints' in route:
        if 'nodes' in route or frame_of(route) != WORLD_FRAME or route.get('calibration_verified') is not True:
            raise ValueError('waypoints must be verified world cm with paper_top_left origin')
        points = route['waypoints']
    else:
        if (route.get('coordinate_frame', 'world'), route.get('unit'),
                route.get('origin', 'paper_top_left')) != WORLD_FRAME or route.get('calibration_verified') is False:
            raise ValueError('legacy nodes must use the world cm frame')
        points = route.get('nodes')
    if not isinstance(points, list) or not points:
        raise ValueError('at least one target node is required')
    for key in IDENTITY_KEYS:
        if key in route and (not isinstance(route[key], str) or not route[key].strip()):
            raise ValueError(f'invalid route {key}')
    nodes = []
    for xy in points:
        if not isinstance(xy, list) or len(xy) != 2:
            raise ValueError('each node needs [x, y]')
        nodes.append(tuple(number(v) for v in xy))
    if require_verified and len(set(nodes)) < 2:
        raise ValueError('실주행 경로에는 서로 다른 출발점과 목표점이 필요합니다.')
    heading = number(route['final_heading_deg']) if 'final_heading_deg' in route else None
    return nodes, heading


def require_start_position(pose, start):
    # 중앙값만 통과하고 일부 관측이 출발 반경 밖인 경우도 거부한다.
    samples = pose.get('position_samples', [])
    if len(samples) != 3:
        raise ValueError('출발 전에 안정된 새 위치 관측 3개가 필요합니다.')
    positions = [(pose['x_cm'], pose['y_cm']), *samples]
    distances = [math.hypot(number(x) - start[0], number(y) - start[1]) for x, y in positions]
    if max(distances) > START_RADIUS_CM:
        raise ValueError(f'로버를 첫 waypoint(출발점)에서 {START_RADIUS_CM:g}cm 이내에 놓으세요. '
                         f'최대 관측 거리: {max(distances):.2f}cm')


def plan(pose, target, track_width_cm=DEFAULT_TRACK_WIDTH_CM, final_heading_deg=None, *, goal_cm=GOAL_CM):
    dx, dy = target[0] - pose['x_cm'], target[1] - pose['y_cm']
    distance = math.hypot(dx, dy)
    positions = pose.get('position_samples', [(pose['x_cm'], pose['y_cm'])])
    within_goal = all(math.hypot(target[0] - x, target[1] - y) <= goal_cm - POSITION_MARGIN_CM
                      for x, y in positions)
    if within_goal and final_heading_deg is None:
        return None, distance, 0.0
    target_heading = final_heading_deg if within_goal else math.degrees(math.atan2(dy, dx))
    error = wrap(target_heading - pose['heading_deg'])
    headings = pose.get('heading_samples', [pose['heading_deg']])
    if any(abs(wrap(target_heading - heading)) > HEADING_TOLERANCE_DEG - HEADING_MARGIN_DEG for heading in headings):
        pivot_deg_per_pulse = math.degrees(2 * CM_PER_PULSE / track_width_cm)
        n = min(2, max(1, int(abs(error) / pivot_deg_per_pulse)))
        return (2 if error > 0 else 3, n), distance, error
    if within_goal:
        return None, distance, error
    n = min(5, max(1, int((distance - goal_cm) / CM_PER_PULSE)))
    return (1, n), distance, error


class Poses:
    def __init__(self, port, route):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', port))
        self.sock.setblocking(False)
        self.map_id = route['map_id']
        self.identity = {key: route[key] for key in IDENTITY_KEYS if key in route}
        self.last = None

    def update(self, required=True):
        # Drain queued packets; never use old packets to refresh age.
        while True:
            try:
                raw = self.sock.recv(65535)
            except BlockingIOError:
                break
            p = json.loads(raw)
            if p.get('type') != 'pose':
                self.last = None
                raise RuntimeError('ArUco lost: stopped; restart manually')
            if (p.get('map_id') != self.map_id or frame_of(p) != WORLD_FRAME
                    or p.get('height_correction_applied') is not True):
                raise ValueError('wrong map/frame/unit or missing height correction')
            sha = p.get('map_image_sha256')
            if not isinstance(sha, str) or not sha.strip():
                raise ValueError('map_image_sha256 missing')
            for key in IDENTITY_KEYS:
                if key in self.identity and p.get(key) != self.identity[key]:
                    raise ValueError(f'map identity mismatch: {key}')
                if key in p:
                    self.identity[key] = p[key]
            for key in ('x_cm', 'y_cm', 'heading_deg', 'captured_monotonic_s'):
                p[key] = number(p[key])
            age = time.monotonic()-p['captured_monotonic_s']
            if not 0 <= age <= POSE_AGE:
                raise ValueError('stale pose: sender must run on SAME laptop')
            if self.last and p['captured_monotonic_s'] <= self.last['captured_monotonic_s']:
                raise ValueError('repeated or reversed pose timestamp')
            self.last = p
        if required and (self.last is None or not 0 <= time.monotonic()-self.last['captured_monotonic_s'] <= POSE_AGE):
            raise TimeoutError('no fresh ArUco pose')
        return self.last

    def close(self):
        self.sock.close()


class MCUFault(RuntimeError):
    """음수 상태를 반환하면 MCU는 제동 상태를 유지하고 수신기는 연결을 닫는다."""


class Rover:
    def __init__(self, host, port=8765):
        # 무장 전 TCP 연결은 핫스팟의 초기 연결 지연을 허용한다.
        try:
            self.sock = socket.create_connection((host, port), timeout=2.0)
        except TimeoutError as error:
            raise TimeoutError(f'보드 {host}:{port} TCP 연결이 2초 안에 완료되지 않았습니다.') from error
        self.sock.settimeout(ROVER_REPLY_TIMEOUT_S)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.seq = 0
        self.buffer = bytearray()

    def request(self, kind, **kwargs):
        if self.sock.fileno() == -1:
            raise ConnectionError('보드 연결이 종료돼 추가 명령을 전송할 수 없습니다.')
        try:
            return self._request(kind, **kwargs)
        except (OSError, ValueError, RuntimeError, KeyError, TypeError):
            # 응답을 놓친 연결에 다음 명령을 보내면 이전 seq 응답을 잘못 읽게 된다.
            # 연결을 닫으면 수신기의 finally와 MCU 자체 시간 제한이 제동한다.
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self.sock.close()
            raise

    def _request(self, kind, **kwargs):
        timeout = ROVER_REPLY_TIMEOUT_S
        self.seq += 1
        deadline = time.monotonic()+timeout
        self.sock.settimeout(timeout)
        self.sock.sendall(json.dumps(dict(type=kind, seq=self.seq, **kwargs)).encode()+b'\n')
        while b'\n' not in self.buffer:
            remaining = deadline-time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f'보드 {kind} 응답이 {timeout*1000:g}ms 안에 도착하지 않았습니다.')
            self.sock.settimeout(remaining)
            try:
                chunk = self.sock.recv(2048)
            except TimeoutError as error:
                raise TimeoutError(f'보드 {kind} 응답이 {timeout*1000:g}ms 안에 도착하지 않았습니다.') from error
            if not chunk:
                raise ConnectionError('rover closed connection')
            self.buffer.extend(chunk)
            if len(self.buffer)>4096:
                raise ValueError('oversized reply')
        line, _, self.buffer = self.buffer.partition(b'\n')
        reply = json.loads(line)
        if reply.get('seq') == self.seq and isinstance(reply.get('error'), str):
            raise RuntimeError(f"보드 내부 통신 중단: {reply['error']}")
        if reply['seq'] != self.seq or type(reply['state']) is not int:
            raise ValueError('invalid reply')
        if reply['state'] < 0:
            code = reply['state']
            details = {-4: '전방 ToF 장애물 감지', -5: '전방 ToF 거리 측정 무효 또는 만료 (150ms)'}
            self.sock.close()
            raise MCUFault(f"MCU fault {code} ({kind}): {details.get(code, '엔코더 수신 앱의 고장 코드 확인')}")
        return reply['state']

    def close(self):
        if self.sock.fileno() == -1:
            return
        try:
            self.request('stop')
        except (OSError, ValueError, RuntimeError, KeyError) as error:
            print(f'STOP reply unavailable: {error}', flush=True)
        finally:
            self.sock.close()


def settled_pose(samples):
    headings = [p['heading_deg'] for p in samples]
    anchor = headings[-1]
    offsets = [wrap(heading - anchor) for heading in headings]
    xs = [p['x_cm'] for p in samples]
    ys = [p['y_cm'] for p in samples]
    positions = list(zip(xs, ys))
    x, y = median(xs), median(ys)
    if max(offsets) - min(offsets) > 3 or any(math.hypot(px - x, py - y) > 1 for px, py in positions):
        return None
    return dict(samples[-1], x_cm=x, y_cm=y, heading_deg=wrap(anchor + median(offsets)),
                heading_samples=headings, position_samples=positions)


def wait_settled(poses, rover, initial=False, *, check=None):
    # Require >=3 newly captured poses AFTER braking/settling, not the pose
    # from before the action. Poll during waiting to maintain TCP watchdog.
    after = time.monotonic() + (0.0 if initial else 0.30)
    deadline = after + 3
    samples = []
    stamp = -1
    while time.monotonic() < deadline:
        p = poses.update(required=not initial)
        if p is not None and check:
            check(p)
        if rover and rover.request('poll') != 0:
            raise RuntimeError('MCU must remain braked while sampling pose')
        if p and p['captured_monotonic_s'] > max(after, stamp):
            stamp = p['captured_monotonic_s']
            samples = (samples + [p])[-3:]
            if len(samples) == 3:
                stable = settled_pose(samples)
                if stable is not None:
                    return stable
        time.sleep(0.04)
    raise TimeoutError('fresh stable observations unavailable')


def execute(rover, poses, action, *, check=None):
    pose = poses.update()
    if check:
        check(pose)
    if rover.request('step', mode=action[0], pulses=action[1]) != 1:
        raise RuntimeError('step rejected')
    deadline = time.monotonic()+7
    while time.monotonic() < deadline:
        time.sleep(0.04)
        pose = poses.update()  # lost/stale aborts, finally sends stop
        if check:
            check(pose)
        state = rover.request('poll')
        if state == 0:
            return wait_settled(poses, rover, check=check)
        if state != 1:
            raise RuntimeError('unexpected MCU state')
    raise TimeoutError('step execution expired')


def projected_point(point, start, end, lookahead_cm=0.0):
    dx, dy = end[0] - start[0], end[1] - start[1]
    length2 = dx * dx + dy * dy
    along = ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length2 if length2 else 0
    fraction = min(1.0, max(0.0, along) + (lookahead_cm / math.sqrt(length2) if length2 else 0))
    return start[0] + fraction * dx, start[1] + fraction * dy


def segment_distance(point, start, end):
    return math.dist(point, projected_point(point, start, end))


def simplify_path(points, tolerance=SIMPLIFY_CM):
    """방향 반전과 회전점을 보존하고 직선의 픽셀 계단만 제한적으로 정리한다."""
    points = [tuple(p) for i, p in enumerate(points) if i == 0 or tuple(p) != tuple(points[i - 1])]
    if len(points) < 2:
        raise ValueError('서로 다른 경로점이 두 개 이상 필요합니다.')
    kept = {0, len(points) - 1}
    for i in range(1, len(points) - 1):
        a, b, c = points[i - 1:i + 2]
        if (b[0] - a[0]) * (c[0] - b[0]) + (b[1] - a[1]) * (c[1] - b[1]) < 0:
            kept.add(i)
    anchors = sorted(kept)
    pending = list(zip(anchors, anchors[1:]))
    while pending:
        start, end = pending.pop()
        if end <= start + 1:
            continue
        error, index = max((segment_distance(points[i], points[start], points[end]), i)
                           for i in range(start + 1, end))
        if error > tolerance:
            kept.add(index)
            pending.extend(((start, index), (index, end)))
    return [points[i] for i in sorted(kept)]


class PathDeviation(RuntimeError):
    pass


class PathGuard:
    def __init__(self, extra=None, max_error_cm=MAX_TRACK_ERROR_CM):
        self.segment = None
        self.extra = extra
        self.max_error_cm = max_error_cm

    def __call__(self, pose, segment=None):
        if segment is not None:
            self.segment = segment
        if self.segment is not None:
            positions = [(pose['x_cm'], pose['y_cm']), *pose.get('position_samples', [])]
            distance = max(segment_distance(p, *self.segment) for p in positions)
            if distance > self.max_error_cm:
                raise PathDeviation(f'현재 경로 구간에서 {distance:.2f}cm 이탈했습니다 (허용 {self.max_error_cm:g}cm).')
        if self.extra:
            self.extra(pose)


def follow(nodes, final_heading, track_width, execute_action, pose, *, clock=time.monotonic,
           log=print, path_mode=False, check=None, recover=None):
    if path_mode:
        nodes = simplify_path(nodes)
        targets = nodes[1:]
    else:
        targets = nodes
    for i, target in enumerate(targets):
        heading = final_heading if i == len(targets) - 1 else None
        goal_cm = CORNER_CM if path_mode and i < len(targets) - 1 else GOAL_CM
        deadline = clock() + 120
        last_progress = clock()
        best = float('inf')
        previous_phase = None
        turns = 0
        while True:
            if check:
                check(pose, (nodes[i], target) if path_mode else None)
            if recover is not None:
                pose = recover(pose, (nodes[i], target))
            action, distance, error = plan(pose, target, track_width, heading, goal_cm=goal_cm)
            log(f'node={i+1} d={distance:.2f}cm errorCW={error:+.2f}deg job={action}', flush=True)
            if action is None:
                break
            now = clock()
            phase = 'turn' if action[0] != 1 else 'move'
            metric = abs(error) if phase == 'turn' else distance
            minimum_progress = 0.5 if phase == 'turn' else 0.25
            if phase != previous_phase or metric < best - minimum_progress:
                best, last_progress = metric, now
            previous_phase = phase
            if now > deadline or now - last_progress > 30:
                raise TimeoutError('no node progress / node timeout')
            turns = turns + 1 if action[0] != 1 else 0
            if turns > 40:
                raise RuntimeError('too many turns; check heading polarity/calibration')
            pose = execute_action(action)
    return pose


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', help='UNO Q IP address (required for --drive)')
    parser.add_argument('--nodes', '--route', dest='route', required=True, help='legacy nodes or verified cm waypoints JSON')
    parser.add_argument('--track-width-cm', type=float, help='measured wheel-centre distance (required for --drive)')
    parser.add_argument('--pose-port', type=int, default=8766)
    parser.add_argument('--drive', action='store_true', help='actually enable motors (default DRY RUN)')
    args = parser.parse_args()
    if args.drive and (not args.host or args.track_width_cm is None):
        parser.error('--drive requires --host and measured --track-width-cm')
    track_width = DEFAULT_TRACK_WIDTH_CM if args.track_width_cm is None else number(args.track_width_cm)
    if not 0 < track_width:
        parser.error('--track-width-cm must be positive')
    with open(args.route, encoding='utf-8-sig') as f:
        route = json.load(f)
    if not 0 < args.pose_port <= 65535:
        parser.error('--pose-port must be between 1 and 65535')
    nodes, final_heading = load_route(route, require_verified=args.drive)
    poses, rover = Poses(args.pose_port, route), None
    try:
        p = wait_settled(poses, None, initial=True)
        if args.drive:
            require_start_position(p, nodes[0])
            rover = Rover(args.host)
            rover.request('stop')
            p = wait_settled(poses, rover)
            # 연결 및 정지 응답 대기 중 위치가 바뀌었으면 step을 보내지 않는다.
            require_start_position(p, nodes[0])
        else:
            print('DRY RUN: motors disconnected. Ctrl+C to exit.', flush=True)
            while True:
                p = poses.update()
                print('next:', plan(p, nodes[0], track_width, final_heading if len(nodes) == 1 else None), flush=True)
                time.sleep(0.10)
        guard = PathGuard()
        follow(nodes, final_heading, track_width,
               lambda action: execute(rover, poses, action, check=guard), p, path_mode=True, check=guard)
        rover.request('stop')
        print('All nodes reached. Braked.', flush=True)
    finally:
        try:
            if rover:
                rover.close()
        finally:
            poses.close()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Stopped by user.')
