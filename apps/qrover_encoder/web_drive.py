"""기존 ParkingHaeven 웹의 선택·중단·위치·점유 API를 엔코더 주행기에 연결한다.

기본 실행은 모터에 연결하지 않는 사전 검사다. --drive가 있어야 TCP를 연결한다.
"""
import argparse
from copy import deepcopy
from datetime import datetime, timezone
from hashlib import sha256
import json
import math
from pathlib import Path
import sys
import threading
import time
from urllib.parse import urlsplit
from urllib.request import build_opener, ProxyHandler

from node_follower import (CM_PER_PULSE, GOAL_CM, IDENTITY_KEYS, PathGuard, Rover, execute, follow,
                           load_route, number, require_start_position, simplify_path, wait_settled)
from vehicle_geometry import VehicleGeometry

WEB_TTL_S = 0.20


class RouteChanged(RuntimeError):
    pass


def route_key(state):
    return (state.get('capture_id'), state.get('started_at'), state.get('case_id'),
            state.get('route_revision'), state.get('parking_space_id'),
            sha256(json.dumps(state.get('waypoints'), separators=(',', ':')).encode()).hexdigest())


def validate_guidance(state, occupancy, identity, capture_id):
    if state.get('status') not in ('running', 'holding', 'success') or state.get('guidance_available') is not True:
        raise RuntimeError('웹 경로 안내가 중단됐습니다. 조건·빈칸·마커 상태를 확인하고 다시 시작하세요.')
    if state.get('height_correction_applied') is not True:
        raise ValueError('웹 지도 생성에서 카메라 실측 좌표와 cm 보정을 등록하세요.')
    for data in (state, occupancy):
        if data.get('capture_id') != capture_id or any(data.get(k) != identity[k] for k in ('map_id', *IDENTITY_KEYS)):
            raise ValueError('웹 경로·점유·보정의 지도 식별자가 다릅니다.')
    age = number(occupancy.get('age_seconds'))
    ttl = number(occupancy.get('ttl_seconds'))
    if occupancy.get('fresh') is not True or not 0 <= age <= min(5.0, ttl):
        raise RuntimeError('점유 정보가 만료됐습니다. 모터를 정지합니다.')
    if occupancy.get('states', {}).get(state.get('parking_space_id')) != 'free':
        raise RuntimeError('선택한 주차칸이 최신 free 상태가 아닙니다.')
    if state.get('coordinate_frame') != 'map_image' or state.get('unit') != 'pixel':
        raise ValueError('웹 안내는 같은 캡처의 지도 픽셀 경로여야 합니다.')
    if type(state.get('route_revision')) is not int or state['route_revision'] < 1 or not state.get('started_at'):
        raise ValueError('웹 테스트 시작 시각과 경로 버전이 필요합니다.')


class WebFeed:
    def __init__(self, url, converter, settings_path):
        parsed = urlsplit(url)
        if parsed.scheme != 'http' or parsed.hostname not in ('localhost', '127.0.0.1', '::1'):
            raise ValueError('같은 노트북의 localhost HTTP 서버만 연결할 수 있습니다.')
        self.url, self.converter = url.rstrip('/'), converter
        self.settings_path = settings_path
        self.settings_bytes = settings_path.read_bytes()
        self.opener = build_opener(ProxyHandler({}))
        self.lock, self.ready, self.done = threading.Lock(), threading.Event(), threading.Event()
        self.latest, self.error, self.last_pose = None, None, None
        self.thread = threading.Thread(target=self._run, daemon=True)

    def read(self, path):
        with self.opener.open(self.url + path, timeout=0.12) as response:
            body = response.read(2_000_001)
        if len(body) > 2_000_000:
            raise ValueError('웹 응답이 너무 큽니다.')
        result = json.loads(body)
        if not isinstance(result, dict):
            raise ValueError('웹 API는 JSON 객체를 반환해야 합니다.')
        return result

    def _run(self):
        try:
            while not self.done.is_set():
                started = time.monotonic()
                self.converter.require_current()
                if self.settings_path.read_bytes() != self.settings_bytes:
                    raise ValueError('카메라 실측값이 변경됐습니다. 다시 실행하세요.')
                observation = self.read('/api/rover-pose')
                occupancy = self.read('/api/parking-occupancy')
                pose = self.converter.convert(observation)
                if pose is not None:
                    self.last_pose = pose
                if self.last_pose is not None:
                    with self.lock:
                        self.latest = {'pose': self.last_pose, 'state': observation.get('arrival_test', {}),
                                       'occupancy': occupancy, 'requested_at': started,
                                       'client_captured_at': observation.get('client_captured_at')}
                    self.ready.set()
                self.done.wait(max(0, 0.05 - (time.monotonic() - started)))
        except Exception as error:
            # 통신·추적 실패를 자동 복구해 모터가 재출발하지 않게 한다.
            with self.lock:
                self.error = error
            self.ready.set()

    def start(self):
        self.thread.start()
        if not self.ready.wait(3):
            raise TimeoutError('현재 웹 ArUco 관측을 3초 동안 받지 못했습니다.')
        return self.current()

    def current(self):
        with self.lock:
            if self.error is not None:
                raise RuntimeError(f'웹 위치·점유 연결 중단: {self.error}') from self.error
            result = deepcopy(self.latest)
        if result is None:
            raise TimeoutError('현재 웹 위치가 없습니다.')
        now = time.monotonic()
        if not 0 <= now - result['requested_at'] <= WEB_TTL_S:
            raise TimeoutError('웹 안내 갱신이 200ms를 넘겼습니다.')
        if not 0 <= now - result['pose']['captured_monotonic_s'] <= WEB_TTL_S:
            raise TimeoutError('로버 현재 위치가 200ms를 넘겼습니다.')
        if result.get('client_captured_at'):
            captured = datetime.fromisoformat(result['client_captured_at'].replace('Z', '+00:00'))
            if not -0.05 <= (datetime.now(timezone.utc) - captured).total_seconds() <= WEB_TTL_S:
                raise TimeoutError('웹캠 촬영 시각이 200ms를 넘겼습니다.')
        # 조회 이후 경과한 시간도 점유 TTL에 포함한다. 반복 조회로 free 수명을 늘리지 않는다.
        result['occupancy']['age_seconds'] = number(result['occupancy'].get('age_seconds')) + now - result['requested_at']
        return result

    def update(self, required=True):
        return self.current()['pose']

    def close(self):
        self.done.set()
        self.thread.join(timeout=0.5)


def make_plan(bundle, converter, semantic, final_heading=None):
    """바닥 픽셀 경로는 바닥 Homography만 적용한다. 마커 높이 보정을 중복 적용하지 않는다."""
    import numpy as np
    from src.map_processing.planar_calibration import project_points

    state, occupancy = bundle['state'], bundle['occupancy']
    capture_id = json.loads(converter.report_bytes)['capture_id']
    validate_guidance(state, occupancy, converter.identity, capture_id)
    points = state.get('waypoints')
    if not isinstance(points, list) or len(points) < 2:
        raise ValueError('주행할 픽셀 경로가 없습니다.')
    pixels = np.asarray([[number(x), number(y)] for x, y in points])
    if (pixels < 0).any() or (pixels >= converter.image_size).any():
        raise ValueError('픽셀 경로가 같은 캡처의 영상 범위를 벗어났습니다.')
    matrix = np.linalg.inv(converter.world_to_image)
    denominators = pixels @ matrix[2, :2] + matrix[2, 2]
    if (np.abs(denominators) < 1e-9).any() or (denominators > 0).any() and (denominators < 0).any():
        raise ValueError('보정 경계를 가로지르는 경로는 사용할 수 없습니다.')
    world = project_points(pixels, matrix)
    if not np.isfinite(world).all() or (world < 0).any() or (world > converter.paper_size).any():
        raise ValueError('변환한 경로가 월드 지도 범위를 벗어났습니다.')
    compact = simplify_path(world.tolist())
    heading = (math.degrees(math.atan2(compact[-1][1] - compact[-2][1], compact[-1][0] - compact[-2][0]))
               if final_heading is None else number(final_heading))
    route = {**converter.identity, 'coordinate_frame': 'world', 'unit': 'cm', 'origin': 'paper_top_left',
             'axis_directions': {'x': 'right', 'y': 'down'}, 'calibration_verified': True,
             'waypoints': world.tolist(), 'final_heading_deg': heading,
             'parking_space_id': state['parking_space_id'], 'route_revision': state['route_revision']}
    load_route(route, require_verified=True)
    obstacles, target = [], None
    for region in semantic['regions']:
        rid = region['id']
        is_parking = region.get('type') in ('normal', 'accessible')
        is_building = str(region.get('role', rid)).startswith('building_')
        if not is_parking and not is_building:
            continue
        polygon = region.get('polygon_px')
        if not polygon:
            x, y, width, height = region['bbox']
            polygon = [[x, y], [x + width, y], [x + width, y + height], [x, y + height]]
        polygon = project_points(polygon, matrix).tolist()
        if rid == state['parking_space_id']:
            target = polygon
        else:
            # 다른 주차칸은 free 여부와 무관하게 이번 주행의 진입 영역에서 제외한다.
            obstacles.append((rid, polygon))
    if target is None:
        raise ValueError('목표 주차칸의 실제 외곽이 없습니다.')
    geometry = VehicleGeometry(converter.paper_size.tolist(), obstacles, target)
    if final_heading is None:
        heading = geometry.parking_heading(compact[-1], heading)
        route['final_heading_deg'] = heading
    geometry.path(compact, bundle['pose']['heading_deg'], heading)
    return route, geometry


def check_revision(bundle, expected, identity, capture_id):
    key = route_key(bundle['state'])
    # 새 테스트 시작은 별도 주행 허가가 필요하다. 같은 테스트의 경로 변경만 교체한다.
    if key[:3] != expected[:3]:
        raise RuntimeError('웹 지도·테스트·조건이 변경됐습니다. 주행기를 다시 실행하세요.')
    if key != expected:
        if type(key[3]) is not int or key[3] <= expected[3]:
            raise RuntimeError('동일하거나 오래된 route_revision에서 경로가 변경됐습니다.')
        raise RouteChanged('목적지·경로 변경: 제동 후 새 위치와 경로를 검증합니다.')
    validate_guidance(bundle['state'], bundle['occupancy'], identity, capture_id)


def braked_work(rover, work):
    """공간 검사 중에는 정지 상태로 poll을 유지한다. 다른 명령과 동시에 실행하지 않는다."""
    done, errors = threading.Event(), []
    def heartbeat():
        try:
            while not done.is_set():
                if rover.request('poll') != 0:
                    raise RuntimeError('경로 검사 중 MCU가 제동 상태가 아닙니다.')
                done.wait(0.04)
        except Exception as error:
            errors.append(error)
    thread = threading.Thread(target=heartbeat, daemon=True)
    thread.start()
    try:
        result = work()
    finally:
        done.set()
        thread.join(timeout=0.5)
    if thread.is_alive():
        raise TimeoutError('제동 상태 확인 응답이 끝나지 않았습니다.')
    if errors:
        raise RuntimeError(f'제동 상태 확인 실패: {errors[0]}') from errors[0]
    return result


def drive_web(feed, converter, semantic, rover, track_width, *, final_heading=None, expected_session=None, log=print):
    rover.request('stop')
    replacements = 0
    while True:
        pose = wait_settled(feed, rover)
        bundle = feed.current()
        session = route_key(bundle['state'])[:3]
        if expected_session is None:
            expected_session = session
        elif session != expected_session:
            raise RuntimeError('웹 테스트가 변경됐습니다. 주행기를 다시 실행하세요.')
        bundle['pose'] = pose
        route, geometry = braked_work(rover, lambda: make_plan(bundle, converter, semantic, final_heading))
        checked_pose = pose
        pose = wait_settled(feed, rover)
        if (math.hypot(pose['x_cm'] - checked_pose['x_cm'], pose['y_cm'] - checked_pose['y_cm']) > 0.5
                or abs((pose['heading_deg'] - checked_pose['heading_deg'] + 180) % 360 - 180) > 1):
            raise RuntimeError('공간 검사 중 제동 위치·방향이 바뀌었습니다. 다시 검사하세요.')
        nodes, heading = load_route(route, require_verified=True)
        require_start_position(pose, nodes[0])
        expected = route_key(bundle['state'])
        capture_id = bundle['state']['capture_id']

        def check(p):
            check_revision(feed.current(), expected, converter.identity, capture_id)
            geometry.check((p['x_cm'], p['y_cm']), p['heading_deg'])

        guard = PathGuard(extra=check)

        def step(action):
            p = feed.update()
            guard(p)
            geometry.action(p, action, CM_PER_PULSE, track_width)
            return execute(rover, feed, action, check=guard)

        try:
            follow(nodes, heading, track_width, step, pose, path_mode=True, check=guard, log=log)
            rover.request('stop')
            actual = feed.update()
            check_revision(feed.current(), expected, converter.identity, capture_id)
            if (math.hypot(actual['x_cm'] - nodes[-1][0], actual['y_cm'] - nodes[-1][1]) > GOAL_CM
                    or abs((actual['heading_deg'] - heading + 180) % 360 - 180) > 5):
                raise RuntimeError('최종 정지 후 위치·방향이 허용 오차를 벗어났습니다.')
            geometry.check((actual['x_cm'], actual['y_cm']), actual['heading_deg'], parked=True)
            log(f'모터 주행 완료: 목표 {GOAL_CM:g}cm 이내·최종 방향 확인, 제동 완료.', flush=True)
            return
        except RouteChanged as error:
            rover.request('stop')
            replacements += 1
            if replacements > 10:
                raise RuntimeError('경로 변경이 10회를 넘었습니다. 점유 상태를 확인하세요.') from error
            log(str(error), flush=True)


def default_repo():
    """저장소 이름이나 사용자별 Desktop 경로에 의존하지 않고 상위 저장소를 찾는다."""
    for candidate in Path(__file__).resolve().parents:
        if (candidate / 'src/rover_control/pose_sender.py').is_file():
            return candidate
    return Path(__file__).resolve().parent.parent / 'ai-valet-parking-rover'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', type=Path, default=default_repo())
    parser.add_argument('--server-url', default='http://127.0.0.1:8001')
    parser.add_argument('--capture-dir', type=Path)
    parser.add_argument('--track-width-cm', type=float)
    parser.add_argument('--final-heading-deg', type=float, help='생략하면 진입 방향과 가까운 주차 가능한 방향')
    connection = parser.add_mutually_exclusive_group()
    connection.add_argument('--rover-host', '--host', dest='rover_host')
    connection.add_argument('--usb', action='store_true')
    parser.add_argument('--rover-port', type=int, default=8765)
    parser.add_argument('--adb')
    parser.add_argument('--drive', action='store_true')
    args = parser.parse_args()
    if args.drive and (not (args.rover_host or args.usb) or args.track_width_cm is None):
        parser.error('--drive에는 --usb 또는 --rover-host와 실측 --track-width-cm이 필요합니다.')
    width = 12 if args.track_width_cm is None else number(args.track_width_cm)
    if width <= 0:
        parser.error('--track-width-cm은 양수여야 합니다.')
    repo = args.repo.resolve()
    if not (repo / 'src/rover_control/pose_sender.py').is_file():
        parser.error('--repo에 기존 웹 서버 저장소 경로를 지정하세요.')
    sys.path.insert(0, str(repo))
    from src.rover_control.connection import rover_endpoint
    from src.rover_control.pose_sender import CapturePose, load_camera_position

    folder = (args.capture_dir or repo / 'data/http_map_captures/latest').resolve()
    settings = load_camera_position(folder)
    converter = CapturePose(folder, camera_position_cm=settings['camera_position_cm'],
                            marker_height_cm=settings['marker_height_cm'])
    if (settings['capture_report_sha256'] != sha256(converter.report_bytes).hexdigest()
            or settings['calibration_sha256'] != sha256(converter.calibration_bytes).hexdigest()):
        raise ValueError('설정을 읽는 동안 지도·보정이 변경됐습니다. 다시 실행하세요.')
    semantic = json.loads((folder / 'semantic.json').read_text(encoding='utf-8'))
    if any(semantic.get(k) != converter.identity[k] for k in ('map_id', *IDENTITY_KEYS)):
        raise ValueError('의미 지도와 위치 보정이 다릅니다.')
    feed, rover = WebFeed(args.server_url, converter, folder / 'world/camera_position.json'), None
    try:
        bundle = feed.start()
        route, _ = make_plan(bundle, converter, semantic, args.final_heading_deg)
        p = wait_settled(feed, None, initial=True)
        require_start_position(p, route['waypoints'][0])
        print(f"목표={route['parking_space_id']} 경로버전={route['route_revision']} "
              f"최종방향={route['final_heading_deg']:.1f}° 차체·회전 공간 검사 통과", flush=True)
        if not args.drive:
            print('사전 검사 완료: 모터에 연결하지 않았습니다. 실물 시험 준비 후 --drive로 실행하세요.')
            return 0
        with rover_endpoint(args) as endpoint:
            rover = Rover(*endpoint)
            drive_web(feed, converter, semantic, rover, width, final_heading=args.final_heading_deg,
                      expected_session=route_key(bundle['state'])[:3])
    finally:
        try:
            if rover is not None:
                rover.close()
        finally:
            feed.close()
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('수동 중단: 정지 요청 완료.')
        raise SystemExit(130)
    except (OSError, ValueError, RuntimeError, KeyError, TypeError) as error:
        print(f'주행 중단: {error}', file=sys.stderr)
        raise SystemExit(1)
