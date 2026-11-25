"""ParkingHaeven의 목적지·위치·점유 API를 짧은 시간 기반 모터 명령에 연결한다.

기본 실행은 모터에 연결하지 않는 사전 검사다. --drive가 있어야 TCP를 연결한다.
"""
import argparse
from contextlib import ExitStack
from copy import deepcopy
from datetime import datetime, timezone
from hashlib import sha256
from http.client import RemoteDisconnected
import json
import math
from pathlib import Path
import sys
import threading
import time
from urllib.parse import urlsplit
from urllib.error import HTTPError, URLError
from urllib.request import build_opener, ProxyHandler

from node_follower import DEFAULT_TRACK_WIDTH_CM, IDENTITY_KEYS, MCUFault, ROVER_REPLY_TIMEOUT_S, Rover, number
from web_navigation import ARRIVAL_RADIUS_CM, RoadMap, WebPath, position

WEB_TTL_S = 0.20
STARTUP_WAIT_S = 10.0
STARTUP_HTTP_TIMEOUT_S = 0.50
CONTROL_HTTP_TIMEOUT_S = 0.12
MCU_BRAKE_S = 0.20
POLL_PERIOD_S = 0.04
MCU_POLL_S = 0.10


class RouteChanged(RuntimeError):
    pass


class WebTransportUnavailable(ConnectionError):
    """응답 단절·소켓 타임아웃. 시작 대기에서만 재시도할 수 있는 통신 실패."""


def route_key(state):
    return (state.get('capture_id'), state.get('started_at'), state.get('case_id'),
            state.get('route_revision'), state.get('parking_space_id'))


def waiting_for_marker(state):
    return (state.get('tracking_available') is False and state.get('guidance_available') is False
            and state.get('status') in ('waiting_for_tracking', 'holding', 'running'))


def validate_guidance(state, occupancy, identity, capture_id, *, waiting=False):
    tracking_wait = waiting and state.get('status') in ('running', 'holding', 'waiting_for_tracking')
    if not tracking_wait and (state.get('status') not in ('running', 'holding', 'success')
                              or state.get('guidance_available') is not True):
        target = state.get('parking_space_id')
        raise RuntimeError(f"웹 경로 안내가 중단됐습니다. 상태={state.get('status')}, "
                           f"사유={state.get('reason')}, 목표={target}, "
                           f"점유={occupancy.get('states', {}).get(target, 'unknown')}")
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
        self.startup_complete, self.loss_count = False, 0
        self.pending_reason = '새 ArUco 관측 대기'
        self.thread = threading.Thread(target=self._run, daemon=True)

    def read(self, path):
        timeout = CONTROL_HTTP_TIMEOUT_S if self.startup_complete else STARTUP_HTTP_TIMEOUT_S
        try:
            with self.opener.open(self.url + path, timeout=timeout) as response:
                body = response.read(2_000_001)
        except HTTPError:
            # 잘못된 API·지도 요청 등을 일시적인 관측 부족으로 숨기지 않는다.
            raise
        except (URLError, TimeoutError, ConnectionError, RemoteDisconnected) as error:
            raise WebTransportUnavailable(f'{path} 응답 대기: {type(error).__name__}: {error}') from error
        if len(body) > 2_000_000:
            raise ValueError('웹 응답이 너무 큽니다.')
        result = json.loads(body)
        if not isinstance(result, dict):
            raise ValueError('웹 API는 JSON 객체를 반환해야 합니다.')
        return result

    def _run(self):
        try:
            from src.rover_control.pose_sender import PoseUnavailable

            while not self.done.is_set():
                started = time.monotonic()
                self.converter.require_current()
                if self.settings_path.read_bytes() != self.settings_bytes:
                    raise ValueError('카메라 실측값이 변경됐습니다. 다시 실행하세요.')
                try:
                    occupancy = self.read('/api/parking-occupancy')
                    observation = self.read('/api/rover-pose')
                except WebTransportUnavailable as error:
                    with self.lock:
                        if self.startup_complete:
                            raise
                        self.latest = self.last_pose = None
                        self.pending_reason = str(error)
                    self.ready.set()
                    self.done.wait(0.05)
                    continue
                try:
                    pose = self.converter.convert(observation)
                except PoseUnavailable as error:
                    if self.last_pose is not None:
                        self.loss_count += 1
                    self.last_pose = None
                    self.pending_reason = str(error)
                else:
                    if pose is not None:
                        self.last_pose = pose
                with self.lock:
                    self.latest = {'pose': self.last_pose, 'state': observation.get('arrival_test', {}),
                                   'occupancy': occupancy, 'requested_at': started, 'loss_count': self.loss_count,
                                   'client_captured_at': observation.get('client_captured_at')}
                self.ready.set()
                self.done.wait(max(0, 0.02 - (time.monotonic() - started)))
        except Exception as error:
            # 지도·카메라 오류와 주행 중 통신 오류는 종료 상태로 유지한다.
            with self.lock:
                self.error = error
            self.ready.set()

    def start(self, timeout=STARTUP_WAIT_S):
        timeout = number(timeout)
        if timeout <= 0:
            raise ValueError('출발 관측 대기 시간은 양수여야 합니다.')
        deadline = time.monotonic() + timeout
        self.thread.start()
        while time.monotonic() < deadline:
            self.ready.wait(min(0.05, max(0, deadline - time.monotonic())))
            self.ready.clear()
            result = self.current()
            if result and result['pose'] is not None and not waiting_for_marker(result['state']):
                with self.lock:
                    if self.latest is None or self.latest['requested_at'] != result['requested_at']:
                        continue
                    self.startup_complete = True
                return result
        self.done.set()
        raise TimeoutError(f'{timeout:g}초 안에 새 ArUco 위치를 받지 못했습니다. 마지막 상태: {self.pending_reason}')

    def current(self):
        with self.lock:
            if self.error is not None:
                raise RuntimeError(f'웹 위치·점유 연결 중단: {self.error}') from self.error
            result = deepcopy(self.latest)
        if result is None:
            return None
        now = time.monotonic()
        pose = result['pose']
        result['pose_age_ms'] = (now - pose['captured_monotonic_s']) * 1000 if pose else None
        result['pose_reason'] = self.pending_reason if pose is None else '새 관측 대기'
        if (not 0 <= now - result['requested_at'] <= WEB_TTL_S or pose is None
                or not 0 <= now - pose['captured_monotonic_s'] <= WEB_TTL_S):
            result['pose'] = None
            if pose is not None:
                result['pose_reason'] = f"위치 또는 HTTP 갱신 만료 (관측 나이 {result['pose_age_ms']:.0f}ms)"
        if result.get('client_captured_at'):
            captured = datetime.fromisoformat(result['client_captured_at'].replace('Z', '+00:00'))
            if not -0.05 <= (datetime.now(timezone.utc) - captured).total_seconds() <= WEB_TTL_S:
                result['pose'] = None
                result['pose_reason'] = '카메라 촬영 시각 200ms 만료'
        # 조회 이후 경과한 시간도 점유 TTL에 포함한다. 반복 조회로 free 수명을 늘리지 않는다.
        if result['occupancy'].get('age_seconds') is not None:
            result['occupancy']['age_seconds'] = number(result['occupancy']['age_seconds']) + now - result['requested_at']
        return result

    def close(self):
        self.done.set()
        if self.thread.ident is not None:
            self.thread.join(timeout=0.5)


def make_plan(bundle, roads, radius):
    goal = roads.parking_center(bundle['state']['parking_space_id'])
    return WebPath(roads, position(bundle['pose']), goal, radius)


def check_revision(bundle, expected, identity, capture_id):
    key = route_key(bundle['state'])
    if key[:3] != expected[:3]:
        raise RuntimeError('웹 지도·테스트·조건이 변경됐습니다. 주행기를 다시 실행하세요.')
    if key != expected and (type(key[3]) is not int or key[3] <= expected[3]):
        raise RuntimeError('동일하거나 오래된 route_revision에서 경로가 변경됐습니다.')
    validate_guidance(bundle['state'], bundle['occupancy'], identity, capture_id,
                      waiting=bundle['pose'] is None or waiting_for_marker(bundle['state']))
    if key != expected:
        raise RouteChanged('목적지·경로 변경: 제동 후 새 위치와 경로를 검증합니다.')


def braked_work(rover, work):
    """준비 작업 중에는 정지 상태로 poll을 유지한다. 다른 명령과 동시에 실행하지 않는다."""
    done, errors = threading.Event(), []
    def heartbeat():
        try:
            while not done.is_set():
                if rover.request('poll') != 0:
                    raise RuntimeError('경로 검사 중 MCU가 제동 상태가 아닙니다.')
                done.wait(MCU_POLL_S)
        except Exception as error:
            errors.append(error)
    thread = threading.Thread(target=heartbeat, daemon=True)
    thread.start()
    try:
        result = work()
    finally:
        done.set()
        thread.join(timeout=ROVER_REPLY_TIMEOUT_S + 0.2)
    if thread.is_alive():
        raise TimeoutError('제동 상태 확인 응답이 끝나지 않았습니다.')
    if errors:
        raise RuntimeError(f'제동 상태 확인 실패: {errors[0]}') from errors[0]
    return result


def brake(rover):
    if rover.request('stop') != 0:
        raise RuntimeError('MCU 정지 응답을 확인할 수 없습니다.')


def drive_web(feed, rover, roads, bundle, path, width, identity, *, encoder_drive=False, log=print):
    """짧은 동작 사이에 새 관측을 받고, 동작 중에도 중단·도착을 감시한다."""
    expected = route_key(bundle['state'])
    capture_id = expected[0]
    active = paused = rebuild = False
    last_command_frame = motion_ended = -math.inf
    loss_count = bundle['loss_count']
    deadline = math.inf
    last_wait_log = last_action_log = -math.inf
    commands = 0
    next_poll = -math.inf
    try:
        while True:
            # 명령 중에도 웹 Stop·점유·관측 유실·도착을 확인한다.
            bundle = feed.current()
            try:
                check_revision(bundle, expected, identity, capture_id)
            except RouteChanged:
                expected = route_key(bundle['state'])
                brake(rover)
                active, rebuild = False, True
                motion_ended = time.monotonic()
                log('웹 경로 변경: 제동 후 새 목적지로 경로를 계산합니다.', flush=True)

            pose = bundle['pose']
            lost = pose is None or bundle['state'].get('guidance_available') is not True
            if lost or bundle['loss_count'] != loss_count:
                if not paused:
                    brake(rover)
                    motion_ended = time.monotonic()
                if time.monotonic() - last_wait_log >= 2:
                    reason = bundle.get('pose_reason', '새 관측 대기')
                    log(f'마커 관측 대기: {reason}. 제동 유지.', flush=True)
                    last_wait_log = time.monotonic()
                active, paused = False, True
                loss_count = bundle['loss_count']
            if not lost:
                paused = False
                if rebuild:
                    path = braked_work(rover, lambda: make_plan(bundle, roads, path.radius))
                    rebuild = False
                    if path.arrived(position(pose)):
                        break
                    continue
                if path.arrived(position(pose)):
                    break

            stamp = pose['captured_monotonic_s'] if not lost else -math.inf
            if (not active and stamp > max(last_command_frame, motion_ended)
                    and time.monotonic() - motion_ended >= MCU_BRAKE_S):
                action = path.encoder_action(pose, width) if encoder_drive else path.action(pose, width)
                # 그래프 재탐색 사이 도착한 새 관측·중단·만료를 먼저 반영한다.
                latest = feed.current()
                if (latest['pose'] is None or latest['pose']['captured_monotonic_s'] != stamp
                        or latest['loss_count'] != loss_count or route_key(latest['state']) != expected
                        or latest['state'].get('guidance_available') is not True):
                    continue
                validate_guidance(latest['state'], latest['occupancy'], identity, capture_id)
                sent_at = time.monotonic()
                age = max(0, (sent_at - stamp) * 1000)
                if age > WEB_TTL_S * 1000:
                    continue
                if encoder_drive:
                    accepted = rover.request('step', mode=action[0], pulses=action[1])
                    amount = f'{action[1]}펄스'
                    execution_timeout = 7.0
                else:
                    accepted = rover.request('timed_step', mode=action[0], duration_ms=action[1])
                    amount = f'{action[1]}ms'
                    execution_timeout = action[1] / 1000 + 0.5
                if accepted != 1:
                    raise RuntimeError('MCU가 모터 작업을 거부했습니다.')
                reply_ms = (time.monotonic() - sent_at) * 1000
                commands += 1
                if time.monotonic() - last_action_log >= 1:
                    direction = {1: '전진', 2: '시계 회전', 3: '반시계 회전'}[action[0]]
                    log(f"구동 #{commands}: {direction} {amount}, "
                        f"위치=({pose['x_cm']:.2f}, {pose['y_cm']:.2f})cm "
                        f"방향={pose['heading_deg']:.1f}°, 전송 전 관측 나이={age:.0f}ms, "
                        f"보드 응답={reply_ms:.0f}ms", flush=True)
                    last_action_log = time.monotonic()
                active, last_command_frame = True, stamp
                deadline = sent_at + execution_timeout
            elif time.monotonic() >= next_poll:
                state = rover.request('poll')
                next_poll = time.monotonic() + MCU_POLL_S
                if active and state == 0:
                    active = False
                    motion_ended = time.monotonic()
                elif active and (state != 1 or time.monotonic() >= deadline):
                    raise RuntimeError('MCU 모터 작업 응답 또는 완료 시간 오류입니다.')
                elif not active and state != 0:
                    raise RuntimeError('MCU가 제동 상태가 아닙니다.')
            time.sleep(POLL_PERIOD_S)
    except MCUFault:
        # 수신기가 이미 고장 제동 후 연결을 종료했다.
        raise
    except BaseException:
        try:
            brake(rover)
        except (OSError, ValueError, RuntimeError, KeyError) as stop_error:
            print(f'중단 후 제동 응답 확인 실패: {stop_error}', file=sys.stderr)
        raise
    brake(rover)
    log(f'모터 주행 완료: 마커 중심이 목표 {path.radius:g}cm 이내, 제동 완료.', flush=True)


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
    parser.add_argument('--startup-wait-s', type=float, default=STARTUP_WAIT_S,
                        help='첫 ArUco 관측을 기다릴 최대 시간 (기본 10초)')
    parser.add_argument('--track-width-cm', type=float)
    parser.add_argument('--reference-cm', type=float, nargs=2, metavar=('X', 'Y'),
                        help='전지 왼쪽 위 기준 실측 마커 중심과 관측값 차이 표시 (사전 검사)')
    parser.add_argument('--arrival-radius-cm', type=float, default=ARRIVAL_RADIUS_CM, help='마커 중심의 도착 반경 (cm)')
    connection = parser.add_mutually_exclusive_group()
    connection.add_argument('--rover-host', '--host', dest='rover_host')
    connection.add_argument('--usb', action='store_true')
    parser.add_argument('--rover-port', type=int, default=8765)
    parser.add_argument('--adb')
    parser.add_argument('--drive', action='store_true')
    parser.add_argument('--encoder-drive', action='store_true',
                        help='기존 ENCODER 수신 앱에 step 명령을 보내 한 펄스씩 주행 (양쪽 엔코더 필요)')
    args = parser.parse_args()
    if args.reference_cm and (args.drive or not all(math.isfinite(v) for v in args.reference_cm)):
        parser.error('--reference-cm은 사전 검사에서 유한한 실측값 두 개로 사용하세요.')
    if args.drive and (not (args.rover_host or args.usb) or args.track_width_cm is None):
        parser.error('--drive에는 --usb 또는 --rover-host와 실측 --track-width-cm이 필요합니다.')
    width = DEFAULT_TRACK_WIDTH_CM if args.track_width_cm is None else number(args.track_width_cm)
    if number(args.arrival_radius_cm) <= 0:
        parser.error('--arrival-radius-cm은 양수여야 합니다.')
    if width <= 0:
        parser.error('--track-width-cm은 양수여야 합니다.')
    if number(args.startup_wait_s) <= 0:
        parser.error('--startup-wait-s는 유한한 양수여야 합니다.')
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
    roads = RoadMap(semantic, converter)
    capture_id = json.loads(converter.report_bytes)['capture_id']
    feed = WebFeed(args.server_url, converter, folder / 'world/camera_position.json')
    with ExitStack() as stack:
        stack.callback(feed.close)
        rover = None
        if args.drive:
            endpoint = stack.enter_context(rover_endpoint(args))
            print(f'보드 {endpoint[0]}:{endpoint[1]} 연결·첫 정지 확인 중', flush=True)
            rover = Rover(*endpoint)
            stack.callback(rover.close)
            brake(rover)
            # MCU 제동 대기와 그래프 준비를 첫 출발 관측보다 먼저 끝낸다.
            braked_work(rover, lambda: time.sleep(MCU_BRAKE_S))
        print(f'현재 웹 테스트의 ArUco 위치 대기 (최대 {args.startup_wait_s:g}초)', flush=True)
        def prepare():
            bundle = feed.start(timeout=args.startup_wait_s)
            validate_guidance(bundle['state'], bundle['occupancy'], converter.identity, capture_id)
            return bundle, make_plan(bundle, roads, args.arrival_radius_cm)
        bundle, path = braked_work(rover, prepare) if rover else prepare()
        pose = bundle['pose']
        mode = '엔코더 step, 전진 1~5펄스·회전 1~2펄스' if args.encoder_drive else '시간 기반 timed_step'
        print(f'주행 모드: {mode}', flush=True)
        print(f"ArUco 출발=({pose['x_cm']:.2f}, {pose['y_cm']:.2f})cm 전방={pose['heading_deg']:.1f}° "
              f"목적지={bundle['state']['parking_space_id']} 중심=({path.goal[0]:.2f}, {path.goal[1]:.2f})cm "
              f"경로점={len(path.points)} 도착 반경={path.radius:g}cm", flush=True)
        if args.reference_cm:
            x, y = args.reference_cm
            print(f"관측 − 실측: Δx={pose['x_cm'] - x:+.2f}cm, Δy={pose['y_cm'] - y:+.2f}cm", flush=True)
        if rover:
            if path.arrived(position(pose)):
                brake(rover)
                print('모터 주행 완료: 첫 관측이 도착 반경 안에 있어 제동했습니다.')
            else:
                drive_web(feed, rover, roads, bundle, path, width, converter.identity,
                          encoder_drive=args.encoder_drive)
        else:
            first = path.encoder_action(pose, width) if args.encoder_drive else path.action(pose, width)
            unit = 'pulses' if args.encoder_drive else 'duration_ms'
            print(f'사전 검사 완료: 첫 명령(mode, {unit})={first or "정지"}. 모터 연결 없음.')
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
