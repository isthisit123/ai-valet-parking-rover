"""양쪽 바퀴를 띄우고 정지를 확인한 뒤 전진 한 펄스를 시험한다."""
import argparse
from contextlib import closing
import time

from src.rover_control.connection import add_connection_arguments, rover_endpoint
from .node_follower import Rover


def forward_once(rover):
    if rover.request('stop') != 0:
        raise RuntimeError('MCU 정지 확인 실패')
    # MCU의 제동 대기 시간(200ms) 동안 수신기 heartbeat를 유지한다.
    ready_at = time.monotonic() + 0.25
    while time.monotonic() < ready_at:
        if rover.request('poll') != 0:
            raise RuntimeError('출발 대기 중 MCU 상태 변경')
        time.sleep(0.04)

    deadline = time.monotonic() + 0.5
    if rover.request('step', mode=1, pulses=1) != 1:
        raise RuntimeError('전진 명령 거부')
    while time.monotonic() < deadline:
        state = rover.request('poll')
        if state == 0:
            return
        if state != 1:
            raise RuntimeError(f'예상하지 못한 MCU 상태 {state}')
        time.sleep(0.04)
    raise TimeoutError('0.5초 안에 양쪽 엔코더 목표에 도달하지 못했습니다.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    add_connection_arguments(parser)
    args = parser.parse_args()
    answer = input("두 바퀴를 띄웠고 Run 후 양쪽 모두 정지해 있으면 '전진' 입력: ")
    if answer.strip() != '전진':
        print('시험 취소')
        return
    with rover_endpoint(args) as endpoint, closing(Rover(*endpoint)) as rover:
        forward_once(rover)
    print('MCU 전진 작업 완료. 양쪽 바퀴의 회전 방향과 실제 정지를 확인하세요.')


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, RuntimeError) as error:
        raise SystemExit(f'시험 중단: {error}')
