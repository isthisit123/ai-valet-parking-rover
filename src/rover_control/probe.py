import argparse
import socket
import sys
import time

from .connection import add_connection_arguments, rover_endpoint
from .drive import RoverLink
from .tracking import CONTROL_PERIOD_S


PROBE_ACK_TIMEOUT_S = 0.25


def probe(rover, *, bench_drive=False):
    status = rover.exchange("ping")
    print(status, flush=True)
    mode = status.get("mode")
    if status.get("state") != "ready":
        raise ValueError("로버의 모드 응답이 올바르지 않습니다.")
    expected_mode = "bench" if bench_drive else "dry_run"
    if mode != expected_mode:
        raise ValueError(f"로버 모드가 {mode!r}입니다. 이 시험에는 {expected_mode!r} 모드가 필요합니다.")

    stop_confirmed = False
    try:
        if bench_drive:
            started = time.monotonic()
            status = rover.exchange("bench")
            print(status, flush=True)
            if status.get("state") != "moving":
                raise ValueError(f"MCU가 바퀴 시험을 거부했습니다. {status.get('stop_reason')}")
            time.sleep(max(0, 0.22 - (time.monotonic() - started)))
        else:
            for _ in range(5):
                started = time.monotonic()
                status = rover.exchange("drive", linear_cm_s=2, angular_deg_s=0)
                print(status, flush=True)
                if (status.get("state"), status.get("stop_reason")) != ("stopped", "motor_output_disabled"):
                    raise ValueError("모터 출력이 꺼진 상태의 MCU 응답이 필요합니다.")
                time.sleep(max(0, CONTROL_PERIOD_S - (time.monotonic() - started)))
        status = rover.exchange("stop", reason="probe_complete")
        print(status, flush=True)
        stop_confirmed = status.get("state") == "stopped"
        if not stop_confirmed:
            raise ValueError("정지 명령의 적용을 확인하지 못했습니다.")
    finally:
        if not stop_confirmed:
            try:
                rover.send("stop", reason="probe_exit")
            except OSError:
                pass
    result = "바퀴 시험 명령" if bench_drive else "속도 명령 5개"
    print(f"통신 성공: {result} 수신과 정지 응답을 확인했습니다.", flush=True)


def probe_endpoint(endpoint, *, bench_drive):
    try:
        sock = socket.create_connection(endpoint, timeout=2)
    except TimeoutError as error:
        raise TimeoutError("TCP 연결 시간 초과. 연결 설정과 App Lab의 LISTEN 로그를 확인하세요.") from error
    with sock:
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        print(f"TCP 연결됨: {endpoint[0]}:{endpoint[1]}", flush=True)
        try:
            probe(RoverLink(sock, ack_timeout_s=PROBE_ACK_TIMEOUT_S), bench_drive=bench_drive)
        except TimeoutError as error:
            raise TimeoutError("로버 응답 시간 초과. App Lab Python 콘솔의 RX 및 SESSION CLOSED 로그를 확인하세요.") from error


def main():
    parser = argparse.ArgumentParser(description="카메라 없이 UNO Q 명령 전송과 응답을 확인합니다.")
    add_connection_arguments(parser)
    parser.add_argument("--bench-drive", action="store_true", help="바퀴를 띄운 상태에서 0.2초 단발 시험")
    args = parser.parse_args()
    try:
        with rover_endpoint(args) as endpoint:
            probe_endpoint(endpoint, bench_drive=args.bench_drive)
    except (OSError, ValueError) as error:
        print(f"통신 시험 중단: {error}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
