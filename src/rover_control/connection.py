from contextlib import contextmanager
import subprocess
import sys


def add_connection_arguments(parser):
    connection = parser.add_mutually_exclusive_group(required=True)
    connection.add_argument("--rover-host", help="Q-ROVER TCP 서버 IP 또는 호스트명")
    connection.add_argument("--usb", action="store_true", help="UNO Q USB-C 케이블로 연결")
    parser.add_argument("--rover-port", type=int, default=8765, help="UNO Q의 TCP 서버 포트")
    parser.add_argument("--adb", help="ADB 실행 파일 경로 (--usb 전용, 기본값 PATH의 adb)")


def adb_forward(adb, *args):
    try:
        result = subprocess.run(
            [adb, "-d", "forward", *args],
            capture_output=True, text=True, errors="replace", timeout=8,
        )
    except FileNotFoundError as error:
        raise OSError("adb를 찾을 수 없습니다. --adb로 adb.exe 경로를 지정하세요.") from error
    except subprocess.TimeoutExpired as error:
        raise TimeoutError("ADB USB 연결 시간 초과. 데이터 케이블과 adb -d get-state를 확인하세요.") from error
    if result.returncode:
        detail = result.stderr.strip() or result.stdout.strip()
        raise OSError(f"ADB USB 포트 전달 실패. {detail}")
    return result.stdout.strip()


@contextmanager
def rover_endpoint(args):
    if not 0 < args.rover_port <= 65535:
        raise ValueError("로버 포트는 1~65535로 지정하세요.")
    if not args.usb:
        if args.adb is not None:
            raise ValueError("--adb는 --usb와 함께 사용하세요.")
        yield args.rover_host, args.rover_port
        return

    adb = args.adb or "adb"
    # ADB가 빈 로컬 포트를 고르게 해 기존 포트 전달을 덮어쓰지 않는다.
    port = int(adb_forward(adb, "tcp:0", f"tcp:{args.rover_port}"))
    try:
        yield "127.0.0.1", port
    finally:
        try:
            adb_forward(adb, "--remove", f"tcp:{port}")
        except OSError as error:
            print(f"USB 포트 정리 실패. {error}", file=sys.stderr)
