import json
from math import isfinite
import socket
import time


PORT = 8765
COMMAND_TIMEOUT_S = 0.30
BRIDGE_TIMEOUT_S = 0.055
MAX_LINE = 4096
OUTPUT_MODES = {0: "dry_run", 1: "motor_enabled", 2: "bench"}
STOP_REASONS = {
    2: "motor_output_disabled",
    3: "command_timeout",
    4: "tof_obstacle",
    5: "tof_unavailable",
    6: "stop_latched",
    7: "invalid_command",
    8: "output_mode_mismatch",
}


def speed_milli(value, minimum, maximum):
    if type(value) not in (int, float) or not minimum <= value <= maximum or not isfinite(value):
        raise ValueError("speed outside supported range")
    return round(value * 1000)


def serve_client(connection, bridge_call):
    buffer = bytearray()
    previous_seq = 0
    stop_needed = False
    deadline = time.monotonic() + COMMAND_TIMEOUT_S
    try:
        while True:
            while b"\n" not in buffer:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("command timeout")
                connection.settimeout(remaining)
                chunk = connection.recv(MAX_LINE - len(buffer))
                if not chunk:
                    return
                buffer.extend(chunk)
                if len(buffer) == MAX_LINE and b"\n" not in buffer:
                    raise ValueError("command line too long")
            if time.monotonic() >= deadline:
                raise TimeoutError("command timeout")
            line, _, buffer = buffer.partition(b"\n")
            message = json.loads(line)
            if not isinstance(message, dict):
                raise ValueError("command must be a JSON object")
            seq = message.get("seq")
            if type(seq) is not int or seq <= previous_seq:
                raise ValueError("seq must strictly increase")
            previous_seq = seq
            deadline = time.monotonic() + COMMAND_TIMEOUT_S
            kind = message.get("type")
            reply = {"type": "status", "applied_seq": seq}
            log = None
            if kind == "ping":
                if stop_needed:
                    raise ValueError("ping is only available while stopped")
                mode = bridge_call("rover_mode", timeout=BRIDGE_TIMEOUT_S)
                if mode not in OUTPUT_MODES:
                    raise ValueError("unexpected rover_mode reply")
                reply.update(state="ready", mode=OUTPUT_MODES[mode])
            elif kind in ("drive", "bench"):
                if kind == "drive":
                    linear = speed_milli(message.get("linear_cm_s"), 0, 8)
                    angular = speed_milli(message.get("angular_deg_s"), -45, 45)
                    method, args = "rover_drive", (linear, angular)
                    detail = f"drive v={linear / 1000:g} w={angular / 1000:g}"
                else:
                    method, args = "rover_bench", ()
                    detail = "bench"
                stop_needed = True
                result = bridge_call(method, *args, timeout=BRIDGE_TIMEOUT_S)
                if result == 1:
                    reply.update(state="moving")
                elif result in STOP_REASONS:
                    reply.update(state="stopped", stop_reason=STOP_REASONS[result])
                    stop_needed = False
                else:
                    raise ValueError(f"unexpected {method} reply")
                log = f"RX seq={seq} {detail} MCU={result}"
            elif kind == "stop":
                if bridge_call("rover_stop", timeout=BRIDGE_TIMEOUT_S) != 0:
                    raise ValueError("MCU did not confirm stop")
                stop_needed = False
                reply.update(state="stopped")
                log = f"RX seq={seq} stop MCU=0"
            else:
                raise ValueError("unknown command")
            connection.settimeout(BRIDGE_TIMEOUT_S)
            connection.sendall(json.dumps(reply).encode() + b"\n")
            if log:
                print(log, flush=True)
            if kind == "stop" or reply.get("stop_reason") not in (None, "motor_output_disabled"):
                return
    finally:
        if stop_needed:
            try:
                bridge_call("rover_stop", timeout=BRIDGE_TIMEOUT_S)
            except Exception as error:
                print(f"MCU stop RPC failed: {error}", flush=True)


def main():
    from arduino.app_utils import Bridge

    # Run 시 스케치가 RPC를 등록할 때까지 기다린 뒤 포트를 연다.
    for _ in range(30):
        try:
            mode = Bridge.call("rover_mode", timeout=0.25)
            break
        except (OSError, ValueError, RuntimeError):
            time.sleep(0.25)
    else:
        raise RuntimeError("MCU Bridge unavailable. Check App launch and sketch build.")
    if mode not in OUTPUT_MODES:
        raise RuntimeError("unexpected rover_mode reply")

    with socket.create_server(("0.0.0.0", PORT)) as listener:
        print(f"LISTEN TCP {PORT} mode={OUTPUT_MODES[mode]}", flush=True)
        while True:
            connection, peer = listener.accept()
            with connection:
                connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                print(f"CONNECTED {peer[0]}", flush=True)
                try:
                    serve_client(connection, Bridge.call)
                except (OSError, ValueError, RuntimeError) as error:
                    print(f"SESSION CLOSED: {error}", flush=True)


if __name__ == "__main__":
    main()
