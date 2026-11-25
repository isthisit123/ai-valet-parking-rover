"""UNO Q App Lab TCP receiver for the encoder job protocol."""

import json
import socket
import time

PORT = 8765
TIMEOUT = 5.0
RPC_TIMEOUT = 2.0


class MCUBridgeUnavailable(RuntimeError):
    pass


def mcu_call(call, name, *args):
    try:
        return call(name, *args, timeout=RPC_TIMEOUT)
    except Exception as error:
        raise MCUBridgeUnavailable(
            f"MCU Bridge {name}: {error}"
        ) from error


def serve(connection, call):
    data = bytearray()
    previous = 0
    deadline = time.monotonic() + TIMEOUT

    try:
        while True:
            while b"\n" not in data:
                remaining = deadline - time.monotonic()

                if remaining <= 0:
                    raise TimeoutError("client heartbeat expired")

                connection.settimeout(remaining)
                chunk = connection.recv(2048 - len(data))

                if not chunk:
                    return

                data.extend(chunk)

                if len(data) >= 2048:
                    raise ValueError("line too long")

            if time.monotonic() >= deadline:
                raise TimeoutError("client heartbeat expired")

            line, _, data = data.partition(b"\n")
            message = json.loads(line)

            seq = message["seq"]

            if type(seq) is not int or seq <= previous:
                raise ValueError("nonincreasing seq")

            previous = seq
            kind = message["type"]

            if kind == "step":
                mode = message["mode"]
                pulses = message["pulses"]

                if type(mode) is not int or type(pulses) is not int:
                    raise ValueError("integer mode and pulses required")

                if mode not in (1, 2, 3):
                    raise ValueError("invalid encoder mode")

                if not 1 <= pulses <= (8 if mode == 1 else 2):
                    raise ValueError("encoder job outside limits")

                result = mcu_call(
                    call, "encoder_step", mode, pulses
                )

            elif kind == "timed_step":
                mode = message["mode"]
                duration = message["duration_ms"]

                if (
                    type(mode) is not int
                    or mode not in (1, 2, 3)
                    or type(duration) is not int
                    or not 40 <= duration <= 200
                ):
                    raise ValueError(
                        "timed job requires mode 1..3 "
                        "and duration_ms 40..200"
                    )

                result = mcu_call(
                    call, "timed_step", mode, duration
                )

            elif kind == "poll":
                result = mcu_call(call, "encoder_poll")

            elif kind == "stop":
                result = mcu_call(call, "encoder_stop")

            else:
                raise ValueError("unknown message")

            connection.settimeout(RPC_TIMEOUT)

            reply = {
                "seq": seq,
                "state": result,
            }

            connection.sendall(
                json.dumps(reply).encode() + b"\n"
            )

            deadline = time.monotonic() + TIMEOUT

            if result < 0:
                return

    except MCUBridgeUnavailable as error:
        try:
            connection.settimeout(RPC_TIMEOUT)

            reply = {
                "seq": previous,
                "error": str(error),
            }

            connection.sendall(
                json.dumps(reply).encode() + b"\n"
            )
        except OSError:
            pass

        raise

    finally:
        try:
            call("encoder_stop", timeout=RPC_TIMEOUT)
        except (OSError, RuntimeError) as error:
            print(
                f"MCU stop reply unavailable: {error}",
                flush=True,
            )


def main():
    from arduino.app_utils import Bridge

    for _ in range(40):
        try:
            stopped = Bridge.call(
                "encoder_stop",
                timeout=0.25,
            )
            break
        except (OSError, ValueError, RuntimeError):
            time.sleep(0.25)
    else:
        raise RuntimeError("MCU Bridge unavailable")

    if type(stopped) is not int or stopped != 0:
        raise RuntimeError(
            f"MCU stop not ready: {stopped}; "
            "repair fault and restart App Lab"
        )

    with socket.create_server(("0.0.0.0", PORT)) as server:
        print(
            f"Q-ROVER TCP {PORT} | MCU stopped | step / timed_step",
            flush=True,
        )

        while True:
            connection, peer = server.accept()

            with connection:
                connection.setsockopt(
                    socket.IPPROTO_TCP,
                    socket.TCP_NODELAY,
                    1,
                )

                try:
                    serve(connection, Bridge.call)
                except Exception as error:
                    print(
                        f"Stopped {peer}: {error}",
                        flush=True,
                    )


if __name__ == "__main__":
    main()