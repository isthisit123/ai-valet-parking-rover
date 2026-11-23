"""UNO Q App Lab TCP receiver for the encoder job protocol."""
import json
import socket
import time

PORT = 8765
TIMEOUT = 0.25
RPC_TIMEOUT = 0.08


def serve(connection, call):
    data = bytearray()
    previous = 0
    deadline = time.monotonic() + TIMEOUT
    try:
        while True:
            while b'\n' not in data:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError('client heartbeat expired')
                connection.settimeout(remaining)
                chunk = connection.recv(2048 - len(data))
                if not chunk:
                    return
                data.extend(chunk)
                if len(data) >= 2048:
                    raise ValueError('line too long')
            if time.monotonic() >= deadline:
                raise TimeoutError('client heartbeat expired')
            line, _, data = data.partition(b'\n')
            message = json.loads(line)
            seq = message['seq']
            if type(seq) is not int or seq <= previous:
                raise ValueError('nonincreasing seq')
            previous = seq
            kind = message['type']
            if kind == 'step':
                mode, pulses = message['mode'], message['pulses']
                if type(mode) is not int or type(pulses) is not int:
                    raise ValueError('integer mode and pulses required')
                if mode not in (1, 2, 3) or not 1 <= pulses <= (8 if mode == 1 else 2):
                    raise ValueError('encoder job outside limits')
                result = call('encoder_step', mode, pulses, timeout=RPC_TIMEOUT)
            elif kind == 'poll':
                result = call('encoder_poll', timeout=RPC_TIMEOUT)
            elif kind == 'stop':
                result = call('encoder_stop', timeout=RPC_TIMEOUT)
            else:
                raise ValueError('unknown message')
            connection.settimeout(RPC_TIMEOUT)
            connection.sendall(json.dumps({'seq': seq, 'state': result}).encode()+b'\n')
            deadline = time.monotonic() + TIMEOUT
            if result < 0:
                return
    finally:
        call('encoder_stop', timeout=RPC_TIMEOUT)


def main():
    from arduino.app_utils import Bridge
    for _ in range(40):
        try:
            stopped = Bridge.call('encoder_stop', timeout=0.25)
            break
        except (OSError, ValueError, RuntimeError):
            time.sleep(0.25)
    else:
        raise RuntimeError('MCU Bridge unavailable')
    if type(stopped) is not int or stopped != 0:
        raise RuntimeError(f'MCU stop not ready: {stopped}; repair fault and restart App Lab')
    with socket.create_server(('0.0.0.0', PORT)) as server:
        print(f'Q-ROVER ENCODER TCP {PORT} | MCU stopped', flush=True)
        while True:
            connection, peer = server.accept()
            with connection:
                connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                try:
                    serve(connection, Bridge.call)
                except Exception as error:
                    print(f'Stopped {peer}: {error}', flush=True)


if __name__ == '__main__':
    main()
