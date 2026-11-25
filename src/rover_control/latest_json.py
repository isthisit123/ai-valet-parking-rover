"""실시간 응답을 막지 않고 최신 진단 JSON 한 개만 별도 스레드에서 기록한다."""

from copy import deepcopy
import json
import logging
from pathlib import Path
from tempfile import NamedTemporaryFile
import threading
import time


LOGGER = logging.getLogger(__name__)
RETRY_DELAYS_S = (0.0, 0.01, 0.02)


class LatestJsonWriter:
    def __init__(self, path):
        self.path = Path(path)
        self._condition = threading.Condition()
        self._thread = None
        self._pending = None
        self._last_submitted = None
        self._closed = False
        self._writing = False
        self._submitted = self._written = self._coalesced = 0
        self._last_write_ms = None
        self._last_error = None

    def submit(self, payload):
        # 호출자가 상태를 갱신해도 기록 중인 관측 시각·지도·좌표는 바뀌지 않는다.
        snapshot = deepcopy(payload)
        with self._condition:
            if self._closed:
                raise RuntimeError('종료된 진단 JSON 기록기에 요청했습니다.')
            if snapshot == self._last_submitted and self._last_error is None:
                return False
            self._submitted += 1
            if self._pending is not None:
                self._coalesced += 1
            self._pending = (self._submitted, snapshot)
            self._last_submitted = snapshot
            if self._thread is None:
                self._thread = threading.Thread(target=self._run, name=f'json-{self.path.parent.name}', daemon=True)
                self._thread.start()
            self._condition.notify_all()
            return True

    def status(self):
        with self._condition:
            return {'mode': 'async_latest', 'pending': self._pending is not None, 'writing': self._writing,
                    'submitted_version': self._submitted, 'written_version': self._written,
                    'coalesced': self._coalesced, 'last_write_ms': self._last_write_ms,
                    'last_error': self._last_error}

    def annotate(self, result):
        result['persistence'] = self.status()
        if result['persistence']['last_error']:
            result['persistence_warning'] = ('진단 JSON 저장 실패: '
                + result['persistence']['last_error'] + '. 현재 위치·상태는 API에서 제공합니다.')
        return result

    def flush(self, timeout=1.0):
        deadline = time.monotonic() + timeout
        with self._condition:
            while self._pending is not None or self._writing:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                self._condition.wait(remaining)
            return self._last_error is None

    def close(self, timeout=1.0):
        with self._condition:
            self._closed = True
            self._condition.notify_all()
            thread = self._thread
        if thread is not None:
            thread.join(timeout)
        return thread is None or not thread.is_alive()

    def _run(self):
        while True:
            with self._condition:
                while self._pending is None and not self._closed:
                    self._condition.wait()
                if self._pending is None:
                    return
                version, payload = self._pending
                self._pending = None
                self._writing = True
            # 직렬화·디렉터리 생성·교체·재시도 모두 상태 잠금 밖에서 수행한다.
            started = time.perf_counter()
            error = None
            try:
                self._write(payload)
            except Exception as caught:
                error = f'{type(caught).__name__}: {caught}'
            elapsed = (time.perf_counter() - started) * 1000
            with self._condition:
                previous_error = self._last_error
                self._last_error = error
                self._last_write_ms = elapsed
                if error is None:
                    self._written = version
                self._writing = False
                self._condition.notify_all()
            if error and error != previous_error:
                LOGGER.warning('진단 JSON 기록 실패: %s (%s)', self.path, error)

    def _write(self, payload):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        pending = None
        try:
            with NamedTemporaryFile(mode='w', encoding='utf-8', dir=self.path.parent,
                                    prefix='.snapshot-', delete=False) as stream:
                pending = Path(stream.name)
                json.dump(payload, stream, ensure_ascii=False, allow_nan=False)
                stream.write('\n')
            for attempt, delay in enumerate(RETRY_DELAYS_S):
                if delay:
                    time.sleep(delay)
                try:
                    pending.replace(self.path)
                    break
                except OSError as error:
                    if getattr(error, 'winerror', None) not in (5, 32, 33) or attempt == len(RETRY_DELAYS_S) - 1:
                        raise
        finally:
            if pending is not None:
                try:
                    pending.unlink(missing_ok=True)
                except OSError as error:
                    LOGGER.warning('진단 JSON 임시 파일 정리 보류: %s (%s)', pending, error)
