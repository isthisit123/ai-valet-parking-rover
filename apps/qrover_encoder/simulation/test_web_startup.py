"""단일 출발 관측, 추적 재개와 통신 단절 제동를 검증한다."""
from pathlib import Path
from http.client import RemoteDisconnected
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import MagicMock
from urllib.error import HTTPError, URLError

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from web_drive import WebFeed, WebTransportUnavailable, default_repo
sys.path.insert(0, str(default_repo()))
from src.rover_control.pose_sender import PoseUnavailable


class Converter:
    def __init__(self, *, invalid_map=False):
        self.invalid_map, self.last_capture = invalid_map, -1

    def require_current(self):
        if self.invalid_map:
            raise ValueError('지도 불일치')

    def convert(self, observation):
        status = observation['tracking_status']
        if status in ('lost', 'stale', 'not_started'):
            raise PoseUnavailable(status)
        if status != 'detected':
            raise ValueError('마커 ID 중복')
        captured = observation['captured_monotonic_s']
        if captured <= self.last_capture:
            return None
        self.last_capture = captured
        return observation


class WebStartupTest(unittest.TestCase):
    def feed(self, states, *, repeat=False, invalid_map=False):
        folder = Path(self.enterContext(tempfile.TemporaryDirectory()))
        settings = folder / 'settings.json'
        settings.write_text('{}')
        feed = WebFeed('http://127.0.0.1:8001', Converter(invalid_map=invalid_map), settings)
        self.addCleanup(feed.close)
        feed.lost = threading.Event()
        feed.transport_lost = threading.Event()
        feed.pose_reads = 0
        first_stamp = time.monotonic()

        def read(path):
            if feed.transport_lost.is_set():
                raise WebTransportUnavailable('응답 연결 단절')
            if path == '/api/parking-occupancy':
                return {'age_seconds': 0.01}
            feed.pose_reads += 1
            status = 'lost' if feed.lost.is_set() else states[min(feed.pose_reads - 1, len(states) - 1)]
            if isinstance(status, Exception):
                raise status
            return {'tracking_status': status, 'detection_status': 'detected' if status == 'stale' else status,
                    'frame_id': feed.pose_reads,
                    'captured_monotonic_s': first_stamp if repeat else time.monotonic(),
                    'x_cm': 20, 'y_cm': 20, 'heading_deg': 90, 'arrival_test': {}}

        feed.read = read
        return feed

    def test_loss_and_staleness_before_first_pose_are_retried(self):
        feed = self.feed(['not_started', 'lost', 'stale', 'detected', 'detected', 'detected'])
        result = feed.start(timeout=1)
        self.assertTrue(feed.startup_complete)
        self.assertEqual(result['pose']['frame_id'], 4)
        self.assertNotIn('position_samples', result['pose'])
        self.assertIsNone(feed.error)

    def test_permanent_loss_times_out_and_never_becomes_ready(self):
        feed = self.feed(['lost'])
        started = time.monotonic()
        with self.assertRaisesRegex(TimeoutError, '마지막 상태'):
            feed.start(timeout=0.2)
        self.assertLess(time.monotonic() - started, 0.8)
        self.assertFalse(feed.startup_complete)
        self.assertIsNone((feed.latest or {}).get("pose"))
        self.assertTrue(feed.done.is_set())

    def test_first_frame_starts_but_duplicate_cannot_refresh_pose(self):
        feed = self.feed(['detected'], repeat=True)
        first = feed.start(timeout=0.3)
        self.assertEqual(first['pose']['frame_id'], 1)
        time.sleep(0.24)
        self.assertIsNone(feed.current()['pose'])
        self.assertIsNone(feed.error)

    def test_ambiguous_marker_is_fatal_without_startup_retry(self):
        feed = self.feed(['ambiguous', 'detected'])
        with self.assertRaisesRegex(RuntimeError, '중복'):
            feed.start(timeout=1)
        self.assertEqual(feed.pose_reads, 1)
        self.assertIsInstance(feed.error, ValueError)

    def test_map_mismatch_is_fatal_without_startup_retry(self):
        feed = self.feed(['detected'], invalid_map=True)
        with self.assertRaisesRegex(RuntimeError, '지도 불일치'):
            feed.start(timeout=1)
        self.assertEqual(feed.pose_reads, 0)

    def test_loss_after_startup_publishes_state_then_recovers(self):
        feed = self.feed(['detected'])
        first = feed.start(timeout=1)
        feed.lost.set()
        deadline = time.monotonic() + 1
        while feed.current()['pose'] and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertIsNone(feed.current()['pose'])
        self.assertTrue(feed.thread.is_alive())
        feed.lost.clear()
        while not feed.current()['pose'] and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertGreater(feed.current()['pose']['frame_id'], first['pose']['frame_id'])
        self.assertEqual(feed.current()['loss_count'], 1)

    def test_invalid_timeout_does_not_start_the_reader(self):
        feed = self.feed(['detected'])
        for timeout in (0, -1, float('nan'), float('inf')):
            with self.subTest(timeout=timeout), self.assertRaises(ValueError):
                feed.start(timeout=timeout)
        self.assertIsNone(feed.thread.ident)

    def test_transient_transport_errors_before_startup_are_retried(self):
        feed = self.feed([WebTransportUnavailable('연결 닫힘'), WebTransportUnavailable('시간 초과'),
                          'detected', 'detected', 'detected'])
        result = feed.start(timeout=1)
        self.assertEqual(result['pose']['frame_id'], 3)
        self.assertTrue(feed.startup_complete)

    def test_permanent_transport_failure_is_bounded_by_startup_timeout(self):
        feed = self.feed([WebTransportUnavailable('시간 초과')])
        with self.assertRaisesRegex(TimeoutError, '시간 초과'):
            feed.start(timeout=0.2)
        self.assertFalse(feed.startup_complete)
        self.assertIsNone((feed.latest or {}).get("pose"))

    def test_transport_failure_after_ready_remains_latched(self):
        feed = self.feed(['detected'])
        feed.start(timeout=1)
        feed.transport_lost.set()
        feed.thread.join(timeout=0.5)
        self.assertFalse(feed.thread.is_alive())
        self.assertIsInstance(feed.error, WebTransportUnavailable)
        feed.transport_lost.clear()
        with self.assertRaises(RuntimeError):
            feed.current()

    def test_read_classifies_only_network_errors_and_uses_phase_timeout(self):
        feed = self.feed(['detected'])
        feed.opener = MagicMock()
        for ready, expected_timeout in ((False, 0.5), (True, 0.12)):
            feed.startup_complete = ready
            for error in (RemoteDisconnected('closed'), TimeoutError('slow'), URLError('timed out')):
                feed.opener.open.side_effect = error
                with self.subTest(ready=ready, error=error), self.assertRaises(WebTransportUnavailable) as raised:
                    WebFeed.read(feed, '/api/rover-pose')
                self.assertIs(raised.exception.__cause__, error)
                self.assertEqual(feed.opener.open.call_args.kwargs['timeout'], expected_timeout)
        feed.opener.open.side_effect = HTTPError('http://127.0.0.1:8001/api/rover-pose', 400, 'bad request', {}, None)
        with self.assertRaises(HTTPError):
            WebFeed.read(feed, '/api/rover-pose')


if __name__ == '__main__':
    unittest.main(verbosity=2)
