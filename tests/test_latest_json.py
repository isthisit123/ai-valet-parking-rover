"""느린 저장·파일 잠금에서도 최신 진단 한 개와 원래 관측 시각을 보존한다."""
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import threading
import unittest
from unittest.mock import patch

from src.rover_control.latest_json import LatestJsonWriter, RETRY_DELAYS_S


class LatestJsonTest(unittest.TestCase):
    def setUp(self):
        self.root = Path(self.enterContext(TemporaryDirectory()))
        self.path = self.root / 'latest.json'
        self.writer = LatestJsonWriter(self.path)
        self.addCleanup(self.writer.close)

    def test_blocked_write_keeps_only_latest_pending_snapshot_and_original_capture_time(self):
        entered, release = threading.Event(), threading.Event()
        written = []
        original = self.writer._write

        def blocked(payload):
            written.append(payload['frame_id'])
            if len(written) == 1:
                entered.set()
                if not release.wait(2):
                    raise TimeoutError('시험 기록 대기')
            original(payload)

        with patch.object(self.writer, '_write', side_effect=blocked):
            try:
                self.writer.submit({'frame_id': 1, 'received_monotonic_s': 10})
                self.assertTrue(entered.wait(1))
                self.writer.submit({'frame_id': 2, 'received_monotonic_s': 20})
                latest = {'frame_id': 3, 'received_monotonic_s': 30, 'camera_source': {'session_id': 'A'}}
                self.writer.submit(latest)
                latest['camera_source']['session_id'] = 'changed after submit'
                status = self.writer.status()
                self.assertTrue(status['writing'])
                self.assertTrue(status['pending'])
                self.assertEqual(status['coalesced'], 1)
                self.assertFalse(self.writer.flush(timeout=0.01))
            finally:
                release.set()
            self.assertTrue(self.writer.flush())
        self.assertEqual(written, [1, 3])
        self.assertEqual(json.loads(self.path.read_text()), {
            'frame_id': 3, 'received_monotonic_s': 30, 'camera_source': {'session_id': 'A'}})
        self.assertEqual(self.writer.status()['written_version'], 3)
        self.assertEqual(list(self.root.glob('.snapshot-*')), [])

    def test_identical_snapshot_does_not_rewrite_or_refresh_capture_time(self):
        payload = {'frame_id': 1, 'received_monotonic_s': 10}
        self.assertTrue(self.writer.submit(payload))
        self.assertTrue(self.writer.flush())
        with patch.object(self.writer, '_write') as write:
            self.assertFalse(self.writer.submit(payload))
            self.assertTrue(self.writer.flush())
        write.assert_not_called()
        self.assertEqual(self.writer.status()['submitted_version'], 1)

    def test_file_lock_preserves_old_file_reports_error_and_allows_same_snapshot_retry(self):
        self.path.write_text('{"frame_id":0}')
        error = PermissionError('file locked')
        error.winerror = 5
        with patch('src.rover_control.latest_json.Path.replace', side_effect=error) as replace:
            self.writer.submit({'frame_id': 1})
            self.assertFalse(self.writer.flush())
        self.assertEqual(replace.call_count, len(RETRY_DELAYS_S))
        self.assertEqual(json.loads(self.path.read_text())['frame_id'], 0)
        state = self.writer.annotate({'tracking_status': 'detected'})
        self.assertIn('PermissionError', state['persistence_warning'])
        self.assertEqual(state['tracking_status'], 'detected')
        self.assertEqual(list(self.root.glob('.snapshot-*')), [])
        self.assertTrue(self.writer.submit({'frame_id': 1}))
        self.assertTrue(self.writer.flush())
        self.assertNotIn('persistence_warning', self.writer.annotate({}))
        self.assertEqual(json.loads(self.path.read_text())['frame_id'], 1)

    def test_serialization_failure_preserves_file_and_does_not_stop_writer(self):
        self.path.write_text('{"frame_id":0}')
        self.writer.submit({'invalid': float('nan')})
        self.assertFalse(self.writer.flush())
        self.assertIn('ValueError', self.writer.status()['last_error'])
        self.assertEqual(json.loads(self.path.read_text())['frame_id'], 0)
        self.writer.submit({'frame_id': 2})
        self.assertTrue(self.writer.flush())
        self.assertEqual(json.loads(self.path.read_text())['frame_id'], 2)

    def test_close_finishes_pending_snapshot_and_rejects_later_submit(self):
        self.writer.submit({'frame_id': 'final'})
        self.assertTrue(self.writer.close())
        self.assertEqual(json.loads(self.path.read_text())['frame_id'], 'final')
        with self.assertRaises(RuntimeError):
            self.writer.submit({'frame_id': 'after close'})
