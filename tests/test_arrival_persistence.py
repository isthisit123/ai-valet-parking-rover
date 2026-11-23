"""Windows의 도착 JSON 교체 잠금이 위치 HTTP 응답을 끊지 않는지 검증한다."""
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import threading
import unittest
from unittest.mock import MagicMock, patch
from urllib.error import HTTPError
from urllib.request import urlopen

from scripts.http_map_server import handler_for
from src.rover_control.arrival_test import ArrivalTest, SAVE_RETRY_DELAYS_S


def windows_error(code):
    error = PermissionError('Windows file replacement denied')
    error.winerror = code
    return error


class ArrivalPersistenceTest(unittest.TestCase):
    def setUp(self):
        self.root = Path(self.enterContext(TemporaryDirectory()))
        self.path = self.root / 'arrival/latest.json'
        self.arrival = ArrivalTest(self.root, output_path=self.path)
        self.assertTrue(self.arrival._save())
        self.original = self.path.read_bytes()

    def test_brief_file_lock_retries_then_publishes_current_state(self):
        real_replace = Path.replace
        calls = []
        def replace(pending, target):
            calls.append(target)
            if len(calls) == 1:
                raise windows_error(5)
            return real_replace(pending, target)
        self.arrival.state['reason'] = 'new state'
        with patch('src.rover_control.arrival_test.Path.replace', replace):
            self.assertTrue(self.arrival._save())
        self.assertEqual(len(calls), 2)
        self.assertEqual(json.loads(self.path.read_text())['reason'], 'new state')
        self.assertNotIn('persistence_warning', self.arrival.current())

    def test_persistent_file_lock_preserves_old_file_and_reports_warning(self):
        for code in (5, 32, 33):
            with self.subTest(code=code), \
                    patch('src.rover_control.arrival_test.Path.replace', side_effect=windows_error(code)) as replace, \
                    self.assertLogs('src.rover_control.arrival_test', level='WARNING'):
                self.assertFalse(self.arrival._save())
                self.assertEqual(replace.call_count, len(SAVE_RETRY_DELAYS_S))
            self.assertEqual(self.path.read_bytes(), self.original)
            self.assertIn(f'WinError {code}', self.arrival.current()['persistence_warning'])
        self.assertEqual(list(self.path.parent.glob('.arrival-*')), [])

    def test_success_after_file_unlock_clears_warning_in_api_and_file(self):
        with patch('src.rover_control.arrival_test.Path.replace', side_effect=windows_error(5)), \
                self.assertLogs('src.rover_control.arrival_test', level='WARNING'):
            self.arrival._save()
        self.assertTrue(self.arrival._save())
        self.assertNotIn('persistence_warning', self.arrival.current())
        self.assertNotIn('persistence_warning', json.loads(self.path.read_text()))

    def test_other_io_failures_are_not_treated_as_temporary_locks(self):
        with patch('src.rover_control.arrival_test.Path.replace', side_effect=windows_error(112)) as replace:
            with self.assertRaises(OSError):
                self.arrival._save()
        replace.assert_called_once()
        self.assertEqual(self.path.read_bytes(), self.original)

    def test_status_api_returns_json_instead_of_closing_connection_on_io_failure(self):
        broken = MagicMock()
        broken.current.side_effect = OSError('state storage unavailable')
        with patch('scripts.http_map_server.ArrivalTest', return_value=broken):
            handler = handler_for(self.root, self.root / 'maps', self.root / 'labels.json', self.root / 'mapping.json')
        server = ThreadingHTTPServer(('127.0.0.1', 0), handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            for path in ('/api/rover-pose', '/api/arrival-test'):
                with self.subTest(path=path), self.assertRaises(HTTPError) as raised:
                    urlopen(f'http://127.0.0.1:{server.server_port}{path}', timeout=2)
                response = raised.exception
                try:
                    self.assertEqual(response.code, 503)
                    self.assertIn('state storage unavailable', json.load(response)['error'])
                finally:
                    response.close()
        finally:
            server.shutdown()
            thread.join()
            server.server_close()
