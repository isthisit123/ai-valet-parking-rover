"""기록·검출이 막혀도 위치 API와 현재 좌표의 만료 검사가 진행되는지 검증한다."""
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import threading
import time
import unittest
from unittest.mock import patch
from urllib.request import Request, urlopen

import cv2

from scripts.http_map_server import handler_for
from src.rover_control.aruco_tracker import ArucoTracker
from src.rover_control.arrival_test import ArrivalTest
from src.rover_control.latest_json import LatestJsonWriter
from src.map_processing.path_visualizer import DEFAULT_ROUTE_CASES
from test_aruco_tracker import marker_image
from test_arrival_test import write_test_map


class AsyncDiagnosticsTest(unittest.TestCase):
    def setUp(self):
        self.root = Path(self.enterContext(TemporaryDirectory()))

    def test_position_get_returns_during_slow_detection_and_expires_previous_pose(self):
        tracker = ArucoTracker()
        image = marker_image()
        tracker.observe(image, received_monotonic_s=time.monotonic() - 1)
        entered, release, read_done = threading.Event(), threading.Event(), threading.Event()
        original = tracker._detect
        result = []

        def blocked(*args):
            entered.set()
            release.wait(2)
            return original(*args)

        def read():
            result.append(tracker.current())
            read_done.set()

        with patch.object(tracker, '_detect', side_effect=blocked):
            worker = threading.Thread(target=tracker.observe, args=(image,))
            worker.start()
            try:
                self.assertTrue(entered.wait(1))
                reader = threading.Thread(target=read)
                reader.start()
                self.assertTrue(read_done.wait(.5), '위치 조회가 새 검출을 기다렸습니다.')
                self.assertEqual(result[0]['tracking_status'], 'stale')
                self.assertIsNone(result[0]['rover'])
            finally:
                release.set()
                worker.join()
                if 'reader' in locals():
                    reader.join()

    def test_arrival_batches_observation_saves_and_get_does_not_rewrite_same_expiry(self):
        report = write_test_map(self.root)
        arrival = ArrivalTest(self.root, output_path=self.root / 'arrival/latest.json', async_persistence=True)
        self.addCleanup(arrival.writer.close)
        arrival.start(DEFAULT_ROUTE_CASES[0][0], capture_id=report['capture_id'])

        def observation(t):
            return {'tracking_status': 'detected', 'rover': {'id': 7, 'center_px': [200, 160]},
                    'camera_source': report['camera_source'], 'width': 800, 'height': 600,
                    'received_monotonic_s': t, 'frame_id': str(t)}

        arrival.observe(observation(10), now=10)
        before = arrival.writer.status()['submitted_version']
        arrival.observe(observation(11), now=11)
        self.assertEqual(arrival.writer.status()['submitted_version'], before + 1)
        arrival.current(now=11.3)
        expired_version = arrival.writer.status()['submitted_version']
        for now in (11.4, 11.6, 12):
            state = arrival.current(now=now)
            self.assertIsNone(state['current_position_px'])
            self.assertFalse(state['guidance_available'])
        self.assertEqual(arrival.writer.status()['submitted_version'], expired_version)
        state = arrival.current(now=17)
        self.assertFalse(state['tracking_paused'])
        self.assertIsNone(state['last_known_position_px'])
        self.assertGreater(arrival.writer.status()['submitted_version'], expired_version)

    def test_slow_arrival_file_write_does_not_block_stop_or_current(self):
        arrival = ArrivalTest(self.root, output_path=self.root / 'arrival/latest.json', async_persistence=True)
        self.addCleanup(arrival.writer.close)
        entered, release, stopped = threading.Event(), threading.Event(), threading.Event()
        original = arrival.writer._write

        def blocked(payload):
            entered.set()
            release.wait(2)
            original(payload)

        with patch.object(arrival.writer, '_write', side_effect=blocked):
            arrival._save()
            try:
                self.assertTrue(entered.wait(1))
                worker = threading.Thread(target=lambda: (arrival.stop(), stopped.set()))
                worker.start()
                self.assertTrue(stopped.wait(.5), '정지 상태가 파일 저장을 기다렸습니다.')
                self.assertEqual(arrival.current()['status'], 'stopped')
                self.assertFalse(arrival.current()['guidance_available'])
            finally:
                release.set()
                if 'worker' in locals():
                    worker.join()
                self.assertTrue(arrival.writer.flush())
        self.assertEqual(json.loads(arrival.output_path.read_text())['status'], 'stopped')

    def test_http_response_and_stale_pose_continue_while_disk_write_is_blocked(self):
        output = self.root / 'pose/latest.json'
        output.parent.mkdir()
        output.write_text('{"frame_id":"before"}')
        handler = handler_for(self.root, self.root / 'maps', self.root / 'labels.json', self.root / 'mapping.json',
                              aruco_output_path=output, arrival_output_path=self.root / 'arrival/latest.json',
                              async_diagnostics=True)
        server = ThreadingHTTPServer(('127.0.0.1', 0), handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        url = f'http://127.0.0.1:{server.server_port}'
        entered, release = threading.Event(), threading.Event()
        original = LatestJsonWriter._write

        def blocked(writer, payload):
            entered.set()
            release.wait(2)
            original(writer, payload)

        try:
            with patch.object(LatestJsonWriter, '_write', blocked):
                try:
                    _, encoded = cv2.imencode('.jpg', marker_image())
                    with urlopen(Request(url + '/api/aruco', encoded.tobytes(),
                                         {'Content-Type': 'image/jpeg', 'X-Frame-Id': 'one'}), timeout=1) as response:
                        result = json.load(response)
                    self.assertTrue(entered.wait(1))
                    self.assertEqual(result['persistence']['mode'], 'async_latest')
                    self.assertEqual(json.loads(output.read_text())['frame_id'], 'before')
                    with urlopen(url + '/api/rover-pose', timeout=.5) as response:
                        current = json.load(response)
                    self.assertEqual(current['frame_id'], 'one')
                    self.assertTrue(current['persistence']['writing'])
                    time.sleep(.21)
                    with urlopen(url + '/api/rover-pose', timeout=.5) as response:
                        expired = json.load(response)
                    self.assertEqual(expired['tracking_status'], 'stale')
                    self.assertIsNone(expired['rover'])
                    self.assertEqual(expired['received_monotonic_s'], result['received_monotonic_s'])
                finally:
                    release.set()
                    handler.close_diagnostics()
            persisted = json.loads(output.read_text())
            self.assertEqual(persisted['frame_id'], 'one')
            self.assertEqual(persisted['received_monotonic_s'], result['received_monotonic_s'])
        finally:
            release.set()
            server.shutdown()
            thread.join()
            server.server_close()
            handler.close_diagnostics()
