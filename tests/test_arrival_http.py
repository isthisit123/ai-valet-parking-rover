from http.server import ThreadingHTTPServer
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import threading
import time
import unittest
from urllib.request import Request, urlopen
from urllib.error import HTTPError
from unittest.mock import patch
from datetime import datetime, timedelta, timezone

import cv2
import numpy as np

from scripts.http_map_server import handler_for
from src.map_processing.map_config import DEFAULT_MAP
from test_live_parking import write_live_map
from test_aruco_tracker import marker_image


class ArrivalHttpTest(unittest.TestCase):
    def test_choose_case_then_real_aruco_observations_complete_pixel_test(self):
        root = Path(self.enterContext(TemporaryDirectory()))
        maps = root / "maps"
        report = write_live_map(maps)
        self.enterContext(patch('src.rover_control.live_parking.LiveParking.FREE_CONFIRM_SECONDS', 0))
        output = root / "arrival/latest.json"
        server = ThreadingHTTPServer(("127.0.0.1", 0), handler_for(
            root, maps, DEFAULT_MAP.label_path, root / "mapping.json",
            aruco_output_path=root / "pose/latest.json", arrival_output_path=output,
            occupancy_output_path=root / 'occupancy/latest.json'))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        url = f"http://127.0.0.1:{server.server_port}"

        def get(path):
            with urlopen(url + path, timeout=5) as response:
                return json.load(response)

        try:
            catalog = get("/api/arrival-tests")
            self.assertTrue(catalog["ready"])
            self.assertEqual(len(catalog["cases"]), 5)
            case = catalog["cases"][2]["case_id"]
            options = json.dumps({"case_id": case, "capture_id": report["capture_id"], "hold_seconds": 0.5}).encode()
            with self.assertRaises(HTTPError) as unavailable:
                urlopen(Request(url + '/api/arrival-test/start', options, {'Content-Type': 'application/json'}))
            self.assertEqual(unavailable.exception.code, 400)
            snapshot = {**{key: report[key] for key in ('capture_id', 'map_id', 'map_image_sha256', 'id_assignment_revision', 'camera_source')},
                        'schema_version': '0.1-draft',
                        'spaces': [{'parking_space_id': f'space_{i}', 'status': 'occupied' if i == 4 else 'free',
                                    'confidence': 0.5} for i in range(1, 7)]}
            captured_base = datetime.now(timezone.utc) - timedelta(seconds=0.5)
            frame_index = 0

            def next_frame_time():
                nonlocal frame_index
                frame_index += 1
                # Windows 시계 해상도 때문에 서로 다른 시험 프레임의 시각이 같아지지 않게 한다.
                # 6개 모두 과거의 서로 다른 촬영 시각이며 서버의 중복 거부는 유지한다.
                return (captured_base + timedelta(milliseconds=50 * frame_index)).isoformat()

            for _ in range(3):
                snapshot['observed_at'] = next_frame_time()
                with urlopen(Request(url + '/api/parking-occupancy', json.dumps(snapshot).encode(),
                                     {'Content-Type': 'application/json'})) as response:
                    self.assertIn('occupancy', json.load(response))
            self.assertTrue(get('/api/parking-occupancy')['fresh'])
            manual = json.dumps({'case_id': case, 'capture_id': report['capture_id'], 'parking_space_id': 'space_4'}).encode()
            with self.assertRaises(HTTPError) as occupied:
                urlopen(Request(url + '/api/arrival-test/start', manual, {'Content-Type': 'application/json'}))
            self.assertIn('이미 점유된 주차칸', json.load(occupied.exception)['error'])
            with urlopen(Request(url + "/api/arrival-test/start", options, {"Content-Type": "application/json"})) as response:
                started = json.load(response)
                self.assertEqual(started["status"], "waiting_for_tracking")
                self.assertEqual(started['parking_space_id'], 'space_5')
                self.assertEqual(started['goal_px'], [260, 230])
                self.assertEqual(started['road_goal_px'], [260, 200])
                self.assertEqual(started['waypoint_target'], 'parking_center')
                self.assertAlmostEqual(started['arrival_region']['actual_area_ratio'], 0.5)
                self.assertAlmostEqual(started['radius_px'], started['arrival_region']['radius_px'])
            snapshot['spaces'][3]['status'] = 'free'
            for _ in range(3):
                snapshot['observed_at'] = next_frame_time()
                with urlopen(Request(url + '/api/parking-occupancy', json.dumps(snapshot).encode(),
                                     {'Content-Type': 'application/json'})) as response:
                    json.load(response)
            changed = get('/api/arrival-test')
            self.assertEqual(changed['parking_space_id'], 'space_4')
            self.assertEqual(changed['target_changes'][-1]['from'], 'space_5')
            image = np.full((400, 400, 3), 255, np.uint8)
            image[180:280, 270:370] = marker_image()[100:200, 100:200]
            _, encoded = cv2.imencode(".png", image)
            for index in range(16):
                headers = {"Content-Type": "image/png", "X-Camera-Session": "camera-A", "X-Source-Type": "webcam",
                           "X-Source-Width": "400", "X-Source-Height": "400", "X-Frame-Id": str(index)}
                with urlopen(Request(url + "/api/aruco", encoded.tobytes(), headers)) as response:
                    result = json.load(response)["arrival_test"]
                if result["status"] == "success":
                    break
                time.sleep(0.105)
            self.assertEqual(result["status"], "success")
            self.assertEqual(result["case_id"], case)
            self.assertEqual(json.loads(output.read_text())["status"], "success")
            self.assertEqual(get("/api/arrival-test")["status"], "success")
            with urlopen(Request(url + "/api/arrival-test/stop", b"{}", {"Content-Type": "application/json"})) as response:
                self.assertEqual(json.load(response)["status"], "stopped")
            with urlopen(url + "/rover-arrival-bridge.js") as response:
                self.assertIn("테스트케이스 선택", response.read().decode("utf-8"))
        finally:
            server.shutdown()
            thread.join()
            server.server_close()


if __name__ == "__main__":
    unittest.main()
