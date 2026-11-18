from http.server import ThreadingHTTPServer
import json
import hashlib
from pathlib import Path
from tempfile import TemporaryDirectory
import threading
import unittest
from urllib.request import Request, urlopen

import cv2

from scripts.http_map_server import handler_for
from scripts.http_map_server import decode_tracking_image
from src.map_processing.map_config import DEFAULT_MAP
from test_aruco_tracker import marker_image


class ArucoHttpTest(unittest.TestCase):
    def test_jpeg_tracks_marker_and_checks_dimensions_before_decoding(self):
        image = marker_image()
        _, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 85])
        decoded = decode_tracking_image(encoded.tobytes(), 'image/jpeg')
        self.assertEqual(decoded.shape, image.shape)
        from src.rover_control.aruco_tracker import ArucoTracker
        self.assertEqual(ArucoTracker().observe(decoded)['rover']['id'], 7)
        invalid = b'\xff\xd8\xff\xc0\x00\x08\x08\xff\xff\xff\xff\x03'
        with self.assertRaisesRegex(ValueError, '해상도'):
            decode_tracking_image(invalid, 'image/jpeg')
        for body in (b'not jpeg', b'\xff\xd8\xff\xc0\x00\x01', b'\xff\xd8'):
            with self.assertRaises(ValueError):
                decode_tracking_image(body, 'image/jpeg')

    def test_shared_web_camera_api_has_pixel_heading_config_and_fixed_output(self):
        root = Path(self.enterContext(TemporaryDirectory()))
        output = root / "pose/latest.json"
        server = ThreadingHTTPServer(("127.0.0.1", 0), handler_for(
            root, root / "maps", DEFAULT_MAP.label_path, root / "mapping.json", aruco_output_path=output))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            url = f"http://127.0.0.1:{server.server_port}"
            with urlopen(url + "/api/aruco-config") as response:
                config = json.load(response)
            self.assertEqual(config["marker_id"], 7)
            self.assertEqual(config["period_ms"], 100)
            (root / 'js').mkdir()
            (root / 'js/YOLODetector.js').write_text('export class YOLODetector {}')
            for asset, expected in [('/js/YOLODetector.js', b'new Worker'),
                                    ('/rover-original-yolo.js', b'export class YOLODetector'),
                                    ('/parking-detection-worker.js', b'MIN_VEHICLE_CONFIDENCE = 0.70')]:
                with urlopen(url + asset) as response:
                    self.assertIn(expected, response.read())
            with urlopen(url + "/js/ArUcoBridge.js") as response:
                self.assertIn(b"export class ArUcoBridge", response.read())
            _, encoded = cv2.imencode(".png", marker_image())
            with urlopen(Request(url + "/api/aruco", encoded.tobytes(), {"Content-Type": "image/png"})) as response:
                result = json.load(response)
            self.assertEqual(result["rover"]["id"], 7)
            self.assertEqual(result["coordinate_frame"], "camera_image")
            self.assertIsNone(result["map_pose"])
            self.assertTrue(output.is_file())
            with urlopen(url + '/api/aruco-debug') as response:
                diagnostic = json.load(response)
            self.assertEqual(diagnostic['image_sha256'], hashlib.sha256(encoded.tobytes()).hexdigest())
            self.assertEqual(diagnostic['detection_status'], 'detected')
            self.assertEqual(diagnostic['detection_attempt_details'][0]['decoded_ids'], [7])
            self.assertIsInstance(diagnostic['fallback_budget_exhausted'], bool)
            _, jpeg = cv2.imencode('.jpg', marker_image(), [cv2.IMWRITE_JPEG_QUALITY, 85])
            with urlopen(Request(url + '/api/aruco', jpeg.tobytes(), {'Content-Type': 'image/jpeg'})) as response:
                self.assertEqual(json.load(response)['rover']['id'], 7)
            with urlopen(url + diagnostic['frame_url']) as response:
                self.assertEqual(response.read(), encoded.tobytes())
            self.assertFalse((root / "maps").exists())
            with urlopen(url + "/api/rover-pose") as response:
                self.assertIn(json.load(response)["tracking_status"], ("detected", "stale"))
            missing = marker_image() * 0 + 255
            _, missing_jpeg = cv2.imencode('.jpg', missing)
            with urlopen(Request(url + '/api/aruco', missing_jpeg.tobytes(), {'Content-Type': 'image/jpeg'})) as response:
                self.assertEqual(json.load(response)['detection_status'], 'lost')
            with urlopen(url + '/api/aruco-debug?miss=1') as response:
                failed = json.load(response)
            self.assertEqual(failed['detection_status'], 'lost')
            with urlopen(url + failed['frame_url']) as response:
                self.assertEqual(response.read(), missing_jpeg.tobytes())
        finally:
            server.shutdown()
            thread.join()
            server.server_close()


if __name__ == "__main__":
    unittest.main()
