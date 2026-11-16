from http.server import ThreadingHTTPServer
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import threading
import unittest
from urllib.request import Request, urlopen

import cv2

from scripts.http_map_server import handler_for
from src.map_processing.map_config import DEFAULT_MAP
from test_aruco_tracker import marker_image


class ArucoHttpTest(unittest.TestCase):
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
            with urlopen(url + "/js/ArUcoBridge.js") as response:
                self.assertIn(b"export class ArUcoBridge", response.read())
            _, encoded = cv2.imencode(".png", marker_image())
            with urlopen(Request(url + "/api/aruco", encoded.tobytes(), {"Content-Type": "image/png"})) as response:
                result = json.load(response)
            self.assertEqual(result["rover"]["id"], 7)
            self.assertEqual(result["coordinate_frame"], "camera_image")
            self.assertIsNone(result["map_pose"])
            self.assertTrue(output.is_file())
            self.assertFalse((root / "maps").exists())
            with urlopen(url + "/api/rover-pose") as response:
                self.assertIn(json.load(response)["tracking_status"], ("detected", "stale"))
        finally:
            server.shutdown()
            thread.join()
            server.server_close()


if __name__ == "__main__":
    unittest.main()
