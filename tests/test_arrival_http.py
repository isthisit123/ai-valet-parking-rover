from http.server import ThreadingHTTPServer
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import threading
import time
import unittest
from urllib.request import Request, urlopen

import cv2

from scripts.http_map_server import handler_for
from src.map_processing.map_config import DEFAULT_MAP
from test_arrival_test import write_test_map
from test_aruco_tracker import marker_image


class ArrivalHttpTest(unittest.TestCase):
    def test_choose_case_then_real_aruco_observations_complete_pixel_test(self):
        root = Path(self.enterContext(TemporaryDirectory()))
        maps = root / "maps"
        report = write_test_map(maps, size=(400, 400), native=(400, 400), goal=(150, 150))
        output = root / "arrival/latest.json"
        server = ThreadingHTTPServer(("127.0.0.1", 0), handler_for(
            root, maps, DEFAULT_MAP.label_path, root / "mapping.json",
            aruco_output_path=root / "pose/latest.json", arrival_output_path=output))
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
            with urlopen(Request(url + "/api/arrival-test/start", options, {"Content-Type": "application/json"})) as response:
                self.assertEqual(json.load(response)["status"], "waiting_for_tracking")
            _, encoded = cv2.imencode(".png", marker_image())
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
