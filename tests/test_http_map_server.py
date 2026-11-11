import hashlib
import json
import tempfile
import threading
import unittest
from http.server import ThreadingHTTPServer
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import cv2
import numpy as np

from scripts.http_map_server import decode_png, handler_for, prepare_paper_image, transform_hints
from src.map_processing.map_config import DEFAULT_MAP


class HttpMapServerTest(unittest.TestCase):
    def setUp(self):
        self.root = Path(self.enterContext(tempfile.TemporaryDirectory()))
        web = self.root / "web"
        web.mkdir()
        (web / "index.html").write_text('<html><body><canvas id="main-canvas"></canvas></body></html>')
        self.output = self.root / "captures"
        self.mapping_path = DEFAULT_MAP.image_path.with_name("parking_map_real_parkinghaeven_mapping.json")
        server = ThreadingHTTPServer(("127.0.0.1", 0), handler_for(web, self.output, DEFAULT_MAP.label_path, self.mapping_path))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        self.addCleanup(server.server_close)
        self.addCleanup(thread.join)
        self.addCleanup(server.shutdown)
        self.url = f"http://127.0.0.1:{server.server_port}"

    def post(self, body, content_type="image/png"):
        return urlopen(Request(self.url + "/api/map-capture", body,
                               {"Content-Type": content_type, "X-Captured-At": "test-frame-time"}), timeout=30)

    def test_http_png_to_graph_and_semantics_without_replacing_default_map(self):
        body = DEFAULT_MAP.image_path.read_bytes()
        baseline = {p: hashlib.sha256(p.read_bytes()).hexdigest() for p in
                    (DEFAULT_MAP.image_path, DEFAULT_MAP.graph_path, DEFAULT_MAP.semantic_path, DEFAULT_MAP.label_path)}
        with self.post(body) as response:
            report = json.load(response)
        self.assertEqual(report["status"], "map_generated")
        self.assertEqual(report["graph"], {"nodes": 14, "edges": 13, "connected": True})
        self.assertEqual(report["detected"]["normal"], 4)
        self.assertEqual(report["detected"]["accessible"], 2)
        folder = self.output / report["capture_id"]
        self.assertEqual((folder / "source.png").read_bytes(), body)
        semantic = json.loads((folder / "semantic.json").read_text(encoding="utf-8"))
        self.assertEqual(len(semantic["regions"]), 10)
        with urlopen(self.url + report["files"]["preview"]) as response:
            self.assertTrue(response.read().startswith(b"\x89PNG"))
        self.assertEqual(baseline, {p: hashlib.sha256(p.read_bytes()).hexdigest() for p in baseline})

    def test_failed_mapping_is_reported_and_received_image_is_preserved(self):
        image = np.full((720, 1280, 3), 255, np.uint8)
        ok, encoded = cv2.imencode(".png", image)
        self.assertTrue(ok)
        with self.assertRaises(HTTPError) as failure:
            self.post(encoded.tobytes())
        self.assertEqual(failure.exception.code, 422)
        report = json.load(failure.exception)
        self.assertEqual(report["status"], "failed")
        self.assertTrue(report["error"])
        self.assertTrue((self.output / report["capture_id"] / "source.png").is_file())
        self.assertNotIn("semantic", report["files"])

    def test_web_bridge_and_invalid_uploads(self):
        with urlopen(self.url) as response:
            self.assertIn(b"rover-map-bridge.js", response.read())
        with urlopen(self.url + "/rover-map-bridge.js") as response:
            self.assertIn(b"/api/map-capture", response.read())
        for content_type in ("image/png", "image/jpeg"):
            with self.subTest(content_type=content_type), self.assertRaises(HTTPError) as failure:
                self.post(b"not a png", content_type)
            self.assertEqual(failure.exception.code, 400)
        self.assertFalse(self.output.exists())
        with self.assertRaises(ValueError):
            decode_png(b"not png")

    def test_canvas_hints_use_saved_inverse_transform(self):
        mapping = json.loads(self.mapping_path.read_text(encoding="utf-8"))
        labels = json.loads(DEFAULT_MAP.label_path.read_text(encoding="utf-8"))
        # 실제 옛 매핑은 무효이며, 역변환 자체는 별도 수학 fixture로 검증한다.
        with self.assertRaisesRegex(ValueError, "무효"):
            transform_hints(labels, mapping, [887, 665])
        mapping["status"] = "active"
        mapping["source"]["canvas_size_px"] = [320, 240]
        mapping["transform"] = {"matrix_3x3": [[4, 0, 0], [0, 3, 0], [0, 0, 1]]}
        converted, metadata = transform_hints(labels, mapping, [320, 240])
        self.assertEqual(metadata["basis"], "saved_camera_canvas_aspect")
        forward = np.array(mapping["transform"]["matrix_3x3"])
        for group in ("parking_spaces", "buildings", "gates"):
            for original, new in zip(labels[group], converted[group]):
                pixel = forward @ [*new["hint"], 1]
                np.testing.assert_allclose(pixel[:2] / pixel[2], original["hint"])
        with self.assertRaisesRegex(ValueError, "비율"):
            transform_hints(labels, mapping, [500, 500])

    def test_paper_mask_keeps_inside_pixels_and_does_not_modify_source(self):
        image = np.zeros((100, 200, 3), np.uint8)
        prepared = prepare_paper_image(image, {"reference_size_px": [200, 100],
                                             "paper_corners_px": [[20, 20], [180, 20], [180, 80], [20, 80]]})
        self.assertTrue(np.all(prepared[0, 0] == 255))
        self.assertTrue(np.all(prepared[50, 100] == 0))
        self.assertTrue(np.all(image == 0))


if __name__ == "__main__":
    unittest.main()
