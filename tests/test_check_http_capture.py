from contextlib import redirect_stdout
from copy import deepcopy
from hashlib import sha256
from http.server import ThreadingHTTPServer
import io
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import threading
import unittest
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from scripts.check_http_capture import check_capture
from scripts.http_map_server import handler_for
from scripts.ocr_buildings import LABEL_IDS
from src.map_processing.map_config import DEFAULT_MAP


class CheckHttpCaptureTest(unittest.TestCase):
    def setUp(self):
        self.root = Path(self.enterContext(TemporaryDirectory()))
        self.capture = self.root / "capture"
        self.capture.mkdir()
        self.body = DEFAULT_MAP.image_path.read_bytes()
        self.received = {"source_sha256": sha256(self.body).hexdigest(), "image_size_px": [1280, 720]}
        (self.capture / "source.png").write_bytes(self.body)
        (self.capture / "report.json").write_text(json.dumps(self.received), encoding="utf-8")
        labels = json.loads(DEFAULT_MAP.label_path.read_text(encoding="utf-8"))
        self.ocr = {"source_sha256": self.received["source_sha256"], "image_size_px": [1280, 720],
                    "status": "all_required_labels_recognized", "required_ids": list(LABEL_IDS.values()),
                    "labels": [{"id": item["id"], "status": "recognized", "center_candidate_px": item["hint"]}
                               for group in ("buildings", "gates", "parking_spaces") for item in labels[group]]}

    def run_check(self, report=None):
        with patch("scripts.generate_map.run_ocr", return_value=report or self.ocr), redirect_stdout(io.StringIO()):
            return check_capture(self.capture, Path(sys.executable))

    def test_http_received_png_runs_local_pipeline_without_old_hints(self):
        web = self.root / "web"
        web.mkdir()
        # 기존 힌트 파일이 없어 HTTP 지도 생성은 실패해도 수신 PNG 검사는 가능해야 한다.
        server = ThreadingHTTPServer(("127.0.0.1", 0), handler_for(
            web, self.root / "captures", self.root / "missing_labels.json", self.root / "missing_mapping.json",
            legacy_hints=True))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            request = Request(f"http://127.0.0.1:{server.server_port}/api/map-capture", self.body,
                              {"Content-Type": "image/png"})
            with self.assertRaises(HTTPError) as failure:
                urlopen(request, timeout=30)
            self.assertEqual(failure.exception.code, 422)
            with failure.exception as response:
                received = json.load(response)
        finally:
            server.shutdown()
            thread.join()
            server.server_close()
        self.capture = Path(received["output_directory"])
        baseline = {path: path.read_bytes() for path in (DEFAULT_MAP.image_path, DEFAULT_MAP.label_path,
                                                        DEFAULT_MAP.graph_path, DEFAULT_MAP.semantic_path)}
        first = self.run_check()
        second = self.run_check()
        self.assertNotEqual(first["output_directory"], second["output_directory"])
        self.assertEqual(first["status"], "check_passed")
        self.assertEqual(set(first["mapped_ids"]), set(LABEL_IDS.values()))
        self.assertEqual(first["graph"], {"nodes": 14, "edges": 13})
        output = Path(first["output_directory"])
        self.assertEqual((output / "source.png").read_bytes(), self.body)
        self.assertEqual((self.capture / "source.png").read_bytes(), self.body)
        self.assertEqual(json.loads((self.capture / "report.json").read_text()), received)
        self.assertEqual(baseline, {path: path.read_bytes() for path in baseline})

    def test_changed_hash_or_dimensions_stops_before_ocr(self):
        for key, wrong in (("source_sha256", "wrong"), ("image_size_px", [640, 360])):
            with self.subTest(field=key):
                (self.capture / "report.json").write_text(json.dumps({**self.received, key: wrong}), encoding="utf-8")
                with patch("scripts.check_http_capture.generate_map") as generate:
                    result = check_capture(self.capture, Path(sys.executable))
                generate.assert_not_called()
                self.assertEqual(result["status"], "failed")
                self.assertEqual(result["stage"], "input")
                self.assertIn("다릅니다", result["error"])
                self.assertTrue((Path(result["output_directory"]) / "report.json").is_file())

    def test_missing_label_is_failure_and_preserves_received_files(self):
        missing = deepcopy(self.ocr)
        missing["status"] = "needs_review"
        missing["labels"][0]["status"] = "missing"
        before = {path: path.read_bytes() for path in self.capture.iterdir()}
        result = self.run_check(missing)
        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["stage"], "ocr_map")
        self.assertIn("building_a(missing)", result["error"])
        self.assertNotIn("semantic", result["files"])
        self.assertEqual(before, {path: path.read_bytes() for path in before})

    def test_invalid_png_is_reported_without_running_ocr(self):
        (self.capture / "source.png").write_bytes(b"not PNG")
        with patch("scripts.check_http_capture.generate_map") as generate:
            result = check_capture(self.capture, Path(sys.executable))
        generate.assert_not_called()
        self.assertEqual(result["stage"], "input")
        self.assertEqual(result["status"], "failed")
        self.assertIn("PNG", result["error"])


if __name__ == "__main__":
    unittest.main()
