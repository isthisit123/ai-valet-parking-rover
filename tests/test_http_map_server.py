import hashlib
import json
import tempfile
import threading
import unittest
from http.server import ThreadingHTTPServer
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import Request, urlopen
from unittest.mock import patch

import cv2
import numpy as np

from scripts.http_map_server import decode_png, handler_for, prepare_paper_image, transform_hints
from src.map_processing.map_reader import detect_road_mask, load_map_image
from src.map_processing.graph_builder import remove_small_components
from src.map_processing.path_visualizer import DEFAULT_ROUTE_CASES, connect_waypoints
from src.map_processing.map_config import DEFAULT_MAP
from src.rover_control.tracking import load_route
from scripts.ocr_buildings import LABEL_IDS


class HttpMapServerTest(unittest.TestCase):
    def setUp(self):
        self.root = Path(self.enterContext(tempfile.TemporaryDirectory()))
        self.web = self.root / "web"
        self.web.mkdir()
        (self.web / "index.html").write_text('<html><body><canvas id="main-canvas"></canvas></body></html>')
        self.output = self.root / "captures"
        self.mapping_path = DEFAULT_MAP.image_path.with_name("parking_map_real_parkinghaeven_mapping.json")
        labels = json.loads(DEFAULT_MAP.label_path.read_text(encoding="utf-8"))
        self.hints = {item["id"]: item["hint"] for key in ("buildings", "gates", "parking_spaces")
                      for item in labels[key]}
        self.enterContext(patch("scripts.generate_map.run_ocr", side_effect=self.ocr_report))
        self.start_server()

    def start_server(self, **options):
        handler = handler_for(self.web, self.output, DEFAULT_MAP.label_path, self.mapping_path, **options)
        server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        self.addCleanup(server.server_close)
        self.addCleanup(thread.join)
        self.addCleanup(server.shutdown)
        self.url = f"http://127.0.0.1:{server.server_port}"

    def test_world_calibration_uses_http_snapshot_and_withholds_routes_on_wrong_check(self):
        frame = load_map_image(DEFAULT_MAP.image_path)
        # 원본 사진의 A~D와 독립 확인점 E가 모두 같은 입력에 있어야 한다.
        cv2.line(frame, (653, 377), (681, 377), (0, 0, 220), 5)
        cv2.line(frame, (667, 366), (667, 388), (0, 0, 220), 5)
        ok, encoded = cv2.imencode(".png", frame)
        self.assertTrue(ok)
        body = encoded.tobytes()
        layout = {"map_id": DEFAULT_MAP.map_id, "paper_size_cm": [109.5, 79],
                  "markers_cm": {"A": [7.68, 19.76], "B": [86.87, 31.64], "C": [103.63, 67.87],
                                 "D": [16.67, 49.75], "E": [51.30, 48.19]}}
        self.start_server(marker_layout=layout)
        with self.post(body) as response:
            report = json.load(response)
        world = report["world_calibration"]
        self.assertEqual(world["status"], "verified", world)
        self.assertEqual(len(world["cases"]), 5)
        with urlopen(self.url + world["files"]["calibration"]) as response:
            calibration = json.load(response)
        self.assertEqual(calibration["source_frame"]["sha256"], hashlib.sha256(body).hexdigest())
        route_url = world["cases"][0]["files"]["waypoints"]
        with urlopen(self.url + route_url) as response:
            route = json.load(response)
        self.assertEqual((route["coordinate_frame"], route["unit"], route["waypoint_target"]),
                         ("world", "cm", "parking_center"))
        self.assertFalse(route["occupancy_checked"])
        path = self.output / route_url.removeprefix("/captures/")
        self.assertEqual(load_route(path, require_verified=True).map_id, DEFAULT_MAP.map_id)
        cm_map_url = world["cases"][0]["files"]["preview"].replace(".png", ".json")
        with urlopen(self.url + cm_map_url) as response:
            cm_map = json.load(response)
        cm_spaces = [r for r in cm_map["regions"] if r.get("type") in ("normal", "accessible")]
        self.assertEqual(len(cm_spaces), 6)
        self.assertTrue(all("polygon_cm" in r and "polygon_px" not in r for r in cm_spaces))

        layout["markers_cm"]["E"] = [58.0, 48.19]
        with self.post(body) as response:
            failed_world = json.load(response)
        self.assertEqual(failed_world["route_generation"]["status"], "generated")
        self.assertEqual(failed_world["world_calibration"]["status"], "failed")
        self.assertFalse(path.exists())

    def ocr_report(self, path, _python, output, _score):
        body = path.read_bytes()
        height, width = cv2.imdecode(np.frombuffer(body, np.uint8), cv2.IMREAD_COLOR).shape[:2]
        offset = 120 if height == 960 else 0
        report = {"source_sha256": hashlib.sha256(body).hexdigest(), "image_size_px": [width, height],
                  "status": "all_required_labels_recognized", "required_ids": list(LABEL_IDS.values()),
                  "labels": [{"id": key, "status": "recognized", "center_candidate_px": [x, y + offset]}
                             for key, (x, y) in self.hints.items()]}
        # 실제 OCR과 동일한 기록 위치를 써서 실패 시 웹 진단 링크도 확인한다.
        folder = output / "ocr_run"
        folder.mkdir()
        (folder / "report.json").write_text(json.dumps(report), encoding="utf-8")
        return report

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
        self.assertEqual(report["generation_method"], "ocr")
        self.assertEqual(report["graph"], {"nodes": 14, "edges": 13, "connected": True})
        self.assertEqual(report["detected"]["normal"], 4)
        self.assertEqual(report["detected"]["accessible"], 2)
        folder = Path(report["output_directory"])
        self.assertEqual((folder / "source.png").read_bytes(), body)
        semantic = json.loads((folder / "semantic.json").read_text(encoding="utf-8"))
        self.assertEqual(len(semantic["regions"]), 10)
        self.assertEqual(semantic["map_image_sha256"], hashlib.sha256(body).hexdigest())
        roi = report["parking_regions"]
        self.assertEqual(roi["status"], "generated")
        self.assertEqual([r["id"] for r in roi["regions"]], [f"P{i}" for i in range(1, 7)])
        self.assertTrue(all(len(r["points"]) == 4 for r in roi["regions"]))
        self.assertEqual(roi["map_image_sha256"], semantic["map_image_sha256"])
        with urlopen(self.url + report["files"]["parking_regions"]) as response:
            self.assertEqual(json.load(response), roi)
        with urlopen(self.url + "/") as response:
            self.assertIn('/parking-regions-bridge.js', response.read().decode())
        with urlopen(self.url + "/parking-regions-bridge.js") as response:
            self.assertIn('RoverParkingRegions', response.read().decode())
        routes = report["route_generation"]
        self.assertEqual(routes["status"], "generated")
        self.assertEqual(routes["planning_mode"], "static_map")
        self.assertFalse(routes["occupancy_checked"])
        self.assertEqual(routes["unit"], "pixel")
        self.assertEqual([case["case_id"] for case in routes["cases"]], [case[0] for case in DEFAULT_ROUTE_CASES])
        road_mask = remove_small_components(detect_road_mask(load_map_image(DEFAULT_MAP.image_path)))
        nodes = {node["id"]: node for node in semantic["nodes"]}
        for case in routes["cases"]:
            with self.subTest(case=case["case_id"]):
                with urlopen(self.url + case["files"]["waypoints"]) as response:
                    route = json.load(response)
                self.assertEqual(route["map_id"], semantic["map_id"])
                self.assertEqual(route["map_image_sha256"], semantic["map_image_sha256"])
                self.assertEqual(route["id_assignment_revision"], semantic["id_assignment_revision"])
                self.assertEqual(route["planning_mode"], "static_map")
                self.assertFalse(route["occupancy_checked"])
                self.assertEqual(route["waypoint_target"], "parking_road_node")
                points = np.asarray(route["waypoints"])
                self.assertTrue(np.all(road_mask[points[:, 1], points[:, 0]]))
                steps = np.linalg.norm(np.diff(points, axis=0), axis=1)
                self.assertTrue(np.all((steps > 0) & (steps <= np.sqrt(2))))
                self.assertEqual(nodes[route["start_node_id"]]["role"], "entrance")
                self.assertEqual(nodes[route["destination_node_id"]]["role"], "parking_" + case["parking_type"])
                with urlopen(self.url + case["files"]["preview"]) as response:
                    self.assertEqual(cv2.imdecode(np.frombuffer(response.read(), np.uint8), cv2.IMREAD_COLOR).shape,
                                     (720, 1280, 3))
        with urlopen(self.url + report["files"]["preview"]) as response:
            self.assertTrue(response.read().startswith(b"\x89PNG"))
        self.assertEqual(baseline, {p: hashlib.sha256(p.read_bytes()).hexdigest() for p in baseline})

    def test_region_export_failure_keeps_map_and_routes_without_partial_roi(self):
        with patch("scripts.http_map_server.export_parking_regions", side_effect=ValueError("invalid polygon")):
            with self.post(DEFAULT_MAP.image_path.read_bytes()) as response:
                report = json.load(response)
        self.assertTrue(report["published_as_latest"])
        self.assertEqual(report["status"], "map_generated")
        self.assertEqual(report["route_generation"]["status"], "generated")
        self.assertEqual(report["parking_regions"], {"status": "failed", "error": "invalid polygon"})
        self.assertNotIn("parking_regions", report["files"])
        self.assertFalse((Path(report["output_directory"]) / "parking_regions.json").exists())

    def test_repeated_success_overwrites_fixed_files_without_accumulating_folders(self):
        old = self.output / "old_capture"
        old.mkdir(parents=True)
        (old / "keep.txt").write_text("historical capture")
        with self.post(DEFAULT_MAP.image_path.read_bytes()) as response:
            first = json.load(response)
        frame = load_map_image(DEFAULT_MAP.image_path)
        frame[0, 0] = 254
        _, encoded = cv2.imencode(".png", frame)
        with self.post(encoded.tobytes()) as response:
            second = json.load(response)
        self.assertNotEqual(first["capture_id"], second["capture_id"])
        self.assertEqual(first["output_directory"], second["output_directory"])
        self.assertEqual(first["files"]["source"], "/captures/latest/source.png")
        self.assertEqual(first["files"], second["files"])
        self.assertEqual((self.output / "latest/source.png").read_bytes(), encoded.tobytes())
        self.assertEqual({p.name for p in self.output.iterdir()}, {"latest", "old_capture"})
        self.assertEqual({p.name for p in (self.output / "latest/ocr").iterdir()}, {"report.json"})
        self.assertEqual(len(list((self.output / "latest/routes").iterdir())), 10)
        self.assertEqual((old / "keep.txt").read_text(), "historical capture")

    def test_failed_attempts_preserve_latest_and_replace_fixed_diagnostics(self):
        with self.post(DEFAULT_MAP.image_path.read_bytes()) as response:
            success = json.load(response)
        latest = Path(success["output_directory"])
        before = {p.relative_to(latest): p.read_bytes() for p in latest.rglob("*") if p.is_file()}
        for value in (255, 200):
            _, encoded = cv2.imencode(".png", np.full((720, 1280, 3), value, np.uint8))
            with self.assertRaises(HTTPError) as failure:
                self.post(encoded.tobytes())
            with failure.exception as response:
                report = json.load(response)
            self.assertTrue(report["latest_preserved"])
            self.assertEqual(report["storage_key"], "last_failed")
            self.assertEqual(report["files"]["source"], "/captures/last_failed/source.png")
            self.assertEqual((self.output / "last_failed/source.png").read_bytes(), encoded.tobytes())
            self.assertEqual(before, {p.relative_to(latest): p.read_bytes() for p in latest.rglob("*") if p.is_file()})
            self.assertEqual({p.name for p in self.output.iterdir()}, {"latest", "last_failed"})

    def test_publish_failure_rolls_back_previous_latest(self):
        with self.post(DEFAULT_MAP.image_path.read_bytes()) as response:
            report = json.load(response)
        latest = Path(report["output_directory"])
        before = {p.relative_to(latest): p.read_bytes() for p in latest.rglob("*") if p.is_file()}
        original = Path.rename

        def fail_ready(path, target):
            if path.name == "ready":
                raise OSError("test publish failure")
            return original(path, target)

        with patch("scripts.http_map_server.Path.rename", new=fail_ready):
            with self.assertRaises(HTTPError) as failure:
                self.post(DEFAULT_MAP.image_path.read_bytes())
        failure.exception.close()
        self.assertEqual(before, {p.relative_to(latest): p.read_bytes() for p in latest.rglob("*") if p.is_file()})
        self.assertEqual({p.name for p in self.output.iterdir()}, {"latest"})

    def test_invalid_second_route_keeps_map_and_publishes_no_partial_routes(self):
        calls = []

        def fail_second(*args):
            calls.append(1)
            if len(calls) == 2:
                raise ValueError("경로 좌표가 검출된 도로 영역을 벗어났습니다. (261, 292)")
            return connect_waypoints(*args)

        with patch("src.map_processing.path_visualizer.connect_waypoints", side_effect=fail_second):
            with self.post(DEFAULT_MAP.image_path.read_bytes()) as response:
                report = json.load(response)
        self.assertEqual(len(calls), 2)
        self.assertEqual(report["status"], "map_generated")
        self.assertEqual(report["route_generation"]["status"], "failed")
        self.assertIn("(261, 292)", report["route_generation"]["error"])
        self.assertNotIn("cases", report["route_generation"])
        folder = Path(report["output_directory"])
        self.assertTrue((folder / "semantic.json").is_file())
        self.assertFalse((folder / "routes").exists())
        self.assertFalse(list(folder.glob(".route-build-*")))
        with urlopen(self.url + report["files"]["semantic"]) as response:
            self.assertEqual(json.load(response)["map_id"], DEFAULT_MAP.map_id)

    def test_new_aspect_and_camera_frame_uses_new_ocr_without_old_mask(self):
        frame = np.full((960, 1280, 3), 255, np.uint8)
        frame[120:840] = load_map_image(DEFAULT_MAP.image_path)
        ok, encoded = cv2.imencode(".png", frame)
        self.assertTrue(ok)
        body = encoded.tobytes()
        with patch("scripts.http_map_server.transform_hints", side_effect=AssertionError("old hints used")), \
                patch("scripts.http_map_server.prepare_paper_image", side_effect=AssertionError("old mask used")):
            with self.post(body) as response:
                report = json.load(response)
        self.assertEqual(report["status"], "map_generated")
        self.assertEqual(report["image_size_px"], [1280, 960])
        folder = Path(report["output_directory"])
        self.assertEqual((folder / "source.png").read_bytes(), body)
        self.assertFalse((folder / "map.png").exists())
        labels = json.loads((folder / "labels.json").read_text())
        self.assertEqual(labels["buildings"][0]["hint"][1], self.hints["building_a"][1] + 120)

    def test_missing_ocr_label_returns_error_without_reusing_old_hints(self):
        original = self.ocr_report

        def missing(*args):
            report = original(*args)
            report["status"] = "needs_review"
            report["labels"][0]["status"] = "missing"
            (args[2] / "ocr_run" / "report.json").write_text(json.dumps(report), encoding="utf-8")
            return report

        with patch("scripts.generate_map.run_ocr", side_effect=missing):
            with self.assertRaises(HTTPError) as failure:
                self.post(DEFAULT_MAP.image_path.read_bytes())
        self.assertEqual(failure.exception.code, 422)
        with failure.exception as response:
            report = json.load(response)
        self.assertIn("building_a(missing)", report["error"])
        self.assertEqual(report["ocr"]["status"], "needs_review")
        self.assertIn("ocr_report", report["files"])
        self.assertNotIn("semantic", report["files"])

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
        self.assertTrue((Path(report["output_directory"]) / "source.png").is_file())
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
