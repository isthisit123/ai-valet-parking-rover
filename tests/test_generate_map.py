from contextlib import redirect_stdout
from hashlib import sha256
import io
import json
from copy import deepcopy
from pathlib import Path
from shutil import copyfile
import sys
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import MagicMock, patch

import cv2

from scripts.generate_map import capture_png, generate_map, read_camera_png
from scripts.ocr_buildings import LABEL_IDS
from src.map_processing.map_config import DEFAULT_MAP, MapConfig


class GenerateMapTest(unittest.TestCase):
    def setUp(self):
        root = Path(self.enterContext(TemporaryDirectory()))
        self.config = MapConfig("ocr_test_map", root / "ocr_test_map.png",
                                root / "ocr_test_map_labels.json", root / "output")
        copyfile(DEFAULT_MAP.image_path, self.config.image_path)
        self.diagnostics = root / "ocr"
        labels = json.loads(DEFAULT_MAP.label_path.read_text(encoding="utf-8"))
        hints = {item["id"]: item["hint"] for key in ("buildings", "gates", "parking_spaces")
                 for item in labels[key]}
        self.report = {
            "source_sha256": sha256(self.config.image_path.read_bytes()).hexdigest(),
            "status": "all_required_labels_recognized",
            "image_size_px": [1280, 720],
            "required_ids": list(LABEL_IDS.values()),
            "labels": [{"id": key, "status": "recognized", "center_candidate_px": hint}
                       for key, hint in hints.items()],
        }

    def test_builds_new_map_from_png_without_existing_labels(self):
        self.assertFalse(self.config.label_path.exists())
        # 원본 사진은 출구가 배경에 닿으므로 사진에 대응하는 외곽 설정을 자동 적용한다.
        copyfile(DEFAULT_MAP.image_path.with_name("parking_map_ocr.png"), self.config.image_path)
        copyfile(DEFAULT_MAP.image_path.with_name("parking_map_real_preparation.json"),
                 self.config.image_path.with_name("parking_map_real_preparation.json"))
        original_image = self.config.image_path.read_bytes()
        self.report["source_sha256"] = sha256(original_image).hexdigest()
        with patch("scripts.generate_map.run_ocr", return_value=self.report), redirect_stdout(io.StringIO()):
            generate_map(self.config, Path(sys.executable), ocr_output_root=self.diagnostics)
        labels = json.loads(self.config.label_path.read_text(encoding="utf-8"))
        semantic = json.loads(self.config.semantic_path.read_text(encoding="utf-8"))
        self.assertEqual(semantic["map_id"], "ocr_test_map")
        self.assertEqual(semantic["map_image_sha256"], self.report["source_sha256"])
        self.assertEqual({item["id"] for item in semantic["regions"]}, set(LABEL_IDS.values()))
        self.assertEqual([item["id"] for item in labels["parking_spaces"] if item["type"] == "accessible"],
                         ["space_3", "space_6"])
        self.assertIn("paper_corners_px", labels)
        self.assertEqual(self.config.image_path.read_bytes(), original_image)
        self.assertTrue(self.config.preview_path.is_file())

    def test_invalid_ocr_or_color_match_leaves_existing_files_alone(self):
        self.config.label_path.write_bytes(b"old labels")
        self.config.output_dir.mkdir()
        self.config.semantic_path.write_bytes(b"old map")
        ambiguous = deepcopy(self.report)
        ambiguous["labels"][0]["status"] = "ambiguous"
        ambiguous["status"] = "needs_review"
        mismatched = deepcopy(self.report)
        mismatched["labels"][0]["center_candidate_px"] = [5, 5]
        for report, error in ((ambiguous, r"building_a\(ambiguous\)"), (mismatched, "검은색 영역이 0개")):
            with self.subTest(error=error):
                with patch("scripts.generate_map.run_ocr", return_value=report), redirect_stdout(io.StringIO()):
                    with self.assertRaisesRegex(ValueError, error):
                        generate_map(self.config, Path(sys.executable), replace=True, ocr_output_root=self.diagnostics)
                self.assertEqual(self.config.label_path.read_bytes(), b"old labels")
                self.assertEqual(self.config.semantic_path.read_bytes(), b"old map")
                self.assertFalse(self.config.graph_path.exists())

    def test_camera_capture_replaces_png_and_map_with_matching_hashes(self):
        self.config.image_path.unlink()
        frame = cv2.imread(str(DEFAULT_MAP.image_path))

        def report_for_capture(path, *_):
            return {**self.report, "source_sha256": sha256(path.read_bytes()).hexdigest()}

        with patch("scripts.generate_map.cv2.VideoCapture") as open_camera, \
                patch("scripts.generate_map.capture_png", side_effect=lambda index: read_camera_png(index, cv2.CAP_DSHOW)), \
                patch("scripts.generate_map.run_ocr", side_effect=report_for_capture), \
                redirect_stdout(io.StringIO()):
            camera = open_camera.return_value
            camera.read.return_value = (True, frame)
            generate_map(self.config, Path(sys.executable), camera_index=1, ocr_output_root=self.diagnostics)
            old_image = self.config.image_path.read_bytes()
            frame[0, 0] ^= 255
            generate_map(self.config, Path(sys.executable), camera_index=1, replace=True,
                         ocr_output_root=self.diagnostics)

        self.assertEqual(open_camera.call_count, 2)
        open_camera.assert_called_with(1, cv2.CAP_DSHOW)
        self.assertEqual(camera.release.call_count, 2)
        image_bytes = self.config.image_path.read_bytes()
        self.assertNotEqual(old_image, image_bytes)
        self.assertEqual(cv2.imread(str(self.config.image_path))[0, 0].tolist(), frame[0, 0].tolist())
        digest = sha256(image_bytes).hexdigest()
        labels = json.loads(self.config.label_path.read_text(encoding="utf-8"))
        semantic = json.loads(self.config.semantic_path.read_text(encoding="utf-8"))
        self.assertEqual(labels["image_sha256"], digest)
        self.assertEqual(semantic["map_image_sha256"], digest)

    def test_camera_or_ocr_failure_preserves_previous_png_and_results(self):
        previous = {
            self.config.image_path: self.config.image_path.read_bytes(),
            self.config.label_path: b"old labels",
            self.config.graph_path: b"old graph",
            self.config.preview_path: b"old preview",
            self.config.semantic_path: b"old map",
        }
        self.config.output_dir.mkdir()
        for path, data in previous.items():
            path.write_bytes(data)

        with patch("scripts.generate_map.cv2.VideoCapture") as open_camera, \
                patch("scripts.generate_map.capture_png", side_effect=lambda index: read_camera_png(index, cv2.CAP_DSHOW)), \
                patch("scripts.generate_map.run_ocr", side_effect=ValueError("OCR failed")) as run_ocr, \
                redirect_stdout(io.StringIO()):
            camera = open_camera.return_value
            camera.read.return_value = (False, None)
            with self.assertRaisesRegex(OSError, "프레임을 읽지 못했습니다"):
                generate_map(self.config, Path(sys.executable), camera_index=0, replace=True,
                             ocr_output_root=self.diagnostics)
            run_ocr.assert_not_called()

            camera.read.return_value = (True, cv2.imread(str(self.config.image_path)))
            with self.assertRaisesRegex(ValueError, "OCR failed"):
                generate_map(self.config, Path(sys.executable), camera_index=0, replace=True,
                             ocr_output_root=self.diagnostics)

        self.assertEqual(camera.release.call_count, 2)
        for path, data in previous.items():
            self.assertEqual(path.read_bytes(), data)

    def test_camera_timeout_stops_unresponsive_capture(self):
        with patch("scripts.generate_map.get_context") as context_factory, redirect_stdout(io.StringIO()):
            context = context_factory.return_value
            receiver, sender = MagicMock(), MagicMock()
            context.Pipe.return_value = (receiver, sender)
            receiver.poll.return_value = False
            process = context.Process.return_value
            process.is_alive.return_value = True

            with self.assertRaisesRegex(TimeoutError, "카메라 0번이 15초 동안 응답하지 않았습니다"):
                capture_png(0)

        receiver.recv.assert_not_called()
        process.kill.assert_called_once()
        process.close.assert_called_once()
