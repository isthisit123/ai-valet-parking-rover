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
from unittest.mock import patch

from scripts.generate_map import generate_map
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
