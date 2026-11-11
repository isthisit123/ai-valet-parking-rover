import unittest
from types import SimpleNamespace

import cv2
import numpy as np

from scripts.ocr_buildings import (
    LABEL_IDS, analyze_records, bright_text_boxes, collect_records, group_text_boxes, identify_label,
    merge_overlapping_glyphs,
)


def record(text, score=0.95):
    return {"text": text, "score": score, "polygon_px": [[10, 10], [90, 10], [90, 30], [10, 30]]}


class BuildingOcrTest(unittest.TestCase):
    def test_exact_ten_label_rules_and_no_guessing(self):
        for text, label_id in LABEL_IDS.items():
            self.assertEqual(identify_label(text), label_id)
        self.assertEqual(identify_label(" Ａ "), "building_a")
        self.assertEqual(identify_label("en"), "entrance")
        self.assertEqual(identify_label("p2"), "space_2")
        self.assertEqual(identify_label("Ｐ６"), "space_6")
        for text in ("BUILDING_A", "BUILDING_B", "I", "O", "8", "12", "E N", "ENEX",
                     "1", "2", "3", "4", "5", "6", "P0", "P7", "P01", "PI"):
            self.assertIsNone(identify_label(text))

    def test_both_labels_produce_unverified_text_center_candidates(self):
        report = analyze_records([record("A"), record("B")], [100, 100], buildings_only=True)
        self.assertEqual(report["status"], "all_required_labels_recognized")
        self.assertEqual(report["building_labels"][0]["center_candidate_px"], [50, 20])
        self.assertFalse(report["facility_region_matching_completed"])
        self.assertEqual(analyze_records([record("A"), record("B")], [100, 100])["status"], "needs_review")

    def test_all_ten_are_required_by_default(self):
        report = analyze_records([record(text) for text in LABEL_IDS], [100, 100])
        self.assertEqual(report["status"], "all_required_labels_recognized")
        self.assertEqual(len(report["labels"]), 10)

    def test_missing_low_confidence_and_duplicate_labels_do_not_pass(self):
        for records, state in (([], "missing"), ([record("A", 0.5)], "low_confidence"),
                               ([record("A"), record("A", 0.4)], "ambiguous")):
            with self.subTest(state=state):
                report = analyze_records(records, [100, 100])
                self.assertEqual(report["status"], "needs_review")
                self.assertEqual(report["building_labels"][0]["status"], state)
                self.assertNotIn("center_candidate_px", report["building_labels"][0])

    def test_invalid_scores_and_coordinates_are_rejected(self):
        for score in (float("nan"), -0.1, 1.1):
            with self.subTest(score=score), self.assertRaises(ValueError):
                analyze_records([record("A", score)], [100, 100])
        with self.assertRaises(ValueError):
            analyze_records([record("A")], [20, 20])

    def test_region_recognition_recovers_a_without_duplicate_b(self):
        image = np.full((300, 500, 3), 255, np.uint8)
        cv2.rectangle(image, (30, 30), (90, 80), (0, 0, 0), -1)
        cv2.rectangle(image, (310, 30), (370, 80), (0, 0, 0), -1)
        calls = []

        def engine(frame, **kwargs):
            calls.append(kwargs)
            if kwargs.get("use_det") is False:
                return SimpleNamespace(txts=("A",), scores=(0.99,))
            return SimpleNamespace(txts=("B",), scores=(0.99,),
                                   boxes=[[[320, 40], [340, 40], [340, 60], [320, 60]]])

        records = collect_records(engine, image)
        self.assertEqual(sorted(r["text"] for r in records), ["A", "B"])
        self.assertEqual(len(calls), 2)
        self.assertEqual(next(r for r in records if r["text"] == "A")["box_kind"], "color_region_recognition")

    def test_bright_text_boxes_exclude_border_background(self):
        crop = np.zeros((90, 150, 3), np.uint8)
        crop[:, :10] = 255
        crop[30:42, 40:48] = 255
        self.assertEqual(bright_text_boxes(crop), [(40, 30, 8, 12)])

    def test_adjacent_p_and_digit_are_grouped_but_other_lines_and_distant_text_are_not(self):
        boxes = [(10, 10, 23, 18), (36, 15, 13, 16), (100, 10, 15, 18), (10, 70, 15, 18)]
        self.assertEqual(group_text_boxes(boxes), [(10, 10, 39, 21), (100, 10, 15, 18), (10, 70, 15, 18)])
        self.assertEqual(group_text_boxes([]), [])
        self.assertEqual(group_text_boxes([(10, 10, 20, 30), (32, 10, 4, 6)]),
                         [(10, 10, 20, 30), (32, 10, 4, 6)])

    def test_only_overlapping_same_glyph_is_merged(self):
        first = record("P4", 0.99)
        shadow = record("P4", 0.95)
        separate = record("P4", 0.9)
        separate["polygon_px"] = [[110, 10], [190, 10], [190, 30], [110, 30]]
        merged = merge_overlapping_glyphs([shadow, separate, first])
        self.assertEqual(len(merged), 2)
        self.assertEqual(merged[0]["score"], 0.99)
        self.assertEqual(len(merged[0]["supporting_observations"]), 1)
        report = analyze_records(merged, [200, 100])
        self.assertEqual(next(r for r in report["labels"] if r["id"] == "space_4")["status"], "ambiguous")

    def test_small_bright_patch_recovers_label_after_whole_region_fails(self):
        image = np.full((300, 500, 3), 255, np.uint8)
        image[30:91, 30:111] = 0
        image[50:62, 60:68] = 255
        calls = []
        diagnostics = []

        def engine(frame, **kwargs):
            calls.append(frame.shape)
            if not kwargs:
                return SimpleNamespace(txts=None)
            if len(calls) == 2:
                return SimpleNamespace(txts=("unreadable",), scores=(0.9,))
            return SimpleNamespace(txts=("A",), scores=(0.99,))

        records = collect_records(engine, image, diagnostics=diagnostics)
        self.assertEqual(len(records), 1)
        self.assertEqual(records[0]["text"], "A")
        self.assertEqual(records[0]["box_kind"], "bright_text_patch")
        self.assertEqual([r["kind"] for r in diagnostics], ["whole_region", "bright_text_patch"])


if __name__ == "__main__":
    unittest.main()
