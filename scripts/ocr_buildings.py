"""A/B, P1~P6, EN/EX 이름표 OCR 시험. 지도 힌트와 src는 자동 변경하지 않는다."""

import argparse
import hashlib
import json
import math
import unicodedata
import uuid
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np


BASE_DIR = Path(__file__).resolve().parents[1]
LABEL_IDS = {"A": "building_a", "B": "building_b",
             **{f"P{i}": f"space_{i}" for i in range(1, 7)}, "EN": "entrance", "EX": "exit"}
BUILDING_IDS = ("building_a", "building_b")
BRIGHT_TEXT_HSV_LOWER = (0, 0, 130)
BRIGHT_TEXT_HSV_UPPER = (179, 110, 255)
PATCH_UPSCALE = 3


def identify_label(text):
    normalized = unicodedata.normalize("NFKC", text).strip().upper()
    # I/1, B/8 등은 추측해 바꾸지 않는다. 규칙 밖 글자는 확인 대상으로 남긴다.
    return LABEL_IDS.get(normalized)


def analyze_records(records, image_size, *, min_score=0.75, buildings_only=False):
    if not math.isfinite(min_score) or not 0 <= min_score <= 1:
        raise ValueError("최소 OCR 신뢰도는 0~1 범위여야 합니다.")
    candidates = {label_id: [] for label_id in LABEL_IDS.values()}
    normalized = []
    width, height = image_size
    for record in records:
        text, score = record["text"], float(record["score"])
        box = np.asarray(record["polygon_px"], dtype=float)
        if (not isinstance(text, str) or not math.isfinite(score) or not 0 <= score <= 1
                or box.shape != (4, 2) or not np.isfinite(box).all()):
            raise ValueError("OCR 엔진 결과의 글자/점수/사각형 형식이 잘못되었습니다.")
        if (box < 0).any() or (box[:, 0] >= width).any() or (box[:, 1] >= height).any():
            raise ValueError("OCR 글자 위치가 원본 이미지 범위를 벗어났습니다.")
        label_id = identify_label(text)
        item = {"text": text, "score": score, "polygon_px": box.tolist(),
                "center_px": box.mean(axis=0).tolist(), "label_id": label_id,
                "box_kind": record.get("box_kind", "text_detection")}
        normalized.append(item)
        if label_id:
            candidates[label_id].append(item)
    labels = []
    for label_id, matches in candidates.items():
        accepted = [item for item in matches if item["score"] >= min_score]
        if len(matches) > 1:
            # 낮은 점수의 같은 ID도 중복 가능성으로 남겨 잘못된 자동 확정을 막는다.
            state = "ambiguous"
        elif accepted:
            state = "recognized"
        elif matches:
            state = "low_confidence"
        else:
            state = "missing"
        result = {"id": label_id, "status": state, "matches": matches}
        if state == "recognized":
            result["center_candidate_px"] = accepted[0]["center_px"]
        labels.append(result)
    required = list(BUILDING_IDS) if buildings_only else list(candidates)
    return {
        "schema_version": "0.2-ocr-test",
        "status": "all_required_labels_recognized" if all(
            item["status"] == "recognized" for item in labels if item["id"] in required
        ) else "needs_review",
        "coordinate_frame": "input_image", "unit": "pixel", "origin": "top_left",
        "axis_directions": {"x": "right", "y": "down"}, "image_size_px": [width, height],
        "min_score": min_score, "required_ids": required, "label_rules": LABEL_IDS,
        "labels": labels,
        "building_labels": [item for item in labels if item["id"] in BUILDING_IDS],
        "all_text": normalized,
        "facility_region_matching_completed": False,
        "note": "원본 글자 또는 색상 영역의 중심 후보입니다. 전체 시설 매핑과 기존 라벨 JSON 변경은 수행하지 않습니다.",
    }


def bright_text_boxes(crop):
    """시설 내부의 밝은 글자 후보를 찾는다. 배경과 연결된 가장자리 조각은 제외한다."""
    height, width = crop.shape[:2]
    mask = cv2.inRange(cv2.cvtColor(crop, cv2.COLOR_BGR2HSV), BRIGHT_TEXT_HSV_LOWER, BRIGHT_TEXT_HSV_UPPER)
    _, _, stats, _ = cv2.connectedComponentsWithStats(mask)
    boxes = []
    for x, y, w, h, area in stats[1:]:
        if (area < 6 or area > width*height*0.15 or w < 2 or h < 6
                or x <= 1 or y <= 1 or x+w >= width-1 or y+h >= height-1):
            continue
        boxes.append(tuple(map(int, (x, y, w, h))))
    return boxes


def group_text_boxes(boxes):
    """P와 1처럼 같은 줄에 인접한 글자 조각을 단어 단위로 묶는다."""
    parents = list(range(len(boxes)))

    def root(index):
        while parents[index] != index:
            parents[index] = parents[parents[index]]
            index = parents[index]
        return index

    for index, (x, y, w, h) in enumerate(boxes):
        for other in range(index+1, len(boxes)):
            ox, oy, ow, oh = boxes[other]
            vertical_overlap = max(0, min(y+h, oy+oh)-max(y, oy))
            horizontal_gap = max(0, max(x, ox)-min(x+w, ox+ow))
            if (min(h, oh) / max(h, oh) >= 0.5
                    and vertical_overlap / min(h, oh) >= 0.5
                    and horizontal_gap <= max(3, max(h, oh)*0.6)):
                parents[root(other)] = root(index)
    groups = {}
    for index, box in enumerate(boxes):
        groups.setdefault(root(index), []).append(box)
    merged = []
    for group in groups.values():
        left = min(box[0] for box in group)
        top = min(box[1] for box in group)
        right = max(box[0]+box[2] for box in group)
        bottom = max(box[1]+box[3] for box in group)
        merged.append((left, top, right-left, bottom-top))
    return sorted(merged, key=lambda box: (box[1], box[0]))


def merge_overlapping_glyphs(records):
    """같은 글자의 겹치는 그림자 후보만 병합한다. 떨어진 동일 ID는 중복으로 남긴다."""
    merged = []
    for record in sorted(records, key=lambda item: item["score"], reverse=True):
        points = np.asarray(record["polygon_px"])
        left, top = points.min(axis=0)
        right, bottom = points.max(axis=0)
        duplicate = None
        for other in merged:
            if identify_label(record["text"]) != identify_label(other["text"]):
                continue
            other_points = np.asarray(other["polygon_px"])
            ox, oy = other_points.min(axis=0)
            ex, ey = other_points.max(axis=0)
            overlap = max(0, min(right, ex)-max(left, ox)) * max(0, min(bottom, ey)-max(top, oy))
            smaller = min((right-left)*(bottom-top), (ex-ox)*(ey-oy))
            if smaller > 0 and overlap / smaller >= 0.25:
                duplicate = other
                break
        if duplicate is None:
            merged.append(record)
        else:
            duplicate.setdefault("supporting_observations", []).append(record)
    return merged


def collect_records(engine, image, *, min_score=0.75, region_fallback=True, diagnostics=None):
    diagnostics = diagnostics if diagnostics is not None else []
    output = engine(image)
    records = []
    if output.txts is not None and len(output.txts):
        for text, score, box in zip(output.txts, output.scores, output.boxes, strict=True):
            records.append({"text": text, "score": float(score), "polygon_px": np.asarray(box).tolist(),
                            "box_kind": "text_detection"})
    if not region_fallback:
        return records
    # 한 글자를 전체 프레임 검출기가 놓칠 때 색상 영역을 잘라 직접 읽는다.
    # 기존 함수만 재사용하며 힌트/기존 시설 위치는 입력하지 않는다.
    from src.map_processing.semantic_mapper import detect_regions
    from src.map_processing.map_config import REAL_MAP_ID
    normal, accessible, dark, _ = detect_regions(image, map_id=REAL_MAP_ID)
    parking_ids = {f"space_{i}" for i in range(1, 7)}
    groups = ((normal + accessible, parking_ids), (dark, {*BUILDING_IDS, "entrance", "exit"}))
    for regions, allowed in groups:
        for region in regions:
            x, y, w, h = region["bbox"]
            def inside(record):
                center = np.asarray(record["polygon_px"]).mean(axis=0)
                return x <= center[0] < x+w and y <= center[1] < y+h
            if any(identify_label(r["text"]) in allowed and r["score"] >= min_score and inside(r)
                   for r in records):
                continue
            crop = image[y:y+h, x:x+w]
            recognized = engine(crop, use_det=False, use_cls=False)
            diagnostics.append({"kind": "whole_region", "bbox_xywh_px": [x, y, w, h],
                                "texts": recognized.txts, "scores": recognized.scores})
            if recognized.txts is not None and len(recognized.txts) == 1:
                text, score = recognized.txts[0], float(recognized.scores[0])
                if identify_label(text) in allowed and score >= min_score:
                    records.append({"text": text, "score": score, "box_kind": "color_region_recognition",
                                    "polygon_px": [[x, y], [x+w-1, y], [x+w-1, y+h-1], [x, y+h-1]]})
                    continue
            patches = []
            for px, py, pw, ph in group_text_boxes(bright_text_boxes(crop)):
                left, top = max(0, px-4), max(0, py-4)
                right, bottom = min(w, px+pw+4), min(h, py+ph+4)
                patch = cv2.resize(crop[top:bottom, left:right], None, fx=PATCH_UPSCALE,
                                   fy=PATCH_UPSCALE, interpolation=cv2.INTER_CUBIC)
                result = engine(patch, use_det=False, use_cls=False)
                diagnostics.append({"kind": "bright_text_patch", "bbox_xywh_px": [x+px, y+py, pw, ph],
                                    "texts": result.txts, "scores": result.scores})
                if result.txts is None or len(result.txts) != 1:
                    continue
                text, score = result.txts[0], float(result.scores[0])
                if identify_label(text) in allowed and score >= min_score:
                    patches.append({"text": text, "score": score, "box_kind": "bright_text_patch",
                                    "polygon_px": [[x+px, y+py], [x+px+pw-1, y+py],
                                                   [x+px+pw-1, y+py+ph-1], [x+px, y+py+ph-1]]})
            # 한 시설 안에서 서로 다른 ID가 읽히면 어느 쪽도 자동 확정하지 않는다.
            if len({identify_label(item["text"]) for item in patches}) <= 1:
                records.extend(merge_overlapping_glyphs(patches))
            elif patches:
                diagnostics.append({"kind": "conflicting_patch_labels", "bbox_xywh_px": [x, y, w, h],
                                    "candidates": patches})
    return records


def run(image_path, output_root, *, min_score=0.75, buildings_only=False, region_fallback=True):
    if not math.isfinite(min_score) or not 0 <= min_score <= 1:
        raise ValueError("최소 OCR 신뢰도는 0~1 범위여야 합니다.")
    image_bytes = image_path.read_bytes()
    image = cv2.imdecode(np.frombuffer(image_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"이미지를 읽을 수 없습니다: {image_path}")
    try:
        from rapidocr import RapidOCR
    except ImportError as exc:
        raise RuntimeError("OCR 전용 .venv-ocr 환경에 requirements-ocr.txt를 설치한 뒤 실행하세요.") from exc
    engine = RapidOCR(params={"Global.text_score": 0.0})
    diagnostics = []
    records = collect_records(engine, image, min_score=min_score, region_fallback=region_fallback,
                              diagnostics=diagnostics)
    report = analyze_records(records, [image.shape[1], image.shape[0]],
                             min_score=min_score, buildings_only=buildings_only)
    report.update(engine="RapidOCR", source_file_name=image_path.name,
                  source_sha256=hashlib.sha256(image_bytes).hexdigest(),
                  processed_at=datetime.now(timezone.utc).isoformat(), region_fallback=region_fallback,
                  region_attempts=diagnostics)
    folder = output_root / (datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ_") + uuid.uuid4().hex[:8])
    folder.mkdir(parents=True, exist_ok=False)
    (folder / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    preview = image.copy()
    accepted_ids = {item["id"] for item in report["labels"] if item["status"] == "recognized"}
    for item in report["all_text"]:
        color = (0, 200, 0) if item["label_id"] in accepted_ids else (0, 165, 255)
        points = np.round(item["polygon_px"]).astype(np.int32)
        cv2.polylines(preview, [points], True, color, 2)
        x, y = points.min(axis=0)
        # 인식 원문(한글 등)은 JSON에 보존하고 미리보기는 ASCII ID/점수로 표시한다.
        label = f"{item['label_id'] or 'other'} {item['score']:.2f}"
        cv2.putText(preview, label, (int(x), max(18, int(y)-6)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    ok, encoded = cv2.imencode(".png", preview)
    if not ok:
        raise OSError("OCR 미리보기 PNG 인코딩 실패")
    (folder / "preview.png").write_bytes(encoded.tobytes())
    print("OCR:", report["status"])
    for item in report["labels"]:
        print(item["id"], item["status"], item.get("center_candidate_px"))
    print("Results:", folder.resolve())
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", required=True, type=Path)
    parser.add_argument("--output-dir", type=Path, default=BASE_DIR / "data/ocr_building_tests")
    parser.add_argument("--min-score", type=float, default=0.75, help="초기 실험 기준이며 현장 검증 필요")
    parser.add_argument("--buildings-only", action="store_true", help="현재 A/B만 붙였다면 두 건물만 필수로 검사")
    parser.add_argument("--no-region-fallback", action="store_true", help="색상 영역 보완 인식 없이 전체 이미지 OCR만 실행")
    parser.add_argument("--allow-single-letter", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    try:
        report = run(args.image, args.output_dir, min_score=args.min_score,
                     buildings_only=args.buildings_only, region_fallback=not args.no_region_fallback)
    except (OSError, ValueError, RuntimeError) as exc:
        parser.exit(1, f"OCR 실행 실패: {exc}\n")
    # 하나라도 누락/중복이면 결과는 남기고 성공 종료는 하지 않는다.
    raise SystemExit(0 if report["status"] == "all_required_labels_recognized" else 2)


if __name__ == "__main__":
    main()
