"""학습된 단일 car YOLOv8 ONNX를 기존 OpenCV로 실행한다."""

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np

from src.map_processing.parking_selector import load_parking_map


def read_image(path):
    image = cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"이미지를 읽을 수 없습니다: {path}")
    return image


class CarDetector:
    def __init__(self, model_path):
        self.net = cv2.dnn.readNetFromONNX(str(model_path))

    def detect(self, image):
        height, width = image.shape[:2]
        scale = min(640 / width, 640 / height)
        rw, rh = round(width * scale), round(height * scale)
        ox, oy = (640 - rw) // 2, (640 - rh) // 2
        canvas = np.full((640, 640, 3), 114, dtype=np.uint8)
        canvas[oy:oy + rh, ox:ox + rw] = cv2.resize(image, (rw, rh))
        self.net.setInput(cv2.dnn.blobFromImage(canvas, 1 / 255, swapRB=True))
        output = self.net.forward()
        if output.shape != (1, 5, 8400) or not np.isfinite(output).all():
            raise ValueError(f"지원하지 않거나 손상된 모델 출력: {output.shape}")
        boxes, scores = [], []
        # 낮은 점수 후보도 보존하여 빈칸 판정에서 제외한다.
        for cx, cy, bw, bh, confidence in output[0].T:
            if confidence < 0.1:
                continue
            x1 = float(np.clip((cx - bw / 2 - ox) / scale, 0, width))
            y1 = float(np.clip((cy - bh / 2 - oy) / scale, 0, height))
            x2 = float(np.clip((cx + bw / 2 - ox) / scale, 0, width))
            y2 = float(np.clip((cy + bh / 2 - oy) / scale, 0, height))
            if x2 > x1 and y2 > y1:
                boxes.append([x1, y1, x2 - x1, y2 - y1])
                scores.append(float(confidence))
        indices = np.asarray(cv2.dnn.NMSBoxes(boxes, scores, 0.1, 0.45)).reshape(-1)
        return [{"bbox_xywh_px": boxes[i], "confidence": scores[i]} for i in indices]


def validate_regions(config, map_data, image):
    if config.get("map_id") != map_data["map_id"]:
        raise ValueError("카메라 ROI와 의미 지도의 map_id가 다릅니다.")
    height, width = image.shape[:2]
    if config.get("coordinate_frame") != "camera_image" or config.get("image_size_px") != [width, height]:
        raise ValueError("카메라 ROI 좌표계 또는 해상도가 현재 영상과 다릅니다.")
    valid_ids = {node["region_id"] for node in map_data["nodes"]
                 if node.get("role") in ("parking_normal", "parking_accessible")}
    masks = {}
    for region in config["spaces"]:
        space_id = region["parking_space_id"]
        if space_id not in valid_ids or space_id in masks:
            raise ValueError(f"잘못되거나 중복된 ROI ID: {space_id}")
        points = np.asarray(region["polygon_px"], dtype=np.float64)
        if (points.ndim != 2 or points.shape[1] != 2 or len(points) < 3
                or not np.isfinite(points).all()
                or np.any(points < 0) or np.any(points[:, 0] >= width)
                or np.any(points[:, 1] >= height)):
            raise ValueError(f"잘못된 ROI 좌표: {space_id}")
        points = np.round(points).astype(np.int32)
        if not cv2.isContourConvex(points) or cv2.contourArea(points) < 100:
            raise ValueError(f"ROI는 100px² 이상의 볼록 다각형이어야 합니다: {space_id}")
        mask = np.zeros((height, width), np.uint8)
        cv2.fillPoly(mask, [points], 1)
        if any(np.any(mask & other) for other in masks.values()):
            raise ValueError("카메라 주차칸 ROI가 서로 겹칩니다.")
        masks[space_id] = mask.astype(bool)
    return masks


def build_snapshot(map_data, image, detections, observed_at, *, regions=None, empty_reference=None):
    """빈칸은 검증된 빈 배경 일치로만 판정하며, 누락 ROI는 unknown이다."""
    masks = validate_regions(regions, map_data, image) if regions is not None else {}
    if empty_reference is not None:
        if empty_reference.shape != image.shape:
            raise ValueError("빈 배경 사진과 현재 영상 해상도가 다릅니다.")
        if regions is None or regions.get("empty_reference_verified") is not True:
            raise ValueError("같은 고정 카메라의 빈 배경 사진을 확인한 ROI 설정이 필요합니다.")
        difference = np.max(np.abs(image.astype(np.int16) - empty_reference.astype(np.int16)), axis=2)
    spaces = []
    for node in map_data["nodes"]:
        if node.get("role") not in ("parking_normal", "parking_accessible"):
            continue
        space_id = node["region_id"]
        state = {"parking_space_id": space_id, "status": "unknown", "confidence": None}
        mask = masks.get(space_id)
        if mask is not None:
            matching_scores = []
            for detection in detections:
                x, y, w, h = detection["bbox_xywh_px"]
                # 픽셀 단위 반올림을 바깥 방향으로 적용한다.
                left, top = max(0, int(np.floor(x))), max(0, int(np.floor(y)))
                right = min(image.shape[1], int(np.ceil(x + w)))
                bottom = min(image.shape[0], int(np.ceil(y + h)))
                if right <= left or bottom <= top:
                    continue
                overlap = float(mask[top:bottom, left:right].sum())
                # 차체 높이/인접 칸 경계에서는 bbox 중심만으로 판단하지 않는다.
                if overlap / min(float(mask.sum()), float((right-left)*(bottom-top))) >= 0.1:
                    matching_scores.append(detection["confidence"])
            if matching_scores:
                score = max(matching_scores)
                if score >= 0.3:
                    state.update(status="occupied", confidence=score)
            elif empty_reference is not None:
                similarity = float(np.mean(difference[mask] <= 20))
                if similarity >= 0.98:
                    state.update(status="free", confidence=similarity)
                    state["evidence"] = "verified_empty_reference_similarity"
        spaces.append(state)
    return {
        "schema_version": "0.1-draft", "map_id": map_data["map_id"],
        "observed_at": observed_at, "coordinate_frame": "camera_image",
        "image_size_px": [image.shape[1], image.shape[0]], "spaces": spaces,
    }


def main():
    parser = argparse.ArgumentParser(description="ONNX 차량 검출과 주차칸 점유 스냅샷 생성")
    parser.add_argument("--model", type=Path, required=True)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--image", type=Path)
    source.add_argument("--camera", type=int)
    parser.add_argument("--regions", type=Path, help="같은 카메라의 칸 ID별 다각형 설정 JSON")
    parser.add_argument("--empty-reference", type=Path, help="같은 카메라에서 확인한 빈 주차장 사진")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--preview", type=Path)
    parser.add_argument("--save-frame", type=Path, help="ROI 설정/빈 배경 확인용 원본 프레임 PNG 저장")
    args = parser.parse_args()
    map_data, _ = load_parking_map()
    regions = json.loads(args.regions.read_text(encoding="utf-8-sig")) if args.regions else None
    reference = read_image(args.empty_reference) if args.empty_reference else None
    # 카메라를 열기 전에 모델 로드 실패를 드러낸다.
    detector = CarDetector(args.model)
    if args.image:
        image = read_image(args.image)
        # 예전 사진을 현재 관측으로 위장하지 않는다.
        observed_at = datetime.fromtimestamp(args.image.stat().st_mtime, timezone.utc).isoformat()
    else:
        capture = cv2.VideoCapture(args.camera)
        try:
            success, image = capture.read()
            if not success or image is None:
                raise RuntimeError(f"카메라 프레임을 읽지 못했습니다: {args.camera}")
            observed_at = datetime.now(timezone.utc).isoformat()
        finally:
            capture.release()
    detections = detector.detect(image)
    snapshot = build_snapshot(
        map_data, image, detections, observed_at, regions=regions, empty_reference=reference,
    )
    snapshot["detections"] = detections
    if args.save_frame:
        if args.save_frame.exists():
            raise FileExistsError("기존 기준 사진을 보호합니다. 새로운 --save-frame 경로를 지정하세요.")
        args.save_frame.parent.mkdir(parents=True, exist_ok=True)
        ok, encoded = cv2.imencode(".png", image)
        if not ok:
            raise OSError("원본 프레임 인코딩 실패")
        args.save_frame.write_bytes(encoded.tobytes())
    # 검증/추론 실패 시 출력하지 않는다. 이전 파일은 observed_at/TTL로 만료된다.
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_name(args.output.name + ".tmp")
    temporary.write_text(json.dumps(snapshot, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    temporary.replace(args.output)
    if args.preview:
        preview = image.copy()
        for detection in detections:
            x, y, w, h = map(round, detection["bbox_xywh_px"])
            cv2.rectangle(preview, (x, y), (x+w, y+h), (0, 200, 255), 2)
            cv2.putText(preview, f"car {detection['confidence']:.2f}", (x, max(y-6, 18)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
        args.preview.parent.mkdir(parents=True, exist_ok=True)
        ok, encoded = cv2.imencode(".png", preview)
        if not ok:
            raise OSError("검출 미리보기 저장 실패")
        args.preview.write_bytes(encoded.tobytes())
    print(f"검출 {len(detections)}개; 점유 결과: {args.output}")
    for space in snapshot["spaces"]:
        print(space["parking_space_id"], space["status"])


if __name__ == "__main__":
    main()
