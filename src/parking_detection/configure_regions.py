"""고정 카메라 사진 위에서 지도 ID별 사각형 ROI를 수동 지정한다."""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from src.map_processing.map_config import DEFAULT_MAP
from src.map_processing.parking_selector import load_parking_map
from .onnx_occupancy import read_image, validate_regions


def main():
    parser = argparse.ArgumentParser(description="사진 위 6개 주차칸 ROI 설정 (OpenCV 창)")
    parser.add_argument("--image", required=True, type=Path, help="실제 사용할 고정 카메라 사진")
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--verified-empty-reference", action="store_true",
                        help="이 사진의 모든 칸이 비어 있고 카메라 위치가 고정되었음을 직접 확인")
    args = parser.parse_args()
    if args.output.exists():
        parser.error("기존 설정을 보호합니다. 새 --output 경로를 지정하세요.")
    map_data, _ = load_parking_map()
    image = read_image(args.image)
    reference = read_image(DEFAULT_MAP.image_path)
    for region in map_data["regions"]:
        if region.get("role") in ("parking_normal", "parking_accessible"):
            x, y, w, h = region["bbox"]
            cv2.rectangle(reference, (x, y), (x+w, y+h), (0, 0, 255), 2)
            cv2.putText(reference, region["id"], (x, y+20), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (255, 255, 255), 2)
    height, width = image.shape[:2]
    scale = min(1, 1100 / width, 700 / height)
    display = cv2.resize(image, (round(width*scale), round(height*scale)))
    regions = []
    points = []

    def mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(points) < 4:
            points.append([min(width-1, round(x/scale)), min(height-1, round(y/scale))])

    window = "Camera ROI: 4 corners, ENTER accept, R redo, ESC cancel"
    cv2.namedWindow(window)
    cv2.setMouseCallback(window, mouse)
    try:
        cv2.imshow("Map IDs (reference only)", cv2.resize(reference, None, fx=0.6, fy=0.6))
        for node in sorted(map_data["nodes"], key=lambda n: n.get("region_id", "")):
            if node.get("role") not in ("parking_normal", "parking_accessible"):
                continue
            space_id = node["region_id"]
            points.clear()
            while True:
                preview = display.copy()
                cv2.putText(preview, f"{space_id}: click 4 corners in order", (12, 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                for saved in regions:
                    polygon = np.round(np.array(saved["polygon_px"]) * scale).astype(np.int32)
                    cv2.polylines(preview, [polygon], True, (0, 255, 0), 2)
                if points:
                    polygon = np.round(np.array(points) * scale).astype(np.int32)
                    cv2.polylines(preview, [polygon], len(points) == 4, (0, 200, 255), 2)
                cv2.imshow(window, preview)
                key = cv2.waitKey(30) & 0xFF
                if key == 27 or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                    print("취소했습니다. 설정을 저장하지 않았습니다.")
                    return
                if key in (ord("r"), ord("R")):
                    points.clear()
                if key in (10, 13) and len(points) == 4:
                    config = {
                        "map_id": map_data["map_id"], "coordinate_frame": "camera_image",
                        "image_size_px": [width, height],
                        "empty_reference_verified": args.verified_empty_reference,
                        "spaces": regions + [{"parking_space_id": space_id, "polygon_px": list(points)}],
                    }
                    try:
                        validate_regions(config, map_data, image)
                    except ValueError as exc:
                        print(exc, "R 키로 다시 지정하세요.")
                        continue
                    regions = config["spaces"]
                    break
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(f"ROI 설정 저장: {args.output}")
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
