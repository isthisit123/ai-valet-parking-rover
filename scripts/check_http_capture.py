"""HTTP 수신 원본 PNG를 검사하고 기존 로컬 OCR 지도 생성을 별도로 시험한다."""

import argparse
from hashlib import sha256
import json
import math
from pathlib import Path
from tempfile import mkdtemp

import cv2

from scripts.generate_map import default_ocr_python, generate_map
from scripts.http_map_server import decode_png, write_json
from scripts.ocr_buildings import LABEL_IDS
from src.map_processing.map_config import MapConfig


def check_capture(capture_dir, ocr_python, *, min_score=0.75, preparation_path=None):
    capture_dir = Path(capture_dir).resolve()
    if not capture_dir.is_dir():
        raise ValueError(f"HTTP 캡처 폴더가 없습니다: {capture_dir}")
    if not math.isfinite(min_score) or not 0 <= min_score <= 1:
        raise ValueError("최소 OCR 신뢰도는 0~1 범위여야 합니다.")
    # 재시험마다 별도 폴더를 사용해 수신 자료와 이전 검사 결과를 보존한다.
    output = Path(mkdtemp(prefix="ocr_check_", dir=capture_dir))
    report = {"status": "failed", "stage": "input", "capture_directory": str(capture_dir),
              "output_directory": str(output), "files": {"report": "report.json"}}
    try:
        received = json.loads((capture_dir / "report.json").read_text(encoding="utf-8-sig"))
        if not isinstance(received, dict):
            raise ValueError("수신 report.json은 JSON 객체여야 합니다.")
        body = (capture_dir / "source.png").read_bytes()
        image = decode_png(body)
        digest = sha256(body).hexdigest()
        size = list(image.shape[1::-1])
        report["input"] = {"image_size_px": size, "byte_length": len(body), "sha256": digest,
                           "coordinate_frame": "map_image", "origin": "top_left",
                           "axis_directions": {"x": "right", "y": "down"}, "unit": "pixel"}
        if received.get("source_sha256") != digest:
            raise ValueError("수신 보고서의 해시와 source.png가 다릅니다. 원본 변경 여부를 확인하세요.")
        if received.get("image_size_px") != size:
            raise ValueError("수신 보고서의 해상도와 source.png가 다릅니다.")
        report["input"]["matches_http_report"] = True
        report["stage"] = "ocr_map"
        # 디코딩 이미지를 다시 인코딩하지 않고 수신한 바이트 그대로 사용한다.
        source = output / "source.png"
        source.write_bytes(body)
        config = MapConfig("parking_map_http_check", source, output / "labels.json", output / "map")
        generate_map(config, Path(ocr_python).resolve(), min_score=min_score,
                     ocr_output_root=output / "ocr", preparation_path=preparation_path)
        semantic = json.loads(config.semantic_path.read_text(encoding="utf-8"))
        ids = [item["id"] for item in semantic["regions"]]
        if len(ids) != len(LABEL_IDS) or set(ids) != set(LABEL_IDS.values()):
            raise ValueError("의미 지도에 시설 ID 10개가 각각 한 번씩 필요합니다.")
        if semantic["map_image_sha256"] != digest:
            raise ValueError("생성한 의미 지도와 수신 PNG의 해시가 다릅니다.")
        graph = json.loads(config.graph_path.read_text(encoding="utf-8"))
        report.update(status="check_passed", stage="complete", map_id=config.map_id,
                      mapped_ids=sorted(ids), graph={"nodes": len(graph["nodes"]), "edges": len(graph["edges"])})
        report["files"].update(source="source.png", labels="labels.json", graph="map/graph.json",
                               preview="map/graph_preview.png", semantic="map/semantic.json")
    except (OSError, ValueError, TypeError, KeyError, cv2.error) as error:
        report["error"] = str(error)
    write_json(output / "report.json", report)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture-dir", required=True, type=Path, help="source.png와 수신 report.json이 있는 폴더")
    parser.add_argument("--ocr-python", type=Path, default=default_ocr_python())
    parser.add_argument("--min-score", type=float, default=0.75)
    parser.add_argument("--paper-preparation", type=Path, help="이 수신 PNG와 해시가 일치하는 종이 외곽 설정")
    args = parser.parse_args()
    try:
        report = check_capture(args.capture_dir, args.ocr_python, min_score=args.min_score,
                               preparation_path=args.paper_preparation)
    except (OSError, ValueError) as error:
        parser.exit(1, f"입력 검사 실패: {error}\n")
    if "input" in report:
        info = report["input"]
        print(f"수신 PNG: {info['image_size_px']} px, {info['byte_length']} bytes, SHA-256 {info['sha256']}")
    print(f"검사 결과: {report['output_directory']}")
    if report["status"] != "check_passed":
        parser.exit(1, f"검사 실패 ({report['stage']}): {report['error']}\n")
    print(f"검사 통과: 이름표·시설 {len(report['mapped_ids'])}개, "
          f"노드 {report['graph']['nodes']}개, 간선 {report['graph']['edges']}개")


if __name__ == "__main__":
    main()
