"""지도 PNG에서 OCR 라벨, 도로 그래프, 의미 지도를 한 번에 생성한다."""

import argparse
from contextlib import ExitStack
from hashlib import sha256
import json
from pathlib import Path
import re
import subprocess
import sys
from tempfile import NamedTemporaryFile, TemporaryDirectory, mkdtemp

import cv2
import numpy as np

from scripts.ocr_buildings import LABEL_IDS
from src.map_processing.graph_exporter import build_road_graph, save_graph
from src.map_processing.map_config import (
    BASE_DIR, DEFAULT_MAP_ID, DEFAULT_OUTPUT_ROOT, OCR_ID_REVISION, OCR_LABEL_SCHEME,
    MapConfig, map_paths,
)
from src.map_processing.semantic_mapper import create_semantic_map


OCR_OUTPUT_ROOT = BASE_DIR / "data" / "ocr_building_tests"


def labels_from_report(report, map_id, image_digest):
    if report.get("source_sha256") != image_digest:
        raise ValueError("OCR 결과를 생성한 PNG와 지도 PNG가 다릅니다.")
    required = set(LABEL_IDS.values())
    labels = report["labels"]
    by_id = {item["id"]: item for item in labels}
    if set(by_id) != required or len(labels) != len(required):
        raise ValueError("OCR 보고서에 필수 시설 ID 10개가 각각 한 번씩 필요합니다.")
    unresolved = [f"{key}({item['status']})" for key, item in by_id.items()
                  if item["status"] != "recognized"]
    if unresolved:
        raise ValueError("OCR 이름표 확인 필요: " + ", ".join(unresolved))
    if (set(report.get("required_ids", [])) != required
            or report.get("status") != "all_required_labels_recognized"):
        raise ValueError("A/B, P1~P6, EN/EX 전체 OCR 검사를 통과해야 합니다.")

    def spec(label_id):
        hint = by_id[label_id]["center_candidate_px"]
        values = np.asarray(hint, dtype=float)
        if (values.shape != (2,) or not np.isfinite(values).all()
                or (values < 0).any() or (values >= report["image_size_px"]).any()):
            raise ValueError(f"OCR 힌트가 이미지 범위를 벗어났습니다: {label_id}")
        return {"id": label_id, "hint": values.tolist()}

    return {
        "map_id": map_id,
        "buildings": [spec(key) for key in ("building_a", "building_b")],
        "gates": [spec(key) for key in ("entrance", "exit")],
        "parking_spaces": [
            {**spec(f"space_{index}"), "type": "accessible" if index in (3, 6) else "normal"}
            for index in range(1, 7)
        ],
        "label_scheme": OCR_LABEL_SCHEME,
        "id_assignment_revision": OCR_ID_REVISION,
        "image_sha256": image_digest,
        "source_ocr_image_sha256": image_digest,
    }


def run_ocr(image_path, ocr_python, output_dir, min_score):
    result = subprocess.run(
        [str(ocr_python), "-B", "-X", "utf8", "-m", "scripts.ocr_buildings",
         "--image", str(image_path), "--output-dir", str(output_dir), "--min-score", str(min_score)],
        cwd=BASE_DIR, check=False,
    )
    reports = list(output_dir.glob("*/report.json"))
    if result.returncode not in (0, 2) or len(reports) != 1:
        raise ValueError("OCR 실행에 실패했습니다. OCR 전용 환경과 실행 로그를 확인하세요.")
    return json.loads(reports[0].read_text(encoding="utf-8"))


def find_paper_corners(image_path, digest, image_size, preparation_path=None):
    # 원본 사진과 해시가 같은 외곽점만 자동으로 재사용한다.
    paths = [preparation_path] if preparation_path is not None else sorted(
        image_path.parent.glob("*_preparation.json")
    )
    for path in paths:
        preparation = json.loads(path.read_text(encoding="utf-8-sig"))
        source_digest = preparation.get("source_sha256")
        if source_digest != digest and (preparation_path is None or source_digest is not None):
            if preparation_path is not None:
                raise ValueError("종이 외곽 설정을 만든 사진과 입력 PNG가 다릅니다.")
            continue
        if preparation["reference_size_px"] != image_size:
            raise ValueError("종이 외곽 설정과 지도 PNG의 해상도가 다릅니다.")
        return preparation["paper_corners_px"]
    return None


def publish_files(pairs, *, replace):
    # 결과가 모두 검증된 후에만 게시하고, 저장 중 실패하면 이전 파일을 되돌린다.
    with ExitStack() as stack:
        def stage(target, data):
            with NamedTemporaryFile(prefix=".map-", dir=target.parent, delete=False) as stream:
                pending = Path(stream.name)
                stack.callback(pending.unlink, missing_ok=True)
                stream.write(data)
            return pending

        pending = []
        for source, target in pairs:
            target.parent.mkdir(parents=True, exist_ok=True)
            backup = None
            if target.exists():
                if not replace:
                    raise FileExistsError(f"기존 결과가 있습니다. 갱신하려면 --replace를 사용하세요: {target}")
                backup = stage(target, target.read_bytes())
            pending.append((stage(target, source.read_bytes()), target, backup))
        published = []
        try:
            for temporary, target, backup in pending:
                if not replace and target.exists():
                    raise FileExistsError(f"기존 결과가 생겼습니다: {target}")
                temporary.replace(target)
                published.append((target, backup))
        except OSError:
            for target, backup in reversed(published):
                if backup is None:
                    target.unlink()
                else:
                    backup.replace(target)
            raise


def generate_map(config, ocr_python, *, replace=False, min_score=0.75,
                 ocr_output_root=OCR_OUTPUT_ROOT, preparation_path=None):
    destinations = (config.label_path, config.graph_path, config.preview_path, config.semantic_path)
    if not replace and any(path.exists() for path in destinations):
        raise FileExistsError(f"{config.map_id!r} 결과가 이미 있습니다. 갱신하려면 --replace를 사용하세요.")
    image_bytes = config.image_path.read_bytes()
    if not image_bytes.startswith(b"\x89PNG\r\n\x1a\n"):
        raise ValueError(f"PNG 지도 이미지가 필요합니다: {config.image_path}")
    digest = sha256(image_bytes).hexdigest()
    ocr_output_root.mkdir(parents=True, exist_ok=True)
    diagnostics = Path(mkdtemp(prefix="map_", dir=ocr_output_root))
    print(f"OCR 기록: {diagnostics}", flush=True)

    with TemporaryDirectory(prefix="rover_map_build_") as scratch:
        root = Path(scratch)
        staged = MapConfig(config.map_id, root / config.image_path.name, root / config.label_path.name, root / "map")
        staged.image_path.write_bytes(image_bytes)
        report = run_ocr(staged.image_path, ocr_python, diagnostics, min_score)
        labels = labels_from_report(report, config.map_id, digest)
        corners = find_paper_corners(config.image_path, digest, report["image_size_px"], preparation_path)
        if corners is not None:
            labels["paper_corners_px"] = corners
        staged.label_path.write_text(json.dumps(labels, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        graph, skeleton = build_road_graph(staged.image_path)
        save_graph(graph, skeleton, config=staged)
        create_semantic_map(
            image_path=staged.image_path, graph_path=staged.graph_path,
            label_path=staged.label_path, output_path=staged.semantic_path,
            expected_map_id=config.map_id,
        )
        if sha256(config.image_path.read_bytes()).hexdigest() != digest:
            raise ValueError("처리 중 지도 PNG가 바뀌었습니다. 현재 사진으로 다시 실행하세요.")
        sources = (staged.label_path, staged.graph_path, staged.preview_path, staged.semantic_path)
        publish_files(zip(sources, destinations, strict=True), replace=replace)


def default_ocr_python():
    env = BASE_DIR / ".venv-ocr"
    for path in (env / "Scripts/python.exe", env / "bin/python"):
        if path.is_file():
            return path
    return Path(sys.executable)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-id", default=DEFAULT_MAP_ID, help="assets/maps/<지도 ID>.png의 지도 ID")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--ocr-python", type=Path, default=default_ocr_python(), help="OCR 환경의 Python 실행 파일")
    parser.add_argument("--paper-preparation", type=Path, help="배경과 시설이 붙어 보일 때 사용할 종이 외곽 JSON")
    parser.add_argument("--min-score", type=float, default=0.75)
    parser.add_argument("--replace", action="store_true", help="검증 성공 시 같은 ID의 라벨·지도 결과 갱신")
    args = parser.parse_args()
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]*", args.map_id):
        parser.error("--map-id에는 영문자, 숫자, 밑줄, 하이픈만 사용하세요.")
    config = map_paths(args.map_id, output_root=args.output_dir.resolve())
    try:
        generate_map(
            config, args.ocr_python.resolve(), replace=args.replace, min_score=args.min_score,
            preparation_path=args.paper_preparation,
        )
    except (OSError, ValueError, TypeError, KeyError, cv2.error) as error:
        parser.exit(1, f"지도 생성 실패: {error}\n")
    print(f"라벨: {config.label_path}\n도로 그래프: {config.graph_path}\n의미 지도: {config.semantic_path}")


if __name__ == "__main__":
    main()
