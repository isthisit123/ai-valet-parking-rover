import argparse
from hashlib import sha256
import json
from pathlib import Path
from tempfile import mkdtemp

import cv2
import numpy as np

from src.map_processing.graph_exporter import build_road_graph, save_graph
from src.map_processing.map_config import MAPS_DIR, available_map_ids, get_map_config
from src.map_processing.path_visualizer import DEFAULT_ROUTE_CASES, export_routes
from src.map_processing.planar_calibration import calibrate_floor
from src.map_processing.semantic_mapper import create_semantic_map

from .map_preview import build_map, export_cm_map, floor_transform, read_json
from .route_preview import save_world_route
from .tracking import load_route


def prepare_map(map_id, points, run_dir, case, max_error_cm):
    spec = read_json(points)
    if spec["map_id"] != map_id:
        raise ValueError(f"{points}의 map_id가 {map_id}와 다릅니다.")
    config = get_map_config(map_id, output_root=run_dir)
    image_bytes = config.image_path.read_bytes()
    image = cv2.imdecode(np.frombuffer(image_bytes, np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"지도 이미지를 읽을 수 없습니다. {config.image_path}")
    image_size = image.shape[1::-1]
    graph, skeleton = build_road_graph(config.image_path)
    save_graph(graph, skeleton, config=config)

    calibration = None
    controls = spec["control_points"]
    if len(controls) == 4 and any(value is None for point in controls for value in point["world_cm"]):
        print("실측 기준점이 비어 있어 전지 모서리로 cm 추정 지도를 생성합니다.")
    else:
        report = calibrate_floor(spec, image_size, max_error_cm=max_error_cm)
        report["source_frame"].update(image=config.image_path.name, sha256=sha256(image_bytes).hexdigest())
        calibration = config.output_dir / "map_calibration.json"
        calibration.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        if not report["validation"]["passed"]:
            error = report["validation"]["max_held_out_error_cm"]
            raise ValueError(f"보정 검증 실패 (최대 오차 {error:.2f} cm). {calibration}")

    semantic = create_semantic_map(
        image_path=config.image_path, graph_path=config.graph_path, label_path=config.label_path,
        output_path=config.semantic_path, expected_map_id=map_id, calibration_path=calibration,
    )
    routes = export_routes(config)
    matrix, size = floor_transform(
        map_id, image_bytes, image_size, points=points, calibration=calibration,
    )
    for _, source_route in routes:
        name = source_route.stem
        if case is not None and name != case:
            continue
        data = build_map(semantic, read_json(source_route), matrix, size, verified=calibration is not None)
        json_path, png_path = export_cm_map(image, matrix, data, name, config.output_dir / "cm_preview")
        print(f"{name}: {data['route']['total_distance_cm']:.2f} cm")
        print(f"  PNG: {png_path}")
        print(f"  JSON: {json_path}")
        if calibration is not None:
            route_path = source_route.with_name(f"{name}_cm.json")
            save_world_route(source_route, route_path, load_route(source_route, calibration))
            print(f"  검증된 cm 경로: {route_path}")


def map_inputs(explicit_points):
    if explicit_points is not None:
        return [(read_json(explicit_points)["map_id"], explicit_points)]

    fallback = Path("floor_points.json")
    fallback_map = read_json(fallback)["map_id"] if fallback.is_file() else None
    inputs = []
    for map_id in available_map_ids():
        points = MAPS_DIR / f"{map_id}_points.json"
        if not points.is_file() and map_id == fallback_map:
            points = fallback
        inputs.append((map_id, points))
    return inputs


def main():
    parser = argparse.ArgumentParser(description="등록된 모든 지도와 경로의 cm 지도를 생성합니다.")
    parser.add_argument("--points", type=Path, help="특정 지도 한 장의 기준점 파일")
    parser.add_argument("--output-dir", type=Path, default=Path("data/rover_runs"))
    parser.add_argument("--case", choices=[name for name, _, _ in DEFAULT_ROUTE_CASES], help="특정 경로만 cm로 생성")
    parser.add_argument("--max-error-cm", type=float, default=1.0)
    args = parser.parse_args()
    try:
        inputs = map_inputs(args.points)
        if not inputs:
            raise ValueError("등록된 지도 이미지와 라벨 파일이 없습니다.")
        args.output_dir.mkdir(parents=True, exist_ok=True)
        run_dir = Path(mkdtemp(prefix="run_", dir=args.output_dir)).resolve()
    except (OSError, ValueError, TypeError, KeyError) as error:
        parser.error(str(error))

    completed = 0
    for map_id, points in inputs:
        print(f"지도: {map_id}", flush=True)
        if not points.is_file():
            print(f"  기준점 파일 없음: {points}")
            continue
        try:
            prepare_map(map_id, points, run_dir, args.case, args.max_error_cm)
        except (OSError, ValueError, TypeError, KeyError, StopIteration, cv2.error) as error:
            print(f"  생성 실패: {error}")
            continue
        completed += 1
    print(f"지도 처리 완료: {completed}/{len(inputs)} | 결과 폴더: {run_dir}")
    return 0 if completed == len(inputs) else 1


if __name__ == "__main__":
    raise SystemExit(main())
