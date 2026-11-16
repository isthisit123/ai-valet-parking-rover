import argparse
from hashlib import sha256
import json
from pathlib import Path
from shutil import copyfile
from tempfile import TemporaryDirectory, mkdtemp

import cv2
import networkx as nx
import numpy as np

from scripts.generate_map import default_ocr_python, generate_map
from src.map_processing.floor_markers import calibrate_markers, evaluate_check
from src.map_processing.graph_exporter import build_road_graph, save_graph
from src.map_processing.map_config import MAPS_DIR, MapConfig, available_map_ids, get_map_config, validate_map_id
from src.map_processing.map_identity import configured_map_identity, require_map_identity
from src.map_processing.occupancy import DEFAULT_OCCUPANCY_TTL, load_occupancy
from src.map_processing.parking_selector import get_region_by_id, load_parking_map
from src.map_processing.path_visualizer import DEFAULT_ROUTE_CASES, export_routes
from src.map_processing.planar_calibration import calibrate_floor
from src.map_processing.semantic_mapper import create_semantic_map

from .map_preview import build_map, export_cm_map, floor_transform, read_floor_points, read_json
from .route_preview import save_world_route
from .tracking import Route


def publish_previews(source_dir, destination):
    destination.mkdir(parents=True, exist_ok=True)
    with TemporaryDirectory(dir=destination) as folder:
        staged = []
        for source in sorted(source_dir.iterdir()):
            temporary = Path(folder) / source.name
            copyfile(source, temporary)
            staged.append(temporary)
        # 완성된 파일만 같은 지도·경로 이름으로 갱신한다.
        for temporary in staged:
            temporary.replace(destination / temporary.name)


def prepare_map(map_id, points, run_dir, case, max_error_cm, *, occupancy=None,
                max_age_seconds=DEFAULT_OCCUPANCY_TTL):
    config = get_map_config(map_id, output_root=run_dir)
    if occupancy is not None:
        if not isinstance(occupancy, dict):
            raise ValueError("점유 스냅샷은 JSON 객체여야 합니다.")
        require_map_identity(configured_map_identity(config), occupancy, "점유 스냅샷")
    image_bytes = config.image_path.read_bytes()
    spec = read_floor_points(points, map_id, image_bytes)
    image = cv2.imdecode(np.frombuffer(image_bytes, np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"지도 이미지를 읽을 수 없습니다. {config.image_path}")
    image_size = image.shape[1::-1]
    graph, skeleton = build_road_graph(config.image_path)
    save_graph(graph, skeleton, config=config)

    calibration = None
    controls = spec["control_points"]
    if not controls or (
        len(controls) == 4 and any(value is None for point in controls for value in point["world_cm"])
    ):
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
    matrix, size = floor_transform(
        map_id, image_bytes, image_size, points=points, calibration=calibration,
    )
    return prepare_routes(config, semantic, image, matrix, size, case, calibration is not None,
                          occupancy=occupancy, max_age_seconds=max_age_seconds)


def prepare_routes(config, semantic, image, matrix, size, case, verified, *, occupancy=None,
                   max_age_seconds=DEFAULT_OCCUPANCY_TTL):
    routes = export_routes(config, occupancy=occupancy, max_age_seconds=max_age_seconds, case_id=case)
    preview_dir = config.output_dir / "cm_preview"
    for _, source_route in routes:
        name = source_route.stem
        data = build_map(semantic, read_json(source_route), matrix, size, verified=verified)
        json_path, png_path = export_cm_map(image, matrix, data, name, preview_dir)
        print(f"{name}: {data['route']['total_distance_cm']:.2f} cm")
        print(f"  PNG: {png_path}")
        print(f"  JSON: {json_path}")
        if verified:
            route_path = source_route.with_name(f"{name}_cm.json")
            save_world_route(source_route, route_path, Route(config.map_id, data["route"]["waypoints_cm"]), verified=True)
            print(f"  검증된 cm 경로: {route_path}")
    return preview_dir


def capture_png(camera_index):
    camera = cv2.VideoCapture(camera_index)
    try:
        if not camera.isOpened():
            raise ValueError(f"카메라 {camera_index}를 열 수 없습니다.")
        # 열자마자 들어오는 노출이 불안정한 프레임을 넘긴다.
        for _ in range(12):
            ok, frame = camera.read()
            if not ok:
                raise ValueError(f"카메라 {camera_index}의 프레임을 읽을 수 없습니다.")
    finally:
        camera.release()
    ok, encoded = cv2.imencode(".png", frame)
    if not ok:
        raise ValueError("카메라 프레임의 PNG 저장에 실패했습니다.")
    return encoded.tobytes()


def parking_junction_pixel(config, parking_space_id):
    semantic, graph = load_parking_map(config.semantic_path, config=config)
    entrance, exit_node, parking = (
        get_region_by_id(semantic, facility)["node_id"]
        for facility in ("entrance", "exit", parking_space_id)
    )
    try:
        main_road = nx.shortest_path(graph, entrance, exit_node, weight="weight")
        from_parking = [nx.shortest_path(graph, parking, gate, weight="weight")
                        for gate in (entrance, exit_node)]
    except (nx.NetworkXNoPath, nx.NodeNotFound) as error:
        raise ValueError(f"{parking_space_id}에서 EN–EX 도로로 연결되는 경로가 없습니다.") from error
    main_nodes = set(main_road)
    joins = [next(node for node in path if node in main_nodes) for path in from_parking]
    junction = joins[0]
    if (junction != joins[1] or junction == parking or junction not in main_road[1:-1]
            or len(graph[junction]) < 3 or graph.nodes[junction]["type"] != "junction"):
        raise ValueError(f"{parking_space_id} 도로가 EN–EX 도로에 합류하는 교점을 하나로 찾을 수 없습니다.")
    node = graph.nodes[junction]
    return [node["x"], node["y"]]


def verify_junction(config, spec, report, check):
    if not isinstance(check, dict) or not isinstance(check.get("parking_space_id"), str):
        raise ValueError("도로 확인점에 parking_space_id와 실측 world_cm을 지정하세요.")
    parking_space_id = check["parking_space_id"]
    if not parking_space_id.startswith("space_"):
        raise ValueError("도로 확인점에는 주차칸 ID(space_…)가 필요합니다.")
    world_cm = np.asarray(check["world_cm"], dtype=float)
    outer_cm = np.float32([point["world_cm"] for point in spec["control_points"]])
    if (world_cm.shape != (2,) or not np.isfinite(world_cm).all()
            or cv2.pointPolygonTest(outer_cm, tuple(world_cm), False) <= 0):
        raise ValueError("도로 확인점의 실측 [x, y] cm는 A~D 안쪽이어야 합니다.")
    pixel = parking_junction_pixel(config, parking_space_id)
    outer_px = np.float32([point["pixel"] for point in spec["control_points"]])
    if cv2.pointPolygonTest(outer_px, tuple(pixel), False) <= 0:
        raise ValueError("사진에서 찾은 도로 확인점이 십자 A~D 밖에 있습니다.")
    candidate = report["candidate_image_to_world"]
    validation = evaluate_check(pixel, world_cm, np.asarray(candidate),
                                tolerance_cm=report["validation"]["tolerance_cm"],
                                method="independent_road_junction")
    validation.update(parking_space_id=parking_space_id, pixel=pixel, world_cm=world_cm.tolist())
    report["validation"] = validation
    if validation["passed"]:
        report["image_to_world"] = report.pop("candidate_image_to_world")


def print_check_result(report):
    validation = report["validation"]
    error = validation["max_held_out_error_cm"]
    if not validation["passed"]:
        raise ValueError(f"확인점 E 오차 {error:.2f} cm가 허용값 {validation['tolerance_cm']:g} cm를 초과했습니다.")
    print(f"확인점 E 오차: {error:.2f} cm")


def prepare_camera_map(args, run_dir):
    layout = read_json(args.marker_layout)
    map_id = validate_map_id(layout["map_id"])
    uses_junction = "junction_check" in layout
    if uses_junction and "E" in layout["markers_cm"]:
        raise ValueError("확인점은 빨간 십자 E 또는 도로 교점 중 하나를 지정하세요.")
    folder = run_dir / map_id
    folder.mkdir()
    config = MapConfig(map_id, folder / f"{map_id}.png", folder / f"{map_id}_labels.json", folder)
    image_bytes = args.image.read_bytes() if args.image else capture_png(args.camera if args.camera is not None else 0)
    if not image_bytes.startswith(b"\x89PNG\r\n\x1a\n"):
        raise ValueError("읽을 수 있는 PNG 사진이 필요합니다.")
    image = cv2.imdecode(np.frombuffer(image_bytes, np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("읽을 수 있는 PNG 사진이 필요합니다.")
    config.image_path.write_bytes(image_bytes)
    digest = sha256(image_bytes).hexdigest()
    spec, report = calibrate_markers(image, layout, max_error_cm=args.max_error_cm)
    spec["image_sha256"] = digest
    report["source_frame"].update(image=config.image_path.name, sha256=digest)
    points_path = folder / "floor_points.json"
    calibration_path = folder / "map_calibration.json"
    for path, data in ((points_path, spec), (calibration_path, report)):
        path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"자동 기준점: {points_path}\n보정 결과: {calibration_path}")
    verified = report["validation"]["passed"]
    if "E" in layout["markers_cm"]:
        print_check_result(report)
    elif not uses_junction:
        print("확인점 E가 없어 이번 변환은 cm 추정값입니다.")
    if args.calibration_only and not uses_junction:
        return

    preparation = None
    if verified:
        # 검증되지 않은 외곽을 마스크로 쓰면 경계의 입출구를 잘라낼 수 있다.
        preparation = folder / "paper_preparation.json"
        preparation.write_text(json.dumps({
            "source_sha256": digest,
            "reference_size_px": report["source_frame"]["size_px"],
            "paper_corners_px": spec["paper_corners_px"],
        }), encoding="utf-8")
    ocr_python = args.ocr_python.resolve()
    try:
        generate_map(config, ocr_python, ocr_output_root=folder / "ocr", preparation_path=preparation)
    except ValueError as error:
        print(f"기본 시설 검출 실패 ({error})\n어두운 영상 기준으로 같은 사진에서 다시 시도합니다.", flush=True)
        generate_map(config, ocr_python, ocr_output_root=folder / "ocr", preparation_path=preparation, low_light=True)
    if uses_junction:
        verify_junction(config, spec, report, layout["junction_check"])
        calibration_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print_check_result(report)
        verified = report["validation"]["passed"]
    if args.calibration_only:
        return
    validated_matrix = report["image_to_world" if verified else "candidate_image_to_world"]
    preview_dir = prepare_routes(config, read_json(config.semantic_path), image, np.asarray(validated_matrix),
                                 spec["paper_size_cm"], args.case, verified)
    preview_root = run_dir.parent / "cm_preview" / map_id
    publish_previews(preview_dir, preview_root)
    print(f"공용 cm 지도 폴더: {preview_root}")


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
    parser.add_argument("--occupancy", type=Path, help="현재 지도 ID 기준 점유 JSON")
    parser.add_argument("--occupancy-ttl", type=float, default=DEFAULT_OCCUPANCY_TTL, help="점유 유효 시간(초)")
    parser.add_argument("--max-error-cm", type=float, default=1.0)
    parser.add_argument("--marker-layout", type=Path, help="고정 빨간 십자의 실측 cm 좌표 JSON")
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--camera", type=int, help="직접 촬영할 카메라 번호 (기본 0)")
    source.add_argument("--image", type=Path, help="이미 촬영한 PNG에서 자동 검출")
    parser.add_argument("--ocr-python", type=Path, default=default_ocr_python())
    parser.add_argument("--calibration-only", action="store_true", help="보정 결과까지만 저장 (도로 교점 확인에는 OCR 사용)")
    args = parser.parse_args()
    if args.marker_layout is None and (args.camera is not None or args.image or args.calibration_only):
        parser.error("카메라 자동 보정에는 --marker-layout이 필요합니다.")
    if args.marker_layout and (args.points or args.occupancy):
        parser.error("자동 보정은 새 사진을 사용합니다. --points 또는 기존 --occupancy와 함께 쓰지 마세요.")
    try:
        if args.marker_layout:
            if args.camera is not None and args.camera < 0:
                raise ValueError("카메라 번호는 0 이상이어야 합니다.")
            args.output_dir.mkdir(parents=True, exist_ok=True)
            run_dir = Path(mkdtemp(prefix="run_", dir=args.output_dir)).resolve()
            print(f"결과 폴더: {run_dir}", flush=True)
            prepare_camera_map(args, run_dir)
            return 0
        inputs = map_inputs(args.points)
        occupancy = None
        if args.occupancy is not None:
            occupancy = load_occupancy(args.occupancy)
            inputs = [(map_id, path) for map_id, path in inputs if map_id == occupancy.get("map_id")]
            if not inputs:
                raise ValueError("점유 스냅샷의 map_id에 해당하는 지도가 없습니다.")
        if not inputs:
            raise ValueError("등록된 지도 이미지와 라벨 파일이 없습니다.")
        args.output_dir.mkdir(parents=True, exist_ok=True)
        run_dir = Path(mkdtemp(prefix="run_", dir=args.output_dir)).resolve()
    except (OSError, ValueError, TypeError, KeyError, np.linalg.LinAlgError, cv2.error) as error:
        parser.error(str(error))

    completed = 0
    preview_root = run_dir.parent / "cm_preview"
    for map_id, points in inputs:
        print(f"지도: {map_id}", flush=True)
        if not points.is_file():
            print(f"  기준점 파일 없음: {points}")
            continue
        try:
            preview_dir = prepare_map(
                map_id, points, run_dir, args.case, args.max_error_cm,
                occupancy=occupancy, max_age_seconds=args.occupancy_ttl,
            )
            if occupancy is None:
                publish_previews(preview_dir, preview_root / map_id)
        except (OSError, ValueError, TypeError, KeyError, StopIteration, cv2.error) as error:
            print(f"  생성 실패: {error}")
            continue
        completed += 1
    print(f"지도 처리 완료: {completed}/{len(inputs)} | 결과 폴더: {run_dir}")
    if completed and occupancy is None:
        print(f"공용 cm 지도 폴더: {preview_root}")
    return 0 if completed == len(inputs) else 1


if __name__ == "__main__":
    raise SystemExit(main())
