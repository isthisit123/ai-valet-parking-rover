"""ParkingHaeven을 로컬에서 제공하고 원본 캔버스 PNG로 별도 맵을 생성한다."""

import argparse
import copy
import hashlib
import importlib.util
import json
import math
import struct
import threading
import time
import uuid
from datetime import datetime, timezone
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from shutil import copy2, copytree, rmtree
from tempfile import TemporaryDirectory
from urllib.parse import unquote, urlsplit

import cv2
import networkx as nx
import numpy as np

from src.map_processing.graph_exporter import build_road_graph, save_graph
from src.map_processing.map_config import BASE_DIR, DEFAULT_MAP, MapConfig, validate_map_id
from src.map_processing.semantic_mapper import detect_regions, map_semantics, mask_paper_background
from src.map_processing.path_visualizer import export_routes
from scripts.generate_map import default_ocr_python, generate_map as generate_ocr_map
from scripts.parking_regions import export_parking_regions
from src.rover_control.aruco_tracker import ArucoTracker
from src.rover_control.arrival_test import ArrivalTest
from src.rover_control.live_parking import LiveParking
from src.rover_control.map_preview import capture_floor_points, export_cm_routes, read_json, verify_junction


MAX_UPLOAD_BYTES = 12 * 1024 * 1024
MAX_IMAGE_PIXELS = 8_000_000
BRIDGE_PATH = Path(__file__).with_name("http_map_bridge.js")
ARUCO_BRIDGE_PATH = Path(__file__).with_name("rover_aruco_bridge.js")
ARRIVAL_BRIDGE_PATH = Path(__file__).with_name("rover_arrival_bridge.js")
OCCUPANCY_BRIDGE_PATH = Path(__file__).with_name("parking_occupancy_bridge.js")
DETECTION_WORKER_PATH = Path(__file__).with_name("parking_detection_worker.js")
DETECTOR_PROXY_PATH = Path(__file__).with_name("parking_detector_proxy.js")
PARKING_REGIONS_BRIDGE_PATH = Path(__file__).with_name("parking_regions_bridge.js")
PUBLISH_LOCK = threading.RLock()


def rename_for_publish(source, target):
    # OneDrive·미리보기·백신 등이 디렉터리를 잠깐 잡는 Windows 오류만 재시도한다.
    delays = (0.05, 0.1, 0.2, 0.4, 0.8)
    for attempt in range(len(delays) + 1):
        try:
            return source.rename(target)
        except OSError as error:
            if getattr(error, "winerror", None) not in (5, 32, 33) or attempt == len(delays):
                raise
            time.sleep(delays[attempt])


def write_json(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def decode_png(body):
    if len(body) < 33 or body[:8] != b"\x89PNG\r\n\x1a\n" or body[12:16] != b"IHDR":
        raise ValueError("유효한 PNG 파일이 아닙니다.")
    width, height = struct.unpack(">II", body[16:24])
    if width < 1 or height < 1 or width * height > MAX_IMAGE_PIXELS:
        raise ValueError("PNG 해상도가 허용 범위를 벗어났습니다.")
    image = cv2.imdecode(np.frombuffer(body, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None or image.shape[:2] != (height, width):
        raise ValueError("PNG 디코딩에 실패했습니다.")
    return image


def decode_tracking_image(body, content_type):
    if content_type == "image/png":
        return decode_png(body)
    if content_type != "image/jpeg" or not body.startswith(b"\xff\xd8"):
        raise ValueError("마커 프레임은 image/jpeg 또는 image/png여야 합니다.")
    # 디코딩 전에 SOF의 해상도를 검사해 큰 JPEG도 PNG와 같은 제한을 적용한다.
    offset, size = 2, None
    while offset < len(body):
        if body[offset] != 0xFF:
            raise ValueError("JPEG 헤더가 올바르지 않습니다.")
        while offset < len(body) and body[offset] == 0xFF:
            offset += 1
        if offset >= len(body):
            break
        marker = body[offset]
        offset += 1
        if marker in (0xD9, 0xDA):
            break
        if marker == 0x01 or 0xD0 <= marker <= 0xD8:
            continue
        if offset + 2 > len(body):
            break
        length = int.from_bytes(body[offset:offset + 2], "big")
        if length < 2 or offset + length > len(body):
            raise ValueError("JPEG 세그먼트 길이가 올바르지 않습니다.")
        if marker in (0xC0, 0xC1, 0xC2):
            if length < 8:
                break
            height, width = struct.unpack(">HH", body[offset + 3:offset + 7])
            size = (height, width)
            break
        offset += length
    if size is None or min(size) < 1 or size[0] * size[1] > MAX_IMAGE_PIXELS:
        raise ValueError("JPEG 해상도가 허용 범위를 벗어났습니다.")
    image = cv2.imdecode(np.frombuffer(body, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None or image.shape[:2] != size:
        raise ValueError("JPEG 디코딩에 실패했습니다.")
    return image


def transform_hints(labels, mapping, size):
    """현재 카메라 구도가 매핑 당시와 같다는 조건에서 크기/기준 프레임을 선택한다."""
    if labels["map_id"] != mapping["map_id"]:
        raise ValueError("라벨과 매핑의 map_id가 다릅니다.")
    width, height = size
    reference = mapping["target"]["image_size_px"]
    canvas = mapping["source"]["canvas_size_px"]
    close_aspect = lambda other: abs(width / height - other[0] / other[1]) < 0.005
    if close_aspect(reference):
        matrix = np.diag([width / reference[0], height / reference[1], 1.0])
        basis = "reference_map_aspect"
    elif mapping.get("status", "active") == "active" and canvas is not None and close_aspect(canvas):
        inverse = np.linalg.inv(np.asarray(mapping["transform"]["matrix_3x3"], dtype=float))
        matrix = np.diag([width / canvas[0], height / canvas[1], 1.0]) @ inverse
        basis = "saved_camera_canvas_aspect"
    else:
        raise ValueError("현재 기준 사진과 영상 비율이 다르거나 이전 카메라 매핑이 무효입니다. 새 영역/힌트를 지정하세요.")
    converted = copy.deepcopy(labels)
    for group in ("parking_spaces", "buildings", "gates"):
        for item in converted[group]:
            if "hint_cm" in item or "hint" not in item:
                raise ValueError("이 수신 도구는 픽셀 hint만 지원합니다.")
            point = matrix @ [*item["hint"], 1]
            point = point[:2] / point[2]
            if not np.isfinite(point).all() or (point < 0).any() or point[0] >= width or point[1] >= height:
                raise ValueError(f"변환된 힌트가 영상을 벗어났습니다: {item['id']}")
            item["hint"] = point.tolist()
    return converted, {"basis": basis, "reference_map_to_received_image": matrix.tolist(),
                       "condition": "같은 카메라 위치/각도/줌/크롭. 종횡비 일치만으로 구도 일치를 보장하지 않음."}


def prepare_paper_image(image, preparation):
    """현재 사진에서 확인한 종이 외곽을 같은 구도의 입력에만 적용한다."""
    height, width = image.shape[:2]
    rw, rh = preparation["reference_size_px"]
    if abs(width/height-rw/rh) >= 0.005:
        raise ValueError("종이 외곽 설정과 영상 비율이 다릅니다.")
    corners = np.asarray(preparation["paper_corners_px"], dtype=float) * [width/rw, height/rh]
    return mask_paper_background(image, corners)


def generate_legacy_map(body, output_root, *, labels_path=DEFAULT_MAP.label_path,
                 mapping_path=BASE_DIR / "assets/maps/parking_map_real_parkinghaeven_mapping.json",
                 captured_at=None):
    image = decode_png(body)
    capture_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ_") + uuid.uuid4().hex[:8]
    folder = output_root / capture_id
    folder.mkdir(parents=True, exist_ok=False)
    image_path = folder / "source.png"
    image_path.write_bytes(body)  # 수신 PNG 바이트를 리사이즈/크롭 없이 보존한다.
    report = {"capture_id": capture_id, "status": "failed", "image_size_px": [image.shape[1], image.shape[0]],
              "received_at": datetime.now(timezone.utc).isoformat(), "client_captured_at": captured_at,
              "output_directory": str(folder.resolve()), "source_sha256": hashlib.sha256(body).hexdigest(),
              "files": {"source": f"/captures/{capture_id}/source.png"}}
    try:
        labels = json.loads(labels_path.read_text(encoding="utf-8-sig"))
        mapping = json.loads(mapping_path.read_text(encoding="utf-8-sig"))
        reference_path = BASE_DIR / mapping["target"]["image_path"]
        if hashlib.sha256(reference_path.read_bytes()).hexdigest() != mapping["target"]["image_sha256"]:
            raise ValueError("기준 지도가 매핑 생성 이후 바뀌었습니다. 매핑을 다시 확인하세요.")
        labels, transform = transform_hints(labels, mapping, report["image_size_px"])
        labels_file = folder / "labels.json"
        write_json(labels_file, labels)
        report["hint_transform"] = transform
        report["map_id"] = labels["map_id"]
        config = MapConfig(labels["map_id"], image_path, labels_file, folder)
        preparation_path = labels_path.with_name(labels_path.stem.removesuffix("_labels") + "_preparation.json")
        if preparation_path.is_file():
            preparation = json.loads(preparation_path.read_text(encoding="utf-8"))
            if preparation["prepared_sha256"] != mapping["target"]["image_sha256"]:
                raise ValueError("종이 외곽 설정과 기준 지도 버전이 다릅니다.")
            image = prepare_paper_image(image, preparation)
            ok, encoded = cv2.imencode(".png", image)
            if not ok:
                raise ValueError("종이 배경 처리 이미지 저장 실패")
            prepared_path = folder / "map.png"
            prepared_path.write_bytes(encoded.tobytes())
            config = MapConfig(labels["map_id"], prepared_path, labels_file, folder)
            report["files"]["map"] = f"/captures/{capture_id}/map.png"
            report["image_preparation"] = "outside_paper_mask; same camera pose required"
        graph, skeleton = build_road_graph(config.image_path)
        save_graph(graph, skeleton, config=config)
        report["graph"] = {"nodes": graph.number_of_nodes(), "edges": graph.number_of_edges(),
                           "connected": nx.is_connected(graph)}
        report["files"].update(graph=f"/captures/{capture_id}/graph.json",
                               preview=f"/captures/{capture_id}/graph_preview.png")
        graph_data = json.loads(config.graph_path.read_text(encoding="utf-8"))
        detected = detect_regions(image, map_id=config.map_id)
        report["detected"] = {"normal": len(detected[0]), "accessible": len(detected[1]),
                              "dark_candidates": len(detected[2])}
        semantic = map_semantics(graph_data, labels, detected, expected_map_id=config.map_id)
        write_json(config.semantic_path, semantic)
        report["files"]["semantic"] = f"/captures/{capture_id}/semantic.json"
        report["status"] = "map_generated"
    except (ValueError, OSError, KeyError, TypeError, np.linalg.LinAlgError, cv2.error) as exc:
        report["error"] = str(exc)
    write_json(folder / "report.json", report)
    return report


def export_capture_routes(config, capture_id):
    """같은 캡처의 의미 지도만 사용하며, 5개 조건이 모두 통과한 결과를 게시한다."""
    with TemporaryDirectory(prefix=".route-build-", dir=config.output_dir) as scratch:
        staged = MapConfig(config.map_id, config.image_path, config.label_path, Path(scratch))
        staged.semantic_path.write_bytes(config.semantic_path.read_bytes())
        outputs = export_routes(staged)
        cases = []
        for png_path, json_path in outputs:
            data = json.loads(json_path.read_text(encoding="utf-8"))
            data.update(planning_mode="static_map", occupancy_checked=False,
                        waypoint_target="parking_road_node")
            write_json(json_path, data)
            base_url = f"/captures/{capture_id}/routes/"
            cases.append({"case_id": data["case_id"], "preference": data["preference"],
                          "parking_type": data["parking_type"], "parking_space_id": data["parking_space_id"],
                          "total_distance": data["total_distance"], "waypoint_count": len(data["waypoints"]),
                          "files": {"preview": base_url + png_path.name, "waypoints": base_url + json_path.name}})
        # 한 조건이라도 실패하면 임시 결과를 정리하고 미완성 경로 링크를 노출하지 않는다.
        (Path(scratch) / "routes").rename(config.output_dir / "routes")
    return {"status": "generated", "planning_mode": "static_map", "occupancy_checked": False,
            "coordinate_frame": "map_image", "unit": "pixel", "cases": cases}


def export_capture_world(config, image, semantic, routes, layout, max_error_cm, url):
    points, calibration = capture_floor_points(config, image, layout, max_error_cm=max_error_cm)
    if "junction_check" in layout:
        verify_junction(config, points, calibration, layout["junction_check"])
    validation = calibration["validation"]
    verified = validation["passed"]
    failed = not verified and validation["method"] != "no_independent_check"
    result = {"status": "verified" if verified else "failed" if failed else "estimated",
              "validation": validation,
              "files": {"calibration": url("world/map_calibration.json"),
                        "floor_points": url("world/floor_points.json")}}
    if failed:
        error = validation["max_held_out_error_cm"]
        result["error"] = f"확인점 E 오차 {error:.2f} cm가 허용값 {max_error_cm:g} cm를 초과했습니다."

    # 사진·픽셀 경로와 같은 회차에서 cm 결과가 완성된 뒤 공개한다.
    with TemporaryDirectory(prefix=".world-build-", dir=config.output_dir) as scratch:
        world = Path(scratch) / "world"
        world.mkdir()
        write_json(world / "floor_points.json", points)
        write_json(world / "map_calibration.json", calibration)
        if not failed:
            matrix = np.asarray(calibration["image_to_world" if verified else "candidate_image_to_world"])
            paths = [config.output_dir / "routes" / f"{case['case_id']}.json" for case in routes["cases"]]
            export_cm_routes(semantic, image, matrix, points["paper_size_cm"], paths,
                             world / "cm_preview", world / "routes", verified=verified)
        world.rename(config.output_dir / "world")

    if not failed:
        cases = []
        for route in routes["cases"]:
            case_id = route["case_id"]
            files = {"preview": url(f"world/cm_preview/{case_id}_map_cm.png")}
            if verified:
                files["waypoints"] = url(f"world/routes/{case_id}_cm.json")
            cases.append({"case_id": case_id, "files": files})
        result["cases"] = cases
    return result


def _generate_map_attempt(body, output_root, *, labels_path=DEFAULT_MAP.label_path,
                 mapping_path=BASE_DIR / "assets/maps/parking_map_real_parkinghaeven_mapping.json",
                 captured_at=None, ocr_python=None, min_score=0.75, legacy_hints=False,
                 preparation_path=None, map_id=DEFAULT_MAP.map_id, camera_source=None,
                 marker_layout=None, max_error_cm=1.0):
    if legacy_hints:
        return generate_legacy_map(body, output_root, labels_path=labels_path,
                                   mapping_path=mapping_path, captured_at=captured_at)
    image = decode_png(body)
    capture_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ_") + uuid.uuid4().hex[:8]
    folder = output_root / capture_id
    folder.mkdir(parents=True, exist_ok=False)
    source = folder / "source.png"
    source.write_bytes(body)
    url = lambda name: f"/captures/{capture_id}/{name}"
    report = {"capture_id": capture_id, "status": "failed", "generation_method": "ocr",
              "map_id": map_id, "image_size_px": list(image.shape[1::-1]),
              "received_at": datetime.now(timezone.utc).isoformat(), "client_captured_at": captured_at,
              "output_directory": str(folder.resolve()), "source_sha256": hashlib.sha256(body).hexdigest(),
              "files": {"source": url("source.png")}}
    report["camera_source"] = camera_source
    try:
        # 로컬 PNG·웹캠과 같은 검증·임시 저장 처리를 재사용한다.
        config = MapConfig(map_id, source, folder / "labels.json", folder)
        generate_ocr_map(config, ocr_python or default_ocr_python(), min_score=min_score,
                         ocr_output_root=folder / "ocr", preparation_path=preparation_path)
        graph = json.loads(config.graph_path.read_text(encoding="utf-8"))
        semantic = json.loads(config.semantic_path.read_text(encoding="utf-8"))
        regions = semantic["regions"]
        report.update(status="map_generated", map_image_sha256=semantic["map_image_sha256"],
                      id_assignment_revision=semantic["id_assignment_revision"],
                      graph={"nodes": len(graph["nodes"]), "edges": len(graph["edges"]), "connected": True},
                      detected={"normal": sum(r.get("type") == "normal" for r in regions),
                                "accessible": sum(r.get("type") == "accessible" for r in regions),
                                "dark_candidates": sum(r["id"] in ("building_a", "building_b", "entrance", "exit")
                                                       for r in regions)})
        report["files"].update(labels=url("labels.json"), graph=url("graph.json"),
                               preview=url("graph_preview.png"), semantic=url("semantic.json"))
    except (ValueError, OSError, KeyError, TypeError, nx.NetworkXException, cv2.error) as exc:
        report["error"] = str(exc)
    if report["status"] == "map_generated":
        try:
            report["parking_regions"] = export_parking_regions(semantic, report)
            write_json(folder / "parking_regions.json", report["parking_regions"])
            report["files"]["parking_regions"] = url("parking_regions.json")
        except (ValueError, KeyError, TypeError, OSError, cv2.error) as exc:
            report["parking_regions"] = {"status": "failed", "error": str(exc)}
        try:
            report["route_generation"] = export_capture_routes(config, capture_id)
        except (ValueError, OSError, KeyError, TypeError, nx.NetworkXException, cv2.error) as exc:
            # 의미 지도 생성 성공과 경로 검증 실패를 구분해 알려준다.
            report["route_generation"] = {"status": "failed", "error": str(exc),
                                          "planning_mode": "static_map", "occupancy_checked": False}
    if marker_layout is not None and report.get("route_generation", {}).get("status") == "generated":
        try:
            report["world_calibration"] = export_capture_world(
                config, image, semantic, report["route_generation"], marker_layout, max_error_cm, url,
            )
        except (ValueError, OSError, KeyError, TypeError, nx.NetworkXException, np.linalg.LinAlgError, cv2.error) as exc:
            report["world_calibration"] = {"status": "failed", "error": str(exc)}
    # 미인식일 때도 이름표별 상태와 미리보기를 웹에서 확인할 수 있게 한다.
    for path in sorted((folder / "ocr").glob("*/*/report.json")):
        ocr = json.loads(path.read_text(encoding="utf-8"))
        report["ocr"] = {"status": ocr["status"], "labels": ocr["labels"]}
        report["files"]["ocr_report"] = url(path.relative_to(folder).as_posix())
        preview = path.with_name("preview.png")
        if preview.is_file():
            report["files"]["ocr_preview"] = url(preview.relative_to(folder).as_posix())
    write_json(folder / "report.json", report)
    return report


def generate_map(body, output_root, **options):
    """검증 성공 결과는 latest, 실패 진단은 last_failed의 고정 경로에 갱신한다."""
    decode_png(body)  # 잘못된 요청은 출력 폴더도 만들지 않는다.
    output_root = Path(output_root).resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    with TemporaryDirectory(prefix=".http-build-", dir=output_root) as scratch:
        scratch = Path(scratch)
        report = _generate_map_attempt(body, scratch / "attempt", **options)
        attempt = Path(report["output_directory"])
        complete = report["status"] == "map_generated" and report.get("route_generation", {}).get("status") != "failed"
        key = "latest" if complete else "last_failed"
        target = output_root / key
        pending = scratch / "ready"
        pending.mkdir()
        # 새로 생성한 자료만 복사해 이전 사진의 파일이 남지 않도록 한다.
        for source in attempt.iterdir():
            if source.name == "ocr":
                continue
            if source.is_dir():
                copytree(source, pending / source.name)
            else:
                copy2(source, pending / source.name)
        old_prefix = f"/captures/{report['capture_id']}/"
        prefix = f"/captures/{key}/"

        def fixed_urls(value):
            if isinstance(value, dict):
                return {k: fixed_urls(v) for k, v in value.items()}
            if isinstance(value, list):
                return [fixed_urls(v) for v in value]
            if isinstance(value, str) and value.startswith(old_prefix):
                return prefix + value[len(old_prefix):]
            return value

        report = fixed_urls(report)
        # OCR 내부의 임시 회차 폴더도 게시하지 않고 고정 파일명으로 정리한다.
        for path in (attempt / "ocr").glob("*/*/report.json"):
            (pending / "ocr").mkdir(exist_ok=True)
            copy2(path, pending / "ocr/report.json")
            report["files"]["ocr_report"] = prefix + "ocr/report.json"
            if path.with_name("preview.png").is_file():
                copy2(path.with_name("preview.png"), pending / "ocr/preview.png")
                report["files"]["ocr_preview"] = prefix + "ocr/preview.png"
        report.update(storage_key=key, output_directory=str(target), published_as_latest=complete,
                      latest_output_directory=str(output_root / "latest"))
        with PUBLISH_LOCK:
            if target.is_symlink() or target.resolve().parent != output_root:
                raise ValueError("고정 결과 폴더가 지정한 출력 폴더 밖을 가리킵니다.")
            report["latest_preserved"] = not complete and (output_root / "latest").is_dir()
            write_json(pending / "report.json", report)
            # 복원도 잠길 경우 TemporaryDirectory 정리가 이전 지도를 삭제하지 않도록
            # 백업은 임시 빌드 폴더 밖에 둔다.
            backup = output_root / (".previous-" + uuid.uuid4().hex)
            had_previous = target.exists()
            if had_previous:
                rename_for_publish(target, backup)
            try:
                rename_for_publish(pending, target)
            except OSError as error:
                if had_previous:
                    try:
                        rename_for_publish(backup, target)
                    except OSError as restore_error:
                        raise OSError(f"지도 게시·복원 실패: 이전 결과는 {backup}에 보존했습니다. "
                                      f"폴더 잠금을 해제한 뒤 복원하세요. {restore_error}") from error
                raise OSError(f"지도 결과 폴더 교체 실패: {target}. 기존 결과는 보존했습니다. "
                              f"열린 이미지·폴더 미리보기와 동기화 잠금을 확인하고 다시 시도하세요. {error}") from error
            if had_previous:
                if backup.is_symlink() or backup.resolve().parent != output_root:
                    raise ValueError("이전 결과 백업이 출력 폴더 밖을 가리킵니다.")
                try:
                    rmtree(backup)
                except OSError as cleanup_error:
                    # 새 지도 게시 자체는 성공했다. 잠긴 백업은 삭제하지 않고 위치를 알린다.
                    report["cleanup_warning"] = f"이전 결과 백업 정리 대기: {backup}. {cleanup_error}"
                    print(report["cleanup_warning"], flush=True)
    return report


def handler_for(web_root, output_root, labels_path, mapping_path, aruco_helper=None, *,
                ocr_python=None, min_score=0.75, legacy_hints=False, preparation_path=None,
                marker_layout=None, max_error_cm=1.0,
                map_id=DEFAULT_MAP.map_id, rover_marker_id=7, aruco_dictionary="DICT_4X4_50",
                aruco_period_ms=100, marker_front_offset_deg=0.0, aruco_output_path=None, arrival_output_path=None,
                occupancy_output_path=None):
    tracker = ArucoTracker(marker_id=rover_marker_id, dictionary=aruco_dictionary,
                           front_offset_deg=marker_front_offset_deg,
                           output_path=aruco_output_path or BASE_DIR / "data/rover_runs/aruco/latest.json")
    aruco_busy = threading.Lock()
    parking = LiveParking(output_root, map_lock=PUBLISH_LOCK,
                          output_path=occupancy_output_path or BASE_DIR / "data/rover_runs/occupancy/latest.json")
    arrival = ArrivalTest(output_root, marker_id=rover_marker_id, map_lock=PUBLISH_LOCK, parking=parking,
                           output_path=arrival_output_path)
    base = SimpleHTTPRequestHandler
    if aruco_helper is not None and aruco_helper.is_file():
        # 기존 로컬 서버의 ArUco 기능을 그대로 사용한다.
        spec = importlib.util.spec_from_file_location("parkinghaeven_aruco_helper", aruco_helper)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        base = module.handler_for(web_root)
    busy = threading.Lock()

    class Handler(base):
        def __init__(self, *args, **kwargs):
            if base is SimpleHTTPRequestHandler:
                kwargs["directory"] = str(web_root)
            super().__init__(*args, **kwargs)

        def send_bytes(self, code, body, content_type):
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def send_json(self, code, data):
            self.send_bytes(code, json.dumps(data, ensure_ascii=False).encode("utf-8"), "application/json; charset=utf-8")

        def camera_source(self):
            return {"session_id": self.headers.get("X-Camera-Session", ""),
                    "kind": self.headers.get("X-Source-Type", ""),
                    "width": int(self.headers.get("X-Source-Width", "0")),
                    "height": int(self.headers.get("X-Source-Height", "0"))}

        def do_GET(self):
            path = unquote(urlsplit(self.path).path)
            if path in ("/", "/index.html"):
                html = (web_root / "index.html").read_text(encoding="utf-8")
                html = html.replace("</body>", '<script src="/parking-regions-bridge.js"></script>'
                                    '<script src="/rover-map-bridge.js"></script>'
                                    '<script src="/parking-occupancy-bridge.js"></script>'
                                    '<script src="/rover-arrival-bridge.js"></script></body>')
                self.send_bytes(200, html.encode("utf-8"), "text/html; charset=utf-8")
            elif path == "/rover-map-bridge.js":
                self.send_bytes(200, BRIDGE_PATH.read_bytes(), "text/javascript; charset=utf-8")
            elif path == "/parking-regions-bridge.js":
                self.send_bytes(200, PARKING_REGIONS_BRIDGE_PATH.read_bytes(), "text/javascript; charset=utf-8")
            elif path == "/parking-occupancy-bridge.js":
                self.send_bytes(200, OCCUPANCY_BRIDGE_PATH.read_bytes(), "text/javascript; charset=utf-8")
            elif path == "/parking-detection-worker.js":
                self.send_bytes(200, DETECTION_WORKER_PATH.read_bytes(), "text/javascript; charset=utf-8")
            elif path == "/js/YOLODetector.js":
                self.send_bytes(200, DETECTOR_PROXY_PATH.read_bytes(), "text/javascript; charset=utf-8")
            elif path == "/rover-original-yolo.js":
                self.send_bytes(200, (web_root / "js/YOLODetector.js").read_bytes(), "text/javascript; charset=utf-8")
            elif path == "/js/ArUcoBridge.js":
                self.send_bytes(200, ARUCO_BRIDGE_PATH.read_bytes(), "text/javascript; charset=utf-8")
            elif path == "/rover-arrival-bridge.js":
                self.send_bytes(200, ARRIVAL_BRIDGE_PATH.read_bytes(), "text/javascript; charset=utf-8")
            elif path == "/api/arrival-tests":
                self.send_json(200, arrival.catalog())
            elif path == "/api/arrival-test":
                self.send_json(200, arrival.current())
            elif path == "/api/parking-occupancy":
                try:
                    self.send_json(200, parking.current())
                except (ValueError, OSError, KeyError) as error:
                    self.send_json(400, {"error": str(error)})
            elif path == "/api/aruco-config":
                self.send_json(200, {"marker_id": rover_marker_id, "dictionary": aruco_dictionary,
                                     "period_ms": aruco_period_ms, "front_offset_deg": marker_front_offset_deg})
            elif path == "/api/rover-pose":
                self.send_json(200, {**tracker.current(), "arrival_test": arrival.current()})
            elif path.startswith("/captures/"):
                with PUBLISH_LOCK:
                    file_path = (output_root / path.removeprefix("/captures/")).resolve()
                    if not file_path.is_relative_to(output_root.resolve()) or not file_path.is_file():
                        self.send_error(404)
                        return
                    body = file_path.read_bytes()
                content_type = "image/png" if file_path.suffix == ".png" else "application/json; charset=utf-8"
                self.send_bytes(200, body, content_type)
            else:
                super().do_GET()

        def do_POST(self):
            path = urlsplit(self.path).path
            if path == "/api/parking-occupancy":
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                    if self.headers.get_content_type() != "application/json" or not 0 < length <= 16384:
                        raise ValueError("점유 정보는 16KiB 이내 application/json이어야 합니다.")
                    self.connection.settimeout(2)
                    data = json.loads(self.rfile.read(length))
                    if not isinstance(data, dict):
                        raise ValueError("점유 정보는 JSON 객체여야 합니다.")
                    live = parking.update(data)
                    self.send_json(200, {"occupancy": live, "arrival_test": arrival.current()})
                except (ValueError, OSError, KeyError, TypeError) as error:
                    self.send_json(400, {"error": str(error)})
                return
            if path in ("/api/arrival-test/start", "/api/arrival-test/stop"):
                try:
                    if path.endswith("/stop"):
                        # 다음 keep-alive 요청과 섞이지 않도록 짧은 JSON 본문도 읽는다.
                        length = int(self.headers.get("Content-Length", "0"))
                        if not 0 <= length <= 4096:
                            raise ValueError("중단 요청 크기가 올바르지 않습니다.")
                        self.connection.settimeout(2)
                        self.rfile.read(length)
                        self.send_json(200, arrival.stop())
                        return
                    if busy.locked():
                        self.send_json(409, {"error": "지도 생성이 끝난 뒤 테스트를 시작하세요."})
                        return
                    length = int(self.headers.get("Content-Length", "0"))
                    if self.headers.get_content_type() != "application/json" or not 0 < length <= 4096:
                        raise ValueError("테스트 시작에는 짧은 application/json 요청이 필요합니다.")
                    self.connection.settimeout(2)
                    options = json.loads(self.rfile.read(length))
                    if not isinstance(options, dict):
                        raise ValueError("테스트 설정은 JSON 객체여야 합니다.")
                    self.send_json(200, arrival.start(options.get("case_id"), capture_id=options.get("capture_id"),
                                                      radius_px=options.get("radius_px", 15),
                                                      hold_seconds=options.get("hold_seconds", 1.0),
                                                      parking_space_id=options.get("parking_space_id"),
                                                      auto_retarget=options.get("auto_retarget", True),
                                                      arrival_area_ratio=options.get("arrival_area_ratio", 0.5)))
                except (ValueError, OSError) as error:
                    self.send_json(400, {"error": str(error)})
                return
            if urlsplit(self.path).path == "/api/aruco":
                if not aruco_busy.acquire(blocking=False):
                    self.send_json(409, {"error": "이전 마커 프레임 처리 중입니다."})
                    return
                try:
                    content_type = self.headers.get_content_type()
                    if content_type not in ("image/png", "image/jpeg"):
                        raise ValueError("마커 프레임은 image/jpeg 또는 image/png여야 합니다.")
                    length = int(self.headers.get("Content-Length", "0"))
                    if not 0 < length <= MAX_UPLOAD_BYTES:
                        raise ValueError("마커 프레임 크기는 1~12MiB 범위여야 합니다.")
                    self.connection.settimeout(2)
                    body = self.rfile.read(length)
                    if len(body) != length:
                        raise ValueError("마커 프레임 수신이 완료되지 않았습니다.")
                    received = time.monotonic()
                    result = tracker.observe(decode_tracking_image(body, content_type), received_monotonic_s=received,
                                             client_captured_at=self.headers.get("X-Captured-At"),
                                             frame_id=self.headers.get("X-Frame-Id"), camera_source=self.camera_source())
                    result["arrival_test"] = arrival.observe(result)
                    self.send_json(200, result)
                except (ValueError, OSError, cv2.error) as error:
                    self.send_json(400, {"error": str(error)})
                finally:
                    aruco_busy.release()
                return
            if urlsplit(self.path).path != "/api/map-capture":
                if base is not SimpleHTTPRequestHandler:
                    super().do_POST()
                else:
                    self.send_error(404)
                return
            if not busy.acquire(blocking=False):
                self.send_json(409, {"error": "이전 지도 생성이 진행 중입니다."})
                return
            try:
                if self.headers.get_content_type() != "image/png":
                    raise ValueError("Content-Type은 image/png여야 합니다.")
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= MAX_UPLOAD_BYTES:
                    raise ValueError("PNG 크기는 1~12MiB 범위여야 합니다.")
                self.connection.settimeout(20)
                body = self.rfile.read(length)
                if len(body) != length:
                    raise ValueError("PNG 전송이 완료되지 않았습니다.")
                arrival.stop("map_regeneration")
                report = generate_map(body, output_root, labels_path=labels_path, mapping_path=mapping_path,
                                      captured_at=self.headers.get("X-Captured-At"), ocr_python=ocr_python,
                                      min_score=min_score, legacy_hints=legacy_hints,
                                      preparation_path=preparation_path, map_id=map_id, camera_source=self.camera_source(),
                                      marker_layout=marker_layout, max_error_cm=max_error_cm)
                self.send_json(200 if report["status"] == "map_generated" else 422, report)
            except (ValueError, OSError) as exc:
                self.send_json(400, {"error": str(exc)})
            finally:
                busy.release()

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--web-root", type=Path, default=BASE_DIR.parent / "ParkingHaeven/Source")
    parser.add_argument("--output-dir", type=Path, default=BASE_DIR / "data/http_map_captures")
    parser.add_argument("--labels", type=Path, default=DEFAULT_MAP.label_path)
    parser.add_argument("--mapping", type=Path, default=BASE_DIR / "assets/maps/parking_map_real_parkinghaeven_mapping.json")
    parser.add_argument("--aruco-helper", type=Path, default=BASE_DIR.parent / "parking_training_kit/aruco_server.py")
    parser.add_argument("--port", type=int, default=8001)
    parser.add_argument("--ocr-python", type=Path, default=default_ocr_python(), help="OCR 전용 환경의 Python")
    parser.add_argument("--min-score", type=float, default=0.75)
    parser.add_argument("--map-id", default=DEFAULT_MAP.map_id)
    parser.add_argument("--paper-preparation", type=Path, help="수신 PNG와 해시가 일치하는 종이 외곽 설정")
    parser.add_argument("--marker-layout", type=Path, help="같은 캡처에서 cm 지도를 만들 빨간 십자 실측 JSON")
    parser.add_argument("--max-error-cm", type=float, default=1.0, help="확인점 E의 허용 오차")
    parser.add_argument("--legacy-hints", action="store_true", help="기존 구도용 힌트·외곽 설정 사용 (기본: 새 PNG의 OCR)")
    parser.add_argument("--rover-marker-id", type=int, default=7)
    parser.add_argument("--aruco-dictionary", default="DICT_4X4_50")
    parser.add_argument("--aruco-period-ms", type=int, default=100)
    parser.add_argument("--marker-front-offset-deg", type=float, default=0.0,
                        help="마커 오른쪽 축에서 로버 전방까지의 보정각 (시계 방향 +)")
    args = parser.parse_args()
    if not math.isfinite(args.min_score) or not 0 <= args.min_score <= 1:
        parser.error("--min-score는 0~1 범위여야 합니다.")
    if not 50 <= args.aruco_period_ms <= 1000 or not math.isfinite(args.marker_front_offset_deg):
        parser.error("ArUco 주기는 50~1000ms, 전방 보정각은 유한한 숫자여야 합니다.")
    try:
        map_id = validate_map_id(args.map_id)
        marker_layout = read_json(args.marker_layout) if args.marker_layout else None
        if marker_layout is not None:
            if args.legacy_hints:
                raise ValueError("--marker-layout은 OCR 지도 생성 모드에서 사용하세요.")
            if marker_layout["map_id"] != map_id:
                raise ValueError("--marker-layout의 map_id는 --map-id와 일치해야 합니다.")
            if not math.isfinite(args.max_error_cm) or args.max_error_cm <= 0:
                raise ValueError("--max-error-cm은 양수여야 합니다.")
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.error(str(error))
    if not (args.web_root / "index.html").is_file():
        parser.error("--web-root에 ParkingHaeven/Source 경로를 지정하세요.")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    handler = handler_for(args.web_root.resolve(), args.output_dir.resolve(), args.labels, args.mapping, args.aruco_helper,
                          ocr_python=args.ocr_python.resolve(), min_score=args.min_score, legacy_hints=args.legacy_hints,
                          preparation_path=args.paper_preparation, map_id=map_id,
                          marker_layout=marker_layout, max_error_cm=args.max_error_cm,
                          rover_marker_id=args.rover_marker_id, aruco_dictionary=args.aruco_dictionary,
                          aruco_period_ms=args.aruco_period_ms, marker_front_offset_deg=args.marker_front_offset_deg,
                          arrival_output_path=BASE_DIR / "data/rover_runs/arrival/latest.json")
    server = ThreadingHTTPServer(("127.0.0.1", args.port), handler)
    print(f"ParkingHaeven + PNG map: http://localhost:{server.server_port}", flush=True)
    print(f"Captures: {args.output_dir.resolve()}", flush=True)
    print("Generation: " + ("legacy hints (fixed camera pose)" if args.legacy_hints else f"OCR ({args.ocr_python})"), flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
