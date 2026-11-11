"""ParkingHaeven을 로컬에서 제공하고 원본 캔버스 PNG로 별도 맵을 생성한다."""

import argparse
import copy
import hashlib
import importlib.util
import json
import struct
import threading
import uuid
from datetime import datetime, timezone
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlsplit

import cv2
import networkx as nx
import numpy as np

from src.map_processing.graph_exporter import build_road_graph, save_graph
from src.map_processing.map_config import BASE_DIR, DEFAULT_MAP, MapConfig
from src.map_processing.semantic_mapper import detect_regions, map_semantics


MAX_UPLOAD_BYTES = 12 * 1024 * 1024
MAX_IMAGE_PIXELS = 8_000_000
BRIDGE_PATH = Path(__file__).with_name("http_map_bridge.js")


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
    if corners.shape != (4, 2) or not np.isfinite(corners).all():
        raise ValueError("종이 외곽점은 유효한 네 점이어야 합니다.")
    if (corners < 0).any() or (corners[:, 0] >= width).any() or (corners[:, 1] >= height).any():
        raise ValueError("종이 외곽점이 입력 이미지를 벗어났습니다.")
    mask = np.zeros((height, width), np.uint8)
    cv2.fillConvexPoly(mask, np.round(corners).astype(np.int32), 255)
    result = image.copy()
    result[mask == 0] = 255
    return result


def generate_map(body, output_root, *, labels_path=DEFAULT_MAP.label_path,
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


def handler_for(web_root, output_root, labels_path, mapping_path, aruco_helper=None):
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

        def do_GET(self):
            path = unquote(urlsplit(self.path).path)
            if path in ("/", "/index.html"):
                html = (web_root / "index.html").read_text(encoding="utf-8")
                html = html.replace("</body>", '<script src="/rover-map-bridge.js"></script></body>')
                self.send_bytes(200, html.encode("utf-8"), "text/html; charset=utf-8")
            elif path == "/rover-map-bridge.js":
                self.send_bytes(200, BRIDGE_PATH.read_bytes(), "text/javascript; charset=utf-8")
            elif path.startswith("/captures/"):
                file_path = (output_root / path.removeprefix("/captures/")).resolve()
                if not file_path.is_relative_to(output_root.resolve()) or not file_path.is_file():
                    self.send_error(404)
                    return
                content_type = "image/png" if file_path.suffix == ".png" else "application/json; charset=utf-8"
                self.send_bytes(200, file_path.read_bytes(), content_type)
            else:
                super().do_GET()

        def do_POST(self):
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
                report = generate_map(body, output_root, labels_path=labels_path, mapping_path=mapping_path,
                                      captured_at=self.headers.get("X-Captured-At"))
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
    args = parser.parse_args()
    if not (args.web_root / "index.html").is_file():
        parser.error("--web-root에 ParkingHaeven/Source 경로를 지정하세요.")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    handler = handler_for(args.web_root.resolve(), args.output_dir.resolve(), args.labels, args.mapping, args.aruco_helper)
    server = ThreadingHTTPServer(("127.0.0.1", args.port), handler)
    print(f"ParkingHaeven + PNG map: http://localhost:{server.server_port}", flush=True)
    print(f"Captures: {args.output_dir.resolve()}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
