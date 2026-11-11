import io
import json
import tempfile
import unittest
from contextlib import redirect_stdout
from math import dist
from pathlib import Path
from unittest.mock import patch

import cv2
import networkx as nx
import numpy as np

from src.map_processing import (
    graph_exporter,
    map_config,
    map_reader,
    parking_selector,
    semantic_mapper,
)
from src.map_processing.map_config import REAL_MAP_ID, get_map_config


MAP_CASES = {
    REAL_MAP_ID: {
        "nodes": ("N0", "N1", "N2", "N7", "N6", "N5"),
        "routes": {
            "exit": (170.55, "N3 N8 N10 N9 N11 N12 N13 N7", 694.02),
            "entrance": (187.22, "N3 N8 N0", 187.22),
            "building_a": (140.22, "N3 N8 N0", 187.22),
            "building_b": (130.00, "N3 N8 N10 N9 N11 N12 N13 N7", 694.02),
        },
    },
}


class ParkingIdsTest(unittest.TestCase):
    def test_centimeter_hints_match_the_same_facilities_as_pixel_hints(self):
        output_root = Path(self.enterContext(tempfile.TemporaryDirectory()))
        config = get_map_config(output_root=output_root)
        graph, skeleton = graph_exporter.build_road_graph(config.image_path)
        graph_exporter.save_graph(graph, skeleton, config=config)
        image = map_reader.load_map_image(config.image_path)
        height, width = image.shape[:2]
        matrix = np.array([[0.12, 0.02, -20], [0.01, 0.13, -18], [0.0003, -0.0001, 1]])
        report = {
            "map_id": config.map_id,
            "source_frame": {"coordinate_frame": "camera_image", "size_px": [width, height]},
            "target_frame": {
                "coordinate_frame": "world", "unit": "cm", "origin": "paper_top_left",
                "axis_directions": {"x": "right", "y": "down"},
            },
            "paper_size_cm": [200, 200],
            "image_to_world": matrix.tolist(),
            "validation": {"passed": True},
        }
        calibration_path = output_root / "homography.json"
        calibration_path.write_text(json.dumps(report), encoding="utf-8")

        labels = json.loads(config.label_path.read_text(encoding="utf-8-sig"))
        for group in ("buildings", "gates", "parking_spaces"):
            for spec in labels[group]:
                if spec["id"] == "exit":
                    continue  # 픽셀과 cm 힌트를 한 파일에 섞어도 된다.
                x, y, scale = matrix @ [*spec.pop("hint"), 1]
                spec["hint_cm"] = [x / scale, y / scale]
        cm_label_path = output_root / "labels_cm.json"
        cm_label_path.write_text(json.dumps(labels), encoding="utf-8-sig")
        paths = {
            "image_path": config.image_path,
            "graph_path": config.graph_path,
            "output_path": config.semantic_path,
            "expected_map_id": config.map_id,
        }
        with redirect_stdout(io.StringIO()):
            expected = semantic_mapper.create_semantic_map(**paths, label_path=config.label_path)
            actual = semantic_mapper.create_semantic_map(
                **paths, label_path=cm_label_path, calibration_path=calibration_path,
            )
        self.assertEqual(actual, expected)
        self.assertEqual(actual["unit"], "pixel")
        self.assertEqual(json.loads(cm_label_path.read_text(encoding="utf-8-sig")), labels)

        with self.assertRaisesRegex(ValueError, "--calibration"):
            semantic_mapper.resolve_cm_hints(labels, (width, height), None)
        for overrides, error in (
            ({"map_id": "another_map"}, "map_id"),
            ({"source_frame": {"size_px": [width + 1, height]}}, "해상도"),
            ({"validation": {"passed": False}}, "검증을 통과"),
        ):
            with self.subTest(overrides=overrides):
                calibration_path.write_text(json.dumps({**report, **overrides}), encoding="utf-8")
                with self.assertRaisesRegex(ValueError, error):
                    semantic_mapper.resolve_cm_hints(labels, (width, height), calibration_path)

    def test_new_map_pair_is_discovered_without_code_registration(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            maps_dir = root / "inputs"
            maps_dir.mkdir()
            image_path = maps_dir / "new_map.png"
            labels_path = maps_dir / "new_map_labels.json"
            image_path.write_bytes(b"image")
            with patch.object(map_config, "MAPS_DIR", maps_dir):
                with self.assertRaisesRegex(ValueError, "_labels.json"):
                    map_config.get_map_config("new_map")
                labels_path.write_text('{"map_id": "new_map"}', encoding="utf-8")
                config = map_config.get_map_config("new_map", output_root=root / "results")
                self.assertEqual(config.image_path, image_path)
                self.assertEqual(config.label_path, labels_path)
                self.assertEqual(config.graph_path, root / "results" / "new_map" / "graph.json")
                self.assertEqual(config.semantic_path.name, "semantic.json")
                with self.assertRaises(ValueError):
                    map_config.get_map_config("../new_map")

    def test_accessible_parking_selection_for_all_preferences(self):
        output_root = self.enterContext(tempfile.TemporaryDirectory())
        config = get_map_config(output_root=output_root)
        graph, skeleton = graph_exporter.build_road_graph(config.image_path)
        graph_exporter.save_graph(graph, skeleton, config=config)
        with redirect_stdout(io.StringIO()):
            semantic_mapper.create_semantic_map(
                image_path=config.image_path, graph_path=config.graph_path,
                label_path=config.label_path, output_path=config.semantic_path,
                expected_map_id=config.map_id,
            )
        map_data, graph = parking_selector.load_parking_map(config.semantic_path)
        expected = {
            "exit": ("space_3", "N3 N8 N10 N9 N11 N12 N2", 565.93),
            "entrance": ("space_6", "N3 N8 N10 N5", 292.96),
            "building_a": ("space_6", "N3 N8 N10 N5", 292.96),
            "building_b": ("space_3", "N3 N8 N10 N9 N11 N12 N2", 565.93),
        }
        for preference, (space_id, route, distance) in expected.items():
            with self.subTest(preference=preference):
                result = parking_selector.select_parking(
                    preference, map_path=config.semantic_path, parking_type="accessible"
                )
                selected = graph.nodes[result["selected"]]
                self.assertEqual(selected["role"], "parking_accessible")
                self.assertEqual(selected["region_id"], space_id)
                self.assertEqual(result["route"], route.split())
                self.assertEqual(result["route_distance"], distance)

                normal = parking_selector.select_parking(preference, map_path=config.semantic_path)
                self.assertEqual(graph.nodes[normal["selected"]]["role"], "parking_normal")

        # 거리 조건이 없는 선택은 도로 노드 번호보다 고유 주차칸 ID를 따른다.
        node_ids = {"N2": "Z5", "N5": "A6"}
        renamed = nx.relabel_nodes(graph, node_ids)
        renamed_data = {
            **map_data,
            "nodes": [
                {**node, "id": node_ids.get(node["id"], node["id"])}
                for node in map_data["nodes"]
            ],
        }
        result = parking_selector.select_parking_from_map(
            renamed_data, renamed, "any", parking_type="accessible"
        )
        self.assertEqual(renamed.nodes[result["selected"]]["region_id"], "space_3")

    def test_real_map_matches_dim_blue_and_edge_touching_dark_facilities(self):
        image = np.full((300, 400, 3), 255, dtype=np.uint8)
        for x, y in ((30, 30), (310, 30), (30, 190), (339, 190)):
            cv2.rectangle(image, (x, y), (x + 60, y + 50), (0, 0, 0), -1)
        cv2.line(image, (299, 215), (339, 215), (0, 0, 0), 1)
        cv2.rectangle(image, (170, 190), (230, 240), (50, 200, 60), -1)
        cv2.rectangle(image, (170, 85), (230, 135), (130, 68, 33), -1)
        labels = {
            "map_id": REAL_MAP_ID,
            "buildings": [
                {"id": "building_a", "hint": [60, 55]},
                {"id": "building_b", "hint": [340, 55]},
            ],
            "gates": [
                {"id": "entrance", "hint": [60, 215]},
                {"id": "exit", "hint": [369, 215]},
            ],
            "parking_spaces": [
                {"id": "space_1", "type": "normal", "hint": [200, 215]},
                {"id": "space_2", "type": "accessible", "hint": [200, 110]},
            ],
        }
        graph_data = {
            "nodes": [
                {"id": "start", "type": "endpoint", "x": 100, "y": 215},
                {"id": "end", "type": "endpoint", "x": 329, "y": 215},
                {"id": "parking", "type": "endpoint", "x": 200, "y": 180},
                {"id": "accessible", "type": "endpoint", "x": 200, "y": 145},
            ]
        }
        detected = semantic_mapper.detect_regions(image, map_id=REAL_MAP_ID)
        self.assertEqual(tuple(map(len, detected)), (1, 1, 4, 0))

        result = semantic_mapper.map_semantics(graph_data, labels, detected)
        regions = {region["id"]: region for region in result["regions"]}
        self.assertEqual(regions["entrance"]["node_id"], "start")
        self.assertEqual(regions["exit"]["node_id"], "end")
        self.assertEqual(regions["space_1"]["node_id"], "parking")
        self.assertEqual(regions["space_2"]["node_id"], "accessible")
        self.assertEqual(regions["exit"]["bbox"][0], 339)
        for building in ("building_a", "building_b"):
            self.assertNotIn("node_id", regions[building])
        for spec in labels["buildings"] + labels["gates"]:
            self.assertEqual(regions[spec["id"]]["center"], spec["hint"])

        for invalid_hint, message in (([1, 1], "검은색 영역이 0개"), ([60, 55], "이미 매핑된")):
            with self.subTest(hint=invalid_hint):
                wrong_labels = {
                    **labels,
                    "gates": [{**labels["gates"][0], "hint": invalid_hint}, labels["gates"][1]],
                }
                with self.assertRaisesRegex(ValueError, message):
                    semantic_mapper.map_semantics(graph_data, wrong_labels, detected)

    def test_real_map_preserves_geometry_routes_and_ids(self):
        output_root = Path(self.enterContext(tempfile.TemporaryDirectory()))
        for map_id, case in MAP_CASES.items():
            with self.subTest(map_id=map_id):
                config = get_map_config(map_id, output_root=output_root)
                graph, skeleton = graph_exporter.build_road_graph(config.image_path)
                self.assertEqual((graph.number_of_nodes(), graph.number_of_edges()), (14, 13))
                self.assertTrue(nx.is_connected(graph))
                graph_path, preview_path = graph_exporter.save_graph(graph, skeleton, config=config)
                self.assertTrue(preview_path.is_file())

                paths = {
                    "image_path": config.image_path,
                    "graph_path": graph_path,
                    "label_path": config.label_path,
                    "output_path": config.semantic_path,
                    "expected_map_id": config.map_id,
                }
                with redirect_stdout(io.StringIO()):
                    original = semantic_mapper.create_semantic_map(**paths)
                    repeated = semantic_mapper.create_semantic_map(**paths)
                self.assertEqual(original, repeated)
                self.assertEqual(original, json.loads(config.semantic_path.read_text(encoding="utf-8")))
                self.assertEqual(original["map_id"], map_id)
                self.assertEqual(original["unit"], "pixel")

                normal, accessible, buildings, gates = semantic_mapper.detect_regions(
                    map_reader.load_map_image(config.image_path), map_id=map_id
                )
                # 실물 입출구는 검은색 시설로 매칭한다. 밝은 배경 후보 수는 의미가 없다.
                self.assertEqual(tuple(map(len, (normal, accessible))), (4, 2))
                spaces = {
                    region["id"]: region for region in original["regions"]
                    if region["role"] in ("parking_normal", "parking_accessible")
                }
                self.assertEqual(len(spaces), 6)
                self.assertEqual(
                    {region["id"] for region in original["regions"] if region["id"] not in spaces},
                    {"building_a", "building_b", "entrance", "exit"},
                )
                nodes = {node["id"]: node for node in original["nodes"]}
                labels = json.loads(config.label_path.read_text(encoding="utf-8"))
                regions_by_type = {"normal": normal, "accessible": accessible}
                for spec, node_id in zip(labels["parking_spaces"], case["nodes"], strict=True):
                    space_id, space_type = spec["id"], spec["type"]
                    detected = min(
                        regions_by_type[space_type],
                        key=lambda region: dist(region["center"], spec["hint"]),
                    )
                    self.assertEqual(spaces[space_id], {
                        **detected, "id": space_id, "type": space_type,
                        "role": f"parking_{space_type}", "node_id": node_id,
                    })
                    self.assertEqual(nodes[node_id]["region_id"], space_id)
                    self.assertEqual(nodes[node_id]["role"], spaces[space_id]["role"])

                for preference, (score, route, distance) in case["routes"].items():
                    result = parking_selector.select_parking(preference, map_path=config.semantic_path)
                    self.assertEqual(result["selected"], route.split()[-1])
                    self.assertEqual(result["score"], score)
                    self.assertEqual(result["route"], route.split())
                    self.assertEqual(result["route_distance"], distance)

                graph_data = json.loads(graph_path.read_text(encoding="utf-8"))
                for edge in graph_data["edges"]:
                    path = edge["path"]
                    start, end = nodes[edge["from"]], nodes[edge["to"]]
                    self.assertEqual(path[0], [start["x"], start["y"]])
                    self.assertEqual(path[-1], [end["x"], end["y"]])
                    length = sum(dist(a, b) for a, b in zip(path, path[1:]))
                    self.assertEqual(edge["weight"], round(length, 2))

                # 검출 순서와 도로 노드 ID를 바꿔도 같은 위치에 같은 ID가 붙어야 한다.
                renamed_ids = {node_id: f"R{i}" for i, node_id in enumerate(reversed(list(nodes)))}
                for node in graph_data["nodes"]:
                    node["id"] = renamed_ids[node["id"]]
                for edge in graph_data["edges"]:
                    edge["from"] = renamed_ids[edge["from"]]
                    edge["to"] = renamed_ids[edge["to"]]
                reversed_detection = (normal[::-1], accessible[::-1], buildings[::-1], gates)
                reordered = semantic_mapper.map_semantics(graph_data, labels, reversed_detection)
                self.assertEqual(reordered["regions"][:6], [
                    {**space, "node_id": renamed_ids[space["node_id"]]}
                    for space in spaces.values()
                ])

        for map_id in MAP_CASES:
            config = get_map_config(map_id, output_root=output_root)
            data = json.loads(config.semantic_path.read_text(encoding="utf-8"))
            self.assertEqual(data["map_id"], map_id)
        self.assertTrue((output_root / REAL_MAP_ID / "graph_preview.png").is_file())

    def test_invalid_hints_raise_errors(self):
        first = {"id": "space_1", "type": "normal", "hint": [15, 15]}
        second = {"id": "space_2", "type": "normal", "hint": [35, 15]}
        normal = [{"bbox": [10, 10, 10, 10]}, {"bbox": [30, 10, 10, 10]}]
        cases = (
            ([], [first, second], "검출하지 못했습니다"),
            (normal, None, "parking_spaces"),
            (normal, [first], "검출 수와 위치 힌트 수가 다릅니다"),
            (normal, [first, {**second, "id": first["id"]}], "중복 주차칸 ID"),
            (normal, [first, {**second, "hint": first["hint"]}], "이미 매핑된 영역"),
            (normal, [first, {**second, "hint": [40, 15]}], "검출 영역이 0개"),
            (normal, [first, {**second, "type": "accessible"}], "검출 영역이 0개"),
            ([normal[0], normal[0]], [first, second], "검출 영역이 2개"),
        )
        for regions, labels, error in cases:
            with self.subTest(error=error):
                with self.assertRaisesRegex(ValueError, error):
                    semantic_mapper.assign_parking_ids(regions, [], labels)
