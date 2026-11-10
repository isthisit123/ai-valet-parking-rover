import io
import json
import tempfile
import unittest
from contextlib import redirect_stdout
from math import dist
from pathlib import Path

import cv2
import networkx as nx
import numpy as np

from src.map_processing import (
    graph_exporter,
    map_reader,
    parking_selector,
    semantic_mapper,
)
from src.map_processing.map_config import REAL_MAP_ID, get_map_config


MAP_CASES = {
    "parking_map_v1": {
        # 초록색 지도에서 위치 힌트는 같은 칸을 가리키고 도로 노드 번호는 새 좌표를 따른다.
        "nodes": ("N0", "N1", "N5", "N6", "N2", "N7"),
        "routes": {
            "exit": (480.20, "N4 N10 N9 N8 N1", 893.66),
            "entrance": (686.62, "N4 N10 N11 N6", 686.62),
            "building_a": (309.0, "N4 N10 N9 N8 N1", 893.66),
            "building_b": (108.0, "N4 N10 N11 N5", 831.42),
        },
    },
    "parking_map_v1_test": {
        "nodes": ("N0", "N2", "N5", "N7", "N1", "N6"),
        "routes": {
            "exit": (870.86, "N3 N10 N9 N8 N2", 1203.93),
            "entrance": (807.95, "N3 N10 N11 N7", 807.95),
            "building_a": (112.0, "N3 N10 N9 N8 N2", 1203.93),
            "building_b": (144.0, "N3 N10 N11 N5", 833.95),
        },
    },
}


class ParkingIdsTest(unittest.TestCase):
    def test_accessible_parking_selection_for_all_preferences(self):
        _, graph = parking_selector.load_parking_map()
        expected = {
            "exit": ("space_5", "N4 N10 N9 N8 N2", 1063.32),
            "entrance": ("space_6", "N4 N10 N11 N7", 844.84),
            "building_a": ("space_5", "N4 N10 N9 N8 N2", 1063.32),
            "building_b": ("space_6", "N4 N10 N11 N7", 844.84),
        }
        for preference, (space_id, route, distance) in expected.items():
            with self.subTest(preference=preference):
                result = parking_selector.select_parking(preference, parking_type="accessible")
                selected = graph.nodes[result["selected"]]
                self.assertEqual(selected["role"], "parking_accessible")
                self.assertEqual(selected["region_id"], space_id)
                self.assertEqual(result["route"], route.split())
                self.assertEqual(result["route_distance"], distance)

                normal = parking_selector.select_parking(preference)
                self.assertEqual(graph.nodes[normal["selected"]]["role"], "parking_normal")

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

    def test_both_maps_preserve_geometry_routes_and_ids(self):
        output_root = Path(self.enterContext(tempfile.TemporaryDirectory()))
        for map_id, case in MAP_CASES.items():
            with self.subTest(map_id=map_id):
                config = get_map_config(map_id, output_root=output_root)
                graph, skeleton = graph_exporter.build_road_graph(config.image_path)
                self.assertEqual((graph.number_of_nodes(), graph.number_of_edges()), (12, 11))
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
                self.assertEqual(tuple(map(len, (normal, accessible, buildings, gates))), (4, 2, 2, 2))
                spaces = {
                    region["id"]: region for region in original["regions"]
                    if region["role"] in ("parking_normal", "parking_accessible")
                }
                self.assertEqual(len(spaces), 6)
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
                reversed_detection = (normal[::-1], accessible[::-1], buildings, gates)
                reordered = semantic_mapper.map_semantics(graph_data, labels, reversed_detection)
                self.assertEqual(reordered["regions"][:6], [
                    {**space, "node_id": renamed_ids[space["node_id"]]}
                    for space in spaces.values()
                ])

        # 두 번째 지도의 그래프 미리보기와 결과가 원본과 같은 경로를 쓰면 안 된다.
        for map_id in MAP_CASES:
            config = get_map_config(map_id, output_root=output_root)
            data = json.loads(config.semantic_path.read_text(encoding="utf-8"))
            self.assertEqual(data["map_id"], map_id)
        self.assertTrue((output_root / "graph_preview.png").is_file())
        self.assertTrue((output_root / "parking_map_v1_test" / "graph_preview.png").is_file())

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
