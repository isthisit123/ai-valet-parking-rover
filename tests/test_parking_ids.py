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

from src.map_processing import (
    graph_builder,
    graph_exporter,
    map_reader,
    parking_selector,
    semantic_mapper,
)


MAP_CASES = {
    "": {
        "nodes": ("N0", "N2", "N7", "N5", "N1", "N6"),
        "routes": {
            "exit": (481.55, "N4 N10 N9 N8 N2", 864.0),
            "entrance": (644.13, "N4 N10 N11 N5", 644.13),
            "building_a": (312.0, "N4 N10 N9 N8 N2", 864.0),
            "building_b": (110.0, "N4 N10 N11 N7", 791.66),
        },
    },
    "_test": {
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
    def test_both_maps_preserve_geometry_routes_and_ids(self):
        maps_dir = semantic_mapper.IMAGE_PATH.parent
        for suffix, case in MAP_CASES.items():
            map_id = f"parking_map_v1{suffix}"
            with self.subTest(map_id=map_id), tempfile.TemporaryDirectory() as folder:
                image_path = maps_dir / f"{map_id}.png"
                label_path = maps_dir / f"parking_map_v1_labels{suffix}.json"
                output_path = Path(folder) / f"{map_id}_semantic.json"
                with patch.object(map_reader, "IMAGE_PATH", image_path):
                    mask = graph_builder.remove_small_components(map_reader.get_road_mask())
                skeleton = graph_builder.create_skeleton(mask)
                endpoints, junctions = graph_builder.detect_nodes(skeleton)
                graph = graph_exporter.build_graph(skeleton, endpoints, junctions)
                self.assertEqual((graph.number_of_nodes(), graph.number_of_edges()), (12, 11))
                self.assertTrue(nx.is_connected(graph))
                with patch.object(graph_exporter, "BASE_DIR", Path(folder)):
                    graph_path, _ = graph_exporter.save_graph(graph, skeleton)

                paths = {
                    "image_path": image_path,
                    "graph_path": graph_path,
                    "label_path": label_path,
                    "output_path": output_path,
                }
                with redirect_stdout(io.StringIO()):
                    original = semantic_mapper.create_semantic_map(**paths)
                    repeated = semantic_mapper.create_semantic_map(**paths)
                self.assertEqual(original, repeated)
                self.assertEqual(original, json.loads(output_path.read_text(encoding="utf-8")))
                self.assertEqual(original["map_id"], map_id)
                self.assertEqual(original["unit"], "pixel")

                normal, accessible, buildings, gates = semantic_mapper.detect_regions(
                    cv2.imread(str(image_path))
                )
                self.assertEqual(tuple(map(len, (normal, accessible, buildings, gates))), (4, 2, 2, 2))
                spaces = {
                    region["id"]: region for region in original["regions"]
                    if region["role"] in ("parking_normal", "parking_accessible")
                }
                self.assertEqual(len(spaces), 6)
                nodes = {node["id"]: node for node in original["nodes"]}
                labels = json.loads(label_path.read_text(encoding="utf-8"))
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

                with patch.object(parking_selector, "MAP_PATH", output_path):
                    for preference, (score, route, distance) in case["routes"].items():
                        result = parking_selector.select_parking(preference)
                        self.assertEqual(result["selected"], route.split()[-1])
                        self.assertEqual(result["score"], score)
                        self.assertEqual(result["route"], route.split())
                        self.assertEqual(result["route_distance"], distance)

                # 검출 순서와 도로 노드 ID를 바꿔도 같은 위치에 같은 ID가 붙어야 한다.
                graph_data = json.loads(graph_path.read_text(encoding="utf-8"))
                renamed_ids = {node_id: f"R{i}" for i, node_id in enumerate(reversed(list(nodes)))}
                for node in graph_data["nodes"]:
                    node["id"] = renamed_ids[node["id"]]
                for edge in graph_data["edges"]:
                    edge["from"] = renamed_ids[edge["from"]]
                    edge["to"] = renamed_ids[edge["to"]]
                graph_path.write_text(json.dumps(graph_data), encoding="utf-8")
                reversed_detection = (normal[::-1], accessible[::-1], buildings, gates)
                with (
                    patch.object(semantic_mapper, "detect_regions", return_value=reversed_detection),
                    redirect_stdout(io.StringIO()),
                ):
                    reordered = semantic_mapper.create_semantic_map(**paths)
                self.assertEqual(reordered["regions"][:6], [
                    {**space, "node_id": renamed_ids[space["node_id"]]}
                    for space in spaces.values()
                ])

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
