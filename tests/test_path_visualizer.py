import io
import json
import tempfile
import unittest
from contextlib import redirect_stdout
from math import dist, sqrt

import cv2
import networkx as nx
import numpy as np

from src.map_processing import graph_exporter, map_reader, parking_selector, semantic_mapper
from src.map_processing.map_config import DEFAULT_MAP_ID, get_map_config
from src.map_processing.path_visualizer import ROUTE_COLOR, connect_waypoints, export_routes


class PathVisualizerTest(unittest.TestCase):
    def test_five_real_map_route_cases(self):
        expected_cases = (
            ("01_building_a", "building_a", "normal", "space_1", 187.22),
            ("02_accessible", "any", "accessible", "space_3", 565.93),
            ("03_exit", "exit", "normal", "space_4", 694.02),
            ("04_building_b_accessible", "building_b", "accessible", "space_3", 565.93),
            ("05_entrance_accessible", "entrance", "accessible", "space_6", 292.96),
        )
        with tempfile.TemporaryDirectory() as folder:
            for map_id in (DEFAULT_MAP_ID,):
                with self.subTest(map_id=map_id):
                    config = get_map_config(map_id, output_root=folder)
                    graph, skeleton = graph_exporter.build_road_graph(config.image_path)
                    graph_exporter.save_graph(graph, skeleton, config=config)
                    with redirect_stdout(io.StringIO()):
                        semantic_mapper.create_semantic_map(
                            image_path=config.image_path,
                            graph_path=config.graph_path,
                            label_path=config.label_path,
                            output_path=config.semantic_path,
                            expected_map_id=map_id,
                        )

                    image = map_reader.load_map_image(config.image_path)
                    road_mask = map_reader.detect_road_mask(image)
                    map_data, parking_graph = parking_selector.load_parking_map(config.semantic_path)
                    outputs = export_routes(config)
                    self.assertEqual(len(outputs), 5)
                    for expected, (png_path, json_path) in zip(
                        expected_cases, outputs, strict=True
                    ):
                        case_id, preference, parking_type, space_id, distance = expected
                        result = parking_selector.select_parking_from_map(
                            map_data, parking_graph, preference, parking_type=parking_type
                        )
                        data = json.loads(json_path.read_text(encoding="utf-8"))
                        self.assertEqual(data["map_id"], map_id)
                        self.assertEqual(data["preference"], preference)
                        self.assertEqual(data["case_id"], case_id)
                        self.assertEqual(data["parking_type"], parking_type)
                        self.assertEqual(data["parking_space_id"], space_id)
                        self.assertEqual(data["total_distance"], distance)
                        self.assertEqual(data["node_route"], result["route"])
                        self.assertEqual(data["total_distance"], result["route_distance"])
                        self.assertEqual(data["unit"], "pixel")
                        self.assertEqual(data["coordinate_frame"], "map_image")
                        self.assertEqual(
                            data["parking_space_id"],
                            parking_graph.nodes[result["selected"]]["region_id"],
                        )

                        points = data["waypoints"]
                        start, end = [graph.nodes[node] for node in (result["route"][0], result["selected"])]
                        self.assertEqual(points[0], [start["x"], start["y"]])
                        self.assertEqual(points[-1], [end["x"], end["y"]])
                        self.assertTrue(all(road_mask[y, x] for x, y in points))
                        self.assertTrue(all(0 < dist(a, b) <= sqrt(2) for a, b in zip(points, points[1:])))
                        self.assertEqual(png_path.parent, config.output_dir / "routes")
                        self.assertEqual(png_path.name, f"{case_id}.png")
                        self.assertEqual(json_path.name, f"{case_id}.json")
                        preview = cv2.imread(str(png_path))
                        self.assertEqual(preview.shape, image.shape)
                        # 출발·도착·교차점 표식을 제외한 경로 픽셀과 JSON 좌표를 비교한다.
                        markers = [
                            (graph.nodes[node]["x"], graph.nodes[node]["y"])
                            for node in result["route"]
                        ]
                        line_points = [
                            point for point in points
                            if all(dist(point, marker) > 12 for marker in markers)
                        ]
                        self.assertTrue(line_points)
                        self.assertTrue(all(tuple(preview[y, x]) == ROUTE_COLOR for x, y in line_points))

    def test_cheapest_parallel_edge_reversed_and_joined(self):
        graph = nx.MultiGraph()
        graph.add_node("A", x=1, y=1)
        graph.add_node("B", x=5, y=1)
        graph.add_node("C", x=5, y=3)
        graph.add_edge("A", "B", weight=8, path=[(1, 1), (1, 3), (5, 3), (5, 1)])
        graph.add_edge("A", "B", weight=4, path=[(5, 1), (5, 1), (1, 1)])
        graph.add_edge("A", "B", weight=4, path=[(1, 1), (3, 2), (5, 1)])
        graph.add_edge("B", "C", weight=2, path=[(5, 1), (5, 3)])
        road_mask = np.full((6, 6), 255, dtype=np.uint8)
        self.assertEqual(
            connect_waypoints(graph, ["A", "B", "C"], road_mask),
            [(1, 1), (2, 1), (3, 1), (4, 1), (5, 1), (5, 2), (5, 3)],
        )

        road_mask[1, 3] = 0
        with self.assertRaisesRegex(ValueError, "도로 영역을 벗어났습니다"):
            connect_waypoints(graph, ["A", "B", "C"], road_mask)
        with self.assertRaisesRegex(ValueError, "도로 간선을 찾을 수 없습니다"):
            connect_waypoints(graph, ["A", "C"], road_mask)
