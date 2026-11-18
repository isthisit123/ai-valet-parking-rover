from math import dist
import unittest

import networkx as nx
import numpy as np

from src.map_processing.graph_exporter import build_graph


class GraphExporterTest(unittest.TestCase):
    def test_touching_junction_regions_keep_real_short_road(self):
        skeleton = np.zeros((41, 101), np.uint8)
        skeleton[20, 5:96] = 255
        graph = build_graph(skeleton, [(5, 20), (95, 20)], [(35, 20), (65, 20)])
        self.assertTrue(nx.is_connected(graph))
        self.assertEqual(graph.number_of_edges(), 3)
        edge = graph.get_edge_data("N2", "N3")[0]
        self.assertEqual(edge["path"][0], (35, 20))
        self.assertEqual(edge["path"][-1], (65, 20))
        self.assertTrue(all(skeleton[y, x] for x, y in edge["path"]))
        self.assertTrue(all(dist(a, b) == 1 for a, b in zip(edge["path"], edge["path"][1:])))
        self.assertEqual(edge["weight"], 30)

    def test_endpoint_inside_junction_radius_remains_connected(self):
        skeleton = np.zeros((31, 41), np.uint8)
        skeleton[15, 10:31] = 255
        graph = build_graph(skeleton, [(10, 15), (30, 15)], [(20, 15)])
        self.assertTrue(nx.is_connected(graph))
        self.assertEqual(graph.number_of_edges(), 2)

    def test_nearby_regions_do_not_bridge_disconnected_skeletons(self):
        skeleton = np.zeros((31, 41), np.uint8)
        skeleton[10, 18:23] = 255
        skeleton[12, 18:23] = 255
        graph = build_graph(skeleton, [(20, 10), (20, 12)], [])
        self.assertEqual(graph.number_of_edges(), 0)
        self.assertFalse(nx.is_connected(graph))

    def test_parallel_roads_remain_separate_edges(self):
        skeleton = np.zeros((61, 81), np.uint8)
        skeleton[10, 15:66] = 255
        skeleton[50, 15:66] = 255
        skeleton[10:51, 15] = 255
        skeleton[10:51, 65] = 255
        graph = build_graph(skeleton, [], [(15, 30), (65, 30)])
        self.assertTrue(nx.is_connected(graph))
        self.assertEqual(graph.number_of_edges("N0", "N1"), 2)


if __name__ == "__main__":
    unittest.main()
