from math import pi
import unittest

import cv2
import numpy as np

from src.rover_control.arrival_geometry import arrival_circle


class ArrivalGeometryTest(unittest.TestCase):
    def test_square_circle_is_centered_and_covers_half_parking_area(self):
        region = {'polygon_px': [[20, 30], [120, 30], [120, 130], [20, 130]]}
        result = arrival_circle(region)
        self.assertEqual(result['center_px'], [70, 80])
        self.assertAlmostEqual(pi * result['radius_px'] ** 2, 5000)
        self.assertFalse(result['limited_to_parking_space'])

    def test_tilted_polygon_uses_geometric_center_and_circle_stays_inside(self):
        polygon = np.array([[60, 30], [165, 45], [180, 145], [35, 125]], np.float32)
        result = arrival_circle({'polygon_px': polygon.tolist()})
        center = np.asarray(result['center_px'])
        for angle in np.linspace(0, 2 * pi, 100):
            point = center + result['radius_px'] * np.array([np.cos(angle), np.sin(angle)])
            self.assertGreaterEqual(cv2.pointPolygonTest(polygon, tuple(map(float, point)), True), -0.001)
        self.assertAlmostEqual(result['actual_area_ratio'], 0.5)

    def test_long_narrow_parking_caps_radius_instead_of_accepting_points_outside_slot(self):
        result = arrival_circle({'bbox': [10, 20, 200, 20]})
        self.assertEqual(result['center_px'], [110, 30])
        self.assertEqual(result['radius_px'], 10)
        self.assertTrue(result['limited_to_parking_space'])
        self.assertLess(result['actual_area_ratio'], 0.5)

    def test_invalid_geometry_and_ratio_are_rejected(self):
        for region in ({'polygon_px': [[0, 0], [0, 0], [0, 0]]},
                       {'polygon_px': [[0, 0], [100, 100], [100, 0], [0, 100]]},
                       {'polygon_px': [[0, 0], [float('nan'), 100], [100, 0]]}):
            with self.assertRaises(ValueError):
                arrival_circle(region)
        with self.assertRaises(ValueError):
            arrival_circle({'bbox': [-1, 0, 20, 20]}, image_size=[100, 100])
