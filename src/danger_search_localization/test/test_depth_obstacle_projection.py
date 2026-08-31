#!/usr/bin/env python3

import unittest

import numpy as np

from danger_search_localization.config import ScanProjectionConfig
from danger_search_localization.depth_obstacle_projection import (
    DepthObstacleProjectionConfig,
    depth_image_to_optical_points,
    estimate_floor_z,
    select_ground_relative_obstacles,
)
from danger_search_localization.scan_projection import project_planar_scan


class DepthObstacleProjectionTest(unittest.TestCase):
    def setUp(self):
        self.config = DepthObstacleProjectionConfig(
            floor_min_support_points=20,
        )

    def test_depth_image_conversion_uses_metric_intrinsics_and_stride(self):
        depth = np.full((4, 4), 2.0, dtype=np.float32)
        config = DepthObstacleProjectionConfig(image_stride=2)
        points = depth_image_to_optical_points(
            depth, (2.0, 2.0, 1.0, 1.0), config
        )
        self.assertEqual(points.shape, (4, 3))
        np.testing.assert_allclose(points[0], (-1.0, -1.0, 2.0))

    def test_floor_mode_is_estimated_and_not_marked_as_obstacle(self):
        floor = np.column_stack((
            np.linspace(0.5, 2.5, 200),
            np.linspace(-0.8, 0.8, 200),
            np.full(200, -0.30),
        ))
        low_box = np.array([
            [0.70, -0.10, -0.22],
            [0.70, 0.00, -0.18],
            [0.70, 0.10, -0.12],
        ])
        points = np.vstack((floor, low_box))
        floor_z = estimate_floor_z(points, self.config)
        self.assertAlmostEqual(floor_z, -0.30, places=3)
        obstacles = select_ground_relative_obstacles(
            points, floor_z, self.config
        )
        self.assertEqual(len(obstacles), len(low_box))

    def test_object_below_six_centimetres_is_ground_clearance(self):
        points = np.array([
            [0.8, 0.0, -0.30],
            [0.8, 0.1, -0.25],
            [0.8, -0.1, -0.23],
        ])
        obstacles = select_ground_relative_obstacles(
            points, -0.30, self.config
        )
        self.assertEqual(len(obstacles), 1)
        self.assertAlmostEqual(obstacles[0, 2], -0.23)

    def test_previous_floor_rejects_large_false_mode_jump(self):
        false_surface = np.column_stack((
            np.linspace(0.5, 2.0, 100),
            np.zeros(100),
            np.full(100, -0.12),
        ))
        self.assertIsNone(
            estimate_floor_z(false_surface, self.config, previous_floor_z=-0.30)
        )

    def test_low_box_produces_near_navigation_range(self):
        points = np.array([
            [0.75, -0.02, -0.20],
            [0.76, 0.00, -0.18],
            [0.75, 0.02, -0.16],
        ])
        obstacles = select_ground_relative_obstacles(
            points, -0.30, self.config
        )
        scan = ScanProjectionConfig(
            range_min=0.4,
            range_max=3.0,
            min_height=-0.24,
            max_height=0.90,
            self_exclusion_min_x=-0.35,
            self_exclusion_max_x=0.35,
            self_exclusion_half_width_y=0.20,
        )
        ranges = project_planar_scan(obstacles, scan)
        self.assertLess(float(np.min(ranges)), 0.80)


if __name__ == "__main__":
    unittest.main()

