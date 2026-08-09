#!/usr/bin/env python3

import importlib.util
import math
import pathlib
import unittest
from types import SimpleNamespace

import numpy as np


SCRIPT = pathlib.Path(__file__).parents[1] / "scripts" / "exploration_planner.py"
SPEC = importlib.util.spec_from_file_location("exploration_planner", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def make_planner(grid, resolution=1.0, min_frontier_length=1.0):
    planner = MODULE.ExplorationPlanner.__new__(MODULE.ExplorationPlanner)
    planner.map_data = np.array(grid, dtype=np.int8)
    planner.map_info = SimpleNamespace(
        width=planner.map_data.shape[1],
        height=planner.map_data.shape[0],
        resolution=resolution,
        origin=SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )
    planner.min_frontier_length = min_frontier_length
    return planner


class SimpleFrontierTest(unittest.TestCase):
    def test_all_unknown_has_no_frontier(self):
        planner = make_planner(np.full((4, 4), -1))
        self.assertFalse(planner._frontier_mask().any())
        self.assertEqual(planner._frontier_representatives(), [])

    def test_all_known_has_no_frontier(self):
        planner = make_planner(np.zeros((4, 4)))
        self.assertFalse(planner._frontier_mask().any())
        self.assertEqual(planner._frontier_representatives(), [])

    def test_free_cells_next_to_unknown_form_frontier(self):
        planner = make_planner([
            [100, 100, 100, 100, 100],
            [100, 0, 0, -1, -1],
            [100, 0, 0, -1, -1],
            [100, 100, 100, 100, 100],
        ])
        expected = np.zeros((4, 5), dtype=bool)
        expected[1, 2] = True
        expected[2, 2] = True
        np.testing.assert_array_equal(planner._frontier_mask(), expected)
        self.assertEqual(len(planner._frontier_representatives()), 1)

    def test_short_frontier_is_filtered(self):
        planner = make_planner([
            [100, 100, 100, 100],
            [100, 0, -1, 100],
            [100, 100, 100, 100],
        ], resolution=0.5, min_frontier_length=1.0)
        self.assertTrue(planner._frontier_mask().any())
        self.assertEqual(planner._frontier_representatives(), [])

    def test_map_transform_respects_rotated_origin(self):
        planner = make_planner(np.zeros((3, 3)))
        planner.map_info.origin.orientation.z = math.sin(math.pi / 4.0)
        planner.map_info.origin.orientation.w = math.cos(math.pi / 4.0)
        world_x, world_y = planner._map_to_world(1, 0)
        self.assertAlmostEqual(world_x, -0.5)
        self.assertAlmostEqual(world_y, 1.5)
        self.assertEqual(planner._world_to_map(world_x, world_y), (1, 0))

    def test_known_ratio_uses_observed_bounding_box(self):
        planner = make_planner([
            [-1, -1, -1, -1, -1],
            [-1, 0, 0, -1, -1],
            [-1, 0, -1, -1, -1],
            [-1, -1, -1, -1, -1],
        ])
        self.assertAlmostEqual(planner._known_grid_ratio(), 0.75)

    def test_select_goal_distinguishes_service_unavailable(self):
        planner = make_planner([
            [100, 100, 100, 100, 100],
            [100, 0, 0, -1, -1],
            [100, 0, 0, -1, -1],
            [100, 100, 100, 100, 100],
        ])
        planner.current_pose = SimpleNamespace(position=SimpleNamespace(x=1.5, y=1.5))
        planner.max_frontier_candidates = 20
        planner.failed_goals = []
        planner.failed_goal_cooldown = 30.0
        planner.failed_goal_radius = 0.75
        planner._goal_is_cooled_down = lambda x, y: False
        planner._check_path = lambda *args: "unavailable"
        goal, reason = planner._select_goal()
        self.assertIsNone(goal)
        self.assertEqual(reason, "navigation_service_unavailable")
        self.assertEqual(planner.remaining_frontier_count, 1)


if __name__ == "__main__":
    unittest.main()
