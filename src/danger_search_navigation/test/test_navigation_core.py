#!/usr/bin/env python3
"""Static unit tests for collision policy and action-state primitives."""

import os
import sys
import unittest


SCRIPT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
sys.path.insert(0, SCRIPT_DIR)

from navigation_core import GoalState, InflatedOccupancyGrid


def make_grid(width, height, occupied=(), robot_radius=0.0, inflation_padding=0.0):
    data = [0] * (width * height)
    for cell_x, cell_y in occupied:
        data[cell_y * width + cell_x] = 100
    return InflatedOccupancyGrid(
        width=width,
        height=height,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
        data=data,
        occupied_threshold=65,
        robot_radius=robot_radius,
        inflation_padding=inflation_padding,
        allow_diagonal=True,
        max_expansions=1000,
    )


class InflatedOccupancyGridTest(unittest.TestCase):
    def test_astar_routes_around_static_obstacle(self):
        grid = make_grid(7, 5, occupied=((3, 0), (3, 1), (3, 2), (3, 3)))
        route = grid.plan((0.5, 0.5), (6.5, 0.5))
        self.assertIsNotNone(route)
        self.assertTrue(any(point[1] > 3.5 for point in route))
        self.assertTrue(grid.path_is_traversable(route))

    def test_inflation_blocks_robot_radius_boundary(self):
        grid = make_grid(7, 7, occupied=((3, 3),), robot_radius=1.0)
        self.assertFalse(grid.traversable((3, 3)))
        self.assertFalse(grid.traversable((4, 3)))
        self.assertFalse(grid.traversable((3, 4)))
        self.assertTrue(grid.traversable((5, 3)))

    def test_unreachable_goal_returns_none(self):
        grid = make_grid(7, 5, occupied=tuple((3, cell_y) for cell_y in range(5)))
        self.assertIsNone(grid.plan((0.5, 2.5), (6.5, 2.5)))

    def test_dynamic_obstacle_uses_same_inflation_policy(self):
        grid = make_grid(7, 5, robot_radius=1.0)
        route = grid.plan((0.5, 2.5), (6.5, 2.5), dynamic_cells=((3, 2),))
        self.assertIsNotNone(route)
        self.assertTrue(grid.path_is_traversable(route, dynamic_blocked=((3, 2),)))


class GoalStateTest(unittest.TestCase):
    def test_cancel_marks_terminal_state_and_returns_zero_velocity(self):
        state = GoalState()
        state.begin("goal-1")
        linear_x, angular_z = state.cancel()
        self.assertFalse(state.active)
        self.assertFalse(state.controller_active)
        self.assertEqual(state.active_goal_id, "")
        self.assertEqual(state.failure_code, "CANCELED")
        self.assertEqual((linear_x, angular_z), (0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
