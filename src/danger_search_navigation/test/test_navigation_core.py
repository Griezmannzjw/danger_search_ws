#!/usr/bin/env python3
"""共享规划核心和导航状态语义的 ROS 无关单元测试。"""

import math
import os
import sys
import unittest


SCRIPTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

from navigation_core import (
    DynamicObstacleWait,
    GoalState,
    InflatedOccupancyGrid,
    goal_reached,
    is_non_decreasing_stamp,
    map_snapshot_fingerprint,
    plan_route_variants,
    path_lengths,
    path_progress,
)


def make_grid(width, height, occupied=(), unknown=(), **kwargs):
    """构造 1 米分辨率、默认全自由的测试地图。"""
    data = [0] * (width * height)
    for cell_x, cell_y in occupied:
        data[cell_y * width + cell_x] = 100
    for cell_x, cell_y in unknown:
        data[cell_y * width + cell_x] = -1
    return InflatedOccupancyGrid(
        width, height, 1.0,
        kwargs.pop("origin_x", 0.0),
        kwargs.pop("origin_y", 0.0),
        kwargs.pop("origin_yaw", 0.0),
        data,
        **kwargs
    )


class InflatedOccupancyGridTest(unittest.TestCase):
    def test_a_star_routes_around_static_obstacle(self):
        # 墙没有到达顶边，路径必须绕墙而不是直线穿越。
        wall = [(3, cell_y) for cell_y in range(5)]
        grid = make_grid(7, 7, occupied=wall)
        route = grid.plan((1.5, 3.5), (5.5, 3.5))
        self.assertIsNotNone(route)
        self.assertGreater(len(route), 2)
        self.assertTrue(any(point[1] > 5.0 for point in route))
        self.assertTrue(grid.path_is_traversable(route))

    def test_unreachable_goal_returns_none(self):
        grid = make_grid(7, 5, occupied=[(3, cell_y) for cell_y in range(5)])
        self.assertIsNone(grid.plan((0.5, 2.5), (6.5, 2.5)))

    def test_unknown_occupied_and_outside_are_not_traversable(self):
        grid = make_grid(3, 3, occupied=[(2, 2)], unknown=[(0, 0)])
        self.assertFalse(grid.traversable(grid.world_to_cell(0.5, 0.5)))
        self.assertFalse(grid.traversable(grid.world_to_cell(2.5, 2.5)))
        self.assertFalse(grid.traversable(grid.world_to_cell(-0.1, 0.5)))
        self.assertIsNone(grid.plan((0.5, 0.5), (1.5, 1.5)))
        self.assertIsNone(grid.plan((1.5, 1.5), (2.5, 2.5)))

    def test_inflation_blocks_narrow_corridor(self):
        occupied = [(cell_x, 0) for cell_x in range(7)]
        occupied += [(cell_x, 2) for cell_x in range(7)]
        grid = make_grid(7, 3, occupied=occupied, robot_radius=1.0)
        self.assertIsNone(grid.plan((1.5, 1.5), (5.5, 1.5)))

    def test_dynamic_obstacle_uses_same_inflation_policy(self):
        grid = make_grid(7, 5, robot_radius=1.0)
        expanded = grid.expanded_cells([(3, 2)])
        self.assertIn((3, 2), expanded)
        self.assertIn((4, 2), expanded)
        route = grid.plan((0.5, 2.5), (6.5, 2.5), dynamic_cells=[(3, 2)])
        self.assertIsNotNone(route)
        self.assertTrue(grid.path_is_traversable(route, dynamic_cells=[(3, 2)]))

    def test_dynamic_obstacle_can_temporarily_block_an_otherwise_valid_route(self):
        grid = make_grid(5, 1)

        self.assertIsNotNone(grid.plan((0.5, 0.5), (4.5, 0.5)))
        self.assertIsNone(
            grid.plan((0.5, 0.5), (4.5, 0.5), dynamic_cells=[(2, 0)])
        )

    def test_unknown_is_blocked_without_expanding_over_free_cells(self):
        # unknown 本身不可通行，但机器人膨胀只围绕真实占据栅格。
        grid = make_grid(5, 3, unknown=[(2, 1)], robot_radius=1.0)
        self.assertFalse(grid.traversable((2, 1)))
        self.assertTrue(grid.traversable((3, 1)))

    def test_rotated_origin_and_negative_coordinates_use_floor(self):
        grid = make_grid(2, 2, origin_x=-1.0, origin_y=-1.0, origin_yaw=math.pi / 2.0)
        center = grid.cell_to_world(0, 0)
        self.assertEqual(grid.world_to_cell(*center), (0, 0))

        unrotated = make_grid(2, 2, origin_x=-1.0, origin_y=-1.0)
        self.assertIsNone(unrotated.world_to_cell(-1.1, -0.5))
        self.assertEqual(unrotated.world_to_cell(-0.1, -0.1), (0, 0))

    def test_diagonal_does_not_cut_blocked_corner(self):
        grid = make_grid(3, 3, occupied=[(1, 0), (0, 1)])
        self.assertIsNone(grid.plan((0.5, 0.5), (1.5, 1.5)))


class NavigationStateTest(unittest.TestCase):
    def test_latest_sensor_snapshot_rejects_regressing_stamp(self):
        self.assertTrue(is_non_decreasing_stamp(0, 100))
        self.assertTrue(is_non_decreasing_stamp(100, 100))
        self.assertTrue(is_non_decreasing_stamp(100, 101))
        self.assertFalse(is_non_decreasing_stamp(101, 100))

    def test_route_variants_use_one_planner_snapshot(self):
        class RecordingPlanner:
            def __init__(self):
                self.calls = []

            def plan_expanded(self, start, goal, dynamic_blocked=()):
                self.calls.append((start, goal, dynamic_blocked))
                return None if dynamic_blocked == {"blocked"} else [start, goal]

        planner = RecordingPlanner()
        static_route, dynamic_route = plan_route_variants(
            planner, (0.0, 0.0), (1.0, 0.0), {"blocked"}, True
        )

        self.assertEqual(static_route, [(0.0, 0.0), (1.0, 0.0)])
        self.assertIsNone(dynamic_route)
        self.assertEqual(len(planner.calls), 2)

    def test_preexpanded_dynamic_cells_are_not_expanded_again(self):
        class CountingGrid(InflatedOccupancyGrid):
            def __init__(self):
                super().__init__(5, 3, 1.0, 0.0, 0.0, 0.0, [0] * 15)
                self.expansion_calls = 0

            def expanded_cells(self, cells):
                self.expansion_calls += 1
                return super().expanded_cells(cells)

        grid = CountingGrid()
        dynamic_blocked = grid.expanded_cells([(2, 1)])
        grid.plan_expanded((0.5, 1.5), (4.5, 1.5), dynamic_blocked)
        grid.path_is_traversable_expanded(
            [(0.5, 0.5), (4.5, 0.5)], dynamic_blocked
        )

        self.assertEqual(grid.expansion_calls, 1)

    def test_identical_map_snapshots_keep_the_same_fingerprint(self):
        first = map_snapshot_fingerprint(
            2, 2, 0.05, -1.0, -1.0, 0.0, [0, 0, -1, 100]
        )
        same = map_snapshot_fingerprint(
            2, 2, 0.05, -1.0, -1.0, 0.0, [0, 0, -1, 100]
        )
        changed = map_snapshot_fingerprint(
            2, 2, 0.05, -1.0, -1.0, 0.0, [0, 100, -1, 100]
        )

        self.assertEqual(first, same)
        self.assertNotEqual(first, changed)

    def test_dynamic_blockage_waits_then_recovers_when_route_clears(self):
        wait = DynamicObstacleWait(timeout_s=2.0, retry_interval_s=0.2)
        wait.begin(10.0)

        self.assertFalse(wait.retry_due(10.1))
        self.assertTrue(wait.retry_due(10.2))
        self.assertFalse(wait.expired(11.9))
        wait.clear()

        self.assertIsNone(wait.blocked_since_s)
        self.assertFalse(wait.expired(12.0))

    def test_dynamic_blockage_expires_after_configured_timeout(self):
        wait = DynamicObstacleWait(timeout_s=2.0, retry_interval_s=0.2)
        wait.begin(10.0)

        self.assertFalse(wait.expired(11.99))
        self.assertTrue(wait.expired(12.0))

    def test_cancel_clears_goal_and_has_zero_velocity_semantics(self):
        state = GoalState()
        state.begin("goal-42")
        self.assertTrue(state.active)
        self.assertEqual(state.cancel("客户端取消"), (0.0, 0.0))
        self.assertFalse(state.active)
        self.assertFalse(state.controller_active)
        self.assertEqual(state.active_goal_id, "")
        self.assertEqual(state.failure_code, "CANCELED")

    def test_success_requires_xy_and_final_yaw(self):
        goal = (2.0, 3.0, math.pi / 2.0)
        self.assertFalse(goal_reached((2.01, 2.99, 0.0), goal, 0.1, 0.1))
        self.assertTrue(goal_reached((2.01, 2.99, math.pi / 2.0), goal, 0.1, 0.1))

    def test_health_state_uses_goal_id_progress_and_command_time(self):
        state = GoalState()
        state.begin("goal-health")
        state.record_command("发送命令时刻")
        lengths = path_lengths([(0.0, 0.0), (1.0, 0.0), (3.0, 0.0)])
        state.progress = path_progress(lengths, 1)
        self.assertEqual(state.active_goal_id, "goal-health")
        self.assertEqual(state.last_cmd_time, "发送命令时刻")
        self.assertAlmostEqual(state.progress, 1.0 / 3.0)
        state.finish("SUCCEEDED", "完成")
        self.assertEqual(state.progress, 1.0)
        self.assertEqual(state.failure_code, "SUCCEEDED")


if __name__ == "__main__":
    unittest.main()
