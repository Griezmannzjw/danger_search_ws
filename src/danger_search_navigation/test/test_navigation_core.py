#!/usr/bin/env python3
"""共享规划核心和导航状态语义的 ROS 无关单元测试。"""

import math
import os
import sys
import threading
import time
import unittest
from types import SimpleNamespace

import rospy


SCRIPTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

from navigation_core import (
    GoalState,
    InflatedOccupancyGrid,
    goal_reached,
    path_lengths,
    path_progress,
)
from nav_controller import LatestMapWorker, NavController


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

    def test_unknown_is_blocked_without_expanding_over_free_cells(self):
        # unknown 本身不可通行，但机器人膨胀只围绕真实占据栅格。
        grid = make_grid(5, 3, unknown=[(2, 1)], robot_radius=1.0)
        self.assertFalse(grid.traversable((2, 1)))
        self.assertTrue(grid.traversable((3, 1)))

    def test_action_can_approach_unknown_goal_without_entering_unknown(self):
        unknown = [(cell_x, 1) for cell_x in range(4, 8)]
        grid = make_grid(8, 3, unknown=unknown)

        route = grid.plan_toward_unknown((0.5, 1.5), (7.5, 1.5))

        self.assertIsNotNone(route)
        self.assertLess(route[-1][0], 4.0)
        self.assertTrue(grid.path_is_traversable(route))

    def test_action_does_not_approach_occupied_goal_as_unknown(self):
        grid = make_grid(8, 3, occupied=[(7, 1)])

        self.assertIsNone(
            grid.plan_toward_unknown((0.5, 1.5), (7.5, 1.5))
        )

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


class LatestMapWorkerTest(unittest.TestCase):
    def test_inflight_old_map_cannot_replace_newest_pending_map(self):
        first_build_started = threading.Event()
        release_first_build = threading.Event()
        installed = []

        def build(value):
            if value == "old":
                first_build_started.set()
                release_first_build.wait(1.0)
            return value

        worker = LatestMapWorker(build, installed.append)
        try:
            worker.submit("old")
            self.assertTrue(first_build_started.wait(1.0))
            worker.submit("new")
            release_first_build.set()
            deadline = time.monotonic() + 1.0
            while installed != ["new"] and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertEqual(installed, ["new"])
        finally:
            worker.stop()

    def test_invalidation_discards_inflight_map(self):
        build_started = threading.Event()
        release_build = threading.Event()
        installed = []

        def build(value):
            build_started.set()
            release_build.wait(1.0)
            return value

        worker = LatestMapWorker(build, installed.append)
        try:
            worker.submit("map")
            self.assertTrue(build_started.wait(1.0))
            worker.invalidate()
            release_build.set()
            time.sleep(0.05)
            self.assertEqual(installed, [])
        finally:
            worker.stop()


class NavigationReadinessTest(unittest.TestCase):
    def setUp(self):
        self.controller = NavController.__new__(NavController)
        self.controller.lock = threading.RLock()
        self.controller.pose_valid = True
        self.controller.pose_stamp = rospy.Time.from_sec(1.0)
        self.controller.pose_timeout = 1.0
        self.controller.map_valid = True
        self.controller.planner = object()
        self.controller.map_stamp = rospy.Time.from_sec(1.0)
        self.controller.map_timeout = 2.0
        self.controller.mapping_status = SimpleNamespace(
            ready=True,
            stable=True,
            lost=False,
            status_reason="TRACKING",
        )
        self.controller.mapping_status_stamp = rospy.Time.from_sec(99.5)
        self.controller.mapping_status_timeout = 1.5
        self.controller.obstacle_frame_valid = False
        self.controller.obstacle_stamp = rospy.Time(0)
        self.controller.obstacle_cloud_timeout = 0.5
        self.controller.safety_stop = False
        self.controller.max_future_stamp_skew = 0.05

    def test_healthy_mapping_status_owns_source_freshness(self):
        ready, code, detail = self.controller._navigation_readiness(
            rospy.Time.from_sec(100.0), require_obstacles=False
        )

        self.assertTrue(ready)
        self.assertEqual(code, "NONE")
        self.assertEqual(detail, "")

    def test_unstable_mapping_status_still_blocks_navigation(self):
        self.controller.mapping_status.stable = False
        self.controller.mapping_status.status_reason = (
            "GICP_ODOMETRY_DEGRADED_HOLDING_LAST_POSE"
        )

        ready, code, detail = self.controller._navigation_readiness(
            rospy.Time.from_sec(100.0), require_obstacles=False
        )

        self.assertFalse(ready)
        self.assertEqual(code, "LOCALIZATION_LOST")
        self.assertEqual(detail, "GICP_ODOMETRY_DEGRADED_HOLDING_LAST_POSE")


if __name__ == "__main__":
    unittest.main()
