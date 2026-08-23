#!/usr/bin/env python3
"""共享规划核心和导航状态语义的 ROS 无关单元测试。"""

import math
import os
import sys
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import rospy
import tf.transformations
import yaml
from geometry_msgs.msg import Twist


SCRIPTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
PACKAGE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

from navigation_core import (
    DynamicObstacleTemporalFilter,
    FootprintHistory,
    GoalState,
    InflatedOccupancyGrid,
    PoseProgressChecker,
    event_replan_required,
    goal_reached,
    path_lengths,
    path_progress,
    point_at_path_progress,
    project_to_polyline,
    remove_collinear_path_points,
)
from nav_controller import NavController, UrdfFootprintProvider


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
    def test_event_replan_ignores_elapsed_time_but_keeps_safety_events(self):
        self.assertFalse(event_replan_required(False, False, False, 0.79, 0.80))
        self.assertTrue(event_replan_required(True, False, False, 0.0, 0.80))
        self.assertTrue(event_replan_required(False, True, False, 0.0, 0.80))
        self.assertTrue(event_replan_required(False, False, True, 0.0, 0.80))
        self.assertTrue(event_replan_required(False, False, False, 0.81, 0.80))

    def test_blacklisted_start_can_escape_but_never_reenter(self):
        grid = make_grid(9, 5, robot_radius=0.0)
        blacklist = {(1, 2), (2, 2)}

        route = grid.plan(
            (1.5, 2.5), (7.5, 2.5), blacklist_cells=blacklist
        )

        self.assertIsNotNone(route)
        route_cells = [grid.world_to_cell(*point) for point in route]
        first_outside = next(
            index for index, cell in enumerate(route_cells) if cell not in blacklist
        )
        self.assertTrue(all(
            cell not in blacklist for cell in route_cells[first_outside:]
        ))
        self.assertTrue(grid.path_is_traversable(route, blacklist_cells=blacklist))
        self.assertFalse(grid.path_is_traversable(
            [(1.5, 2.5), (3.5, 2.5), (2.5, 2.5)],
            blacklist_cells=blacklist,
        ))

    def test_blacklisted_goal_is_still_rejected(self):
        grid = make_grid(7, 5, robot_radius=0.0)
        self.assertIsNone(grid.plan(
            (1.5, 2.5), (5.5, 2.5), blacklist_cells={(5, 2)}
        ))

    def test_blacklist_sweep_allows_monotonic_exit_only(self):
        grid = make_grid(30, 15, robot_radius=0.0)
        footprint = ((-0.2, -0.2), (0.2, -0.2), (0.2, 0.2), (-0.2, 0.2))
        blacklist = {(cell_x, cell_y) for cell_x in range(4, 8) for cell_y in range(6, 9)}

        safe, _ = grid.swept_path_metrics(
            [(5.5, 7.5), (10.5, 7.5)], footprint,
            blacklist_cells=blacklist,
            allow_blacklist_escape=True,
        )
        reentering, _ = grid.trajectory_metrics(
            [(5.5, 7.5, 0.0), (10.5, 7.5, 0.0), (5.5, 7.5, 0.0)],
            footprint,
            blacklist_cells=blacklist,
            allow_blacklist_escape=True,
        )

        self.assertTrue(safe)
        self.assertFalse(reentering)

    def test_blacklist_escape_translates_before_turning(self):
        grid = InflatedOccupancyGrid(
            80, 40, 0.10, 0.0, 0.0, 0.0, [0] * (80 * 40),
            robot_radius=0.0,
        )
        footprint = ((-0.35, -0.15), (0.30, -0.15),
                     (0.30, 0.15), (-0.35, 0.15))
        blacklist = {(20, 20)}

        safe, _ = grid.swept_path_metrics(
            [(2.05, 2.05), (3.05, 2.05)],
            footprint,
            blacklist_cells=blacklist,
            initial_yaw=math.pi / 2.0,
            final_yaw=0.0,
            allow_blacklist_escape=True,
        )

        self.assertTrue(safe)

    def test_vectorized_urdf_self_filter_matches_collision_volumes(self):
        provider = UrdfFootprintProvider("", ((-0.1, -0.1), (0.1, -0.1),
                                              (0.1, 0.1), (-0.1, 0.1)), 0.4, 0.04)
        provider.transformed = [
            (np.eye(4), ("box", (0.4, 0.2, 0.2))),
            (np.eye(4), ("sphere", (0.1,))),
        ]
        points = ((0.0, 0.0, 0.0), (0.19, 0.09, 0.09), (0.3, 0.0, 0.0))

        batch = provider.contains_many(points)

        self.assertEqual(batch.tolist(), [True, True, False])
        self.assertEqual(
            batch.tolist(), [provider.contains(point) for point in points]
        )
    def test_clearance_cost_and_blacklist_are_part_of_a_star(self):
        grid = make_grid(9, 7, robot_radius=0.0)
        blocked = {(4, 3)}
        route = grid.plan((1.5, 3.5), (7.5, 3.5), blacklist_cells=blocked)
        self.assertIsNotNone(route)
        self.assertNotIn((4, 3), [grid.world_to_cell(*point) for point in route])
        self.assertFalse(grid.path_is_traversable(
            [(1.5, 3.5), (4.5, 3.5)], blacklist_cells=blocked
        ))

    def test_swept_footprint_rejects_wall_corner(self):
        grid = make_grid(20, 20, occupied=[(10, 10)], robot_radius=0.0)
        footprint = ((-0.4, -0.3), (0.4, -0.3), (0.4, 0.3), (-0.4, 0.3))
        safe, _ = grid.swept_path_metrics(
            [(7.5, 10.5), (9.5, 10.5)], footprint
        )
        self.assertFalse(safe)

    def test_swept_footprint_checks_rotation_at_path_corner(self):
        width = height = 80
        data = [0] * (width * height)
        data[46 * width + 46] = 100
        grid = InflatedOccupancyGrid(
            width, height, 0.05, 0.0, 0.0, 0.0, data,
            robot_radius=0.0,
        )
        footprint = ((-0.50, -0.10), (0.50, -0.10),
                     (0.50, 0.10), (-0.50, 0.10))

        safe, _ = grid.swept_path_metrics(
            [(1.0, 2.0), (2.0, 2.0)], footprint, final_yaw=math.pi / 2.0
        )

        self.assertFalse(safe)

    def test_recovery_prefers_backup_then_clearer_lateral(self):
        grid = InflatedOccupancyGrid(
            40, 40, 0.10, -2.0, -2.0, 0.0, [0] * (40 * 40),
            robot_radius=0.0,
        )
        footprint = ((-0.15, -0.10), (0.15, -0.10), (0.15, 0.10), (-0.15, 0.10))
        backup = grid.choose_recovery((0.0, 0.0, 0.0), footprint)
        self.assertIsNotNone(backup)
        self.assertEqual(backup.maneuver, "BACKUP")
        self.assertGreaterEqual(backup.distance, 0.30)
        rear = grid.world_to_cell(-0.30, 0.0)
        left = grid.world_to_cell(0.0, 0.30)
        lateral = grid.choose_recovery(
            (0.0, 0.0, 0.0), footprint, dynamic_cells={rear, left}
        )
        self.assertIsNotNone(lateral)
        self.assertEqual(lateral.maneuver, "STRAFE_RIGHT")


class FootprintHistoryTest(unittest.TestCase):
    def test_history_covers_recent_leg_sweep_and_expires(self):
        fallback = ((-0.3, -0.1), (0.3, -0.1), (0.3, 0.1), (-0.3, 0.1))
        history = FootprintHistory(fallback, history_seconds=0.4, padding=0.04)
        first = history.update(1.0, fallback)
        swept = history.update(1.2, fallback + ((0.45, 0.0),))
        self.assertGreater(max(point[0] for point in swept), max(point[0] for point in first))
        expired = history.current(1.7)
        self.assertLess(max(point[0] for point in expired), 0.40)
    def test_cached_opencv_inflation_matches_reference_offsets(self):
        width = height = 9
        data = [0] * (width * height)
        data[4 * width + 4] = 100
        data[1 * width + 1] = 65
        grid = InflatedOccupancyGrid(
            width, height, 0.05, 0.0, 0.0, 0.0, data,
            occupied_threshold=65, robot_radius=0.15, inflation_padding=0.05,
        )
        expected = [value < 0 or value >= 65 for value in data]
        offsets = [
            (dx, dy)
            for dy in range(-4, 5)
            for dx in range(-4, 5)
            if math.hypot(dx * 0.05, dy * 0.05) <= 0.20 + 1e-9
        ]
        for index, value in enumerate(data):
            if value < 65:
                continue
            cell_x, cell_y = index % width, index // width
            for dx, dy in offsets:
                nx, ny = cell_x + dx, cell_y + dy
                if 0 <= nx < width and 0 <= ny < height:
                    expected[ny * width + nx] = True
        self.assertEqual(
            tuple(bool(value) for value in grid.inflated_blocked),
            tuple(expected),
        )

    def test_occupancy_threshold_allows_low_probability_noise(self):
        data = [0, 1, 24, 64, 65, 100, -1]
        grid = InflatedOccupancyGrid(
            len(data), 1, 1.0, 0.0, 0.0, 0.0, data,
            occupied_threshold=65,
        )

        self.assertTrue(all(grid.traversable((cell_x, 0)) for cell_x in range(4)))
        self.assertFalse(grid.traversable((4, 0)))
        self.assertFalse(grid.traversable((5, 0)))
        self.assertFalse(grid.traversable((6, 0)))

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

    def test_static_and_dynamic_inflation_radii_are_independent(self):
        resolution = 0.05
        width = height = 21
        center = (10, 10)
        data = [0] * (width * height)
        data[center[1] * width + center[0]] = 100
        grid = InflatedOccupancyGrid(
            width, height, resolution, 0.0, 0.0, 0.0, data,
            robot_radius=0.30,
            inflation_padding=0.03,
            dynamic_inflation_radius=0.30,
        )
        diagonal_cell = (15, 14)  # sqrt(0.25^2 + 0.20^2) ~= 0.320 m

        self.assertFalse(grid.traversable(diagonal_cell))
        self.assertNotIn(diagonal_cell, grid.expanded_cells([center]))

    def test_dynamic_footprint_clearing_frees_start_but_not_static_obstacles(self):
        width = height = 20
        resolution = 0.10
        start_cell = (10, 10)
        dynamic_cell = (13, 10)
        start_world = (1.05, 1.05)
        free_grid = InflatedOccupancyGrid(
            width, height, resolution, 0.0, 0.0, 0.0, [0] * (width * height),
            robot_radius=0.30, dynamic_inflation_radius=0.30,
        )
        dynamic_blocked = free_grid.expanded_cells(
            [dynamic_cell], dynamic_clear_world=start_world
        )

        self.assertNotIn(start_cell, dynamic_blocked)
        self.assertIn((16, 10), dynamic_blocked)
        self.assertTrue(free_grid.traversable(start_cell, dynamic_blocked))
        self.assertIsNone(
            free_grid.plan(start_world, (0.05, 1.05), [dynamic_cell])
        )
        self.assertIsNotNone(
            free_grid.plan(
                start_world, (0.05, 1.05), [dynamic_cell], start_world
            )
        )

        static_data = [0] * (width * height)
        static_data[start_cell[1] * width + start_cell[0]] = 100
        static_grid = InflatedOccupancyGrid(
            width, height, resolution, 0.0, 0.0, 0.0, static_data,
            robot_radius=0.30, dynamic_inflation_radius=0.30,
        )
        self.assertFalse(static_grid.traversable(start_cell, dynamic_blocked))

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

    def test_even_sized_mapper_places_world_zero_inside_center_cell(self):
        # OccupancyGrid serializes resolution as float32. Keeping zero half a
        # cell away from a boundary prevents rounding from selecting UNKNOWN.
        size = 1024
        resolution = 0.05000000074505806
        center = size // 2
        origin = -(center + 0.5) * 0.05
        data = [-1] * (size * size)
        data[center * size + center] = 0
        grid = InflatedOccupancyGrid(
            size, size, resolution, origin, origin, 0.0, data,
            robot_radius=0.0,
        )

        self.assertEqual(grid.world_to_cell(0.0, 0.0), (center, center))
        self.assertTrue(grid.traversable((center, center)))

    def test_diagonal_does_not_cut_blocked_corner(self):
        grid = make_grid(3, 3, occupied=[(1, 0), (0, 1)])
        self.assertIsNone(grid.plan((0.5, 0.5), (1.5, 1.5)))


class DynamicObstacleTemporalFilterTest(unittest.TestCase):
    def setUp(self):
        self.cell = (7, 11)
        self.filter = DynamicObstacleTemporalFilter(3, 2, 0.50)

    def test_single_frame_cell_is_not_confirmed(self):
        self.assertEqual(self.filter.observe(1.0, {self.cell}), set())

    def test_two_hits_in_three_frames_confirm_cell(self):
        self.filter.observe(1.0, {self.cell})
        self.filter.observe(1.1, set())
        self.assertEqual(self.filter.observe(1.2, {self.cell}), {self.cell})

    def test_duplicate_scan_timestamp_counts_once(self):
        self.filter.observe(1.0, {self.cell})
        self.filter.observe(1.1, set())
        self.assertEqual(self.filter.observe(1.0, {self.cell}), set())

    def test_confirmed_cell_survives_brief_disappearance_within_ttl(self):
        self.filter.observe(1.0, {self.cell})
        self.filter.observe(1.1, {self.cell})
        self.filter.observe(1.2, set())
        self.assertEqual(self.filter.confirmed_cells(1.59), {self.cell})

    def test_confirmed_cell_expires_after_ttl(self):
        self.filter.observe(1.0, {self.cell})
        self.filter.observe(1.1, {self.cell})
        self.assertEqual(self.filter.confirmed_cells(1.61), set())

    def test_valid_empty_frame_does_not_immediately_clear_confirmed_cell(self):
        self.filter.observe(1.0, {self.cell})
        self.filter.observe(1.1, {self.cell})
        self.assertEqual(self.filter.observe(1.2, set()), {self.cell})

    def test_observing_confirmed_cell_renews_its_ttl(self):
        self.filter.observe(1.0, {self.cell})
        self.filter.observe(1.1, {self.cell})
        self.filter.observe(1.2, set())
        self.filter.observe(1.3, set())
        self.filter.observe(1.4, set())
        self.assertEqual(self.filter.observe(1.45, {self.cell}), {self.cell})
        self.assertEqual(self.filter.confirmed_cells(1.94), {self.cell})
        self.assertEqual(self.filter.confirmed_cells(1.96), set())

    def test_reset_clears_scan_history_and_confirmed_cells(self):
        self.filter.observe(1.0, {self.cell})
        self.filter.observe(1.1, {self.cell})
        self.filter.reset()
        self.assertEqual(self.filter.confirmed_cells(1.1), set())
        self.assertEqual(self.filter.observe(1.2, {self.cell}), set())


class NavigationStateTest(unittest.TestCase):
    def test_pose_progress_checker_counts_continuous_rotation(self):
        checker = PoseProgressChecker(0.05, 0.20, 8.0, 0.05, 0.05)
        for step in range(31):
            self.assertFalse(checker.update(
                (0.0, 0.0, 0.04 * step), (0.0, 0.0, 0.20), 0.5 * step
            ))
        self.assertEqual(checker.mode, PoseProgressChecker.ROTATING)

    def test_pose_progress_checker_retimes_translation_after_rotation(self):
        checker = PoseProgressChecker(0.05, 0.20, 8.0, 0.05, 0.05)
        self.assertFalse(checker.update((0.0, 0.0, 0.0), (0.0, 0.0, 0.2), 0.0))
        self.assertFalse(checker.update((0.0, 0.0, 0.19), (0.0, 0.0, 0.2), 7.9))
        self.assertFalse(checker.update((0.0, 0.0, 0.19), (0.1, 0.0, 0.0), 8.0))
        self.assertFalse(checker.update((0.049, 0.0, 0.19), (0.1, 0.0, 0.0), 15.9))
        self.assertTrue(checker.update((0.049, 0.0, 0.19), (0.1, 0.0, 0.0), 16.0))

    def test_pose_progress_checker_idle_does_not_accumulate(self):
        checker = PoseProgressChecker(0.05, 0.20, 8.0, 0.05, 0.05)
        self.assertFalse(checker.update((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0))
        self.assertFalse(checker.update((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 100.0))
        self.assertEqual(checker.mode, PoseProgressChecker.IDLE)

    def test_pose_progress_checker_excludes_planning_pause(self):
        checker = PoseProgressChecker(0.05, 0.20, 8.0, 0.05, 0.05)
        self.assertFalse(checker.update((0.0, 0.0, 0.0), (0.1, 0.0, 0.0), 0.0))
        checker.pause(4.0)
        checker.resume(104.0)
        self.assertFalse(checker.update((0.0, 0.0, 0.0), (0.1, 0.0, 0.0), 107.9))
        self.assertTrue(checker.update((0.0, 0.0, 0.0), (0.1, 0.0, 0.0), 108.0))

    def test_no_valid_trajectory_uses_independent_three_second_clock(self):
        controller = NavController.__new__(NavController)
        controller.lock = threading.RLock()
        controller.blocked_since = None
        controller.blocked_recovery_timeout = 3.0
        controller.progress_checker = PoseProgressChecker()
        controller.goal_state = SimpleNamespace(stuck=False)
        controller.is_stuck = False
        pose = (0.0, 0.0, 0.0)
        command = (0.0, 0.0, 0.0)

        self.assertIsNone(controller._recovery_should_start(
            pose, command, False, rospy.Time.from_sec(10.0)
        ))
        self.assertIsNone(controller._recovery_should_start(
            pose, command, False, rospy.Time.from_sec(12.99)
        ))
        self.assertEqual(controller._recovery_should_start(
            pose, command, False, rospy.Time.from_sec(13.0)
        ), "NO_VALID_TRAJECTORY")

    def test_obstacle_timeout_default_covers_map_rebuild_stall(self):
        with open(
            os.path.join(PACKAGE_DIR, "config", "default.yaml"),
            encoding="utf-8",
        ) as stream:
            config = yaml.safe_load(stream)

        self.assertEqual(config["obstacle_cloud_timeout"], 1.0)
        self.assertEqual(config["dynamic_confirmation_frames"], 3)
        self.assertEqual(config["dynamic_confirmation_hits"], 2)
        self.assertAlmostEqual(config["dynamic_obstacle_ttl"], 0.50)
        self.assertAlmostEqual(config["inflation_padding"], 0.0)
        self.assertAlmostEqual(config["dynamic_inflation_radius"], 0.15)
        self.assertAlmostEqual(config["footprint_padding"], 0.04)
        self.assertAlmostEqual(config["clearance_soft_margin"], 0.15)
        self.assertAlmostEqual(config["clearance_cost_weight"], 1.0)
        self.assertAlmostEqual(config["dynamic_stop_distance"], 0.60)
        self.assertEqual(config["cruise_speed"], 0.35)
        self.assertEqual(config["max_linear_speed"], 0.35)
        self.assertAlmostEqual(config["lidar_pitch"], 0.0)
        self.assertAlmostEqual(config["goal_projection_max_radius"], 0.40)
        self.assertAlmostEqual(config["goal_projection_step"], 0.05)
        self.assertAlmostEqual(config["projection_tracking_tolerance"], 0.10)
        self.assertAlmostEqual(config["goal_tolerance_xy"], 0.45)
        self.assertAlmostEqual(config["goal_tolerance_yaw"], 0.35)
        self.assertAlmostEqual(config["goal_timeout"], 120.0)
        self.assertAlmostEqual(config["planning_failure_tolerance_s"], 5.0)
        self.assertAlmostEqual(config["replan_deviation_distance"], 0.80)
        self.assertNotIn("replan_period", config)
        self.assertAlmostEqual(config["progress_distance"], 0.03)
        self.assertAlmostEqual(config["progress_angle"], 0.12)
        self.assertAlmostEqual(config["stuck_timeout"], 12.0)
        self.assertAlmostEqual(config["blocked_recovery_timeout"], 5.0)
        self.assertAlmostEqual(config["recovery_speed"], 0.15)
        self.assertAlmostEqual(config["recovery_min_distance"], 0.20)
        self.assertAlmostEqual(config["recovery_time_allowance"], 15.0)
        self.assertAlmostEqual(config["recovery_no_progress_timeout"], 5.0)
        self.assertAlmostEqual(config["recovery_progress_distance"], 0.01)

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

    def test_navigation_publishes_terminal_health_before_action_result(self):
        source = os.path.join(SCRIPTS_DIR, "nav_controller.py")
        with open(source, encoding="utf-8") as stream:
            text = stream.read()
        finish = text.index("    def _finish_goal")
        preempt = text.index("    def preempt_cb", finish)
        body = text[finish:preempt]
        self.assertLess(body.index("self.publish_health()"), body.index("set_aborted"))

    def test_obstacle_subscription_keeps_only_the_latest_cloud(self):
        source = os.path.join(SCRIPTS_DIR, "nav_controller.py")
        with open(source, encoding="utf-8") as stream:
            text = stream.read()
        start = text.index("        self.obstacle_sub = rospy.Subscriber(")
        finish = text.index("        self.safety_stop_sub", start)

        self.assertIn("queue_size=1", text[start:finish])

    def test_obstacle_footprint_contains_edges_but_not_adjacent_points(self):
        controller = NavController.__new__(NavController)
        controller.robot_radius = 0.30
        controller.obstacle_footprint_min_x = -0.35
        controller.obstacle_footprint_max_x = 0.30
        controller.obstacle_footprint_min_y = -0.15
        controller.obstacle_footprint_max_y = 0.15

        self.assertTrue(controller._point_inside_obstacle_footprint(0.0, 0.0))
        self.assertTrue(controller._point_inside_obstacle_footprint(0.30, 0.15))
        self.assertTrue(controller._point_inside_obstacle_footprint(-0.35, -0.15))
        self.assertTrue(controller._point_inside_obstacle_footprint(0.18, 0.16))
        self.assertTrue(controller._point_inside_obstacle_footprint(0.30, 0.0))
        self.assertFalse(controller._point_inside_obstacle_footprint(0.301, 0.0))
        self.assertFalse(controller._point_inside_obstacle_footprint(0.0, -0.301))

    def test_obstacle_callback_filters_footprint_after_tf(self):
        controller = NavController.__new__(NavController)
        controller.lock = threading.RLock()
        controller.tf_listener = SimpleNamespace(
            lookupTransform=lambda *_args: ((0.20, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        )
        controller.base_frame = "base"
        controller.obstacle_max_points = 100
        controller.zero_point_radius = 0.05
        controller.obstacle_min_z = -0.30
        controller.obstacle_max_z = 0.80
        controller.obstacle_range_min = 0.15
        controller.obstacle_range_max = 8.00
        controller.robot_radius = 0.30
        controller.obstacle_footprint_min_x = -0.35
        controller.obstacle_footprint_max_x = 0.30
        controller.obstacle_footprint_min_y = -0.15
        controller.obstacle_footprint_max_y = 0.15
        controller.obstacle_tf_failure_count = 0
        controller.obstacle_invalid_count = 0
        controller.goal_state = SimpleNamespace(active=False)
        controller.require_obstacle_cloud = False
        controller.dynamic_obstacle_filter = DynamicObstacleTemporalFilter(3, 2, 0.50)
        stamp = rospy.Time.from_sec(10.0)
        cloud = SimpleNamespace(
            header=SimpleNamespace(frame_id="laser_livox", stamp=stamp),
            points=[
                SimpleNamespace(x=0.10, y=0.0, z=0.0),  # TF 后位于前边界。
                SimpleNamespace(x=-0.10, y=0.0, z=0.0),  # TF 后位于 footprint 内。
                SimpleNamespace(x=0.11, y=0.0, z=0.0),  # TF 后刚好位于 footprint 外。
                SimpleNamespace(x=0.31, y=0.0, z=0.0),  # 传感器坐标内，base 坐标外。
            ],
        )

        controller.obstacle_callback(cloud)

        self.assertEqual(controller.obstacle_stamp, stamp)
        self.assertTrue(controller.obstacle_frame_valid)
        self.assertEqual(
            controller.latest_obstacles_base,
            [(0.31, 0.0, 0.0), (0.51, 0.0, 0.0)],
        )

        controller.planner = InflatedOccupancyGrid(
            40, 20, 0.10, 0.0, 0.0, 0.0, [0] * 800,
            robot_radius=0.30, inflation_padding=0.03,
        )
        controller.current_pose = (1.0, 1.0, 0.0)
        controller.obstacle_cloud_timeout = 1.0
        controller.max_future_stamp_skew = 0.05
        controller.dynamic_front_half_angle = 0.52
        dynamic_cells, front_clearance, fresh = controller._dynamic_obstacle_snapshot(
            rospy.Time.from_sec(10.5)
        )
        self.assertTrue(fresh)
        # 单帧原始点必须立即触发前方净空保护，但不能直接传给 A*。
        self.assertEqual(dynamic_cells, set())
        self.assertAlmostEqual(front_clearance, 0.31)
        self.assertTrue(controller.planner.plan((0.05, 1.05), (3.95, 1.05)))

    def test_dynamic_snapshot_passes_only_confirmed_cells_to_a_star(self):
        controller = NavController.__new__(NavController)
        controller.lock = threading.RLock()
        controller.planner = InflatedOccupancyGrid(
            40, 20, 0.10, 0.0, 0.0, 0.0, [0] * 800,
            robot_radius=0.30,
            inflation_padding=0.03,
            dynamic_inflation_radius=0.30,
        )
        controller.current_pose = (1.0, 1.0, 0.0)
        controller.latest_obstacles_base = [(0.60, 0.0, 0.0)]
        controller.obstacle_frame_valid = True
        controller.obstacle_cloud_timeout = 1.0
        controller.max_future_stamp_skew = 0.05
        controller.dynamic_front_half_angle = 0.52
        controller.dynamic_obstacle_filter = DynamicObstacleTemporalFilter(3, 2, 0.50)

        controller.obstacle_stamp = rospy.Time.from_sec(10.0)
        first_cells, first_clearance, fresh = controller._dynamic_obstacle_snapshot(
            rospy.Time.from_sec(10.1)
        )
        self.assertTrue(fresh)
        self.assertEqual(first_cells, set())
        self.assertAlmostEqual(first_clearance, 0.60)

        controller.obstacle_stamp = rospy.Time.from_sec(10.2)
        confirmed_cells, _, fresh = controller._dynamic_obstacle_snapshot(
            rospy.Time.from_sec(10.3)
        )
        self.assertTrue(fresh)
        self.assertEqual(confirmed_cells, {(16, 10)})
        self.assertIsNone(
            controller.planner.plan((0.05, 1.05), (1.65, 1.05), confirmed_cells)
        )

    def test_dynamic_snapshot_filters_static_background_but_keeps_clearance(self):
        controller = NavController.__new__(NavController)
        controller.lock = threading.RLock()
        data = [0] * (20 * 10)
        data[5 * 20 + 16] = 100
        data[5 * 20 + 19] = -1
        controller.planner = InflatedOccupancyGrid(
            20, 10, 0.10, 0.0, 0.0, 0.0, data,
            robot_radius=0.30,
            inflation_padding=0.03,
            dynamic_inflation_radius=0.30,
        )
        controller.current_pose = (1.0, 0.5, 0.0)
        # (15, 5) 位于静态膨胀区，(16, 5) 是静态占据格，(19, 5) 未知。
        controller.latest_obstacles_base = [
            (0.50, 0.0, 0.0), (0.60, 0.0, 0.0), (0.90, 0.0, 0.0)
        ]
        controller.obstacle_frame_valid = True
        controller.obstacle_cloud_timeout = 1.0
        controller.max_future_stamp_skew = 0.05
        controller.dynamic_front_half_angle = 0.52
        controller.dynamic_obstacle_filter = DynamicObstacleTemporalFilter(3, 2, 0.50)

        for scan_time in (10.0, 10.2):
            controller.obstacle_stamp = rospy.Time.from_sec(scan_time)
            dynamic_cells, front_clearance, fresh = controller._dynamic_obstacle_snapshot(
                rospy.Time.from_sec(scan_time + 0.1)
            )
            self.assertTrue(fresh)
            self.assertEqual(dynamic_cells, set())
            self.assertAlmostEqual(front_clearance, 0.50)

    def test_confirmed_dynamic_cell_is_removed_after_static_map_update(self):
        controller = NavController.__new__(NavController)
        controller.lock = threading.RLock()
        controller.planner = InflatedOccupancyGrid(
            40, 20, 0.10, 0.0, 0.0, 0.0, [0] * 800,
            robot_radius=0.30,
            dynamic_inflation_radius=0.30,
        )
        controller.current_pose = (1.0, 1.0, 0.0)
        controller.latest_obstacles_base = [(0.60, 0.0, 0.0)]
        controller.obstacle_frame_valid = True
        controller.obstacle_cloud_timeout = 1.0
        controller.max_future_stamp_skew = 0.05
        controller.dynamic_front_half_angle = 0.52
        controller.dynamic_obstacle_filter = DynamicObstacleTemporalFilter(3, 2, 0.50)

        for scan_time in (10.0, 10.2):
            controller.obstacle_stamp = rospy.Time.from_sec(scan_time)
            confirmed_cells, _, _ = controller._dynamic_obstacle_snapshot(
                rospy.Time.from_sec(scan_time + 0.1)
            )
        self.assertEqual(confirmed_cells, {(16, 10)})

        updated_data = [0] * 800
        updated_data[10 * 40 + 16] = 100
        controller.planner = InflatedOccupancyGrid(
            40, 20, 0.10, 0.0, 0.0, 0.0, updated_data,
            robot_radius=0.30,
            dynamic_inflation_radius=0.30,
        )
        controller.latest_obstacles_base = []
        controller.obstacle_stamp = rospy.Time.from_sec(10.3)
        dynamic_cells, _, fresh = controller._dynamic_obstacle_snapshot(
            rospy.Time.from_sec(10.35)
        )

        self.assertTrue(fresh)
        self.assertEqual(dynamic_cells, set())

    def test_static_wall_echoes_do_not_close_a_narrow_corridor(self):
        controller = NavController.__new__(NavController)
        controller.lock = threading.RLock()
        width, height = 12, 7
        data = [0] * (width * height)
        for cell_x in range(width):
            data[cell_x] = 100
            data[(height - 1) * width + cell_x] = 100
        controller.planner = InflatedOccupancyGrid(
            width, height, 0.10, 0.0, 0.0, 0.0, data,
            robot_radius=0.10,
            dynamic_inflation_radius=0.20,
        )
        start = (0.15, 0.35)
        goal = (1.05, 0.35)
        controller.current_pose = (start[0], start[1], 0.0)
        # 两个点分别位于上下墙的静态膨胀区；若作为动态点二次膨胀，
        # 会在 x=5 处合并并封住整个三格宽的可通行走廊。
        controller.latest_obstacles_base = [(0.40, -0.20, 0.0), (0.40, 0.20, 0.0)]
        controller.obstacle_frame_valid = True
        controller.obstacle_cloud_timeout = 1.0
        controller.max_future_stamp_skew = 0.05
        controller.dynamic_front_half_angle = 0.52
        controller.dynamic_obstacle_filter = DynamicObstacleTemporalFilter(3, 2, 0.50)

        self.assertTrue(controller.planner.plan(start, goal))
        self.assertIsNone(controller.planner.plan(start, goal, {(5, 1), (5, 5)}))
        for scan_time in (10.0, 10.2):
            controller.obstacle_stamp = rospy.Time.from_sec(scan_time)
            dynamic_cells, front_clearance, fresh = controller._dynamic_obstacle_snapshot(
                rospy.Time.from_sec(scan_time + 0.1)
            )

        self.assertTrue(fresh)
        self.assertEqual(dynamic_cells, set())
        self.assertAlmostEqual(front_clearance, math.hypot(0.40, 0.20))
        self.assertTrue(controller.planner.plan(start, goal, dynamic_cells))

    def test_remaining_path_ignores_obstacles_behind_robot(self):
        controller = NavController.__new__(NavController)
        controller.lock = threading.RLock()
        controller.waypoint_index = 0
        controller.planner = make_grid(
            5, 1, robot_radius=0.0, dynamic_inflation_radius=0.0
        )
        route = [(0.5, 0.5), (1.5, 0.5), (2.5, 0.5), (3.5, 0.5), (4.5, 0.5)]
        current = (2.5, 0.5)
        remaining = controller._remaining_route(current, route)

        self.assertEqual(remaining, route[2:])
        self.assertFalse(controller._path_is_blocked(remaining, {(0, 0)}, current))
        self.assertTrue(controller._path_is_blocked(remaining, {(3, 0)}, current))

    def test_control_timer_publishes_zero_while_planning(self):
        controller = NavController.__new__(NavController)
        controller.lock = threading.RLock()
        controller.goal_state = SimpleNamespace(
            active=True,
            record_command=lambda stamp: setattr(controller, "recorded_stamp", stamp),
        )
        controller.planning_active = True
        controller.last_cmd_time = rospy.Time(0)

        class Publisher:
            def __init__(self):
                self.messages = []

            def publish(self, message):
                self.messages.append(message)

        controller.cmd_pub = Publisher()
        with patch("nav_controller.rospy.Time.now", return_value=rospy.Time.from_sec(1.0)):
            controller.control_loop(None)
        self.assertEqual(len(controller.cmd_pub.messages), 1)
        message = controller.cmd_pub.messages[0]
        self.assertIsInstance(message, Twist)
        self.assertEqual(message.linear.x, 0.0)
        self.assertEqual(message.angular.z, 0.0)
        self.assertNotEqual(controller.last_cmd_time, rospy.Time(0))

    def test_map_content_dedup_keeps_planner_and_generation(self):
        controller = NavController.__new__(NavController)
        controller.lock = threading.RLock()
        controller.map_frame = "map"
        controller.map_valid = False
        controller.map_data = None
        controller.map_geometry = None
        controller.map_stamp = rospy.Time(0)
        controller.map_generation = 0
        controller.map_unchanged_count = 0
        controller.map_build_count = 0
        controller.dynamic_obstacle_geometry = None
        controller.dynamic_obstacle_filter = DynamicObstacleTemporalFilter(3, 2, 0.50)
        controller.planner = None
        controller.goal_state = SimpleNamespace(active=False)
        controller.occupied_threshold = 65
        controller.robot_radius = 0.30
        controller.inflation_padding = 0.10
        controller.dynamic_inflation_radius = 0.30
        controller.allow_diagonal = True
        controller.max_expansions = 100
        controller.quaternion_norm_tolerance = 0.05
        origin = SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        info = SimpleNamespace(width=2, height=2, resolution=1.0, origin=origin)
        make_message = lambda stamp, data: SimpleNamespace(
            header=SimpleNamespace(frame_id="map", stamp=stamp),
            info=info,
            data=data,
        )
        with patch("nav_controller.rospy.loginfo_throttle"), patch(
            "nav_controller.rospy.logdebug_throttle"
        ), patch("nav_controller.rospy.logwarn_throttle"):
            with patch("nav_controller.InflatedOccupancyGrid", return_value=object()) as build:
                controller.map_callback(
                    make_message(rospy.Time.from_sec(1.0), [0, 0, 0, 0])
                )
                first_planner = controller.planner
                controller.map_callback(
                    make_message(rospy.Time.from_sec(2.0), [0, 0, 0, 0])
                )
                self.assertEqual(build.call_count, 1)
                self.assertIs(controller.planner, first_planner)
                self.assertEqual(controller.map_generation, 1)
                self.assertEqual(controller.map_unchanged_count, 1)
                controller.map_callback(
                    make_message(rospy.Time.from_sec(3.0), [0, 100, 0, 0])
                )
        self.assertEqual(build.call_count, 2)
        self.assertEqual(controller.map_generation, 2)


class NavigationPathTrackingTest(unittest.TestCase):
    def setUp(self):
        self.controller = NavController.__new__(NavController)
        self.controller.lock = threading.RLock()
        self.controller.waypoint_index = 0
        self.controller.lookahead_distance = 0.60
        self.controller.cruise_speed = 0.35
        self.controller.max_linear_speed = 0.35
        self.controller.rotate_in_place_angle = 0.45
        self.controller.rotate_in_place_gain = 1.50

    def test_straight_path_uses_full_cruise_speed(self):
        linear_x, angular_z = self.controller._path_tracking_command(
            (0.0, 0.0, 0.0),
            [(0.0, 0.0), (2.0, 0.0)],
        )

        self.assertAlmostEqual(linear_x, 0.35)
        self.assertAlmostEqual(angular_z, 0.0)

    def test_heading_error_reduces_linear_speed(self):
        heading = 0.20
        linear_x, angular_z = self.controller._path_tracking_command(
            (0.0, 0.0, 0.0),
            [(0.0, 0.0), (2.0 * math.cos(heading), 2.0 * math.sin(heading))],
        )

        self.assertAlmostEqual(linear_x, 0.35 * math.cos(heading))
        self.assertGreater(angular_z, 0.0)
        self.assertLess(linear_x, 0.40)

    def test_polyline_projection_is_continuous_not_nearest_grid_point(self):
        path = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
        distance, progress, segment, point, tangent = project_to_polyline(
            (0.62, 0.12), path, path_lengths(path)
        )
        self.assertAlmostEqual(distance, 0.12)
        self.assertAlmostEqual(progress, 0.62)
        self.assertEqual(segment, 0)
        self.assertEqual(point, (0.62, 0.0))
        self.assertAlmostEqual(tangent, 0.0)

    def test_forward_point_uses_arc_length_across_corner(self):
        path = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
        point, tangent = point_at_path_progress(path, path_lengths(path), 1.25)
        self.assertEqual(point, (1.0, 0.25))
        self.assertAlmostEqual(tangent, math.pi / 2.0)

    def test_collinear_cell_points_are_removed_without_cutting_corner(self):
        path = [(0.0, 0.0), (0.05, 0.0), (0.10, 0.0), (0.10, 0.05)]
        self.assertEqual(
            remove_collinear_path_points(path),
            [(0.0, 0.0), (0.10, 0.0), (0.10, 0.05)],
        )

    def test_dynamic_window_starts_from_post_mux_velocity(self):
        self.controller.control_rate = 20.0
        self.controller.sent_command = (0.20, 0.0, 0.10)
        self.controller.sent_command_stamp = rospy.Time.from_sec(10.0)
        self.controller.local_linear_accel = 1.0
        self.controller.local_lateral_accel = 1.0
        self.controller.local_angular_accel = 2.0
        self.controller.max_linear_speed = 0.35
        self.controller.local_lateral_speed = 0.08
        self.controller.path_align_max_angular_speed = 0.35
        window = self.controller._dynamic_window(rospy.Time.from_sec(10.05))
        self.assertAlmostEqual(window[0], 0.15)
        self.assertAlmostEqual(window[1], 0.25)
        self.assertAlmostEqual(window[4], 0.0)
        self.assertAlmostEqual(window[5], 0.20)


class NavigationGoalProjectionTest(unittest.TestCase):
    def setUp(self):
        self.controller = NavController.__new__(NavController)
        self.controller.lock = threading.RLock()
        self.controller.planner = None
        self.controller.goal_projection_max_radius = 0.28
        self.controller.goal_projection_step = 0.05
        self.controller.goal_tolerance_xy = 0.30
        self.controller.goal_tolerance_yaw = 0.20
        self.controller.projection_tracking_tolerance = 0.05

    def _grid(self, occupied=()):
        width, height, resolution = 80, 40, 0.05
        data = [0] * (width * height)
        for cell_x, cell_y in occupied:
            data[cell_y * width + cell_x] = 100
        return InflatedOccupancyGrid(
            width, height, resolution, 0.0, 0.0, 0.0, data,
            robot_radius=0.0,
        )

    def test_free_goal_is_not_projected(self):
        self.controller.planner = self._grid()
        route, target, projected = self.controller._plan_goal_path(
            (0.125, 0.525), (1.525, 0.525, 0.0)
        )
        self.assertIsNotNone(route)
        self.assertEqual(target, (1.525, 0.525, 0.0))
        self.assertFalse(projected)

    def test_blocked_goal_projects_to_lateral_free_cell(self):
        self.controller.planner = self._grid(occupied=[(30, 10)])
        route, target, projected = self.controller._plan_goal_path(
            (0.125, 0.525), (1.525, 0.525, 0.0)
        )
        self.assertIsNotNone(route)
        self.assertTrue(projected)
        self.assertLessEqual(
            math.hypot(target[0] - 1.525, target[1] - 0.525), 0.30
        )
        self.assertNotEqual(target[:2], (1.525, 0.525))

    def test_projected_goal_does_not_succeed_from_tracking_tolerance_only(self):
        requested = (2.8413, 0.0358, 0.1074)
        current = (2.4436, -0.0070, 0.1074)
        projected = (2.6761, 0.1689, 0.1074)

        self.assertLess(math.hypot(current[0] - projected[0], current[1] - projected[1]), 0.30)
        self.assertEqual(self.controller._tracking_tolerance(True), 0.05)
        self.assertFalse(self.controller._requested_goal_reached(current, requested))

    def test_projection_radius_reserves_tracking_tolerance(self):
        candidates = self.controller._goal_candidates((1.0, 1.0), 0.0, (0.0, 1.0))
        self.assertTrue(candidates)
        self.assertLessEqual(
            max(math.hypot(x - 1.0, y - 1.0) for x, y in candidates),
            self.controller.goal_tolerance_xy - self.controller.projection_tracking_tolerance,
        )

    def test_projection_rejects_candidate_materially_behind_start(self):
        candidates = self.controller._goal_candidates((0.10, 0.0), 0.0, (0.0, 0.0))
        self.assertTrue(candidates)
        self.assertTrue(all(x >= -0.05 - 1e-9 for x, _ in candidates))

    def test_projection_prefers_nearest_goal_offset_before_route_length(self):
        class OffsetPlanner:
            def plan(_self, _start, candidate, _dynamic, _dynamic_clear_world=None):
                offset = math.hypot(candidate[0] - 1.0, candidate[1])
                if offset < 1e-9:
                    return None
                if offset <= 0.051:
                    return [(0.0, 0.0), (0.0, 2.0), candidate]
                return [(0.0, 0.0), candidate]

        self.controller.planner = OffsetPlanner()
        route, target, projected = self.controller._plan_goal_path(
            (0.0, 0.0), (1.0, 0.0, 0.0)
        )
        self.assertTrue(projected)
        self.assertIsNotNone(route)
        self.assertAlmostEqual(math.hypot(target[0] - 1.0, target[1]), 0.05)

    def test_projection_returns_none_when_all_candidates_are_blocked(self):
        occupied = [(x, y) for x in range(20, 42) for y in range(0, 40)]
        self.controller.planner = self._grid(occupied=occupied)
        route, target, projected = self.controller._plan_goal_path(
            (0.125, 0.525), (1.525, 0.525, 0.0)
        )
        self.assertIsNone(route)
        self.assertIsNone(target)
        self.assertFalse(projected)

    def test_tf_transform_uses_translation_without_manual_pitch(self):
        rotation = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        point = self.controller._transform_point(
            ((0.2, 0.0, 0.08), rotation), 1.0, 0.0, 0.0
        )
        self.assertAlmostEqual(point[0], 1.2)
        self.assertAlmostEqual(point[1], 0.0)
        self.assertAlmostEqual(point[2], 0.08)

    def test_batch_tf_transform_matches_single_point_transform(self):
        rotation = tf.transformations.quaternion_from_euler(0.1, -0.2, 0.3)
        transform = ((0.2, -0.1, 0.08), rotation)
        points = [(1.0, 0.0, 0.2), (-0.5, 0.3, 0.1)]
        expected = tuple(
            NavController._transform_point(transform, *point)
            for point in points
        )
        actual = NavController._transform_points(transform, points)
        for actual_point, expected_point in zip(actual, expected):
            for actual_value, expected_value in zip(actual_point, expected_point):
                self.assertAlmostEqual(actual_value, expected_value)


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
        self.controller.obstacle_cloud_timeout = 1.0
        self.controller.require_obstacle_cloud = True
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

    def test_obstacle_cloud_is_accepted_at_nine_tenths_of_a_second(self):
        self.controller.obstacle_frame_valid = True
        self.controller.obstacle_stamp = rospy.Time.from_sec(99.1)

        ready, code, detail = self.controller._navigation_readiness(
            rospy.Time.from_sec(100.0), require_obstacles=True
        )

        self.assertTrue(ready)
        self.assertEqual(code, "NONE")
        self.assertEqual(detail, "")

    def test_obstacle_cloud_older_than_one_second_is_rejected(self):
        self.controller.obstacle_frame_valid = True
        self.controller.obstacle_stamp = rospy.Time.from_sec(98.9)

        ready, code, detail = self.controller._navigation_readiness(
            rospy.Time.from_sec(100.0), require_obstacles=True
        )

        self.assertFalse(ready)
        self.assertEqual(code, "CONTROL_FAILED")
        self.assertIn("/scan", detail)

    def test_invalid_obstacle_cloud_frame_is_rejected(self):
        self.controller.obstacle_frame_valid = False
        self.controller.obstacle_stamp = rospy.Time.from_sec(99.9)

        ready, code, detail = self.controller._navigation_readiness(
            rospy.Time.from_sec(100.0), require_obstacles=True
        )

        self.assertFalse(ready)
        self.assertEqual(code, "CONTROL_FAILED")
        self.assertIn("/scan", detail)


if __name__ == "__main__":
    unittest.main()
