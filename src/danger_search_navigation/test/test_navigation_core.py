#!/usr/bin/env python3
"""共享规划核心和导航状态语义的 ROS 无关单元测试。"""

import math
import os
import sys
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import rospy
import tf.transformations
import yaml
from geometry_msgs.msg import Twist


SCRIPTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
PACKAGE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

from navigation_core import (
    GoalState,
    InflatedOccupancyGrid,
    goal_reached,
    path_lengths,
    path_progress,
)
from nav_controller import NavController


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
    def test_cached_opencv_inflation_matches_reference_offsets(self):
        width = height = 9
        data = [0] * (width * height)
        data[4 * width + 4] = 100
        data[1 * width + 1] = 65
        grid = InflatedOccupancyGrid(
            width, height, 0.05, 0.0, 0.0, 0.0, data,
            occupied_threshold=65, robot_radius=0.15, inflation_padding=0.05,
        )
        expected = [value != 0 for value in data]
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


class NavigationStateTest(unittest.TestCase):
    def test_obstacle_timeout_default_covers_map_rebuild_stall(self):
        with open(
            os.path.join(PACKAGE_DIR, "config", "default.yaml"),
            encoding="utf-8",
        ) as stream:
            config = yaml.safe_load(stream)

        self.assertEqual(config["obstacle_cloud_timeout"], 1.0)
        self.assertEqual(config["cruise_speed"], 0.35)
        self.assertEqual(config["max_linear_speed"], 0.35)
        self.assertAlmostEqual(config["lidar_pitch"], 0.0)
        self.assertAlmostEqual(config["goal_projection_max_radius"], 0.28)
        self.assertAlmostEqual(config["goal_projection_step"], 0.05)
        self.assertAlmostEqual(config["projection_tracking_tolerance"], 0.05)
        self.assertAlmostEqual(config["planning_failure_tolerance_s"], 0.8)

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
        controller.planner = None
        controller.goal_state = SimpleNamespace(active=False)
        controller.occupied_threshold = 65
        controller.robot_radius = 0.30
        controller.inflation_padding = 0.10
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
            def plan(_self, _start, candidate, _dynamic):
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
