#!/usr/bin/env python3
"""多楼层探索单元测试：电梯井自主检测（门缝发现）。

只测 ROS 无关的 _detect_elevator_halls / _perimeter_door_gaps 逻辑。
"""

import importlib.util
import math
import pathlib
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
import yaml


SCRIPT = pathlib.Path(__file__).parents[1] / "scripts" / "exploration_planner.py"
SPEC = importlib.util.spec_from_file_location("exploration_planner", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def make_planner(grid, resolution=0.10):
    planner = MODULE.ExplorationPlanner.__new__(MODULE.ExplorationPlanner)
    size = grid.shape[0]
    planner.map_data = np.array(grid, dtype=np.int8)
    planner.map_info = SimpleNamespace(
        width=size,
        height=size,
        resolution=resolution,
        origin=SimpleNamespace(
            position=SimpleNamespace(x=-size * resolution / 2.0,
                                     y=-size * resolution / 2.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )
    planner.connectivity_occupied_threshold = 65
    planner.shaft_min_area_m2 = 4.0
    planner.shaft_max_area_m2 = 50.0
    planner.door_gap_min_width_m = 0.8
    planner.door_gap_max_width_m = 2.5
    planner.free_threshold = 40
    return planner


def build_shaft_map(size_cells, shaft=(1.5, 4.0, 1.0, 4.0), door=(2.0, 3.0)):
    """自由地图 + 一个矩形井道，西墙带门缝。所有值：0=自由, 100=占用。"""
    res = 0.10
    half = size_cells * res / 2.0
    grid = np.zeros((size_cells, size_cells), dtype=np.int8)

    def setrect(x1, y1, x2, y2, val):
        for cx in range(int((x1 + half) / res), int((x2 + half) / res) + 1):
            for cy in range(int((y1 + half) / res), int((y2 + half) / res) + 1):
                if 0 <= cx < size_cells and 0 <= cy < size_cells:
                    grid[cy, cx] = val

    x0, x1, y0, y1 = shaft
    d0, d1 = door
    setrect(x0, y0, x0 + 0.2, d0, 100)        # 西墙下段
    setrect(x0, d1, x0 + 0.2, y1, 100)        # 西墙上段
    setrect(x0, y0, x1, y0 + 0.2, 100)        # 南墙
    setrect(x0, y1 - 0.2, x1, y1, 100)        # 北墙
    setrect(x1 - 0.2, y0, x1, y1, 100)        # 东墙
    return grid


class ElevatorDetectionTest(unittest.TestCase):
    def test_shaft_with_west_door_is_detected(self):
        planner = make_planner(build_shaft_map(400))
        halls = planner._detect_elevator_halls()
        self.assertTrue(halls, "应当检测到至少一个电梯厅候选")
        # 电梯门缝中心约 (1.5, 2.5)，外侧应落在其附近
        self.assertTrue(
            any(math.hypot(hx - 1.5, hy - 2.5) < 1.0 for hx, hy, _ in halls),
            "应在西墙门缝附近找到候选",
        )

    def test_door_yaw_points_into_shaft(self):
        planner = make_planner(build_shaft_map(400))
        halls = planner._detect_elevator_halls()
        west = [yaw for hx, hy, yaw in halls
                if abs(hx - 1.5) < 1.0 and abs(hy - 2.5) < 1.0]
        self.assertTrue(west, "应找到西墙门缝候选")
        # 西墙门缝朝井道内 = +x = yaw 0
        self.assertLess(abs(west[0]), 0.2)

    def test_no_shaft_no_candidates(self):
        planner = make_planner(np.zeros((200, 200), dtype=np.int8))
        self.assertEqual(planner._detect_elevator_halls(), [])

    def test_sealed_shaft_without_door_is_not_candidate(self):
        grid = np.zeros((400, 400), dtype=np.int8)
        half = 20.0
        res = 0.10
        # 封闭的实心矩形，无门缝
        for cx in range(int((1.5 + half) / res), int((4.0 + half) / res) + 1):
            for cy in range(int((1.0 + half) / res), int((4.0 + half) / res) + 1):
                grid[cy, cx] = 100
        planner = make_planner(grid)
        self.assertEqual(planner._detect_elevator_halls(), [])

    def test_shaft_is_detected_when_wall_joins_building_wall(self):
        grid = build_shaft_map(400)
        # Join the north shaft wall to the map boundary. The original occupied
        # component now has a huge bounding box and cannot pass the legacy
        # isolated-component heuristic.
        res = 0.10
        half = 20.0
        wall_y = int((3.9 + half) / res)
        shaft_x = int((1.5 + half) / res)
        grid[wall_y:wall_y + 3, :shaft_x + 1] = 100
        planner = make_planner(grid)

        halls = planner._detect_elevator_halls()

        self.assertTrue(any(
            math.hypot(hx - 1.5, hy - 2.5) < 1.0
            for hx, hy, _yaw in halls
        ))

    def test_two_shafts_produce_distinct_candidates(self):
        grid = build_shaft_map(500)
        second = build_shaft_map(
            500, shaft=(-8.0, -5.5, -4.0, -1.0), door=(-3.2, -2.0)
        )
        grid = np.maximum(grid, second)
        planner = make_planner(grid)

        halls = planner._detect_elevator_halls()

        self.assertTrue(any(hx > 0.0 for hx, _hy, _yaw in halls))
        self.assertTrue(any(hx < -4.0 for hx, _hy, _yaw in halls))

    def test_wide_room_recess_is_not_a_door_candidate(self):
        grid = np.zeros((300, 300), dtype=np.int8)
        # Three-sided room recess with a 4 m opening, well beyond the public
        # elevator-door contract.
        grid[80:83, 80:160] = 100
        grid[80:160, 80:83] = 100
        grid[80:160, 157:160] = 100
        planner = make_planner(grid)
        self.assertEqual(planner._detect_elevator_halls(), [])


class PublicTopologyTest(unittest.TestCase):
    def setUp(self):
        self.topology = {
            "served_floors": [0, 1, 2, 3],
            "elevators": [
                {"id": "low", "served_floors": [0, 1, 2]},
                {"id": "high", "served_floors": [2, 3]},
            ],
        }

    def test_shortest_route_supports_transfer(self):
        self.assertEqual(
            MODULE.shortest_floor_route(self.topology, 0, 3),
            [(2, "low"), (3, "high")],
        )

    def test_next_floor_is_not_current_plus_one(self):
        selection = MODULE.select_next_floor_transition(
            self.topology, current_floor=0, completed_floors={0, 1}
        )
        self.assertEqual(selection["final_target"], 2)
        self.assertEqual(selection["next_floor"], 2)
        self.assertEqual(selection["elevator_id"], "low")

    def test_public_contract_parser_rejects_non_public_filename(self):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "building_config.json"
            path.write_text("{}", encoding="utf-8")
            with self.assertRaises(ValueError):
                MODULE.load_public_scene_topology(str(path))

    def test_door_animation_fits_independent_service_timeout(self):
        config_path = pathlib.Path(__file__).parents[1] / "config" / "default.yaml"
        with config_path.open(encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        self.assertGreaterEqual(config["elevator_service_timeout_s"], 40.0)
        self.assertEqual(config["elevator_crossing_timeout_s"], 20.0)
        self.assertEqual(config["floor_map_stable_time_s"], 15.0)
        self.assertEqual(config["active_map_topic"], "/mapping/active_map")
        self.assertGreater(config["elevator_footprint_margin_m"], 0.0)


class DoorScanValidationTest(unittest.TestCase):
    def test_real_door_change_is_detected(self):
        opened = np.full(40, 4.0, dtype=np.float32)
        closed = opened.copy()
        closed[15:25] = 0.8
        self.assertTrue(MODULE.scan_door_changed(opened, closed))

    def test_static_room_candidate_is_rejected(self):
        opened = np.full(40, 2.0, dtype=np.float32)
        closed = opened.copy()
        closed[0] += 0.3
        self.assertFalse(MODULE.scan_door_changed(opened, closed))


class ElevatorHallCacheTest(unittest.TestCase):
    def test_map_version_change_invalidates_cached_hall_before_replanning(self):
        planner = MODULE.ExplorationPlanner.__new__(MODULE.ExplorationPlanner)
        planner.current_floor = 1
        planner.active_elevator_id = "main"
        planner.current_map_version = 12
        planner.accepted_map_load_identity = (1, 2, 3)
        key = (1, "main")
        planner.elevator_hall_bindings = {
            key: {
                "hall": (2.0, 3.0, 0.0),
                "map_version": 11,
                "map_load_identity": (1, 2, 3),
            }
        }

        result = planner._cached_hall_candidate()

        self.assertIsNone(result)
        self.assertNotIn(key, planner.elevator_hall_bindings)

    def test_matching_map_version_still_requires_reachability_validation(self):
        planner = MODULE.ExplorationPlanner.__new__(MODULE.ExplorationPlanner)
        planner.current_floor = 1
        planner.active_elevator_id = "main"
        planner.current_map_version = 12
        planner.accepted_map_load_identity = (1, 2, 3)
        planner.elevator_hall_approach_m = 0.8
        planner.current_pose = SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0)
        )
        planner._world_to_map = Mock(return_value=(20, 30))
        planner._is_free = Mock(return_value=True)
        planner._check_path = Mock(return_value="reachable")
        planner.elevator_hall_bindings = {
            (1, "main"): {
                "hall": (2.0, 3.0, 0.0),
                "map_version": 12,
                "map_load_identity": (1, 2, 3),
            }
        }

        self.assertEqual(planner._cached_hall_candidate(), (2.0, 3.0, 0.0))
        planner._check_path.assert_called_once()


class SweptFootprintTest(unittest.TestCase):
    @staticmethod
    def scan_with_point(x, y):
        count = 721
        angle_min = -math.pi
        increment = 2.0 * math.pi / float(count - 1)
        ranges = np.full(count, float("inf"), dtype=np.float64)
        angle = math.atan2(y, x)
        index = int(round((angle - angle_min) / increment))
        ranges[index] = math.hypot(x, y)
        return ranges, angle_min, increment

    def obstacle(self, x, y, direction=1.0):
        ranges, angle_min, increment = self.scan_with_point(x, y)
        return MODULE.swept_footprint_obstacle(
            ranges, angle_min, increment, 0.05, 10.0,
            direction, 1.2, (-0.35, 0.30, -0.15, 0.15), 0.08,
        )

    def test_forward_and_side_margin_obstacles_are_detected(self):
        self.assertIsNotNone(self.obstacle(0.8, 0.0))
        self.assertIsNotNone(self.obstacle(0.8, 0.20))

    def test_point_outside_swept_width_is_ignored(self):
        self.assertIsNone(self.obstacle(0.8, 0.35))

    def test_direction_selects_forward_or_backward_sweep(self):
        self.assertIsNone(self.obstacle(-0.8, 0.0, direction=1.0))
        self.assertIsNotNone(self.obstacle(-0.8, 0.0, direction=-1.0))


class TransitStateMachineTest(unittest.TestCase):
    class Future:
        def __init__(self, value=None, done=True):
            self.value = value
            self.done_value = done
            self.result_calls = 0

        def done(self):
            return self.done_value

        def result(self):
            self.result_calls += 1
            return self.value

    def test_service_timeout_invalidates_late_response_epoch(self):
        planner = MODULE.ExplorationPlanner.__new__(MODULE.ExplorationPlanner)
        old = self.Future(value=SimpleNamespace(accepted=True), done=False)
        planner._service_future = old
        planner._service_kind = "door"
        planner._service_generation = 4
        planner._service_future_generation = 4
        planner.floor_change_stage_deadline = MODULE.rospy.Time.from_sec(10.0)

        status, response = planner._poll_service(
            MODULE.rospy.Time.from_sec(10.1), "door"
        )

        self.assertEqual(status, "timeout")
        self.assertIsNone(response)
        self.assertIsNone(planner._service_future)
        self.assertEqual(planner._service_generation, 5)
        self.assertEqual(old.result_calls, 0)

    def test_hall_candidate_index_advances_exactly_once(self):
        planner = MODULE.ExplorationPlanner.__new__(MODULE.ExplorationPlanner)
        planner.current_pose = SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0)
        )
        planner.elevator_halls = [(1.0, 0.0, 0.0), (3.0, 0.0, 0.0)]
        planner.elevator_hall_index = 0
        planner.elevator_hall_approach_m = 0.8
        planner.elevator_hall_navigation_max_s = 180.0
        planner.elevator_hall_nominal_speed_mps = 0.25
        planner.elevator_car_target_m = 1.4
        planner._world_to_map = lambda _x, _y: (1, 1)
        planner._is_free = lambda _x, _y: True
        checks = iter(["unreachable", "reachable"])
        planner._check_path = lambda *_args: next(checks)
        planner.last_checked_path_metrics = {"path_length": 3.0}
        planner._send_goal = Mock(return_value=True)
        planner._set_floor_change_phase = Mock()

        with patch.object(
                MODULE.rospy.Time, "now",
                return_value=MODULE.rospy.Time.from_sec(1.0)):
            self.assertTrue(planner._pick_elevator_hall_and_send())
        self.assertEqual(planner.elevator_hall_index, 2)
        planner._send_goal.assert_called_once()

    def test_wait_stable_requires_epoch_two_versions_and_full_hold(self):
        planner = MODULE.ExplorationPlanner.__new__(MODULE.ExplorationPlanner)
        planner.floor_change_deadline = MODULE.rospy.Time.from_sec(1000.0)
        planner.floor_change_step = "WAIT_STABLE"
        planner.floor_change_target = 2
        planner.current_floor = 2
        planner.map_epoch = 8
        planner.floor_change_expected_epoch = 8
        planner.floor_change_start_target_version = 4
        planner.floor_min_new_map_versions = 2
        planner.current_map_version = 6
        planner.mapping_ready = True
        planner.mapping_stable = True
        planner.mapping_transitioning = False
        planner.nav_ready = True
        planner.nav_transitioning = False
        planner.nav_floor = 2
        planner.nav_map_epoch = 8
        planner.nav_map_version = 6
        planner.accepted_map_context = (2, 8, 6)
        planner.floor_change_stable_since = MODULE.rospy.Time(0)
        planner.floor_map_stable_time_s = 15.0
        planner._floor_change_done = Mock()

        planner._advance_floor_change(MODULE.rospy.Time.from_sec(100.0))
        planner._advance_floor_change(MODULE.rospy.Time.from_sec(114.9))
        planner._floor_change_done.assert_not_called()
        planner._advance_floor_change(MODULE.rospy.Time.from_sec(115.1))
        planner._floor_change_done.assert_called_once()

    def test_rejected_elevator_service_has_fixed_failure_code(self):
        planner = MODULE.ExplorationPlanner.__new__(MODULE.ExplorationPlanner)
        planner.floor_change_deadline = MODULE.rospy.Time.from_sec(1000.0)
        planner.floor_change_step = "CALL_TARGET_WAIT"
        planner._service_outcome = Mock(return_value=(
            "rejected", SimpleNamespace(message="not served")
        ))
        planner._floor_change_fail = Mock()

        planner._advance_floor_change(MODULE.rospy.Time.from_sec(10.0))

        planner._floor_change_fail.assert_called_once_with(
            "SERVICE_REJECTED", "call target floor: not served"
        )

    def test_wait_stable_rejects_navigation_from_previous_epoch(self):
        planner = MODULE.ExplorationPlanner.__new__(MODULE.ExplorationPlanner)
        planner.floor_change_deadline = MODULE.rospy.Time.from_sec(1000.0)
        planner.floor_change_step = "WAIT_STABLE"
        planner.floor_change_target = planner.current_floor = 1
        planner.map_epoch = planner.floor_change_expected_epoch = 4
        planner.floor_change_start_target_version = 0
        planner.floor_min_new_map_versions = 2
        planner.current_map_version = 2
        planner.mapping_ready = planner.mapping_stable = planner.nav_ready = True
        planner.mapping_transitioning = planner.nav_transitioning = False
        planner.accepted_map_context = (1, 4, 2)
        planner.nav_floor, planner.nav_map_epoch, planner.nav_map_version = (1, 3, 2)
        planner.floor_change_stable_since = MODULE.rospy.Time(0)
        planner.floor_map_stable_time_s = 0.0
        planner._floor_change_done = Mock()

        planner._advance_floor_change(MODULE.rospy.Time.from_sec(10.0))

        planner._floor_change_done.assert_not_called()
        self.assertEqual(planner.floor_change_stable_since, MODULE.rospy.Time(0))

    def test_wait_stable_survives_interleaved_versions_in_same_epoch(self):
        planner = MODULE.ExplorationPlanner.__new__(MODULE.ExplorationPlanner)
        planner.floor_change_deadline = MODULE.rospy.Time.from_sec(1000.0)
        planner.floor_change_step = "WAIT_STABLE"
        planner.floor_change_target = planner.current_floor = 1
        planner.map_epoch = planner.floor_change_expected_epoch = 4
        planner.floor_change_start_target_version = 4
        planner.floor_min_new_map_versions = 2
        planner.current_map_version = 7
        planner.mapping_ready = planner.mapping_stable = planner.nav_ready = True
        planner.mapping_transitioning = planner.nav_transitioning = False
        planner.accepted_map_context = (1, 4, 6)
        planner.nav_floor, planner.nav_map_epoch, planner.nav_map_version = (1, 4, 8)
        planner.floor_change_stable_since = MODULE.rospy.Time(0)
        planner.floor_map_stable_time_s = 15.0
        planner._floor_change_done = Mock()

        planner._advance_floor_change(MODULE.rospy.Time.from_sec(100.0))
        self.assertEqual(
            planner.floor_change_stable_since, MODULE.rospy.Time.from_sec(100.0)
        )
        planner.current_map_version = 9
        planner.accepted_map_context = (1, 4, 10)
        planner.nav_map_version = 11
        planner._advance_floor_change(MODULE.rospy.Time.from_sec(115.1))

        planner._floor_change_done.assert_called_once()

    def test_restore_floor_runtime_preserves_frontier_retry_count(self):
        planner = MODULE.ExplorationPlanner.__new__(MODULE.ExplorationPlanner)
        planner.current_floor = planner.floor_change_target = 0
        planner.visited_floors = {0, 1}
        planner.floor_change_final_target = 0
        planner.floor_change_external = False
        planner.floor_change_exit_to_hall = True
        planner.floor_runtime = {
            0: {
                "retry_count": 3,
                "coverage_debt": {(8, 9, "unreachable")},
            }
        }
        planner.coverage_debt_by_floor = {}
        planner.failed_goals = []
        planner.trap_blacklist = {}
        planner.backoff_until = MODULE.rospy.Time(0)
        planner.no_reachable_frontier_cycles = 1
        planner.floor_no_frontier_since = MODULE.rospy.Time(0)
        planner.elevator_halls = []
        planner.elevator_hall_found = None
        planner.floor_change_gave_up_count = 2
        planner.last_recovery_goal_id = ""
        planner.nav_active_goal_id = ""
        planner.navigation_goal_sent_at = MODULE.rospy.Time(0)
        planner.blacklist_pub = None
        planner.map_info = None
        planner._record_floor_change_result = Mock()
        planner._set_state = Mock()

        with patch.object(MODULE.rospy.Time, "now", return_value=MODULE.rospy.Time.from_sec(9.0)):
            planner._floor_change_done()

        self.assertEqual(planner.retry_count, 3)
        self.assertEqual(
            planner.coverage_debt_by_floor[0], {(8, 9, "unreachable")}
        )

    def test_safety_stop_has_deterministic_canceled_code(self):
        planner = MODULE.ExplorationPlanner.__new__(MODULE.ExplorationPlanner)
        planner.floor_change_step = "ENTER"
        planner.safety_stop_active = True

        healthy, code, detail = planner._transit_phase_health(
            MODULE.rospy.Time.from_sec(5.0)
        )

        self.assertFalse(healthy)
        self.assertEqual(code, "CANCELED")
        self.assertIn("safety", detail)

    def test_floor_map_reset_clears_only_that_floor_coordinates(self):
        planner = MODULE.ExplorationPlanner.__new__(MODULE.ExplorationPlanner)
        planner.current_floor = 1
        planner.floor_runtime = {0: {"failed_goals": ["keep"]}, 1: {}}
        planner.coverage_debt_by_floor = {0: {"keep"}, 1: {"drop"}}
        planner.completed_floors = {0, 1}
        planner.elevator_hall_bindings = {
            (0, "low"): {"hall": (0.0, 0.0, 0.0)},
            (1, "high"): {"hall": (1.0, 1.0, 0.0)},
        }
        planner.failed_goals = [(1.0, 1.0, MODULE.rospy.Time(0))]
        planner.trap_blacklist = {(1, 1): 0}
        planner.retry_count = 2
        planner.backoff_until = MODULE.rospy.Time.from_sec(9.0)
        planner.no_reachable_frontier_cycles = 4
        planner.floor_no_frontier_since = MODULE.rospy.Time.from_sec(2.0)
        planner.current_goal = (1.0, 1.0)
        planner.waiting_for_result = True
        planner.last_recovery_goal_id = "goal"
        planner.nav_active_goal_id = "goal"
        planner.navigation_goal_sent_at = MODULE.rospy.Time.from_sec(1.0)
        planner.blacklist_pub = None
        planner.map_info = None

        planner._clear_floor_runtime_for_map_reset(1)

        self.assertIn(0, planner.floor_runtime)
        self.assertNotIn(1, planner.floor_runtime)
        self.assertEqual(planner.completed_floors, {0})
        self.assertIn((0, "low"), planner.elevator_hall_bindings)
        self.assertNotIn((1, "high"), planner.elevator_hall_bindings)
        self.assertEqual(planner.failed_goals, [])
        self.assertEqual(planner.trap_blacklist, {})

    def test_control_zero_requires_fresh_echo(self):
        planner = MODULE.ExplorationPlanner.__new__(MODULE.ExplorationPlanner)
        planner.last_sent_command = MODULE.Twist()
        planner.last_sent_command_time = MODULE.rospy.Time.from_sec(9.8)
        self.assertTrue(planner._control_output_is_zero(
            MODULE.rospy.Time.from_sec(10.0)
        ))
        planner.last_sent_command.linear.x = 0.01
        self.assertFalse(planner._control_output_is_zero(
            MODULE.rospy.Time.from_sec(10.0)
        ))


class ActiveMapContractTest(unittest.TestCase):
    @staticmethod
    def planner():
        planner = MODULE.ExplorationPlanner.__new__(MODULE.ExplorationPlanner)
        planner.map_frame = "map"
        planner.current_floor = 1
        planner.map_epoch = 7
        planner.current_map_version = 3
        planner.accepted_map_context = (1, 7, 3)
        planner.pending_active_map = None
        planner._try_accept_pending_active_map = Mock(return_value=False)
        return planner

    @staticmethod
    def envelope(floor, epoch, version):
        stamp = MODULE.rospy.Time.from_sec(1.0)
        grid = SimpleNamespace(
            header=SimpleNamespace(frame_id="map", stamp=stamp),
            info=SimpleNamespace(width=2, height=2, resolution=0.1),
            data=[0, 0, 0, 0],
        )
        return SimpleNamespace(
            header=SimpleNamespace(frame_id="map", stamp=stamp),
            floor_id=floor,
            map_epoch=epoch,
            map_version=version,
            occupancy_grid=grid,
        )

    def test_wrong_floor_and_old_epoch_envelopes_are_rejected(self):
        planner = self.planner()
        with patch.object(MODULE.rospy, "logwarn_throttle"):
            planner.active_map_callback(self.envelope(0, 7, 3))
            planner.active_map_callback(self.envelope(1, 6, 99))
        self.assertIsNone(planner.pending_active_map)
        planner._try_accept_pending_active_map.assert_not_called()

    def test_future_epoch_is_buffered_until_status_matches(self):
        planner = self.planner()
        envelope = self.envelope(2, 8, 1)
        planner.active_map_callback(envelope)
        self.assertIs(planner.pending_active_map, envelope)
        planner._try_accept_pending_active_map.assert_called_once()

    def test_future_pending_map_waits_until_status_catches_up(self):
        planner = self.planner()
        planner._try_accept_pending_active_map = (
            MODULE.ExplorationPlanner._try_accept_pending_active_map.__get__(
                planner, MODULE.ExplorationPlanner
            )
        )
        planner.pending_active_map = self.envelope(1, 7, 4)
        planner.mapping_transitioning = False
        planner.mapping_ready = planner.mapping_stable = True
        planner._apply_occupancy_grid = Mock()
        self.assertFalse(planner._try_accept_pending_active_map())
        planner.current_map_version = 4
        self.assertTrue(planner._try_accept_pending_active_map())
        planner._apply_occupancy_grid.assert_called_once()
        self.assertIsNone(planner.pending_active_map)

    def test_delayed_same_epoch_envelope_is_not_discarded(self):
        planner = self.planner()
        planner.current_map_version = 5
        planner.accepted_map_context = (1, 7, 3)

        envelope = self.envelope(1, 7, 4)
        planner.active_map_callback(envelope)

        self.assertIs(planner.pending_active_map, envelope)
        planner._try_accept_pending_active_map.assert_called_once()

    def test_committed_delayed_snapshot_is_accepted_after_newer_status(self):
        planner = self.planner()
        planner._try_accept_pending_active_map = (
            MODULE.ExplorationPlanner._try_accept_pending_active_map.__get__(
                planner, MODULE.ExplorationPlanner
            )
        )
        planner.current_map_version = 5
        planner.pending_active_map = self.envelope(1, 7, 4)
        planner.mapping_transitioning = False
        planner.mapping_ready = planner.mapping_stable = True
        planner._apply_occupancy_grid = Mock()

        self.assertTrue(planner._try_accept_pending_active_map())
        planner._apply_occupancy_grid.assert_called_once()

    def test_regressive_envelope_behind_accepted_snapshot_is_rejected(self):
        planner = self.planner()
        planner.current_map_version = 5
        planner.accepted_map_context = (1, 7, 4)

        with patch.object(MODULE.rospy, "logwarn_throttle"):
            planner.active_map_callback(self.envelope(1, 7, 3))

        self.assertIsNone(planner.pending_active_map)
        planner._try_accept_pending_active_map.assert_not_called()

    def test_regressive_mapping_status_is_ignored(self):
        planner = self.planner()
        planner.last_mapping_status_time = MODULE.rospy.Time.from_sec(2.0)
        planner.mapping_ready = planner.mapping_stable = True
        planner.mapping_lost = planner.mapping_transitioning = False
        stale = SimpleNamespace(
            current_floor=1,
            map_epoch=6,
            ready=False,
            stable=False,
            lost=True,
            transitioning=True,
            floor_maps=[SimpleNamespace(floor_id=1, map_version=99)],
        )

        with patch.object(MODULE.rospy, "logwarn_throttle"):
            planner.mapping_status_callback(stale)

        self.assertEqual((planner.current_floor, planner.map_epoch), (1, 7))
        self.assertTrue(planner.mapping_ready)
        self.assertFalse(planner.mapping_lost)


class PlanningContextHealthTest(unittest.TestCase):
    @staticmethod
    def planner():
        planner = MODULE.ExplorationPlanner.__new__(MODULE.ExplorationPlanner)
        planner.current_map = object()
        planner.current_pose = object()
        planner.input_timeout = 2.0
        recent = MODULE.rospy.Time.from_sec(9.5)
        planner.last_map_time = recent
        planner.last_pose_time = recent
        planner.last_mapping_status_time = recent
        planner.last_nav_health_time = recent
        planner.mapping_lost = False
        planner.mapping_transitioning = False
        planner.mapping_ready = True
        planner.mapping_stable = True
        planner.multifloor_enabled = True
        planner.current_floor = 0
        planner.map_epoch = 1
        planner.current_map_version = 12
        planner.accepted_map_context = (0, 1, 10)
        planner.nav_ready = True
        planner.nav_transitioning = False
        planner.nav_floor = 0
        planner.nav_map_epoch = 1
        planner.nav_map_version = 11
        return planner

    def test_same_epoch_committed_versions_may_lag_latest_mapping(self):
        healthy, reason = self.planner()._inputs_health(
            MODULE.rospy.Time.from_sec(10.0)
        )

        self.assertTrue(healthy)
        self.assertEqual(reason, "healthy")

    def test_active_map_from_wrong_epoch_is_rejected(self):
        planner = self.planner()
        planner.accepted_map_context = (0, 0, 99)

        healthy, reason = planner._inputs_health(
            MODULE.rospy.Time.from_sec(10.0)
        )

        self.assertFalse(healthy)
        self.assertEqual(reason, "active_map_context_mismatch")

    def test_navigation_snapshot_cannot_lead_mapping_status(self):
        planner = self.planner()
        planner.nav_map_version = 13

        healthy, reason = planner._inputs_health(
            MODULE.rospy.Time.from_sec(10.0)
        )

        self.assertFalse(healthy)
        self.assertEqual(reason, "navigation_not_ready")

    def test_zero_map_versions_are_never_committed(self):
        self.assertFalse(MODULE.map_context_is_committed(
            (0, 1, 1), (0, 1, 0)
        ))
        self.assertFalse(MODULE.map_context_is_committed(
            (0, 1, 0), (0, 1, 0)
        ))

    def test_regressive_navigation_health_does_not_replace_committed_state(self):
        planner = self.planner()
        planner.last_nav_health_time = MODULE.rospy.Time.from_sec(9.0)
        planner.nav_map_version = 11
        message = SimpleNamespace(
            current_floor=0,
            map_epoch=1,
            map_version=10,
            ready=False,
            has_active_goal=False,
            stuck=False,
            failure_code="CONTROL_FAILED",
            failure_detail="late",
            active_goal_id="",
            transitioning=True,
        )

        with patch.object(MODULE.rospy, "logwarn_throttle"):
            planner.nav_health_callback(message)

        self.assertTrue(planner.nav_ready)
        self.assertEqual(planner.nav_map_version, 11)
        self.assertFalse(planner.nav_transitioning)


if __name__ == "__main__":
    unittest.main()
