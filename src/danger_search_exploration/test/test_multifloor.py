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


if __name__ == "__main__":
    unittest.main()
