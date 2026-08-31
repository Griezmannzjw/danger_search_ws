#!/usr/bin/env python3
"""多楼层探索、电梯门主动定位与换层状态机回归测试。"""

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
    planner.shaft_max_area_m2 = 12.0
    planner.shaft_min_side_m = 1.8
    planner.shaft_max_side_m = 3.6
    planner.shaft_wall_support_min = 0.70
    planner.door_gap_min_width_m = 0.9
    planner.door_gap_max_width_m = 1.8
    planner.door_center_tolerance_fraction = 0.25
    planner.elevator_hall_min_score = 0.75
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
            500, shaft=(-8.0, -5.5, -4.0, -1.0), door=(-3.1, -2.1)
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

    def test_large_room_and_two_opening_shaft_are_rejected(self):
        large = build_shaft_map(
            400, shaft=(1.0, 6.0, 1.0, 6.0), door=(3.0, 4.2)
        )
        self.assertEqual(make_planner(large)._detect_elevator_halls(), [])

        grid = build_shaft_map(400)
        # Add a second, door-sized opening to the east wall.
        half = 20.0
        grid[int((2.0 + half) / 0.1):int((3.0 + half) / 0.1) + 1,
             int((3.8 + half) / 0.1):int((4.1 + half) / 0.1) + 1] = 0
        self.assertEqual(make_planner(grid)._detect_elevator_halls(), [])

    def test_passive_candidate_requires_three_versions_and_two_seconds(self):
        planner = make_planner(build_shaft_map(400))
        planner.current_floor = 0
        planner.map_epoch = 3
        planner.accepted_map_load_identity = (1, 2, 3)
        planner.elevator_hall_tracks = {}
        planner.elevator_hall_last_observed_version = None
        planner.elevator_hall_min_versions = 3
        planner.elevator_hall_min_duration_s = 2.0
        planner.elevator_hall_min_score = 0.75
        for version, seconds in ((4, 0.0), (5, 1.0)):
            planner.current_map_version = version
            planner.accepted_map_context = (0, 3, version)
            planner._observe_elevator_halls(MODULE.rospy.Time.from_sec(seconds))
        self.assertEqual(
            planner._confirmed_elevator_halls(MODULE.rospy.Time.from_sec(1.0)), []
        )
        planner.current_map_version = 6
        planner.accepted_map_context = (0, 3, 6)
        planner._observe_elevator_halls(MODULE.rospy.Time.from_sec(2.0))
        self.assertTrue(
            planner._confirmed_elevator_halls(MODULE.rospy.Time.from_sec(2.0))
        )


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
        self.assertEqual(config["elevator_crossing_timeout_s"], 40.0)
        self.assertEqual(config["floor_map_stable_time_s"], 15.0)
        self.assertEqual(config["elevator_crossing_speed_mps"], 0.40)
        self.assertEqual(config["shaft_max_area_m2"], 12.0)
        self.assertEqual(config["door_gap_min_width_m"], 0.9)
        self.assertEqual(config["door_gap_max_width_m"], 1.8)
        self.assertEqual(config["active_map_topic"], "/mapping/active_map")
        self.assertGreater(config["elevator_footprint_margin_m"], 0.0)
        self.assertFalse(config["fixed_elevator_hall_enabled"])
        self.assertEqual(config["fixed_elevator_hall_x"], -2.40)
        self.assertEqual(config["fixed_elevator_hall_y"], -1.65)
        self.assertAlmostEqual(
            config["fixed_elevator_hall_into_yaw"], -math.pi / 2.0, places=6
        )


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


class ActiveDoorLocalizationTest(unittest.TestCase):
    @staticmethod
    def scans_for_door(x=2.0, half_width=0.65, count=721, noise=0.0):
        angle_min = -math.pi
        increment = 2.0 * math.pi / float(count - 1)
        angles = angle_min + np.arange(count) * increment
        closed = np.full(count, np.nan, dtype=np.float32)
        if x > 0.0:
            valid = ((np.cos(angles) > 0.0)
                     & (np.abs(x * np.tan(angles)) <= half_width))
        else:
            valid = ((np.cos(angles) < 0.0)
                     & (np.abs(x * np.tan(angles)) <= half_width))
        closed[valid] = x / np.cos(angles[valid])
        opened = np.full(count, np.nan, dtype=np.float32)
        opened_stack = np.tile(opened, (5, 1))
        closed_stack = np.tile(closed, (5, 1))
        if noise:
            random = np.random.RandomState(7)
            closed_stack[:, valid] += random.normal(
                0.0, noise, size=(5, int(np.count_nonzero(valid)))
            )
        return opened_stack, closed_stack, angle_min, increment

    def localize(self, opened, closed, angle_min, increment):
        return MODULE.localize_actuated_door(
            opened, closed, angle_min, increment, 0.05, 10.0,
            (0.0, 0.0, 0.0), (0.0, 0.0),
        )

    def test_infinite_open_ranges_recover_center_and_inward_yaw(self):
        opened, closed, angle_min, increment = self.scans_for_door(noise=0.01)
        candidate = self.localize(opened, closed, angle_min, increment)
        self.assertIsNotNone(candidate)
        self.assertAlmostEqual(candidate.x, 2.0, delta=0.04)
        self.assertAlmostEqual(candidate.y, 0.0, delta=0.04)
        self.assertLess(abs(candidate.into_yaw), math.radians(2.0))
        self.assertTrue(candidate.validated)
        self.assertEqual(candidate.source, "door_motion")

    def test_sparse_projected_scan_bridges_five_no_return_bins(self):
        opened, closed, angle_min, increment = self.scans_for_door()
        finite = np.flatnonzero(np.isfinite(closed[0]))
        for offset in (25, 50):
            closed[:, finite[offset:offset + 5]] = np.nan
        candidate = self.localize(opened, closed, angle_min, increment)
        self.assertIsNotNone(candidate)
        self.assertAlmostEqual(candidate.x, 2.0, delta=0.04)
        self.assertAlmostEqual(candidate.y, 0.0, delta=0.06)

    def test_scan_seam_cluster_is_merged(self):
        opened, closed, angle_min, increment = self.scans_for_door(x=-2.0)
        candidate = self.localize(opened, closed, angle_min, increment)
        self.assertIsNotNone(candidate)
        yaw_error = abs(math.atan2(
            math.sin(candidate.into_yaw - math.pi),
            math.cos(candidate.into_yaw - math.pi),
        ))
        self.assertLess(yaw_error, math.radians(2.0))

    def test_scan_points_are_transformed_into_map_frame(self):
        opened, closed, angle_min, increment = self.scans_for_door(x=2.0)
        candidate = MODULE.localize_actuated_door(
            opened, closed, angle_min, increment, 0.05, 10.0,
            (1.0, 2.0, math.pi / 2.0), (1.0, 2.0),
        )
        self.assertIsNotNone(candidate)
        self.assertAlmostEqual(candidate.x, 1.0, delta=0.04)
        self.assertAlmostEqual(candidate.y, 4.0, delta=0.04)
        self.assertAlmostEqual(candidate.into_yaw, math.pi / 2.0, delta=0.04)

    def test_no_motion_and_ambiguous_two_doors_are_rejected(self):
        opened, front, angle_min, increment = self.scans_for_door(x=2.0)
        self.assertIsNone(self.localize(front, front, angle_min, increment))
        _opened_back, back, _angle_min, _increment = self.scans_for_door(x=-2.0)
        combined = np.where(np.isfinite(front), front, back)
        self.assertIsNone(self.localize(opened, combined, angle_min, increment))

    def test_robot_motion_contract(self):
        planner = MODULE.ExplorationPlanner.__new__(MODULE.ExplorationPlanner)
        planner.initial_hall_discovery_pose = (0.0, 0.0, 0.0)
        planner.initial_hall_discovery_max_translation_m = 0.03
        planner.initial_hall_discovery_max_yaw_deg = 1.0
        planner.current_pose = SimpleNamespace(
            position=SimpleNamespace(x=0.031, y=0.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        self.assertTrue(planner._initial_discovery_robot_moved())

    def test_close_is_submitted_once_and_restore_requests_open(self):
        planner = MODULE.ExplorationPlanner.__new__(MODULE.ExplorationPlanner)
        planner.initial_hall_discovery_active = True
        planner.initial_hall_discovery_step = "CLOSE_START"
        planner.initial_hall_discovery_started = MODULE.rospy.Time.from_sec(1.0)
        planner.initial_hall_discovery_timeout_s = 60.0
        planner.current_floor = 0
        planner._initial_discovery_robot_moved = Mock(return_value=False)
        planner._submit_service = Mock(return_value=True)
        planner._set_initial_discovery_step = Mock()
        planner._door_request = Mock(return_value=object())

        planner._advance_initial_hall_discovery(
            MODULE.rospy.Time.from_sec(2.0)
        )
        self.assertEqual(planner._submit_service.call_count, 1)
        kind, close_callback = planner._submit_service.call_args.args
        self.assertEqual(kind, "discovery_close")
        close_callback()
        planner._door_request.assert_called_once_with(0, False)

        planner.initial_hall_discovery_step = "RESTORE_OPEN_START"
        planner._submit_service.reset_mock()
        planner._door_request.reset_mock()
        planner._advance_initial_hall_discovery(
            MODULE.rospy.Time.from_sec(3.0)
        )
        kind, open_callback = planner._submit_service.call_args.args
        self.assertEqual(kind, "discovery_restore_open")
        open_callback()
        planner._door_request.assert_called_once_with(0, True)


class FixedHallOverrideTest(unittest.TestCase):
    def test_override_is_restricted_to_simulation_truth(self):
        enabled, values = MODULE.validate_fixed_elevator_hall_mode(
            True, False, "simulation_truth", -2.4, -1.65, -math.pi / 2.0
        )
        self.assertTrue(enabled)
        self.assertEqual(values, (-2.4, -1.65, -math.pi / 2.0))

        for competition_mode, profile in (
                (True, "simulation_truth"), (False, "formal")):
            with self.assertRaises(ValueError):
                MODULE.validate_fixed_elevator_hall_mode(
                    True, competition_mode, profile,
                    -2.4, -1.65, -math.pi / 2.0,
                )

    def test_disabled_override_does_not_restrict_other_profiles(self):
        enabled, _values = MODULE.validate_fixed_elevator_hall_mode(
            False, True, "formal", -2.4, -1.65, -math.pi / 2.0
        )
        self.assertFalse(enabled)

    def test_override_skips_initial_door_discovery(self):
        planner = MODULE.ExplorationPlanner.__new__(MODULE.ExplorationPlanner)
        planner.multifloor_enabled = True
        planner.initial_hall_discovery_enabled = True
        planner.fixed_elevator_hall_enabled = True
        planner.current_floor = 0
        planner.elevator_door_initial_open = {0: True}

        planner._begin_initial_hall_discovery()

        self.assertFalse(planner.initial_hall_discovery_active)
        self.assertEqual(planner.initial_hall_discovery_step, "FIXED_OVERRIDE")

    def test_override_candidate_is_single_and_unvalidated(self):
        planner = MODULE.ExplorationPlanner.__new__(MODULE.ExplorationPlanner)
        planner.fixed_elevator_hall_enabled = True
        planner.fixed_elevator_hall_x = -2.40
        planner.fixed_elevator_hall_y = -1.65
        planner.fixed_elevator_hall_into_yaw = -math.pi / 2.0

        candidate = planner._fixed_elevator_hall_candidate()

        self.assertEqual(candidate.hall(), (-2.40, -1.65, -math.pi / 2.0))
        self.assertEqual(candidate.source, "fixed_test")
        self.assertEqual(candidate.score, 1.0)
        self.assertEqual(candidate.confidence, 1.0)
        self.assertFalse(candidate.validated)

    def test_override_excludes_cached_and_passive_candidates(self):
        planner = MODULE.ExplorationPlanner.__new__(MODULE.ExplorationPlanner)
        planner.fixed_elevator_hall_enabled = True
        planner.fixed_elevator_hall_x = -2.40
        planner.fixed_elevator_hall_y = -1.65
        planner.fixed_elevator_hall_into_yaw = -math.pi / 2.0
        planner._cached_hall_candidate = Mock()
        planner._observe_elevator_halls = Mock()
        planner._confirmed_elevator_halls = Mock()

        candidates = planner._hall_candidates_for_floor_change(
            MODULE.rospy.Time.from_sec(1.0)
        )

        self.assertEqual(len(candidates), 1)
        self.assertEqual(candidates[0].source, "fixed_test")
        planner._cached_hall_candidate.assert_not_called()
        planner._observe_elevator_halls.assert_not_called()
        planner._confirmed_elevator_halls.assert_not_called()

    def test_unreachable_override_reports_unreachable_hall(self):
        planner = MODULE.ExplorationPlanner.__new__(MODULE.ExplorationPlanner)
        planner.current_pose = SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0)
        )
        planner.elevator_halls = [MODULE.ElevatorHallCandidate(
            -2.40, -1.65, -math.pi / 2.0,
            score=1.0, source="fixed_test", confidence=1.0,
            validated=False,
        )]
        planner.elevator_hall_index = 0
        planner.elevator_hall_min_score = 0.75
        planner.elevator_hall_approach_m = 0.8
        planner._world_to_map = Mock(return_value=(1, 1))
        planner._is_free = Mock(return_value=True)
        planner._check_path = Mock(return_value="unreachable")
        planner._floor_change_fail = Mock()

        self.assertFalse(planner._pick_elevator_hall_and_send())
        planner._floor_change_fail.assert_called_once_with(
            "UNREACHABLE_HALL", "all elevator hall candidates are unreachable"
        )


class ElevatorHallCacheTest(unittest.TestCase):
    @staticmethod
    def planner_with_binding(epoch=4, version=12, binding_epoch=4,
                             binding_version=10, load=(1, 2, 3)):
        planner = MODULE.ExplorationPlanner.__new__(MODULE.ExplorationPlanner)
        planner.current_floor = 1
        planner.map_epoch = epoch
        planner.active_elevator_id = "main"
        planner.current_map_version = version
        planner.accepted_map_load_identity = load
        planner.elevator_hall_approach_m = 0.8
        planner.current_pose = SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0)
        )
        planner._world_to_map = Mock(return_value=(20, 30))
        planner._is_free = Mock(return_value=True)
        planner._check_path = Mock(return_value="reachable")
        planner.last_checked_path_metrics = {"path_length": 4.0}
        key = (1, "main")
        planner.elevator_hall_bindings = {
            key: {
                "hall": (2.0, 3.0, 0.0),
                "floor": 1,
                "epoch": binding_epoch,
                "map_version": binding_version,
                "map_load_identity": load,
                "source": "door_motion",
                "confidence": 1.0,
                "score": 1.0,
                "validated": True,
            }
        }
        return planner, key

    def test_same_epoch_newer_map_version_keeps_cached_hall(self):
        planner, key = self.planner_with_binding()

        result = planner._cached_hall_candidate()

        self.assertIsNotNone(result)
        self.assertEqual(result.hall(), (2.0, 3.0, 0.0))
        self.assertTrue(result.validated)
        self.assertIn(key, planner.elevator_hall_bindings)

    def test_wrong_epoch_or_map_load_invalidates_binding(self):
        planner, key = self.planner_with_binding(binding_epoch=3)
        self.assertIsNone(planner._cached_hall_candidate())
        self.assertNotIn(key, planner.elevator_hall_bindings)

        planner, key = self.planner_with_binding()
        planner.accepted_map_load_identity = (9, 9, 9)
        self.assertIsNone(planner._cached_hall_candidate())
        self.assertNotIn(key, planner.elevator_hall_bindings)

    def test_cached_hall_still_requires_reachability_validation(self):
        planner, _key = self.planner_with_binding()

        self.assertEqual(
            planner._cached_hall_candidate().hall(), (2.0, 3.0, 0.0)
        )
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

    @staticmethod
    def to_hall_planner():
        planner = MODULE.ExplorationPlanner.__new__(MODULE.ExplorationPlanner)
        planner.floor_change_step = "TO_HALL"
        planner.safety_stop_active = False
        planner.current_pose = object()
        planner.input_timeout = 2.0
        recent = MODULE.rospy.Time.from_sec(9.5)
        planner.last_pose_time = recent
        planner.last_mapping_status_time = recent
        planner.last_map_time = recent
        planner.current_floor = 0
        planner.map_epoch = 1
        planner.current_map_version = 12
        planner.mapping_lost = False
        planner.mapping_transitioning = False
        planner.mapping_ready = True
        planner.mapping_stable = True
        planner.accepted_map_context = (0, 1, 12)
        return planner

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
        self.assertEqual(planner.elevator_hall_index, 1)
        planner._send_goal.assert_called_once()

    def test_higher_score_far_candidate_beats_lower_score_near_candidate(self):
        planner = MODULE.ExplorationPlanner.__new__(MODULE.ExplorationPlanner)
        planner.current_pose = SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0)
        )
        planner.elevator_halls = [
            MODULE.ElevatorHallCandidate(1.0, 0.0, 0.0, score=0.80),
            MODULE.ElevatorHallCandidate(4.0, 0.0, 0.0, score=0.95),
        ]
        planner.elevator_hall_index = 0
        planner.elevator_hall_min_score = 0.75
        planner.elevator_hall_approach_m = 0.8
        planner.elevator_hall_navigation_max_s = 180.0
        planner.elevator_hall_nominal_speed_mps = 0.25
        planner.elevator_car_target_m = 1.4
        planner._world_to_map = Mock(return_value=(1, 1))
        planner._is_free = Mock(return_value=True)
        planner._check_path = Mock(return_value="reachable")
        planner.last_checked_path_metrics = {"path_length": 2.0}
        planner._send_goal = Mock(return_value=True)
        planner._set_floor_change_phase = Mock()

        with patch.object(
                MODULE.rospy.Time, "now",
                return_value=MODULE.rospy.Time.from_sec(1.0)):
            self.assertTrue(planner._pick_elevator_hall_and_send())
        sent_x = planner._send_goal.call_args.args[0]
        self.assertGreater(sent_x, 3.0)

    def test_motion_validated_binding_skips_runtime_close_reopen(self):
        planner = MODULE.ExplorationPlanner.__new__(MODULE.ExplorationPlanner)
        planner.floor_change_deadline = MODULE.rospy.Time.from_sec(100.0)
        planner.floor_change_step = "OPEN_CURRENT_WAIT"
        planner.hall_validation_required = True
        planner.floor_change_hall_candidate = MODULE.ElevatorHallCandidate(
            2.0, 0.0, 0.0, score=1.0, source="door_motion",
            confidence=1.0, validated=True,
        )
        planner._service_outcome = Mock(return_value=("success", object()))
        planner._start_crossing = Mock()

        planner._advance_floor_change(MODULE.rospy.Time.from_sec(1.0))

        planner._start_crossing.assert_called_once_with(+1.0)

    def test_fixed_candidate_failed_door_validation_never_enters(self):
        planner = MODULE.ExplorationPlanner.__new__(MODULE.ExplorationPlanner)
        planner.floor_change_deadline = MODULE.rospy.Time.from_sec(100.0)
        planner.floor_change_step = "REOPEN_CURRENT_WAIT"
        planner.floor_change_hall_candidate = MODULE.ElevatorHallCandidate(
            -2.40, -1.65, -math.pi / 2.0,
            score=1.0, source="fixed_test", confidence=1.0,
            validated=False,
        )
        planner._hall_validation_passed = False
        planner._service_outcome = Mock(return_value=("success", object()))
        planner._discard_active_hall_binding = Mock()
        planner._retry_hall_or_fail = Mock()
        planner._start_crossing = Mock()

        planner._advance_floor_change(MODULE.rospy.Time.from_sec(1.0))

        planner._start_crossing.assert_not_called()
        planner._retry_hall_or_fail.assert_called_once_with(
            "NO_HALL", "door motion did not change the local scan"
        )

    def test_validated_fixed_candidate_keeps_diagnostic_source(self):
        planner = MODULE.ExplorationPlanner.__new__(MODULE.ExplorationPlanner)
        planner.floor_change_hall_point = (-2.40, -1.65, -math.pi / 2.0)
        planner.floor_change_hall_candidate = MODULE.ElevatorHallCandidate(
            -2.40, -1.65, -math.pi / 2.0,
            score=1.0, source="fixed_test", confidence=1.0,
            validated=False,
        )
        planner.floor_change_start_floor = 0
        planner.floor_change_start_epoch = 1
        planner._save_hall_binding = Mock()

        planner._remember_validated_hall()

        candidate = planner._save_hall_binding.call_args.args[0]
        self.assertTrue(candidate.validated)
        self.assertEqual(candidate.source, "fixed_test")

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

    def test_to_hall_accepts_fresh_committed_map_that_lags_status(self):
        planner = self.to_hall_planner()
        planner.accepted_map_context = (0, 1, 10)

        healthy, code, detail = planner._transit_phase_health(
            MODULE.rospy.Time.from_sec(10.0)
        )

        self.assertTrue(healthy)
        self.assertEqual(code, "")
        self.assertEqual(detail, "")

    def test_to_hall_tolerates_degraded_readiness_after_committed_start(self):
        planner = self.to_hall_planner()
        planner.mapping_ready = False
        planner.mapping_stable = False

        with patch.object(MODULE.rospy, "logwarn_throttle") as warning:
            healthy, code, detail = planner._transit_phase_health(
                MODULE.rospy.Time.from_sec(10.0)
            )

        self.assertTrue(healthy)
        self.assertEqual(code, "")
        self.assertEqual(detail, "")
        warning.assert_called_once()

    def test_to_hall_rejects_uncommitted_map_contexts(self):
        invalid_contexts = (
            (1, 1, 10),
            (0, 0, 10),
            (0, 2, 10),
            (0, 1, 0),
            (0, 1, 13),
        )
        for context in invalid_contexts:
            with self.subTest(context=context):
                planner = self.to_hall_planner()
                planner.accepted_map_context = context

                healthy, code, detail = planner._transit_phase_health(
                    MODULE.rospy.Time.from_sec(10.0)
                )

                self.assertFalse(healthy)
                self.assertEqual(code, "UNREACHABLE_HALL")
                self.assertIn("not committed", detail)
                self.assertIn("mapping=(0, 1, 12)", detail)

    def test_to_hall_rejects_stale_committed_map(self):
        planner = self.to_hall_planner()
        planner.accepted_map_context = (0, 1, 10)
        planner.last_map_time = MODULE.rospy.Time.from_sec(7.0)

        healthy, code, detail = planner._transit_phase_health(
            MODULE.rospy.Time.from_sec(10.0)
        )

        self.assertFalse(healthy)
        self.assertEqual(code, "UNREACHABLE_HALL")
        self.assertIn("stale", detail)

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


class CompletedFloorTransitionTest(unittest.TestCase):
    @staticmethod
    def planner():
        planner = MODULE.ExplorationPlanner.__new__(MODULE.ExplorationPlanner)
        planner.current_floor = 0
        planner.completed_floors = {0}
        planner.served_floors = {0, 1, 2}
        planner.multifloor_enabled = True
        planner.floor_change_active = False
        planner.floor_change_gave_up_count = 0
        planner.elevator_max_retries = 3
        planner.floor_change_retry_after = MODULE.rospy.Time.from_sec(20.0)
        planner.floor_transit_fatal = False
        planner.exploration_state = "EXPLORE_FLOOR"
        planner.state_reason = "floor_complete"
        planner._select_next_floor = Mock(return_value=1)
        planner._begin_floor_change = Mock(return_value=True)
        planner._complete_exploration = Mock()
        planner.state_calls = []

        def set_state(state, reason):
            planner.exploration_state = state
            planner.state_reason = reason
            planner.state_calls.append((state, reason))

        planner._set_state = set_state
        return planner

    def test_floor_completion_is_persisted_and_logged_once(self):
        planner = self.planner()
        planner.completed_floors = set()
        planner.coverage_debt_by_floor = {0: {(1, 2, "unreachable")}}
        planner.remaining_frontier_count = 4
        planner._save_current_floor_runtime = Mock()

        with patch.object(MODULE.rospy, "loginfo") as loginfo:
            first = planner._mark_current_floor_complete(
                "bounded_unreachable", "all_frontiers_unreachable_or_blacklisted"
            )
            second = planner._mark_current_floor_complete(
                "bounded_unreachable", "all_frontiers_unreachable_or_blacklisted"
            )

        self.assertTrue(first)
        self.assertFalse(second)
        self.assertEqual(planner.completed_floors, {0})
        planner._save_current_floor_runtime.assert_called_once()
        loginfo.assert_called_once()

    def test_completed_floor_bypasses_frontier_selection(self):
        planner = self.planner()
        planner.exploring = True
        planner.complete_published = False
        planner._inputs_health = Mock(return_value=(True, "healthy"))
        planner._continue_completed_floor = Mock()
        planner._select_goal = Mock()

        with patch.object(
                MODULE.rospy.Time, "now",
                return_value=MODULE.rospy.Time.from_sec(10.0)):
            planner.planner_loop(None)

        planner._continue_completed_floor.assert_called_once_with(
            MODULE.rospy.Time.from_sec(10.0)
        )
        planner._select_goal.assert_not_called()

    def test_retry_backoff_state_is_set_once(self):
        planner = self.planner()

        planner._continue_completed_floor(MODULE.rospy.Time.from_sec(10.0))
        planner._continue_completed_floor(MODULE.rospy.Time.from_sec(11.0))

        self.assertEqual(
            planner.state_calls,
            [("WAITING", "floor_transit_retry_backoff")],
        )
        planner._begin_floor_change.assert_not_called()

    def test_retry_starts_once_after_backoff(self):
        planner = self.planner()
        planner.floor_change_retry_after = MODULE.rospy.Time.from_sec(5.0)

        def begin(_floor):
            planner.floor_change_active = True
            return True

        planner._begin_floor_change.side_effect = begin

        planner._continue_completed_floor(MODULE.rospy.Time.from_sec(10.0))
        planner._continue_completed_floor(MODULE.rospy.Time.from_sec(10.1))

        planner._begin_floor_change.assert_called_once_with(1)

    def test_retry_exhaustion_is_terminal_and_idempotent(self):
        planner = self.planner()
        planner.floor_change_gave_up_count = 3

        planner._continue_completed_floor(MODULE.rospy.Time.from_sec(30.0))
        planner._continue_completed_floor(MODULE.rospy.Time.from_sec(31.0))

        self.assertEqual(
            planner.state_calls,
            [("FAILED", "floor_transit_unavailable")],
        )
        planner._begin_floor_change.assert_not_called()


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
