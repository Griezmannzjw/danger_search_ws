#!/usr/bin/env python3

import importlib.util
import math
import pathlib
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import yaml


SCRIPT = pathlib.Path(__file__).parents[1] / "scripts" / "exploration_planner.py"
SPEC = importlib.util.spec_from_file_location("exploration_planner", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def make_planner(grid, resolution=1.0, min_frontier_length=1.0, free_threshold=25):
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
    planner.free_threshold = free_threshold
    planner.connectivity_occupied_threshold = 65
    planner.connectivity_clearance_radius = 0.0
    return planner


class SimpleFrontierTest(unittest.TestCase):
    def test_relaxed_exploration_defaults(self):
        config_path = pathlib.Path(__file__).parents[1] / "config" / "default.yaml"
        with config_path.open(encoding="utf-8") as stream:
            config = yaml.safe_load(stream)

        self.assertEqual(config["free_threshold"], 40)
        self.assertAlmostEqual(config["connectivity_clearance_radius"], 0.30)
        self.assertAlmostEqual(config["min_frontier_length"], 0.40)
        self.assertAlmostEqual(config["goal_timeout"], 120.0)
        self.assertAlmostEqual(config["observation_min_distance"], 0.30)
        self.assertAlmostEqual(config["observation_target_distance"], 0.45)
        self.assertAlmostEqual(config["observation_max_distance"], 0.70)
        self.assertAlmostEqual(config["goal_clearance_margin"], 0.02)
        self.assertAlmostEqual(config["path_clearance_weight"], 0.15)
        self.assertAlmostEqual(config["failed_goal_cooldown"], 15.0)
        self.assertAlmostEqual(config["failed_goal_radius"], 0.50)
        self.assertAlmostEqual(config["success_goal_cooldown"], 45.0)
        self.assertAlmostEqual(config["success_goal_radius"], 0.50)
        self.assertEqual(config["success_goal_clear_revisions"], 2)
        self.assertFalse(config["elevator"]["enabled"])
        self.assertTrue(config["elevator"]["autonomous_when_floor_complete"])
        self.assertEqual(config["elevator"]["served_floors"], [0, 1])
        self.assertEqual(
            config["elevator"]["max_autonomous_transition_failures"], 1
        )
        self.assertAlmostEqual(config["elevator"]["portal_near_distance"], 0.40)
        self.assertAlmostEqual(config["elevator"]["cabin_distance"], 0.60)
        self.assertTrue(config["elevator"]["portal_traversal_enabled"])
        self.assertAlmostEqual(config["elevator"]["portal_speed"], 0.25)
        self.assertAlmostEqual(config["elevator"]["portal_duration"], 3.0)
        self.assertAlmostEqual(config["trap_blacklist_radius"], 0.70)
        self.assertAlmostEqual(config["trap_clearance_margin"], 0.08)
        self.assertEqual(config["blacklist_clear_revisions"], 2)

    def test_recovery_trigger_is_diagnostic_and_failed_event_blacklists(self):
        planner = make_planner(np.zeros((5, 5), dtype=np.int8))
        planner.map_frame = "map"
        planner.last_recovery_event_id = 0
        planner.last_recovery_stuck_pose = None
        remembered = []
        planner._remember_trap_region = lambda x, y: remembered.append((x, y))
        event = MODULE.RecoveryEvent()
        event.header.frame_id = "map"
        event.event_id = 7
        event.stuck_pose.position.x = 1.0
        event.stuck_pose.position.y = 2.0
        event.attempt = 1

        event.phase = MODULE.RecoveryEvent.PHASE_TRIGGERED
        planner.recovery_event_callback(event)
        self.assertEqual(remembered, [])

        event.phase = MODULE.RecoveryEvent.PHASE_FAILED
        planner.recovery_event_callback(event)
        self.assertEqual(remembered, [(1.0, 2.0)])
        planner.recovery_event_callback(event)
        self.assertEqual(remembered, [(1.0, 2.0)])

    def test_make_plan_path_may_leave_but_not_reenter_exploration_blacklist(self):
        planner = make_planner(np.zeros((3, 7), dtype=np.int8))
        planner.map_frame = "map"
        planner.make_plan_service = "/move_base/make_plan"
        planner.dependency_check_timeout = 0.1
        planner.plan_tolerance = 0.5
        planner.trap_blacklist = {(0, 1): 0, (1, 1): 0}

        def response(xs):
            return SimpleNamespace(plan=SimpleNamespace(poses=[
                SimpleNamespace(pose=SimpleNamespace(
                    position=SimpleNamespace(x=x, y=1.5)
                )) for x in xs
            ]))

        planner.make_plan_client = lambda *_args: response((0.5, 1.5, 2.5, 5.5))
        with patch.object(MODULE.rospy, "wait_for_service"), patch.object(
                MODULE.rospy.Time, "now", return_value=MODULE.rospy.Time.from_sec(1.0)):
            self.assertEqual(planner._check_path(0.5, 1.5, 5.5, 1.5), "reachable")

        planner.make_plan_client = lambda *_args: response((0.5, 2.5, 1.5, 5.5))
        with patch.object(MODULE.rospy, "wait_for_service"), patch.object(
                MODULE.rospy.Time, "now", return_value=MODULE.rospy.Time.from_sec(1.0)):
            self.assertEqual(planner._check_path(0.5, 1.5, 5.5, 1.5), "unreachable")

    def test_frontier_observation_goal_stays_inside_and_faces_unknown(self):
        grid = np.full((20, 30), 100, dtype=np.int8)
        grid[2:18, 2:20] = 0
        grid[2:18, 20:29] = -1
        planner = make_planner(
            grid, resolution=0.10, min_frontier_length=0.20
        )
        planner.current_pose = SimpleNamespace(
            position=SimpleNamespace(x=0.55, y=1.05)
        )
        planner.connectivity_clearance_radius = 0.20
        planner.goal_clearance_margin = 0.04
        planner.observation_min_distance = 0.40
        planner.observation_target_distance = 0.50
        planner.observation_max_distance = 0.60
        planner.trap_blacklist = {}
        planner.observation_goals_pub = None

        frontier = planner._frontier_mask()
        goals = planner._observation_goals(frontier, planner._reachable_free_mask())

        self.assertTrue(goals)
        goal = goals[0]
        frontier_x = 1.95
        self.assertGreaterEqual(frontier_x - goal["x"], 0.40 - 1e-6)
        self.assertLessEqual(frontier_x - goal["x"], 0.60 + 1e-6)
        self.assertAlmostEqual(goal["yaw"], 0.0, delta=0.25)

    def test_recovery_trap_is_long_lived_and_clearance_scoped(self):
        grid = np.zeros((21, 21), dtype=np.int8)
        grid[:, 10] = 100
        planner = make_planner(grid, resolution=0.10)
        planner.trap_blacklist = {}
        planner.trap_blacklist_radius = 1.0
        planner.trap_clearance_margin = 0.15
        planner.blacklist_clear_revisions = 3
        planner.connectivity_clearance_radius = 0.20
        planner.blacklist_pub = None

        planner._remember_trap_region(0.95, 1.05)

        self.assertTrue(planner.trap_blacklist)
        self.assertIn((9, 10), planner.trap_blacklist)
        planner.map_data[:, 10] = 0
        planner._update_blacklist_validity()
        planner._update_blacklist_validity()
        self.assertTrue(planner.trap_blacklist)
        planner._update_blacklist_validity()
        self.assertFalse(planner.trap_blacklist)

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

    def test_low_probability_observed_cell_can_be_frontier(self):
        planner = make_planner([
            [100, 100, 100, 100],
            [100, 20, -1, 100],
            [100, 100, 100, 100],
        ], free_threshold=25)
        self.assertTrue(planner._is_free(1, 1))
        self.assertTrue(planner._frontier_mask()[1, 1])

        strict = make_planner(planner.map_data, free_threshold=20)
        self.assertFalse(strict._is_free(1, 1))
        self.assertFalse(strict._frontier_mask()[1, 1])

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

    def test_significant_map_change_preserves_failed_goals(self):
        planner = make_planner(np.zeros((2, 3), dtype=np.int8))
        planner.map_frame = "map"
        planner.current_map = None
        planner.map_change_cell_threshold = 2
        planner.map_revision = 4
        planner.last_map_time = MODULE.rospy.Time(0)
        planner.last_significant_map_change = MODULE.rospy.Time(0)
        planner.no_reachable_frontier_cycles = 3
        failed_at = MODULE.rospy.Time.from_sec(10.0)
        planner.failed_goals = [(1.5, 2.5, failed_at)]
        message = SimpleNamespace(
            header=SimpleNamespace(frame_id="map"),
            info=planner.map_info,
            data=[100, 100, 0, 0, 0, 0],
        )
        update_time = MODULE.rospy.Time.from_sec(12.0)

        with patch.object(
            MODULE.rospy, "Time", SimpleNamespace(now=lambda: update_time)
        ):
            planner.map_callback(message)

        self.assertEqual(planner.map_revision, 5)
        self.assertEqual(planner.last_significant_map_change, update_time)
        self.assertEqual(planner.no_reachable_frontier_cycles, 0)
        self.assertEqual(planner.failed_goals, [(1.5, 2.5, failed_at)])

    def test_failed_goal_radius_is_cooled_down_until_timeout(self):
        planner = make_planner(np.zeros((1, 1), dtype=np.int8))
        planner.failed_goal_cooldown = 30.0
        planner.failed_goal_radius = 0.75
        planner.failed_goals = [
            (1.0, 2.0, MODULE.rospy.Time.from_sec(10.0))
        ]
        within_cooldown = MODULE.rospy.Time.from_sec(20.0)
        after_cooldown = MODULE.rospy.Time.from_sec(40.1)

        with patch.object(
            MODULE.rospy,
            "Time",
            SimpleNamespace(now=lambda: within_cooldown),
        ):
            self.assertTrue(planner._goal_is_cooled_down(1.74, 2.0))
            self.assertFalse(planner._goal_is_cooled_down(1.75, 2.0))

        with patch.object(
            MODULE.rospy,
            "Time",
            SimpleNamespace(now=lambda: after_cooldown),
        ):
            self.assertFalse(planner._goal_is_cooled_down(1.0, 2.0))
        self.assertEqual(planner.failed_goals, [])

    def test_select_goal_skips_cooled_down_frontier(self):
        planner = make_planner(np.zeros((1, 3), dtype=np.int8))
        planner.current_pose = SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.5)
        )
        planner.max_frontier_candidates = 20
        planner.failed_goal_cooldown = 30.0
        planner.failed_goal_radius = 0.75
        planner.failed_goals = [
            (0.5, 0.5, MODULE.rospy.Time.from_sec(10.0))
        ]
        within_cooldown = MODULE.rospy.Time.from_sec(20.0)
        planner._frontier_mask = lambda: np.ones((1, 3), dtype=bool)
        planner._reachable_free_mask = lambda: np.ones((1, 3), dtype=bool)
        planner._frontier_representatives = lambda _frontier=None: [(0, 0), (2, 0)]
        planner._check_path = lambda *_args: "reachable"

        with patch.object(
            MODULE.rospy,
            "Time",
            SimpleNamespace(now=lambda: within_cooldown),
        ):
            goal, reason = planner._select_goal()

        self.assertEqual(goal, (2.5, 0.5))
        self.assertEqual(reason, "reachable_frontier")

    def test_reachable_frontiers_are_filtered_before_candidate_limit(self):
        grid = np.full((7, 12), 100, dtype=np.int8)
        grid[5, 1:10] = 0
        grid[5, 10] = -1
        grid[3, 2] = 0
        grid[2, 2] = -1
        planner = make_planner(grid)
        planner.current_pose = SimpleNamespace(
            position=SimpleNamespace(x=1.5, y=5.5)
        )
        planner.max_frontier_candidates = 1
        planner.failed_goals = []
        planner._goal_is_cooled_down = lambda *_args: False
        checked_goals = []
        planner._check_path = lambda _sx, _sy, gx, gy: (
            checked_goals.append((gx, gy)) or "reachable"
        )

        goal, reason = planner._select_goal()

        self.assertEqual(reason, "reachable_frontier")
        self.assertEqual(goal, (9.5, 5.5))
        self.assertEqual(checked_goals, [(9.5, 5.5)])
        self.assertEqual(planner.remaining_frontier_count, 2)

    def test_inflation_disconnects_a_too_narrow_gap(self):
        grid = np.zeros((9, 9), dtype=np.int8)
        grid[:, 4] = 100
        grid[4, 4] = 0
        grid[4, 8] = -1
        planner = make_planner(grid)
        planner.connectivity_clearance_radius = 1.0
        planner.current_pose = SimpleNamespace(
            position=SimpleNamespace(x=1.5, y=4.5)
        )

        reachable = planner._reachable_free_mask()

        self.assertTrue(reachable[4, 1])
        self.assertFalse(reachable[4, 7])
        self.assertFalse((planner._frontier_mask() & reachable).any())

    def test_nearest_seed_is_limited_to_clearance_radius(self):
        planner = make_planner([[100, 0, 0, 0, 0]])
        planner.connectivity_clearance_radius = 1.0
        planner.current_pose = SimpleNamespace(
            position=SimpleNamespace(x=1.5, y=0.5)
        )

        reachable = planner._reachable_free_mask()

        self.assertFalse(reachable[0, 1])
        self.assertTrue(reachable[0, 2])
        self.assertTrue(reachable[0, 4])

        isolated = make_planner([[100, 0, 100, 0, 0]])
        isolated.connectivity_clearance_radius = 1.0
        isolated.current_pose = planner.current_pose
        self.assertFalse(isolated._reachable_free_mask().any())

    def test_existing_but_disconnected_frontier_is_not_no_frontier(self):
        grid = np.full((5, 7), 100, dtype=np.int8)
        grid[3, 1] = 0
        grid[1, 5] = 0
        grid[1, 6] = -1
        planner = make_planner(grid)
        planner.current_pose = SimpleNamespace(
            position=SimpleNamespace(x=1.5, y=3.5)
        )
        planner.max_frontier_candidates = 20
        planner.failed_goals = []

        goal, reason = planner._select_goal()

        self.assertIsNone(goal)
        self.assertEqual(reason, "all_frontiers_unreachable_or_blacklisted")
        self.assertEqual(planner.remaining_frontier_count, 1)

    def test_map_without_frontiers_reports_no_frontier(self):
        planner = make_planner(np.zeros((3, 3), dtype=np.int8))
        planner.current_pose = SimpleNamespace(
            position=SimpleNamespace(x=1.5, y=1.5)
        )

        goal, reason = planner._select_goal()

        self.assertIsNone(goal)
        self.assertEqual(reason, "no_frontier")
        self.assertEqual(planner.remaining_frontier_count, 0)

    def test_successful_goal_cools_until_map_progress(self):
        planner = make_planner(np.zeros((3, 3), dtype=np.int8))
        planner.failed_goals = []
        planner.successful_goals = [
            (1.0, 2.0, 5, MODULE.rospy.Time.from_sec(10.0))
        ]
        planner.failed_goal_cooldown = 15.0
        planner.failed_goal_radius = 0.5
        planner.success_goal_cooldown = 45.0
        planner.success_goal_radius = 0.5
        planner.success_goal_clear_revisions = 2
        planner.map_revision = 5

        with patch.object(
                MODULE.rospy.Time, "now",
                return_value=MODULE.rospy.Time.from_sec(20.0)):
            self.assertTrue(planner._goal_is_cooled_down(1.2, 2.0))

        planner.map_revision = 7
        with patch.object(
                MODULE.rospy.Time, "now",
                return_value=MODULE.rospy.Time.from_sec(20.0)):
            self.assertTrue(planner._goal_is_cooled_down(1.2, 2.0))

        with patch.object(
                MODULE.rospy.Time, "now",
                return_value=MODULE.rospy.Time.from_sec(60.0)):
            self.assertFalse(planner._goal_is_cooled_down(1.2, 2.0))

    def test_elevator_poses_follow_portal_heading(self):
        poses = MODULE.ExplorationPlanner._derive_elevator_poses(
            (2.0, 3.0, 0.0), 0.7, 0.9, 0.8
        )

        self.assertEqual(poses["portal"], (2.0, 3.0, 0.0))
        self.assertAlmostEqual(poses["lobby"][0], 1.3)
        self.assertAlmostEqual(poses["lobby"][1], 3.0)
        self.assertAlmostEqual(poses["cabin"][0], 2.9)
        self.assertAlmostEqual(poses["cabin"][1], 3.0)
        self.assertAlmostEqual(poses["exit"][0], 1.2)
        self.assertAlmostEqual(abs(poses["exit"][2]), math.pi)

    def test_autonomous_floor_selection_is_nearest_and_deterministic(self):
        planner = MODULE.ExplorationPlanner.__new__(MODULE.ExplorationPlanner)
        planner.current_floor = 1
        planner.elevator_served_floors = [0, 1, 2, 3]
        planner.completed_floors = {1, 3}
        planner.autonomous_transition_failures = {}
        planner.elevator_max_autonomous_transition_failures = 1

        self.assertEqual(planner._select_next_autonomous_floor(), 0)

        planner.completed_floors.add(0)
        self.assertEqual(planner._select_next_autonomous_floor(), 2)

    def test_blocked_floor_transition_is_not_global_completion(self):
        planner = MODULE.ExplorationPlanner.__new__(MODULE.ExplorationPlanner)
        planner.current_floor = 0
        planner.elevator_enabled = True
        planner.elevator_autonomous_when_floor_complete = True
        planner.elevator_served_floors = [0, 1]
        planner.elevator_max_autonomous_transition_failures = 1
        planner.autonomous_transition_failures = {(0, 1): 1}
        planner.visited_floors = set()
        planner.completed_floors = set()
        planner.complete_published = False
        planner.next_autonomous_floor = None
        states = []
        planner._set_state = lambda state, reason: states.append((state, reason))

        planner._handle_converged_floor("no_frontier")

        self.assertEqual(planner.completed_floors, {0})
        self.assertFalse(planner.complete_published)
        self.assertEqual(
            states[-1], ("FAILED", "autonomous_floor_transition_unavailable")
        )

    def test_all_served_floors_publish_global_completion(self):
        planner = MODULE.ExplorationPlanner.__new__(MODULE.ExplorationPlanner)
        planner.current_floor = 1
        planner.elevator_enabled = True
        planner.elevator_autonomous_when_floor_complete = True
        planner.elevator_served_floors = [0, 1]
        planner.visited_floors = {0}
        planner.completed_floors = {0}
        planner.complete_published = False
        published = []
        states = []
        planner.complete_pub = SimpleNamespace(
            publish=lambda message: published.append(message.data)
        )
        planner._set_state = lambda state, reason: states.append((state, reason))

        planner._handle_converged_floor("no_frontier")

        self.assertEqual(planner.completed_floors, {0, 1})
        self.assertEqual(published, [True])
        self.assertEqual(states[-1], ("COMPLETE", "all_served_floors_complete"))

    def test_autonomous_floor_change_requires_explicit_portal_pose(self):
        planner = MODULE.ExplorationPlanner.__new__(MODULE.ExplorationPlanner)
        planner.current_floor = 0
        planner.elevator_enabled = True
        planner.elevator_autonomous_when_floor_complete = True
        planner.elevator_served_floors = [0, 1]
        planner.elevator_max_autonomous_transition_failures = 1
        planner.autonomous_transition_failures = {}
        planner.elevator_portal_pose = None
        planner.visited_floors = set()
        planner.completed_floors = set()
        planner.complete_published = False
        planner.next_autonomous_floor = None
        states = []
        planner._set_state = lambda state, reason: states.append((state, reason))

        with patch.object(MODULE.rospy, "get_param", return_value=[]):
            planner._handle_converged_floor("no_frontier")

        self.assertFalse(planner.complete_published)
        self.assertEqual(planner.next_autonomous_floor, 1)
        self.assertEqual(
            states[-1], ("FAILED", "autonomous_elevator_portal_not_configured")
        )

    def test_manual_elevator_keeps_last_successful_goal_fallback(self):
        planner = MODULE.ExplorationPlanner.__new__(MODULE.ExplorationPlanner)
        planner.state_lock = threading.RLock()
        planner.elevator_target_floor = 1
        planner.elevator_portal_pose = None
        planner.last_successful_goal = (2.0, 3.0, 0.5)
        calls = []
        planner._start_elevator_locked = lambda target, portal, autonomous: (
            calls.append((target, portal, autonomous)) or (True, "started")
        )

        def get_param(name, default):
            if name == "~elevator/target_floor":
                return 1
            if name == "~elevator/portal_pose":
                return []
            return default

        with patch.object(MODULE.rospy, "get_param", side_effect=get_param):
            response = planner.start_elevator_cb(None)

        self.assertTrue(response.success)
        self.assertEqual(calls, [(1, (2.0, 3.0, 0.5), False)])

    def test_elevator_completion_resets_floor_local_runtime_state(self):
        planner = MODULE.ExplorationPlanner.__new__(MODULE.ExplorationPlanner)
        planner.elevator_source_floor = 0
        planner.elevator_target_floor = 1
        planner.elevator_active = True
        planner.elevator_autonomous_transition = True
        planner.next_autonomous_floor = 1
        planner.visited_floors = {0}
        planner.no_reachable_frontier_cycles = 5
        planner.retry_count = 2
        planner.failed_goals = [(1.0, 1.0, None)]
        planner.successful_goals = [(2.0, 2.0, 1, None)]
        planner.last_successful_goal = (2.0, 2.0, 0.0)
        planner.trap_blacklist = {(1, 1): 0}
        planner.observation_goal_cells = [(1, 1)]
        states = []
        planner._set_state = lambda state, reason: states.append((state, reason))

        now = MODULE.rospy.Time.from_sec(20.0)
        with patch.object(MODULE.rospy.Time, "now", return_value=now):
            planner._complete_elevator()

        self.assertEqual(planner.visited_floors, {0, 1})
        self.assertEqual(planner.failed_goals, [])
        self.assertEqual(planner.successful_goals, [])
        self.assertEqual(planner.trap_blacklist, {})
        self.assertEqual(planner.no_reachable_frontier_cycles, 0)
        self.assertEqual(planner.last_significant_map_change, now)
        self.assertEqual(states[-1], ("WAITING", "elevator_complete"))


if __name__ == "__main__":
    unittest.main()
