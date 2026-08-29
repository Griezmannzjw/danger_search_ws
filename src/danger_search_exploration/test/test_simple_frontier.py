#!/usr/bin/env python3

import importlib.util
import math
import pathlib
import time
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
    planner.current_floor = 0
    planner.coverage_debt_by_floor = {}
    return planner


class SimpleFrontierTest(unittest.TestCase):
    def test_cached_wfd_1024_planning_p95_is_below_50_ms(self):
        size = 1024
        grid = np.full((size, size), -1, dtype=np.int8)
        grid[112:912, 112:912] = 0
        grid[300:305, 250:750] = 100
        planner = make_planner(
            grid, resolution=0.05, min_frontier_length=0.40, free_threshold=40
        )
        planner.current_pose = SimpleNamespace(
            position=SimpleNamespace(x=25.6, y=25.6)
        )
        planner.connectivity_clearance_radius = 0.30
        planner.goal_clearance_margin = 0.02
        planner.observation_min_distance = 0.30
        planner.observation_target_distance = 0.45
        planner.observation_max_distance = 0.70
        planner.trap_blacklist = {}
        planner.observation_goals_pub = None
        planner.map_epoch = 1
        planner.map_revision = 1
        samples_ms = []
        for _index in range(20):
            started = time.perf_counter()
            frontier = planner._frontier_mask()
            reachable = planner._reachable_free_mask()
            planner._observation_goals(frontier, reachable)
            samples_ms.append((time.perf_counter() - started) * 1000.0)
        self.assertLess(float(np.percentile(samples_ms, 95)), 50.0)

    def test_relaxed_exploration_defaults(self):
        config_path = pathlib.Path(__file__).parents[1] / "config" / "default.yaml"
        with config_path.open(encoding="utf-8") as stream:
            config = yaml.safe_load(stream)

        self.assertEqual(config["free_threshold"], 40)
        self.assertAlmostEqual(config["connectivity_clearance_radius"], 0.30)
        self.assertAlmostEqual(config["min_frontier_length"], 0.40)
        self.assertAlmostEqual(config["goal_timeout"], 120.0)
        self.assertAlmostEqual(config["max_frontier_goal_path_m"], 8.0)
        self.assertLess(
            MODULE.bounded_navigation_timeout(
                config["max_frontier_goal_path_m"],
                config["goal_timeout"],
                config["goal_timeout_base_s"],
                config["goal_timeout_per_path_m"],
                config["goal_timeout_min_s"],
            ),
            config["goal_timeout"],
        )
        self.assertAlmostEqual(config["observation_min_distance"], 0.60)
        self.assertAlmostEqual(config["observation_target_distance"], 0.70)
        self.assertAlmostEqual(config["observation_max_distance"], 0.90)
        self.assertFalse(config["entrance_boundary_guard_enabled"])
        self.assertAlmostEqual(config["entrance_boundary_allowance_m"], 1.0)
        self.assertAlmostEqual(config["goal_clearance_margin"], 0.15)
        self.assertAlmostEqual(config["path_clearance_weight"], 0.15)
        self.assertAlmostEqual(config["failed_goal_cooldown"], 15.0)
        self.assertAlmostEqual(config["failed_goal_radius"], 0.50)
        self.assertAlmostEqual(config["min_goal_dispatch_distance_m"], 0.45)
        self.assertAlmostEqual(config["floor_unreachable_hold_s"], 30.0)
        self.assertAlmostEqual(config["trap_blacklist_radius"], 0.70)
        self.assertAlmostEqual(config["trap_clearance_margin"], 0.08)
        self.assertEqual(config["blacklist_clear_revisions"], 2)
        self.assertGreaterEqual(config["max_frontier_candidates"], 64)

        navigation_path = (
            pathlib.Path(__file__).parents[2]
            / "danger_search_navigation"
            / "config"
            / "dwa_planner.yaml"
        )
        with navigation_path.open(encoding="utf-8") as stream:
            dwa = yaml.safe_load(stream)["DWAPlannerROS"]
        self.assertGreater(
            config["min_goal_dispatch_distance_m"],
            dwa["xy_goal_tolerance"],
        )
        self.assertLessEqual(
            config["observation_target_distance"] + dwa["xy_goal_tolerance"],
            config["observation_max_distance"],
        )
        footprint_radius = math.hypot(0.35, 0.15) + 0.04
        self.assertGreaterEqual(
            config["observation_min_distance"] - dwa["xy_goal_tolerance"],
            footprint_radius,
        )
        self.assertGreaterEqual(
            config["connectivity_clearance_radius"]
            + config["goal_clearance_margin"],
            footprint_radius,
        )

    def test_recovery_trigger_is_diagnostic_and_failed_event_blacklists(self):
        planner = make_planner(np.zeros((5, 5), dtype=np.int8))
        planner.map_frame = "map"
        planner.state_lock = MODULE.threading.RLock()
        planner.exploring = True
        planner.floor_change_active = False
        planner.nav_active_goal_id = "goal-a"
        planner.navigation_goal_sent_at = MODULE.rospy.Time(0)
        planner.last_recovery_event_id = 0
        planner.last_recovery_goal_id = ""
        planner.last_recovery_stuck_pose = None
        remembered = []
        planner._remember_trap_region = lambda x, y: remembered.append((x, y))
        event = MODULE.RecoveryEvent()
        event.header.frame_id = "map"
        event.event_id = 7
        event.active_goal_id = "goal-a"
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

    def test_recovery_from_mission_goal_does_not_pollute_blacklist(self):
        planner = make_planner(np.zeros((5, 5), dtype=np.int8))
        planner.map_frame = "map"
        planner.state_lock = MODULE.threading.RLock()
        planner.exploring = False
        planner.floor_change_active = False
        planner.nav_active_goal_id = "mission-entry"
        planner.navigation_goal_sent_at = MODULE.rospy.Time(0)
        planner.last_recovery_event_id = 0
        planner.last_recovery_goal_id = ""
        remembered = []
        planner._remember_trap_region = lambda x, y: remembered.append((x, y))
        event = MODULE.RecoveryEvent()
        event.header.frame_id = "map"
        event.event_id = 8
        event.active_goal_id = "mission-entry"
        event.phase = MODULE.RecoveryEvent.PHASE_FAILED

        planner.recovery_event_callback(event)

        self.assertEqual(remembered, [])
        self.assertEqual(planner.last_recovery_event_id, 0)

    def test_delayed_recovery_from_previous_goal_is_ignored(self):
        planner = make_planner(np.zeros((5, 5), dtype=np.int8))
        planner.map_frame = "map"
        planner.state_lock = MODULE.threading.RLock()
        planner.exploring = True
        planner.floor_change_active = False
        planner.nav_active_goal_id = "goal-b"
        planner.navigation_goal_sent_at = MODULE.rospy.Time.from_sec(10.0)
        planner.last_recovery_event_id = 0
        planner.last_recovery_goal_id = ""
        remembered = []
        planner._remember_trap_region = lambda x, y: remembered.append((x, y))
        event = MODULE.RecoveryEvent()
        event.header.frame_id = "map"
        event.header.stamp = MODULE.rospy.Time.from_sec(9.0)
        event.event_id = 9
        event.active_goal_id = "goal-a"
        event.phase = MODULE.RecoveryEvent.PHASE_FAILED

        planner.recovery_event_callback(event)

        self.assertEqual(remembered, [])
        self.assertEqual(planner.last_recovery_event_id, 0)

    def test_floor_transit_recovery_does_not_pollute_exploration_blacklist(self):
        planner = make_planner(np.zeros((5, 5), dtype=np.int8))
        planner.map_frame = "map"
        planner.state_lock = MODULE.threading.RLock()
        planner.exploring = True
        planner.floor_change_active = True
        planner.nav_active_goal_id = "hall-goal"
        planner.navigation_goal_sent_at = MODULE.rospy.Time.from_sec(10.0)
        planner.last_recovery_event_id = 0
        planner.last_recovery_goal_id = ""
        remembered = []
        planner._remember_trap_region = lambda x, y: remembered.append((x, y))
        event = MODULE.RecoveryEvent()
        event.header.frame_id = "map"
        event.header.stamp = MODULE.rospy.Time.from_sec(11.0)
        event.event_id = 10
        event.active_goal_id = "hall-goal"
        event.phase = MODULE.RecoveryEvent.PHASE_FAILED
        event.stuck_pose.position.x = 1.0
        event.stuck_pose.position.y = 2.0

        planner.recovery_event_callback(event)

        self.assertEqual(remembered, [])
        self.assertEqual(planner.last_recovery_event_id, 0)

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

    def test_close_frontier_clusters_do_not_starve_each_other(self):
        planner = make_planner(
            np.zeros((30, 30), dtype=np.int8),
            resolution=0.10,
            min_frontier_length=0.20,
        )
        planner.connectivity_clearance_radius = 0.20
        planner.goal_clearance_margin = 0.04
        planner.observation_min_distance = 0.60
        planner.observation_target_distance = 0.70
        planner.observation_max_distance = 0.80
        planner.trap_blacklist = {}
        planner.observation_goals_pub = None

        # Two independently reachable frontier lines bound a one-metre-wide
        # known corridor.  No cell can be 0.60 m from *both* lines, but each
        # line still has a valid viewpoint relative to its own cluster.
        frontier = np.zeros((30, 30), dtype=bool)
        frontier[5:25, 9] = True
        frontier[5:25, 19] = True
        reachable = np.zeros_like(frontier)
        reachable[5:25, 9:20] = True
        global_distance = MODULE.cv2.distanceTransform(
            (frontier == 0).astype(np.uint8),
            MODULE.cv2.DIST_L2,
            MODULE.cv2.DIST_MASK_PRECISE,
        ) * planner.map_info.resolution
        self.assertLess(float(global_distance[reachable].max()), 0.60)

        goals = planner._observation_goals(frontier, reachable)

        self.assertEqual(len(goals), 2)
        required_clearance = (
            planner.connectivity_clearance_radius
            + planner.goal_clearance_margin
        )
        for goal in goals:
            cell_x, cell_y = goal["cell"]
            self.assertGreaterEqual(
                float(global_distance[cell_y, cell_x]), required_clearance
            )

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
        self.assertIn((0, 0, "cooldown"), planner.coverage_debt_by_floor[0])

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

    def test_goal_inside_move_base_tolerance_does_not_hide_far_candidate(self):
        planner = make_planner(np.zeros((1, 3), dtype=np.int8))
        planner.current_pose = SimpleNamespace(
            position=SimpleNamespace(x=0.5, y=0.5)
        )
        planner.max_frontier_candidates = 1
        planner.min_goal_dispatch_distance_m = 0.45
        planner.failed_goals = []
        planner._goal_is_cooled_down = lambda *_args: False
        planner._frontier_mask = lambda: np.ones((1, 3), dtype=bool)
        planner._reachable_free_mask = lambda: np.ones((1, 3), dtype=bool)
        planner._frontier_representatives = lambda _frontier=None: [(0, 0), (2, 0)]
        checked_goals = []
        planner._check_path = lambda _sx, _sy, gx, gy: (
            checked_goals.append((gx, gy)) or "reachable"
        )

        goal, reason = planner._select_goal()

        self.assertEqual(reason, "reachable_frontier")
        self.assertEqual(goal, (2.5, 0.5))
        self.assertEqual(checked_goals, [(2.5, 0.5)])

    def test_path_scoring_reaches_candidate_beyond_old_nearest_twenty(self):
        """A geodesically short frontier must not lose to a Euclidean cutoff."""
        planner = make_planner(np.zeros((2, 80), dtype=np.int8))
        planner.current_pose = SimpleNamespace(
            position=SimpleNamespace(x=0.5, y=0.5)
        )
        planner.max_frontier_candidates = 64
        planner.min_goal_dispatch_distance_m = 0.0
        planner.observation_target_distance = 0.70
        planner.failed_goals = []
        planner.trap_blacklist = {}
        planner._goal_is_cooled_down = lambda *_args: False
        planner._frontier_mask = lambda: np.ones((2, 80), dtype=bool)
        planner._reachable_free_mask = lambda: np.ones((2, 80), dtype=bool)
        planner._frontier_representatives = lambda _frontier=None: [(1, 0)]
        planner._observation_goals = lambda _frontier, _reachable: [
            {
                "x": float(index) + 0.5,
                "y": 0.5,
                "yaw": 0.0,
                "gain": 0.0,
                "clearance": 1.0,
            }
            for index in range(1, 22)
        ]

        def checked_path(_start_x, _start_y, goal_x, _goal_y):
            # The first 20 straight-line-nearest candidates require a long
            # detour.  Candidate 21 is slightly farther in Euclidean space but
            # has the shortest valid global plan.
            planner.last_checked_path_metrics = {
                "path_length": 100.0 if goal_x < 21.5 else 2.0,
                "min_clearance": 1.0,
            }
            return "reachable"

        planner._check_path = checked_path

        goal, reason = planner._select_goal()

        self.assertEqual(reason, "reachable_frontier")
        self.assertEqual(goal, (21.5, 0.5, 0.0))

    def test_long_global_plan_is_dispatched_as_receding_horizon_waypoint(self):
        planner = make_planner(np.zeros((2, 40), dtype=np.int8))
        planner.current_pose = SimpleNamespace(
            position=SimpleNamespace(x=0.0, y=0.0)
        )
        planner.max_frontier_candidates = 64
        planner.max_frontier_goal_path_m = 8.0
        planner.min_goal_dispatch_distance_m = 0.0
        planner.observation_target_distance = 0.70
        planner.failed_goals = []
        planner.trap_blacklist = {}
        planner._goal_is_cooled_down = lambda *_args: False
        planner._frontier_mask = lambda: np.ones((2, 40), dtype=bool)
        planner._reachable_free_mask = lambda: np.ones((2, 40), dtype=bool)
        planner._frontier_representatives = lambda _frontier=None: [(1, 0)]
        planner._observation_goals = lambda _frontier, _reachable: [{
            "x": 4.0,
            "y": 8.0,
            "yaw": 1.0,
            "gain": 1.0,
            "clearance": 1.0,
        }]

        def checked_path(*_args):
            planner.last_checked_path_metrics = {
                "path_length": 12.0,
                "min_clearance": 1.0,
                "points": [(0.0, 0.0), (6.0, 0.0), (6.0, 6.0)],
            }
            return "reachable"

        planner._check_path = checked_path
        goal, reason = planner._select_goal()

        self.assertEqual(reason, "reachable_frontier_waypoint")
        self.assertEqual(goal[:2], (6.0, 2.0))
        self.assertAlmostEqual(goal[2], math.pi / 2.0)
        self.assertEqual(planner.selected_goal_metrics["dispatch_path_length"], 8.0)
        self.assertTrue(planner.selected_goal_metrics["truncated"])

    def test_path_prefix_and_dynamic_timeout_validate_boundaries(self):
        goal = MODULE.path_prefix_goal([(0.0, 0.0), (3.0, 4.0)], 2.5)
        self.assertEqual(goal[:2], (1.5, 2.0))
        self.assertAlmostEqual(goal[2], math.atan2(4.0, 3.0))
        self.assertEqual(goal[3:], (2.5, True))
        self.assertEqual(
            MODULE.bounded_navigation_timeout(8.0, 120.0, 20.0, 4.0, 30.0),
            52.0,
        )
        self.assertEqual(
            MODULE.bounded_navigation_timeout(100.0, 120.0, 20.0, 4.0, 30.0),
            120.0,
        )
        with self.assertRaises(ValueError):
            MODULE.path_prefix_goal([], 8.0)

    def test_only_near_frontier_waits_without_claiming_completion(self):
        planner = make_planner(np.zeros((1, 1), dtype=np.int8))
        planner.current_pose = SimpleNamespace(
            position=SimpleNamespace(x=0.5, y=0.5)
        )
        planner.max_frontier_candidates = 20
        planner.min_goal_dispatch_distance_m = 0.45
        planner.failed_goals = []
        planner._goal_is_cooled_down = lambda *_args: False
        planner._frontier_mask = lambda: np.ones((1, 1), dtype=bool)
        planner._reachable_free_mask = lambda: np.ones((1, 1), dtype=bool)
        planner._frontier_representatives = lambda _frontier=None: [(0, 0)]
        planner._check_path = lambda *_args: self.fail("near goal reached make_plan")

        goal, reason = planner._select_goal()

        self.assertIsNone(goal)
        self.assertEqual(reason, "frontier_already_in_observation_range")

    @staticmethod
    def _completion_planner():
        planner = make_planner(np.zeros((1, 1), dtype=np.int8))
        planner.no_reachable_frontier_cycles = 0
        planner.floor_no_frontier_since = MODULE.rospy.Time(0)
        planner.floor_unreachable_since = MODULE.rospy.Time(0)
        planner.no_frontier_cycles_required = 5
        planner.floor_no_frontier_hold_s = 10.0
        planner.floor_unreachable_hold_s = 30.0
        planner.map_stable_time = 8.0
        planner.last_significant_map_change = MODULE.rospy.Time(0)
        planner.waiting_for_result = False
        planner.nav_has_active_goal = False
        planner._navigation_service_available = lambda: True
        return planner

    def test_strict_no_frontier_completion_contract_is_unchanged(self):
        planner = self._completion_planner()
        mode = None
        for stamp in (1.0, 3.0, 5.0, 7.0, 11.0):
            mode, reason = planner._floor_completion_mode(
                "no_frontier", MODULE.rospy.Time.from_sec(stamp)
            )
        self.assertEqual(reason, "no_frontier")
        self.assertEqual(mode, "strict_no_frontier")

        planner = self._completion_planner()
        planner.coverage_debt_by_floor[0] = {(3, 4, "unreachable")}
        for stamp in (1.0, 3.0, 5.0, 7.0, 11.0):
            mode, _reason = planner._floor_completion_mode(
                "no_frontier", MODULE.rospy.Time.from_sec(stamp)
            )
        self.assertIsNone(mode)

    def test_bounded_unreachable_completion_waits_and_preserves_debt(self):
        planner = self._completion_planner()
        debt = {(5, 1, "disconnected")}
        planner.coverage_debt_by_floor[0] = set(debt)
        reason = "all_frontiers_unreachable_or_blacklisted"
        mode, _ = planner._floor_completion_mode(
            reason, MODULE.rospy.Time.from_sec(1.0)
        )
        self.assertIsNone(mode)
        mode, _ = planner._floor_completion_mode(
            reason, MODULE.rospy.Time.from_sec(30.99)
        )
        self.assertIsNone(mode)
        mode, _ = planner._floor_completion_mode(
            reason, MODULE.rospy.Time.from_sec(31.0)
        )
        self.assertEqual(mode, "bounded_unreachable")
        self.assertEqual(planner.coverage_debt_by_floor[0], debt)

    def test_bounded_unreachable_timer_resets_on_reachable_goal(self):
        planner = self._completion_planner()
        reason = "frontier_already_in_observation_range"
        planner._floor_completion_mode(
            reason, MODULE.rospy.Time.from_sec(1.0)
        )
        planner._floor_completion_mode(
            "reachable_frontier", MODULE.rospy.Time.from_sec(20.0)
        )
        mode, _ = planner._floor_completion_mode(
            reason, MODULE.rospy.Time.from_sec(31.0)
        )
        self.assertIsNone(mode)
        self.assertEqual(
            planner.floor_unreachable_since,
            MODULE.rospy.Time.from_sec(31.0),
        )

    def test_navigation_service_failure_resets_bounded_evidence(self):
        planner = self._completion_planner()
        reason = "all_frontiers_unreachable_or_blacklisted"
        planner._navigation_service_available = lambda: False
        planner._floor_completion_mode(
            reason, MODULE.rospy.Time.from_sec(1.0)
        )
        mode, wait_reason = planner._floor_completion_mode(
            reason, MODULE.rospy.Time.from_sec(31.0)
        )
        self.assertIsNone(mode)
        self.assertEqual(wait_reason, "navigation_service_unavailable")
        self.assertEqual(planner.floor_unreachable_since, MODULE.rospy.Time(0))

    def test_input_failure_reset_and_active_goal_block_completion(self):
        planner = self._completion_planner()
        reason = "frontier_already_in_observation_range"
        planner._floor_completion_mode(
            reason, MODULE.rospy.Time.from_sec(1.0)
        )
        planner._reset_bounded_floor_completion_evidence()
        mode, _ = planner._floor_completion_mode(
            reason, MODULE.rospy.Time.from_sec(31.0)
        )
        self.assertIsNone(mode)

        planner.nav_has_active_goal = True
        mode, _ = planner._floor_completion_mode(
            reason, MODULE.rospy.Time.from_sec(61.0)
        )
        self.assertIsNone(mode)

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
        self.assertIn((5, 1, "disconnected"), planner.coverage_debt_by_floor[0])

    def test_map_without_frontiers_reports_no_frontier(self):
        planner = make_planner(np.zeros((3, 3), dtype=np.int8))
        planner.current_pose = SimpleNamespace(
            position=SimpleNamespace(x=1.5, y=1.5)
        )
        planner.coverage_debt_by_floor[0] = {(1, 1, "stale")}

        goal, reason = planner._select_goal()

        self.assertIsNone(goal)
        self.assertEqual(reason, "no_frontier")
        self.assertEqual(planner.remaining_frontier_count, 0)
        self.assertNotIn(0, planner.coverage_debt_by_floor)


if __name__ == "__main__":
    unittest.main()
