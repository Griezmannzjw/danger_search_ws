#!/usr/bin/env python3

import importlib.util
import math
import pathlib
import unittest
from types import SimpleNamespace

import numpy as np


SCRIPT = pathlib.Path(__file__).parents[1] / "scripts" / "exploration_planner.py"
SPEC = importlib.util.spec_from_file_location("exploration_planner", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def pose(x, y, yaw):
    return SimpleNamespace(
        position=SimpleNamespace(x=x, y=y),
        orientation=SimpleNamespace(
            x=0.0, y=0.0, z=math.sin(0.5 * yaw), w=math.cos(0.5 * yaw)
        ),
    )


class EntranceBoundaryTest(unittest.TestCase):
    def test_forward_side_and_allowance_goals_are_retained(self):
        anchor = MODULE.entrance_boundary_anchor_from_pose(pose(2.0, 3.0, 0.0), 0)
        self.assertTrue(MODULE.entrance_boundary_allows_goal(anchor, 2.5, 3.0, 1.0))
        self.assertTrue(MODULE.entrance_boundary_allows_goal(anchor, 2.0, 9.0, 1.0))
        self.assertTrue(MODULE.entrance_boundary_allows_goal(anchor, 1.0, 3.0, 1.0))

    def test_goal_beyond_backwards_allowance_is_rejected(self):
        anchor = MODULE.entrance_boundary_anchor_from_pose(pose(2.0, 3.0, 0.0), 0)
        self.assertFalse(MODULE.entrance_boundary_allows_goal(anchor, 0.99, 3.0, 1.0))

    def test_other_floors_are_not_filtered_by_entrance_anchor(self):
        planner = MODULE.ExplorationPlanner.__new__(MODULE.ExplorationPlanner)
        planner.entrance_boundary_guard_enabled = True
        planner.entrance_boundary_allowance_m = 1.0
        planner.entrance_boundary_anchor = MODULE.entrance_boundary_anchor_from_pose(
            pose(2.0, 3.0, 0.0), 0
        )
        planner.current_floor = 1
        self.assertTrue(planner._entrance_boundary_allows_goal(-100.0, 3.0))

    def test_start_service_recaptures_anchor_for_a_new_session(self):
        planner = MODULE.ExplorationPlanner.__new__(MODULE.ExplorationPlanner)
        planner.state_lock = MODULE.threading.RLock()
        planner.exploring = False
        planner.current_floor = 0
        planner.current_pose = pose(0.0, 0.0, 0.0)
        planner.entrance_boundary_guard_enabled = True
        planner.entrance_boundary_allowance_m = 1.0
        planner.session_id = 0
        planner.complete_pub = SimpleNamespace(publish=lambda _message: None)
        planner._set_state = lambda *_args: None

        first = planner.start_exploration_cb(None)
        self.assertTrue(first.success)
        self.assertEqual(planner.entrance_boundary_anchor[:2], (0.0, 0.0))

        # Simulate a stop between sessions without invoking unrelated action
        # client cleanup.  The next service start must not retain old geometry.
        planner.exploring = False
        planner.current_pose = pose(4.0, -2.0, math.pi / 2.0)
        second = planner.start_exploration_cb(None)
        self.assertTrue(second.success)
        self.assertEqual(planner.entrance_boundary_anchor[:2], (4.0, -2.0))
        self.assertAlmostEqual(planner.entrance_boundary_anchor[2], math.pi / 2.0)

    def test_excluded_exterior_frontier_does_not_block_completion(self):
        planner = MODULE.ExplorationPlanner.__new__(MODULE.ExplorationPlanner)
        planner.current_pose = pose(4.0, 0.0, 0.0)
        planner.current_floor = 0
        planner.map_data = np.zeros((1, 6), dtype=np.int8)
        planner.map_info = SimpleNamespace(
            width=6,
            height=1,
            resolution=1.0,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        )
        planner.entrance_boundary_guard_enabled = True
        planner.entrance_boundary_allowance_m = 1.0
        planner.entrance_boundary_anchor = (4.0, 0.0, 0.0, 0)
        planner.coverage_debt_by_floor = {0: {(1, 0, "old")}}
        planner._frontier_mask = lambda: np.ones((1, 6), dtype=bool)
        planner._frontier_representatives = lambda _frontier=None: [(0, 0)]

        goal, reason = planner._select_goal()

        self.assertIsNone(goal)
        self.assertEqual(reason, "no_frontier")
        self.assertEqual(planner.remaining_frontier_count, 0)
        self.assertNotIn(0, planner.coverage_debt_by_floor)


if __name__ == "__main__":
    unittest.main()
