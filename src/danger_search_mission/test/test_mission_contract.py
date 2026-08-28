#!/usr/bin/env python3

import pathlib
import unittest

import yaml

from danger_search_mission.mission_core import (
    DEFAULT_ENTRY_COMPLETION_TOLERANCE_M,
)


PACKAGE = pathlib.Path(__file__).parents[1]


class MissionContractTest(unittest.TestCase):
    def test_required_topics_and_timeouts_are_configured(self):
        with (PACKAGE / "config" / "default.yaml").open(encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        self.assertEqual(config["exploration_complete_topic"], "/exploration/complete")
        self.assertEqual(config["pose_topic"], "/localization/pose")
        self.assertEqual(config["return_home_service"], "/danger_search/return_home")
        self.assertEqual(
            config["posture_fallen_topic"], "/danger_search/posture_fallen"
        )
        self.assertEqual(
            config["posture_reason_topic"],
            "/danger_search/posture_safety_reason",
        )
        self.assertEqual(config["recoverable_safety_abort_s"], 3.0)
        self.assertGreaterEqual(config["min_detections"], 2)
        self.assertGreater(config["return_timeout_s"], 0)
        self.assertEqual(
            config["mission_timeout_s"],
            0.0,
            "zero disables automatic timeout return while exploration is active",
        )
        self.assertLessEqual(
            config["return_attempt_timeout_s"],
            config["return_timeout_s"]
            - config["return_retry_reserve_s"]
            - config["return_terminal_reserve_s"],
        )
        self.assertGreater(config["return_min_attempt_timeout_s"], 0.0)
        self.assertTrue(config["entry_enabled"])
        self.assertEqual(config["entry_step_m"], 0.6)
        self.assertLess(config["entry_step_m"], config["entry_distance_m"])
        self.assertEqual(config["entry_timeout_s"], 150.0)
        self.assertGreaterEqual(config["entry_max_retries"], 1)
        self.assertGreater(config["entry_health_settle_s"], 0.0)
        self.assertGreater(config["entry_map_retry_delay_s"], config["entry_retry_delay_s"])
        self.assertLess(
            config["entry_completion_tolerance_m"], config["entry_distance_m"]
        )
        self.assertTrue(config["require_entrance_ready"])
        self.assertTrue(config["require_preflight_ready"])
        self.assertEqual(config["transit_floor_action_name"], "/danger_search/transit_floor")
        self.assertEqual(config["return_stationary_hold_s"], 2.0)
        self.assertGreater(config["return_retry_delay_s"], 0.0)

    def test_entry_completion_accepts_move_base_goal_tolerance(self):
        with (PACKAGE / "config" / "default.yaml").open(encoding="utf-8") as stream:
            mission = yaml.safe_load(stream)
        self.assertEqual(
            mission["entry_completion_tolerance_m"],
            DEFAULT_ENTRY_COMPLETION_TOLERANCE_M,
        )
        navigation = PACKAGE.parent / "danger_search_navigation" / "config"
        for planner_file in ("dwa_planner.yaml", "trajectory_planner.yaml"):
            with (navigation / planner_file).open(encoding="utf-8") as stream:
                planner = yaml.safe_load(stream)
            planner_config = next(iter(planner.values()))
            self.assertGreaterEqual(
                mission["entry_completion_tolerance_m"],
                planner_config["xy_goal_tolerance"],
                "%s can report success before Mission accepts entry completion"
                % planner_file,
            )

    def test_guarded_entry_and_control_mux_share_hard_limits(self):
        with (PACKAGE / "config" / "default.yaml").open(encoding="utf-8") as stream:
            mission = yaml.safe_load(stream)
        control_path = PACKAGE.parent / "danger_search_control" / "config" / "default.yaml"
        with control_path.open(encoding="utf-8") as stream:
            control = yaml.safe_load(stream)
        self.assertTrue(mission["entry_short_range_enabled"])
        self.assertTrue(mission["entry_crossing_sweep_enabled"])
        self.assertGreater(mission["entry_crossing_sweep_lookahead_m"], 0.0)
        self.assertLessEqual(
            mission["entry_crossing_sweep_lookahead_m"],
            mission["entry_crossing_distance_m"],
        )
        self.assertGreaterEqual(
            mission["entry_crossing_sweep_lookahead_m"],
            2.0 * mission["entry_crossing_speed_mps"],
            "rolling sweep must preserve at least two seconds of lookahead",
        )
        self.assertEqual(mission["entry_cmd_topic"], control["entry_cmd_topic"])
        self.assertLessEqual(
            mission["entry_crossing_speed_mps"],
            control["entry_max_linear_speed"],
        )
        self.assertLessEqual(
            mission["entry_crossing_max_angular_speed_rps"],
            control["entry_max_angular_speed"],
        )
        self.assertLess(
            mission["entry_margin_avoidance_speed_mps"],
            mission["entry_crossing_speed_mps"],
        )
        self.assertLessEqual(
            mission["entry_margin_avoidance_yaw_rps"],
            mission["entry_crossing_max_angular_speed_rps"],
        )
        self.assertLessEqual(
            mission["entry_margin_avoidance_lookahead_m"],
            mission["entry_crossing_sweep_lookahead_m"],
        )
        self.assertLessEqual(
            mission["entry_arc_sweep_sample_spacing_m"], 0.01
        )
        self.assertGreaterEqual(mission["entry_crossing_yaw_deadband_rad"], 0.0)
        self.assertLess(
            mission["entry_crossing_yaw_deadband_rad"],
            mission["entry_crossing_max_yaw_error_rad"],
        )
        self.assertGreater(mission["entry_crossing_yaw_lowpass_alpha"], 0.0)
        self.assertLessEqual(mission["entry_crossing_yaw_lowpass_alpha"], 1.0)
        self.assertGreater(
            mission["entry_crossing_release_s"], control["entry_timeout_s"]
        )
        self.assertLess(
            mission["entry_crossing_distance_m"], mission["entry_distance_m"]
        )
        self.assertGreaterEqual(
            mission["entry_crossing_distance_m"],
            mission["entry_distance_m"]
            - mission["entry_completion_tolerance_m"],
            "guarded crossing must not hand off against a stale closed-door map",
        )
        self.assertGreaterEqual(mission["entry_crossing_timeout_s"], 30.0)
        self.assertLess(
            mission["entry_crossing_max_tilt_rad"], 0.55,
            "entry must stop before the localization unstable-scan cutoff",
        )

    def test_manager_owns_complete_subscription_and_return_goal(self):
        source = (PACKAGE / "scripts" / "mission_manager.py").read_text(encoding="utf-8")
        self.assertIn("self.exploration_complete_sub = rospy.Subscriber", source)
        self.assertIn("goal = MoveBaseGoal()", source)
        self.assertIn("self._return_done_callback(", source)
        self.assertIn("self.return_retry_at = (", source)
        self.assertIn('return_retry_reason or "action_retry"', source)
        self.assertIn("allocate_return_attempt_budget(", source)
        self.assertIn("self.return_goal_deadline = rospy.Time(0)", source)
        self.assertIn('self._finalize("return_budget_exhausted"', source)
        self.assertIn("self._return_transit_done_callback(", source)
        self.assertIn("return_epoch != self.return_epoch", source)
        self.assertIn("self._entry_done_callback(", source)
        self.assertIn("self.entry_retry_at = rospy.Time.now()", source)
        self.assertIn('navigation_failure == "LOCALIZATION_LOST"', source)
        self.assertIn('"UNREACHABLE",', source)
        self.assertIn("self._classify_entry_failure(sequence, state)", source)
        self.assertIn("if entry_goal_active:", source)
        self.assertIn("self._entry_localization_ready(now)", source)
        self.assertIn("self._entry_control_timer_callback", source)
        self.assertIn("swept_footprint_hit(", source)
        self.assertIn("swept_arc_footprint_hit(", source)
        self.assertIn("swept_footprint_hits(", source)
        self.assertIn("margin_avoidance_command(", source)
        self.assertIn("rolling_sweep_distance(", source)
        self.assertIn("self.move_base_client.cancel_all_goals()", source)
        self.assertIn('self.entry_short_range_phase = "HANDOFF"', source)
        self.assertIn("os.replace(temporary, self.result_file)", source)
        self.assertIn("self._abort_for_safety_stop", source)
        self.assertIn("self.posture_fallen_sub = rospy.Subscriber", source)
        self.assertIn("self.posture_reason_sub = rospy.Subscriber", source)
        self.assertIn('self._finalize("posture_safety_stop:" + detail', source)
        self.assertIn("and not self.safety_abort_started", source)
        self.assertIn('"SAFETY_STOP",', source)
        self.assertIn('self.return_retry_reason = "safety_recovery"', source)
        self.assertIn("if safety_stop:\n            return", source)
        self.assertIn("if self.shutting_down or rospy.is_shutdown():", source)


if __name__ == "__main__":
    unittest.main()
