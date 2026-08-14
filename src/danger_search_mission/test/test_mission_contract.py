#!/usr/bin/env python3

import pathlib
import unittest

import yaml


PACKAGE = pathlib.Path(__file__).parents[1]


class MissionContractTest(unittest.TestCase):
    def test_required_topics_and_timeouts_are_configured(self):
        with (PACKAGE / "config" / "default.yaml").open(encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        self.assertEqual(config["exploration_complete_topic"], "/exploration/complete")
        self.assertEqual(config["pose_topic"], "/localization/pose")
        self.assertEqual(config["return_home_service"], "/danger_search/return_home")
        self.assertGreaterEqual(config["min_detections"], 2)
        self.assertGreater(config["return_timeout_s"], 0)
        self.assertEqual(config["mission_timeout_s"], 0.0)
        self.assertTrue(config["entry_enabled"])
        self.assertGreater(config["entry_step_m"], 0.0)
        self.assertLess(config["entry_step_m"], config["entry_distance_m"])
        self.assertGreaterEqual(config["entry_max_retries"], 1)
        self.assertGreater(config["entry_health_settle_s"], 0.0)
        self.assertGreater(config["entry_map_retry_delay_s"], config["entry_retry_delay_s"])
        self.assertLess(
            config["entry_completion_tolerance_m"], config["entry_distance_m"]
        )
        self.assertTrue(config["require_entrance_ready"])

    def test_manager_owns_complete_subscription_and_return_goal(self):
        source = (PACKAGE / "scripts" / "mission_manager.py").read_text(encoding="utf-8")
        self.assertIn("self.exploration_complete_sub = rospy.Subscriber", source)
        self.assertIn("goal = MoveBaseGoal()", source)
        self.assertIn("done_cb=self._return_done_callback", source)
        self.assertIn("self._entry_done_callback(", source)
        self.assertIn("self.entry_retry_at = rospy.Time.now()", source)
        self.assertIn('navigation_failure == "LOCALIZATION_LOST"', source)
        self.assertIn('"UNREACHABLE",', source)
        self.assertIn("self._classify_entry_failure(sequence, state)", source)
        self.assertIn("if entry_goal_active:", source)
        self.assertIn("self._entry_localization_ready(now)", source)
        self.assertIn("os.replace(temporary, self.result_file)", source)


if __name__ == "__main__":
    unittest.main()
