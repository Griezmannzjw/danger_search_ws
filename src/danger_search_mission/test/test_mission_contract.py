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
        self.assertTrue(config["require_entrance_ready"])

    def test_manager_owns_complete_subscription_and_return_goal(self):
        source = (PACKAGE / "scripts" / "mission_manager.py").read_text(encoding="utf-8")
        self.assertIn("self.exploration_complete_sub = rospy.Subscriber", source)
        self.assertIn("goal = MoveBaseGoal()", source)
        self.assertIn("done_cb=self._return_done_callback", source)
        self.assertIn("done_cb=self._entry_done_callback", source)
        self.assertIn("os.replace(temporary, self.result_file)", source)


if __name__ == "__main__":
    unittest.main()
