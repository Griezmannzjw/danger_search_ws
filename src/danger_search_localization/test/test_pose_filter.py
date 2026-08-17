#!/usr/bin/env python3

import math
import unittest

from danger_search_localization.config import AdapterConfig
from danger_search_localization.pose_filter import PoseStabilizer


class TestPoseStabilizer(unittest.TestCase):
    def setUp(self):
        self.filter = PoseStabilizer(AdapterConfig())

    def test_first_pose_is_mission_origin(self):
        result = self.filter.update(1.0, 12.0, -4.0, 1.2)
        self.assertTrue(result.accepted)
        self.assertEqual(result.pose.x, 0.0)
        self.assertEqual(result.pose.y, 0.0)
        self.assertEqual(result.pose.yaw, 0.0)

    def test_stationary_backend_jitter_is_suppressed(self):
        self.filter.update(1.0, 2.0, 3.0, 0.5)
        for index, jitter in enumerate((0.004, -0.006, 0.009, -0.003), 1):
            result = self.filter.update(
                1.0 + 0.1 * index,
                2.0 + jitter,
                3.0 - jitter,
                0.5 + jitter * 0.2,
            )
            self.assertTrue(result.accepted)
            self.assertAlmostEqual(result.pose.x, 0.0)
            self.assertAlmostEqual(result.pose.y, 0.0)
            self.assertAlmostEqual(result.pose.yaw, 0.0)

    def test_gradual_real_motion_is_followed(self):
        self.filter.update(1.0, 0.0, 0.0, 0.0)
        result = None
        for index in range(1, 11):
            result = self.filter.update(1.0 + index * 0.1, index * 0.02, 0.0, 0.0)
            self.assertTrue(result.accepted)
        self.assertGreater(result.pose.x, 0.15)
        self.assertLessEqual(result.pose.x, 0.20)

    def test_impossible_translation_jump_is_rejected_and_held(self):
        self.filter.update(1.0, 0.0, 0.0, 0.0)
        result = self.filter.update(1.1, 5.0, -10.0, 0.0)
        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "RAW_POSE_TRANSLATION_JUMP")
        self.assertEqual(result.pose.x, 0.0)
        self.assertEqual(result.pose.y, 0.0)

    def test_rejected_jump_cannot_teleport_later_output(self):
        self.filter.update(1.0, 0.0, 0.0, 0.0)
        rejected = self.filter.update(1.1, 5.0, 0.0, 0.0)
        self.assertFalse(rejected.accepted)

        recovered = self.filter.update(1.2, 0.02, 0.0, 0.0)
        self.assertTrue(recovered.accepted)
        self.assertLessEqual(recovered.pose.x, 0.02)
        self.assertEqual(recovered.consecutive_rejections, 0)

    def test_filter_has_no_unbounded_snap_recovery(self):
        self.assertFalse(hasattr(self.filter, "recover"))

    def test_impossible_yaw_jump_is_rejected(self):
        self.filter.update(1.0, 0.0, 0.0, 0.0)
        result = self.filter.update(1.1, 0.0, 0.0, math.pi)
        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "RAW_POSE_YAW_JUMP")

    def test_anchor_rotation_is_applied_to_position(self):
        self.filter.update(1.0, 5.0, 8.0, math.pi / 2.0)
        result = self.filter.update(1.5, 5.0, 8.2, math.pi / 2.0)
        self.assertTrue(result.accepted)
        self.assertGreater(result.pose.x, 0.15)
        self.assertAlmostEqual(result.pose.y, 0.0, places=6)

    def test_nonfinite_pose_is_rejected(self):
        result = self.filter.update(1.0, float("nan"), 0.0, 0.0)
        self.assertFalse(result.accepted)
        self.assertFalse(result.initialized)


if __name__ == "__main__":
    unittest.main()
