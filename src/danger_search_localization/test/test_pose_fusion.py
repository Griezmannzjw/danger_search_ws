#!/usr/bin/env python3

import math
import unittest

from danger_search_localization.config import AdapterConfig
from danger_search_localization.pose_fusion import HectorGicpFusion


class TestHectorGicpFusion(unittest.TestCase):
    def setUp(self):
        self.fusion = HectorGicpFusion(AdapterConfig())

    def _initialize(self):
        self.fusion.update_local(1.0, 0.0, 0.0, 0.0)
        result = self.fusion.update_global(1.0, 0.0, 0.0, 0.0)
        self.assertTrue(result.accepted)

    def test_matching_origins_initialize_identity_correction(self):
        self._initialize()
        result = self.fusion.snapshot()
        self.assertTrue(result.initialized)
        self.assertAlmostEqual(result.correction.x, 0.0)
        self.assertAlmostEqual(result.pose.x, 0.0)

    def test_local_motion_remains_continuous_without_new_hector_pose(self):
        self._initialize()
        result = self.fusion.update_local(1.1, 0.08, 0.0, 0.0)
        self.assertTrue(result.accepted)
        self.assertAlmostEqual(result.pose.x, 0.08)

    def test_stationary_hector_drift_is_rejected(self):
        self._initialize()
        self.fusion.update_local(1.1, 0.0, 0.0, 0.0)
        result = self.fusion.update_global(1.1, 2.0, 0.0, 0.0)
        self.assertFalse(result.accepted)
        self.assertEqual(
            result.reason, "HECTOR_DRIFT_WHILE_LOCAL_ODOMETRY_STATIONARY"
        )
        self.assertAlmostEqual(result.pose.x, 0.0)

    def test_small_stationary_hector_noise_cannot_accumulate(self):
        self._initialize()
        result = None
        for index in range(1, 101):
            stamp = 1.0 + index * 0.1
            self.fusion.update_local(stamp, 0.0, 0.0, 0.0)
            result = self.fusion.update_global(
                stamp, 0.01, -0.01, 0.005
            )
            self.assertTrue(result.accepted)
        self.assertAlmostEqual(result.correction.x, 0.0)
        self.assertAlmostEqual(result.correction.y, 0.0)
        self.assertAlmostEqual(result.correction.yaw, 0.0)

    def test_hector_jump_does_not_teleport_moving_local_pose(self):
        self._initialize()
        self.fusion.update_local(1.1, 0.10, 0.0, 0.0)
        result = self.fusion.update_global(1.1, 2.10, 0.0, 0.0)
        self.assertFalse(result.accepted)
        self.assertAlmostEqual(result.pose.x, 0.10)

    def test_small_correction_is_applied_gradually(self):
        self._initialize()
        self.fusion.update_local(1.5, 0.20, 0.0, 0.0)
        result = self.fusion.update_global(1.5, 0.25, 0.0, 0.0)
        self.assertTrue(result.accepted)
        self.assertGreater(result.pose.x, 0.20)
        self.assertLess(result.pose.x, 0.25)

    def test_unsynchronized_hector_pose_is_rejected(self):
        self._initialize()
        result = self.fusion.update_global(5.0, 0.0, 0.0, 0.0)
        self.assertFalse(result.accepted)
        self.assertEqual(
            result.reason, "HECTOR_POSE_HAS_NO_SYNCHRONIZED_LOCAL_POSE"
        )

    def test_non_increasing_hector_stamp_is_rejected_without_rewinding_clock(self):
        self._initialize()
        self.fusion.update_local(1.1, 0.1, 0.0, 0.0)
        accepted = self.fusion.update_global(1.1, 0.1, 0.0, 0.0)
        self.assertTrue(accepted.accepted)

        stale = self.fusion.update_global(1.05, 0.1, 0.0, 0.0)

        self.assertFalse(stale.accepted)
        self.assertEqual(stale.reason, "NON_INCREASING_HECTOR_POSE_STAMP")
        self.assertAlmostEqual(self.fusion.last_global_stamp_s, 1.1)

    def test_initial_large_offset_is_rejected(self):
        self.fusion.update_local(1.0, 0.0, 0.0, 0.0)
        result = self.fusion.update_global(1.0, 3.0, 0.0, math.pi / 2.0)
        self.assertFalse(result.accepted)
        self.assertFalse(result.initialized)

    def test_huge_finite_local_pose_is_rejected_without_polluting_state(self):
        self._initialize()
        before = self.fusion.latest_local

        with self.assertRaises(ValueError):
            self.fusion.update_local(1.1, 1e180, -1e180, 0.0)

        self.assertEqual(self.fusion.latest_local, before)


if __name__ == "__main__":
    unittest.main()
