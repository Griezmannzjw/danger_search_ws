#!/usr/bin/env python3

import math
import unittest

from danger_search_localization.gazebo_truth import (
    GazeboTruthCore,
    normalize_angle,
    quaternion_yaw,
    relative_planar_pose,
)


class TestGazeboTruth(unittest.TestCase):
    def test_first_pose_is_origin(self):
        core = GazeboTruthCore()
        core.update(10.0, 0.0, -3.2, math.pi / 2.0)

        self.assertEqual(core.pose_at(10.0), (0.0, 0.0, 0.0))

    def test_world_motion_is_expressed_in_start_frame(self):
        core = GazeboTruthCore()
        core.update(10.0, 0.0, -3.2, math.pi / 2.0)
        core.update(10.1, -0.6, -1.5, math.pi / 2.0 + 0.25)

        x, y, yaw = core.pose_at(10.1)
        self.assertAlmostEqual(x, 1.7)
        self.assertAlmostEqual(y, 0.6)
        self.assertAlmostEqual(yaw, 0.25)

    def test_relative_pose_handles_nonzero_origin_yaw(self):
        pose = relative_planar_pose(
            (2.0, 3.0, math.pi / 2.0),
            (1.0, 5.0, -math.pi),
        )

        self.assertAlmostEqual(pose[0], 2.0)
        self.assertAlmostEqual(pose[1], 1.0)
        self.assertAlmostEqual(pose[2], math.pi / 2.0)

    def test_quaternion_is_normalized_before_yaw(self):
        yaw = quaternion_yaw(0.0, 0.0, 2.0 * math.sin(0.2), 2.0 * math.cos(0.2))

        self.assertAlmostEqual(yaw, 0.4)

    def test_angle_wrap(self):
        self.assertAlmostEqual(normalize_angle(3.0 * math.pi), math.pi)
        self.assertAlmostEqual(normalize_angle(-3.0 * math.pi), -math.pi)

    def test_stale_and_too_far_future_samples_are_rejected(self):
        core = GazeboTruthCore(max_age_s=0.2, max_future_s=0.05)
        core.update(10.0, 0.0, 0.0, 0.0)

        self.assertIsNone(core.pose_at(10.21))
        self.assertIsNone(core.pose_at(9.94))
        self.assertIsNotNone(core.pose_at(10.2))
        self.assertIsNotNone(core.pose_at(9.95))

    def test_time_rollback_reanchors_pose(self):
        core = GazeboTruthCore()
        core.update(10.0, 1.0, 2.0, 0.3)
        core.update(10.1, 2.0, 2.0, 0.3)
        self.assertGreater(core.pose_at(10.1)[0], 0.0)

        core.update(1.0, 7.0, 8.0, -0.5)

        self.assertEqual(core.pose_at(1.0), (0.0, 0.0, 0.0))

    def test_invalid_inputs_are_rejected(self):
        with self.assertRaises(ValueError):
            GazeboTruthCore(max_age_s=0.0)
        with self.assertRaises(ValueError):
            quaternion_yaw(0.0, 0.0, 0.0, 0.0)
        with self.assertRaises(ValueError):
            relative_planar_pose((0.0, 0.0, 0.0), (math.nan, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
