#!/usr/bin/env python3

import math
import unittest

from danger_search_mission.entry_core import (
    CrossingReference,
    ProgressWatchdog,
    YawErrorFilter,
    crossing_command,
    margin_avoidance_command,
    normalize_angle,
    quaternion_roll_pitch,
    rolling_sweep_distance,
)


class EntryCoreTest(unittest.TestCase):
    def test_crossing_reference_is_a_local_frame_not_task_home(self):
        reference = CrossingReference(10.0, -4.0, math.pi / 2.0)
        forward, lateral, yaw_error = reference.errors(9.8, -3.5, math.pi / 2.0 + 0.1)
        self.assertAlmostEqual(forward, 0.5, places=6)
        self.assertAlmostEqual(lateral, 0.2, places=6)
        self.assertAlmostEqual(yaw_error, 0.1, places=6)
        start_forward, start_lateral, start_yaw_error = reference.errors(
            10.0, -4.0, math.pi / 2.0
        )
        self.assertEqual((start_forward, start_lateral, start_yaw_error), (0.0, 0.0, 0.0))

    def test_yaw_filter_deadband_and_low_pass_prevent_jitter_reversal(self):
        heading_filter = YawErrorFilter(deadband_rad=0.05, alpha=0.25)
        self.assertEqual(heading_filter.update(0.04), 0.0)
        self.assertEqual(heading_filter.update(-0.04), 0.0)
        first = heading_filter.update(0.10)
        self.assertAlmostEqual(first, 0.0125, places=6)
        second = heading_filter.update(0.10)
        self.assertGreater(second, first)
        # One opposite noisy sample cannot immediately reverse the command.
        self.assertGreater(heading_filter.update(-0.10), 0.0)
        self.assertEqual(heading_filter.update(0.01), 0.0)

    def test_yaw_filter_rejects_invalid_configuration(self):
        with self.assertRaises(ValueError):
            YawErrorFilter(deadband_rad=-0.01, alpha=0.25)
        with self.assertRaises(ValueError):
            YawErrorFilter(deadband_rad=0.05, alpha=0.0)

    def test_heading_and_lateral_feedback_are_bounded(self):
        speed, angular = crossing_command(
            lateral_error_m=0.12,
            yaw_error_rad=math.radians(6.0),
            speed_mps=0.4,
            yaw_gain=1.5,
            lateral_gain=0.8,
            max_angular_speed_rps=0.3,
        )
        self.assertEqual(speed, 0.4)
        self.assertLess(angular, 0.0)
        self.assertGreaterEqual(angular, -0.3)
        _, saturated = crossing_command(1.0, 1.0, 0.4, 1.5, 0.8, 0.3)
        self.assertEqual(saturated, -0.3)

    def test_crossing_command_rejects_unsafe_configuration(self):
        with self.assertRaises(ValueError):
            crossing_command(0.0, 0.0, 0.0, 1.5, 0.8, 0.3)
        with self.assertRaises(ValueError):
            crossing_command(0.0, float("nan"), 0.4, 1.5, 0.8, 0.3)

    def test_lateral_margin_avoidance_slows_and_turns_away(self):
        left_wall = margin_avoidance_command(
            0.4, 0.04, 0.185, 0.22, 0.15, 0.30
        )
        self.assertEqual(left_wall[0], 0.22)
        self.assertAlmostEqual(left_wall[1], -0.11)
        right_wall = margin_avoidance_command(
            0.4, -0.04, -0.185, 0.22, 0.15, 0.30
        )
        self.assertEqual(right_wall[0], 0.22)
        self.assertAlmostEqual(right_wall[1], 0.11)
        self.assertEqual(
            margin_avoidance_command(0.2, -0.25, 0.185, 0.22, 0.15, 0.30),
            (0.2, -0.30),
        )
        self.assertEqual(
            margin_avoidance_command(0.4, 0.05, 0.0, 0.25, 0.08, 0.30),
            (0.25, 0.05),
        )
        self.assertEqual(
            margin_avoidance_command(0.4, 0.20, 0.185, 0.25, 0.08, 0.30),
            (0.25, -0.08),
        )

    def test_margin_avoidance_rejects_center_or_invalid_configuration(self):
        with self.assertRaises(ValueError):
            margin_avoidance_command(0.4, 0.0, 0.18, 0.0, 0.15, 0.30)

    def test_quaternion_roll_pitch_and_angle_normalization(self):
        half = math.radians(10.0) / 2.0
        roll, pitch = quaternion_roll_pitch(math.sin(half), 0.0, 0.0, math.cos(half))
        self.assertAlmostEqual(roll, math.radians(10.0), places=6)
        self.assertAlmostEqual(pitch, 0.0, places=6)
        self.assertAlmostEqual(normalize_angle(3.0 * math.pi), math.pi, places=6)

    def test_progress_watchdog_resets_only_after_real_progress(self):
        watchdog = ProgressWatchdog(0.03, 6.0)
        watchdog.reset(0.0, 10.0)
        self.assertTrue(watchdog.update(0.02, 15.9))
        self.assertFalse(watchdog.update(0.02, 16.1))
        watchdog.reset(0.0, 20.0)
        self.assertTrue(watchdog.update(0.04, 25.0))
        self.assertTrue(watchdog.update(0.04, 30.9))
        self.assertFalse(watchdog.update(0.04, 31.1))

    def test_rolling_sweep_uses_local_horizon_then_remaining_distance(self):
        self.assertEqual(rolling_sweep_distance(3.4, 0.8), 0.8)
        self.assertEqual(rolling_sweep_distance(0.35, 0.8), 0.35)
        self.assertEqual(rolling_sweep_distance(0.0, 0.8), 0.0)
        with self.assertRaises(ValueError):
            rolling_sweep_distance(1.0, 0.0)
        with self.assertRaises(ValueError):
            rolling_sweep_distance(-0.1, 0.8)


if __name__ == "__main__":
    unittest.main()
