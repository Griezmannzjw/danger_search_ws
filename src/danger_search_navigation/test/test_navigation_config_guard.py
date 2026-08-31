#!/usr/bin/env python3

import os
import sys
import unittest


SCRIPT_DIR = os.path.join(os.path.dirname(__file__), "..", "scripts")
sys.path.insert(0, os.path.abspath(SCRIPT_DIR))

from navigation_config_guard import validate_dwa_velocity_domain


class NavigationConfigGuardTest(unittest.TestCase):
    @staticmethod
    def _safe_domain():
        return {
            "min_vel_trans": 0.30,
            "max_vel_trans": 0.30,
            "min_vel_x": 0.0,
            "max_vel_x": 0.30,
            "min_vel_y": 0.0,
            "max_vel_y": 0.0,
            "min_vel_theta": 0.40,
            "max_vel_theta": 0.40,
            "acc_lim_x": 3.0,
            "vx_samples": 2,
            "vth_samples": 9,
        }

    def test_accepts_nonholonomic_zero_including_sample_domain(self):
        self.assertIsNone(validate_dwa_velocity_domain(self._safe_domain()))

    def test_rejects_forced_high_speed_or_missing_zero_sample(self):
        forced = self._safe_domain()
        forced["min_vel_theta"] = forced["max_vel_theta"] = 0.80
        with self.assertRaisesRegex(ValueError, "exceeds"):
            validate_dwa_velocity_domain(forced)

        even_samples = self._safe_domain()
        even_samples["vth_samples"] = 6
        with self.assertRaisesRegex(ValueError, "odd >=9"):
            validate_dwa_velocity_domain(even_samples)

    def test_rejects_missing_rotation_or_non_discrete_forward_domain(self):
        no_rotation = self._safe_domain()
        no_rotation["min_vel_x"] = 0.30
        with self.assertRaisesRegex(ValueError, "zero-velocity rotation"):
            validate_dwa_velocity_domain(no_rotation)

        too_fast = self._safe_domain()
        too_fast["max_vel_trans"] = too_fast["max_vel_x"] = 0.40
        with self.assertRaisesRegex(ValueError, "ordinary navigation speed"):
            validate_dwa_velocity_domain(too_fast)

        intermediate_samples = self._safe_domain()
        intermediate_samples["vx_samples"] = 5
        with self.assertRaisesRegex(ValueError, "exactly two"):
            validate_dwa_velocity_domain(intermediate_samples)

    def test_rejects_acceleration_that_cannot_span_domain(self):
        slow_acceleration = self._safe_domain()
        slow_acceleration["acc_lim_x"] = 2.9
        with self.assertRaisesRegex(ValueError, "one control cycle"):
            validate_dwa_velocity_domain(slow_acceleration)

        with self.assertRaisesRegex(ValueError, "controller frequency"):
            validate_dwa_velocity_domain(
                self._safe_domain(), controller_frequency_hz=0.0
            )

    def test_rejects_holonomic_or_over_limit_domain(self):
        holonomic = self._safe_domain()
        holonomic["max_vel_y"] = 0.10
        with self.assertRaisesRegex(ValueError, "nonholonomic"):
            validate_dwa_velocity_domain(holonomic)

        over_limit = self._safe_domain()
        over_limit["max_vel_theta"] = 0.80
        with self.assertRaisesRegex(ValueError, "exceeds"):
            validate_dwa_velocity_domain(over_limit)

        invalid_minimum = self._safe_domain()
        invalid_minimum["min_vel_theta"] = -0.10
        with self.assertRaisesRegex(ValueError, "0 < min <= max"):
            validate_dwa_velocity_domain(invalid_minimum)

        inert_minimum = self._safe_domain()
        inert_minimum["min_vel_theta"] = 0.30
        with self.assertRaisesRegex(ValueError, "policy deadband"):
            validate_dwa_velocity_domain(inert_minimum)

        coarse_samples = self._safe_domain()
        coarse_samples["vth_samples"] = 7
        with self.assertRaisesRegex(ValueError, "odd >=9"):
            validate_dwa_velocity_domain(coarse_samples)

if __name__ == "__main__":
    unittest.main()
