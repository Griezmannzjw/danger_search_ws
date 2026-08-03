#!/usr/bin/env python3

import math
import unittest

from danger_search_localization.config import AdapterConfig
from danger_search_localization.vertical_estimation import (
    VerticalEstimator,
    quaternion_from_rpy,
    quaternion_inverse,
    quaternion_multiply,
    rotate_vector,
)


class TestVerticalEstimation(unittest.TestCase):
    def setUp(self):
        self.config = AdapterConfig(
            stationary_hold_s=0.02,
            vertical_integration_rate_hz=200.0,
        )

    def test_stationary_level_imu_stays_on_start_floor(self):
        estimator = VerticalEstimator(self.config)
        orientation = quaternion_from_rpy(0.0, 0.0, 0.0)
        for index in range(20):
            estimator.update(
                index * 0.01,
                orientation,
                (0.0, 0.0, 0.0),
                (0.0, 0.0, self.config.gravity_mps2),
            )

        state = estimator.snapshot()
        self.assertTrue(state.initialized)
        self.assertTrue(state.stationary)
        self.assertAlmostEqual(state.z, 0.0)
        self.assertEqual(state.current_floor, 0)

    def test_floor_state_uses_hysteresis(self):
        estimator = VerticalEstimator(self.config)
        estimator.z = 1.60
        estimator._update_floor()
        self.assertEqual(estimator.current_floor, 1)

        estimator.z = 1.20
        estimator._update_floor()
        self.assertEqual(estimator.current_floor, 1)

        estimator.z = 1.00
        estimator._update_floor()
        self.assertEqual(estimator.current_floor, 0)

    def test_sensor_extrinsic_rotation_can_be_removed(self):
        base_from_imu = quaternion_from_rpy(0.0, math.pi / 4.0, 0.0)
        world_from_base = quaternion_from_rpy(0.1, -0.2, 0.3)
        world_from_imu = quaternion_multiply(world_from_base, base_from_imu)

        recovered = quaternion_multiply(
            world_from_imu, quaternion_inverse(base_from_imu)
        )

        for actual, expected in zip(recovered, world_from_base):
            self.assertAlmostEqual(actual, expected)

    def test_vector_rotation_does_not_add_translation(self):
        rotated = rotate_vector(
            (1.0, 0.0, 0.0), quaternion_from_rpy(0.0, 0.0, math.pi / 2.0)
        )
        self.assertAlmostEqual(rotated[0], 0.0, places=7)
        self.assertAlmostEqual(rotated[1], 1.0, places=7)
        self.assertAlmostEqual(rotated[2], 0.0, places=7)

    def test_large_tilt_freezes_height_instead_of_drifting(self):
        estimator = VerticalEstimator(self.config)
        level = quaternion_from_rpy(0.0, 0.0, 0.0)
        estimator.update(0.0, level, (0, 0, 0), (0, 0, self.config.gravity_mps2))
        tilted = quaternion_from_rpy(0.0, 0.4, 0.0)
        estimator.update(0.01, tilted, (0, 0, 0), (5.0, 0.0, 0.0))
        self.assertEqual(estimator.snapshot().z, 0.0)
        self.assertEqual(estimator.snapshot().velocity_z, 0.0)


if __name__ == "__main__":
    unittest.main()
