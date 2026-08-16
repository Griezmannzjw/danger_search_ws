#!/usr/bin/env python3

import math
import unittest
from collections import deque

import numpy as np

from danger_search_localization.config import ScanProjectionConfig
from danger_search_localization.scan_projection import (
    estimate_ground_clearance,
    gravity_level_points,
    merge_scan_history,
    project_planar_scan,
    quaternion_multiply,
    TimedScanAccumulator,
    transform_points,
)


class TestScanProjection(unittest.TestCase):
    def setUp(self):
        self.config = ScanProjectionConfig(
            angle_min=-math.pi,
            angle_max=math.pi,
            angle_increment=math.pi / 4.0,
            range_min=0.2,
            range_max=5.0,
            min_height=0.0,
            max_height=1.0,
            self_exclusion_min_x=-0.2,
            self_exclusion_max_x=0.2,
            self_exclusion_half_width_y=0.1,
            enable_isolated_hit_filter=False,
        )

    def test_nearest_supported_surface_wins_in_angular_bin(self):
        points = np.array(
            [
                [2.00, 0.0, 0.5],
                [2.04, 0.0, 0.5],
                [1.00, 0.0, 0.5],
                [1.02, 0.0, 0.5],
            ]
        )
        ranges = project_planar_scan(points, self.config)
        zero_bin = int((0.0 - self.config.angle_min) / self.config.angle_increment)
        self.assertAlmostEqual(float(ranges[zero_bin]), 1.01, places=5)

    def test_short_range_outlier_does_not_hide_supported_wall(self):
        points = np.array(
            [[0.5, 0.0, 0.5], [3.0, 0.0, 0.5], [3.04, 0.0, 0.5]]
        )

        ranges = project_planar_scan(points, self.config)

        zero_bin = int((0.0 - self.config.angle_min) / self.config.angle_increment)
        self.assertAlmostEqual(float(ranges[zero_bin]), 3.02, places=5)

    def test_height_and_range_filters_are_applied(self):
        points = np.array(
            [[1.0, 0.0, -0.1], [1.0, 0.0, 2.0], [10.0, 0.0, 0.5]]
        )
        self.assertTrue(np.isinf(project_planar_scan(points, self.config)).all())

    def test_rigid_transform_is_applied(self):
        half_yaw = math.pi / 4.0
        result = transform_points(
            [[1.0, 0.0, 0.0]],
            (1.0, 2.0, 0.0),
            (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)),
        )
        np.testing.assert_allclose(result, [[1.0, 3.0, 0.0]], atol=1e-7)

    def test_gravity_leveling_removes_roll_and_pitch_but_keeps_heading(self):
        roll = math.radians(20.0)
        pitch = math.radians(-15.0)
        yaw = math.radians(40.0)
        quaternion = self._quaternion_from_rpy(roll, pitch, yaw)
        point_heading = np.array([[2.0, 0.5, 0.3]])
        # Convert a heading-frame point back to body coordinates, then verify
        # that gravity leveling reconstructs it.
        yaw_quaternion = self._quaternion_from_rpy(0.0, 0.0, yaw)
        world_point = transform_points(point_heading, (0, 0, 0), yaw_quaternion)
        inverse = (-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3])
        body_point = transform_points(world_point, (0, 0, 0), inverse)

        levelled, measured_roll, measured_pitch = gravity_level_points(
            body_point, quaternion
        )

        np.testing.assert_allclose(levelled, point_heading, atol=1e-7)
        self.assertAlmostEqual(measured_roll, roll)
        self.assertAlmostEqual(measured_pitch, pitch)

    def test_sensor_extrinsic_can_be_removed_from_imu_orientation(self):
        world_from_base = self._quaternion_from_rpy(0.1, -0.2, 0.3)
        base_from_imu = self._quaternion_from_rpy(0.0, math.pi / 4.0, 0.0)
        world_from_imu = quaternion_multiply(world_from_base, base_from_imu)
        recovered = quaternion_multiply(
            world_from_imu,
            (-base_from_imu[0], -base_from_imu[1], -base_from_imu[2], base_from_imu[3]),
        )
        np.testing.assert_allclose(recovered, world_from_base, atol=1e-7)

    def test_robot_self_returns_are_rejected(self):
        points = np.array([[0.1, 0.0, 0.5], [0.1, 0.0, 0.5]])

        ranges = project_planar_scan(points, self.config)

        self.assertTrue(np.isinf(ranges).all())

    def test_ground_clearance_detects_collapsed_sensor_height(self):
        points = np.array(
            [[0.5 + index * 0.01, 0.0, -0.10] for index in range(40)]
        )
        self.assertAlmostEqual(
            estimate_ground_clearance(points, self.config), 0.10
        )

    def test_single_return_bin_is_rejected(self):
        points = np.array([[1.0, 0.0, 0.5]])

        ranges = project_planar_scan(points, self.config)

        self.assertTrue(np.isinf(ranges).all())

    def test_isolated_hit_is_rejected_but_continuous_surface_remains(self):
        config = ScanProjectionConfig(
            angle_min=-math.pi,
            angle_max=math.pi,
            angle_increment=math.pi / 8.0,
            range_min=0.2,
            range_max=5.0,
            min_height=0.0,
            max_height=1.0,
            self_exclusion_min_x=-0.1,
            self_exclusion_max_x=0.1,
            self_exclusion_half_width_y=0.1,
            min_returns_per_bin=1,
            neighbor_window_bins=2,
            max_neighbor_range_jump=0.5,
        )
        points = np.array(
            [
                [1.0, 0.0, 0.5],
                [0.921, 0.389, 0.5],
                [-3.0, 0.0, 0.5],
            ]
        )

        ranges = project_planar_scan(points, config)

        self.assertEqual(int(np.isfinite(ranges).sum()), 2)

    def test_isolated_filter_connects_across_full_scan_seam(self):
        config = ScanProjectionConfig(
            angle_min=-math.pi,
            angle_max=math.pi,
            angle_increment=math.pi / 8.0,
            range_min=0.2,
            range_max=5.0,
            min_height=0.0,
            max_height=1.0,
            self_exclusion_min_x=-0.1,
            self_exclusion_max_x=0.1,
            self_exclusion_half_width_y=0.1,
            min_returns_per_bin=1,
            neighbor_window_bins=2,
            max_neighbor_range_jump=0.5,
        )
        points = np.array(
            [
                [math.cos(-math.pi + 0.01), math.sin(-math.pi + 0.01), 0.5],
                [math.cos(math.pi - 0.01), math.sin(math.pi - 0.01), 0.5],
            ]
        )

        ranges = project_planar_scan(points, config)

        self.assertEqual(int(np.isfinite(ranges).sum()), 2)

    def test_scan_history_uses_median_after_minimum_observations(self):
        history = [
            np.array([1.0, np.inf, 4.0], dtype=np.float32),
            np.array([3.0, 6.0, np.inf], dtype=np.float32),
            np.array([5.0, 8.0, 2.0], dtype=np.float32),
        ]

        merged = merge_scan_history(history, min_samples_per_bin=2)

        np.testing.assert_allclose(merged, [3.0, 7.0, 3.0])

    def test_scan_history_rejects_single_frame_returns(self):
        history = [
            np.array([1.0, np.inf], dtype=np.float32),
            np.array([np.inf, 2.0], dtype=np.float32),
        ]

        merged = merge_scan_history(history, min_samples_per_bin=2)

        self.assertTrue(np.isinf(merged).all())

    def test_scan_history_uses_only_the_configured_window(self):
        history = deque(maxlen=2)
        history.append(np.array([1.0], dtype=np.float32))
        history.append(np.array([3.0], dtype=np.float32))
        history.append(np.array([5.0], dtype=np.float32))

        merged = merge_scan_history(history, min_samples_per_bin=2)

        np.testing.assert_allclose(merged, [4.0])

    def test_scan_history_requires_valid_minimum_sample_count(self):
        with self.assertRaises(ValueError):
            merge_scan_history([np.array([1.0])], min_samples_per_bin=0)

    def test_timed_scan_history_resets_after_long_gap(self):
        accumulator = TimedScanAccumulator(max_frames=5, max_age_s=0.6)
        self.assertFalse(accumulator.add(1.0, [1.0, np.inf]))
        self.assertFalse(accumulator.add(1.1, [1.1, np.inf]))

        reset = accumulator.add(2.0, [2.0, np.inf])

        self.assertTrue(reset)
        self.assertEqual(len(accumulator), 1)
        np.testing.assert_allclose(accumulator.scans[0][0], 2.0)

    def test_timed_scan_history_resets_when_clock_moves_backwards(self):
        accumulator = TimedScanAccumulator(max_frames=5, max_age_s=0.6)
        accumulator.add(2.0, [2.0])

        self.assertTrue(accumulator.add(1.0, [1.0]))
        self.assertEqual(len(accumulator), 1)

    @staticmethod
    def _quaternion_from_rpy(roll, pitch, yaw):
        cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
        cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )
