#!/usr/bin/env python3

import math
import unittest
from types import SimpleNamespace

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid

from danger_search_common.msg import LocalizationStatus
from danger_search_localization.adapter_node import LocalizationAdapterNode
from danger_search_localization.config import AdapterConfig
from danger_search_localization.vertical_estimation import quaternion_to_rpy


class TestLocalizationAdapter(unittest.TestCase):
    def setUp(self):
        self.adapter = LocalizationAdapterNode.__new__(
            LocalizationAdapterNode
        )
        self.adapter.config = AdapterConfig()

    def test_empty_covariance_receives_conservative_fallback(self):
        pose = PoseWithCovarianceStamped()

        self.adapter._ensure_covariance(pose)

        self.assertEqual(
            pose.pose.covariance[0], self.adapter.config.fallback_xy_variance
        )
        self.assertEqual(
            pose.pose.covariance[7], self.adapter.config.fallback_xy_variance
        )
        self.assertEqual(
            pose.pose.covariance[35],
            self.adapter.config.fallback_yaw_variance,
        )
        for index in (14, 21, 28):
            self.assertEqual(
                pose.pose.covariance[index],
                self.adapter.config.fallback_unobserved_variance,
            )

    def test_backend_covariance_can_be_enabled_explicitly(self):
        self.adapter.config = AdapterConfig(use_backend_covariance=True)
        pose = PoseWithCovarianceStamped()
        pose.pose.covariance[0] = 0.01
        pose.pose.covariance[7] = 0.02
        pose.pose.covariance[35] = 0.03

        self.adapter._ensure_covariance(pose)

        self.assertEqual(pose.pose.covariance[0], 0.01)
        self.assertEqual(pose.pose.covariance[7], 0.02)
        self.assertEqual(pose.pose.covariance[35], 0.03)

    def test_map_checksum_changes_with_grid_content(self):
        grid = OccupancyGrid()
        grid.info.width = 2
        grid.info.height = 2
        grid.info.resolution = 0.05
        grid.data = [-1, 0, 0, 100]
        first = self.adapter._map_checksum(grid)

        grid.data[1] = 100

        self.assertNotEqual(first, self.adapter._map_checksum(grid))

    def test_tracking_state_reflects_freshness(self):
        pose = PoseWithCovarianceStamped()
        self.assertEqual(
            self.adapter._tracking_state(None, False, False),
            LocalizationStatus.STATE_INITIALIZING,
        )
        self.assertEqual(
            self.adapter._tracking_state(pose, False, True),
            LocalizationStatus.STATE_LOST,
        )
        self.assertEqual(
            self.adapter._tracking_state(pose, True, False),
            LocalizationStatus.STATE_DEGRADED,
        )
        self.assertEqual(
            self.adapter._tracking_state(pose, True, True),
            LocalizationStatus.STATE_TRACKING,
        )

    def test_vertical_state_adds_z_and_tilt_without_replacing_slam_yaw(self):
        self.adapter.config = AdapterConfig(vertical_estimation_enabled=True)
        pose = PoseWithCovarianceStamped()
        yaw = 0.7
        pose.pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.pose.orientation.w = math.cos(yaw / 2.0)
        vertical = SimpleNamespace(
            initialized=True,
            z=2.6,
            roll=0.1,
            pitch=-0.2,
        )

        self.adapter._apply_vertical_state(pose, vertical)

        result = pose.pose.pose.orientation
        roll, pitch, result_yaw = quaternion_to_rpy(
            (result.x, result.y, result.z, result.w)
        )
        self.assertAlmostEqual(pose.pose.pose.position.z, 2.6)
        self.assertAlmostEqual(roll, 0.1)
        self.assertAlmostEqual(pitch, -0.2)
        self.assertAlmostEqual(result_yaw, yaw)
