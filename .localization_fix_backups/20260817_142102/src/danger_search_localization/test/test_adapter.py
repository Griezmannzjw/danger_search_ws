#!/usr/bin/env python3

import math
import threading
import unittest
from unittest import mock
from types import SimpleNamespace

import rospy
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
        self.adapter.map_frame = "map"
        self.adapter.use_hector_correction = True

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

    def test_public_map_requires_fresh_gicp_and_hector(self):
        self.adapter.lock = threading.RLock()
        self.adapter.pose_fusion = SimpleNamespace(initialized=True)
        self.adapter.last_hector_update_accepted = True
        self.adapter.latest_raw_map = OccupancyGrid()
        self.adapter.last_gicp_pose_accepted = rospy.Time(0)
        self.adapter.map_version = 0
        self.adapter.last_map_stamp = rospy.Time(0)
        with mock.patch.object(rospy, "logwarn_throttle"):
            self.adapter._publish_cached_map_if_safe(rospy.Time.from_sec(10.0))
        self.assertEqual(self.adapter.map_version, 0)

    def test_map_callback_keeps_received_grid_without_deep_copy(self):
        self.adapter.lock = threading.RLock()
        self.adapter.latest_raw_map = None
        self.adapter.last_map_received = rospy.Time(0)
        message = OccupancyGrid()
        message.header.frame_id = "map"

        with mock.patch.object(rospy.Time, "now", return_value=rospy.Time.from_sec(1.0)), \
                mock.patch.object(self.adapter, "_publish_cached_map_if_safe"):
            self.adapter._map_callback(message)

        self.assertIs(self.adapter.latest_raw_map, message)

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
            self.adapter._tracking_state(
                pose, True, True, degraded=True
            ),
            LocalizationStatus.STATE_DEGRADED,
        )
        self.assertEqual(
            self.adapter._tracking_state(pose, True, True, lost=True),
            LocalizationStatus.STATE_LOST,
        )

    def test_high_gicp_covariance_is_unhealthy(self):
        pose = PoseWithCovarianceStamped()
        pose.pose.covariance[0] = 0.01
        pose.pose.covariance[7] = 0.01
        pose.pose.covariance[35] = 0.02
        self.assertTrue(self.adapter._gicp_covariance_healthy(pose))

        pose.pose.covariance[0] = 10.0
        self.assertFalse(self.adapter._gicp_covariance_healthy(pose))

    def test_unhealthy_gicp_pose_gets_large_public_covariance(self):
        pose = PoseWithCovarianceStamped()
        self.adapter._set_output_covariance(pose, healthy=False)
        for index in (0, 7, 14, 21, 28, 35):
            self.assertEqual(
                pose.pose.covariance[index],
                self.adapter.config.gicp_unhealthy_variance_threshold,
            )
        self.assertEqual(
            self.adapter._tracking_state(pose, True, True),
            LocalizationStatus.STATE_TRACKING,
        )

    def test_status_reason_preserves_hector_rejection_reason(self):
        pose = PoseWithCovarianceStamped()

        reason = self.adapter._status_reason(
            pose,
            pose_fresh=True,
            map_fresh=True,
            stable=False,
            hector_degraded=True,
            gicp_fusion_reason="TRACKING_FUSED_POSE",
            hector_fusion_reason="HECTOR_CORRECTION_TRANSLATION_JUMP",
        )

        self.assertEqual(
            reason,
            "HECTOR_CORRECTION_DEGRADED:HECTOR_CORRECTION_TRANSLATION_JUMP",
        )

    def test_gicp_lost_reason_takes_priority(self):
        pose = PoseWithCovarianceStamped()
        reason = self.adapter._status_reason(
            pose,
            pose_fresh=True,
            map_fresh=True,
            stable=False,
            gicp_degraded=True,
            gicp_lost=True,
            gicp_fusion_reason="GICP_TRACKING_LOST",
        )
        self.assertEqual(reason, "GICP_ODOMETRY_LOST:GICP_TRACKING_LOST")

    def test_gicp_lost_tracking_state_is_lost(self):
        pose = PoseWithCovarianceStamped()
        self.assertEqual(
            self.adapter._tracking_state(
                pose, True, True, degraded=True, lost=True
            ),
            LocalizationStatus.STATE_LOST,
        )

    def test_non_hector_tracking_reason_describes_local_map(self):
        pose = PoseWithCovarianceStamped()
        reason = self.adapter._status_reason(
            pose,
            pose_fresh=True,
            map_fresh=True,
            stable=True,
            use_hector_correction=False,
        )
        self.assertEqual(
            reason, "TRACKING_GICP_ODOMETRY_WITH_LOCAL_OCCUPANCY_MAP"
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
