#!/usr/bin/env python3

import math
import threading
import unittest
from unittest import mock
from types import SimpleNamespace

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid

from danger_search_common.msg import FloorOccupancyGrid, LocalizationStatus
from danger_search_localization.adapter_node import (
    LocalizationAdapterNode,
    LocalVelocityEstimator,
)
from danger_search_localization.config import AdapterConfig
from danger_search_localization.floor_mapping import (
    FloorHeightClassifier,
    FloorSwitchState,
)
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

    def test_local_velocity_is_expressed_in_base_frame(self):
        estimator = LocalVelocityEstimator(max_dt_s=0.5)
        estimator.update(1.0, SimpleNamespace(x=0.0, y=0.0, yaw=math.pi / 2.0))

        velocity, degraded = estimator.update(
            1.2, SimpleNamespace(x=0.0, y=0.10, yaw=math.pi / 2.0)
        )

        self.assertFalse(degraded)
        self.assertAlmostEqual(velocity[0], 0.5, places=6)
        self.assertAlmostEqual(velocity[1], 0.0, places=6)
        self.assertAlmostEqual(velocity[2], 0.0, places=6)

    def test_local_velocity_rejects_nonincreasing_and_stale_intervals(self):
        estimator = LocalVelocityEstimator(max_dt_s=0.5)
        pose = SimpleNamespace(x=0.0, y=0.0, yaw=0.0)
        estimator.update(2.0, pose)

        velocity, degraded = estimator.update(2.0, pose)
        self.assertTrue(degraded)
        self.assertEqual(velocity, (0.0, 0.0, 0.0))

        velocity, degraded = estimator.update(
            3.0, SimpleNamespace(x=0.5, y=0.0, yaw=0.0)
        )
        self.assertTrue(degraded)
        self.assertEqual(velocity, (0.0, 0.0, 0.0))

        velocity, degraded = estimator.update(
            3.2, SimpleNamespace(x=0.6, y=0.0, yaw=0.0)
        )
        self.assertFalse(degraded)
        self.assertAlmostEqual(velocity[0], 0.5, places=6)

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
        self.adapter.latest_raw_map_identity = None
        self.adapter.last_map_received = rospy.Time(0)
        self.adapter.last_map_load_time = rospy.Time(0)
        self.adapter.current_floor = 0
        self.adapter.map_epoch = 1
        self.adapter.map_version = 0
        self.adapter.last_map_stamp = rospy.Time(0)
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

    def test_pose_guard_rejection_has_specific_degraded_reason(self):
        pose = PoseWithCovarianceStamped()
        reason = self.adapter._status_reason(
            pose,
            pose_fresh=True,
            map_fresh=True,
            stable=False,
            gicp_degraded=True,
            pose_guard_degraded=True,
            pose_guard_reason="RAW_POSE_TRANSLATION_JUMP",
        )
        self.assertEqual(
            reason,
            "POSE_GUARD_DEGRADED_HOLDING_LAST_POSE:"
            "RAW_POSE_TRANSLATION_JUMP",
        )

    def test_pose_guard_lost_reason_takes_priority(self):
        pose = PoseWithCovarianceStamped()
        reason = self.adapter._status_reason(
            pose,
            pose_fresh=True,
            map_fresh=True,
            stable=False,
            gicp_degraded=True,
            gicp_lost=True,
            pose_guard_degraded=True,
            pose_guard_lost=True,
            pose_guard_reason="RAW_POSE_YAW_JUMP",
        )
        self.assertEqual(reason, "POSE_GUARD_LOST:RAW_POSE_YAW_JUMP")

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

    def test_gazebo_truth_tracking_reason_does_not_claim_gicp(self):
        pose = PoseWithCovarianceStamped()
        reason = self.adapter._status_reason(
            pose,
            pose_fresh=True,
            map_fresh=True,
            stable=True,
            use_hector_correction=False,
            localization_source="gazebo_truth",
        )
        self.assertEqual(
            reason, "TRACKING_GAZEBO_TRUTH_WITH_LOCAL_OCCUPANCY_MAP"
        )

    def test_gazebo_truth_stale_reason_does_not_claim_scan_matching(self):
        pose = PoseWithCovarianceStamped()
        reason = self.adapter._status_reason(
            pose,
            pose_fresh=False,
            map_fresh=True,
            stable=False,
            localization_source="gazebo_truth",
        )
        self.assertEqual(reason, "GAZEBO_TRUTH_POSE_STALE")

    def test_floor_transition_has_explicit_degraded_reason(self):
        pose = PoseWithCovarianceStamped()

        reason = self.adapter._status_reason(
            pose,
            pose_fresh=True,
            map_fresh=True,
            stable=False,
            localization_source="gazebo_truth",
            floor_transition_active=True,
        )

        self.assertEqual(
            reason, "FLOOR_TRANSITION_WAITING_FOR_CURRENT_MAP"
        )

    def test_test_only_height_transition_requires_explicit_opt_in(self):
        self.adapter.multifloor_enabled = True
        self.adapter.floor_classifier = FloorHeightClassifier(
            [0.0, 2.6, 5.2], assignment_tolerance_m=0.45
        )
        self.adapter.lock = threading.RLock()
        self.adapter.current_floor = 0
        self.adapter.current_height = 0.0
        self.adapter.floor_transition_active = False
        self.adapter.floor_transition_baseline_version = 0
        self.adapter.floor_map_versions = {0: 8}
        self.adapter.map_update_count = 8
        self.adapter.map_version = 8
        self.adapter.last_map_stamp = rospy.Time.from_sec(1.0)
        self.adapter.last_map_received = rospy.Time.from_sec(1.0)
        self.adapter.last_public_map_published = rospy.Time.from_sec(1.0)
        self.adapter.last_map_update = rospy.Time.from_sec(1.0)
        self.adapter.latest_raw_map = OccupancyGrid()
        self.adapter.latest_raw_map_identity = (0, 1, 8)
        self.adapter.current_floor_pub = mock.Mock()
        self.adapter.allow_pose_height_floor_assignment = True
        self.adapter.explicit_floor_switching = False

        self.adapter._observe_floor_height(1.3)
        self.assertTrue(self.adapter.floor_transition_active)
        self.assertEqual(self.adapter.current_floor, 0)
        self.adapter._observe_floor_height(2.6)

        self.assertEqual(self.adapter.current_floor, 1)
        self.assertIsNone(self.adapter.latest_raw_map)
        self.assertIsNone(self.adapter.latest_raw_map_identity)
        self.assertEqual(self.adapter.map_update_count, 0)
        self.adapter.current_floor_pub.publish.assert_called_once()
        published = self.adapter.current_floor_pub.publish.call_args.args[0]
        self.assertEqual(published.data, 1)

    def test_truth_height_cannot_change_floor_without_switch_service(self):
        self.adapter.multifloor_enabled = True
        self.adapter.floor_classifier = FloorHeightClassifier(
            [0.0, 2.6, 5.2], assignment_tolerance_m=0.45
        )
        self.adapter.lock = threading.RLock()
        self.adapter.current_floor = 0
        self.adapter.current_height = 0.0
        self.adapter.floor_transition_active = False
        self.adapter.explicit_floor_switching = True
        self.adapter.allow_pose_height_floor_assignment = False
        self.adapter.current_floor_pub = mock.Mock()

        self.adapter._observe_floor_height(2.6)

        self.assertEqual(self.adapter.current_floor, 0)
        self.assertEqual(self.adapter.current_height, 0.0)
        self.adapter.current_floor_pub.publish.assert_not_called()

    def test_explicit_floor_switch_retries_do_not_repeat_dependencies(self):
        self.adapter.multifloor_enabled = True
        self.adapter.lock = threading.RLock()
        self.adapter.current_floor = 0
        self.adapter.current_height = 0.0
        self.adapter.floor_switch_state = FloorSwitchState([0.0, 2.6, 5.2])
        self.adapter.map_epoch = 1
        self.adapter.floor_transition_active = False
        self.adapter.floor_transition_baseline_version = 0
        self.adapter.floor_map_versions = {0: 8}
        self.adapter.map_version = 8
        self.adapter.map_update_count = 8
        self.adapter.last_map_stamp = rospy.Time.from_sec(1.0)
        self.adapter.last_map_received = rospy.Time.from_sec(1.0)
        self.adapter.last_public_map_published = rospy.Time.from_sec(1.0)
        self.adapter.last_map_update = rospy.Time.from_sec(1.0)
        self.adapter.latest_raw_map = OccupancyGrid()
        self.adapter.latest_raw_map_identity = (0, 1, 8)
        self.adapter.floor_switch_service_timeout_s = 0.1
        self.adapter.gicp_rebaseline_service_name = "/gicp/rebaseline"
        self.adapter.mapper_switch_floor_service_name = "/mapper/switch_floor"
        self.adapter.gicp_rebaseline = mock.Mock(
            return_value=SimpleNamespace(success=True, message="scheduled")
        )
        self.adapter.mapper_switch_floor = mock.Mock(
            return_value=SimpleNamespace(
                success=True, map_epoch=2, message="switched"
            )
        )
        self.adapter.current_floor_pub = mock.Mock()
        request = SimpleNamespace(
            transition_id="elevator-run-1", target_floor=1
        )

        with mock.patch.object(rospy, "wait_for_service"):
            first = self.adapter._switch_floor_callback(request)
            replay = self.adapter._switch_floor_callback(request)

        self.assertTrue(first.success)
        self.assertTrue(replay.success)
        self.assertEqual(first.map_epoch, 2)
        self.assertEqual(replay.map_epoch, 2)
        self.assertEqual(self.adapter.current_floor, 1)
        self.assertEqual(self.adapter.current_height, 2.6)
        self.assertTrue(self.adapter.floor_transition_active)
        self.assertEqual(self.adapter.gicp_rebaseline.call_count, 1)
        self.assertEqual(self.adapter.mapper_switch_floor.call_count, 1)

    def test_floor_map_restores_only_after_fresh_updates(self):
        self.adapter.config = AdapterConfig(min_map_updates_for_stable=2)
        self.adapter.multifloor_enabled = True
        self.adapter.lock = threading.RLock()
        self.adapter.map_frame = "map"
        self.adapter.current_floor = 1
        self.adapter.map_epoch = 2
        self.adapter.floor_transition_active = True
        self.adapter.floor_transition_baseline_version = 5
        self.adapter.floor_map_versions = {0: 8, 1: 5}
        self.adapter.floor_map_epochs = {0: 1, 1: 2}
        self.adapter.floor_map_last_updates = {
            0: rospy.Time.from_sec(8.0)
        }
        self.adapter.floor_last_seen_versions = {(0, 1): 8, (1, 2): 5}
        self.adapter.floor_map_load_times = {}
        self.adapter.last_map_received = rospy.Time(0)
        self.adapter.last_map_load_time = rospy.Time(0)
        self.adapter.latest_raw_map = None
        self.adapter.map_version = 5
        self.adapter.map_update_count = 0
        self.adapter.last_map_update = rospy.Time(0)

        def envelope(version, stamp):
            message = FloorOccupancyGrid()
            message.floor_id = 1
            message.map_epoch = 2
            message.map_version = version
            message.occupancy_grid.header.frame_id = "map"
            message.occupancy_grid.header.stamp = rospy.Time.from_sec(stamp)
            message.occupancy_grid.info.map_load_time = rospy.Time.from_sec(1.0)
            return message

        with mock.patch.object(
            rospy.Time, "now", return_value=rospy.Time.from_sec(20.0)
        ), mock.patch.object(
            self.adapter, "_publish_cached_map_if_safe"
        ) as publish:
            self.adapter._floor_map_callback(envelope(6, 10.0))
            self.assertTrue(self.adapter.floor_transition_active)
            self.assertEqual(self.adapter.map_update_count, 1)

            self.adapter._floor_map_callback(envelope(7, 11.0))

        self.assertFalse(self.adapter.floor_transition_active)
        self.assertEqual(self.adapter.map_update_count, 2)
        self.assertEqual(self.adapter.floor_map_versions[0], 8)
        self.assertEqual(self.adapter.floor_map_versions[1], 7)
        self.assertEqual(publish.call_count, 2)

    def test_old_epoch_map_cannot_poison_new_epoch_version_watermark(self):
        self.adapter.multifloor_enabled = True
        self.adapter.lock = threading.RLock()
        self.adapter.current_floor = 1
        self.adapter.map_epoch = 3
        self.adapter.floor_transition_active = False
        self.adapter.floor_map_versions = {1: 2}
        self.adapter.floor_map_epochs = {1: 3}
        self.adapter.floor_map_last_updates = {}
        self.adapter.floor_last_seen_versions = {(1, 3): 2}
        self.adapter.floor_map_load_times = {1: rospy.Time.from_sec(3.0)}
        self.adapter.last_map_received = rospy.Time(0)
        self.adapter.last_map_load_time = rospy.Time(0)
        self.adapter.latest_raw_map = None
        self.adapter.latest_raw_map_identity = None
        self.adapter.map_version = 2
        self.adapter.map_update_count = 2
        self.adapter.last_map_update = rospy.Time(0)

        def envelope(epoch, version, stamp):
            result = FloorOccupancyGrid()
            result.floor_id = 1
            result.map_epoch = epoch
            result.map_version = version
            result.occupancy_grid.header.frame_id = "map"
            result.occupancy_grid.header.stamp = rospy.Time.from_sec(stamp)
            result.occupancy_grid.info.map_load_time = rospy.Time.from_sec(3.0)
            return result

        with mock.patch.object(
            self.adapter, "_publish_cached_map_if_safe"
        ) as publish, mock.patch.object(
            rospy.Time, "now", return_value=rospy.Time.from_sec(20.0)
        ), mock.patch.object(rospy, "logwarn_throttle"):
            self.adapter._floor_map_callback(envelope(2, 99, 10.0))
            self.adapter._floor_map_callback(envelope(3, 3, 11.0))

        self.assertEqual(self.adapter.floor_map_epochs[1], 3)
        self.assertEqual(self.adapter.floor_map_versions[1], 3)
        self.assertNotIn((1, 2), self.adapter.floor_last_seen_versions)
        self.assertEqual(self.adapter.floor_last_seen_versions[(1, 3)], 3)
        publish.assert_called_once()

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

    def test_shutdown_stops_all_adapter_timers(self):
        self.adapter._shutdown_requested = False
        self.adapter.pose_timer = mock.Mock()
        self.adapter.status_timer = mock.Mock()

        self.adapter._on_shutdown()

        self.assertTrue(self.adapter._shutdown_requested)
        self.adapter.pose_timer.shutdown.assert_called_once_with()
        self.adapter.status_timer.shutdown.assert_called_once_with()

    def test_shutdown_callbacks_exit_before_touching_messages_or_publishers(self):
        self.adapter._shutdown_requested = True

        self.adapter._gicp_pose_callback(mock.sentinel.gicp_message)
        self.adapter._publish_pose_and_tf()
        self.adapter._publish_status()

    def test_publish_swallows_only_closed_publisher_shutdown_race(self):
        publisher = mock.Mock()
        publisher.publish.side_effect = rospy.ROSException("publisher closed")
        self.adapter._shutdown_requested = False

        with mock.patch.object(
            rospy, "is_shutdown", side_effect=(False, True)
        ):
            published = self.adapter._publish_if_running(
                publisher, mock.sentinel.message
            )

        self.assertFalse(published)
        publisher.publish.assert_called_once_with(mock.sentinel.message)

    def test_publish_preserves_non_shutdown_rospy_exception(self):
        publisher = mock.Mock()
        publisher.publish.side_effect = rospy.ROSException("transport failed")
        self.adapter._shutdown_requested = False

        with mock.patch.object(rospy, "is_shutdown", return_value=False):
            with self.assertRaises(rospy.ROSException):
                self.adapter._publish_if_running(
                    publisher, mock.sentinel.message
                )
