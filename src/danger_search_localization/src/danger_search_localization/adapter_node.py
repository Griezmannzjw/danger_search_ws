"""Expose a SLAM backend through the team localization interface."""

import copy
import math
import threading

import rospy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Header, Int32
from std_srvs.srv import Trigger

from danger_search_common.msg import (
    FloorOccupancyGrid,
    FloorMapInfo,
    LocalizationStatus,
    MappingStatus,
)
from danger_search_common.srv import SwitchFloor, SwitchFloorResponse

from .config import AdapterConfig
from .floor_mapping import FloorHeightClassifier, FloorSwitchState
from .pose_filter import PoseStabilizer
from .pose_fusion import compose, HectorGicpFusion, Pose2D
from .vertical_estimation import (
    VerticalEstimator,
    quaternion_from_rpy,
    quaternion_inverse,
    quaternion_multiply,
    quaternion_to_rpy,
    rotate_vector,
)


class LocalVelocityEstimator:
    """Differentiate trusted odom-frame poses into a base-frame planar twist."""

    def __init__(self, max_dt_s=0.50):
        if not math.isfinite(max_dt_s) or max_dt_s <= 0.0:
            raise ValueError("max odometry differentiation interval must be positive")
        self.max_dt_s = float(max_dt_s)
        self.previous = None

    @staticmethod
    def _angle_delta(current, previous):
        return math.atan2(
            math.sin(current - previous), math.cos(current - previous)
        )

    def update(self, stamp_s, pose):
        values = (stamp_s, pose.x, pose.y, pose.yaw)
        if not all(math.isfinite(value) for value in values):
            self.previous = None
            return (0.0, 0.0, 0.0), True

        current = (float(stamp_s), float(pose.x), float(pose.y), float(pose.yaw))
        if self.previous is None:
            self.previous = current
            return (0.0, 0.0, 0.0), True

        previous = self.previous
        dt = current[0] - previous[0]
        if dt <= 0.0:
            # Do not replace a newer reference with an out-of-order sample.
            return (0.0, 0.0, 0.0), True
        self.previous = current
        if dt > self.max_dt_s:
            return (0.0, 0.0, 0.0), True

        odom_vx = (current[1] - previous[1]) / dt
        odom_vy = (current[2] - previous[2]) / dt
        cosine = math.cos(current[3])
        sine = math.sin(current[3])
        base_vx = cosine * odom_vx + sine * odom_vy
        base_vy = -sine * odom_vx + cosine * odom_vy
        yaw_rate = self._angle_delta(current[3], previous[3]) / dt
        return (base_vx, base_vy, yaw_rate), False


class LocalizationAdapterNode:
    """Adapt Hector pose/map outputs without hiding backend limitations."""

    def __init__(self):
        rospy.init_node("localization_adapter", anonymous=False)
        self.competition_mode = bool(rospy.get_param("~competition_mode", True))
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base")
        self.backend_pose_topic = rospy.get_param(
            "~backend_pose_topic", "/localization/hector_pose"
        )
        self.gicp_pose_topic = rospy.get_param(
            "~gicp_pose_topic", "/localization/raw_pose"
        )
        self.validated_gicp_pose_topic = rospy.get_param(
            "~validated_gicp_pose_topic", "/localization/validated_pose"
        )
        self.raw_map_topic = rospy.get_param(
            "~raw_map_topic", "/localization/raw_map"
        )
        self.raw_floor_map_topic = rospy.get_param(
            "~raw_floor_map_topic", "/localization/raw_floor_map"
        )
        self.current_floor_topic = rospy.get_param(
            "~current_floor_topic", "/mapping/current_floor"
        )
        self.mapping_pause_topic = rospy.get_param(
            "~mapping_pause_topic", "/localization/mapping_pause"
        )
        self.switch_floor_service_name = rospy.get_param(
            "~switch_floor_service", "/localization/switch_floor"
        )
        self.mapper_switch_floor_service_name = rospy.get_param(
            "~mapper_switch_floor_service", "/localization/mapper_switch_floor"
        )
        self.gicp_rebaseline_service_name = rospy.get_param(
            "~gicp_rebaseline_service", "/localization/gicp_rebaseline"
        )
        self.floor_switch_service_timeout_s = float(rospy.get_param(
            "~floor_switch_service_timeout_s", 2.0
        ))
        if self.floor_switch_service_timeout_s <= 0.0:
            raise rospy.ROSInitException(
                "~floor_switch_service_timeout_s must be positive"
            )
        self.pose_topic = rospy.get_param(
            "~pose_topic", "/localization/pose"
        )
        self.odom_topic = rospy.get_param(
            "~odom_topic", "/localization/odom"
        )
        self.map_topic = rospy.get_param("~map_topic", "/map")
        self.active_map_topic = rospy.get_param(
            "~active_map_topic", "/mapping/active_map"
        )
        self.mapping_status_topic = rospy.get_param(
            "~mapping_status_topic", "/mapping/status"
        )
        self.localization_status_topic = rospy.get_param(
            "~localization_status_topic", "/localization/status"
        )
        self.localization_source = rospy.get_param(
            "~localization_backend",
            rospy.get_param("~localization_source", "gicp"),
        )
        if self.localization_source not in ("gicp", "gazebo_truth"):
            raise rospy.ROSInitException(
                "~localization_backend must be 'gicp' or 'gazebo_truth'"
            )
        self.config = self._load_config()
        self.multifloor_enabled = bool(
            rospy.get_param("~multifloor_enabled", False)
        )
        if self.competition_mode and self.localization_source == "gazebo_truth":
            raise rospy.ROSInitException(
                "gazebo_truth localization is forbidden in competition_mode"
            )
        if self.competition_mode and not self.multifloor_enabled:
            raise rospy.ROSInitException(
                "competition_mode requires multifloor_enabled=true"
            )
        # Both supported continuous-localization backends use the same
        # discrete-floor contract.  Truth pose z is useful for diagnostics,
        # but must not silently select a floor during a mission.  The old
        # height-driven path is deliberately opt-in for isolated tests only.
        self.allow_pose_height_floor_assignment = bool(rospy.get_param(
            "~allow_pose_height_floor_assignment", False
        ))
        self.explicit_floor_switching = bool(rospy.get_param(
            "~explicit_floor_switching", self.multifloor_enabled
        ))
        if (
            self.multifloor_enabled
            and not self.explicit_floor_switching
            and not self.allow_pose_height_floor_assignment
        ):
            raise rospy.ROSInitException(
                "multifloor localization requires explicit_floor_switching=true; "
                "pose-height assignment is test-only"
            )
        self.floor_classifier = FloorHeightClassifier(
            self.config.floor_heights,
            float(rospy.get_param("~floor_map_assignment_tolerance_m", 0.45)),
        )
        self.use_hector_correction = bool(
            rospy.get_param("~use_hector_correction", False)
        )
        if self.multifloor_enabled and self.use_hector_correction:
            raise rospy.ROSInitException(
                "multifloor maps currently require the odometry-driven mapper"
            )
        self.pose_fusion = HectorGicpFusion(self.config)
        self.pose_stabilizer = PoseStabilizer(self.config)
        self.vertical_estimator = VerticalEstimator(self.config)
        self.odom_velocity_max_dt_s = float(
            rospy.get_param("~odom_velocity_max_dt_s", 0.50)
        )
        self.odom_velocity_stale_timeout_s = float(
            rospy.get_param("~odom_velocity_stale_timeout_s", 0.50)
        )
        self.odom_twist_variance = float(
            rospy.get_param("~odom_twist_variance", 0.02)
        )
        self.odom_twist_degraded_variance = float(
            rospy.get_param("~odom_twist_degraded_variance", 1.0)
        )
        odom_values = (
            self.odom_velocity_max_dt_s,
            self.odom_velocity_stale_timeout_s,
            self.odom_twist_variance,
            self.odom_twist_degraded_variance,
        )
        if (not all(math.isfinite(value) and value > 0.0 for value in odom_values)
                or self.odom_twist_degraded_variance < self.odom_twist_variance):
            raise rospy.ROSInitException("invalid localization odometry parameters")
        self.velocity_estimator = LocalVelocityEstimator(
            self.odom_velocity_max_dt_s
        )

        self.lock = threading.RLock()
        # ROS can run a queued subscriber/timer callback while roslaunch is
        # tearing publishers down.  Keep an adapter-local latch in addition
        # to rospy.is_shutdown() so the callback side of that race never
        # publishes through a closed topic.
        self._shutdown_requested = False
        self.pose_timer = None
        self.status_timer = None
        self.latest_pose = None
        self.last_pose_received = rospy.Time(0)
        self.last_gicp_pose_accepted = rospy.Time(0)
        self.last_hector_pose_received = rospy.Time(0)
        self.last_hector_pose_accepted = rospy.Time(0)
        self.last_map_received = rospy.Time(0)
        self.last_public_map_published = rospy.Time(0)
        self.last_map_update = rospy.Time(0)
        self.last_map_load_time = rospy.Time(0)
        self.map_reset_pending = False
        self.last_mapping_pause_received = rospy.Time(0)
        self.map_version = 0
        self.map_update_count = 0
        self.last_map_stamp = rospy.Time(0)
        self.latest_raw_map = None
        # `(floor_id, map_epoch, map_version)` belonging to
        # `latest_raw_map`.  A raw OccupancyGrid itself has no floor identity,
        # so never infer it from arrival order.
        self.latest_raw_map_identity = None
        self.current_floor = int(self.config.current_floor)
        self.floor_switch_state = FloorSwitchState(
            self.config.floor_heights, initial_floor=self.current_floor
        )
        self.map_epoch = self.floor_switch_state.map_epoch
        self.current_height = float(
            self.config.floor_heights[self.current_floor]
        )
        self.floor_transition_active = False
        self.floor_transition_baseline_version = 0
        self.floor_map_versions = {}
        self.floor_map_epochs = {}
        self.floor_map_last_updates = {}
        self.floor_last_seen_versions = {}
        self.floor_map_load_times = {}
        self.ever_ready = False
        self.base_from_imu_quaternion = None
        self.latest_local_pose = None
        self.latest_map_to_odom = Pose2D(0.0, 0.0, 0.0)
        self.latest_local_velocity = (0.0, 0.0, 0.0)
        self.latest_local_velocity_degraded = True
        self.last_local_velocity_received = rospy.Time(0)
        self.last_tf_stamp = rospy.Time(0)
        self.gicp_consecutive_failures = 0
        self.hector_consecutive_rejections = 0
        self.last_hector_update_accepted = False
        self.pending_hector_pose = None
        self.last_gicp_fusion_reason = "WAITING_FOR_LOCAL_ODOMETRY"
        self.last_hector_fusion_reason = "WAITING_FOR_HECTOR_POSE"

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.pose_pub = rospy.Publisher(
            self.pose_topic, PoseWithCovarianceStamped, queue_size=10
        )
        self.odom_pub = rospy.Publisher(
            self.odom_topic, Odometry, queue_size=10
        )
        self.validated_pose_pub = rospy.Publisher(
            self.validated_gicp_pose_topic,
            PoseWithCovarianceStamped,
            queue_size=10,
        )
        self.map_pub = rospy.Publisher(
            self.map_topic, OccupancyGrid, queue_size=1, latch=True
        )
        self.active_map_pub = rospy.Publisher(
            self.active_map_topic, FloorOccupancyGrid, queue_size=1, latch=True
        )
        self.mapping_status_pub = rospy.Publisher(
            self.mapping_status_topic, MappingStatus, queue_size=5, latch=True
        )
        self.current_floor_pub = rospy.Publisher(
            self.current_floor_topic, Int32, queue_size=2, latch=True
        )
        self.localization_status_pub = rospy.Publisher(
            self.localization_status_topic,
            LocalizationStatus,
            queue_size=5,
            latch=True,
        )
        self.backend_pose_sub = None
        if self.use_hector_correction:
            self.backend_pose_sub = rospy.Subscriber(
                self.backend_pose_topic,
                PoseWithCovarianceStamped,
                self._backend_pose_callback,
                queue_size=10,
            )
        self.gicp_pose_sub = rospy.Subscriber(
            self.gicp_pose_topic,
            PoseWithCovarianceStamped,
            self._gicp_pose_callback,
            queue_size=10,
        )
        if self.multifloor_enabled:
            self.map_sub = rospy.Subscriber(
                self.raw_floor_map_topic,
                FloorOccupancyGrid,
                self._floor_map_callback,
                queue_size=1,
            )
        else:
            self.map_sub = rospy.Subscriber(
                self.raw_map_topic,
                OccupancyGrid,
                self._map_callback,
                queue_size=1,
            )
        self.mapping_pause_sub = rospy.Subscriber(
            self.mapping_pause_topic, Header, self._mapping_pause_callback,
            queue_size=10,
        )
        self.imu_sub = rospy.Subscriber(
            self.config.imu_topic,
            Imu,
            self._imu_callback,
            queue_size=200,
        )
        self.mapper_switch_floor = rospy.ServiceProxy(
            self.mapper_switch_floor_service_name, SwitchFloor
        )
        self.gicp_rebaseline = (
            rospy.ServiceProxy(self.gicp_rebaseline_service_name, Trigger)
            if self.localization_source == "gicp"
            else None
        )
        self.switch_floor_service = rospy.Service(
            self.switch_floor_service_name,
            SwitchFloor,
            self._switch_floor_callback,
        )

        self.pose_timer = rospy.Timer(
            rospy.Duration(1.0 / self.config.pose_publish_rate_hz),
            self._publish_pose_and_tf,
        )
        self.status_timer = rospy.Timer(
            rospy.Duration(1.0 / self.config.status_publish_rate_hz),
            self._publish_status,
        )
        rospy.on_shutdown(self._on_shutdown)
        self._publish_if_running(
            self.current_floor_pub, Int32(data=self.current_floor)
        )
        rospy.loginfo(
            "[localization] adapter started: backend=%s gicp=%s pose=%s map=%s "
            "(multifloor=%s)",
            (
                "gazebo_truth"
                if self.localization_source == "gazebo_truth"
                else (
                    "hector"
                    if self.use_hector_correction
                    else "gicp_occupancy"
                )
            ),
            self.gicp_pose_topic,
            self.pose_topic,
            self.map_topic,
            self.multifloor_enabled,
        )

    def _is_shutting_down(self):
        """Return whether callbacks must no longer touch ROS endpoints."""
        return bool(getattr(self, "_shutdown_requested", False)) or rospy.is_shutdown()

    def _on_shutdown(self):
        """Stop adapter timers before ROS closes publisher connections."""
        self._shutdown_requested = True
        for timer_name in ("pose_timer", "status_timer"):
            timer = getattr(self, timer_name, None)
            if timer is None:
                continue
            try:
                timer.shutdown()
            except rospy.ROSException:
                # This hook itself is running during shutdown. A timer may
                # already have been removed by rospy, which is harmless.
                pass

    def _publish_if_running(self, publisher, message):
        """Publish unless shutdown owns the endpoint.

        Do not hide ordinary ROS errors: the only exception tolerated here is
        the known roslaunch shutdown race where the publisher was closed after
        the initial lifecycle check.
        """
        if self._is_shutting_down():
            return False
        try:
            publisher.publish(message)
        except rospy.ROSException:
            if self._is_shutting_down():
                return False
            raise
        return True

    def _send_transform_if_running(self, transforms):
        """Broadcast TF with the same shutdown-race rule as publishers."""
        if self._is_shutting_down():
            return False
        try:
            self.tf_broadcaster.sendTransform(transforms)
        except rospy.ROSException:
            if self._is_shutting_down():
                return False
            raise
        return True

    def _switch_floor_callback(self, request):
        """Select the active floor without duplicating side effects on retry."""
        if self._is_shutting_down():
            return SwitchFloorResponse(
                success=False,
                map_epoch=int(getattr(self, "map_epoch", 0)),
                message="localization adapter is shutting down",
            )
        target_floor = int(request.target_floor)
        transition_id = str(request.transition_id).strip()
        with self.lock:
            replay = self.floor_switch_state.replay(
                transition_id, target_floor
            )
            if replay is not None:
                return SwitchFloorResponse(
                    success=replay.success,
                    map_epoch=replay.map_epoch,
                    message=replay.message,
                )
            validation = self.floor_switch_state.validate(
                transition_id, target_floor
            )
            if validation is not None:
                return SwitchFloorResponse(
                    success=validation.success,
                    map_epoch=validation.map_epoch,
                    message=validation.message,
                )
            if not self.multifloor_enabled:
                return SwitchFloorResponse(
                    success=False,
                    map_epoch=self.map_epoch,
                    message="multifloor localization is disabled",
                )
            changed = target_floor != self.current_floor
            transition_was_active = self.floor_transition_active
            if changed:
                self.floor_transition_active = True

        mapper_response = None
        try:
            if changed and self.gicp_rebaseline is not None:
                rospy.wait_for_service(
                    self.gicp_rebaseline_service_name,
                    timeout=self.floor_switch_service_timeout_s,
                )
                rebaseline_response = self.gicp_rebaseline()
                if not rebaseline_response.success:
                    raise rospy.ServiceException(rebaseline_response.message)
            if changed:
                rospy.wait_for_service(
                    self.mapper_switch_floor_service_name,
                    timeout=self.floor_switch_service_timeout_s,
                )
                mapper_response = self.mapper_switch_floor(
                    transition_id=transition_id,
                    target_floor=target_floor,
                )
                if not mapper_response.success:
                    raise rospy.ServiceException(mapper_response.message)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            with self.lock:
                self.floor_transition_active = transition_was_active
            return SwitchFloorResponse(
                success=False,
                map_epoch=self.map_epoch,
                message="floor switch dependency failed: %s" % exc,
            )

        publish_floor = None
        with self.lock:
            decision = self.floor_switch_state.request(
                transition_id, target_floor
            )
            expected_epoch = decision.map_epoch
            if (
                mapper_response is not None
                and int(mapper_response.map_epoch) != expected_epoch
            ):
                rospy.logerr(
                    "[localization] mapper/adapter map epoch mismatch: %d != %d",
                    int(mapper_response.map_epoch),
                    expected_epoch,
                )
            self.map_epoch = expected_epoch
            if decision.changed:
                previous_floor = self.current_floor
                self.current_floor = decision.current_floor
                self.current_height = decision.floor_z_m
                self.floor_transition_active = True
                self.floor_transition_baseline_version = int(
                    self.floor_map_versions.get(self.current_floor, 0)
                )
                self.map_version = self.floor_transition_baseline_version
                self.map_update_count = 0
                self.last_map_stamp = rospy.Time(0)
                self.last_map_received = rospy.Time(0)
                self.last_public_map_published = rospy.Time(0)
                self.last_map_update = rospy.Time(0)
                self.latest_raw_map = None
                self.latest_raw_map_identity = None
                publish_floor = self.current_floor
                rospy.loginfo(
                    "[localization] explicit floor transition %d -> %d "
                    "(transition=%s epoch=%d)",
                    previous_floor,
                    self.current_floor,
                    transition_id,
                    self.map_epoch,
                )
        if publish_floor is not None:
            self._publish_if_running(
                self.current_floor_pub, Int32(data=publish_floor)
            )
        return SwitchFloorResponse(
            success=True,
            map_epoch=decision.map_epoch,
            message=decision.message,
        )

    def _backend_pose_callback(self, message):
        if self._is_shutting_down():
            return
        if message.header.frame_id != self.map_frame:
            rospy.logwarn_throttle(
                2.0,
                "[localization] rejecting Hector pose in frame '%s', expected '%s'",
                message.header.frame_id,
                self.map_frame,
            )
            return
        orientation = message.pose.pose.orientation
        try:
            _, _, yaw = quaternion_to_rpy(
                (orientation.x, orientation.y, orientation.z, orientation.w)
            )
        except ValueError:
            rospy.logerr_throttle(
                1.0, "[localization] rejecting raw pose with invalid quaternion"
            )
            return
        now = rospy.Time.now()
        try:
            with self.lock:
                result = self.pose_fusion.update_global(
                    message.header.stamp.to_sec(),
                    message.pose.pose.position.x,
                    message.pose.pose.position.y,
                    yaw,
                )
                self.last_hector_pose_received = now
                self.last_hector_fusion_reason = result.reason
                if result.reason == "HECTOR_POSE_HAS_NO_SYNCHRONIZED_LOCAL_POSE":
                    self.pending_hector_pose = copy.deepcopy(message)
                    return
                self.latest_map_to_odom = result.correction
                self.last_hector_update_accepted = result.accepted
                if result.accepted:
                    self.last_hector_pose_accepted = now
                    self.hector_consecutive_rejections = 0
                    publish_cached_map = True
                else:
                    self.hector_consecutive_rejections += 1
                    publish_cached_map = False
        except ValueError as exc:
            rospy.logerr_throttle(1.0, "[localization] invalid Hector pose: %s", str(exc))
            return
        if not result.accepted:
            rospy.logwarn_throttle(
                1.0,
                "[localization] rejected Hector map correction: %s (consecutive=%d)",
                result.reason,
                result.consecutive_global_rejections,
            )
        elif publish_cached_map:
            self._publish_cached_map_if_safe()
    def _gicp_pose_callback(self, message):
        if self._is_shutting_down():
            return
        if message.header.frame_id != self.odom_frame:
            rospy.logwarn_throttle(
                2.0,
                "[localization] rejecting GICP pose in frame '%s', expected '%s'",
                message.header.frame_id,
                self.odom_frame,
            )
            return
        orientation = message.pose.pose.orientation
        try:
            _, _, yaw = quaternion_to_rpy(
                (orientation.x, orientation.y, orientation.z, orientation.w)
            )
        except ValueError as exc:
            rospy.logerr_throttle(1.0, "[localization] invalid GICP pose: %s", str(exc))
            return

        gicp_healthy = self._gicp_covariance_healthy(message)
        if not gicp_healthy:
            with self.lock:
                self.gicp_consecutive_failures += 1
                self.last_gicp_fusion_reason = "GICP_COVARIANCE_UNHEALTHY"
            rospy.logwarn_throttle(
                1.0,
                "[localization] holding last trusted pose because raw GICP "
                "covariance is unhealthy",
            )
            return

        stamp_s = message.header.stamp.to_sec()
        with self.lock:
            guarded = self.pose_stabilizer.update(
                stamp_s,
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                yaw,
            )
        if not guarded.accepted:
            with self.lock:
                self.gicp_consecutive_failures += 1
                self.last_gicp_fusion_reason = (
                    "POSE_GUARD_REJECTED:" + guarded.reason
                )
            rospy.logwarn_throttle(
                1.0,
                "[localization] rejected discontinuous GICP pose: %s "
                "(consecutive=%d)",
                guarded.reason,
                guarded.consecutive_rejections,
            )
            return

        local_pose = Pose2D(
            guarded.pose.x,
            guarded.pose.y,
            guarded.pose.yaw,
        )
        raw_position = message.pose.pose.position
        raw_height = float(raw_position.z)
        if not math.isfinite(raw_height):
            rospy.logwarn_throttle(
                1.0, "[localization] rejecting pose with invalid height"
            )
            return
        self._observe_floor_height(raw_height)
        with self.lock:
            canonical_height = (
                self.current_height
                if self.explicit_floor_switching
                else raw_height
            )
        raw_delta_xy = math.hypot(
            local_pose.x - float(raw_position.x),
            local_pose.y - float(raw_position.y),
        )
        raw_delta_yaw = abs(math.atan2(
            math.sin(local_pose.yaw - yaw), math.cos(local_pose.yaw - yaw)
        ))
        rospy.loginfo_throttle(
            2.0,
            "[localization] canonical/raw pose delta: mode=%s xy=%.4fm yaw=%.3fdeg",
            self.config.pose_stabilizer_mode,
            raw_delta_xy,
            math.degrees(raw_delta_yaw),
        )
        with self.lock:
            local_velocity, velocity_degraded = self.velocity_estimator.update(
                stamp_s, local_pose
            )
        try:
            if self.use_hector_correction:
                with self.lock:
                    result = self.pose_fusion.update_local(
                        stamp_s,
                        local_pose.x,
                        local_pose.y,
                        local_pose.yaw,
                    )
        except ValueError as exc:
            rospy.logerr_throttle(1.0, "[localization] invalid GICP pose: %s", str(exc))
            return

        if self.use_hector_correction:
            fused_pose = result.pose
            correction = result.correction
            fusion_reason = result.reason
        else:
            fused_pose = local_pose
            correction = Pose2D(0.0, 0.0, 0.0)
            fusion_reason = "TRACKING_GICP_LOCAL_OCCUPANCY_MAP"

        validated_pose = copy.deepcopy(message)
        validated_pose.header.frame_id = self.odom_frame
        validated_pose.pose.pose.position.x = local_pose.x
        validated_pose.pose.pose.position.y = local_pose.y
        validated_pose.pose.pose.position.z = (
            canonical_height if self.multifloor_enabled else 0.0
        )
        local_qx, local_qy, local_qz, local_qw = quaternion_from_rpy(
            0.0, 0.0, local_pose.yaw
        )
        validated_pose.pose.pose.orientation.x = local_qx
        validated_pose.pose.pose.orientation.y = local_qy
        validated_pose.pose.pose.orientation.z = local_qz
        validated_pose.pose.pose.orientation.w = local_qw

        pose = copy.deepcopy(validated_pose)
        pose.header.frame_id = self.map_frame
        pose.pose.pose.position.x = fused_pose.x
        pose.pose.pose.position.y = fused_pose.y
        pose.pose.pose.position.z = (
            canonical_height if self.multifloor_enabled else 0.0
        )
        qx, qy, qz, qw = quaternion_from_rpy(0.0, 0.0, fused_pose.yaw)
        pose.pose.pose.orientation.x = qx
        pose.pose.pose.orientation.y = qy
        pose.pose.pose.orientation.z = qz
        pose.pose.pose.orientation.w = qw
        self._set_output_covariance(validated_pose, gicp_healthy)
        self._set_output_covariance(pose, gicp_healthy)
        with self.lock:
            if gicp_healthy:
                self.gicp_consecutive_failures = 0
                self.last_gicp_pose_accepted = rospy.Time.now()
            else:
                self.gicp_consecutive_failures += 1
            self.latest_pose = pose
            self.latest_local_pose = local_pose
            self.latest_local_velocity = local_velocity
            self.latest_local_velocity_degraded = velocity_degraded
            self.last_local_velocity_received = rospy.Time.now()
            self.latest_map_to_odom = correction
            self.last_gicp_fusion_reason = (
                fusion_reason + ":" + guarded.reason
            )
            self.last_pose_received = rospy.Time.now()
            pending_hector_pose = (
                self.pending_hector_pose if self.use_hector_correction else None
            )
            if (
                pending_hector_pose is not None
                and abs(
                    pending_hector_pose.header.stamp.to_sec()
                    - message.header.stamp.to_sec()
                )
                <= self.config.fusion_max_pose_pair_age_s
            ):
                self.pending_hector_pose = None
            else:
                pending_hector_pose = None
        # The mapper only sees poses that passed both the discontinuity guard
        # and the raw GICP covariance check.  No publication means map freeze.
        if gicp_healthy:
            self._publish_if_running(self.validated_pose_pub, validated_pose)
        if pending_hector_pose is not None:
            self._backend_pose_callback(pending_hector_pose)

    def _observe_floor_height(self, height):
        if self._is_shutting_down():
            return
        if not getattr(self, "multifloor_enabled", False):
            return
        if getattr(self, "explicit_floor_switching", False):
            # The explicit SwitchFloor service is the authoritative source of
            # floor identity and configured z for both GICP and Gazebo-truth
            # continuous localization.
            return
        if not getattr(self, "allow_pose_height_floor_assignment", False):
            return
        assignment = self.floor_classifier.classify(height)
        publish_floor = None
        with self.lock:
            self.current_height = float(height)
            if assignment is None:
                if not self.floor_transition_active:
                    self.floor_transition_active = True
                    if hasattr(self, "floor_switch_state"):
                        self.floor_switch_state.transitioning = True
                    self.floor_transition_baseline_version = int(
                        self.floor_map_versions.get(self.current_floor, 0)
                    )
                    self.map_update_count = 0
                return
            floor_id = int(assignment.floor_id)
            if floor_id != self.current_floor:
                previous_floor = self.current_floor
                if hasattr(self, "floor_switch_state"):
                    self.floor_switch_state.force_floor(floor_id)
                    self.map_epoch = self.floor_switch_state.map_epoch
                else:
                    self.map_epoch = getattr(self, "map_epoch", 1) + 1
                self.current_floor = floor_id
                self.current_height = float(assignment.floor_height)
                self.floor_transition_active = True
                self.floor_transition_baseline_version = int(
                    self.floor_map_versions.get(floor_id, 0)
                )
                self.map_version = self.floor_transition_baseline_version
                self.map_update_count = 0
                self.last_map_stamp = rospy.Time(0)
                self.last_map_received = rospy.Time(0)
                self.last_public_map_published = rospy.Time(0)
                self.last_map_update = rospy.Time(0)
                self.latest_raw_map = None
                self.latest_raw_map_identity = None
                publish_floor = floor_id
                rospy.loginfo(
                    "[localization] confirmed floor transition %d -> %d "
                    "at relative z=%.3f m",
                    previous_floor,
                    floor_id,
                    height,
                )
        if publish_floor is not None:
            self._publish_if_running(
                self.current_floor_pub, Int32(data=publish_floor)
            )

    def _map_callback(self, message):
        if self._is_shutting_down():
            return
        if message.header.frame_id != self.map_frame:
            rospy.logwarn_throttle(
                2.0,
                "[localization] rejecting map in frame '%s', expected '%s'",
                message.header.frame_id,
                self.map_frame,
            )
            return
        now = rospy.Time.now()
        with self.lock:
            self.last_map_received = now
            load_time = message.info.map_load_time
            previous_load_time = getattr(self, "last_map_load_time", rospy.Time(0))
            if (
                previous_load_time != rospy.Time(0)
                and load_time != rospy.Time(0)
                and load_time != previous_load_time
            ):
                if hasattr(self, "floor_switch_state"):
                    self.floor_switch_state.reset_map()
                    self.map_epoch = self.floor_switch_state.map_epoch
                else:
                    self.map_epoch = getattr(self, "map_epoch", 1) + 1
                self.map_update_count = 0
                self.last_map_stamp = rospy.Time(0)
                self.map_reset_pending = True
                self.last_map_update = now
                rospy.logwarn("[localization] detected occupancy map reset epoch")
            self.last_map_load_time = load_time
            # rospy delivers a new message object per callback. Keep that immutable
            # snapshot directly; copying a 1024x1024 grid twice starves sensor callbacks.
            self.latest_raw_map = message
            # In the single-map compatibility path map_version is advanced
            # immediately before publication below.  Reserve that version so
            # `/map` and `/mapping/active_map` can still be compared exactly.
            next_version = self.map_version
            if message.header.stamp != self.last_map_stamp:
                next_version += 1
            self.latest_raw_map_identity = (
                int(self.current_floor), int(self.map_epoch), int(next_version)
            )
        self._publish_cached_map_if_safe()

    def _floor_map_callback(self, envelope):
        if self._is_shutting_down():
            return
        floor_id = int(envelope.floor_id)
        if floor_id < 0 or floor_id >= len(self.config.floor_heights):
            rospy.logwarn_throttle(
                2.0, "[localization] rejecting invalid floor id %d", floor_id
            )
            return
        message = envelope.occupancy_grid
        if message.header.frame_id != self.map_frame:
            rospy.logwarn_throttle(
                2.0,
                "[localization] rejecting floor %d map in frame '%s'",
                floor_id,
                message.header.frame_id,
            )
            return
        map_epoch = int(envelope.map_epoch)
        version = int(envelope.map_version)
        if map_epoch < 1:
            rospy.logwarn_throttle(
                2.0,
                "[localization] rejecting floor %d map with invalid epoch %d",
                floor_id,
                map_epoch,
            )
            return
        now = rospy.Time.now()
        publish_current = False
        with self.lock:
            previous_epoch = self.floor_map_epochs.get(floor_id)
            if previous_epoch is not None and map_epoch < previous_epoch:
                rospy.logwarn_throttle(
                    2.0,
                    "[localization] rejecting stale floor %d map epoch %d < %d",
                    floor_id,
                    map_epoch,
                    previous_epoch,
                )
                return
            epoch_key = (floor_id, map_epoch)
            load_time = message.info.map_load_time
            previous_load_time = self.floor_map_load_times.get(
                floor_id, rospy.Time(0)
            )
            reset_epoch = (
                previous_load_time != rospy.Time(0)
                and load_time != rospy.Time(0)
                and load_time != previous_load_time
            )
            self.floor_map_load_times[floor_id] = load_time
            if reset_epoch:
                self.floor_last_seen_versions[epoch_key] = 0
                if floor_id == self.current_floor:
                    if hasattr(self, "floor_switch_state"):
                        self.floor_switch_state.reset_map()
                        self.map_epoch = self.floor_switch_state.map_epoch
                    else:
                        self.map_epoch = getattr(self, "map_epoch", 1) + 1
                    self.floor_transition_baseline_version = 0
                    self.floor_transition_active = True
                    self.map_update_count = 0

            previous_version = int(
                self.floor_last_seen_versions.get(epoch_key, 0)
            )
            if version < previous_version and not reset_epoch:
                rospy.logwarn_throttle(
                    2.0,
                    "[localization] rejecting regressed floor %d map version "
                    "%d < %d",
                    floor_id,
                    version,
                    previous_version,
                )
                return
            self.floor_last_seen_versions[epoch_key] = version
            self.floor_map_versions[floor_id] = version
            self.floor_map_epochs[floor_id] = map_epoch
            self.floor_map_last_updates[floor_id] = (
                message.header.stamp
                if message.header.stamp != rospy.Time(0)
                else now
            )
            if floor_id != self.current_floor:
                return

            # Delayed per-floor maps are expected while an elevator changes
            # floors.  They must never overwrite the map selected by the
            # explicit transition service, even if their version is newer.
            if map_epoch != self.map_epoch:
                rospy.logwarn_throttle(
                    1.0,
                    "[localization] rejecting stale active map floor=%d "
                    "epoch=%d (expected=%d)",
                    floor_id,
                    map_epoch,
                    self.map_epoch,
                )
                return

            self.last_map_received = now
            self.last_map_load_time = load_time
            self.latest_raw_map = message
            self.latest_raw_map_identity = (floor_id, map_epoch, version)
            self.map_version = version
            if self.floor_transition_active:
                self.map_update_count = max(
                    0, version - self.floor_transition_baseline_version
                )
                if (
                    self.map_update_count
                    >= self.config.min_map_updates_for_stable
                ):
                    self.floor_transition_active = False
                    if hasattr(self, "floor_switch_state"):
                        self.floor_switch_state.mark_stable()
                    rospy.loginfo(
                        "[localization] floor %d map restored and refreshed "
                        "at version %d",
                        floor_id,
                        version,
                    )
            else:
                self.map_update_count += max(0, version - previous_version)
            self.last_map_update = self.floor_map_last_updates[floor_id]
            publish_current = True
        if publish_current:
            self._publish_cached_map_if_safe()

    def _mapping_pause_callback(self, _message):
        if self._is_shutting_down():
            return
        with self.lock:
            self.last_mapping_pause_received = rospy.Time.now()

    def _publish_cached_map_if_safe(self, now=None):
        if self._is_shutting_down():
            return
        now = now or rospy.Time.now()
        with self.lock:
            map_correction_healthy = (
                (
                    self.pose_fusion.initialized
                    and self.last_hector_update_accepted
                )
                if self.use_hector_correction
                else self.latest_local_pose is not None
            )
            gicp_healthy = (
                self.last_gicp_pose_accepted != rospy.Time(0)
                and self._age(now, self.last_gicp_pose_accepted)
                <= self.config.gicp_healthy_fresh_timeout_s
            )
            message = self.latest_raw_map
            identity = getattr(self, "latest_raw_map_identity", None)
            floor_transition_active = getattr(
                self, "floor_transition_active", False
            )
        if floor_transition_active:
            rospy.loginfo_throttle(
                1.0,
                "[localization] withholding /map during floor transition",
            )
            return
        if not map_correction_healthy or not gicp_healthy:
            rospy.logwarn_throttle(
                1.0,
                "[localization] withholding public map until odometry and "
                "mapping are healthy",
            )
            return
        if message is not None:
            with self.lock:
                stamp = message.header.stamp
                if stamp != self.last_map_stamp:
                    self.last_map_stamp = stamp
                    if not getattr(self, "multifloor_enabled", False):
                        self.map_version += 1
                        if self.map_reset_pending:
                            self.map_reset_pending = False
                        else:
                            self.map_update_count += 1
                    self.last_map_update = stamp or rospy.Time.now()
                    self.last_public_map_published = now
                current_identity = (
                    int(self.current_floor),
                    int(self.map_epoch),
                    int(self.map_version),
                )
                # A raw map is allowed onto the public navigation interface
                # only when it still belongs to the current floor selection.
                # The envelope gives navigation an atomic identity alongside
                # the legacy `/map` payload.
                if identity is None or identity[:2] != current_identity[:2]:
                    rospy.logwarn_throttle(
                        1.0,
                        "[localization] withholding map with stale floor/epoch identity",
                    )
                    return
                self.latest_raw_map_identity = current_identity
                active_map_pub = getattr(self, "active_map_pub", None)
            self._publish_if_running(self.map_pub, message)
            if active_map_pub is not None:
                envelope = FloorOccupancyGrid()
                envelope.header = message.header
                envelope.floor_id = current_identity[0]
                envelope.map_epoch = current_identity[1]
                envelope.map_version = current_identity[2]
                envelope.occupancy_grid = message
                self._publish_if_running(active_map_pub, envelope)

    def _imu_callback(self, message):
        if self._is_shutting_down():
            return
        if not self.config.vertical_estimation_enabled:
            return
        base_from_imu = self._base_from_imu(message.header.frame_id)
        if base_from_imu is None:
            return
        world_from_imu = (
            message.orientation.x,
            message.orientation.y,
            message.orientation.z,
            message.orientation.w,
        )
        world_from_base = quaternion_multiply(
            world_from_imu, quaternion_inverse(base_from_imu)
        )
        acceleration_base = rotate_vector(
            (
                message.linear_acceleration.x,
                message.linear_acceleration.y,
                message.linear_acceleration.z,
            ),
            base_from_imu,
        )
        angular_velocity_base = rotate_vector(
            (
                message.angular_velocity.x,
                message.angular_velocity.y,
                message.angular_velocity.z,
            ),
            base_from_imu,
        )
        try:
            with self.lock:
                self.vertical_estimator.update(
                    message.header.stamp.to_sec(),
                    world_from_base,
                    angular_velocity_base,
                    acceleration_base,
                )
        except ValueError as exc:
            rospy.logwarn_throttle(
                2.0, "[localization] invalid IMU sample: %s", str(exc)
            )

    def _base_from_imu(self, imu_frame):
        if self._is_shutting_down():
            return None
        if not imu_frame:
            rospy.logwarn_throttle(2.0, "[localization] IMU frame_id is empty")
            return None
        with self.lock:
            cached = self.base_from_imu_quaternion
        if cached is not None:
            return cached
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, imu_frame, rospy.Time(0), rospy.Duration(0.1)
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            rospy.logwarn_throttle(
                2.0,
                "[localization] TF %s <- %s unavailable for IMU: %s",
                self.base_frame,
                imu_frame,
                str(exc),
            )
            return None
        rotation = transform.transform.rotation
        quaternion = (rotation.x, rotation.y, rotation.z, rotation.w)
        with self.lock:
            self.base_from_imu_quaternion = quaternion
        return quaternion

    def _publish_pose_and_tf(self, _event=None):
        if self._is_shutting_down():
            return
        pose_stamp = rospy.Time.now()
        with self.lock:
            pose = copy.deepcopy(self.latest_pose)
            vertical = self.vertical_estimator.snapshot()
            local_pose = self.latest_local_pose
            correction = self.latest_map_to_odom
            local_velocity = self.latest_local_velocity
            velocity_degraded = self.latest_local_velocity_degraded
            velocity_received = self.last_local_velocity_received
            current_height = self.current_height
        if pose is None:
            return
        # The cached GICP measurement may be older than the adapter timer.
        # Public pose messages are a live interface, so give each publication
        # the timer stamp instead of repeating the sensor stamp.
        pose.header.stamp = pose_stamp
        # Recompose from the same correction and local pose used for TF.  A
        # Hector callback may update map->odom between GICP frames; publishing
        # cached XY/yaw here would briefly disagree with the TF tree.
        if local_pose is not None:
            fused_pose = compose(correction, local_pose)
            pose.pose.pose.position.x = fused_pose.x
            pose.pose.pose.position.y = fused_pose.y
            qx, qy, qz, qw = quaternion_from_rpy(
                0.0, 0.0, fused_pose.yaw
            )
            pose.pose.pose.orientation.x = qx
            pose.pose.pose.orientation.y = qy
            pose.pose.pose.orientation.z = qz
            pose.pose.pose.orientation.w = qw
        self._apply_vertical_state(pose, vertical)
        self._publish_if_running(self.pose_pub, pose)

        if local_pose is None:
            return
        velocity_stale = (
            velocity_received == rospy.Time(0)
            or self._age(pose_stamp, velocity_received)
            > self.odom_velocity_stale_timeout_s
        )
        self._publish_odometry(
            pose_stamp,
            local_pose,
            vertical,
            (0.0, 0.0, 0.0) if velocity_stale else local_velocity,
            velocity_degraded or velocity_stale,
            current_height,
        )
        # Hector and perception query TF at sensor timestamps that can lead the
        # adapter timer under simulation load. A short, bounded future stamp is
        # the standard ROS transform-tolerance pattern for this scheduling gap.
        stamp = pose_stamp + rospy.Duration(
            self.config.tf_publish_future_tolerance_s
        )
        with self.lock:
            if stamp <= self.last_tf_stamp:
                return
            self.last_tf_stamp = stamp
        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = stamp
        map_to_odom.header.frame_id = self.map_frame
        map_to_odom.child_frame_id = self.odom_frame
        map_to_odom.transform.translation.x = correction.x
        map_to_odom.transform.translation.y = correction.y
        _, _, qz, qw = quaternion_from_rpy(0.0, 0.0, correction.yaw)
        map_to_odom.transform.rotation.z = qz
        map_to_odom.transform.rotation.w = qw

        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = stamp
        odom_to_base.header.frame_id = self.odom_frame
        odom_to_base.child_frame_id = self.base_frame
        odom_to_base.transform.translation.x = local_pose.x
        odom_to_base.transform.translation.y = local_pose.y
        if getattr(self, "multifloor_enabled", False):
            odom_to_base.transform.translation.z = current_height
            qx, qy, qz, qw = quaternion_from_rpy(
                0.0, 0.0, local_pose.yaw
            )
        elif self.config.vertical_estimation_enabled and vertical.initialized:
            odom_to_base.transform.translation.z = vertical.z
            qx, qy, qz, qw = quaternion_from_rpy(
                vertical.roll, vertical.pitch, local_pose.yaw
            )
        else:
            qx, qy, qz, qw = quaternion_from_rpy(0.0, 0.0, local_pose.yaw)
        odom_to_base.transform.rotation.x = qx
        odom_to_base.transform.rotation.y = qy
        odom_to_base.transform.rotation.z = qz
        odom_to_base.transform.rotation.w = qw
        self._send_transform_if_running([map_to_odom, odom_to_base])

    def _publish_odometry(
        self, stamp, local_pose, vertical, velocity, degraded, current_height=None
    ):
        if self._is_shutting_down():
            return
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = self.odom_frame
        message.child_frame_id = self.base_frame
        message.pose.pose.position.x = local_pose.x
        message.pose.pose.position.y = local_pose.y
        if getattr(self, "multifloor_enabled", False):
            message.pose.pose.position.z = float(
                0.0 if current_height is None else current_height
            )
            qx, qy, qz, qw = quaternion_from_rpy(
                0.0, 0.0, local_pose.yaw
            )
        elif self.config.vertical_estimation_enabled and vertical.initialized:
            message.pose.pose.position.z = vertical.z
            qx, qy, qz, qw = quaternion_from_rpy(
                vertical.roll, vertical.pitch, local_pose.yaw
            )
        else:
            qx, qy, qz, qw = quaternion_from_rpy(0.0, 0.0, local_pose.yaw)
        message.pose.pose.orientation.x = qx
        message.pose.pose.orientation.y = qy
        message.pose.pose.orientation.z = qz
        message.pose.pose.orientation.w = qw
        message.twist.twist.linear.x = velocity[0]
        message.twist.twist.linear.y = velocity[1]
        message.twist.twist.angular.z = velocity[2]

        for index in (0, 7):
            message.pose.covariance[index] = self.config.fallback_xy_variance
        message.pose.covariance[35] = self.config.fallback_yaw_variance
        for index in (14, 21, 28):
            message.pose.covariance[index] = self.config.fallback_unobserved_variance
        twist_variance = (
            self.odom_twist_degraded_variance
            if degraded else self.odom_twist_variance
        )
        for index in (0, 7, 35):
            message.twist.covariance[index] = twist_variance
        for index in (14, 21, 28):
            message.twist.covariance[index] = self.config.fallback_unobserved_variance
        self._publish_if_running(self.odom_pub, message)

    def _publish_status(self, _event=None):
        if self._is_shutting_down():
            return
        now = rospy.Time.now()
        with self.lock:
            pose = copy.deepcopy(self.latest_pose)
            pose_age = self._age(now, self.last_pose_received)
            map_age = self._age(now, self.last_public_map_published)
            map_version = self.map_version
            map_update_count = self.map_update_count
            last_map_update = self.last_map_update
            mapping_pause_received = self.last_mapping_pause_received
            vertical = self.vertical_estimator.snapshot()
            fusion_initialized = (
                self.pose_fusion.initialized
                if self.use_hector_correction
                else self.latest_local_pose is not None
            )
            gicp_healthy_age = self._age(now, self.last_gicp_pose_accepted)
            hector_age = self._age(now, self.last_hector_pose_accepted)
            gicp_fusion_reason = self.last_gicp_fusion_reason
            hector_fusion_reason = self.last_hector_fusion_reason
            pose_guard_rejections = self.pose_stabilizer.consecutive_rejections
            pose_guard_reason = self.pose_stabilizer.last_reason

            multifloor_enabled = getattr(self, "multifloor_enabled", False)
            floor_transition_active = getattr(
                self, "floor_transition_active", False
            )
            current_floor = (
                int(self.current_floor)
                if multifloor_enabled
                else (
                    int(vertical.current_floor)
                    if vertical.initialized
                    else int(self.config.current_floor)
                )
            )
            map_epoch = int(getattr(self, "map_epoch", 1))
            floor_z_m = float(self.config.floor_heights[current_floor])
            floor_map_versions = dict(
                getattr(self, "floor_map_versions", {})
            )
            floor_map_last_updates = dict(
                getattr(self, "floor_map_last_updates", {})
            )

        pose_fresh = pose_age <= self.config.pose_fresh_timeout_s
        mapping_paused = (
            self._age(now, mapping_pause_received)
            <= self.config.mapping_pause_timeout_s
        )
        map_fresh = (
            map_age <= self.config.map_fresh_timeout_s
            or (
                mapping_paused
                and map_update_count >= self.config.min_map_updates_for_stable
            )
        )
        hector_fresh = (
            hector_age <= self.config.hector_pose_fresh_timeout_s
            if self.use_hector_correction
            else True
        )
        pose_guard_degraded = pose_guard_rejections > 0
        pose_guard_lost = (
            pose_guard_rejections >= self.config.pose_rejections_before_lost
        )
        gicp_degraded = pose_guard_degraded or (
            gicp_healthy_age > self.config.gicp_healthy_fresh_timeout_s
        )
        gicp_lost = pose_guard_lost or (
            gicp_healthy_age > self.config.gicp_healthy_lost_timeout_s
        )
        hector_degraded = not hector_fresh
        ready = (
            pose is not None
            and pose_fresh
            and map_fresh
            and fusion_initialized
            and not gicp_lost
            and not floor_transition_active
        )
        stable = (
            ready
            and not gicp_degraded
            and not hector_degraded
            and map_update_count >= self.config.min_map_updates_for_stable
        )
        vertical_fresh = (
            not self.config.vertical_estimation_enabled
            or (
                vertical.initialized
                and now.to_sec() - vertical.stamp_s
                <= self.config.vertical_imu_fresh_timeout_s
            )
        )
        lost = self.ever_ready and (not pose_fresh or gicp_lost)
        self.ever_ready = self.ever_ready or ready
        reason = self._status_reason(
            pose,
            pose_fresh,
            map_fresh,
            stable,
            vertical_fresh,
            gicp_degraded,
            gicp_lost,
            hector_degraded,
            gicp_fusion_reason,
            hector_fusion_reason,
            self.use_hector_correction,
            pose_guard_degraded,
            pose_guard_lost,
            pose_guard_reason,
            mapping_paused,
            self.localization_source,
            floor_transition_active,
        )

        mapping = MappingStatus()
        mapping.header.stamp = now
        mapping.header.frame_id = self.map_frame
        mapping.ready = ready
        mapping.stable = stable
        mapping.lost = lost
        mapping.current_floor = current_floor
        mapping.transitioning = floor_transition_active
        mapping.map_epoch = map_epoch
        mapping.floor_z_m = floor_z_m
        if multifloor_enabled:
            for floor_id in sorted(floor_map_versions):
                floor = FloorMapInfo()
                floor.floor_id = int(floor_id)
                floor.map_version = int(floor_map_versions[floor_id])
                floor.last_update = floor_map_last_updates.get(
                    floor_id, rospy.Time(0)
                )
                mapping.floor_maps.append(floor)
        elif map_version > 0:
            floor = FloorMapInfo()
            floor.floor_id = current_floor
            floor.map_version = map_version
            floor.last_update = last_map_update
            mapping.floor_maps.append(floor)
        mapping.status_reason = reason
        self._publish_if_running(self.mapping_status_pub, mapping)

        localization = LocalizationStatus()
        localization.header = mapping.header
        localization.tracking_state = self._tracking_state(
            pose,
            pose_fresh,
            map_fresh and fusion_initialized and not floor_transition_active,
            degraded=(
                gicp_degraded
                or hector_degraded
                or floor_transition_active
            ),
            lost=gicp_lost,
        )
        covariance_trace = self._covariance_trace(pose)
        localization.pose_covariance_trace = covariance_trace
        localization.drift_warning = (
            not stable
            or not vertical_fresh
            or covariance_trace > self.config.covariance_warning_trace
        )
        localization.correction_version = 0
        if stable and pose is not None:
            localization.last_stable_time = pose.header.stamp
        localization.status_reason = reason
        self._publish_if_running(self.localization_status_pub, localization)

    def _ensure_covariance(self, pose):
        covariance = list(pose.pose.covariance)
        diagonal = [covariance[index] for index in (0, 7, 14, 21, 28, 35)]
        backend_covariance_valid = (
            self.config.use_backend_covariance
            and all(math.isfinite(value) and value >= 0.0 for value in diagonal)
            and any(value > 0.0 for value in diagonal)
        )
        if backend_covariance_valid:
            return

        covariance = [0.0] * 36
        covariance[0] = self.config.fallback_xy_variance
        covariance[7] = self.config.fallback_xy_variance
        covariance[14] = self.config.fallback_unobserved_variance
        covariance[21] = self.config.fallback_unobserved_variance
        covariance[28] = self.config.fallback_unobserved_variance
        covariance[35] = self.config.fallback_yaw_variance
        pose.pose.covariance = covariance

    def _gicp_covariance_healthy(self, pose):
        covariance = pose.pose.covariance
        return all(
            math.isfinite(covariance[index])
            and covariance[index] < self.config.gicp_unhealthy_variance_threshold
            for index in (0, 7, 35)
        )

    def _set_output_covariance(self, pose, healthy):
        if healthy:
            self._ensure_covariance(pose)
            return
        covariance = [0.0] * 36
        for index in (0, 7, 14, 21, 28, 35):
            covariance[index] = self.config.gicp_unhealthy_variance_threshold
        pose.pose.covariance = covariance

    def _apply_vertical_state(self, pose, vertical):
        if getattr(self, "multifloor_enabled", False):
            with self.lock:
                pose.pose.pose.position.z = self.current_height
            return
        if not self.config.vertical_estimation_enabled or not vertical.initialized:
            return
        _, _, yaw = quaternion_to_rpy(
            (
                pose.pose.pose.orientation.x,
                pose.pose.pose.orientation.y,
                pose.pose.pose.orientation.z,
                pose.pose.pose.orientation.w,
            )
        )
        qx, qy, qz, qw = quaternion_from_rpy(
            vertical.roll, vertical.pitch, yaw
        )
        pose.pose.pose.position.z = vertical.z
        pose.pose.pose.orientation.x = qx
        pose.pose.pose.orientation.y = qy
        pose.pose.pose.orientation.z = qz
        pose.pose.pose.orientation.w = qw

    @staticmethod
    def _age(now, stamp):
        if stamp == rospy.Time(0):
            return float("inf")
        return max(0.0, (now - stamp).to_sec())

    @staticmethod
    def _covariance_trace(pose):
        if pose is None:
            # A negative value explicitly means "not available" while keeping
            # the status message finite and serialization-friendly.
            return -1.0
        return sum(
            float(pose.pose.covariance[index])
            for index in (0, 7, 14, 21, 28, 35)
        )

    @staticmethod
    def _status_reason(
        pose,
        pose_fresh,
        map_fresh,
        stable,
        vertical_fresh=True,
        gicp_degraded=False,
        gicp_lost=False,
        hector_degraded=False,
        gicp_fusion_reason="",
        hector_fusion_reason="",
        use_hector_correction=True,
        pose_guard_degraded=False,
        pose_guard_lost=False,
        pose_guard_reason="",
        mapping_paused=False,
        localization_source="gicp",
        floor_transition_active=False,
    ):
        using_gazebo_truth = localization_source == "gazebo_truth"
        if pose is None:
            return (
                "WAITING_FOR_GAZEBO_TRUTH_POSE"
                if using_gazebo_truth
                else "WAITING_FOR_SCAN_MATCHING_POSE"
            )
        if not pose_fresh:
            return (
                "GAZEBO_TRUTH_POSE_STALE"
                if using_gazebo_truth
                else "SCAN_MATCHING_POSE_STALE"
            )
        if pose_guard_lost:
            return "POSE_GUARD_LOST:" + pose_guard_reason
        if pose_guard_degraded:
            return "POSE_GUARD_DEGRADED_HOLDING_LAST_POSE:" + pose_guard_reason
        if gicp_lost:
            prefix = (
                "GAZEBO_TRUTH_LOST:"
                if using_gazebo_truth
                else "GICP_ODOMETRY_LOST:"
            )
            return prefix + gicp_fusion_reason
        if gicp_degraded:
            return (
                "GAZEBO_TRUTH_DEGRADED_HOLDING_LAST_POSE"
                if using_gazebo_truth
                else "GICP_ODOMETRY_DEGRADED_HOLDING_LAST_POSE"
            )
        if hector_degraded:
            return "HECTOR_CORRECTION_DEGRADED:" + hector_fusion_reason
        if floor_transition_active:
            return "FLOOR_TRANSITION_WAITING_FOR_CURRENT_MAP"
        if not map_fresh:
            return "MAP_STALE"
        if mapping_paused:
            return "MAPPING_PAUSED_HIGH_ANGULAR_RATE"
        if not stable:
            return "WAITING_FOR_STABLE_MAP"
        if not vertical_fresh:
            return "VERTICAL_IMU_STALE"
        if use_hector_correction:
            return "TRACKING_FUSED_GICP_ODOMETRY_WITH_BOUNDED_HECTOR_CORRECTION"
        if using_gazebo_truth:
            return "TRACKING_GAZEBO_TRUTH_WITH_LOCAL_OCCUPANCY_MAP"
        return "TRACKING_GICP_ODOMETRY_WITH_LOCAL_OCCUPANCY_MAP"

    @staticmethod
    def _tracking_state(pose, pose_fresh, map_fresh, degraded=False, lost=False):
        if pose is None:
            return LocalizationStatus.STATE_INITIALIZING
        if not pose_fresh or lost:
            return LocalizationStatus.STATE_LOST
        if not map_fresh or degraded:
            return LocalizationStatus.STATE_DEGRADED
        return LocalizationStatus.STATE_TRACKING

    @staticmethod
    def _load_config():
        defaults = AdapterConfig()
        return AdapterConfig(
            **{
                name: rospy.get_param("~" + name, value)
                for name, value in vars(defaults).items()
            }
        )

    @staticmethod
    def run():
        rospy.spin()
