"""Expose a SLAM backend through the team localization interface."""

import copy
import math
import threading

import rospy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Imu

from danger_search_common.msg import (
    FloorMapInfo,
    LocalizationStatus,
    MappingStatus,
)

from .config import AdapterConfig
from .pose_fusion import compose, HectorGicpFusion, Pose2D
from .vertical_estimation import (
    VerticalEstimator,
    quaternion_from_rpy,
    quaternion_inverse,
    quaternion_multiply,
    quaternion_to_rpy,
    rotate_vector,
)


class LocalizationAdapterNode:
    """Adapt Hector pose/map outputs without hiding backend limitations."""

    def __init__(self):
        rospy.init_node("localization_adapter", anonymous=False)
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base")
        self.backend_pose_topic = rospy.get_param(
            "~backend_pose_topic", "/localization/hector_pose"
        )
        self.gicp_pose_topic = rospy.get_param(
            "~gicp_pose_topic", "/localization/raw_pose"
        )
        self.raw_map_topic = rospy.get_param(
            "~raw_map_topic", "/localization/raw_map"
        )
        self.pose_topic = rospy.get_param(
            "~pose_topic", "/localization/pose"
        )
        self.map_topic = rospy.get_param("~map_topic", "/map")
        self.mapping_status_topic = rospy.get_param(
            "~mapping_status_topic", "/mapping/status"
        )
        self.localization_status_topic = rospy.get_param(
            "~localization_status_topic", "/localization/status"
        )
        self.config = self._load_config()
        self.pose_fusion = HectorGicpFusion(self.config)
        self.vertical_estimator = VerticalEstimator(self.config)

        self.lock = threading.RLock()
        self.latest_pose = None
        self.last_pose_received = rospy.Time(0)
        self.last_gicp_pose_accepted = rospy.Time(0)
        self.last_hector_pose_received = rospy.Time(0)
        self.last_hector_pose_accepted = rospy.Time(0)
        self.last_map_received = rospy.Time(0)
        self.last_map_update = rospy.Time(0)
        self.map_version = 0
        self.map_update_count = 0
        self.last_map_stamp = rospy.Time(0)
        self.latest_raw_map = None
        self.ever_ready = False
        self.base_from_imu_quaternion = None
        self.latest_local_pose = None
        self.latest_map_to_odom = Pose2D(0.0, 0.0, 0.0)
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
        self.map_pub = rospy.Publisher(
            self.map_topic, OccupancyGrid, queue_size=1, latch=True
        )
        self.mapping_status_pub = rospy.Publisher(
            self.mapping_status_topic, MappingStatus, queue_size=5, latch=True
        )
        self.localization_status_pub = rospy.Publisher(
            self.localization_status_topic,
            LocalizationStatus,
            queue_size=5,
            latch=True,
        )
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
        self.map_sub = rospy.Subscriber(
            self.raw_map_topic,
            OccupancyGrid,
            self._map_callback,
            queue_size=1,
        )
        self.imu_sub = rospy.Subscriber(
            self.config.imu_topic,
            Imu,
            self._imu_callback,
            queue_size=200,
        )

        self.pose_timer = rospy.Timer(
            rospy.Duration(1.0 / self.config.pose_publish_rate_hz),
            self._publish_pose_and_tf,
        )
        self.status_timer = rospy.Timer(
            rospy.Duration(1.0 / self.config.status_publish_rate_hz),
            self._publish_status,
        )
        rospy.loginfo(
            "[localization] fusion started: hector=%s gicp=%s pose=%s map=%s",
            self.backend_pose_topic,
            self.gicp_pose_topic,
            self.pose_topic,
            self.map_topic,
        )

    def _backend_pose_callback(self, message):
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
            with self.lock:
                result = self.pose_fusion.update_local(
                    message.header.stamp.to_sec(),
                    message.pose.pose.position.x,
                    message.pose.pose.position.y,
                    yaw,
                )
        except ValueError as exc:
            rospy.logerr_throttle(1.0, "[localization] invalid GICP pose: %s", str(exc))
            return

        pose = copy.deepcopy(message)
        pose.header.frame_id = self.map_frame
        pose.pose.pose.position.x = result.pose.x
        pose.pose.pose.position.y = result.pose.y
        pose.pose.pose.position.z = 0.0
        qx, qy, qz, qw = quaternion_from_rpy(0.0, 0.0, result.pose.yaw)
        pose.pose.pose.orientation.x = qx
        pose.pose.pose.orientation.y = qy
        pose.pose.pose.orientation.z = qz
        pose.pose.pose.orientation.w = qw
        gicp_healthy = self._gicp_covariance_healthy(message)
        self._set_output_covariance(pose, gicp_healthy)
        with self.lock:
            if gicp_healthy:
                self.gicp_consecutive_failures = 0
                self.last_gicp_pose_accepted = rospy.Time.now()
            else:
                self.gicp_consecutive_failures += 1
            self.latest_pose = pose
            self.latest_local_pose = Pose2D(
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                yaw,
            )
            self.latest_map_to_odom = result.correction
            self.last_gicp_fusion_reason = result.reason
            self.last_pose_received = rospy.Time.now()
            pending_hector_pose = self.pending_hector_pose
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
        if pending_hector_pose is not None:
            self._backend_pose_callback(pending_hector_pose)

    def _map_callback(self, message):
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
            self.latest_raw_map = copy.deepcopy(message)
        self._publish_cached_map_if_safe()

    def _publish_cached_map_if_safe(self, now=None):
        now = now or rospy.Time.now()
        with self.lock:
            map_correction_healthy = (
                self.pose_fusion.initialized
                and self.last_hector_update_accepted
            )
            gicp_healthy = (
                self.last_gicp_pose_accepted != rospy.Time(0)
                and self._age(now, self.last_gicp_pose_accepted)
                <= self.config.gicp_healthy_fresh_timeout_s
            )
            message = copy.deepcopy(self.latest_raw_map)
        if not map_correction_healthy or not gicp_healthy:
            rospy.logwarn_throttle(
                1.0,
                "[localization] withholding public map until GICP and Hector "
                "are healthy",
            )
            return
        if message is not None:
            with self.lock:
                stamp = message.header.stamp
                if stamp != self.last_map_stamp:
                    self.last_map_stamp = stamp
                    self.map_version += 1
                    self.map_update_count += 1
                    self.last_map_update = stamp or rospy.Time.now()
            self.map_pub.publish(message)

    def _imu_callback(self, message):
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
        pose_stamp = rospy.Time.now()
        with self.lock:
            pose = copy.deepcopy(self.latest_pose)
            vertical = self.vertical_estimator.snapshot()
            local_pose = self.latest_local_pose
            correction = self.latest_map_to_odom
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
        self.pose_pub.publish(pose)

        if local_pose is None:
            return
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
        if self.config.vertical_estimation_enabled and vertical.initialized:
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
        self.tf_broadcaster.sendTransform([map_to_odom, odom_to_base])

    def _publish_status(self, _event=None):
        now = rospy.Time.now()
        with self.lock:
            pose = copy.deepcopy(self.latest_pose)
            pose_age = self._age(now, self.last_pose_received)
            map_age = self._age(now, self.last_map_received)
            map_version = self.map_version
            map_update_count = self.map_update_count
            last_map_update = self.last_map_update
            vertical = self.vertical_estimator.snapshot()
            fusion_initialized = self.pose_fusion.initialized
            gicp_healthy_age = self._age(now, self.last_gicp_pose_accepted)
            hector_age = self._age(now, self.last_hector_pose_accepted)
            gicp_fusion_reason = self.last_gicp_fusion_reason
            hector_fusion_reason = self.last_hector_fusion_reason

        pose_fresh = pose_age <= self.config.pose_fresh_timeout_s
        map_fresh = map_age <= self.config.map_fresh_timeout_s
        hector_fresh = hector_age <= self.config.hector_pose_fresh_timeout_s
        gicp_degraded = (
            gicp_healthy_age > self.config.gicp_healthy_fresh_timeout_s
        )
        gicp_lost = (
            gicp_healthy_age > self.config.gicp_healthy_lost_timeout_s
        )
        hector_degraded = not hector_fresh
        ready = (
            pose is not None
            and pose_fresh
            and map_fresh
            and fusion_initialized
            and not gicp_lost
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
        )
        current_floor = (
            vertical.current_floor
            if vertical.initialized
            else self.config.current_floor
        )

        mapping = MappingStatus()
        mapping.header.stamp = now
        mapping.header.frame_id = self.map_frame
        mapping.ready = ready
        mapping.stable = stable
        mapping.lost = lost
        mapping.current_floor = current_floor
        if map_version > 0:
            floor = FloorMapInfo()
            floor.floor_id = current_floor
            floor.map_version = map_version
            floor.last_update = last_map_update
            mapping.floor_maps.append(floor)
        mapping.status_reason = reason
        self.mapping_status_pub.publish(mapping)

        localization = LocalizationStatus()
        localization.header = mapping.header
        localization.tracking_state = self._tracking_state(
            pose,
            pose_fresh,
            map_fresh and fusion_initialized,
            degraded=gicp_degraded or hector_degraded,
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
        self.localization_status_pub.publish(localization)

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
    ):
        if pose is None:
            return "WAITING_FOR_SCAN_MATCHING_POSE"
        if not pose_fresh:
            return "SCAN_MATCHING_POSE_STALE"
        if gicp_lost:
            return "GICP_ODOMETRY_LOST:" + gicp_fusion_reason
        if gicp_degraded:
            return "GICP_ODOMETRY_DEGRADED_HOLDING_LAST_POSE"
        if hector_degraded:
            return "HECTOR_CORRECTION_DEGRADED:" + hector_fusion_reason
        if not map_fresh:
            return "MAP_STALE"
        if not stable:
            return "WAITING_FOR_STABLE_MAP"
        if not vertical_fresh:
            return "VERTICAL_IMU_STALE"
        return "TRACKING_FUSED_GICP_ODOMETRY_WITH_BOUNDED_HECTOR_CORRECTION"

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
