"""Expose a SLAM backend through the team localization interface."""

import copy
import math
import threading
import zlib

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
from .pose_filter import PoseStabilizer
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
        self.pose_filter = PoseStabilizer(self.config)
        self.vertical_estimator = VerticalEstimator(self.config)

        self.lock = threading.RLock()
        self.latest_pose = None
        self.last_pose_received = rospy.Time(0)
        self.last_map_received = rospy.Time(0)
        self.last_map_update = rospy.Time(0)
        self.map_version = 0
        self.map_update_count = 0
        self.map_checksum = None
        self.ever_ready = False
        self.base_from_imu_quaternion = None

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self._publish_identity_map_to_odom()

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
        self.pose_sub = rospy.Subscriber(
            self.backend_pose_topic,
            PoseWithCovarianceStamped,
            self._pose_callback,
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
            "[localization] adapter started: backend_pose=%s pose=%s map=%s",
            self.backend_pose_topic,
            self.pose_topic,
            self.map_topic,
        )

    def _publish_identity_map_to_odom(self):
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(transform)

    def _pose_callback(self, message):
        if message.header.frame_id != self.map_frame:
            rospy.logwarn_throttle(
                2.0,
                "[localization] rejecting pose in frame '%s', expected '%s'",
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
        with self.lock:
            result = self.pose_filter.update(
                message.header.stamp.to_sec(),
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                yaw,
            )
        if not result.accepted:
            now = rospy.Time.now()
            if self.pose_filter.needs_recovery(
                now.to_sec(), self.config.pose_recovery_timeout_s
            ):
                rospy.logwarn(
                    "[localization] recovering from prolonged rejection, "
                    "re-anchoring at raw pose (%.2f, %.2f, %.1fdeg)",
                    message.pose.pose.position.x,
                    message.pose.pose.position.y,
                    math.degrees(yaw),
                )
                self.pose_filter.recover(
                    message.header.stamp.to_sec(),
                    message.pose.pose.position.x,
                    message.pose.pose.position.y,
                    yaw,
                )
                with self.lock:
                    result = self.pose_filter.snapshot(True)
            else:
                rospy.logerr_throttle(
                    1.0,
                    "[localization] rejecting unsafe backend pose: %s "
                    "(consecutive=%d)",
                    result.reason,
                    result.consecutive_rejections,
                )
                return

        pose = copy.deepcopy(message)
        pose.pose.pose.position.x = result.pose.x
        pose.pose.pose.position.y = result.pose.y
        pose.pose.pose.position.z = 0.0
        qx, qy, qz, qw = quaternion_from_rpy(0.0, 0.0, result.pose.yaw)
        pose.pose.pose.orientation.x = qx
        pose.pose.pose.orientation.y = qy
        pose.pose.pose.orientation.z = qz
        pose.pose.pose.orientation.w = qw
        self._ensure_covariance(pose)
        with self.lock:
            self.latest_pose = pose
            self.last_pose_received = rospy.Time.now()

    def _map_callback(self, message):
        if message.header.frame_id != self.map_frame:
            rospy.logwarn_throttle(
                2.0,
                "[localization] rejecting map in frame '%s', expected '%s'",
                message.header.frame_id,
                self.map_frame,
            )
            return
        checksum = self._map_checksum(message)
        now = rospy.Time.now()
        with self.lock:
            self.last_map_received = now
            if checksum != self.map_checksum:
                self.map_checksum = checksum
                self.map_version += 1
                self.map_update_count += 1
                self.last_map_update = message.header.stamp or now
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
        with self.lock:
            pose = copy.deepcopy(self.latest_pose)
            vertical = self.vertical_estimator.snapshot()
        if pose is None:
            return
        self._apply_vertical_state(pose, vertical)
        self.pose_pub.publish(pose)

        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = pose.pose.pose.position.x
        transform.transform.translation.y = pose.pose.pose.position.y
        transform.transform.translation.z = pose.pose.pose.position.z
        transform.transform.rotation = pose.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

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
            pose_filter = self.pose_filter.snapshot()

        pose_fresh = pose_age <= self.config.pose_fresh_timeout_s
        map_fresh = map_age <= self.config.map_fresh_timeout_s
        ready = pose is not None and pose_fresh and map_fresh
        pose_filter_healthy = (
            pose_filter.consecutive_rejections
            < self.config.pose_rejections_before_lost
        )
        stable = (
            ready
            and pose_filter_healthy
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
        lost = self.ever_ready and (not pose_fresh or not pose_filter_healthy)
        self.ever_ready = self.ever_ready or ready
        reason = self._status_reason(
            pose,
            pose_fresh,
            map_fresh,
            stable,
            vertical_fresh,
            pose_filter_healthy,
            pose_filter.reason,
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
            pose, pose_fresh, map_fresh
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
    def _map_checksum(message):
        metadata = "{}:{}:{:.9f}:{:.6f}:{:.6f}".format(
            message.info.width,
            message.info.height,
            message.info.resolution,
            message.info.origin.position.x,
            message.info.origin.position.y,
        ).encode("ascii")
        encoded_cells = bytes((int(value) + 1) & 0xFF for value in message.data)
        return zlib.crc32(encoded_cells, zlib.crc32(metadata))

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
        pose_filter_healthy=True,
        pose_filter_reason="",
    ):
        if pose is None:
            return "WAITING_FOR_SCAN_MATCHING_POSE"
        if not pose_fresh:
            return "SCAN_MATCHING_POSE_STALE"
        if not pose_filter_healthy:
            return "SCAN_MATCHING_REJECTED:" + pose_filter_reason
        if not map_fresh:
            return "MAP_STALE"
        if not stable:
            return "WAITING_FOR_STABLE_MAP"
        if not vertical_fresh:
            return "VERTICAL_IMU_STALE"
        return "TRACKING_FILTERED_2D_POSE_FIXED_COVARIANCE_NO_LOOP_CLOSURE"

    @staticmethod
    def _tracking_state(pose, pose_fresh, map_fresh):
        if pose is None:
            return LocalizationStatus.STATE_INITIALIZING
        if not pose_fresh:
            return LocalizationStatus.STATE_LOST
        if not map_fresh:
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
