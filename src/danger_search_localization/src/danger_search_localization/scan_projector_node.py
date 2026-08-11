"""ROS wrapper for IMU-levelled PointCloud to LaserScan projection."""

from collections import deque
import math
import threading

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu, LaserScan, PointCloud

from .config import ScanProjectionConfig
from .scan_projection import (
    estimate_ground_clearance,
    gravity_level_points,
    interpolate_planar_pose,
    PoseCompensatedPointAccumulator,
    project_planar_scan,
    quaternion_inverse,
    quaternion_multiply,
    transform_points,
)


class ScanProjectorNode:
    def __init__(self):
        rospy.init_node("local_scan_projector", anonymous=False)
        self.input_topic = rospy.get_param("~raw_scan_topic", "/scan")
        self.output_topic = rospy.get_param(
            "~projected_scan_topic", "/localization/scan"
        )
        self.base_frame = rospy.get_param("~base_frame", "base")
        self.config = self._load_config()
        self.imu_lock = threading.RLock()
        self.imu_samples = deque(maxlen=500)
        self.point_accumulator = PoseCompensatedPointAccumulator(
            self.config.scan_accumulation_frames,
            self.config.scan_accumulation_max_age_s,
        )
        self.last_published_stamp_s = None
        self.gicp_pose_topic = rospy.get_param(
            "~gicp_pose_topic", "/localization/raw_pose"
        )
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.gicp_unhealthy_variance_threshold = float(
            rospy.get_param("~gicp_unhealthy_variance_threshold", 1.0)
        )
        self.sync_lock = threading.RLock()
        self.pending_point_frames = []
        self.pending_poses = []
        self.last_healthy_pose = None
        self.last_consumed_point_stamp_s = None

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.publisher = rospy.Publisher(
            self.output_topic, LaserScan, queue_size=5
        )
        self.imu_subscriber = rospy.Subscriber(
            self.config.imu_topic, Imu, self._imu_callback, queue_size=200
        )
        self.subscriber = rospy.Subscriber(
            self.input_topic, PointCloud, self._cloud_callback, queue_size=2
        )
        self.gicp_subscriber = rospy.Subscriber(
            self.gicp_pose_topic,
            PoseWithCovarianceStamped,
            self._gicp_pose_callback,
            queue_size=20,
        )
        rospy.loginfo(
            "[localization] projecting official %s to %s in frame %s",
            self.input_topic,
            self.output_topic,
            self.base_frame,
        )

    def _imu_callback(self, message):
        sample = (
            message.header.stamp.to_sec(),
            message.header.frame_id,
            (
                message.orientation.x,
                message.orientation.y,
                message.orientation.z,
                message.orientation.w,
            ),
            (
                message.angular_velocity.x,
                message.angular_velocity.y,
                message.angular_velocity.z,
            ),
        )
        with self.imu_lock:
            self.imu_samples.append(sample)

    def _cloud_callback(self, message):
        if not message.header.frame_id:
            rospy.logwarn_throttle(2.0, "[localization] /scan has no frame_id")
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                message.header.frame_id,
                message.header.stamp,
                rospy.Duration(self.config.tf_timeout_s),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            rospy.logwarn_throttle(
                1.0, "[localization] scan extrinsic TF unavailable: %s", str(exc)
            )
            return

        points = np.asarray(
            [(point.x, point.y, point.z) for point in message.points],
            dtype=np.float64,
        )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        try:
            points_base = transform_points(
                points,
                (translation.x, translation.y, translation.z),
                (rotation.x, rotation.y, rotation.z, rotation.w),
            )
        except ValueError as exc:
            rospy.logwarn_throttle(
                1.0, "[localization] invalid lidar transform: %s", str(exc)
            )
            return
        if self.config.enable_imu_leveling:
            imu_sample = self._closest_imu(message.header.stamp.to_sec())
            if imu_sample is None:
                rospy.logwarn_throttle(
                    2.0, "[localization] IMU unavailable, skipping leveling"
                )
            else:
                _, imu_frame, world_from_imu, angular_velocity = imu_sample
                try:
                    base_from_imu = self.tf_buffer.lookup_transform(
                        self.base_frame,
                        imu_frame,
                        message.header.stamp,
                        rospy.Duration(self.config.tf_timeout_s),
                    ).transform.rotation
                except (
                    tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException,
                ) as exc:
                    rospy.logwarn_throttle(
                        1.0, "[localization] IMU extrinsic TF unavailable: %s", str(exc)
                    )
                    base_from_imu = None
                if base_from_imu is not None:
                    try:
                        world_from_base = quaternion_multiply(
                            world_from_imu,
                            quaternion_inverse(
                                (
                                    base_from_imu.x,
                                    base_from_imu.y,
                                    base_from_imu.z,
                                    base_from_imu.w,
                                )
                            ),
                        )
                        points_base, roll, pitch = gravity_level_points(
                            points_base, world_from_base
                        )
                    except ValueError as exc:
                        rospy.logwarn_throttle(
                            1.0,
                            "[localization] invalid IMU orientation: %s",
                            str(exc),
                        )
                        roll = pitch = None
                    if roll is None:
                        self._queue_points(message, points_base)
                        return
                    angular_speed = math.sqrt(
                        sum(float(value) ** 2 for value in angular_velocity)
                    )
                    if (
                        abs(roll) > self.config.max_abs_roll_rad
                        or abs(pitch) > self.config.max_abs_pitch_rad
                        or not math.isfinite(angular_speed)
                        or angular_speed > self.config.max_angular_speed_rps
                    ):
                        rospy.logwarn_throttle(
                            1.0,
                            "[localization] unstable scan: roll=%.1fdeg "
                            "pitch=%.1fdeg gyro=%.2frad/s",
                            math.degrees(roll),
                            math.degrees(pitch),
                            angular_speed,
                        )
                        if self.config.drop_unstable_scans:
                            return
                    if self.config.enable_ground_clearance_gate:
                        clearance = estimate_ground_clearance(points_base, self.config)
                        if (
                            clearance is not None
                            and clearance < self.config.min_ground_clearance_m
                        ):
                            rospy.logwarn_throttle(
                                1.0,
                                "[localization] low ground clearance=%.3fm "
                                "-- rejecting scan",
                                clearance,
                            )
                            if self.config.drop_unstable_scans:
                                return
        self._queue_points(message, points_base)

    def _queue_points(self, message, points_base):
        stamp_s = message.header.stamp.to_sec()
        if not math.isfinite(stamp_s) or not np.isfinite(points_base).all():
            return
        with self.sync_lock:
            self.pending_point_frames.append(
                (stamp_s, message.header, np.asarray(points_base).copy())
            )
            self.pending_point_frames.sort(key=lambda value: value[0])
            newest = self.pending_point_frames[-1][0]
            self.pending_point_frames = [
                frame
                for frame in self.pending_point_frames[-50:]
                if newest - frame[0]
                <= max(2.0, 3.0 * self.config.scan_accumulation_max_age_s)
            ]
            self._drain_synchronized_observations()

    def _gicp_pose_callback(self, message):
        if message.header.frame_id != self.odom_frame:
            return
        covariance = message.pose.covariance
        if not all(
            math.isfinite(covariance[index])
            and covariance[index] < self.gicp_unhealthy_variance_threshold
            for index in (0, 7, 35)
        ):
            return
        orientation = message.pose.pose.orientation
        norm = math.sqrt(
            orientation.x * orientation.x
            + orientation.y * orientation.y
            + orientation.z * orientation.z
            + orientation.w * orientation.w
        )
        if not math.isfinite(norm) or norm < 1e-9:
            return
        yaw = math.atan2(
            2.0
            * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0
            - 2.0
            * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )
        position = message.pose.pose.position
        values = (
            message.header.stamp.to_sec(),
            (float(position.x), float(position.y), float(yaw)),
        )
        if not all(math.isfinite(value) for value in (values[0], *values[1])):
            return
        with self.sync_lock:
            self.pending_poses.append(values)
            self.pending_poses.sort(key=lambda value: value[0])
            self.pending_poses = self.pending_poses[-20:]
            self._drain_synchronized_observations()

    def _drain_synchronized_observations(self):
        while self.pending_poses and self.pending_point_frames:
            pose_stamp_s, current_pose = self.pending_poses[0]
            has_current_cloud = any(
                abs(frame[0] - pose_stamp_s) <= 1e-6
                for frame in self.pending_point_frames
            )
            if not has_current_cloud:
                if self.pending_point_frames[-1][0] > pose_stamp_s + 1e-6:
                    self.pending_poses.pop(0)
                    continue
                break

            self.pending_poses.pop(0)
            usable_frames = [
                frame
                for frame in self.pending_point_frames
                if frame[0] <= pose_stamp_s + 1e-6
                and (
                    self.last_consumed_point_stamp_s is None
                    or frame[0] > self.last_consumed_point_stamp_s + 1e-9
                )
            ]
            if not usable_frames:
                continue
            previous = self.last_healthy_pose
            for stamp_s, _, points in usable_frames:
                if previous is None or pose_stamp_s <= previous[0]:
                    frame_pose = current_pose
                else:
                    ratio = (stamp_s - previous[0]) / (
                        pose_stamp_s - previous[0]
                    )
                    frame_pose = interpolate_planar_pose(
                        previous[1], current_pose, ratio
                    )
                try:
                    reset = self.point_accumulator.add(
                        stamp_s, frame_pose, points
                    )
                except ValueError as exc:
                    rospy.logwarn_throttle(
                        1.0,
                        "[localization] invalid compensated scan input: %s",
                        str(exc),
                    )
                    continue
                if reset:
                    rospy.logwarn_throttle(
                        2.0,
                        "[localization] compensated scan history reset",
                    )
                self.last_consumed_point_stamp_s = stamp_s

            self.last_healthy_pose = (pose_stamp_s, current_pose)
            self.pending_point_frames = [
                frame
                for frame in self.pending_point_frames
                if frame[0] > pose_stamp_s + 1e-6
            ]
            header = usable_frames[-1][1]
            header.stamp = rospy.Time.from_sec(pose_stamp_s)
            self._publish_compensated_scan(header)

    def _publish_compensated_scan(self, header):
        points = self.point_accumulator.points_in_latest_frame()
        ranges = project_planar_scan(points, self.config)

        finite_mask = np.isfinite(ranges)
        valid_bins = int(np.sum(finite_mask))
        if valid_bins < self.config.min_valid_scan_bins:
            rospy.logwarn_throttle(
                2.0,
                "[localization] dropping sparse scan: %d valid bins < %d",
                valid_bins,
                self.config.min_valid_scan_bins,
            )
            return
        angular_coverage = self._angular_coverage_of_finite(ranges, finite_mask)
        if angular_coverage < self.config.min_angular_coverage_rad:
            rospy.logwarn_throttle(
                2.0,
                "[localization] dropping narrow scan: %.2f rad coverage < %.2f",
                angular_coverage,
                self.config.min_angular_coverage_rad,
            )
            return

        output = LaserScan()
        output.header.stamp = header.stamp
        output.header.frame_id = self.base_frame
        output.angle_min = self.config.angle_min
        output.angle_max = self.config.angle_max
        output.angle_increment = self.config.angle_increment
        stamp_s = header.stamp.to_sec()
        output.scan_time = self._published_scan_interval(stamp_s)
        output.time_increment = 0.0
        output.range_min = self.config.range_min
        output.range_max = self.config.range_max
        output.ranges = ranges.tolist()
        self.publisher.publish(output)

    def _published_scan_interval(self, stamp_s):
        interval = 0.0
        if self.last_published_stamp_s is not None:
            candidate = float(stamp_s) - self.last_published_stamp_s
            if candidate > 0.0 and math.isfinite(candidate):
                interval = candidate
        self.last_published_stamp_s = float(stamp_s)
        return interval

    def _closest_imu(self, stamp_s):
        with self.imu_lock:
            if not self.imu_samples:
                return None
            sample = min(
                self.imu_samples, key=lambda value: abs(value[0] - stamp_s)
            )
        if abs(sample[0] - stamp_s) > self.config.imu_fresh_timeout_s:
            return None
        return sample

    @staticmethod
    def _load_config():
        defaults = ScanProjectionConfig()
        return ScanProjectionConfig(
            **{
                name: rospy.get_param("~" + name, value)
                for name, value in vars(defaults).items()
            }
        )

    def _angular_coverage_of_finite(self, ranges, finite_mask):
        """Return the maximum angular span (radians) of consecutive finite bins."""
        finite = np.asarray(finite_mask, dtype=bool)
        if not np.any(finite):
            return 0.0
        doubled = np.concatenate([finite, finite])
        max_run = 0
        current = 0
        for is_finite in doubled:
            if is_finite:
                current += 1
                max_run = max(max_run, current)
            else:
                current = 0
            if current >= finite.size:
                return 2.0 * math.pi
        return float(max_run) * self.config.angle_increment

    @staticmethod
    def run():
        rospy.spin()
