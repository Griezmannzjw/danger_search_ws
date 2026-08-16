"""ROS wrapper for IMU-levelled PointCloud to LaserScan projection."""

from collections import deque
import math
import threading

import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import Imu, LaserScan, PointCloud

from .config import ScanProjectionConfig
from .scan_projection import (
    estimate_ground_clearance,
    gravity_level_points,
    merge_scan_history,
    project_planar_scan,
    quaternion_inverse,
    quaternion_multiply,
    TimedScanAccumulator,
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
        self.scan_accumulator = TimedScanAccumulator(
            self.config.scan_accumulation_frames,
            self.config.scan_accumulation_max_age_s,
        )
        self.last_published_stamp_s = None
        self.pending_output_scan = None

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
                        frame_ranges = project_planar_scan(
                            points_base, self.config
                        )
                        self._accumulate_and_publish(message, frame_ranges)
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
        frame_ranges = project_planar_scan(points_base, self.config)
        self._accumulate_and_publish(message, frame_ranges)

    def _accumulate_and_publish(self, message, frame_ranges):
        try:
            reset = self.scan_accumulator.add(
                message.header.stamp.to_sec(), frame_ranges
            )
        except ValueError as exc:
            rospy.logwarn_throttle(
                1.0, "[localization] invalid projected scan: %s", str(exc)
            )
            return
        if reset:
            rospy.logwarn_throttle(
                2.0,
                "[localization] scan history reset after time discontinuity",
            )
        ranges = merge_scan_history(
            self.scan_accumulator.scans,
            self.config.scan_accumulation_min_samples_per_bin,
        )

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
        output.header.stamp = message.header.stamp
        output.header.frame_id = self.base_frame
        output.angle_min = self.config.angle_min
        output.angle_max = self.config.angle_max
        output.angle_increment = self.config.angle_increment
        stamp_s = message.header.stamp.to_sec()
        output.scan_time = self._published_scan_interval(stamp_s)
        output.time_increment = 0.0
        output.range_min = self.config.range_min
        output.range_max = self.config.range_max
        output.ranges = ranges.tolist()
        # Delay the projected scan by one input frame.  The local odometry
        # callback publishes map->odom->base for the scan's exact timestamp;
        # this small buffer guarantees that TF is already available when
        # Hector consumes the scan in map_with_known_poses mode.
        pending = self.pending_output_scan
        self.pending_output_scan = output
        if pending is not None:
            self.publisher.publish(pending)

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
        """Return the smallest circular arc containing all finite bins.

        A mechanical 2-D lidar normally fills consecutive angle bins, but a
        sparse non-repeating Mid-360 projection does not.  Measuring only the
        longest consecutive run incorrectly labels a well-distributed 360
        degree cloud as a 1--2 degree scan.  The complement of the largest
        circular gap preserves the intended safety check: clustered returns
        are still rejected while sparse full-FOV returns are accepted.
        """
        finite = np.asarray(finite_mask, dtype=bool)
        if not np.any(finite):
            return 0.0
        indices = np.flatnonzero(finite)
        if indices.size == finite.size:
            return float(finite.size) * self.config.angle_increment
        circular_indices = np.append(indices, indices[0] + finite.size)
        largest_step = int(np.max(np.diff(circular_indices)))
        covered_bins = int(finite.size) - largest_step + 1
        return float(covered_bins) * self.config.angle_increment

    @staticmethod
    def run():
        rospy.spin()
