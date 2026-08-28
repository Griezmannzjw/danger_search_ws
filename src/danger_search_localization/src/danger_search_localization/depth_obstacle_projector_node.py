"""ROS node that projects RealSense depth into a ground-filtered LaserScan."""

from collections import deque
from dataclasses import replace
import math
import threading

import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan

from .config import ScanProjectionConfig
from .depth_obstacle_projection import (
    DepthObstacleProjectionConfig,
    depth_image_to_optical_points,
    estimate_floor_z,
    select_ground_relative_obstacles,
)
from .scan_projection import (
    gravity_level_points,
    project_planar_scan,
    quaternion_inverse,
    quaternion_multiply,
    transform_points,
)


class DepthObstacleProjectorNode:
    def __init__(self):
        rospy.init_node("depth_obstacle_projector", anonymous=False)
        self.base_frame = rospy.get_param("~base_frame", "base")
        self.depth_topic = rospy.get_param(
            "~depth_obstacle_depth_topic", "/real_sense/depth/image_raw"
        )
        self.camera_info_topic = rospy.get_param(
            "~depth_obstacle_camera_info_topic", "/real_sense/depth/camera_info"
        )
        self.output_topic = rospy.get_param(
            "~depth_obstacle_scan_topic", "/localization/depth_obstacle_scan"
        )
        self.imu_topic = rospy.get_param("~imu_topic", "/trunk_imu")
        self.tf_timeout_s = float(rospy.get_param("~tf_timeout_s", 0.10))
        self.imu_fresh_timeout_s = float(
            rospy.get_param("~imu_fresh_timeout_s", 0.20)
        )
        self.enable_imu_leveling = bool(
            rospy.get_param("~enable_imu_leveling", True)
        )
        self.config = self._load_depth_config()
        self.scan_config = self._load_scan_config()

        self.lock = threading.RLock()
        self.intrinsics = None
        self.imu_samples = deque(maxlen=200)
        self.floor_z = None
        self.floor_stamp_s = None
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.publisher = rospy.Publisher(
            self.output_topic, LaserScan, queue_size=3
        )
        self.info_subscriber = rospy.Subscriber(
            self.camera_info_topic, CameraInfo, self._camera_info_callback,
            queue_size=2,
        )
        self.imu_subscriber = rospy.Subscriber(
            self.imu_topic, Imu, self._imu_callback, queue_size=100
        )
        self.depth_subscriber = rospy.Subscriber(
            self.depth_topic, Image, self._depth_callback, queue_size=1,
            buff_size=4 * 1024 * 1024,
        )
        rospy.loginfo(
            "[localization] depth near-field obstacles: %s -> %s in %s",
            self.depth_topic,
            self.output_topic,
            self.base_frame,
        )

    @staticmethod
    def _parameter(name, default):
        return rospy.get_param("~" + name, default)

    def _load_depth_config(self):
        p = self._parameter
        return DepthObstacleProjectionConfig(
            image_stride=int(p("depth_obstacle_image_stride", 4)),
            min_depth_m=float(p("depth_obstacle_min_range_m", 0.40)),
            max_depth_m=float(p("depth_obstacle_max_range_m", 3.00)),
            floor_search_min_z_m=float(
                p("depth_obstacle_floor_search_min_z_m", -0.65)
            ),
            floor_search_max_z_m=float(
                p("depth_obstacle_floor_search_max_z_m", -0.08)
            ),
            floor_histogram_bin_m=float(
                p("depth_obstacle_floor_histogram_bin_m", 0.02)
            ),
            floor_min_support_points=int(
                p("depth_obstacle_floor_min_support_points", 40)
            ),
            floor_max_jump_m=float(
                p("depth_obstacle_floor_max_jump_m", 0.08)
            ),
            floor_cache_timeout_s=float(
                p("depth_obstacle_floor_cache_timeout_s", 2.0)
            ),
            fallback_floor_z_m=float(
                p("depth_obstacle_fallback_floor_z_m", -0.30)
            ),
            min_obstacle_height_m=float(
                p("depth_obstacle_min_height_m", 0.06)
            ),
            max_obstacle_height_m=float(
                p("depth_obstacle_max_height_m", 1.20)
            ),
        )

    def _load_scan_config(self):
        p = self._parameter
        return ScanProjectionConfig(
            angle_min=float(p("angle_min", -math.pi)),
            angle_max=float(p("angle_max", math.pi)),
            angle_increment=float(p("angle_increment", math.radians(0.5))),
            range_min=self.config.min_depth_m,
            range_max=self.config.max_depth_m,
            min_height=-10.0,
            max_height=10.0,
            self_exclusion_min_x=float(p("self_exclusion_min_x", -0.55)),
            self_exclusion_max_x=float(p("self_exclusion_max_x", 0.55)),
            self_exclusion_half_width_y=float(
                p("self_exclusion_half_width_y", 0.40)
            ),
            min_returns_per_bin=1,
            max_intra_bin_range_gap=float(p("max_intra_bin_range_gap", 0.30)),
            enable_isolated_hit_filter=False,
            neighbor_window_bins=int(p("neighbor_window_bins", 8)),
            min_neighbor_support=1,
            max_neighbor_range_jump=float(p("max_neighbor_range_jump", 0.75)),
        )

    def _camera_info_callback(self, message):
        if len(message.K) != 9 or message.K[0] <= 0.0 or message.K[4] <= 0.0:
            return
        with self.lock:
            self.intrinsics = (
                float(message.K[0]),
                float(message.K[4]),
                float(message.K[2]),
                float(message.K[5]),
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
        )
        with self.lock:
            self.imu_samples.append(sample)

    def _closest_imu(self, stamp_s):
        with self.lock:
            if not self.imu_samples:
                return None
            sample = min(self.imu_samples, key=lambda value: abs(value[0] - stamp_s))
        if abs(sample[0] - stamp_s) > self.imu_fresh_timeout_s:
            return None
        return sample

    @staticmethod
    def _decode_depth(message):
        if message.encoding == "32FC1":
            dtype = np.dtype("<f4")
            scale = 1.0
        elif message.encoding in ("16UC1", "mono16"):
            dtype = np.dtype("<u2")
            scale = 0.001
        else:
            raise ValueError("unsupported depth encoding: " + message.encoding)
        dtype = dtype.newbyteorder(">" if message.is_bigendian else "<")
        row_values = message.step // dtype.itemsize
        image = np.frombuffer(message.data, dtype=dtype).reshape(
            message.height, row_values
        )[:, :message.width]
        return image.astype(np.float64, copy=False) * scale

    def _level_points(self, points, stamp):
        if not self.enable_imu_leveling:
            return points
        sample = self._closest_imu(stamp.to_sec())
        if sample is None:
            rospy.logwarn_throttle(
                2.0, "[localization] depth obstacle IMU unavailable"
            )
            return points
        _, imu_frame, world_from_imu = sample
        try:
            base_from_imu = self.tf_buffer.lookup_transform(
                self.base_frame,
                imu_frame,
                stamp,
                rospy.Duration(self.tf_timeout_s),
            ).transform.rotation
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
            levelled, _, _ = gravity_level_points(points, world_from_base)
            return levelled
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
            ValueError,
        ) as exc:
            rospy.logwarn_throttle(
                1.0, "[localization] depth obstacle leveling failed: %s", str(exc)
            )
            return points

    def _floor_for_frame(self, points, stamp_s):
        with self.lock:
            previous = self.floor_z
            previous_stamp = self.floor_stamp_s
        estimate = estimate_floor_z(points, self.config, previous)
        if estimate is not None:
            with self.lock:
                self.floor_z = estimate
                self.floor_stamp_s = stamp_s
            return estimate
        if (
            previous is not None
            and previous_stamp is not None
            and stamp_s - previous_stamp <= self.config.floor_cache_timeout_s
        ):
            return previous
        return self.config.fallback_floor_z_m

    def _depth_callback(self, message):
        if not message.header.frame_id:
            return
        with self.lock:
            intrinsics = self.intrinsics
        if intrinsics is None:
            rospy.logwarn_throttle(
                2.0, "[localization] depth obstacle camera info unavailable"
            )
            return
        try:
            depth = self._decode_depth(message)
            points_optical = depth_image_to_optical_points(
                depth, intrinsics, self.config
            )
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                message.header.frame_id,
                message.header.stamp,
                rospy.Duration(self.tf_timeout_s),
            ).transform
            points_base = transform_points(
                points_optical,
                (
                    transform.translation.x,
                    transform.translation.y,
                    transform.translation.z,
                ),
                (
                    transform.rotation.x,
                    transform.rotation.y,
                    transform.rotation.z,
                    transform.rotation.w,
                ),
            )
        except (
            ValueError,
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            rospy.logwarn_throttle(
                1.0, "[localization] depth obstacle projection failed: %s", str(exc)
            )
            return

        points_base = self._level_points(points_base, message.header.stamp)
        floor_z = self._floor_for_frame(points_base, message.header.stamp.to_sec())
        obstacles = select_ground_relative_obstacles(
            points_base, floor_z, self.config
        )
        dynamic_scan_config = replace(
            self.scan_config,
            min_height=floor_z + self.config.min_obstacle_height_m,
            max_height=floor_z + self.config.max_obstacle_height_m,
        )
        ranges = project_planar_scan(obstacles, dynamic_scan_config)

        output = LaserScan()
        output.header.stamp = message.header.stamp
        output.header.frame_id = self.base_frame
        output.angle_min = dynamic_scan_config.angle_min
        output.angle_max = dynamic_scan_config.angle_max
        output.angle_increment = dynamic_scan_config.angle_increment
        output.time_increment = 0.0
        output.scan_time = 0.1
        output.range_min = dynamic_scan_config.range_min
        output.range_max = dynamic_scan_config.range_max
        output.ranges = ranges.tolist()
        self.publisher.publish(output)
        rospy.loginfo_throttle(
            5.0,
            "[localization] depth obstacle scan: floor=%.3f points=%d bins=%d",
            floor_z,
            len(obstacles),
            int(np.isfinite(ranges).sum()),
        )

    @staticmethod
    def run():
        rospy.spin()

