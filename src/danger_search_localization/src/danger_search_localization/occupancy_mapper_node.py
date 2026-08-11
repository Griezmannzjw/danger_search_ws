"""ROS occupancy mapper using synchronized trusted GICP poses and scans."""

import copy
import math
import threading

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

from .occupancy_mapping import OccupancyMapperCore, OccupancyMappingConfig


class OccupancyMapperNode:
    def __init__(self):
        rospy.init_node("local_occupancy_mapper", anonymous=False)
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base")
        self.scan_topic = rospy.get_param(
            "~projected_scan_topic", "/localization/scan"
        )
        self.pose_topic = rospy.get_param(
            "~gicp_pose_topic", "/localization/raw_pose"
        )
        self.map_topic = rospy.get_param(
            "~raw_map_topic", "/localization/raw_map"
        )
        self.unhealthy_variance = float(
            rospy.get_param("~gicp_unhealthy_variance_threshold", 1.0)
        )
        self.publish_period = float(rospy.get_param("~map_pub_period", 1.0))
        self.config = OccupancyMappingConfig(
            resolution=float(rospy.get_param("~map_resolution", 0.05)),
            size=int(rospy.get_param("~map_size", 1024)),
            start_x=float(rospy.get_param("~map_start_x", 0.5)),
            start_y=float(rospy.get_param("~map_start_y", 0.5)),
            max_rays=int(rospy.get_param("~occupancy_mapper_max_rays", 360)),
            free_update=int(rospy.get_param("~occupancy_mapper_free_update", 1)),
            occupied_update=int(
                rospy.get_param("~occupancy_mapper_occupied_update", 4)
            ),
            min_score=int(rospy.get_param("~occupancy_mapper_min_score", -20)),
            max_score=int(rospy.get_param("~occupancy_mapper_max_score", 20)),
            occupied_score=int(
                rospy.get_param("~occupancy_mapper_occupied_score", 2)
            ),
            clear_radius_m=float(
                rospy.get_param("~occupancy_mapper_clear_radius_m", 0.35)
            ),
        )
        if self.publish_period <= 0.0:
            raise ValueError("map publication period must be positive")
        self.core = OccupancyMapperCore(self.config)
        self.lock = threading.RLock()
        self.pose_cache = {}
        self.scan_cache = {}
        self.last_scan_stamp = rospy.Time(0)
        self.map_dirty = False

        self.publisher = rospy.Publisher(
            self.map_topic, OccupancyGrid, queue_size=1, latch=True
        )
        self.pose_subscriber = rospy.Subscriber(
            self.pose_topic,
            PoseWithCovarianceStamped,
            self._pose_callback,
            queue_size=20,
        )
        self.scan_subscriber = rospy.Subscriber(
            self.scan_topic, LaserScan, self._scan_callback, queue_size=10
        )
        self.timer = rospy.Timer(
            rospy.Duration(self.publish_period), self._publish_map
        )
        rospy.loginfo(
            "[localization] trusted GICP occupancy mapper: %s + %s -> %s",
            self.pose_topic,
            self.scan_topic,
            self.map_topic,
        )

    @staticmethod
    def _key(stamp):
        return int(stamp.secs), int(stamp.nsecs)

    def _pose_callback(self, message):
        if message.header.frame_id != self.odom_frame:
            return
        covariance = message.pose.covariance
        if not all(
            math.isfinite(covariance[index])
            and covariance[index] < self.unhealthy_variance
            for index in (0, 7, 35)
        ):
            return
        orientation = message.pose.pose.orientation
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
        pose = (float(position.x), float(position.y), float(yaw))
        if not all(math.isfinite(value) for value in pose):
            return
        with self.lock:
            self.pose_cache[self._key(message.header.stamp)] = pose
            self._consume(self._key(message.header.stamp))
            self._prune_caches()

    def _scan_callback(self, message):
        if message.header.frame_id != self.base_frame:
            return
        with self.lock:
            self.scan_cache[self._key(message.header.stamp)] = copy.deepcopy(message)
            self._consume(self._key(message.header.stamp))
            self._prune_caches()

    def _consume(self, key):
        pose = self.pose_cache.get(key)
        scan = self.scan_cache.get(key)
        if pose is None or scan is None:
            return
        try:
            updated = self.core.update(pose, scan)
        except ValueError as exc:
            rospy.logwarn_throttle(
                1.0, "[localization] occupancy update rejected: %s", str(exc)
            )
            updated = False
        if updated:
            self.last_scan_stamp = scan.header.stamp
            self.map_dirty = True
        self.pose_cache.pop(key, None)
        self.scan_cache.pop(key, None)

    def _prune_caches(self):
        for cache in (self.pose_cache, self.scan_cache):
            if len(cache) <= 50:
                continue
            for key in sorted(cache)[:-50]:
                cache.pop(key, None)

    def _publish_map(self, _event=None):
        with self.lock:
            if self.core.update_count == 0:
                return
            message = OccupancyGrid()
            message.header.stamp = self.last_scan_stamp
            message.header.frame_id = self.map_frame
            message.info.map_load_time = self.last_scan_stamp
            message.info.resolution = self.config.resolution
            message.info.width = self.config.size
            message.info.height = self.config.size
            message.info.origin.position.x = self.core.origin_x
            message.info.origin.position.y = self.core.origin_y
            message.info.origin.orientation.w = 1.0
            message.data = self.core.occupancy_data()
            self.map_dirty = False
        self.publisher.publish(message)

    @staticmethod
    def run():
        rospy.spin()
