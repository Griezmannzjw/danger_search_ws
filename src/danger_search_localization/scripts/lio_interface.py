#!/usr/bin/env python3
"""Expose the LIO backend through the stable team localization interface."""

import copy
import math
import threading

import rospy
import tf.transformations as transformations
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry

from danger_search_common.msg import FloorMapInfo, LocalizationStatus, MappingStatus


class LioInterface:
    def __init__(self):
        rospy.init_node("localization_adapter", anonymous=False)
        self.lock = threading.RLock()
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base")
        self.current_floor = rospy.get_param("~current_floor", 0)
        self.pose_timeout = rospy.get_param("~pose_fresh_timeout_s", 1.0)
        self.map_timeout = rospy.get_param("~map_fresh_timeout_s", 5.0)
        self.max_speed = rospy.get_param("~lio_guard_max_linear_speed_mps", 2.0)
        self.jump_margin = rospy.get_param("~lio_guard_translation_margin_m", 0.15)
        self.max_yaw_rate = rospy.get_param("~lio_guard_max_yaw_rate_rps", 3.0)
        self.yaw_margin = rospy.get_param("~lio_guard_yaw_margin_rad", 0.20)
        self.latest_pose = None
        self.latest_stamp = rospy.Time(0)
        self.last_raw = None
        self.anchor_position = None
        self.anchor_inverse = None
        self.last_map_stamp = rospy.Time(0)
        self.map_version = 0
        self.rejected = 0
        self.ever_ready = False

        self.pose_pub = rospy.Publisher("/localization/pose", PoseWithCovarianceStamped,
                                        queue_size=10)
        self.localization_pub = rospy.Publisher("/localization/status", LocalizationStatus,
                                                queue_size=5, latch=True)
        self.mapping_pub = rospy.Publisher("/mapping/status", MappingStatus,
                                           queue_size=5, latch=True)
        self.tf_pub = tf2_ros.TransformBroadcaster()
        rospy.Subscriber("/localization/lio/odometry", Odometry, self.odom_callback,
                         queue_size=20)
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback, queue_size=1)
        rospy.Timer(rospy.Duration(0.05), self.publish_pose)
        rospy.Timer(rospy.Duration(0.5), self.publish_status)
        rospy.loginfo("[localization] LIO interface started")

    @staticmethod
    def yaw(quaternion):
        return transformations.euler_from_quaternion(
            [quaternion.x, quaternion.y, quaternion.z, quaternion.w])[2]

    @staticmethod
    def angle_delta(a, b):
        return math.atan2(math.sin(a - b), math.cos(a - b))

    def odom_callback(self, message):
        p = message.pose.pose.position
        q = message.pose.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(value) for value in values):
            rospy.logwarn_throttle(1.0, "[localization] rejected non-finite LIO pose")
            return
        raw_q = [q.x, q.y, q.z, q.w]
        with self.lock:
            if self.anchor_position is None:
                self.anchor_position = [p.x, p.y, p.z]
                self.anchor_inverse = transformations.quaternion_inverse(raw_q)
            relative = transformations.quaternion_multiply(self.anchor_inverse, raw_q)
            delta_world = [p.x - self.anchor_position[0], p.y - self.anchor_position[1],
                           p.z - self.anchor_position[2]]
            delta = transformations.quaternion_matrix(self.anchor_inverse).dot(
                [delta_world[0], delta_world[1], delta_world[2], 0.0])[:3]
            current = (message.header.stamp.to_sec(), delta, relative, self.yaw(q))
            if self.last_raw is not None:
                dt = current[0] - self.last_raw[0]
                if dt <= 0.0:
                    return
                distance = math.sqrt(sum((delta[i] - self.last_raw[1][i]) ** 2
                                         for i in range(3)))
                yaw_step = abs(self.angle_delta(current[3], self.last_raw[3]))
                if (distance > self.jump_margin + self.max_speed * min(dt, 0.5) or
                        yaw_step > self.yaw_margin + self.max_yaw_rate * min(dt, 0.5)):
                    self.rejected += 1
                    rospy.logwarn_throttle(
                        1.0, "[localization] rejected LIO jump: %.3fm %.3frad", distance,
                        yaw_step)
                    return
            self.last_raw = current
            pose = PoseWithCovarianceStamped()
            pose.header.stamp = message.header.stamp
            pose.header.frame_id = self.map_frame
            pose.pose.pose.position.x, pose.pose.pose.position.y, pose.pose.pose.position.z = delta
            pose.pose.pose.orientation.x = relative[0]
            pose.pose.pose.orientation.y = relative[1]
            pose.pose.pose.orientation.z = relative[2]
            pose.pose.pose.orientation.w = relative[3]
            pose.pose.covariance = list(message.pose.covariance)
            if not any(value > 0.0 for value in pose.pose.covariance):
                pose.pose.covariance[0] = pose.pose.covariance[7] = 0.02
                pose.pose.covariance[14] = 0.04
                pose.pose.covariance[21] = pose.pose.covariance[28] = 0.03
                pose.pose.covariance[35] = 0.03
            self.latest_pose = pose
            self.latest_stamp = rospy.Time.now()

    def map_callback(self, message):
        with self.lock:
            if message.header.stamp != self.last_map_stamp:
                self.last_map_stamp = message.header.stamp
                self.map_version += 1

    def publish_pose(self, _event):
        with self.lock:
            pose = copy.deepcopy(self.latest_pose)
        if pose is None:
            return
        now = rospy.Time.now()
        pose.header.stamp = now
        self.pose_pub.publish(pose)
        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = now
        map_to_odom.header.frame_id = self.map_frame
        map_to_odom.child_frame_id = self.odom_frame
        map_to_odom.transform.rotation.w = 1.0
        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = now
        odom_to_base.header.frame_id = self.odom_frame
        odom_to_base.child_frame_id = self.base_frame
        odom_to_base.transform.translation.x = pose.pose.pose.position.x
        odom_to_base.transform.translation.y = pose.pose.pose.position.y
        odom_to_base.transform.translation.z = pose.pose.pose.position.z
        odom_to_base.transform.rotation = pose.pose.pose.orientation
        self.tf_pub.sendTransform([map_to_odom, odom_to_base])

    def publish_status(self, _event):
        now = rospy.Time.now()
        with self.lock:
            pose = copy.deepcopy(self.latest_pose)
            pose_age = (now - self.latest_stamp).to_sec() if self.latest_stamp else math.inf
            map_age = (now - self.last_map_stamp).to_sec() if self.last_map_stamp else math.inf
            version = self.map_version
            rejected = self.rejected
        pose_fresh = pose is not None and pose_age <= self.pose_timeout
        map_fresh = version > 0 and map_age <= self.map_timeout
        ready = pose_fresh and map_fresh
        self.ever_ready = self.ever_ready or ready
        reason = "LIO_TRACKING" if ready else ("WAITING_FOR_LIO" if not pose_fresh else
                                                "WAITING_FOR_MAP")
        localization = LocalizationStatus()
        localization.header.stamp = now
        localization.header.frame_id = self.map_frame
        localization.tracking_state = (LocalizationStatus.STATE_TRACKING if ready else
                                       LocalizationStatus.STATE_INITIALIZING)
        localization.pose_covariance_trace = (sum(pose.pose.covariance[i]
                                                  for i in (0, 7, 14, 21, 28, 35))
                                                if pose else math.inf)
        localization.drift_warning = not ready
        localization.pose_jump_detected = rejected > 0
        localization.status_reason = reason
        if ready:
            localization.last_stable_time = now
        self.localization_pub.publish(localization)
        mapping = MappingStatus()
        mapping.header = localization.header
        mapping.ready = ready
        mapping.stable = ready and version >= 2
        mapping.lost = self.ever_ready and not pose_fresh
        mapping.current_floor = self.current_floor
        mapping.status_reason = reason
        if version:
            floor = FloorMapInfo()
            floor.floor_id = self.current_floor
            floor.map_version = version
            floor.last_update = self.last_map_stamp
            mapping.floor_maps.append(floor)
        self.mapping_pub.publish(mapping)


if __name__ == "__main__":
    try:
        LioInterface()
        rospy.spin()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
