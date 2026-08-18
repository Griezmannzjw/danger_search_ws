#!/usr/bin/env python3
"""Test-only Gazebo truth source for the canonical localization pipeline."""

import math
import threading

import rospy
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud

from danger_search_localization.gazebo_truth import (
    GazeboTruthCore,
    quaternion_yaw,
)


class GazeboTruthOdometry:
    def __init__(self):
        self.link_states_topic = rospy.get_param(
            "~gazebo_link_states_topic", "/gazebo/link_states"
        )
        self.base_link = rospy.get_param(
            "~gazebo_base_link", "a1_gazebo::base"
        )
        self.scan_topic = rospy.get_param("~raw_scan_topic", "/scan")
        self.output_topic = rospy.get_param(
            "~gicp_pose_topic", "/localization/raw_pose"
        )
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.xy_variance = self._positive_param("~gazebo_truth_xy_variance", 1e-4)
        self.yaw_variance = self._positive_param(
            "~gazebo_truth_yaw_variance", 1e-4
        )
        self.core = GazeboTruthCore(
            max_age_s=self._positive_param("~gazebo_truth_max_age_s", 0.20),
            max_future_s=self._nonnegative_param(
                "~gazebo_truth_max_future_s", 0.05
            ),
        )
        self.lock = threading.RLock()
        self.publisher = rospy.Publisher(
            self.output_topic, PoseWithCovarianceStamped, queue_size=10
        )
        self.link_subscriber = rospy.Subscriber(
            self.link_states_topic, LinkStates, self._link_states_callback, queue_size=2
        )
        self.scan_subscriber = rospy.Subscriber(
            self.scan_topic, PointCloud, self._scan_callback, queue_size=2
        )
        rospy.logwarn(
            "[localization] TEST MODE: Gazebo truth %s from %s -> %s",
            self.base_link,
            self.link_states_topic,
            self.output_topic,
        )

    @staticmethod
    def _positive_param(name, default):
        value = float(rospy.get_param(name, default))
        if not math.isfinite(value) or value <= 0.0:
            raise rospy.ROSInitException("%s must be positive and finite" % name)
        return value

    @staticmethod
    def _nonnegative_param(name, default):
        value = float(rospy.get_param(name, default))
        if not math.isfinite(value) or value < 0.0:
            raise rospy.ROSInitException("%s must be non-negative and finite" % name)
        return value

    def _link_states_callback(self, message):
        try:
            index = message.name.index(self.base_link)
        except ValueError:
            rospy.logwarn_throttle(
                2.0, "[localization] Gazebo link '%s' is unavailable", self.base_link
            )
            return
        if index >= len(message.pose):
            rospy.logwarn_throttle(2.0, "[localization] malformed Gazebo LinkStates")
            return
        pose = message.pose[index]
        try:
            yaw = quaternion_yaw(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
            with self.lock:
                self.core.update(
                    rospy.Time.now().to_sec(),
                    pose.position.x,
                    pose.position.y,
                    yaw,
                )
        except ValueError as exc:
            rospy.logwarn_throttle(1.0, "[localization] invalid Gazebo truth: %s", exc)

    def _scan_callback(self, scan):
        stamp = scan.header.stamp
        if stamp.is_zero():
            rospy.logwarn_throttle(2.0, "[localization] /scan has a zero timestamp")
            return
        with self.lock:
            pose = self.core.pose_at(stamp.to_sec())
        if pose is None:
            rospy.logwarn_throttle(
                1.0, "[localization] no fresh Gazebo truth for scan timestamp"
            )
            return
        output = PoseWithCovarianceStamped()
        output.header.stamp = stamp
        output.header.frame_id = self.odom_frame
        output.pose.pose.position.x = pose[0]
        output.pose.pose.position.y = pose[1]
        output.pose.pose.orientation.z = math.sin(0.5 * pose[2])
        output.pose.pose.orientation.w = math.cos(0.5 * pose[2])
        output.pose.covariance[0] = self.xy_variance
        output.pose.covariance[7] = self.xy_variance
        output.pose.covariance[14] = self.xy_variance
        output.pose.covariance[21] = self.yaw_variance
        output.pose.covariance[28] = self.yaw_variance
        output.pose.covariance[35] = self.yaw_variance
        self.publisher.publish(output)


if __name__ == "__main__":
    try:
        rospy.init_node("gazebo_truth_odometry", anonymous=False)
        GazeboTruthOdometry()
        rospy.spin()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
