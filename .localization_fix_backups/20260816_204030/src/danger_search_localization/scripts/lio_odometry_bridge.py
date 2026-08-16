#!/usr/bin/env python3
"""Convert FAST-LIO's Livox-IMU pose into the localization odometry API.

FAST-LIO estimates the pose of its configured IMU state.  A blindly relabelled
FAST-LIO pose can therefore introduce a fixed translation and orientation
error.  This node obtains the fixed ``base <- tracking_frame`` TF,
applies the inverse extrinsic, and rebases the first corrected base pose to
the local ``odom`` origin.  It never publishes TF; ``LocalizationAdapterNode``
remains the sole publisher of ``map -> odom -> base``.
"""

import math

import rospy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf.transformations import (
    concatenate_matrices,
    inverse_matrix,
    quaternion_from_matrix,
    quaternion_matrix,
    translation_from_matrix,
    translation_matrix,
)


class LioOdometryBridge:
    def __init__(self):
        rospy.init_node("lio_odometry_bridge", anonymous=False)
        self.input_topic = rospy.get_param("~lio_odom_topic", "/nav/lio_odom")
        self.output_topic = rospy.get_param(
            "~gicp_pose_topic", "/localization/raw_pose"
        )
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base")
        self.tracking_frame = rospy.get_param(
            "~lio_tracking_frame", "imu_link"
        )
        self.tf_timeout = float(rospy.get_param("~tf_timeout_s", 0.1))
        self.initial_map_from_base = None

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.publisher = rospy.Publisher(
            self.output_topic, PoseWithCovarianceStamped, queue_size=10
        )
        self.subscriber = rospy.Subscriber(
            self.input_topic, Odometry, self._callback, queue_size=10
        )
        rospy.loginfo(
            "[localization] LIO bridge: %s (%s) -> %s (%s)",
            self.input_topic,
            self.tracking_frame,
            self.output_topic,
            self.base_frame,
        )

    def _callback(self, message):
        try:
            base_from_tracking = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.tracking_frame,
                rospy.Time(0),
                rospy.Duration(self.tf_timeout),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            rospy.logwarn_throttle(
                1.0,
                "[localization] LIO extrinsic TF %s <- %s unavailable: %s",
                self.base_frame,
                self.tracking_frame,
                exc,
            )
            return

        map_from_tracking = self._matrix_from_pose(message.pose.pose)
        map_from_base = concatenate_matrices(
            map_from_tracking,
            inverse_matrix(self._matrix_from_transform(base_from_tracking)),
        )
        if self.initial_map_from_base is None:
            self.initial_map_from_base = map_from_base
            rospy.loginfo("[localization] LIO odometry origin initialized")
        odom_from_base = concatenate_matrices(
            inverse_matrix(self.initial_map_from_base), map_from_base
        )

        output = PoseWithCovarianceStamped()
        output.header.stamp = message.header.stamp
        output.header.frame_id = self.odom_frame
        translation = translation_from_matrix(odom_from_base)
        rotation = quaternion_from_matrix(odom_from_base)
        output.pose.pose.position.x = translation[0]
        output.pose.pose.position.y = translation[1]
        output.pose.pose.position.z = translation[2]
        output.pose.pose.orientation.x = rotation[0]
        output.pose.pose.orientation.y = rotation[1]
        output.pose.pose.orientation.z = rotation[2]
        output.pose.pose.orientation.w = rotation[3]
        output.pose.covariance = self._covariance(message.pose.covariance)
        self.publisher.publish(output)

    @staticmethod
    def _matrix_from_pose(pose):
        return concatenate_matrices(
            translation_matrix((pose.position.x, pose.position.y, pose.position.z)),
            quaternion_matrix(
                (
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                )
            ),
        )

    @staticmethod
    def _matrix_from_transform(transform):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return concatenate_matrices(
            translation_matrix((translation.x, translation.y, translation.z)),
            quaternion_matrix((rotation.x, rotation.y, rotation.z, rotation.w)),
        )

    @staticmethod
    def _covariance(source):
        """Use FAST-LIO covariance when supplied, otherwise a safe baseline."""
        covariance = list(source)
        diagonal = (0, 7, 14, 21, 28, 35)
        if len(covariance) != 36 or any(
            not math.isfinite(covariance[index]) or covariance[index] <= 0.0
            for index in diagonal
        ):
            covariance = [0.0] * 36
            covariance[0] = covariance[7] = 0.01
            covariance[14] = 0.04
            covariance[21] = covariance[28] = 0.05
            covariance[35] = 0.02
        return covariance


if __name__ == "__main__":
    try:
        LioOdometryBridge()
        rospy.spin()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
