#!/usr/bin/env python3
"""Expose the trusted local pose as Hector's known map-building pose.

Hector is run with ``map_with_known_poses=true``: it updates occupancy only and
does not scan-match a second, conflicting trajectory.  This bridge supplies the
identity-aligned backend pose expected by the public adapter.
"""

import copy

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped


class KnownPoseBackend:
    def __init__(self):
        input_topic = rospy.get_param("~gicp_pose_topic", "/localization/raw_pose")
        output_topic = rospy.get_param(
            "~backend_pose_topic", "/localization/hector_pose"
        )
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.publisher = rospy.Publisher(
            output_topic, PoseWithCovarianceStamped, queue_size=10
        )
        self.subscriber = rospy.Subscriber(
            input_topic,
            PoseWithCovarianceStamped,
            self._pose_callback,
            queue_size=10,
        )
        rospy.loginfo(
            "[localization] known-pose mapping backend: %s -> %s",
            input_topic,
            output_topic,
        )

    def _pose_callback(self, message):
        mapped = copy.deepcopy(message)
        mapped.header.frame_id = self.map_frame
        self.publisher.publish(mapped)


if __name__ == "__main__":
    try:
        rospy.init_node("known_pose_backend")
        KnownPoseBackend()
        rospy.spin()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
