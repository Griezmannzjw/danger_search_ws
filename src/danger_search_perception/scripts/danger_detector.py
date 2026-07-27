#!/usr/bin/env python3
"""ROS executable entry point for danger source perception."""

import rospy

from danger_search_perception.detector_node import DangerDetectorNode


if __name__ == "__main__":
    try:
        DangerDetectorNode().run()
    except rospy.ROSInterruptException:
        pass
