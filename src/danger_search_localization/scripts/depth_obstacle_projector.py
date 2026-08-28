#!/usr/bin/env python3
"""Entry point for ground-filtered RealSense navigation obstacles."""

import rospy

from danger_search_localization.depth_obstacle_projector_node import (
    DepthObstacleProjectorNode,
)


if __name__ == "__main__":
    try:
        DepthObstacleProjectorNode().run()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass

