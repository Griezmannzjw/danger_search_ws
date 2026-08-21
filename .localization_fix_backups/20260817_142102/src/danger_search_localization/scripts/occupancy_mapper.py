#!/usr/bin/env python3
"""Entry point for the trusted-odometry occupancy mapper."""

import rospy

from danger_search_localization.occupancy_mapper_node import OccupancyMapperNode


if __name__ == "__main__":
    try:
        OccupancyMapperNode().run()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
