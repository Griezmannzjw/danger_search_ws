#!/usr/bin/env python3
"""Entry point for official PointCloud to planar LaserScan projection."""

import rospy

from danger_search_localization.scan_projector_node import ScanProjectorNode


if __name__ == "__main__":
    try:
        ScanProjectorNode().run()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
