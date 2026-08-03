#!/usr/bin/env python3

import unittest
import threading
from types import SimpleNamespace

import numpy as np
import rospy
from geometry_msgs.msg import TransformStamped

from danger_search_common.msg import DangerSource
from danger_search_perception.detector_node import DangerDetectorNode


class TestP0MessageAdapter(unittest.TestCase):
    def test_detection_populates_required_p0_fields(self):
        node = DangerDetectorNode.__new__(DangerDetectorNode)
        node.floor_id = 0
        node.current_floor = 0
        node.floor_lock = threading.Lock()
        node.target_frame = "map"

        transform = TransformStamped()
        transform.header.frame_id = "map"
        transform.transform.rotation.w = 1.0

        stamp = rospy.Time(12, 34)
        geometry = SimpleNamespace(
            center_camera=np.array([1.0, 2.0, 3.0])
        )

        result = node._to_danger_message(
            geometry,
            confidence=0.9,
            camera_frame="camera",
            stamp=stamp,
            transform=transform,
            candidate_index=2,
        )

        self.assertEqual(
            result.class_id, DangerSource.CLASS_DANGER_RED_SPHERE
        )
        self.assertEqual(result.detection_id, "12.34-2")
        self.assertEqual(result.position.header.frame_id, "map")
        self.assertEqual(result.position.header.stamp, stamp)
        self.assertEqual(result.floor_id, 0)
        self.assertAlmostEqual(result.confidence, 0.9)
        self.assertEqual(result.source_time, stamp)


if __name__ == "__main__":
    unittest.main()
