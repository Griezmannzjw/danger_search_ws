#!/usr/bin/env python3

import unittest
import threading
from types import SimpleNamespace

import numpy as np
import rospy
from geometry_msgs.msg import TransformStamped

from danger_search_common.msg import DangerSource
from danger_search_perception.config import TrackingConfig
from danger_search_perception.detector_node import DangerDetectorNode
from danger_search_perception.tracking import MultiFrameDangerTracker


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

    def test_tracking_fields_are_populated_without_changing_message_type(self):
        node = DangerDetectorNode.__new__(DangerDetectorNode)
        node.tracker = MultiFrameDangerTracker(
            TrackingConfig(confirmation_hits=2)
        )

        first = self._danger(0, rospy.Time(20, 0), x=1.0)
        node._annotate_tracks([first], first.source_time)
        second = self._danger(0, rospy.Time(20, 100000000), x=1.04)
        node._annotate_tracks([second], second.source_time)

        self.assertTrue(first.track_id)
        self.assertEqual(first.track_id, second.track_id)
        self.assertFalse(first.confirmed)
        self.assertTrue(first.verification_required)
        self.assertTrue(second.confirmed)
        self.assertFalse(second.verification_required)
        self.assertGreater(second.position_covariance[0], 0.0)

        upper_floor = self._danger(
            1, rospy.Time(20, 200000000), x=1.02, z=2.75
        )
        node._annotate_tracks([upper_floor], upper_floor.source_time)
        self.assertNotEqual(upper_floor.track_id, second.track_id)
        self.assertTrue(upper_floor.track_id.startswith("danger-f1-"))

    @staticmethod
    def _danger(floor_id, stamp, x=1.0, y=0.0, z=0.15):
        danger = DangerSource()
        danger.detection_id = "{}.{}".format(stamp.secs, stamp.nsecs)
        danger.floor_id = floor_id
        danger.confidence = 0.9
        danger.position.header.frame_id = "map"
        danger.position.header.stamp = stamp
        danger.position.point.x = x
        danger.position.point.y = y
        danger.position.point.z = z
        danger.source_time = stamp
        return danger


if __name__ == "__main__":
    unittest.main()
