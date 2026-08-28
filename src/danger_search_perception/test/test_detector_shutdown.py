#!/usr/bin/env python3

import unittest
from unittest import mock

import rospy

from danger_search_perception.detector_node import DangerDetectorNode


class DetectorShutdownTest(unittest.TestCase):
    @staticmethod
    def _node():
        node = DangerDetectorNode.__new__(DangerDetectorNode)
        node._shutdown_requested = False
        node.status_timer = mock.Mock()
        return node

    def test_shutdown_latches_and_stops_status_timer(self):
        node = self._node()

        node._on_shutdown()

        self.assertTrue(node._shutdown_requested)
        node.status_timer.shutdown.assert_called_once_with()

    def test_queued_callbacks_exit_before_messages_or_publishers(self):
        node = self._node()
        node._shutdown_requested = True

        node._sensor_callback(
            mock.sentinel.rgb, mock.sentinel.depth, mock.sentinel.camera_info
        )
        node._mapping_status_callback(mock.sentinel.mapping)
        node._localization_status_callback(mock.sentinel.localization)
        node._publish(mock.sentinel.output)
        node._publish_status()

    def test_publish_skips_closed_ros_before_calling_publisher(self):
        node = self._node()
        publisher = mock.Mock()

        with mock.patch.object(rospy, "is_shutdown", return_value=True):
            published = node._publish_if_running(publisher, mock.sentinel.message)

        self.assertFalse(published)
        publisher.publish.assert_not_called()

    def test_publish_swallows_only_shutdown_race(self):
        node = self._node()
        publisher = mock.Mock()
        publisher.publish.side_effect = rospy.ROSException("publisher closed")

        with mock.patch.object(
                rospy, "is_shutdown", side_effect=(False, True)):
            published = node._publish_if_running(publisher, mock.sentinel.message)

        self.assertFalse(published)
        publisher.publish.assert_called_once_with(mock.sentinel.message)

    def test_publish_preserves_normal_ros_exception(self):
        node = self._node()
        publisher = mock.Mock()
        publisher.publish.side_effect = rospy.ROSException("transport failed")

        with mock.patch.object(rospy, "is_shutdown", return_value=False):
            with self.assertRaises(rospy.ROSException):
                node._publish_if_running(publisher, mock.sentinel.message)


if __name__ == "__main__":
    unittest.main()
