#!/usr/bin/env python3
"""Arbitrate move_base and bounded portal-traversal velocity commands."""

import math
import threading

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse

from danger_search_common.srv import TraversePortal, TraversePortalResponse


class NavigationCommandMux:
    def __init__(self):
        rospy.init_node("navigation_command_mux", anonymous=False)
        self.move_base_cmd_topic = rospy.get_param(
            "~move_base_cmd_topic", "/danger_search/move_base_cmd_vel"
        )
        self.output_cmd_topic = rospy.get_param(
            "~nav_cmd_topic", "/danger_search/nav_cmd_vel"
        )
        self.traverse_service = rospy.get_param(
            "~traverse_portal_service", "/navigation/traverse_portal"
        )
        self.cancel_service = rospy.get_param(
            "~cancel_portal_service", "/navigation/cancel_portal"
        )
        self.output_rate = float(rospy.get_param("~output_rate", 50.0))
        self.input_timeout = float(rospy.get_param("~input_timeout", 0.30))
        self.max_duration = float(rospy.get_param("~max_portal_duration", 6.0))
        self.max_linear_x = float(rospy.get_param("~max_portal_linear_x", 0.35))
        self.max_linear_y = float(rospy.get_param("~max_portal_linear_y", 0.20))
        self.max_angular_z = float(rospy.get_param("~max_portal_angular_z", 0.40))
        if min(self.output_rate, self.input_timeout, self.max_duration) <= 0.0:
            raise rospy.ROSInitException("invalid navigation command mux timing")

        self.lock = threading.RLock()
        self.last_move_base_cmd = Twist()
        self.last_move_base_time = rospy.Time(0)
        self.portal_active = False
        self.portal_canceled = False
        self.portal_command = Twist()

        self.publisher = rospy.Publisher(
            self.output_cmd_topic, Twist, queue_size=10
        )
        self.subscriber = rospy.Subscriber(
            self.move_base_cmd_topic,
            Twist,
            self._move_base_callback,
            queue_size=10,
        )
        self.traverse_server = rospy.Service(
            self.traverse_service, TraversePortal, self._traverse_callback
        )
        self.cancel_server = rospy.Service(
            self.cancel_service, Trigger, self._cancel_callback
        )
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.output_rate), self._publish
        )

    def _move_base_callback(self, message):
        with self.lock:
            self.last_move_base_cmd = message
            self.last_move_base_time = rospy.Time.now()

    def _traverse_callback(self, request):
        values = (
            float(request.linear_x),
            float(request.linear_y),
            float(request.angular_z),
            float(request.duration),
        )
        if not all(math.isfinite(value) for value in values):
            return TraversePortalResponse(False, "command must be finite")
        if request.duration <= 0.0 or request.duration > self.max_duration:
            return TraversePortalResponse(False, "duration outside configured bounds")
        if (abs(request.linear_x) > self.max_linear_x
                or abs(request.linear_y) > self.max_linear_y
                or abs(request.angular_z) > self.max_angular_z):
            return TraversePortalResponse(False, "velocity outside configured bounds")

        with self.lock:
            if self.portal_active:
                return TraversePortalResponse(False, "portal traversal already active")
            self.portal_active = True
            self.portal_canceled = False
            self.portal_command = Twist()
            self.portal_command.linear.x = request.linear_x
            self.portal_command.linear.y = request.linear_y
            self.portal_command.angular.z = request.angular_z

        deadline = rospy.Time.now() + rospy.Duration(request.duration)
        rate = rospy.Rate(self.output_rate)
        canceled = False
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            with self.lock:
                canceled = self.portal_canceled
            if canceled:
                break
            rate.sleep()

        with self.lock:
            self.portal_active = False
            self.portal_command = Twist()
        self.publisher.publish(Twist())
        if canceled:
            return TraversePortalResponse(False, "portal traversal canceled")
        return TraversePortalResponse(True, "portal traversal completed")

    def _cancel_callback(self, _request):
        with self.lock:
            was_active = self.portal_active
            self.portal_canceled = True
        self.publisher.publish(Twist())
        return TriggerResponse(
            success=True,
            message="portal traversal canceled" if was_active else "no portal traversal active",
        )

    def _publish(self, _event):
        now = rospy.Time.now()
        with self.lock:
            if self.portal_active:
                command = self.portal_command
            elif (self.last_move_base_time != rospy.Time(0)
                  and (now - self.last_move_base_time).to_sec() <= self.input_timeout):
                command = self.last_move_base_cmd
            else:
                command = Twist()
        self.publisher.publish(command)


if __name__ == "__main__":
    try:
        NavigationCommandMux()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
