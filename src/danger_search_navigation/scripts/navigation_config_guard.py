#!/usr/bin/env python3
"""Apply and continuously verify TrajectoryPlannerROS dynamic configuration.

TrajectoryPlannerROS owns part of its configuration through the standard
dynamic_reconfigure service.  This guard intentionally has no cmd_vel or
action interface: it only prevents the rest of the stack from accepting goals
until the plugin reports the values requested by the launch configuration.
"""

import math

import rospy
from dynamic_reconfigure.client import Client
from std_msgs.msg import Bool


class NavigationConfigGuard:
    def __init__(self):
        rospy.init_node("navigation_config_guard", anonymous=False)
        self.planner_name = rospy.get_param(
            "~planner_name", "/move_base/TrajectoryPlannerROS"
        )
        self.ready_topic = rospy.get_param(
            "~ready_topic", "/navigation/config_ready"
        )
        self.expected = dict(rospy.get_param("~TrajectoryPlannerROS", {}))
        if not self.expected:
            raise rospy.ROSInitException("TrajectoryPlannerROS configuration is empty")
        y_vels = self.expected.get("y_vels")
        if isinstance(y_vels, (list, tuple)):
            self.expected["y_vels"] = ", ".join(str(value) for value in y_vels)
        if not isinstance(self.expected.get("y_vels"), str):
            raise rospy.ROSInitException("TrajectoryPlannerROS/y_vels must be a string")

        self.publisher = rospy.Publisher(self.ready_topic, Bool, queue_size=1, latch=True)
        self.client = None
        self.applied = False
        self.timer = rospy.Timer(rospy.Duration(1.0), self._tick)
        self.publisher.publish(Bool(data=False))
        rospy.loginfo("[navigation_config_guard] waiting for %s", self.planner_name)

    @staticmethod
    def _same(expected, actual):
        if isinstance(expected, bool):
            return isinstance(actual, bool) and expected == actual
        if isinstance(expected, str):
            return str(actual).replace(" ", "") == expected.replace(" ", "")
        if isinstance(expected, (int, float)):
            return (isinstance(actual, (int, float))
                    and math.isfinite(float(actual))
                    and abs(float(actual) - float(expected)) <= 1e-6)
        return expected == actual

    def _connect_and_apply(self):
        self.client = Client(self.planner_name, timeout=1.0)
        current = self.client.get_configuration(timeout=1.0)
        dynamic = {
            key: value for key, value in self.expected.items()
            if key in current
        }
        if not dynamic:
            raise RuntimeError("TrajectoryPlannerROS exposed no expected dynamic parameters")
        self.client.update_configuration(dynamic)
        self.applied = True
        rospy.loginfo(
            "[navigation_config_guard] applied %d dynamic TrajectoryPlannerROS parameters",
            len(dynamic),
        )

    def _verified(self):
        if self.client is None:
            return False
        current = self.client.get_configuration(timeout=1.0)
        for key, expected in self.expected.items():
            if key in current:
                actual = current[key]
            else:
                actual = rospy.get_param(self.planner_name + "/" + key, None)
            if not self._same(expected, actual):
                rospy.logerr_throttle(
                    5.0,
                    "[navigation_config_guard] %s mismatch: expected=%r actual=%r",
                    key, expected, actual,
                )
                return False
        return True

    def _tick(self, _event):
        if not self.applied:
            try:
                self._connect_and_apply()
            except Exception as error:  # ROS master/plugin can still be starting.
                rospy.logwarn_throttle(
                    5.0,
                    "[navigation_config_guard] planner not ready: %s", error,
                )
                self.publisher.publish(Bool(data=False))
                return
        try:
            ready = self._verified()
        except Exception as error:
            rospy.logerr_throttle(
                5.0,
                "[navigation_config_guard] verification failed: %s", error,
            )
            ready = False
        self.publisher.publish(Bool(data=ready))

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        NavigationConfigGuard().run()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
