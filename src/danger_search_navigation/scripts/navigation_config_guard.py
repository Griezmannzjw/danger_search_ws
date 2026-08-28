#!/usr/bin/env python3
"""Apply and continuously verify the selected standard local planner config.

The planner owns part of its configuration through the standard
dynamic_reconfigure service. This guard intentionally has no cmd_vel or
action interface: it only prevents the rest of the stack from accepting goals
until the plugin reports the values requested by the launch configuration.
"""

import math

import rospy
from dynamic_reconfigure.client import Client
from std_msgs.msg import Bool


def validate_dwa_velocity_domain(
    config,
    safe_max_angular_speed_rps=0.40,
    effective_min_in_place_angular_speed_rps=0.40,
):
    """Reject a DWA config that would recreate unsafe Unitree yaw samples.

    ``cmd_mux`` retains a broader final hard limit for all producers, while
    ordinary move_base operation has a smaller, validated policy domain.  An
    odd symmetric sample count is required so moving arcs still evaluate zero
    and the small/mid yaw candidates.  Pure rotations must clear the measured
    Unitree policy deadband.
    Upstream DWA treats ``min_vel_theta`` as a non-negative minimum magnitude;
    it derives the signed sample interval from ``max_vel_theta``.
    """
    if not isinstance(config, dict):
        raise ValueError("DWA configuration must be a mapping")
    required = (
        "min_vel_trans",
        "min_vel_x",
        "min_vel_y",
        "max_vel_y",
        "min_vel_theta",
        "max_vel_theta",
        "vth_samples",
    )
    missing = [name for name in required if name not in config]
    if missing:
        raise ValueError("DWA configuration missing " + ", ".join(missing))
    try:
        min_trans = float(config["min_vel_trans"])
        min_x = float(config["min_vel_x"])
        min_y = float(config["min_vel_y"])
        max_y = float(config["max_vel_y"])
        min_theta = float(config["min_vel_theta"])
        max_theta = float(config["max_vel_theta"])
        safe_limit = float(safe_max_angular_speed_rps)
        effective_minimum = float(effective_min_in_place_angular_speed_rps)
        samples = int(config["vth_samples"])
    except (TypeError, ValueError) as error:
        raise ValueError("DWA velocity domain is not numeric") from error
    values = (
        min_trans, min_x, min_y, max_y, min_theta, max_theta, safe_limit,
        effective_minimum,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("DWA velocity domain must be finite")
    if abs(min_y) > 1e-9 or abs(max_y) > 1e-9:
        raise ValueError("DWA must remain nonholonomic (min/max_vel_y == 0)")
    if min_trans <= 0.0 or min_x + 1e-9 < min_trans:
        raise ValueError("DWA min_vel_x must exclude the translational policy deadband")
    if safe_limit <= 0.0 or effective_minimum <= 0.0:
        raise ValueError("safe DWA angular speeds must be positive")
    if effective_minimum > safe_limit + 1e-9:
        raise ValueError("effective in-place minimum exceeds the safe policy limit")
    if min_theta <= 0.0 or max_theta <= 0.0 or min_theta > max_theta:
        raise ValueError("DWA angular magnitudes must satisfy 0 < min <= max")
    if max_theta > safe_limit + 1e-9:
        raise ValueError("DWA angular domain exceeds the safe policy limit")
    if min_theta + 1e-9 < effective_minimum:
        raise ValueError("DWA in-place yaw magnitude remains inside the policy deadband")
    if samples < 9 or samples % 2 == 0:
        raise ValueError("DWA requires an odd >=9 angular sample count")
    step = 2.0 * max_theta / float(samples - 1)
    if step > 0.10 + 1e-9:
        raise ValueError("DWA moving-arc yaw sample spacing exceeds 0.10 rad/s")


class NavigationConfigGuard:
    def __init__(self):
        rospy.init_node("navigation_config_guard", anonymous=False)
        self.planner_name = rospy.get_param(
            "~planner_name", "/move_base/DWAPlannerROS"
        )
        self.planner_config_key = rospy.get_param(
            "~planner_config_key", "DWAPlannerROS"
        )
        self.ready_topic = rospy.get_param(
            "~ready_topic", "/navigation/config_ready"
        )
        self.expected = dict(rospy.get_param(
            "~" + self.planner_config_key, {}
        ))
        if not self.expected:
            raise rospy.ROSInitException(
                "%s configuration is empty" % self.planner_config_key
            )
        self.safe_max_angular_speed_rps = float(rospy.get_param(
            "~safe_max_angular_speed_rps", 0.40
        ))
        self.effective_min_in_place_angular_speed_rps = float(rospy.get_param(
            "~effective_min_in_place_angular_speed_rps", 0.40
        ))
        try:
            if self.planner_config_key == "DWAPlannerROS":
                validate_dwa_velocity_domain(
                    self.expected,
                    self.safe_max_angular_speed_rps,
                    self.effective_min_in_place_angular_speed_rps,
                )
        except ValueError as error:
            raise rospy.ROSInitException("unsafe DWA velocity domain: %s" % error)

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
            raise RuntimeError(
                "%s exposed no expected dynamic parameters"
                % self.planner_config_key
            )
        self.client.update_configuration(dynamic)
        self.applied = True
        rospy.loginfo(
            "[navigation_config_guard] applied %d dynamic %s parameters",
            len(dynamic), self.planner_config_key,
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
