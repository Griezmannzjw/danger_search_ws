#!/usr/bin/env python3
"""Latch the global safety stop when robot posture or IMU health is unsafe."""

import math
import os
import sys
import threading

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse

_DIR = os.path.dirname(os.path.abspath(__file__))
if _DIR not in sys.path:
    sys.path.insert(0, _DIR)
from posture_safety_core import PostureSafetyState


class PostureSafetyMonitor:
    def __init__(self):
        rospy.init_node("posture_safety_monitor", anonymous=False)
        self.lock = threading.RLock()
        deg = math.pi / 180.0
        self.state = PostureSafetyState(
            rospy.get_param("~posture_tilt_limit_deg", 30.0) * deg,
            rospy.get_param("~posture_trigger_hold_s", 0.18),
            rospy.get_param("~posture_recovery_tilt_deg", 15.0) * deg,
            rospy.get_param("~posture_recovery_hold_s", 2.0),
            rospy.get_param("~posture_imu_timeout_s", 0.25),
        )
        # ROS can dispatch a final Timer/subscriber callback while transport
        # publishers are being destroyed.  This latch closes that local race
        # in addition to rospy.is_shutdown().
        self._shutdown_started = False
        self._last_logged_state = None
        self.timer = None
        self.safety_pub = rospy.Publisher(
            rospy.get_param("~safety_stop_topic", "/danger_search/safety_stop"),
            Bool, queue_size=2, latch=True,
        )
        self.reason_pub = rospy.Publisher(
            rospy.get_param("~posture_reason_topic", "/danger_search/posture_safety_reason"),
            String, queue_size=2, latch=True,
        )
        self.fallen_pub = rospy.Publisher(
            rospy.get_param("~posture_fallen_topic", "/danger_search/posture_fallen"),
            Bool, queue_size=2, latch=True,
        )
        self.imu_sub = rospy.Subscriber(
            rospy.get_param("~imu_topic", "/trunk_imu"), Imu,
            self._imu_callback, queue_size=20,
        )
        self.reset_service = rospy.Service(
            rospy.get_param("~posture_reset_service", "/danger_search/reset_posture_safety"),
            Trigger, self._reset_callback,
        )
        rate = float(rospy.get_param("~posture_monitor_rate", 50.0))
        if not math.isfinite(rate) or rate <= 0.0:
            raise ValueError("posture_monitor_rate must be positive")
        rospy.on_shutdown(self.shutdown)
        self.timer = rospy.Timer(rospy.Duration(1.0 / rate), self._timer_callback)
        self._publish()

    def _imu_callback(self, message):
        if self._is_stopping():
            return
        q = message.orientation
        with self.lock:
            if self._is_stopping():
                return
            self.state.observe(rospy.Time.now().to_sec(), (q.x, q.y, q.z, q.w))
        self._publish()

    def _timer_callback(self, _event):
        if self._is_stopping():
            return
        with self.lock:
            if self._is_stopping():
                return
            self.state.tick(rospy.Time.now().to_sec())
        self._publish()

    def _reset_callback(self, _request):
        if self._is_stopping():
            return TriggerResponse(success=False, message="shutdown")
        with self.lock:
            if self._is_stopping():
                return TriggerResponse(success=False, message="shutdown")
            success, reason = self.state.reset(rospy.Time.now().to_sec())
        self._publish()
        return TriggerResponse(success=success, message=reason)

    def _is_stopping(self):
        return bool(getattr(self, "_shutdown_started", False) or rospy.is_shutdown())

    def _publish(self):
        with self.lock:
            if self._is_stopping():
                return False
            active, reason, fallen = (
                self.state.latched, self.state.reason, self.state.fallen
            )
            tilt = getattr(self.state, "last_tilt", None)
            log_key = (active, str(reason).split(":", 1)[0], fallen)
            state_changed = log_key != getattr(
                self, "_last_logged_state", None
            )
        if self._is_stopping():
            return False
        if state_changed:
            tilt_text = (
                "unknown" if tilt is None
                else "%.1fdeg" % math.degrees(float(tilt))
            )
            if active:
                rospy.logwarn(
                    "[posture_safety_monitor] safety stop active: "
                    "reason=%s fallen=%s tilt=%s",
                    reason,
                    fallen,
                    tilt_text,
                )
            else:
                rospy.loginfo(
                    "[posture_safety_monitor] safety stop cleared: "
                    "reason=%s fallen=%s tilt=%s",
                    reason,
                    fallen,
                    tilt_text,
                )
        try:
            self.safety_pub.publish(Bool(data=active))
            self.fallen_pub.publish(Bool(data=fallen))
            self.reason_pub.publish(String(data=reason))
        except rospy.ROSException:
            if self._is_stopping():
                return False
            raise
        self._last_logged_state = log_key
        return True

    def shutdown(self):
        """Prevent queued callbacks from publishing after ROS closes topics."""
        with self.lock:
            if getattr(self, "_shutdown_started", False):
                return
            self._shutdown_started = True
            timer = getattr(self, "timer", None)
            if timer is not None:
                timer.shutdown()


if __name__ == "__main__":
    try:
        PostureSafetyMonitor()
        rospy.spin()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
