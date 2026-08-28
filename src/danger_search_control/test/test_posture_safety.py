#!/usr/bin/env python3
import math
import pathlib
import sys
import threading
import types
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).parents[1] / "scripts"))


def _install_monitor_stubs():
    """Allow the monitor callback tests to run without a sourced ROS env."""
    try:
        import rospy  # noqa: F401
    except ImportError:
        rospy = types.ModuleType("rospy")
        rospy.ROSException = RuntimeError
        rospy.ROSInterruptException = RuntimeError
        sys.modules["rospy"] = rospy

    try:
        from sensor_msgs.msg import Imu  # noqa: F401
    except ImportError:
        sensor_msgs = types.ModuleType("sensor_msgs")
        sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
        sensor_msgs_msg.Imu = type("Imu", (), {})
        sensor_msgs.msg = sensor_msgs_msg
        sys.modules["sensor_msgs"] = sensor_msgs
        sys.modules["sensor_msgs.msg"] = sensor_msgs_msg

    try:
        from std_msgs.msg import Bool, String  # noqa: F401
    except ImportError:
        class Bool:
            def __init__(self, data=False):
                self.data = data

        class String:
            def __init__(self, data=""):
                self.data = data

        std_msgs = types.ModuleType("std_msgs")
        std_msgs_msg = types.ModuleType("std_msgs.msg")
        std_msgs_msg.Bool = Bool
        std_msgs_msg.String = String
        std_msgs.msg = std_msgs_msg
        sys.modules["std_msgs"] = std_msgs
        sys.modules["std_msgs.msg"] = std_msgs_msg

    try:
        from std_srvs.srv import Trigger, TriggerResponse  # noqa: F401
    except ImportError:
        class Trigger:
            pass

        class TriggerResponse:
            def __init__(self, success=False, message=""):
                self.success = success
                self.message = message

        std_srvs = types.ModuleType("std_srvs")
        std_srvs_srv = types.ModuleType("std_srvs.srv")
        std_srvs_srv.Trigger = Trigger
        std_srvs_srv.TriggerResponse = TriggerResponse
        std_srvs.srv = std_srvs_srv
        sys.modules["std_srvs"] = std_srvs
        sys.modules["std_srvs.srv"] = std_srvs_srv


_install_monitor_stubs()
from posture_safety_core import PostureSafetyState
import posture_safety_monitor as monitor_module
from posture_safety_monitor import PostureSafetyMonitor


def pitch_quaternion(degrees):
    angle = math.radians(degrees) / 2.0
    return (0.0, math.sin(angle), 0.0, math.cos(angle))


class PostureSafetyTest(unittest.TestCase):
    def state(self):
        return PostureSafetyState(math.radians(30), .18, math.radians(15), 2.0, .25)

    def test_startup_and_stale_imu_fail_safe(self):
        state = self.state()
        self.assertTrue(state.tick(0.0)[0])
        state.observe(0.0, pitch_quaternion(0))
        state.observe(2.1, pitch_quaternion(0))
        self.assertFalse(state.latched)
        self.assertTrue(state.tick(2.36)[0])
        self.assertEqual(state.reason, "imu_stale")

    def test_short_threshold_crossing_does_not_trip(self):
        state = self.state()
        state.observe(0.0, pitch_quaternion(39))
        state.observe(.10, pitch_quaternion(39))
        state.observe(.15, pitch_quaternion(10))
        self.assertEqual(state.reason, "imu_not_received")

    def test_39_and_70_degree_falls_trip_after_debounce(self):
        for tilt in (39, 70):
            state = self.state()
            state.observe(1.0, pitch_quaternion(tilt))
            self.assertTrue(state.observe(1.19, pitch_quaternion(tilt))[0])
            self.assertIn("excessive_tilt", state.reason)
            self.assertTrue(state.fallen)

    def test_invalid_quaternion_trips_immediately(self):
        state = self.state()
        state.latched = False
        self.assertTrue(state.observe(1.0, (0, 0, 0, 0))[0])
        self.assertIn("invalid_imu", state.reason)
        self.assertFalse(state.fallen)
        self.assertFalse(state.reset(1.01)[0])

    def test_recovery_requires_continuous_stable_window(self):
        state = self.state()
        state.observe(0.0, pitch_quaternion(70))
        state.observe(.2, pitch_quaternion(70))
        state.observe(.3, pitch_quaternion(5))
        state.observe(1.0, pitch_quaternion(20))
        state.observe(1.1, pitch_quaternion(5))
        state.observe(3.11, pitch_quaternion(5))
        self.assertFalse(state.latched)

    def test_reset_requires_fresh_safe_posture(self):
        state = self.state()
        state.observe(1.0, pitch_quaternion(5))
        self.assertEqual(state.reset(1.1), (True, "explicit_reset"))
        state.observe(2.0, pitch_quaternion(70))
        state.observe(2.2, pitch_quaternion(70))
        self.assertFalse(state.reset(2.21)[0])
        state.observe(3.0, pitch_quaternion(5))
        self.assertFalse(state.reset(3.26)[0])


class FakeTime:
    now_seconds = 0.0

    @classmethod
    def now(cls):
        return types.SimpleNamespace(to_sec=lambda: cls.now_seconds)


class FakeRospy:
    Time = FakeTime
    ROSException = RuntimeError
    shutdown_active = False

    @classmethod
    def is_shutdown(cls):
        return cls.shutdown_active


class FakePublisher:
    def __init__(self, exception=None):
        self.messages = []
        self.exception = exception

    def publish(self, message):
        if self.exception is not None:
            if callable(self.exception):
                raise self.exception()
            raise self.exception
        self.messages.append(message)


class FakeTimer:
    def __init__(self):
        self.shutdown_calls = 0

    def shutdown(self):
        self.shutdown_calls += 1


class FakeMonitorState:
    def __init__(self):
        self.latched = True
        self.reason = "imu_stale"
        self.fallen = False
        self.observe_calls = 0
        self.tick_calls = 0

    def observe(self, *_args):
        self.observe_calls += 1

    def tick(self, *_args):
        self.tick_calls += 1

    def reset(self, *_args):
        return True, "explicit_reset"


def _monitor_test_node():
    node = PostureSafetyMonitor.__new__(PostureSafetyMonitor)
    node.lock = threading.RLock()
    node.state = FakeMonitorState()
    node.safety_pub = FakePublisher()
    node.reason_pub = FakePublisher()
    node.fallen_pub = FakePublisher()
    node.timer = FakeTimer()
    node._shutdown_started = False
    return node


class PostureSafetyMonitorShutdownTest(unittest.TestCase):
    def setUp(self):
        self.original_rospy = monitor_module.rospy
        monitor_module.rospy = FakeRospy
        FakeRospy.shutdown_active = False

    def tearDown(self):
        FakeRospy.shutdown_active = False
        monitor_module.rospy = self.original_rospy

    def test_shutdown_stops_timer_and_blocks_queued_callbacks(self):
        node = _monitor_test_node()
        node.shutdown()
        self.assertTrue(node._shutdown_started)
        self.assertEqual(node.timer.shutdown_calls, 1)

        node._imu_callback(
            types.SimpleNamespace(
                orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
            )
        )
        node._timer_callback(None)
        response = node._reset_callback(None)
        self.assertEqual(node.state.observe_calls, 0)
        self.assertEqual(node.state.tick_calls, 0)
        self.assertFalse(response.success)
        self.assertEqual(response.message, "shutdown")
        self.assertEqual(node.safety_pub.messages, [])

    def test_publish_skips_closed_ros_and_suppresses_only_shutdown_race(self):
        node = _monitor_test_node()
        FakeRospy.shutdown_active = True
        self.assertFalse(node._publish())
        self.assertEqual(node.safety_pub.messages, [])

        FakeRospy.shutdown_active = False

        def close_ros_then_fail():
            FakeRospy.shutdown_active = True
            return FakeRospy.ROSException("publisher closed")

        node.safety_pub = FakePublisher(exception=close_ros_then_fail)
        self.assertFalse(node._publish())

        FakeRospy.shutdown_active = False
        node.safety_pub = FakePublisher(exception=FakeRospy.ROSException("unexpected"))
        with self.assertRaises(FakeRospy.ROSException):
            node._publish()


if __name__ == "__main__":
    unittest.main()
