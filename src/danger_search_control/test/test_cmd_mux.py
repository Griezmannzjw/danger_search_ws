#!/usr/bin/env python3
"""CmdMux P0 纯计算和接口结构单元测试。"""

import math
import os
import sys
import threading
import types
import unittest
import xml.etree.ElementTree as ElementTree

import yaml


def _install_message_stubs():
    """本机未 source ROS 时提供足够的 Twist/Bool 测试桩。"""
    try:
        from geometry_msgs.msg import Twist as _Twist
    except ImportError:
        class Vector3:
            def __init__(self):
                self.x = 0.0
                self.y = 0.0
                self.z = 0.0

        class Twist:
            def __init__(self):
                self.linear = Vector3()
                self.angular = Vector3()

        geometry = types.ModuleType("geometry_msgs")
        geometry_msg = types.ModuleType("geometry_msgs.msg")
        geometry_msg.Twist = Twist
        geometry.msg = geometry_msg
        sys.modules["geometry_msgs"] = geometry
        sys.modules["geometry_msgs.msg"] = geometry_msg

    try:
        from std_msgs.msg import Bool as _Bool
    except ImportError:
        class Bool:
            def __init__(self, data=False):
                self.data = data

        std = types.ModuleType("std_msgs")
        std_msg = types.ModuleType("std_msgs.msg")
        std_msg.Bool = Bool
        std.msg = std_msg
        sys.modules["std_msgs"] = std
        sys.modules["std_msgs.msg"] = std_msg


def _install_rospy_stub():
    try:
        import rospy
        return
    except ImportError:
        rospy = types.ModuleType("rospy")
        rospy.ROSException = RuntimeError
        rospy.ROSInterruptException = RuntimeError
        sys.modules["rospy"] = rospy


_install_rospy_stub()
_install_message_stubs()

SCRIPTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

import cmd_mux as cmd_mux_module
from cmd_mux import (
    CmdMux,
    CmdMuxCore,
    DEFAULT_PARAMS,
    _finite_velocity,
    validate_parameters,
)
from geometry_msgs.msg import Twist


def _target_message(x=0.0, y=0.0, z=0.0):
    message = Twist()
    message.linear.x = x
    message.linear.y = y
    message.angular.z = z
    return message


def _normal_step(core, now, target=(1.0, 1.0, 1.0), nav_time=0.0):
    return core.step(now, target, True, nav_time)[0]


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeTime:
    now_seconds = 0.0

    def __init__(self, seconds=0.0):
        self.seconds = float(seconds)

    @classmethod
    def now(cls):
        return cls(cls.now_seconds)

    def to_sec(self):
        return self.seconds


class FakeRospy:
    Time = FakeTime

    @staticmethod
    def logwarn_throttle(*_args, **_kwargs):
        pass

    @staticmethod
    def logwarn(*_args, **_kwargs):
        pass

    @staticmethod
    def loginfo(*_args, **_kwargs):
        pass


def _callback_test_node():
    """构造未初始化 ROS 通信的节点实例，用于直接验证回调状态转换。"""
    node = CmdMux.__new__(CmdMux)
    node._lock = threading.RLock()
    node._target_velocity = (0.0, 0.0, 0.0)
    node._has_valid_nav = False
    node._invalid_nav = False
    node._last_nav_time_sec = None
    node.last_nav_time = FakeTime(0.0)
    node.safety_stop = False
    node.last_output = Twist()
    node._core = CmdMuxCore(cmd_timeout_s=0.5)
    node._core.last_output_time = 0.0
    node.cmd_pub = FakePublisher()
    node.sent_cmd_pub = FakePublisher()
    return node


def _assert_echo_pairs(test_case, node):
    test_case.assertEqual(len(node.cmd_pub.messages), len(node.sent_cmd_pub.messages))
    for output, echo in zip(node.cmd_pub.messages, node.sent_cmd_pub.messages):
        test_case.assertIs(output, echo)
        test_case.assertEqual(output.linear.x, echo.linear.x)
        test_case.assertEqual(output.linear.y, echo.linear.y)
        test_case.assertEqual(output.angular.z, echo.angular.z)


class CmdMuxCoreTest(unittest.TestCase):
    def test_three_axes_follow_acceleration_limits(self):
        core = CmdMuxCore(cmd_timeout_s=10.0)
        self.assertEqual(_normal_step(core, 0.0), (0.0, 0.0, 0.0))
        self.assertEqual(_normal_step(core, 0.1), (0.1, 0.1, 0.2))
        self.assertEqual(_normal_step(core, 0.2), (0.2, 0.2, 0.4))

    def test_each_axis_is_clamped_before_acceleration_limit(self):
        core = CmdMuxCore(
            max_linear_accel=100.0,
            max_angular_accel=100.0,
            cmd_timeout_s=10.0,
        )
        _normal_step(core, 0.0, target=(9.0, -9.0, 9.0))
        output = _normal_step(core, 0.1, target=(9.0, -9.0, 9.0))
        self.assertEqual(output, (0.30, -0.25, 0.80))

    def test_safety_stop_is_immediate_and_resets_actual_output(self):
        core = CmdMuxCore(cmd_timeout_s=10.0)
        _normal_step(core, 0.0, target=(0.5, 0.5, 0.5))
        self.assertNotEqual(_normal_step(core, 0.1, target=(0.5, 0.5, 0.5)), (0.0, 0.0, 0.0))
        output, reason = core.step(
            0.2, (0.5, 0.5, 0.5), True, 0.0, safety_stop=True
        )
        self.assertEqual(output, (0.0, 0.0, 0.0))
        self.assertEqual(reason, "safety")
        self.assertEqual(core.last_output, (0.0, 0.0, 0.0))

    def test_no_valid_command_and_timeout_are_zero(self):
        core = CmdMuxCore(cmd_timeout_s=0.5)
        output, reason = core.step(0.0, (1.0, 0.0, 0.0), False, None)
        self.assertEqual(output, (0.0, 0.0, 0.0))
        self.assertEqual(reason, "no_valid_nav")
        _normal_step(core, 0.1, nav_time=0.1)
        output, reason = core.step(0.7, (1.0, 0.0, 0.0), True, 0.1)
        self.assertEqual(output, (0.0, 0.0, 0.0))
        self.assertEqual(reason, "timeout")

    def test_nan_and_inf_are_rejected_and_cannot_refresh_command(self):
        for invalid in (math.nan, math.inf, -math.inf):
            message = _target_message(x=invalid, y=0.1, z=0.1)
            self.assertIsNone(_finite_velocity(message))
        core = CmdMuxCore(cmd_timeout_s=10.0)
        _normal_step(core, 0.0, target=(0.5, 0.0, 0.0))
        _normal_step(core, 0.1, target=(0.5, 0.0, 0.0))
        output, reason = core.step(
            0.2, (math.nan, 0.0, 0.0), False, 0.1, invalid_nav=True
        )
        self.assertEqual(output, (0.0, 0.0, 0.0))
        self.assertEqual(reason, "invalid")

    def test_stop_release_requires_fresh_command_and_starts_at_zero(self):
        core = CmdMuxCore(cmd_timeout_s=10.0)
        _normal_step(core, 0.0, target=(0.5, 0.0, 0.0))
        _normal_step(core, 0.1, target=(0.5, 0.0, 0.0))
        core.step(0.2, (0.5, 0.0, 0.0), True, 0.1, safety_stop=True)
        output, reason = core.step(0.3, (0.5, 0.0, 0.0), False, None)
        self.assertEqual(output, (0.0, 0.0, 0.0))
        self.assertEqual(reason, "no_valid_nav")
        output = _normal_step(core, 0.4, target=(0.5, 0.0, 0.0), nav_time=0.4)
        self.assertAlmostEqual(output[0], 0.1)

    def test_bad_dt_never_causes_a_jump(self):
        core = CmdMuxCore(cmd_timeout_s=100.0, max_dt_s=0.1)
        _normal_step(core, 0.0, target=(1.0, 0.0, 0.0))
        first = _normal_step(core, 0.1, target=(1.0, 0.0, 0.0))
        backwards = _normal_step(core, 0.05, target=(0.0, 0.0, 0.0))
        same_time = _normal_step(core, 0.05, target=(0.0, 0.0, 0.0))
        capped = _normal_step(core, 10.0, target=(1.0, 0.0, 0.0))
        self.assertEqual(first, (0.1, 0.0, 0.0))
        self.assertEqual(backwards, first)
        self.assertEqual(same_time, first)
        self.assertLessEqual(capped[0] - same_time[0], 0.1)


class CmdMuxInterfaceTest(unittest.TestCase):
    def test_unused_twist_axes_are_zero(self):
        output = CmdMux._message_from_velocity((0.1, -0.2, 0.3))
        self.assertEqual(output.linear.x, 0.1)
        self.assertEqual(output.linear.y, -0.2)
        self.assertEqual(output.linear.z, 0.0)
        self.assertEqual(output.angular.x, 0.0)
        self.assertEqual(output.angular.y, 0.0)
        self.assertEqual(output.angular.z, 0.3)

    def test_cmd_vel_and_echo_publish_the_same_message_on_all_paths(self):
        node = CmdMux.__new__(CmdMux)
        node.cmd_pub = FakePublisher()
        node.sent_cmd_pub = FakePublisher()
        outputs = [
            (0.1, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.2, -0.3),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
        ]
        for values in outputs:
            message = CmdMux._message_from_velocity(values)
            node._publish_locked(message)
            self.assertIs(node.cmd_pub.messages[-1], node.sent_cmd_pub.messages[-1])
            self.assertEqual(node.cmd_pub.messages[-1].linear.x, values[0])
            self.assertEqual(node.cmd_pub.messages[-1].linear.y, values[1])
            self.assertEqual(node.cmd_pub.messages[-1].angular.z, values[2])

    def test_callbacks_keep_echo_identical_on_normal_stop_and_fault_paths(self):
        original_rospy = cmd_mux_module.rospy
        cmd_mux_module.rospy = FakeRospy
        try:
            node = _callback_test_node()

            # 无有效命令、正常命令、超时、非法输入、急停和解除急停。
            node.output_loop(None)
            FakeTime.now_seconds = 0.0
            node.nav_cmd_callback(_target_message(0.2, 0.1, 0.4))
            FakeTime.now_seconds = 0.1
            node.output_loop(None)
            FakeTime.now_seconds = 0.7
            node.output_loop(None)

            FakeTime.now_seconds = 0.8
            node.nav_cmd_callback(_target_message(0.2, 0.1, 0.4))
            last_valid_time = node._last_nav_time_sec
            node.nav_cmd_callback(_target_message(math.nan, 0.0, 0.0))
            self.assertEqual(node._last_nav_time_sec, last_valid_time)
            self.assertAlmostEqual(node.last_nav_time.to_sec(), last_valid_time)
            node.output_loop(None)

            node.safety_callback(types.SimpleNamespace(data=True))
            node.output_loop(None)
            node.safety_callback(types.SimpleNamespace(data=False))
            node.output_loop(None)

            _assert_echo_pairs(self, node)
            self.assertEqual(node.cmd_pub.messages[0].linear.x, 0.0)
            self.assertGreater(node.cmd_pub.messages[1].linear.x, 0.0)
            self.assertEqual(node.cmd_pub.messages[2].linear.x, 0.0)
            self.assertEqual(node.cmd_pub.messages[3].linear.x, 0.0)
            self.assertEqual(node.cmd_pub.messages[-1].linear.x, 0.0)
        finally:
            cmd_mux_module.rospy = original_rospy

    def test_shutdown_sends_zero_to_final_output_and_echo(self):
        original_rospy = cmd_mux_module.rospy
        cmd_mux_module.rospy = FakeRospy
        try:
            node = _callback_test_node()
            node._core.last_output = (0.2, -0.1, 0.3)
            FakeTime.now_seconds = 1.0
            node.shutdown()

            self.assertEqual(len(node.cmd_pub.messages), 1)
            _assert_echo_pairs(self, node)
            output = node.cmd_pub.messages[0]
            self.assertEqual(output.linear.x, 0.0)
            self.assertEqual(output.linear.y, 0.0)
            self.assertEqual(output.angular.z, 0.0)
            self.assertEqual(node._core.last_output, (0.0, 0.0, 0.0))
        finally:
            cmd_mux_module.rospy = original_rospy

    def test_defaults_and_startup_validation(self):
        self.assertEqual(DEFAULT_PARAMS["max_linear_speed"], 0.30)
        self.assertEqual(DEFAULT_PARAMS["max_lateral_speed"], 0.25)
        self.assertEqual(DEFAULT_PARAMS["max_angular_speed"], 0.80)
        self.assertEqual(DEFAULT_PARAMS["max_dt_s"], 0.10)
        parameters = dict(DEFAULT_PARAMS, enable_safety=True)
        self.assertEqual(validate_parameters(parameters)["cmd_timeout_s"], 0.5)
        for name in (
            "output_rate",
            "cmd_timeout_s",
            "max_linear_accel",
            "max_angular_accel",
            "max_linear_speed",
            "max_lateral_speed",
            "max_angular_speed",
            "max_dt_s",
        ):
            invalid = dict(parameters, **{name: 0.0})
            with self.assertRaises(ValueError, msg=name):
                validate_parameters(invalid)

    def test_yaml_defaults_and_private_launch_loading(self):
        package_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        with open(os.path.join(package_dir, "config", "default.yaml"), "r") as stream:
            config = yaml.safe_load(stream)
        self.assertEqual(config["sent_cmd_topic"], "/danger_search/cmd_vel_sent")
        self.assertEqual(config["max_linear_speed"], 0.30)
        self.assertEqual(config["max_lateral_speed"], 0.25)
        self.assertEqual(config["max_angular_speed"], 0.80)
        self.assertEqual(config["max_dt_s"], 0.10)

        launch = ElementTree.parse(os.path.join(package_dir, "launch", "cmd_mux.launch"))
        root = launch.getroot()
        self.assertEqual(root.findall("rosparam"), [])
        node = root.find("node")
        self.assertIsNotNone(node)
        rosparam = node.find("rosparam")
        self.assertIsNotNone(rosparam)
        self.assertEqual(rosparam.attrib["command"], "load")
        self.assertEqual(rosparam.attrib["file"], "$(arg config_file)")


if __name__ == "__main__":
    unittest.main()
