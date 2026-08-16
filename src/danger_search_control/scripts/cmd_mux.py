#!/usr/bin/env python3
"""控制执行层：导航速度限幅、平滑输出和最高优先级急停。"""

import math
import threading

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


DEFAULT_PARAMS = {
    "output_rate": 50.0,
    "cmd_timeout_s": 0.5,
    "max_linear_accel": 1.0,
    "max_angular_accel": 2.0,
    "max_linear_speed": 0.40,
    "max_lateral_speed": 0.25,
    "max_angular_speed": 0.80,
    "max_dt_s": 0.10,
}


def _positive_finite(value, name):
    """将参数转换为正有限浮点数，拒绝可能导致危险输出的配置。"""
    if isinstance(value, bool):
        raise ValueError("参数 %s 不能是布尔值" % name)
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise ValueError("参数 %s 必须是数字" % name)
    if not math.isfinite(number) or number <= 0.0:
        raise ValueError("参数 %s 必须是正有限数值" % name)
    return number


def validate_parameters(parameters):
    """校验并规范控制器的数值参数，返回可直接使用的副本。"""
    checked = {}
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
        checked[name] = _positive_finite(parameters[name], name)

    enable_safety = parameters.get("enable_safety", True)
    if not isinstance(enable_safety, bool):
        raise ValueError("参数 enable_safety 必须是布尔值")
    checked["enable_safety"] = enable_safety
    return checked


def _finite_velocity(msg):
    """只提取 Twist 的三个使用轴，任一轴非法则返回 None。"""
    try:
        values = (float(msg.linear.x), float(msg.linear.y), float(msg.angular.z))
    except (AttributeError, TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in values):
        return None
    return values


def _clamp(value, limit):
    return max(-limit, min(limit, value))


class CmdMuxCore:
    """ROS 无关的速度状态机，便于在不启动 ROS 的情况下验证安全语义。"""

    def __init__(
        self,
        max_linear_speed=DEFAULT_PARAMS["max_linear_speed"],
        max_lateral_speed=DEFAULT_PARAMS["max_lateral_speed"],
        max_angular_speed=DEFAULT_PARAMS["max_angular_speed"],
        max_linear_accel=DEFAULT_PARAMS["max_linear_accel"],
        max_angular_accel=DEFAULT_PARAMS["max_angular_accel"],
        max_dt_s=DEFAULT_PARAMS["max_dt_s"],
        cmd_timeout_s=DEFAULT_PARAMS["cmd_timeout_s"],
        enable_safety=True,
    ):
        self.max_linear_speed = _positive_finite(max_linear_speed, "max_linear_speed")
        self.max_lateral_speed = _positive_finite(max_lateral_speed, "max_lateral_speed")
        self.max_angular_speed = _positive_finite(max_angular_speed, "max_angular_speed")
        self.max_linear_accel = _positive_finite(max_linear_accel, "max_linear_accel")
        self.max_angular_accel = _positive_finite(max_angular_accel, "max_angular_accel")
        self.max_dt_s = _positive_finite(max_dt_s, "max_dt_s")
        self.cmd_timeout_s = _positive_finite(cmd_timeout_s, "cmd_timeout_s")
        if not isinstance(enable_safety, bool):
            raise ValueError("参数 enable_safety 必须是布尔值")
        self.enable_safety = enable_safety

        self.last_output = (0.0, 0.0, 0.0)
        self.last_output_time = None

    def reset(self, now=None):
        """强制清零，并让下一条正常命令从零开始加速。"""
        self.last_output = (0.0, 0.0, 0.0)
        if now is not None and math.isfinite(float(now)):
            self.last_output_time = float(now)

    def step(
        self,
        now,
        target,
        has_valid_nav,
        last_nav_time,
        safety_stop=False,
        invalid_nav=False,
    ):
        """计算下一次输出，返回 (x, y, z) 和原因字符串。"""
        now = float(now)
        if not math.isfinite(now):
            self.reset()
            return (0.0, 0.0, 0.0), "invalid_time"

        if self.last_output_time is None:
            dt_raw = 0.0
        else:
            dt_raw = now - self.last_output_time
        self.last_output_time = now

        # 所有强制停车路径都立即清零，不经过减速斜坡。
        if safety_stop:
            self.reset(now)
            return (0.0, 0.0, 0.0), "safety"
        if invalid_nav:
            self.reset(now)
            return (0.0, 0.0, 0.0), "invalid"
        if not has_valid_nav or target is None or last_nav_time is None:
            self.reset(now)
            return (0.0, 0.0, 0.0), "no_valid_nav"

        nav_age = now - float(last_nav_time)
        if self.enable_safety and (nav_age < 0.0 or nav_age > self.cmd_timeout_s):
            self.reset(now)
            return (0.0, 0.0, 0.0), "timeout"

        target = (
            _clamp(float(target[0]), self.max_linear_speed),
            _clamp(float(target[1]), self.max_lateral_speed),
            _clamp(float(target[2]), self.max_angular_speed),
        )

        # 仿真时钟倒退、停住或异常跳跃时，不让输出发生一次大跳变。
        if not math.isfinite(dt_raw) or dt_raw <= 0.0:
            dt = 0.0
        else:
            dt = min(dt_raw, self.max_dt_s)

        previous = self.last_output
        max_linear_delta = self.max_linear_accel * dt
        max_angular_delta = self.max_angular_accel * dt
        output = (
            previous[0] + _clamp(target[0] - previous[0], max_linear_delta),
            previous[1] + _clamp(target[1] - previous[1], max_linear_delta),
            previous[2] + _clamp(target[2] - previous[2], max_angular_delta),
        )
        self.last_output = output
        return output, "normal"


class CmdMux:
    """导航速度通道加外部急停门，且是最终 /cmd_vel 的唯一发布者。"""

    def __init__(self):
        rospy.init_node("cmd_mux", anonymous=False)

        # 话题名称保持现有接口，回显话题补齐既有声明。
        self.nav_cmd_topic = rospy.get_param(
            "~nav_cmd_topic", "/danger_search/nav_cmd_vel"
        )
        self.output_cmd_topic = rospy.get_param("~output_cmd_topic", "/cmd_vel")
        self.sent_cmd_topic = rospy.get_param(
            "~sent_cmd_topic", "/danger_search/cmd_vel_sent"
        )
        self.safety_stop_topic = rospy.get_param(
            "~safety_stop_topic", "/danger_search/safety_stop"
        )
        for name, topic in (
            ("nav_cmd_topic", self.nav_cmd_topic),
            ("output_cmd_topic", self.output_cmd_topic),
            ("sent_cmd_topic", self.sent_cmd_topic),
            ("safety_stop_topic", self.safety_stop_topic),
        ):
            if not isinstance(topic, str) or not topic.strip():
                self._reject_config("参数 %s 必须是非空话题名称" % name)

        raw_parameters = {
            "output_rate": rospy.get_param("~output_rate", DEFAULT_PARAMS["output_rate"]),
            "cmd_timeout_s": rospy.get_param(
                "~cmd_timeout_s", DEFAULT_PARAMS["cmd_timeout_s"]
            ),
            "max_linear_accel": rospy.get_param(
                "~max_linear_accel", DEFAULT_PARAMS["max_linear_accel"]
            ),
            "max_angular_accel": rospy.get_param(
                "~max_angular_accel", DEFAULT_PARAMS["max_angular_accel"]
            ),
            "max_linear_speed": rospy.get_param(
                "~max_linear_speed", DEFAULT_PARAMS["max_linear_speed"]
            ),
            "max_lateral_speed": rospy.get_param(
                "~max_lateral_speed", DEFAULT_PARAMS["max_lateral_speed"]
            ),
            "max_angular_speed": rospy.get_param(
                "~max_angular_speed", DEFAULT_PARAMS["max_angular_speed"]
            ),
            "max_dt_s": rospy.get_param("~max_dt_s", DEFAULT_PARAMS["max_dt_s"]),
            "enable_safety": rospy.get_param("~enable_safety", True),
        }
        try:
            self.parameters = validate_parameters(raw_parameters)
        except ValueError as exc:
            self._reject_config(str(exc))

        self.output_rate = self.parameters["output_rate"]
        self.cmd_timeout_s = self.parameters["cmd_timeout_s"]
        # 保留旧属性名，避免同包内已有代码读取时失效。
        self.cmd_timeout = self.cmd_timeout_s
        self.enable_safety = self.parameters["enable_safety"]
        self.max_linear_accel = self.parameters["max_linear_accel"]
        self.max_angular_accel = self.parameters["max_angular_accel"]

        self._lock = threading.RLock()
        self._target_velocity = (0.0, 0.0, 0.0)
        self._has_valid_nav = False
        self._invalid_nav = False
        self._last_nav_time_sec = None
        self.last_nav_time = rospy.Time(0)
        self.safety_stop = False
        self.last_output = Twist()
        core_parameters = dict(self.parameters)
        core_parameters.pop("output_rate")
        self._core = CmdMuxCore(**core_parameters)
        self._core.last_output_time = self._now_seconds()

        # control 仍是唯一的最终 /cmd_vel 发布者；回显与其发布完全相同的消息。
        self.cmd_pub = rospy.Publisher(self.output_cmd_topic, Twist, queue_size=10)
        self.sent_cmd_pub = rospy.Publisher(self.sent_cmd_topic, Twist, queue_size=10)

        self.nav_sub = rospy.Subscriber(
            self.nav_cmd_topic, Twist, self.nav_cmd_callback
        )
        self.safety_sub = rospy.Subscriber(
            self.safety_stop_topic, Bool, self.safety_callback
        )
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.output_rate), self.output_loop
        )
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo(
            "[control] cmd_mux 已启动，输出 %s，回显 %s"
            % (self.output_cmd_topic, self.sent_cmd_topic)
        )

    @staticmethod
    def _reject_config(message):
        rospy.logfatal("[control] 拒绝启动：%s" % message)
        raise rospy.ROSException(message)

    @staticmethod
    def _now_seconds():
        return rospy.Time.now().to_sec()

    @staticmethod
    def _message_from_velocity(values):
        output = Twist()
        output.linear.x = values[0]
        output.linear.y = values[1]
        output.angular.z = values[2]
        return output

    @staticmethod
    def _zero_message():
        return Twist()

    def _publish_locked(self, output):
        """调用者持有锁时，同时发布最终输出和一模一样的回显。"""
        self.cmd_pub.publish(output)
        self.sent_cmd_pub.publish(output)

    def _clear_navigation_locked(self):
        self._target_velocity = (0.0, 0.0, 0.0)
        self._has_valid_nav = False
        self._invalid_nav = False
        self._last_nav_time_sec = None
        self.last_nav_time = rospy.Time(0)

    def nav_cmd_callback(self, msg):
        values = _finite_velocity(msg)
        with self._lock:
            if values is None:
                self._target_velocity = (0.0, 0.0, 0.0)
                self._has_valid_nav = False
                self._invalid_nav = True
                self._core.reset(self._now_seconds())
                output = self._zero_message()
                self.last_output = output
                self._publish_locked(output)
                rospy.logwarn_throttle(
                    1.0, "[control] 导航速度非法，已输出零速度"
                )
                return

            now = rospy.Time.now()
            self._target_velocity = values
            self._has_valid_nav = True
            self._invalid_nav = False
            self._last_nav_time_sec = now.to_sec()
            self.last_nav_time = now

    def safety_callback(self, msg):
        stop = bool(msg.data)
        with self._lock:
            was_stopped = self.safety_stop
            self.safety_stop = stop
            if stop:
                self._clear_navigation_locked()
                self._core.reset(self._now_seconds())
                output = self._zero_message()
                self.last_output = output
                self._publish_locked(output)
                if not was_stopped:
                    rospy.logwarn("[control] 外部急停已触发，持续输出零速度")
            elif was_stopped:
                # 解除急停不恢复旧命令，必须等待解除后的新鲜导航命令。
                self._clear_navigation_locked()
                self._core.reset(self._now_seconds())
                output = self._zero_message()
                self.last_output = output
                self._publish_locked(output)
                rospy.loginfo("[control] 外部急停已解除，等待新的有效导航命令")

    def output_loop(self, _event):
        """按固定优先级计算并发布一次输出。"""
        with self._lock:
            current_time = self._now_seconds()
            values, reason = self._core.step(
                current_time,
                self._target_velocity,
                self._has_valid_nav,
                self._last_nav_time_sec,
                safety_stop=self.safety_stop,
                invalid_nav=self._invalid_nav,
            )
            output = self._message_from_velocity(values)
            self.last_output = output
            self._publish_locked(output)

            if reason == "invalid":
                rospy.logwarn_throttle(
                    1.0, "[control] 导航速度非法，已输出零速度"
                )
            elif reason == "timeout":
                rospy.logwarn_throttle(
                    1.0, "[control] 导航命令超时，已输出零速度"
                )
            elif reason == "no_valid_nav":
                rospy.logwarn_throttle(
                    2.0, "[control] 尚未收到有效导航命令，持续输出零速度"
                )

    def shutdown(self):
        """节点退出时至少同步发送一次全零指令。"""
        with self._lock:
            self._core.reset(self._now_seconds())
            output = self._zero_message()
            self.last_output = output
            try:
                self._publish_locked(output)
            except Exception as exc:  # ROS 退出阶段可能已关闭通信连接。
                rospy.logwarn("[control] 退出停车发布失败：%s" % exc)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        CmdMux().run()
    except rospy.ROSInterruptException:
        pass
