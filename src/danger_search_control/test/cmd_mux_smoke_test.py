#!/usr/bin/env python3
"""在独立测试话题上验证 cmd_mux 的 ROS 运行时行为。"""

import threading

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


NAV_TOPIC = "/test/nav_cmd_vel"
OUTPUT_TOPIC = "/test/cmd_vel"
SENT_TOPIC = "/test/cmd_vel_sent"
SAFETY_TOPIC = "/test/safety_stop"


class OutputRecorder:
    """记录两个输出话题，供每个运行时断言使用。"""

    def __init__(self):
        self._lock = threading.Lock()
        self.outputs = []
        self.sent = []

    def output_callback(self, message):
        with self._lock:
            self.outputs.append(message)

    def sent_callback(self, message):
        with self._lock:
            self.sent.append(message)

    def latest_output(self):
        with self._lock:
            if not self.outputs:
                return None
            return self.outputs[-1]

    def echo_pairs_match(self):
        with self._lock:
            count = min(len(self.outputs), len(self.sent))
            if count == 0:
                return False
            for index in range(count):
                output = self.outputs[index]
                sent = self.sent[index]
                if (
                    output.linear.x != sent.linear.x
                    or output.linear.y != sent.linear.y
                    or output.angular.z != sent.angular.z
                    or output.linear.z != sent.linear.z
                    or output.angular.x != sent.angular.x
                    or output.angular.y != sent.angular.y
                ):
                    return False
            return True


def make_twist(x=0.0, y=0.0, z=0.0):
    message = Twist()
    message.linear.x = x
    message.linear.y = y
    message.angular.z = z
    return message


def is_zero(message, tolerance=1e-6):
    return (
        message is not None
        and abs(message.linear.x) <= tolerance
        and abs(message.linear.y) <= tolerance
        and abs(message.angular.z) <= tolerance
        and abs(message.linear.z) <= tolerance
        and abs(message.angular.x) <= tolerance
        and abs(message.angular.y) <= tolerance
    )


def wait_for(predicate, timeout_s, description):
    deadline = rospy.Time.now() + rospy.Duration(timeout_s)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        if predicate():
            return
        rate.sleep()
    raise RuntimeError("等待%s超时" % description)


def publish_for(publisher, message, duration_s, rate_hz=30):
    rate = rospy.Rate(rate_hz)
    deadline = rospy.Time.now() + rospy.Duration(duration_s)
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        publisher.publish(message)
        rate.sleep()


def require(condition, description):
    if not condition:
        raise RuntimeError("验证失败：%s" % description)


def main():
    rospy.init_node("cmd_mux_smoke_test", anonymous=True)
    recorder = OutputRecorder()
    nav_publisher = rospy.Publisher(NAV_TOPIC, Twist, queue_size=10)
    safety_publisher = rospy.Publisher(SAFETY_TOPIC, Bool, queue_size=10)
    rospy.Subscriber(OUTPUT_TOPIC, Twist, recorder.output_callback, queue_size=100)
    rospy.Subscriber(SENT_TOPIC, Twist, recorder.sent_callback, queue_size=100)

    wait_for(
        lambda: nav_publisher.get_num_connections() > 0
        and safety_publisher.get_num_connections() > 0,
        5.0,
        "cmd_mux 订阅测试输入话题",
    )
    wait_for(
        lambda: recorder.latest_output() is not None,
        5.0,
        "cmd_mux 初始零速度输出",
    )
    require(is_zero(recorder.latest_output()), "启动时未收到零速度")

    # 正常命令必须能在三轴上产生非零、受限的实际输出。
    publish_for(nav_publisher, make_twist(0.20, 0.10, 0.40), 0.35)
    wait_for(
        lambda: recorder.latest_output() is not None
        and recorder.latest_output().linear.x > 0.05
        and recorder.latest_output().linear.y > 0.05
        and recorder.latest_output().angular.z > 0.05,
        1.0,
        "正常三轴速度输出",
    )

    # 大速度经过限幅和加速度爬升后必须稳定在三个默认上限。
    publish_for(nav_publisher, make_twist(9.0, -9.0, 9.0), 0.70)
    limited = recorder.latest_output()
    require(abs(limited.linear.x - 0.40) < 0.05, "linear.x 未截断到 0.40")
    require(abs(limited.linear.y + 0.25) < 0.05, "linear.y 未截断到 -0.25")
    require(abs(limited.angular.z - 0.80) < 0.08, "angular.z 未截断到 0.80")

    # 停止发布导航命令后必须由超时保护立即归零。
    rospy.sleep(0.75)
    wait_for(lambda: is_zero(recorder.latest_output()), 1.0, "导航命令超时零速度")

    # 急停触发后立即归零；解除后不能恢复旧速度。
    publish_for(nav_publisher, make_twist(0.20, 0.10, 0.40), 0.25)
    safety_publisher.publish(Bool(data=True))
    wait_for(lambda: is_zero(recorder.latest_output()), 1.0, "急停零速度")
    safety_publisher.publish(Bool(data=False))
    rospy.sleep(0.15)
    require(is_zero(recorder.latest_output()), "急停解除后恢复了旧速度")

    # 解除急停后，新鲜命令应从零重新平滑增长。
    publish_for(nav_publisher, make_twist(0.20, 0.10, 0.40), 0.06)
    resumed = recorder.latest_output()
    require(0.0 < resumed.linear.x < 0.20, "解除急停后未从零平滑恢复")
    require(0.0 < resumed.linear.y < 0.10, "解除急停后横向轴未从零平滑恢复")
    require(0.0 < resumed.angular.z < 0.40, "解除急停后角速度未从零平滑恢复")

    wait_for(recorder.echo_pairs_match, 1.0, "cmd_vel 与 cmd_vel_sent 回显一致")
    print("通过：cmd_mux 隔离 ROS smoke test 完成，所有话题均为 /test/*。")


if __name__ == "__main__":
    try:
        main()
    except (rospy.ROSInterruptException, RuntimeError) as exc:
        rospy.logerr("[control] 隔离 smoke test 失败：%s" % exc)
        raise SystemExit(1)
