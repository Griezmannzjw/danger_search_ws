#!/usr/bin/env python3
"""
速度仲裁节点 cmd_mux
功能：
  1. 接收多路速度指令，按优先级仲裁输出
  2. 指令超时自动停车（安全保护）
  3. 加速度限制（平滑输出）
  4. 回显已发送的速度指令（供定位模块使用）
输入：导航速度指令 / 安全停车指令 / 手动指令
输出：/cmd_vel（最终输出给机器人控制器）
      /danger_search/cmd_vel_sent（已发送指令回显）
"""

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class CmdMux:
    def __init__(self):
        rospy.init_node("cmd_mux", anonymous=False)

        # 参数
        self.output_rate = rospy.get_param("~output_rate", 50)
        self.cmd_timeout = rospy.get_param("~cmd_timeout_s", 0.5)
        self.enable_safety = rospy.get_param("~enable_safety", True)
        self.max_linear_accel = rospy.get_param("~max_linear_accel", 1.0)
        self.max_angular_accel = rospy.get_param("~max_angular_accel", 2.0)

        # 状态
        self.nav_cmd = Twist()
        self.safety_stop = False
        self.last_nav_time = rospy.Time(0)
        self.last_output = Twist()
        self.last_output_time = rospy.Time.now()

        # 发布者
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.sent_pub = rospy.Publisher(
            "/danger_search/cmd_vel_sent", Twist, queue_size=10
        )

        # 订阅者
        self.nav_sub = rospy.Subscriber(
            "/danger_search/nav_cmd_vel", Twist, self.nav_cmd_callback
        )
        self.safety_sub = rospy.Subscriber(
            "/danger_search/safety_stop", Bool, self.safety_callback
        )

        # 输出定时器
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.output_rate), self.output_loop
        )

        rospy.loginfo("[control] cmd_mux node started")

    def nav_cmd_callback(self, msg):
        self.nav_cmd = msg
        self.last_nav_time = rospy.Time.now()

    def safety_callback(self, msg):
        self.safety_stop = msg.data
        if msg.data:
            rospy.logwarn("[control] Safety stop activated!")

    def output_loop(self, event):
        """主输出循环：仲裁 + 安全检查 + 加速度限制"""
        current_time = rospy.Time.now()
        dt = (current_time - self.last_output_time).to_sec()
        self.last_output_time = current_time

        output = Twist()

        # 1. 安全停车：最高优先级
        if self.safety_stop:
            self.cmd_pub.publish(output)
            self.sent_pub.publish(output)
            self.last_output = output
            return

        # 2. 指令超时检查
        nav_age = (current_time - self.last_nav_time).to_sec()
        if self.enable_safety and nav_age > self.cmd_timeout:
            # 超时停车
            if nav_age < self.cmd_timeout + 0.1:
                rospy.logwarn_throttle(1,
                    f"[control] Nav cmd timeout ({nav_age:.2f}s), stopping")
            self.cmd_pub.publish(output)
            self.sent_pub.publish(output)
            self.last_output = output
            return

        # 3. 使用导航指令
        target_linear = self.nav_cmd.linear.x
        target_angular = self.nav_cmd.angular.z

        # 4. 加速度限制（平滑处理）
        delta_linear = target_linear - self.last_output.linear.x
        max_delta_linear = self.max_linear_accel * dt
        if abs(delta_linear) > max_delta_linear:
            delta_linear = max_delta_linear if delta_linear > 0 else -max_delta_linear
        output.linear.x = self.last_output.linear.x + delta_linear

        delta_angular = target_angular - self.last_output.angular.z
        max_delta_angular = self.max_angular_accel * dt
        if abs(delta_angular) > max_delta_angular:
            delta_angular = max_delta_angular if delta_angular > 0 else -max_delta_angular
        output.angular.z = self.last_output.angular.z + delta_angular

        # 5. 发布
        self.cmd_pub.publish(output)
        self.sent_pub.publish(output)
        self.last_output = output

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = CmdMux()
        node.run()
    except rospy.ROSInterruptException:
        pass
