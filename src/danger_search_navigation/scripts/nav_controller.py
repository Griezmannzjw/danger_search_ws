#!/usr/bin/env python3
"""
导航控制节点 - move_base Action 服务器
对齐探索规划接口规范 v1.0

首版：简易P控制器 + 占位避障
升级路线：move_base 完整架构（global_planner + DWA/TEB local planner + costmap）

提供：
  - Action: /move_base (move_base_msgs/MoveBaseAction)
  - Service: /move_base/make_plan (nav_msgs/GetPlan)
  - Service: /move_base/clear_costmaps (std_srvs/Empty)
  - Topic: /navigation/path (nav_msgs/Path)
  - Topic: /navigation/health (NavigationHealth)
  - Topic: /danger_search/nav_cmd_vel (Twist) 输出给控制层安全仲裁
"""

import rospy
import actionlib
import math
import tf2_ros
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path, OccupancyGrid
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback
from nav_msgs.srv import GetPlan, GetPlanResponse
from std_srvs.srv import Empty, EmptyResponse
from danger_search_common.msg import NavigationHealth


class NavController:
    def __init__(self):
        rospy.init_node("nav_controller", anonymous=False)

        # 参数
        self.action_name = rospy.get_param("~move_base_action_name", "/move_base")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/danger_search/nav_cmd_vel")
        self.path_topic = rospy.get_param("~path_topic", "/navigation/path")
        self.health_topic = rospy.get_param("~health_topic", "/navigation/health")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base")

        self.max_linear = rospy.get_param("~max_linear_speed", 0.8)
        self.max_angular = rospy.get_param("~max_angular_speed", 1.0)
        self.linear_gain = rospy.get_param("~linear_gain", 0.8)
        self.angular_gain = rospy.get_param("~angular_gain", 1.5)
        self.goal_tol_xy = rospy.get_param("~goal_tolerance_xy", 0.15)
        self.goal_tol_yaw = rospy.get_param("~goal_tolerance_yaw", 0.2)
        self.control_rate = rospy.get_param("~control_rate", 20)
        self.goal_timeout = rospy.get_param("~goal_timeout", 30.0)

        # 状态
        self.current_pose = None
        self.active_goal = None
        self.goal_start_time = None
        self.active_goal_id = ""
        self.goal_counter = 0
        self.stuck_counter = 0
        self.last_position = None
        self.failure_code = ""
        self.failure_detail = ""
        self.controller_active = False

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 发布者
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=10, latch=True)
        self.health_pub = rospy.Publisher(self.health_topic, NavigationHealth, queue_size=10, latch=True)

        # 订阅者
        self.pose_sub = rospy.Subscriber("/localization/pose", PoseWithCovarianceStamped, self.pose_callback)

        # Action 服务器
        self.action_server = actionlib.SimpleActionServer(
            self.action_name,
            MoveBaseAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self.action_server.register_preempt_callback(self.preempt_cb)
        self.action_server.start()

        # 服务
        rospy.Service("/move_base/make_plan", GetPlan, self.make_plan_cb)
        rospy.Service("/move_base/clear_costmaps", Empty, self.clear_costmaps_cb)

        # 定时器
        self.health_timer = rospy.Timer(rospy.Duration(0.1), self.publish_health)

        rospy.loginfo("[navigation] move_base action server started (skeleton P-controller)")
        rospy.loginfo("[navigation] Action: %s", self.action_name)

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def _get_yaw(self, quat):
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def execute_cb(self, goal):
        """Action执行回调 - 导航到目标点"""
        self.goal_counter += 1
        self.active_goal_id = f"goal_{self.goal_counter}"
        self.active_goal = goal.target_pose
        self.goal_start_time = rospy.Time.now()
        self.controller_active = True
        self.stuck_counter = 0
        self.failure_code = ""
        self.failure_detail = ""

        rospy.loginfo(f"[navigation] New goal: {self.active_goal_id}")
        self._publish_path(goal.target_pose)

        rate = rospy.Rate(self.control_rate)
        result = MoveBaseResult()

        while not rospy.is_shutdown():
            if self.action_server.is_preempt_requested():
                rospy.loginfo(f"[navigation] Goal {self.active_goal_id} preempted")
                self._stop_robot()
                self.action_server.set_preempted()
                self.controller_active = False
                return

            if (rospy.Time.now() - self.goal_start_time).to_sec() > self.goal_timeout:
                self.failure_code = "TIMEOUT"
                self.failure_detail = "Goal exceeded timeout limit"
                self._stop_robot()
                self.action_server.set_aborted(result, "timeout")
                self.controller_active = False
                return

            if self.current_pose is None:
                rate.sleep()
                continue

            dx = self.active_goal.pose.position.x - self.current_pose.position.x
            dy = self.active_goal.pose.position.y - self.current_pose.position.y
            dist = math.sqrt(dx * dx + dy * dy)
            target_yaw = math.atan2(dy, dx)
            goal_yaw = self._get_yaw(self.active_goal.pose.orientation)
            current_yaw = self._get_yaw(self.current_pose.orientation)

            if dist < self.goal_tol_xy:
                yaw_err = self._normalize_angle(goal_yaw - current_yaw)
                if abs(yaw_err) < self.goal_tol_yaw:
                    rospy.loginfo(f"[navigation] Goal {self.active_goal_id} reached")
                    self._stop_robot()
                    self.action_server.set_succeeded(result)
                    self.controller_active = False
                    return

            angle_err = self._normalize_angle(target_yaw - current_yaw)
            cmd = Twist()
            if abs(angle_err) > 0.5:
                cmd.linear.x = 0.0
            else:
                cmd.linear.x = min(self.linear_gain * dist, self.max_linear)
            cmd.angular.z = max(-self.max_angular, min(self.max_angular, self.angular_gain * angle_err))

            # TODO: 避障检测
            self.cmd_vel_pub.publish(cmd)

            # 卡住检测
            if self.last_position:
                moved = math.sqrt(
                    (self.current_pose.position.x - self.last_position[0]) ** 2 +
                    (self.current_pose.position.y - self.last_position[1]) ** 2
                )
                if moved < 0.01 and cmd.linear.x > 0.1:
                    self.stuck_counter += 1
                else:
                    self.stuck_counter = max(0, self.stuck_counter - 1)
            self.last_position = (self.current_pose.position.x, self.current_pose.position.y)

            if self.stuck_counter > self.control_rate * 5:
                self.failure_code = "CONTROL_FAILED"
                self.failure_detail = "Robot appears stuck"
                self._stop_robot()
                self.action_server.set_aborted(result, "stuck")
                self.controller_active = False
                return

            # Feedback
            feedback = MoveBaseFeedback()
            feedback.base_position.header.stamp = rospy.Time.now()
            feedback.base_position.header.frame_id = self.map_frame
            feedback.base_position.pose = self.current_pose
            self.action_server.publish_feedback(feedback)

            rate.sleep()

    def preempt_cb(self):
        rospy.loginfo("[navigation] Preempt requested")
        self._stop_robot()

    def _stop_robot(self):
        stop = Twist()
        for _ in range(3):
            self.cmd_vel_pub.publish(stop)
            rospy.sleep(0.02)

    def _publish_path(self, goal_pose):
        """发布路径（首版直线路径占位）"""
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = self.map_frame
        start = PoseStamped()
        start.header = path.header
        if self.current_pose:
            start.pose = self.current_pose
        path.poses.append(start)
        end = PoseStamped()
        end.header = path.header
        end.pose = goal_pose.pose
        path.poses.append(end)
        self.path_pub.publish(path)

    def make_plan_cb(self, req):
        """make_plan服务 - 首版占位：直线路径"""
        resp = GetPlanResponse()
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = self.map_frame
        path.poses.append(req.start)
        path.poses.append(req.goal)
        resp.plan = path
        return resp

    def clear_costmaps_cb(self, req):
        rospy.loginfo("[navigation] clear_costmaps called (skeleton: no-op)")
        return EmptyResponse()

    def publish_health(self, event):
        health = NavigationHealth()
        health.header.stamp = rospy.Time.now()
        health.ready = self.current_pose is not None
        health.controller_active = self.controller_active
        health.stuck = self.stuck_counter > self.control_rate * 2
        health.fallen = False
        health.has_active_goal = self.active_goal is not None and self.controller_active
        health.active_goal_id = self.active_goal_id
        health.progress = 0.0
        health.last_cmd_time = rospy.Time.now() if self.controller_active else rospy.Time(0)
        health.failure_code = self.failure_code
        health.failure_detail = self.failure_detail
        self.health_pub.publish(health)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = NavController()
        node.run()
    except rospy.ROSInterruptException:
        pass
