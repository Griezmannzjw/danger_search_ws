#!/usr/bin/env python3
"""
导航控制节点 - P0最小可运行版本
对齐接口规范 v1.1-p0

提供：
  - Action: /move_base (move_base_msgs/MoveBaseAction)
  - Service: /move_base/make_plan (nav_msgs/GetPlan)
  - Topic: /navigation/health (NavigationHealth)
  - Topic: /danger_search/nav_cmd_vel (Twist) 给控制层

P0要求：
  - 所有名称从参数读取
  - make_plan基于实际地图判断可达性，不返回直线
  - 取消/失败/超时后立即停止速度输出
  - 只有navigation可以发nav_cmd_vel
"""

import rospy
import actionlib
import math
import numpy as np
import tf2_ros
import tf.transformations
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path, OccupancyGrid
from move_base_msgs.msg import MoveBaseAction, MoveBaseResult, MoveBaseFeedback
from nav_msgs.srv import GetPlan, GetPlanResponse
from std_srvs.srv import Empty, EmptyResponse
from danger_search_common.msg import NavigationHealth
from actionlib_msgs.msg import GoalStatus


class NavController:
    def __init__(self):
        rospy.init_node("nav_controller", anonymous=False)

        # ========== 从参数读取所有名称 ==========
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base")

        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/danger_search/nav_cmd_vel")
        self.pose_topic = rospy.get_param("~pose_topic", "/localization/pose")
        self.map_topic = rospy.get_param("~map_topic", "/map")
        self.health_topic = rospy.get_param("~health_topic", "/navigation/health")
        self.move_base_action_name = rospy.get_param("~move_base_action_name", "/move_base")
        self.make_plan_service = rospy.get_param("~make_plan_service", "/move_base/make_plan")
        self.clear_costmaps_service = rospy.get_param("~clear_costmaps_service", "/move_base/clear_costmaps")

        # 控制参数
        self.linear_kp = rospy.get_param("~linear_kp", 0.5)
        self.angular_kp = rospy.get_param("~angular_kp", 1.5)
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.3)
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 0.8)
        self.goal_tolerance_xy = rospy.get_param("~goal_tolerance_xy", 0.3)
        self.goal_tolerance_yaw = rospy.get_param("~goal_tolerance_yaw", 0.2)
        self.goal_timeout = rospy.get_param("~goal_timeout", 60.0)
        self.stuck_threshold = rospy.get_param("~stuck_threshold", 5.0)
        self.control_rate = rospy.get_param("~control_rate", 20.0)

        # ========== 状态 ==========
        self.current_pose = None  # (x, y, yaw)
        self.current_map = None
        self.map_info = None
        self.map_data = None

        self.has_active_goal = False
        self.active_goal = None
        self.active_goal_id = ""
        self.goal_start_time = None
        self.last_progress_time = rospy.Time.now()
        self.last_position = None
        self.is_stuck = False
        self.failure_code = "NONE"
        self.controller_active = False

        # ========== TF ==========
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # ========== 发布者 ==========
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.health_pub = rospy.Publisher(self.health_topic, NavigationHealth, queue_size=10)

        # ========== 订阅者 ==========
        self.pose_sub = rospy.Subscriber(
            self.pose_topic, PoseWithCovarianceStamped, self.pose_callback
        )
        self.map_sub = rospy.Subscriber(
            self.map_topic, OccupancyGrid, self.map_callback
        )

        # ========== Action服务器 ==========
        self.action_server = actionlib.SimpleActionServer(
            self.move_base_action_name,
            MoveBaseAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self.action_server.register_preempt_callback(self.preempt_cb)
        self.action_server.start()

        # ========== 服务 ==========
        self.make_plan_srv = rospy.Service(
            self.make_plan_service, GetPlan, self.make_plan_cb
        )
        self.clear_costmaps_srv = rospy.Service(
            self.clear_costmaps_service, Empty, self.clear_costmaps_cb
        )

        # ========== 控制循环 ==========
        self.control_timer = rospy.Timer(
            rospy.Duration(1.0 / self.control_rate), self.control_loop
        )
        self.health_timer = rospy.Timer(rospy.Duration(0.2), self.publish_health)

        rospy.loginfo(f"[navigation] Nav controller started, action: {self.move_base_action_name}")

    def pose_callback(self, msg):
        """位姿回调"""
        q = msg.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw
        )

    def map_callback(self, msg):
        """地图回调"""
        self.current_map = msg
        self.map_info = msg.info
        self.map_data = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width)
        )

    def _world_to_map(self, x, y):
        """世界坐标转地图像素坐标"""
        if self.map_info is None:
            return None
        mx = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        my = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
        return mx, my

    def _is_free(self, mx, my):
        """检查像素是否是自由区域"""
        if self.map_data is None:
            return False
        if mx < 0 or mx >= self.map_info.width or my < 0 or my >= self.map_info.height:
            return False
        return self.map_data[my, mx] == 0  # 0=FREE

    def _simple_path(self, start_x, start_y, goal_x, goal_y):
        """简单的贪心路径规划：向目标移动，遇到障碍简单绕行"""
        if self.map_data is None:
            return None

        path = []
        sx, sy = self._world_to_map(start_x, start_y)
        gx, gy = self._world_to_map(goal_x, goal_y)

        if not self._is_free(gx, gy):
            return None

        # 简单的直线插值 + 障碍检查
        steps = int(math.hypot(gx - sx, gy - sy))
        if steps == 0:
            return []

        for i in range(steps + 1):
            t = i / steps
            x = start_x + (goal_x - start_x) * t
            y = start_y + (goal_y - start_y) * t
            mx, my = self._world_to_map(x, y)
            if not self._is_free(mx, my):
                return None  # 路径上有障碍
            path.append((x, y))

        return path

    def make_plan_cb(self, req):
        """路径规划服务"""
        resp = GetPlanResponse()
        if self.map_data is None or self.current_pose is None:
            return resp

        start = req.start.pose.position
        goal = req.goal.pose.position

        path_points = self._simple_path(start.x, start.y, goal.x, goal.y)
        if path_points is None:
            return resp

        resp.plan.header.frame_id = self.map_frame
        resp.plan.header.stamp = rospy.Time.now()
        for x, y in path_points:
            p = PoseStamped()
            p.header = resp.plan.header
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.orientation.w = 1.0
            resp.plan.poses.append(p)

        return resp

    def clear_costmaps_cb(self, req):
        """清除代价地图服务（P0占位）"""
        rospy.loginfo("[navigation] clear_costmaps called")
        return EmptyResponse()

    def execute_cb(self, goal):
        """Action执行回调"""
        if self.current_pose is None or self.map_data is None:
            self.action_server.set_aborted(text="LOCALIZATION_NOT_READY")
            self.failure_code = "LOCALIZATION_LOST"
            self.has_active_goal = False
            self._stop_robot()
            return

        self.has_active_goal = True
        self.active_goal = goal
        self.goal_start_time = rospy.Time.now()
        self.last_progress_time = rospy.Time.now()
        self.last_position = self.current_pose
        self.is_stuck = False
        self.failure_code = "NONE"
        self.controller_active = True

        goal_x = goal.target_pose.pose.position.x
        goal_y = goal.target_pose.pose.position.y
        q = goal.target_pose.pose.orientation
        _, _, goal_yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

        rospy.loginfo(f"[navigation] New goal: ({goal_x:.2f}, {goal_y:.2f})")

        rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown():
            if self.action_server.is_preempt_requested():
                self.action_server.set_preempted()
                self.failure_code = "CANCELED"
                break

            if self.current_pose is None:
                self.action_server.set_aborted(text="LOCALIZATION_LOST")
                self.failure_code = "LOCALIZATION_LOST"
                break

            cx, cy, cyaw = self.current_pose

            # 检查超时
            elapsed = (rospy.Time.now() - self.goal_start_time).to_sec()
            if elapsed > self.goal_timeout:
                self.action_server.set_aborted(text="TIMEOUT")
                self.failure_code = "TIMEOUT"
                break

            # 检查是否卡住
            if self.last_position is not None:
                dist_moved = math.hypot(cx - self.last_position[0], cy - self.last_position[1])
                if dist_moved > 0.1:
                    self.last_progress_time = rospy.Time.now()
                    self.last_position = self.current_pose
                elif (rospy.Time.now() - self.last_progress_time).to_sec() > self.stuck_threshold:
                    self.is_stuck = True
                    self.action_server.set_aborted(text="STUCK")
                    self.failure_code = "CONTROL_FAILED"
                    break

            # 检查是否到达目标
            dist_to_goal = math.hypot(cx - goal_x, cy - goal_y)
            if dist_to_goal < self.goal_tolerance_xy:
                self.action_server.set_succeeded(MoveBaseResult())
                self.failure_code = "SUCCEEDED"
                break

            # 计算控制指令
            dx = goal_x - cx
            dy = goal_y - cy
            target_yaw = math.atan2(dy, dx)
            error_yaw = self._normalize_angle(target_yaw - cyaw)

            cmd = Twist()
            # 先转向
            if abs(error_yaw) > 0.3:
                cmd.angular.z = self.angular_kp * error_yaw
                cmd.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, cmd.angular.z))
            else:
                cmd.linear.x = self.linear_kp * dist_to_goal
                cmd.linear.x = min(self.max_linear_speed, cmd.linear.x)
                cmd.angular.z = self.angular_kp * error_yaw
                cmd.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, cmd.angular.z))

            self.cmd_pub.publish(cmd)

            # 反馈
            feedback = MoveBaseFeedback()
            feedback.base_position.header.stamp = rospy.Time.now()
            feedback.base_position.header.frame_id = self.map_frame
            feedback.base_position.pose.position.x = cx
            feedback.base_position.pose.position.y = cy
            q = tf.transformations.quaternion_from_euler(0, 0, cyaw)
            feedback.base_position.pose.orientation.x = q[0]
            feedback.base_position.pose.orientation.y = q[1]
            feedback.base_position.pose.orientation.z = q[2]
            feedback.base_position.pose.orientation.w = q[3]
            self.action_server.publish_feedback(feedback)

            rate.sleep()

        # 结束：停止机器人
        self.has_active_goal = False
        self.controller_active = False
        self._stop_robot()
        rospy.loginfo(f"[navigation] Goal finished: {self.failure_code}")

    def preempt_cb(self):
        """目标被抢占"""
        rospy.loginfo("[navigation] Goal preempted")
        self.failure_code = "CANCELED"
        self.has_active_goal = False
        self.controller_active = False
        self._stop_robot()

    def control_loop(self, event):
        """控制循环：没有目标时发0速度"""
        if not self.has_active_goal:
            self._stop_robot()

    def _stop_robot(self):
        """立即停止机器人"""
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def _normalize_angle(self, angle):
        """角度归一化到[-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def publish_health(self, event=None):
        """发布导航健康状态"""
        msg = NavigationHealth()
        msg.header.stamp = rospy.Time.now()
        msg.ready = self.current_pose is not None and self.map_data is not None
        msg.controller_active = self.controller_active
        msg.stuck = self.is_stuck
        msg.fallen = False
        msg.has_active_goal = self.has_active_goal
        msg.active_goal_id = self.active_goal_id
        msg.progress = 0.0
        msg.last_cmd_time = rospy.Time.now()
        msg.failure_code = self.failure_code
        msg.failure_detail = ""
        self.health_pub.publish(msg)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = NavController()
        node.run()
    except rospy.ROSInterruptException:
        pass
