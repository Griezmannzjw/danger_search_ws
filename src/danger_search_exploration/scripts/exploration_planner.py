#!/usr/bin/env python3
"""
探索规划节点 - move_base Action 客户端
对齐探索规划接口规范 v1.0

首版：随机游走占位
升级路线：前沿点检测 + 信息增益评估 + 房间覆盖规划

输入：
  - /map (OccupancyGrid)
  - /localization/pose (PoseWithCovarianceStamped)
  - /localization/status (LocalizationStatus)
  - /mapping/status (MappingStatus)
  - /navigation/health (NavigationHealth)
  - /danger_detector/detections (DangerSourceArray)
  - /danger_detector/status (DetectionStatus)

输出：
  - /move_base Action 目标
  - 服务: /danger_search/start_exploration, /danger_search/stop_exploration
"""

import rospy
import actionlib
import random
import math
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Trigger, TriggerResponse
from danger_search_common.msg import (
    LocalizationStatus, MappingStatus, NavigationHealth,
    DangerSourceArray, DetectionStatus
)


class ExplorationPlanner:
    def __init__(self):
        rospy.init_node("exploration_planner", anonymous=False)

        # 参数
        self.move_base_action = rospy.get_param("~move_base_action_name", "/move_base")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.replan_interval = rospy.get_param("~replan_interval", 3.0)
        self.home_x = rospy.get_param("~home_x", 0.0)
        self.home_y = rospy.get_param("~home_y", 0.0)
        self.random_range_x = rospy.get_param("~random_range_x", 5.0)
        self.random_range_y = rospy.get_param("~random_range_y", 5.0)

        # 状态
        self.exploring = False
        self.current_pose = None
        self.current_map = None
        self.loc_status = None
        self.map_status = None
        self.nav_health = None
        self.det_status = None
        self.last_goal_time = rospy.Time(0)
        self.goal_active = False

        # Action 客户端
        self.move_base_client = actionlib.SimpleActionClient(
            self.move_base_action, MoveBaseAction
        )
        rospy.loginfo("[exploration] Waiting for move_base action server...")
        server_found = self.move_base_client.wait_for_server(rospy.Duration(2.0))
        if server_found:
            rospy.loginfo("[exploration] move_base action server connected")
        else:
            rospy.logwarn("[exploration] move_base server not found yet (will retry)")

        # 订阅者
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.pose_sub = rospy.Subscriber("/localization/pose", PoseWithCovarianceStamped, self.pose_callback)
        self.loc_status_sub = rospy.Subscriber("/localization/status", LocalizationStatus, self.loc_status_cb)
        self.map_status_sub = rospy.Subscriber("/mapping/status", MappingStatus, self.map_status_cb)
        self.nav_health_sub = rospy.Subscriber("/navigation/health", NavigationHealth, self.nav_health_cb)
        self.det_sub = rospy.Subscriber("/danger_detector/detections", DangerSourceArray, self.detections_cb)
        self.det_status_sub = rospy.Subscriber("/danger_detector/status", DetectionStatus, self.det_status_cb)

        # 服务
        rospy.Service("/danger_search/start_exploration", Trigger, self.start_cb)
        rospy.Service("/danger_search/stop_exploration", Trigger, self.stop_cb)

        # 主循环定时器
        self.timer = rospy.Timer(rospy.Duration(self.replan_interval), self.planner_loop)

        rospy.loginfo("[exploration] exploration_planner started (skeleton random walk)")

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def map_callback(self, msg):
        self.current_map = msg

    def loc_status_cb(self, msg):
        self.loc_status = msg

    def map_status_cb(self, msg):
        self.map_status = msg

    def nav_health_cb(self, msg):
        self.nav_health = msg
        if self.goal_active and not msg.has_active_goal:
            self.goal_active = False

    def detections_cb(self, msg):
        pass

    def det_status_cb(self, msg):
        self.det_status = msg

    def start_cb(self, req):
        self.exploring = True
        rospy.loginfo("[exploration] Exploration started")
        resp = TriggerResponse()
        resp.success = True
        resp.message = "Exploration started"
        return resp

    def stop_cb(self, req):
        self.exploring = False
        if self.goal_active:
            self.move_base_client.cancel_goal()
            self.goal_active = False
        rospy.loginfo("[exploration] Exploration stopped")
        resp = TriggerResponse()
        resp.success = True
        resp.message = "Exploration stopped"
        return resp

    def planner_loop(self, event):
        """探索主循环 - 首版随机游走占位"""
        if not self.exploring:
            return
        if self.current_pose is None:
            return
        if self.goal_active:
            return

        # 定位状态检查（DEGRADED/LOST时不发新目标）
        if self.loc_status and self.loc_status.tracking_state in [
            LocalizationStatus.STATE_RELOCALIZING, LocalizationStatus.STATE_LOST
        ]:
            return

        # 地图不稳定时不发新目标
        if self.map_status and not self.map_status.stable:
            return

        # TODO: 替换为前沿点检测 + 信息增益评估
        goal_x = self.current_pose.position.x + random.uniform(-self.random_range_x, self.random_range_x)
        goal_y = self.current_pose.position.y + random.uniform(-self.random_range_y, self.random_range_y)
        goal_yaw = random.uniform(-math.pi, math.pi)
        self._send_goal(goal_x, goal_y, goal_yaw)

    def _send_goal(self, x, y, yaw):
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = self.map_frame
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.position.z = 0.0
        goal.target_pose.pose.orientation.z = math.sin(yaw / 2)
        goal.target_pose.pose.orientation.w = math.cos(yaw / 2)

        self.move_base_client.send_goal(
            goal,
            done_cb=self._goal_done_cb,
            active_cb=self._goal_active_cb
        )
        self.goal_active = True
        rospy.loginfo(f"[exploration] Sent goal: ({x:.2f}, {y:.2f})")

    def _goal_active_cb(self):
        pass

    def _goal_done_cb(self, state, result):
        self.goal_active = False
        rospy.loginfo(f"[exploration] Goal finished, state={state}")

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = ExplorationPlanner()
        node.run()
    except rospy.ROSInterruptException:
        pass
