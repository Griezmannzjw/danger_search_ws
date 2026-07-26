#!/usr/bin/env python3
"""
探索规划节点 - P0最小可运行版本
对齐接口规范 v1.1-p0

功能：
  - move_base Action客户端
  - 在地图自由区域随机选点
  - 发目标前严格检查所有条件
  - 失败不无限重试
  - stop时取消目标

P0要求：
  - 所有名称从参数读取
  - 只有满足所有条件才发目标
  - 目标必须在自由区域，make_plan返回非空路径
  - 失败候选不立即无限重试
  - stop时必须取消活动目标
"""

import rospy
import actionlib
import random
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Trigger, TriggerResponse
from nav_msgs.srv import GetPlan
from danger_search_common.msg import MappingStatus, NavigationHealth


class ExplorationPlanner:
    def __init__(self):
        rospy.init_node("exploration_planner", anonymous=False)

        # ========== 从参数读取所有名称 ==========
        self.map_frame = rospy.get_param("~map_frame", "map")

        self.map_topic = rospy.get_param("~map_topic", "/map")
        self.pose_topic = rospy.get_param("~pose_topic", "/localization/pose")
        self.mapping_status_topic = rospy.get_param("~mapping_status_topic", "/mapping/status")
        self.navigation_health_topic = rospy.get_param("~navigation_health_topic", "/navigation/health")
        self.move_base_action_name = rospy.get_param("~move_base_action_name", "/move_base")
        self.make_plan_service = rospy.get_param("~make_plan_service", "/move_base/make_plan")
        self.start_service = rospy.get_param("~start_service", "/danger_search/start_exploration")
        self.stop_service = rospy.get_param("~stop_service", "/danger_search/stop_exploration")

        # 探索参数
        self.goal_interval = rospy.get_param("~goal_interval", 2.0)
        self.max_retry = rospy.get_param("~max_retry", 3)
        self.min_goal_dist = rospy.get_param("~min_goal_dist", 2.0)
        self.max_goal_dist = rospy.get_param("~max_goal_dist", 8.0)

        # ========== 状态 ==========
        self.exploring = False
        self.current_pose = None
        self.current_map = None
        self.map_info = None
        self.map_data = None
        self.mapping_ready = False
        self.mapping_stable = False
        self.mapping_lost = True
        self.nav_ready = False
        self.waiting_for_result = False
        self.retry_count = 0
        self.last_goal_time = rospy.Time(0)

        # ========== Action客户端 ==========
        self.move_base_client = actionlib.SimpleActionClient(
            self.move_base_action_name, MoveBaseAction
        )

        # ========== 服务客户端 ==========
        rospy.wait_for_service(self.make_plan_service, timeout=5.0)
        self.make_plan_client = rospy.ServiceProxy(self.make_plan_service, GetPlan)

        # ========== 订阅者 ==========
        self.pose_sub = rospy.Subscriber(
            self.pose_topic, PoseWithCovarianceStamped, self.pose_callback
        )
        self.map_sub = rospy.Subscriber(
            self.map_topic, OccupancyGrid, self.map_callback
        )
        self.mapping_status_sub = rospy.Subscriber(
            self.mapping_status_topic, MappingStatus, self.mapping_status_callback
        )
        self.nav_health_sub = rospy.Subscriber(
            self.navigation_health_topic, NavigationHealth, self.nav_health_callback
        )

        # ========== 服务 ==========
        self.start_srv = rospy.Service(
            self.start_service, Trigger, self.start_exploration_cb
        )
        self.stop_srv = rospy.Service(
            self.stop_service, Trigger, self.stop_exploration_cb
        )

        # ========== 主循环 ==========
        self.planner_timer = rospy.Timer(rospy.Duration(0.5), self.planner_loop)

        rospy.loginfo(f"[exploration] Planner started, action: {self.move_base_action_name}")

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def map_callback(self, msg):
        self.current_map = msg
        self.map_info = msg.info
        self.map_data = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width)
        )

    def mapping_status_callback(self, msg):
        self.mapping_ready = msg.ready
        self.mapping_stable = msg.stable
        self.mapping_lost = msg.lost

    def nav_health_callback(self, msg):
        self.nav_ready = msg.ready

    def _world_to_map(self, x, y):
        mx = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        my = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
        return mx, my

    def _is_free(self, mx, my):
        if self.map_data is None:
            return False
        if mx < 0 or mx >= self.map_info.width or my < 0 or my >= self.map_info.height:
            return False
        return self.map_data[my, mx] == 0

    def _check_path(self, start_x, start_y, goal_x, goal_y):
        """调用make_plan检查路径是否存在"""
        try:
            start = PoseStamped()
            start.header.frame_id = self.map_frame
            start.pose.position.x = start_x
            start.pose.position.y = start_y
            start.pose.orientation.w = 1.0

            goal = PoseStamped()
            goal.header.frame_id = self.map_frame
            goal.pose.position.x = goal_x
            goal.pose.position.y = goal_y
            goal.pose.orientation.w = 1.0

            resp = self.make_plan_client(start, goal, 0.5)
            return len(resp.plan.poses) > 0
        except Exception as e:
            rospy.logwarn_throttle(5, f"[exploration] make_plan failed: {e}")
            return False

    def _select_goal(self):
        """在自由区域选择一个可达的目标点"""
        if self.current_pose is None or self.map_data is None:
            return None

        cx = self.current_pose.position.x
        cy = self.current_pose.position.y

        # 随机尝试选点
        for _ in range(50):
            angle = random.uniform(-math.pi, math.pi)
            dist = random.uniform(self.min_goal_dist, self.max_goal_dist)
            gx = cx + dist * math.cos(angle)
            gy = cy + dist * math.sin(angle)

            mx, my = self._world_to_map(gx, gy)
            if not self._is_free(mx, my):
                continue

            # 检查路径
            if self._check_path(cx, cy, gx, gy):
                return (gx, gy)

        return None

    def _send_goal(self, gx, gy):
        """发送导航目标"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.map_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = gx
        goal.target_pose.pose.position.y = gy
        goal.target_pose.pose.orientation.w = 1.0

        self.move_base_client.send_goal(goal, done_cb=self.goal_done_cb)
        self.waiting_for_result = True
        self.last_goal_time = rospy.Time.now()
        rospy.loginfo(f"[exploration] Sent goal: ({gx:.2f}, {gy:.2f})")

    def goal_done_cb(self, state, result):
        """目标完成回调"""
        self.waiting_for_result = False
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("[exploration] Goal succeeded")
            self.retry_count = 0
        else:
            rospy.loginfo(f"[exploration] Goal failed with state: {state}")
            self.retry_count += 1

    def start_exploration_cb(self, req):
        rospy.loginfo("[exploration] Start exploration")
        self.exploring = True
        self.retry_count = 0
        return TriggerResponse(success=True, message="Exploration started")

    def stop_exploration_cb(self, req):
        rospy.loginfo("[exploration] Stop exploration")
        self.exploring = False
        self.waiting_for_result = False
        # 取消活动目标
        if self.move_base_client.get_state() in [
            actionlib.GoalStatus.ACTIVE, actionlib.GoalStatus.PENDING
        ]:
            self.move_base_client.cancel_goal()
        self._stop_nav()
        return TriggerResponse(success=True, message="Exploration stopped")

    def _stop_nav(self):
        """停止导航（发0速度）"""
        pass  # navigation自己会处理，cancel后会停

    def planner_loop(self, event):
        """主规划循环"""
        if not self.exploring:
            return

        # 检查所有必要条件
        if not self.mapping_ready or not self.mapping_stable or self.mapping_lost:
            rospy.loginfo_throttle(5, "[exploration] Waiting for mapping to be ready...")
            return

        if not self.nav_ready:
            rospy.loginfo_throttle(5, "[exploration] Waiting for navigation to be ready...")
            return

        if self.current_pose is None or self.map_data is None:
            return

        if self.waiting_for_result:
            # 检查目标是否超时
            elapsed = (rospy.Time.now() - self.last_goal_time).to_sec()
            if elapsed > 60.0:
                rospy.logwarn("[exploration] Goal timeout, canceling")
                self.move_base_client.cancel_goal()
                self.waiting_for_result = False
                self.retry_count += 1
            return

        # 重试次数过多，等待一下
        if self.retry_count >= self.max_retry:
            rospy.loginfo_throttle(5, f"[exploration] Too many retries ({self.retry_count}), waiting...")
            return

        # 间隔时间
        if (rospy.Time.now() - self.last_goal_time).to_sec() < self.goal_interval:
            return

        # 选择目标
        goal = self._select_goal()
        if goal is not None:
            self._send_goal(*goal)
        else:
            rospy.loginfo_throttle(5, "[exploration] No valid goal found, retrying...")
            self.retry_count += 1

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = ExplorationPlanner()
        node.run()
    except rospy.ROSInterruptException:
        pass
