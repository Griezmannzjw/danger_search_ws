#!/usr/bin/env python3
"""
任务总控节点 - 状态机 + 融合去重 + 结果输出
对齐探索规划接口规范 v1.0

状态机：IDLE → EXPLORING → RETURNING → FINISHED

输入：
  - /danger_detector/detections (DangerSourceArray)
  - /mapping/status (MappingStatus)
  - /navigation/health (NavigationHealth)
输出：
  - /mission/status (MissionStatus) latch
  - /mission/active (Bool) latch
服务：
  - /danger_search/start 开始任务
  - /danger_search/finish 结束任务并输出结果
  - /danger_search/return_home 触发返航
"""

import rospy
import json
import os
import math
import actionlib
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, TriggerResponse
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from danger_search_common.msg import (
    DangerSourceArray, DangerSource, MissionStatus,
    MappingStatus, NavigationHealth
)


class MissionManager:
    def __init__(self):
        rospy.init_node("mission_manager", anonymous=False)

        # 参数
        self.detections_topic = rospy.get_param("~detections_topic", "/danger_detector/detections")
        self.status_topic = rospy.get_param("~status_topic", "/mission/status")
        self.active_topic = rospy.get_param("~active_topic", "/mission/active")
        self.timeout = rospy.get_param("~exploration_timeout", 600.0)
        self.dedup_dist = rospy.get_param("~dedup_distance_threshold", 0.8)
        self.min_confirms = rospy.get_param("~min_detections_to_confirm", 2)
        self.result_file = rospy.get_param("~result_file", "results/detected_danger.json")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.home_x = rospy.get_param("~home_x", 0.0)
        self.home_y = rospy.get_param("~home_y", 0.0)
        self.home_yaw = rospy.get_param("~home_yaw", 0.0)

        # 状态机
        self.STATE_IDLE = "IDLE"
        self.STATE_EXPLORING = "EXPLORING"
        self.STATE_RETURNING = "RETURNING"
        self.STATE_FINISHED = "FINISHED"
        self.STATE_ERROR = "ERROR"
        self.state = self.STATE_IDLE

        # 任务数据
        self.start_time = None
        self.exploration_time = 0.0
        self.detected_sources = []
        self.tracked_detections = {}
        self.current_floor = 0
        self.remaining_frontier_count = 0
        self.correction_version = 0
        self.finish_reason = ""
        self.active_goal_id = ""

        # 发布者 (latch)
        self.status_pub = rospy.Publisher(self.status_topic, MissionStatus, queue_size=10, latch=True)
        self.active_pub = rospy.Publisher(self.active_topic, Bool, queue_size=10, latch=True)

        # 订阅者
        self.det_sub = rospy.Subscriber(self.detections_topic, DangerSourceArray, self.detections_cb)
        self.map_status_sub = rospy.Subscriber("/mapping/status", MappingStatus, self.map_status_cb)
        self.nav_health_sub = rospy.Subscriber("/navigation/health", NavigationHealth, self.nav_health_cb)

        # 服务
        rospy.Service("/danger_search/start", Trigger, self.start_cb)
        rospy.Service("/danger_search/finish", Trigger, self.finish_cb)
        rospy.Service("/danger_search/return_home", Trigger, self.return_home_cb)

        # 探索控制服务代理
        self.start_explore_srv = rospy.ServiceProxy("/danger_search/start_exploration", Trigger)
        self.stop_explore_srv = rospy.ServiceProxy("/danger_search/stop_exploration", Trigger)

        # 返航用 move_base 客户端
        self.move_base_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)

        # 状态发布定时器
        self.status_timer = rospy.Timer(rospy.Duration(0.5), self.publish_status)

        self._publish_initial_status()
        rospy.loginfo("[mission] Mission manager started, state = IDLE")

    def _publish_initial_status(self):
        self.publish_status(None)
        self.active_pub.publish(Bool(data=False))

    def detections_cb(self, msg):
        """检测结果回调 - 融合去重"""
        if self.state not in [self.STATE_EXPLORING, self.STATE_RETURNING]:
            return

        for danger in msg.dangers:
            # 只处理已确认的红球
            if danger.class_id != DangerSource.CLASS_DANGER_RED_SPHERE:
                continue
            if not danger.confirmed:
                continue

            pos = danger.position.point

            # 去重：距离阈值判断
            is_duplicate = False
            for src in self.detected_sources:
                dx = pos.x - src["position"][0]
                dy = pos.y - src["position"][1]
                dz = pos.z - src["position"][2]
                dist = math.sqrt(dx * dx + dy * dy + dz * dz)
                if dist < self.dedup_dist:
                    is_duplicate = True
                    src["detection_count"] += 1
                    w = 1.0 / src["detection_count"]
                    src["position"][0] = src["position"][0] * (1 - w) + pos.x * w
                    src["position"][1] = src["position"][1] * (1 - w) + pos.y * w
                    src["position"][2] = src["position"][2] * (1 - w) + pos.z * w
                    break

            if not is_duplicate:
                self.detected_sources.append({
                    "position": [pos.x, pos.y, pos.z],
                    "detection_count": 1
                })
                rospy.loginfo(f"[mission] New danger source confirmed, total: {len(self.detected_sources)}")

    def map_status_cb(self, msg):
        self.current_floor = msg.current_floor

    def nav_health_cb(self, msg):
        if msg.has_active_goal:
            self.active_goal_id = msg.active_goal_id

    def start_cb(self, req):
        if self.state != self.STATE_IDLE and self.state != self.STATE_FINISHED:
            resp = TriggerResponse()
            resp.success = False
            resp.message = f"Cannot start, current state: {self.state}"
            return resp

        self.state = self.STATE_EXPLORING
        self.start_time = rospy.Time.now()
        self.detected_sources = []
        self.tracked_detections = {}
        self.finish_reason = ""

        try:
            self.start_explore_srv.call()
        except rospy.ServiceException as e:
            rospy.logwarn(f"[mission] Failed to call start_exploration: {e}")

        self.active_pub.publish(Bool(data=True))
        rospy.loginfo("[mission] Mission started, state = EXPLORING")

        resp = TriggerResponse()
        resp.success = True
        resp.message = "Mission started"
        return resp

    def finish_cb(self, req):
        self.exploration_time = (rospy.Time.now() - self.start_time).to_sec() if self.start_time else 0.0
        self.state = self.STATE_FINISHED
        self.finish_reason = "manual_finish"

        try:
            self.stop_explore_srv.call()
        except:
            pass

        self._write_result()
        self.active_pub.publish(Bool(data=False))

        resp = TriggerResponse()
        resp.success = True
        resp.message = f"Mission finished, detected {len(self.detected_sources)} sources"
        return resp

    def return_home_cb(self, req):
        if self.state == self.STATE_IDLE or self.state == self.STATE_FINISHED:
            resp = TriggerResponse()
            resp.success = False
            resp.message = "Not exploring"
            return resp

        try:
            self.stop_explore_srv.call()
        except:
            pass

        self.state = self.STATE_RETURNING
        self.finish_reason = "return_home"

        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = self.map_frame
        goal.target_pose.pose.position.x = self.home_x
        goal.target_pose.pose.position.y = self.home_y
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.z = math.sin(self.home_yaw / 2)
        goal.target_pose.pose.orientation.w = math.cos(self.home_yaw / 2)

        self.move_base_client.send_goal(goal, done_cb=self._return_done_cb)
        rospy.loginfo("[mission] Returning home...")

        resp = TriggerResponse()
        resp.success = True
        resp.message = "Return home initiated"
        return resp

    def _return_done_cb(self, state, result):
        rospy.loginfo(f"[mission] Return home finished, state={state}")
        self.exploration_time = (rospy.Time.now() - self.start_time).to_sec() if self.start_time else 0.0
        self.state = self.STATE_FINISHED
        self._write_result()
        self.active_pub.publish(Bool(data=False))

    def _write_result(self):
        result = {
            "exploration_time": round(self.exploration_time, 2),
            "detected_danger_sources": [
                {"position": [round(p, 2) for p in src["position"]]}
                for src in self.detected_sources
            ]
        }
        os.makedirs(os.path.dirname(self.result_file) or ".", exist_ok=True)
        with open(self.result_file, "w") as f:
            json.dump(result, f, indent=2)
        rospy.loginfo(f"[mission] Result written to {self.result_file}")
        rospy.loginfo(f"[mission] Total detected: {len(self.detected_sources)} sources")

    def publish_status(self, event):
        status = MissionStatus()
        status.header.stamp = rospy.Time.now()
        status.header.frame_id = self.map_frame
        status.mission_state = self.state
        status.current_floor = self.current_floor

        if self.start_time:
            status.start_time = self.start_time
            elapsed = rospy.Time.now() - self.start_time
            status.elapsed_time = elapsed
            status.scored_exploration_time = elapsed
        else:
            status.start_time = rospy.Time(0)
            status.elapsed_time = rospy.Duration(0)
            status.scored_exploration_time = rospy.Duration(0)

        status.active_goal_id = self.active_goal_id
        status.map_coverage_summary = "skeleton: not computed"
        status.topology_debt_summary = "skeleton: not computed"
        status.room_visibility_summary = "skeleton: not computed"
        status.remaining_frontier_count = self.remaining_frontier_count
        status.localization_correction_version = self.correction_version
        status.finish_reason = self.finish_reason

        self.status_pub.publish(status)

        # 超时保护（600s评分阈值提醒，非硬截止）
        if self.state == self.STATE_EXPLORING and self.start_time:
            elapsed = (rospy.Time.now() - self.start_time).to_sec()
            if elapsed > self.timeout and elapsed % 60 < 1:
                rospy.logwarn_throttle(60, f"[mission] Exploration time > {self.timeout}s, scoring penalty starts")

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = MissionManager()
        node.run()
    except rospy.ROSInterruptException:
        pass
