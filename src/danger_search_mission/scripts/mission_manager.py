#!/usr/bin/env python3
"""
任务总控节点 - P0最小可运行版本
对齐接口规范 v1.1-p0

状态机：IDLE → EXPLORING → FINISHED
         ↓
        ERROR

P0要求：
  - 所有名称从参数读取
  - start/finish严格按流程
  - 负责map->world坐标转换
  - 结果文件绝对路径
  - 只输出class_id=1的红球
  - 幂等性：重复start/finish返回可预测结果
"""

import rospy
import json
import os
import math
import actionlib
import tf2_ros
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, TriggerResponse
from move_base_msgs.msg import MoveBaseAction
from danger_search_common.msg import (
    DangerSourceArray, DangerSource, MissionStatus,
    MappingStatus, NavigationHealth
)


class MissionManager:
    def __init__(self):
        rospy.init_node("mission_manager", anonymous=False)

        # ========== 从参数读取所有名称 ==========
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.world_frame = rospy.get_param("~world_frame", "world")

        self.detections_topic = rospy.get_param("~detections_topic", "/danger_detector/detections")
        self.mapping_status_topic = rospy.get_param("~mapping_status_topic", "/mapping/status")
        self.navigation_health_topic = rospy.get_param("~navigation_health_topic", "/navigation/health")
        self.mission_status_topic = rospy.get_param("~mission_status_topic", "/mission/status")
        self.mission_active_topic = rospy.get_param("~mission_active_topic", "/mission/active")

        self.start_exploration_service = rospy.get_param(
            "~start_exploration_service", "/danger_search/start_exploration"
        )
        self.stop_exploration_service = rospy.get_param(
            "~stop_exploration_service", "/danger_search/stop_exploration"
        )
        self.move_base_action_name = rospy.get_param("~move_base_action_name", "/move_base")

        # 结果文件（必须是绝对路径，展开环境变量）
        default_result = os.path.expandvars(
            os.path.expanduser("~/SimEnv/results/detected_danger.json")
        )
        self.result_file = rospy.get_param("~result_file", default_result)
        self.result_file = os.path.expandvars(os.path.expanduser(self.result_file))
        self.dedup_distance = rospy.get_param("~dedup_distance", 0.8)

        # ========== 状态 ==========
        self.mission_state = "IDLE"
        self.start_time = None
        self.scored_time = 0.0
        self.current_floor = 0
        self.finish_reason = ""

        # 已确认的危险源列表
        self.confirmed_dangers = []  # list of (x, y, z)
        self.seen_detection_ids = set()

        # ========== TF（用于坐标转换） ==========
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # ========== 发布者 ==========
        self.status_pub = rospy.Publisher(
            self.mission_status_topic, MissionStatus, queue_size=10, latch=True
        )
        self.active_pub = rospy.Publisher(
            self.mission_active_topic, Bool, queue_size=10, latch=True
        )

        # 初始状态
        self._publish_status()
        self.active_pub.publish(Bool(data=False))

        # ========== 服务客户端 ==========
        rospy.loginfo("[mission] Waiting for exploration services...")
        rospy.wait_for_service(self.start_exploration_service)
        self.start_explore_client = rospy.ServiceProxy(
            self.start_exploration_service, Trigger
        )
        self.stop_explore_client = rospy.ServiceProxy(
            self.stop_exploration_service, Trigger
        )

        self.move_base_client = actionlib.SimpleActionClient(
            self.move_base_action_name, MoveBaseAction
        )

        # ========== 订阅者 ==========
        self.detections_sub = rospy.Subscriber(
            self.detections_topic, DangerSourceArray, self.detections_callback
        )

        # ========== 服务 ==========
        self.start_srv = rospy.Service(
            "/danger_search/start", Trigger, self.start_mission_cb
        )
        self.finish_srv = rospy.Service(
            "/danger_search/finish", Trigger, self.finish_mission_cb
        )

        # ========== 定时器 ==========
        self.status_timer = rospy.Timer(rospy.Duration(0.5), self._publish_status)

        rospy.loginfo(f"[mission] Mission manager started, result file: {self.result_file}")

    def detections_callback(self, msg):
        """接收检测结果，融合去重"""
        if self.mission_state != "EXPLORING":
            return

        for danger in msg.dangers:
            # 只处理红球危险源
            if danger.class_id != DangerSource.CLASS_DANGER_RED_SPHERE:
                continue

            # 去重：同一个detection_id不重复处理
            if danger.detection_id in self.seen_detection_ids:
                continue
            self.seen_detection_ids.add(danger.detection_id)

            # 坐标转换：map -> world
            pos = danger.position
            try:
                # 首版world和map重合，直接使用坐标
                wx = pos.point.x
                wy = pos.point.y
                wz = pos.point.z

                # 空间去重
                is_duplicate = False
                for (dx, dy, dz) in self.confirmed_dangers:
                    dist = math.sqrt((wx-dx)**2 + (wy-dy)**2 + (wz-dz)**2)
                    if dist < self.dedup_distance:
                        is_duplicate = True
                        break

                if not is_duplicate:
                    self.confirmed_dangers.append((wx, wy, wz))
                    rospy.loginfo(f"[mission] New danger source at ({wx:.2f}, {wy:.2f}, {wz:.2f}), total: {len(self.confirmed_dangers)}")

            except Exception as e:
                rospy.logwarn_throttle(5, f"[mission] Transform failed: {e}")

    def start_mission_cb(self, req):
        """开始任务"""
        if self.mission_state != "IDLE" and self.mission_state != "FINISHED" and self.mission_state != "ERROR":
            return TriggerResponse(success=False, message="Mission already running")

        rospy.loginfo("[mission] Starting mission...")

        # 重置状态
        self.mission_state = "IDLE"
        self.start_time = rospy.Time.now()
        self.scored_time = 0.0
        self.confirmed_dangers = []
        self.seen_detection_ids = set()
        self.finish_reason = ""

        # 调用exploration开始
        try:
            resp = self.start_explore_client()
            if not resp.success:
                self.mission_state = "ERROR"
                self.finish_reason = "Failed to start exploration"
                self._publish_status()
                return TriggerResponse(success=False, message=resp.message)
        except Exception as e:
            self.mission_state = "ERROR"
            self.finish_reason = f"Start exploration error: {e}"
            self._publish_status()
            return TriggerResponse(success=False, message=str(e))

        # 进入探索状态
        self.mission_state = "EXPLORING"
        self.active_pub.publish(Bool(data=True))
        self._publish_status()

        rospy.loginfo("[mission] Mission started, exploring...")
        return TriggerResponse(success=True, message="Mission started")

    def finish_mission_cb(self, req):
        """结束任务"""
        if self.mission_state == "FINISHED":
            return TriggerResponse(success=True, message="Already finished")

        rospy.loginfo("[mission] Finishing mission...")

        # 1. 停止探索
        try:
            self.stop_explore_client()
        except Exception as e:
            rospy.logwarn(f"[mission] Stop exploration error: {e}")

        # 2. 取消导航目标
        if self.move_base_client.get_state() in [
            actionlib.GoalStatus.ACTIVE, actionlib.GoalStatus.PENDING
        ]:
            self.move_base_client.cancel_goal()

        # 3. 冻结时间
        if self.start_time is not None:
            self.scored_time = (rospy.Time.now() - self.start_time).to_sec()

        # 4. 写结果文件
        try:
            self._write_result_file()
        except Exception as e:
            self.mission_state = "ERROR"
            self.finish_reason = f"Write result failed: {e}"
            self._publish_status()
            self.active_pub.publish(Bool(data=False))
            return TriggerResponse(success=False, message=str(e))

        # 5. 完成
        self.mission_state = "FINISHED"
        self.finish_reason = "Completed"
        self._publish_status()
        self.active_pub.publish(Bool(data=False))

        rospy.loginfo(f"[mission] Mission finished in {self.scored_time:.2f}s, found {len(self.confirmed_dangers)} dangers")
        return TriggerResponse(success=True, message=f"Finished, found {len(self.confirmed_dangers)} dangers")

    def _write_result_file(self):
        """写结果文件（原子写入）"""
        result = {
            "exploration_time": round(self.scored_time, 2),
            "detected_danger_sources": [
                {"position": [round(x, 2), round(y, 2), round(z, 2)]}
                for (x, y, z) in self.confirmed_dangers
            ]
        }

        # 确保目录存在
        dirname = os.path.dirname(self.result_file)
        if dirname and not os.path.exists(dirname):
            os.makedirs(dirname)

        # 原子写入：先写临时文件再rename
        tmp_file = self.result_file + ".tmp"
        with open(tmp_file, "w") as f:
            json.dump(result, f, indent=2)
        os.rename(tmp_file, self.result_file)

        rospy.loginfo(f"[mission] Result written to {self.result_file}")

    def _publish_status(self, event=None):
        """发布任务状态"""
        msg = MissionStatus()
        msg.header.stamp = rospy.Time.now()
        msg.mission_state = self.mission_state
        msg.current_floor = self.current_floor
        if self.start_time is not None:
            msg.start_time = self.start_time
            if self.mission_state == "EXPLORING":
                msg.elapsed_time = rospy.Time.now() - self.start_time
                msg.scored_exploration_time = msg.elapsed_time
            else:
                msg.elapsed_time = rospy.Duration(self.scored_time)
                msg.scored_exploration_time = rospy.Duration(self.scored_time)
        msg.finish_reason = self.finish_reason
        self.status_pub.publish(msg)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = MissionManager()
        node.run()
    except rospy.ROSInterruptException:
        pass
