#!/usr/bin/env python3
"""
危险源检测节点 - P0最小可运行版本
对齐接口规范 v1.1-p0

P0要求：
  - 所有名称从参数读取
  - 发布DangerSourceArray和DetectionStatus
  - position带正确frame_id和时间戳
  - P0字段：detection_id, class_id, position, floor_id=0, confidence, source_time

注意：P0版本只提供状态发布骨架，实际颜色/形状检测算法由识别组补充
"""

import rospy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped, Point
from danger_search_common.msg import (
    DangerSourceArray, DangerSource, DetectionStatus
)


class DangerDetector:
    def __init__(self):
        rospy.init_node("danger_detector", anonymous=False)

        # ========== 从参数读取所有名称 ==========
        self.map_frame = rospy.get_param("~map_frame", "map")

        self.rgb_topic = rospy.get_param("~rgb_topic", "/real_sense/rgb/image_raw")
        self.depth_topic = rospy.get_param("~depth_topic", "/real_sense/depth/image_raw")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/real_sense/rgb/camera_info")
        self.detections_topic = rospy.get_param("~detections_topic", "/danger_detector/detections")
        self.status_topic = rospy.get_param("~status_topic", "/danger_detector/status")

        # ========== 状态 ==========
        self.camera_info_received = False
        self.last_rgb_time = rospy.Time(0)
        self.last_depth_time = rospy.Time(0)
        self.detection_count = 0

        # ========== 发布者 ==========
        self.detections_pub = rospy.Publisher(
            self.detections_topic, DangerSourceArray, queue_size=10
        )
        self.status_pub = rospy.Publisher(
            self.status_topic, DetectionStatus, queue_size=10
        )

        # ========== 订阅者 ==========
        self.rgb_sub = rospy.Subscriber(
            self.rgb_topic, Image, self.rgb_callback, queue_size=1
        )
        self.depth_sub = rospy.Subscriber(
            self.depth_topic, Image, self.depth_callback, queue_size=1
        )
        self.camera_info_sub = rospy.Subscriber(
            self.camera_info_topic, CameraInfo, self.camera_info_callback
        )

        # ========== 定时器 ==========
        self.status_timer = rospy.Timer(rospy.Duration(0.5), self.publish_status)
        self.detect_timer = rospy.Timer(rospy.Duration(0.1), self.publish_empty_detections)

        rospy.loginfo("[perception] Danger detector P0 node started (skeleton)")

    def camera_info_callback(self, msg):
        self.camera_info_received = True

    def rgb_callback(self, msg):
        self.last_rgb_time = msg.header.stamp

    def depth_callback(self, msg):
        self.last_depth_time = msg.header.stamp

    def publish_empty_detections(self, event=None):
        """P0版本发布空检测数组（实际检测算法后续补充）"""
        msg = DangerSourceArray()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.map_frame
        msg.dangers = []
        self.detections_pub.publish(msg)

    def publish_status(self, event=None):
        """发布检测器状态"""
        now = rospy.Time.now()
        msg = DetectionStatus()
        msg.header.stamp = now
        msg.ready = self.camera_info_received
        msg.input_fresh = (
            (now - self.last_rgb_time).to_sec() < 1.0 and
            (now - self.last_depth_time).to_sec() < 1.0
        )
        msg.input_latency_ms = (now - self.last_rgb_time).to_sec() * 1000
        msg.total_detections = self.detection_count
        msg.confirmed_count = self.detection_count
        msg.pending_verification = 0
        msg.capability_version = 1
        if not self.camera_info_received:
            msg.status_reason = "WAITING_FOR_CAMERA_INFO"
        elif not msg.input_fresh:
            msg.status_reason = "INPUT_STALE"
        else:
            msg.status_reason = "OK (P0 skeleton, detection algorithm pending)"
        self.status_pub.publish(msg)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = DangerDetector()
        node.run()
    except rospy.ROSInterruptException:
        pass
