#!/usr/bin/env python3
"""
危险源检测节点
对齐探索规划接口规范 v1.0

输入：RGB图像、深度图像、相机内参、TF（camera -> map）
输出：
  - /danger_detector/detections (DangerSourceArray)
  - /danger_detector/status (DetectionStatus)
"""

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformListener
from image_geometry import PinholeCameraModel
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped
from danger_search_common.msg import (
    DangerSourceArray, DangerSource, DetectionStatus
)


class DangerDetector:
    def __init__(self):
        rospy.init_node("danger_detector", anonymous=False)

        # 参数加载
        self.rgb_topic = rospy.get_param("~rgb_topic", "/real_sense/rgb/image_raw")
        self.depth_topic = rospy.get_param("~depth_topic", "/real_sense/depth/image_raw")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/real_sense/rgb/camera_info")
        self.camera_frame = rospy.get_param("~camera_frame", "real_sense")
        self.target_frame = rospy.get_param("~target_frame", "map")
        self.detections_topic = rospy.get_param("~detections_topic", "/danger_detector/detections")
        self.status_topic = rospy.get_param("~status_topic", "/danger_detector/status")
        self.publish_rate = rospy.get_param("~publish_rate", 10)
        self.cap_version = rospy.get_param("~capability/capability_version", 1)

        self.bridge = CvBridge()
        self.camera_model = PinholeCameraModel()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        self.camera_info_received = False
        self.latest_depth = None
        self.last_rgb_time = rospy.Time(0)
        self.detection_counter = 0
        self.track_counter = 0

        # 发布者
        self.detections_pub = rospy.Publisher(
            self.detections_topic, DangerSourceArray, queue_size=10
        )
        self.status_pub = rospy.Publisher(
            self.status_topic, DetectionStatus, queue_size=10, latch=True
        )

        # 订阅者
        self.rgb_sub = rospy.Subscriber(self.rgb_topic, Image, self.rgb_callback)
        self.depth_sub = rospy.Subscriber(self.depth_topic, Image, self.depth_callback)
        self.camera_info_sub = rospy.Subscriber(
            self.camera_info_topic, CameraInfo, self.camera_info_callback
        )

        # 状态发布定时器
        self.status_timer = rospy.Timer(rospy.Duration(0.2), self.publish_status)

        rospy.loginfo("[perception] danger_detector node started")

    def camera_info_callback(self, msg):
        if not self.camera_info_received:
            self.camera_model.fromCameraInfo(msg)
            self.camera_info_received = True
            rospy.loginfo("[perception] Camera info received")

    def depth_callback(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            rospy.logwarn_throttle(1, f"[perception] Depth convert error: {e}")

    def rgb_callback(self, msg):
        """检测主循环 - 首版为骨架"""
        self.last_rgb_time = msg.header.stamp

        if not self.camera_info_received or self.latest_depth is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logwarn_throttle(1, f"[perception] RGB convert error: {e}")
            return

        # ============================================================
        # TODO: 红色球体检测 + 三维定位（感知组实现）
        # 1. HSV转换 + 红色掩膜（红球/红方块/绿球分类）
        # 2. 形态学去噪
        # 3. 轮廓提取 + 圆形度/形状过滤
        # 4. 深度采样 + 像素反投影到相机坐标系
        # 5. TF转换到map坐标系
        # 6. 跨帧跟踪，分配track_id
        # 7. 协方差估计
        # ============================================================

        # 骨架：空输出
        detections = DangerSourceArray()
        detections.header.stamp = msg.header.stamp
        detections.header.frame_id = self.target_frame
        # detections.dangers 列表为空

        self.detections_pub.publish(detections)

    def publish_status(self, event):
        """发布检测器状态"""
        status = DetectionStatus()
        status.header.stamp = rospy.Time.now()
        status.ready = self.camera_info_received
        latency = (rospy.Time.now() - self.last_rgb_time).to_sec() * 1000 if self.last_rgb_time.to_sec() > 0 else 9999
        status.input_fresh = latency < 500  # 500ms内有输入
        status.input_latency_ms = latency
        status.total_detections = 0  # TODO: 实际检测数
        status.confirmed_count = 0
        status.pending_verification = 0
        status.capability_version = self.cap_version
        status.status_reason = "skeleton: detection algorithm TBD" if not self.camera_info_received else "ready, waiting for detection impl"

        self.status_pub.publish(status)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = DangerDetector()
        node.run()
    except rospy.ROSInterruptException:
        pass
