#!/usr/bin/env python3
"""
位姿估计与建图节点
对齐探索规划接口规范 v1.0

首版：简易航位推算 + 占位地图
升级路线：FAST-LIO / Cartographer / GMapping

输出：
  - /localization/pose (PoseWithCovarianceStamped) 带协方差位姿
  - /localization/status (LocalizationStatus) 定位健康状态
  - /map (nav_msgs/OccupancyGrid) 栅格地图
  - /mapping/status (MappingStatus) 建图状态
  - /mapping/current_floor (std_msgs/Int32) 当前楼层
  - TF: map -> odom -> base
"""

import rospy
import tf2_ros
import math
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int32
from danger_search_common.msg import LocalizationStatus, MappingStatus, FloorMapInfo


class PoseEstimator:
    def __init__(self):
        rospy.init_node("pose_estimator", anonymous=False)

        # 参数
        self.imu_topic = rospy.get_param("~imu_topic", "/trunk_imu")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/danger_search/cmd_vel_sent")
        self.pose_topic = rospy.get_param("~pose_topic", "/localization/pose")
        self.status_topic = rospy.get_param("~status_topic", "/localization/status")
        self.map_topic = rospy.get_param("~map_topic", "/map")
        self.mapping_status_topic = rospy.get_param("~mapping_status_topic", "/mapping/status")
        self.current_floor_topic = rospy.get_param("~current_floor_topic", "/mapping/current_floor")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base")
        self.pose_rate = rospy.get_param("~pose_publish_rate", 50)
        self.current_floor = rospy.get_param("~current_floor", 0)

        # 状态
        self.x = rospy.get_param("~initial_x", 0.0)
        self.y = rospy.get_param("~initial_y", 0.0)
        self.yaw = rospy.get_param("~initial_yaw", 0.0)
        self.vx = 0.0
        self.vyaw = 0.0

        self.last_time = rospy.Time.now()
        self.last_cmd_time = rospy.Time.now()
        self.correction_version = 0
        self.last_stable_time = rospy.Time.now()

        # TF发布
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster()

        # 发布者
        self.pose_pub = rospy.Publisher(self.pose_topic, PoseWithCovarianceStamped, queue_size=10)
        self.status_pub = rospy.Publisher(self.status_topic, LocalizationStatus, queue_size=10, latch=True)
        self.map_pub = rospy.Publisher(self.map_topic, OccupancyGrid, queue_size=1, latch=True)
        self.mapping_status_pub = rospy.Publisher(self.mapping_status_topic, MappingStatus, queue_size=10, latch=True)
        self.floor_pub = rospy.Publisher(self.current_floor_topic, Int32, queue_size=10, latch=True)

        # 订阅者
        self.imu_sub = rospy.Subscriber(self.imu_topic, Imu, self.imu_callback)
        self.cmd_vel_sub = rospy.Subscriber(self.cmd_vel_topic, Twist, self.cmd_vel_callback)

        # 定时器
        self.pose_timer = rospy.Timer(rospy.Duration(1.0 / self.pose_rate), self.publish_pose)
        self.status_timer = rospy.Timer(rospy.Duration(0.1), self.publish_status)
        self.map_timer = rospy.Timer(rospy.Duration(1.0), self.publish_map)

        # 发布静态TF: map -> odom
        self._publish_static_map_odom()

        # 发布初始楼层
        self.floor_pub.publish(Int32(data=self.current_floor))

        rospy.loginfo("[localization] pose_estimator started (skeleton: dead reckoning)")
        rospy.loginfo("[localization] Output: %s, %s, %s", self.pose_topic, self.status_topic, self.map_topic)

    def _publish_static_map_odom(self):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_static_broadcaster.sendTransform(t)

    def imu_callback(self, msg):
        self.vyaw = msg.angular_velocity.z

    def cmd_vel_callback(self, msg):
        self.vx = msg.linear.x
        self.last_cmd_time = rospy.Time.now()

    def _integrate(self):
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        self.last_time = now
        if dt > 0 and dt < 0.1:
            self.x += self.vx * math.cos(self.yaw) * dt
            self.y += self.vx * math.sin(self.yaw) * dt
            self.yaw += self.vyaw * dt

    def publish_pose(self, event):
        self._integrate()
        now = rospy.Time.now()

        # odom -> base TF
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.yaw / 2)
        t.transform.rotation.w = math.cos(self.yaw / 2)
        self.tf_broadcaster.sendTransform(t)

        # 带协方差位姿
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = now
        pose.header.frame_id = self.map_frame
        pose.pose.pose.position.x = self.x
        pose.pose.pose.position.y = self.y
        pose.pose.pose.position.z = 0.0
        pose.pose.pose.orientation = t.transform.rotation

        cov = np.zeros(36)
        cov[0] = 0.05   # x
        cov[7] = 0.05   # y
        cov[14] = 0.1   # z
        cov[21] = 0.01  # rx
        cov[28] = 0.01  # ry
        cov[35] = 0.1   # rz
        pose.pose.covariance = cov.tolist()

        self.pose_pub.publish(pose)

    def publish_status(self, event):
        """发布定位状态"""
        status = LocalizationStatus()
        status.header.stamp = rospy.Time.now()
        status.header.frame_id = self.map_frame
        status.tracking_state = LocalizationStatus.STATE_TRACKING
        status.pose_covariance_trace = 0.2
        status.drift_warning = False
        status.drift_rate_linear = 0.0
        status.drift_rate_angular = 0.0
        status.pose_jump_detected = False
        status.last_correction_translation = 0.0
        status.last_correction_rotation = 0.0
        status.correction_version = self.correction_version
        status.relocalization_event_id = ""
        status.last_stable_time = self.last_stable_time
        status.status_reason = "skeleton dead reckoning, SLAM TBD"
        self.status_pub.publish(status)

    def publish_map(self, event):
        """发布建图状态和占位地图"""
        now = rospy.Time.now()

        # 建图状态
        map_status = MappingStatus()
        map_status.header.stamp = now
        map_status.ready = True
        map_status.stable = True
        map_status.lost = False
        map_status.current_floor = self.current_floor

        floor_info = FloorMapInfo()
        floor_info.floor_id = self.current_floor
        floor_info.map_version = 1
        floor_info.last_update = now
        map_status.floor_maps = [floor_info]
        map_status.status_reason = "skeleton empty map, SLAM TBD"
        self.mapping_status_pub.publish(map_status)

        # 占位地图（全未知）
        grid = OccupancyGrid()
        grid.header.stamp = now
        grid.header.frame_id = self.map_frame
        res = rospy.get_param("~map_resolution", 0.05)
        width = rospy.get_param("~map_width", 800)
        height = rospy.get_param("~map_height", 800)
        grid.info.resolution = res
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = rospy.get_param("~map_origin_x", -20.0)
        grid.info.origin.position.y = rospy.get_param("~map_origin_y", -20.0)
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [-1] * (width * height)
        self.map_pub.publish(grid)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = PoseEstimator()
        node.run()
    except rospy.ROSInterruptException:
        pass
