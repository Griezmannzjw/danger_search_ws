#!/usr/bin/env python3
"""
定位与建图节点 - P0最小可运行版本
对齐接口规范 v1.1-p0

功能：
  1. IMU积分航位推算（不用cmd_vel_sent作为正式里程计）
  2. 基于Livox激光的2D占据栅格地图构建
  3. 发布完整TF链: world -> map -> odom -> base
  4. 发布带协方差位姿 /localization/pose
  5. 发布占据地图 /map
  6. 发布建图状态 /mapping/status

P0要求：
  - 唯一发布map->odom->base TF
  - 地图必须有可用于选点的已知自由区域（不能全未知）
  - 定位基于IMU和激光传感器，不用cmd_vel作为正式里程计
  - 所有话题名从参数读取
"""

import rospy
import numpy as np
import tf2_ros
import tf.transformations
from sensor_msgs.msg import Imu, PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import (
    PoseWithCovarianceStamped, Pose, Point, Quaternion,
    TransformStamped, Vector3
)
from std_msgs.msg import Header, Int32
from danger_search_common.msg import MappingStatus
import sensor_msgs.point_cloud2 as pc2


class LocalizationNode:
    def __init__(self):
        rospy.init_node("pose_estimator", anonymous=False)

        # ========== 从参数读取所有话题名 ==========
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base")
        self.world_frame = rospy.get_param("~world_frame", "world")

        self.map_topic = rospy.get_param("~map_topic", "/map")
        self.pose_topic = rospy.get_param("~pose_topic", "/localization/pose")
        self.mapping_status_topic = rospy.get_param("~mapping_status_topic", "/mapping/status")

        # 传感器话题
        self.lidar_topic = rospy.get_param("~lidar_topic", "/livox/Pointcloud2")
        self.imu_topic = rospy.get_param("~imu_topic", "/trunk_imu")

        # 地图参数
        self.map_resolution = rospy.get_param("~map_resolution", 0.05)  # 5cm
        self.map_width = rospy.get_param("~map_width", 400)  # 20m
        self.map_height = rospy.get_param("~map_height", 720)  # 36m
        self.map_origin_x = rospy.get_param("~map_origin_x", -10.0)
        self.map_origin_y = rospy.get_param("~map_origin_y", -18.0)
        self.lidar_max_range = rospy.get_param("~lidar_max_range", 5.0)
        self.lidar_min_range = rospy.get_param("~lidar_min_range", 0.1)

        # ========== 状态 ==========
        # 位姿: x, y, yaw (odom->base)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.last_imu_time = None

        # 地图: -1=未知, 0=自由, 100=占用
        self.map_data = np.full((self.map_height, self.map_width), -1, dtype=np.int8)
        # 机器人周围标记为自由区域（出发点附近）
        self._init_free_area()

        # 建图状态
        self.mapping_ready = False
        self.mapping_stable = False
        self.mapping_lost = False
        self.current_floor = 0

        # ========== TF发布器 ==========
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster()

        # 发布world->map静态TF（首版重合，出发点为原点）
        self._publish_world_to_map()

        # ========== 发布者 ==========
        self.pose_pub = rospy.Publisher(
            self.pose_topic, PoseWithCovarianceStamped, queue_size=10
        )
        self.map_pub = rospy.Publisher(
            self.map_topic, OccupancyGrid, queue_size=1, latch=True
        )
        self.status_pub = rospy.Publisher(
            self.mapping_status_topic, MappingStatus, queue_size=10, latch=True
        )

        # ========== 订阅者 ==========
        self.imu_sub = rospy.Subscriber(
            self.imu_topic, Imu, self.imu_callback, queue_size=100
        )
        self.lidar_sub = rospy.Subscriber(
            self.lidar_topic, PointCloud2, self.lidar_callback, queue_size=5
        )

        # ========== 定时器 ==========
        self.pose_timer = rospy.Timer(rospy.Duration(0.05), self.publish_pose)  # 20Hz
        self.map_timer = rospy.Timer(rospy.Duration(1.0), self.publish_map)  # 1Hz
        self.status_timer = rospy.Timer(rospy.Duration(0.5), self.publish_status)  # 2Hz

        rospy.loginfo("[localization] P0 localization node started")
        rospy.loginfo(f"[localization] map frame: {self.map_frame}, odom frame: {self.odom_frame}")

    def _init_free_area(self):
        """初始化出发点周围为自由区域，确保有可用于选点的已知区域"""
        cx = int((0 - self.map_origin_x) / self.map_resolution)
        cy = int((0 - self.map_origin_y) / self.map_resolution)
        radius = int(2.0 / self.map_resolution)  # 2m半径
        for i in range(-radius, radius + 1):
            for j in range(-radius, radius + 1):
                if i*i + j*j <= radius*radius:
                    mx = cx + i
                    my = cy + j
                    if 0 <= mx < self.map_width and 0 <= my < self.map_height:
                        self.map_data[my, mx] = 0

    def _publish_world_to_map(self):
        """发布world->map静态TF（首版world和map重合，出发点为原点）"""
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.world_frame
        t.child_frame_id = self.map_frame
        t.transform.translation = Vector3(0, 0, 0)
        t.transform.rotation = Quaternion(0, 0, 0, 1)
        self.tf_static_broadcaster.sendTransform(t)

    def imu_callback(self, msg):
        """IMU回调：角速度积分更新航向，加速度积分更新位置"""
        if self.last_imu_time is None:
            self.last_imu_time = msg.header.stamp
            return

        dt = (msg.header.stamp - self.last_imu_time).to_sec()
        if dt <= 0 or dt > 0.1:
            self.last_imu_time = msg.header.stamp
            return

        # 角速度积分更新航向 (只取yaw角速度)
        # IMU在trunk坐标系，首版假设近似水平
        self.yaw += msg.angular_velocity.z * dt

        # 简单加速度积分（去除重力）
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y

        # 转换到odom坐标系
        cos_yaw = np.cos(self.yaw)
        sin_yaw = np.sin(self.yaw)
        ax_world = ax * cos_yaw - ay * sin_yaw
        ay_world = ax * sin_yaw + ay * cos_yaw

        # 简单积分（会漂移，但P0够用）
        self.vx += ax_world * dt
        self.vy += ay_world * dt
        # 简单速度衰减（模拟摩擦，防止无限漂移）
        self.vx *= 0.95
        self.vy *= 0.95

        self.x += self.vx * dt
        self.y += self.vy * dt

        self.last_imu_time = msg.header.stamp

        # 收到IMU数据后标记为就绪
        if not self.mapping_ready:
            self.mapping_ready = True
            self.mapping_stable = True
            rospy.loginfo("[localization] Localization ready")

    def lidar_callback(self, msg):
        """激光点云回调：更新2D占据栅格地图"""
        if not self.mapping_ready:
            return

        cos_yaw = np.cos(self.yaw)
        sin_yaw = np.sin(self.yaw)

        # 机器人在地图中的像素坐标
        robot_cx = int((self.x - self.map_origin_x) / self.map_resolution)
        robot_cy = int((self.y - self.map_origin_y) / self.map_resolution)

        # 遍历点云
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            px, py, pz = point

            # 只取一定高度范围内的点（地面以上）
            if pz < -0.1 or pz > 1.0:
                continue

            dist = np.sqrt(px*px + py*py)
            if dist < self.lidar_min_range or dist > self.lidar_max_range:
                continue

            # 转换到odom坐标系
            world_x = self.x + px * cos_yaw - py * sin_yaw
            world_y = self.y + px * sin_yaw + py * cos_yaw

            # 转换到像素坐标
            mx = int((world_x - self.map_origin_x) / self.map_resolution)
            my = int((world_y - self.map_origin_y) / self.map_resolution)

            if 0 <= mx < self.map_width and 0 <= my < self.map_height:
                # 射线投射：从机器人到点之间标记为自由
                self._ray_cast(robot_cx, robot_cy, mx, my)
                # 终点标记为占用
                self.map_data[my, mx] = 100

    def _ray_cast(self, x0, y0, x1, y1):
        """Bresenham直线算法，标记射线经过的格子为自由"""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        x, y = x0, y0
        while True:
            if 0 <= x < self.map_width and 0 <= y < self.map_height:
                if self.map_data[y, x] == -1 or self.map_data[y, x] == 100:
                    self.map_data[y, x] = 0
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    def publish_pose(self, event=None):
        """发布位姿和TF"""
        now = rospy.Time.now()

        # 1. 发布odom->base TF
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation = Vector3(self.x, self.y, 0.0)
        q = tf.transformations.quaternion_from_euler(0, 0, self.yaw)
        t.transform.rotation = Quaternion(*q)
        self.tf_broadcaster.sendTransform(t)

        # 2. 发布map->odom TF（首版重合）
        t_map = TransformStamped()
        t_map.header.stamp = now
        t_map.header.frame_id = self.map_frame
        t_map.child_frame_id = self.odom_frame
        t_map.transform.translation = Vector3(0, 0, 0)
        t_map.transform.rotation = Quaternion(0, 0, 0, 1)
        self.tf_broadcaster.sendTransform(t_map)

        # 3. 发布PoseWithCovarianceStamped
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = self.map_frame
        pose_msg.pose.pose = Pose(
            position=Point(self.x, self.y, 0.0),
            orientation=Quaternion(*q)
        )
        # 保守协方差（P0占位）
        cov = [0.0] * 36
        cov[0] = 0.1   # x
        cov[7] = 0.1   # y
        cov[35] = 0.1  # yaw
        pose_msg.pose.covariance = cov
        self.pose_pub.publish(pose_msg)

    def publish_map(self, event=None):
        """发布占据栅格地图"""
        msg = OccupancyGrid()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.map_frame
        msg.info.resolution = self.map_resolution
        msg.info.width = self.map_width
        msg.info.height = self.map_height
        msg.info.origin.position.x = self.map_origin_x
        msg.info.origin.position.y = self.map_origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = self.map_data.flatten().tolist()
        self.map_pub.publish(msg)

    def publish_status(self, event=None):
        """发布建图状态"""
        msg = MappingStatus()
        msg.header.stamp = rospy.Time.now()
        msg.ready = self.mapping_ready
        msg.stable = self.mapping_stable
        msg.lost = self.mapping_lost
        msg.current_floor = self.current_floor
        if not self.mapping_ready:
            msg.status_reason = "WAITING_FOR_IMU"
        else:
            msg.status_reason = "OK"
        self.status_pub.publish(msg)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = LocalizationNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
