#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace danger_search_localization {

class LidarOdometryNode {
 public:
  LidarOdometryNode() : private_nh_("~"), tf_listener_(tf_buffer_) {
    private_nh_.param<std::string>("raw_scan_topic", input_topic_, "/scan");
    private_nh_.param<std::string>("imu_topic", imu_topic_, "/trunk_imu");
    private_nh_.param<std::string>("gicp_pose_topic", output_topic_,
                                   "/localization/raw_pose");
    private_nh_.param<std::string>("odom_frame", odom_frame_, "odom");
    private_nh_.param<std::string>("base_frame", base_frame_, "base");
    private_nh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/cmd_vel");
    private_nh_.param<std::string>("leg_odom_topic", leg_odom_topic_,
                                   "/unitree/leg_odom");
    private_nh_.param("lidar_odom_voxel_size_m", voxel_size_, 0.15);
    private_nh_.param("lidar_odom_min_range_m", min_range_, 0.40);
    private_nh_.param("lidar_odom_max_range_m", max_range_, 12.0);
    private_nh_.param("lidar_odom_max_correspondence_m", max_correspondence_, 0.80);
    private_nh_.param("lidar_odom_max_iterations", max_iterations_, 35);
    private_nh_.param("lidar_odom_max_fitness", max_fitness_, 0.20);
    private_nh_.param("lidar_odom_max_step_translation_m", max_step_translation_, 0.35);
    private_nh_.param("lidar_odom_max_step_rotation_rad", max_step_rotation_, 0.45);
    private_nh_.param("lidar_odom_max_linear_speed_mps", max_linear_speed_, 1.0);
    private_nh_.param("lidar_odom_max_angular_speed_rps", max_angular_speed_, 2.0);
    private_nh_.param("lidar_odom_translation_margin_m", translation_margin_, 0.05);
    private_nh_.param("lidar_odom_rotation_margin_rad", rotation_margin_, 0.08);
    private_nh_.param("lidar_odom_translation_deadband_m", translation_deadband_, 0.015);
    private_nh_.param("lidar_odom_rotation_deadband_rad", rotation_deadband_, 0.008);
    private_nh_.param("lidar_odom_min_points", min_points_, 300);
    private_nh_.param("stationary_accel_tolerance_mps2", stationary_accel_tolerance_,
                      0.30);
    private_nh_.param("stationary_gyro_threshold_rps", stationary_gyro_threshold_,
                      0.08);
    private_nh_.param("stationary_hold_s", stationary_hold_s_, 1.00);
    private_nh_.param("use_imu_yaw_constraint", use_imu_yaw_constraint_, true);
    private_nh_.param("use_imu_translation_constraint",
                      use_imu_translation_constraint_, true);
    private_nh_.param("imu_translation_min_command_mps",
                      imu_translation_min_command_mps_, 0.30);
    private_nh_.param("imu_translation_bias_learning_rate",
                      imu_translation_bias_learning_rate_, 0.01);
    private_nh_.param("imu_translation_max_acceleration_mps2",
                      imu_translation_max_acceleration_mps2_, 6.0);
    private_nh_.param("imu_translation_max_velocity_mps",
                      imu_translation_max_velocity_mps_, 1.5);
    private_nh_.param("command_translation_weight",
                      command_translation_weight_, 0.75);
    private_nh_.param("use_cmd_vel_motion_constraints",
                      use_cmd_vel_motion_constraints_, true);
    private_nh_.param("cmd_vel_fresh_timeout_s", cmd_vel_fresh_timeout_s_, 0.50);
    private_nh_.param("turn_in_place_linear_threshold_mps",
                      turn_in_place_linear_threshold_mps_, 0.05);
    private_nh_.param("turn_in_place_angular_threshold_rps",
                      turn_in_place_angular_threshold_rps_, 0.05);
    private_nh_.param("use_leg_odom_translation_constraint",
                      use_leg_odom_translation_constraint_, true);
    private_nh_.param("leg_odom_fresh_timeout_s", leg_odom_fresh_timeout_s_, 0.15);
    private_nh_.param("leg_odom_translation_weight", leg_odom_translation_weight_,
                      1.0);
    private_nh_.param("leg_odom_max_step_translation_m",
                      leg_odom_max_step_translation_m_, 0.20);
    if (voxel_size_ <= 0.0 || min_range_ < 0.0 || max_range_ <= min_range_ ||
        max_correspondence_ <= 0.0 || max_iterations_ < 1 || max_fitness_ <= 0.0 ||
        max_step_translation_ <= 0.0 || max_step_rotation_ <= 0.0 ||
        max_linear_speed_ <= 0.0 || max_angular_speed_ <= 0.0 ||
        translation_margin_ < 0.0 || rotation_margin_ < 0.0 ||
        translation_deadband_ < 0.0 || rotation_deadband_ < 0.0 ||
        min_points_ < 50 || stationary_accel_tolerance_ < 0.0 ||
        stationary_gyro_threshold_ < 0.0 || stationary_hold_s_ < 0.0 ||
        imu_translation_min_command_mps_ < 0.0 ||
        imu_translation_bias_learning_rate_ <= 0.0 ||
        imu_translation_bias_learning_rate_ > 1.0 ||
        imu_translation_max_acceleration_mps2_ <= 0.0 ||
        imu_translation_max_velocity_mps_ <= 0.0 ||
        command_translation_weight_ < 0.0 || command_translation_weight_ > 1.0 ||
        cmd_vel_fresh_timeout_s_ < 0.0 ||
        turn_in_place_linear_threshold_mps_ < 0.0 ||
        turn_in_place_angular_threshold_rps_ < 0.0 ||
        leg_odom_fresh_timeout_s_ <= 0.0 ||
        leg_odom_translation_weight_ < 0.0 ||
        leg_odom_translation_weight_ > 1.0 ||
        leg_odom_max_step_translation_m_ <= 0.0) {
      throw std::invalid_argument("invalid lidar odometry configuration");
    }
    publisher_ = private_nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        output_topic_, 10);
    subscriber_ = private_nh_.subscribe(input_topic_, 2,
                                        &LidarOdometryNode::CloudCallback, this);
    imu_subscriber_ = private_nh_.subscribe(imu_topic_, 200,
                                            &LidarOdometryNode::ImuCallback, this);
    cmd_vel_subscriber_ = private_nh_.subscribe(
        cmd_vel_topic_, 10, &LidarOdometryNode::CmdVelCallback, this);
    leg_odom_subscriber_ = private_nh_.subscribe(
        leg_odom_topic_, 100, &LidarOdometryNode::LegOdomCallback, this);
    ROS_INFO("[localization] 3D GICP odometry: %s -> %s", input_topic_.c_str(),
             output_topic_.c_str());
  }

 private:
  using Point = pcl::PointXYZ;
  using Cloud = pcl::PointCloud<Point>;

  Cloud::Ptr PrepareCloud(const sensor_msgs::PointCloud& message) {
    geometry_msgs::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(base_frame_, message.header.frame_id,
                                             message.header.stamp, ros::Duration(0.10));
    } catch (const tf2::TransformException& exception) {
      ROS_WARN_THROTTLE(1.0, "[localization] lidar extrinsic unavailable: %s",
                        exception.what());
      return Cloud::Ptr();
    }
    const auto& translation = transform.transform.translation;
    const auto& rotation = transform.transform.rotation;
    Eigen::Quaternionf quaternion(rotation.w, rotation.x, rotation.y, rotation.z);
    if (quaternion.norm() < 1e-6f) {
      ROS_WARN_THROTTLE(1.0, "[localization] invalid lidar extrinsic quaternion");
      return Cloud::Ptr();
    }
    quaternion.normalize();
    const Eigen::Vector3f offset(translation.x, translation.y, translation.z);
    Cloud::Ptr unfiltered(new Cloud());
    unfiltered->reserve(message.points.size());
    for (const auto& source : message.points) {
      if (!std::isfinite(source.x) || !std::isfinite(source.y) ||
          !std::isfinite(source.z)) {
        continue;
      }
      const double range = std::sqrt(source.x * source.x + source.y * source.y +
                                     source.z * source.z);
      if (range < min_range_ || range > max_range_) {
        continue;
      }
      const Eigen::Vector3f point =
          quaternion * Eigen::Vector3f(source.x, source.y, source.z) + offset;
      unfiltered->emplace_back(point.x(), point.y(), point.z());
    }
    unfiltered->width = unfiltered->size();
    unfiltered->height = 1;
    Cloud::Ptr filtered(new Cloud());
    pcl::VoxelGrid<Point> voxel_grid;
    voxel_grid.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    voxel_grid.setInputCloud(unfiltered);
    voxel_grid.filter(*filtered);
    return filtered;
  }

  void ImuCallback(const sensor_msgs::Imu::ConstPtr& message) {
    latest_imu_ = *message;
    have_imu_ = true;
    const auto& acceleration = message->linear_acceleration;
    const auto& angular_velocity = message->angular_velocity;
    const double acceleration_norm = std::sqrt(
        acceleration.x * acceleration.x + acceleration.y * acceleration.y +
        acceleration.z * acceleration.z);
    const double angular_speed = std::sqrt(
        angular_velocity.x * angular_velocity.x + angular_velocity.y * angular_velocity.y +
        angular_velocity.z * angular_velocity.z);
    imu_stationary_ = std::isfinite(acceleration_norm) && std::isfinite(angular_speed) &&
                      std::abs(acceleration_norm - 9.80665) <= stationary_accel_tolerance_ &&
                      angular_speed <= stationary_gyro_threshold_;
    if (imu_stationary_) {
      if (stationary_since_.isZero()) {
        stationary_since_ = message->header.stamp;
      }
    } else {
      stationary_since_ = ros::Time(0);
    }

    Eigen::Quaterniond orientation(message->orientation.w, message->orientation.x,
                                   message->orientation.y, message->orientation.z);
    if (!use_imu_translation_constraint_ ||
        !std::isfinite(orientation.norm()) || orientation.norm() < 1e-6) {
      return;
    }
    orientation.normalize();
    if (!have_initial_imu_yaw_) {
      double yaw = 0.0;
      if (QuaternionYaw(message->orientation, &yaw)) {
        initial_imu_yaw_ = yaw;
        have_initial_imu_yaw_ = true;
      }
    }
    const Eigen::Vector3d specific_force(acceleration.x, acceleration.y,
                                         acceleration.z);
    Eigen::Vector3d acceleration_world = orientation * specific_force;
    acceleration_world.z() -= 9.80665;
    if (!acceleration_world.allFinite()) {
      return;
    }

    const double command_speed =
        have_cmd_vel_ ? std::hypot(latest_cmd_vel_.linear.x,
                                   latest_cmd_vel_.linear.y)
                      : 0.0;
    const bool command_fresh =
        have_cmd_vel_ &&
        std::abs((message->header.stamp - latest_cmd_vel_stamp_).toSec()) <=
            cmd_vel_fresh_timeout_s_;
    const bool commanded_translation =
        command_fresh && command_speed >= imu_translation_min_command_mps_;

    if (!imu_translation_motion_active_ && commanded_translation) {
      imu_translation_motion_active_ = true;
      imu_translation_motion_start_ = message->header.stamp;
      imu_translation_segment_start_ = imu_translation_position_world_;
      imu_translation_last_stamp_ = message->header.stamp;
    }

    if (imu_translation_motion_active_ && !imu_translation_last_stamp_.isZero()) {
      const double dt =
          (message->header.stamp - imu_translation_last_stamp_).toSec();
      Eigen::Vector3d corrected =
          acceleration_world - imu_translation_acceleration_bias_world_;
      if (corrected.norm() > imu_translation_max_acceleration_mps2_) {
        corrected *= imu_translation_max_acceleration_mps2_ / corrected.norm();
      }
      if (dt > 0.0 && dt < 0.05) {
        imu_translation_position_world_ +=
            imu_translation_velocity_world_ * dt + 0.5 * corrected * dt * dt;
        imu_translation_velocity_world_ += corrected * dt;
        const double horizontal_speed = std::hypot(
            imu_translation_velocity_world_.x(),
            imu_translation_velocity_world_.y());
        if (horizontal_speed > imu_translation_max_velocity_mps_) {
          const double scale =
              imu_translation_max_velocity_mps_ / horizontal_speed;
          imu_translation_velocity_world_.x() *= scale;
          imu_translation_velocity_world_.y() *= scale;
        }
        if (commanded_translation && have_initial_imu_yaw_) {
          double current_yaw = 0.0;
          if (QuaternionYaw(message->orientation, &current_yaw)) {
            const double relative_yaw =
                NormalizeAngle(current_yaw - initial_imu_yaw_);
            const Eigen::Vector2d command_body(latest_cmd_vel_.linear.x,
                                               latest_cmd_vel_.linear.y);
            const Eigen::Rotation2Dd body_to_odom(relative_yaw);
            imu_translation_command_position_odom_ +=
                body_to_odom * command_body * dt;
          }
        }
      }
      imu_translation_last_stamp_ = message->header.stamp;
    }

    const bool settled_after_command =
        imu_translation_motion_active_ && !commanded_translation &&
        imu_stationary_ && !stationary_since_.isZero() &&
        (message->header.stamp - stationary_since_).toSec() >= stationary_hold_s_;
    if (settled_after_command) {
      const double duration = std::max(
          0.0, (message->header.stamp - imu_translation_motion_start_).toSec());
      // Endpoint zero-velocity correction removes the constant acceleration
      // bias accumulated over this motion segment.
      imu_translation_position_world_.x() -=
          0.5 * imu_translation_velocity_world_.x() * duration;
      imu_translation_position_world_.y() -=
          0.5 * imu_translation_velocity_world_.y() * duration;
      imu_translation_velocity_world_.setZero();
      imu_translation_motion_active_ = false;
      imu_translation_last_stamp_ = ros::Time(0);
    }

    if (!imu_translation_motion_active_ && imu_stationary_) {
      imu_translation_acceleration_bias_world_ =
          (1.0 - imu_translation_bias_learning_rate_) *
              imu_translation_acceleration_bias_world_ +
          imu_translation_bias_learning_rate_ * acceleration_world;
      imu_translation_velocity_world_.setZero();
      imu_translation_ready_ = true;
    }
  }

  void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& message) {
    latest_cmd_vel_ = *message;
    latest_cmd_vel_stamp_ = ros::Time::now();
    have_cmd_vel_ = true;
  }

  void LegOdomCallback(const nav_msgs::Odometry::ConstPtr& message) {
    latest_leg_odom_ = *message;
    have_leg_odom_ = true;
  }

  static double NormalizeAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  static bool QuaternionYaw(const geometry_msgs::Quaternion& message,
                            double* yaw) {
    Eigen::Quaterniond quaternion(message.w, message.x, message.y, message.z);
    if (!std::isfinite(quaternion.norm()) || quaternion.norm() < 1e-6) {
      return false;
    }
    quaternion.normalize();
    const double sin_yaw =
        2.0 * (quaternion.w() * quaternion.z() +
               quaternion.x() * quaternion.y());
    const double cos_yaw =
        1.0 - 2.0 * (quaternion.y() * quaternion.y() +
                     quaternion.z() * quaternion.z());
    *yaw = std::atan2(sin_yaw, cos_yaw);
    return std::isfinite(*yaw);
  }

  bool ImuYawAt(const ros::Time& scan_stamp, double* yaw) const {
    return use_imu_yaw_constraint_ && have_imu_ &&
           std::abs((latest_imu_.header.stamp - scan_stamp).toSec()) <= 0.20 &&
           QuaternionYaw(latest_imu_.orientation, yaw);
  }

  void ApplyAbsoluteImuYaw(double imu_yaw) {
    if (!have_initial_imu_yaw_) {
      initial_imu_yaw_ = imu_yaw;
      have_initial_imu_yaw_ = true;
    }
    const double relative_yaw = NormalizeAngle(imu_yaw - initial_imu_yaw_);
    world_from_base_.linear() =
        Eigen::AngleAxisd(relative_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  }

  bool IsCommandedTurnInPlace(const ros::Time& scan_stamp) const {
    if (!use_cmd_vel_motion_constraints_ || !have_cmd_vel_ ||
        std::abs((scan_stamp - latest_cmd_vel_stamp_).toSec()) >
            cmd_vel_fresh_timeout_s_) {
      return false;
    }
    const double linear_speed = std::hypot(latest_cmd_vel_.linear.x,
                                           latest_cmd_vel_.linear.y);
    return linear_speed <= turn_in_place_linear_threshold_mps_ &&
           std::abs(latest_cmd_vel_.angular.z) >=
               turn_in_place_angular_threshold_rps_;
  }

  bool FreshCommand(const ros::Time& scan_stamp) const {
    return use_cmd_vel_motion_constraints_ && have_cmd_vel_ &&
           std::abs((scan_stamp - latest_cmd_vel_stamp_).toSec()) <=
               cmd_vel_fresh_timeout_s_;
  }

  bool LegPoseAt(const ros::Time& scan_stamp, Eigen::Isometry3d* pose) const {
    if (!use_leg_odom_translation_constraint_ || !have_leg_odom_ ||
        std::abs((latest_leg_odom_.header.stamp - scan_stamp).toSec()) >
            leg_odom_fresh_timeout_s_) {
      return false;
    }
    const auto& source = latest_leg_odom_.pose.pose;
    Eigen::Quaterniond orientation(source.orientation.w, source.orientation.x,
                                   source.orientation.y, source.orientation.z);
    if (!std::isfinite(orientation.norm()) || orientation.norm() < 1e-6 ||
        !std::isfinite(source.position.x) || !std::isfinite(source.position.y) ||
        !std::isfinite(source.position.z)) {
      return false;
    }
    orientation.normalize();
    pose->setIdentity();
    pose->linear() = orientation.toRotationMatrix();
    pose->translation() = Eigen::Vector3d(source.position.x, source.position.y,
                                          source.position.z);
    return true;
  }

  void RebaseLegPose(const Eigen::Isometry3d& pose, bool available) {
    if (available) {
      previous_leg_pose_ = pose;
      have_previous_leg_pose_ = true;
    } else {
      have_previous_leg_pose_ = false;
    }
  }

  void ApplyAbsoluteLegTranslation(const Eigen::Isometry3d& pose,
                                   bool available) {
    if (!available) {
      return;
    }
    if (!have_leg_anchor_) {
      leg_anchor_pose_ = pose;
      leg_anchor_world_translation_ = world_from_base_.translation();
      leg_anchor_world_rotation_ = world_from_base_.linear();
      have_leg_anchor_ = true;
    }
    const Eigen::Isometry3d relative = leg_anchor_pose_.inverse() * pose;
    const Eigen::Vector3d translation =
        leg_anchor_world_translation_ +
        leg_anchor_world_rotation_ * relative.translation();
    if (!translation.allFinite()) {
      return;
    }
    world_from_base_.translation().x() = translation.x();
    world_from_base_.translation().y() = translation.y();
    world_from_base_.translation().z() = 0.0;
  }

  void RebaseAbsoluteLegTranslation(const Eigen::Isometry3d& pose,
                                    bool available) {
    if (!available) {
      return;
    }
    leg_anchor_pose_ = pose;
    leg_anchor_world_translation_ = world_from_base_.translation();
    leg_anchor_world_rotation_ = world_from_base_.linear();
    have_leg_anchor_ = true;
  }

  void ApplyImuTranslation() {
    if (!use_imu_translation_constraint_ || !imu_translation_ready_ ||
        !have_initial_imu_yaw_) {
      return;
    }
    const Eigen::Matrix3d initial_world_to_odom =
        Eigen::AngleAxisd(-initial_imu_yaw_, Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
    const Eigen::Vector3d odom_translation =
        initial_world_to_odom * imu_translation_position_world_;
    world_from_base_.translation().x() =
        (1.0 - command_translation_weight_) * odom_translation.x() +
        command_translation_weight_ * imu_translation_command_position_odom_.x();
    world_from_base_.translation().y() =
        (1.0 - command_translation_weight_) * odom_translation.y() +
        command_translation_weight_ * imu_translation_command_position_odom_.y();
    world_from_base_.translation().z() = 0.0;
  }

  void ConstrainTranslation(const ros::Time& scan_stamp,
                            const Eigen::Isometry3d& current_leg_pose,
                            bool have_current_leg_pose,
                            Eigen::Isometry3d* delta) const {
    (void)current_leg_pose;
    if (use_imu_translation_constraint_ && imu_translation_ready_) {
      delta->translation().setZero();
      return;
    }
    // Translation is applied from the absolute leg-odometry pose below.  Do
    // not integrate GICP or foot-step increments on top of that position.
    if (have_current_leg_pose && have_leg_anchor_) {
      delta->translation().setZero();
      return;
    }
    if (IsCommandedTurnInPlace(scan_stamp)) {
      delta->translation().setZero();
      return;
    }

    // For a commanded straight segment, remove the corridor-unobservable
    // lateral GICP component.  The command supplies only the motion subspace,
    // never the displacement magnitude.
    if (FreshCommand(scan_stamp) &&
        std::abs(latest_cmd_vel_.angular.z) <
            turn_in_place_angular_threshold_rps_) {
      Eigen::Vector2d command(latest_cmd_vel_.linear.x,
                              latest_cmd_vel_.linear.y);
      if (command.norm() > turn_in_place_linear_threshold_mps_) {
        command.normalize();
        const Eigen::Vector2d lidar_translation(delta->translation().x(),
                                                delta->translation().y());
        const double along = std::max(0.0, lidar_translation.dot(command));
        delta->translation().x() = along * command.x();
        delta->translation().y() = along * command.y();
      }
    }

    delta->translation().z() = 0.0;
  }

  bool IsStationary(const ros::Time& scan_stamp) const {
    if (!have_imu_ || !imu_stationary_ || stationary_since_.isZero()) {
      return false;
    }
    if (std::abs((latest_imu_.header.stamp - scan_stamp).toSec()) > 0.20) {
      return false;
    }
    return (scan_stamp - stationary_since_).toSec() >= stationary_hold_s_;
  }

  void CloudCallback(const sensor_msgs::PointCloud::ConstPtr& message) {
    if (message->header.frame_id.empty()) {
      ROS_WARN_THROTTLE(1.0, "[localization] lidar frame_id is empty");
      return;
    }
    Cloud::Ptr current = PrepareCloud(*message);
    if (!current || static_cast<int>(current->size()) < min_points_) {
      ROS_WARN_THROTTLE(1.0, "[localization] too few lidar odometry points");
      return;
    }
    double current_imu_yaw = 0.0;
    const bool have_current_imu_yaw =
        ImuYawAt(message->header.stamp, &current_imu_yaw);
    Eigen::Isometry3d current_leg_pose = Eigen::Isometry3d::Identity();
    const bool have_current_leg_pose =
        LegPoseAt(message->header.stamp, &current_leg_pose);
    if (have_current_leg_pose && !have_leg_anchor_) {
      RebaseAbsoluteLegTranslation(current_leg_pose, true);
    }
    if (have_current_imu_yaw && !have_initial_imu_yaw_) {
      initial_imu_yaw_ = current_imu_yaw;
      have_initial_imu_yaw_ = true;
    }

    // A fresh physical IMU stationary indication is stronger evidence than
    // any scan registration result:
    // hold the pose and rebase the trusted cloud instead of integrating that
    // false displacement.  No Gazebo truth pose is read here.
    if (IsStationary(message->header.stamp)) {
      // Commit the final settled foot-kinematics position once when motion
      // ends, then hold it.  This removes gait-cycle excursions without
      // following millimetre-scale static contact jitter forever.
      if (!stationary_mode_active_) {
        ApplyAbsoluteLegTranslation(current_leg_pose, have_current_leg_pose);
      }
      stationary_mode_active_ = true;
      if (have_current_imu_yaw) {
        ApplyAbsoluteImuYaw(current_imu_yaw);
        previous_scan_imu_yaw_ = current_imu_yaw;
        have_previous_scan_imu_yaw_ = true;
      }
      // Zero-velocity update: retain the settled world position while moving
      // the leg-odometry anchor to the current contact solution.  Static foot
      // contact drift therefore cannot appear in the public trajectory.
      RebaseAbsoluteLegTranslation(current_leg_pose, have_current_leg_pose);
      ApplyImuTranslation();
      previous_ = current;
      last_stamp_ = message->header.stamp;
      last_delta_.setIdentity();
      pending_delta_.setIdentity();
      RebaseLegPose(current_leg_pose, have_current_leg_pose);
      consecutive_failures_ = 0;
      ROS_INFO_THROTTLE(2.0,
                        "[localization] IMU stationary: holding lidar pose and rebaselining scan");
      PublishPose(message->header.stamp, 0.0025, true);
      return;
    }
    stationary_mode_active_ = false;
    if (!previous_) {
      if (have_current_imu_yaw) {
        ApplyAbsoluteImuYaw(current_imu_yaw);
        previous_scan_imu_yaw_ = current_imu_yaw;
        have_previous_scan_imu_yaw_ = true;
      }
      previous_ = current;
      RebaseLegPose(current_leg_pose, have_current_leg_pose);
      last_stamp_ = message->header.stamp;
      PublishPose(message->header.stamp, 0.01, true);
      return;
    }
    // The foot-kinematics pose is a continuous absolute translation source.
    // Applying it directly prevents sparse GICP errors from accumulating and
    // also carries motion through short scan-registration failures.
    ApplyImuTranslation();
    const double dt = (message->header.stamp - last_stamp_).toSec();
    if (dt <= 0.0) {
      ROS_WARN_THROTTLE(1.0, "[localization] invalid lidar scan interval %.3f", dt);
      return;
    }
    if (dt > 0.5) {
      ROS_WARN_THROTTLE(
          1.0,
          "[localization] trusted lidar reference is %.3fs old; "
          "rebaselining without changing pose",
          dt);
      previous_ = current;
      if (have_current_imu_yaw) {
        ApplyAbsoluteImuYaw(current_imu_yaw);
        previous_scan_imu_yaw_ = current_imu_yaw;
        have_previous_scan_imu_yaw_ = true;
      }
      last_stamp_ = message->header.stamp;
      last_delta_.setIdentity();
      RebaseLegPose(current_leg_pose, have_current_leg_pose);
      PublishPose(message->header.stamp,
                  std::numeric_limits<double>::infinity(), false);
      return;
    }

    pcl::GeneralizedIterativeClosestPoint<Point, Point> registration;
    registration.setInputSource(current);
    registration.setInputTarget(previous_);
    registration.setMaxCorrespondenceDistance(max_correspondence_);
    registration.setMaximumIterations(max_iterations_);
    registration.setTransformationEpsilon(1e-5);
    registration.setEuclideanFitnessEpsilon(1e-5);
    Cloud aligned;
    Eigen::Isometry3d initial_guess = last_delta_;
    if (have_current_imu_yaw && have_previous_scan_imu_yaw_) {
      const double yaw_delta =
          NormalizeAngle(current_imu_yaw - previous_scan_imu_yaw_);
      initial_guess.linear() =
          Eigen::AngleAxisd(yaw_delta, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    }
    registration.align(aligned, initial_guess.matrix().cast<float>());
    Eigen::Isometry3d delta = Eigen::Isometry3d::Identity();
    delta.matrix() = registration.getFinalTransformation().cast<double>();
    if (have_current_imu_yaw && have_previous_scan_imu_yaw_) {
      const double yaw_delta =
          NormalizeAngle(current_imu_yaw - previous_scan_imu_yaw_);
      delta.linear() =
          Eigen::AngleAxisd(yaw_delta, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    }
    ConstrainTranslation(message->header.stamp, current_leg_pose,
                         have_current_leg_pose, &delta);
    const double translation = delta.translation().norm();
    const double angle = std::abs(Eigen::AngleAxisd(delta.rotation()).angle());
    const double fitness = registration.hasConverged()
                               ? registration.getFitnessScore(max_correspondence_)
                               : std::numeric_limits<double>::infinity();
    const double dynamic_translation_limit =
        std::min(max_step_translation_, translation_margin_ + max_linear_speed_ * dt);
    const double dynamic_rotation_limit =
        std::min(max_step_rotation_, rotation_margin_ + max_angular_speed_ * dt);
    const bool accepted = registration.hasConverged() && std::isfinite(fitness) &&
                          fitness <= max_fitness_ &&
                          translation <= dynamic_translation_limit &&
                          angle <= dynamic_rotation_limit;
    if (accepted) {
      // Accumulate sub-threshold motion instead of discarding it frame by
      // frame.  Stationary jitter tends to cancel, while sustained low-speed
      // motion eventually crosses the deadband and is committed.
      pending_delta_ = pending_delta_ * delta;
      const double pending_translation = pending_delta_.translation().norm();
      const double pending_angle =
          std::abs(Eigen::AngleAxisd(pending_delta_.rotation()).angle());
      if (pending_translation > translation_deadband_ ||
          pending_angle > rotation_deadband_) {
        world_from_base_ = world_from_base_ * pending_delta_;
        pending_delta_.setIdentity();
      }
      if (have_current_imu_yaw) {
        ApplyAbsoluteImuYaw(current_imu_yaw);
        previous_scan_imu_yaw_ = current_imu_yaw;
        have_previous_scan_imu_yaw_ = true;
      }
      last_delta_ = delta;
      previous_ = current;
      RebaseLegPose(current_leg_pose, have_current_leg_pose);
      last_stamp_ = message->header.stamp;
      consecutive_failures_ = 0;
      PublishPose(message->header.stamp, fitness, true);
    } else {
      ++consecutive_failures_;
      ROS_ERROR_THROTTLE(1.0,
                         "[localization] GICP rejected: converged=%d fitness=%.3f "
                         "translation=%.3f/%.3f rotation=%.3f/%.3f failures=%d; "
                         "holding pose and retaining trusted reference",
                         registration.hasConverged(), fitness, translation,
                         dynamic_translation_limit, angle, dynamic_rotation_limit,
                         consecutive_failures_);
      // Keep the last trusted keyframe so a single sparse Livox frame cannot
      // permanently erase the displacement during the rejected interval.
      // The elapsed-time gate above rebaselines if recovery takes too long.
      last_delta_.setIdentity();
      // A fresh held pose prevents one bad sparse Livox frame from causing a
      // timeout.  Large covariance tells the fusion/status layer to degrade
      // after repeated failures instead of treating the hold as real motion.
      PublishPose(message->header.stamp, fitness, false);
    }
  }

  void PublishPose(const ros::Time& stamp, double fitness, bool healthy) {
    geometry_msgs::PoseWithCovarianceStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = odom_frame_;
    const Eigen::Vector3d translation = world_from_base_.translation();
    Eigen::Quaterniond rotation(world_from_base_.rotation());
    rotation.normalize();
    message.pose.pose.position.x = translation.x();
    message.pose.pose.position.y = translation.y();
    message.pose.pose.position.z = translation.z();
    message.pose.pose.orientation.x = rotation.x();
    message.pose.pose.orientation.y = rotation.y();
    message.pose.pose.orientation.z = rotation.z();
    message.pose.pose.orientation.w = rotation.w();
    const double xy_variance = healthy ? std::max(0.0025, fitness) : 10.0;
    const double yaw_variance = healthy ? std::max(0.005, fitness * 2.0) : 10.0;
    message.pose.covariance[0] = xy_variance;
    message.pose.covariance[7] = xy_variance;
    message.pose.covariance[14] = healthy ? xy_variance * 2.0 : 10.0;
    message.pose.covariance[21] = healthy ? yaw_variance * 2.0 : 10.0;
    message.pose.covariance[28] = healthy ? yaw_variance * 2.0 : 10.0;
    message.pose.covariance[35] = yaw_variance;
    publisher_.publish(message);
  }

  ros::NodeHandle private_nh_;
  ros::Publisher publisher_;
  ros::Subscriber subscriber_, imu_subscriber_, cmd_vel_subscriber_;
  ros::Subscriber leg_odom_subscriber_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string input_topic_, imu_topic_, output_topic_, odom_frame_, base_frame_;
  std::string cmd_vel_topic_, leg_odom_topic_;
  double voxel_size_, min_range_, max_range_, max_correspondence_, max_fitness_;
  double max_step_translation_, max_step_rotation_;
  double max_linear_speed_, max_angular_speed_, translation_margin_, rotation_margin_;
  double translation_deadband_, rotation_deadband_;
  int max_iterations_, min_points_, consecutive_failures_ = 0;
  double stationary_accel_tolerance_, stationary_gyro_threshold_, stationary_hold_s_;
  bool use_imu_yaw_constraint_, use_imu_translation_constraint_;
  bool use_cmd_vel_motion_constraints_;
  double imu_translation_min_command_mps_;
  double imu_translation_bias_learning_rate_;
  double imu_translation_max_acceleration_mps2_;
  double imu_translation_max_velocity_mps_;
  double command_translation_weight_;
  double cmd_vel_fresh_timeout_s_, turn_in_place_linear_threshold_mps_;
  double turn_in_place_angular_threshold_rps_;
  bool use_leg_odom_translation_constraint_;
  double leg_odom_fresh_timeout_s_, leg_odom_translation_weight_;
  double leg_odom_max_step_translation_m_;
  sensor_msgs::Imu latest_imu_;
  geometry_msgs::Twist latest_cmd_vel_;
  nav_msgs::Odometry latest_leg_odom_;
  bool have_imu_ = false, imu_stationary_ = false;
  bool have_cmd_vel_ = false;
  bool have_leg_odom_ = false, have_previous_leg_pose_ = false;
  bool have_leg_anchor_ = false;
  bool stationary_mode_active_ = false;
  ros::Time latest_cmd_vel_stamp_;
  bool have_initial_imu_yaw_ = false, have_previous_scan_imu_yaw_ = false;
  bool imu_translation_ready_ = false;
  bool imu_translation_motion_active_ = false;
  double initial_imu_yaw_ = 0.0, previous_scan_imu_yaw_ = 0.0;
  ros::Time stationary_since_;
  ros::Time imu_translation_last_stamp_, imu_translation_motion_start_;
  Cloud::Ptr previous_;
  ros::Time last_stamp_;
  Eigen::Isometry3d world_from_base_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d last_delta_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d pending_delta_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d previous_leg_pose_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d leg_anchor_pose_ = Eigen::Isometry3d::Identity();
  Eigen::Vector3d leg_anchor_world_translation_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d leg_anchor_world_rotation_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d imu_translation_position_world_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d imu_translation_velocity_world_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d imu_translation_acceleration_bias_world_ =
      Eigen::Vector3d::Zero();
  Eigen::Vector3d imu_translation_segment_start_ = Eigen::Vector3d::Zero();
  Eigen::Vector2d imu_translation_command_position_odom_ = Eigen::Vector2d::Zero();
};

}  // namespace danger_search_localization

int main(int argc, char** argv) {
  ros::init(argc, argv, "lidar_odometry");
  try {
    danger_search_localization::LidarOdometryNode node;
    ros::spin();
  } catch (const std::exception& exception) {
    ROS_FATAL("[localization] lidar odometry startup failed: %s", exception.what());
    return 1;
  }
  return 0;
}
