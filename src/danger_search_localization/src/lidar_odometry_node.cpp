#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <Eigen/Geometry>
#include <danger_search_localization/lidar_odometry_core.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl/filters/voxel_grid.h>
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
    private_nh_.param<std::string>("gicp_pose_topic", output_topic_,
                                   "/localization/raw_pose");
    private_nh_.param<std::string>("odom_frame", odom_frame_, "odom");
    private_nh_.param<std::string>("base_frame", base_frame_, "base");
    private_nh_.param<std::string>("imu_topic", imu_topic_, "/trunk_imu");
    private_nh_.param("enable_imu_leveling", enable_imu_leveling_, true);
    private_nh_.param("imu_fresh_timeout_s", imu_fresh_timeout_s_, 0.20);
    private_nh_.param("gravity_mps2", gravity_mps2_, 9.80665);
    private_nh_.param("lidar_odom_imu_stationary_accel_tolerance_mps2",
                      imu_stationary_accel_tolerance_mps2_, 0.35);
    private_nh_.param("lidar_odom_imu_stationary_gyro_threshold_rps",
                      imu_stationary_gyro_threshold_rps_, 0.12);
    private_nh_.param("lidar_odom_imu_motion_excitation_hold_s",
                      imu_motion_excitation_hold_s_, 0.75);
    private_nh_.param("lidar_odom_observation_scans", observation_scans_, 1);
    private_nh_.param("lidar_odom_observation_max_points",
                      observation_max_points_, 1000);
    private_nh_.param("lidar_odom_voxel_size_m", config_.voxel_size_m, 0.25F);
    private_nh_.param("lidar_odom_min_range_m", min_range_, 0.40);
    private_nh_.param("lidar_odom_max_range_m", max_range_, 12.0);
    private_nh_.param("lidar_odom_max_correspondence_m",
                      config_.max_correspondence_m, 0.60);
    private_nh_.param("lidar_odom_max_iterations", config_.max_iterations, 15);
    private_nh_.param("lidar_odom_min_correspondence_ratio",
                      config_.min_correspondence_ratio, 0.35);
    private_nh_.param("lidar_odom_max_reference_age_s",
                      config_.max_reference_age_s, 3.0);
    private_nh_.param("lidar_odom_max_gate_dt_s", config_.max_gate_dt_s, 0.75);
    private_nh_.param("lidar_odom_submap_scans", config_.submap_scans, 5);
    private_nh_.param("lidar_odom_submap_max_points",
                      config_.submap_max_points, 1200);
    private_nh_.param("lidar_odom_registration_max_points",
                      config_.registration_max_points, 240);
    private_nh_.param("lidar_odom_max_fitness", config_.max_fitness, 0.40);
    private_nh_.param("lidar_odom_max_step_translation_m",
                      config_.max_step_translation_m, 0.90);
    private_nh_.param("lidar_odom_max_step_rotation_rad",
                      config_.max_step_rotation_rad, 0.90);
    private_nh_.param("lidar_odom_max_step_z_m", config_.max_step_z_m, 0.12);
    private_nh_.param("lidar_odom_max_step_roll_pitch_rad",
                      config_.max_step_roll_pitch_rad, 0.20);
    private_nh_.param("lidar_odom_max_linear_speed_mps",
                      config_.max_linear_speed_mps, 0.30);
    private_nh_.param("lidar_odom_max_angular_speed_rps",
                      config_.max_angular_speed_rps, 0.80);
    private_nh_.param("lidar_odom_translation_margin_m",
                      config_.translation_margin_m, 0.04);
    private_nh_.param("lidar_odom_rotation_margin_rad",
                      config_.rotation_margin_rad, 0.06);
    private_nh_.param("lidar_odom_translation_deadband_m",
                      config_.translation_deadband_m, 0.015);
    private_nh_.param("lidar_odom_rotation_deadband_rad",
                      config_.rotation_deadband_rad, 0.008);
    private_nh_.param("lidar_odom_candidate_fitness_slack",
                      config_.candidate_fitness_slack, 0.005);
    private_nh_.param("lidar_odom_imu_yaw_tolerance_rad",
                      config_.imu_yaw_tolerance_rad, 0.20);
    private_nh_.param("lidar_odom_min_points", config_.min_points, 100);
    private_nh_.param("lidar_odom_rebaseline_after_failures",
                      config_.rebaseline_after_failures, 3);
    private_nh_.param("gicp_recovery_consecutive_accepts",
                      config_.recovery_consecutive_accepts, 2);
    private_nh_.param("lidar_odom_max_absolute_translation_m",
                      config_.max_absolute_translation_m, 100.0);

    if (min_range_ < 0.0 || max_range_ <= min_range_ ||
        observation_scans_ < 1 ||
        observation_max_points_ < config_.min_points ||
        imu_fresh_timeout_s_ <= 0.0 || gravity_mps2_ <= 0.0 ||
        imu_stationary_accel_tolerance_mps2_ <= 0.0 ||
        imu_stationary_gyro_threshold_rps_ <= 0.0 ||
        imu_motion_excitation_hold_s_ < 0.0) {
      throw std::invalid_argument("invalid lidar odometry node configuration");
    }

    core_.reset(new LidarOdometryCore(config_));
    publisher_ = private_nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        output_topic_, 10);
    imu_subscriber_ = private_nh_.subscribe(
        imu_topic_, 200, &LidarOdometryNode::ImuCallback, this);
    subscriber_ = private_nh_.subscribe(
        input_topic_, 1, &LidarOdometryNode::CloudCallback, this);
    worker_ = std::thread(&LidarOdometryNode::WorkerLoop, this);
    ROS_INFO(
        "[localization] latest-only SE(2) GICP odometry: %s -> %s "
        "(observation_scans=%d)",
        input_topic_.c_str(), output_topic_.c_str(), observation_scans_);
  }

  ~LidarOdometryNode() {
    {
      std::lock_guard<std::mutex> lock(pending_mutex_);
      stop_worker_ = true;
    }
    pending_condition_.notify_all();
    if (worker_.joinable()) worker_.join();
  }

 private:
  using Cloud = LidarOdometryCore::Cloud;
  using Point = LidarOdometryCore::Point;

  struct ImuSample {
    double stamp_s;
    std::string frame_id;
    Eigen::Quaternionf world_from_imu;
    double acceleration_norm;
    double angular_speed;
  };

  void ImuCallback(const sensor_msgs::Imu::ConstPtr& message) {
    const auto& orientation = message->orientation;
    Eigen::Quaternionf quaternion(orientation.w, orientation.x, orientation.y,
                                  orientation.z);
    if (message->header.frame_id.empty() || !std::isfinite(quaternion.norm()) ||
        quaternion.norm() < 1e-6F) {
      return;
    }
    quaternion.normalize();
    const auto& acceleration = message->linear_acceleration;
    const auto& angular_velocity = message->angular_velocity;
    const double acceleration_norm = std::sqrt(
        acceleration.x * acceleration.x + acceleration.y * acceleration.y +
        acceleration.z * acceleration.z);
    const double angular_speed = std::sqrt(
        angular_velocity.x * angular_velocity.x +
        angular_velocity.y * angular_velocity.y +
        angular_velocity.z * angular_velocity.z);
    if (!std::isfinite(acceleration_norm) || !std::isfinite(angular_speed)) {
      return;
    }
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_samples_.push_back(ImuSample{message->header.stamp.toSec(),
                                     message->header.frame_id, quaternion,
                                     acceleration_norm, angular_speed});
    if (std::abs(acceleration_norm - gravity_mps2_) >
            imu_stationary_accel_tolerance_mps2_ ||
        angular_speed > imu_stationary_gyro_threshold_rps_) {
      last_motion_excitation_stamp_s_ = message->header.stamp.toSec();
    }
    while (imu_samples_.size() > 500U) imu_samples_.pop_front();
  }

  void CloudCallback(const sensor_msgs::PointCloud::ConstPtr& message) {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    pending_message_ = message;
    pending_condition_.notify_one();
  }

  void WorkerLoop() {
    while (ros::ok()) {
      sensor_msgs::PointCloud::ConstPtr message;
      {
        std::unique_lock<std::mutex> lock(pending_mutex_);
        pending_condition_.wait(lock, [this] {
          return stop_worker_ || static_cast<bool>(pending_message_);
        });
        if (stop_worker_) return;
        message = pending_message_;
        pending_message_.reset();
      }
      ProcessMessage(message);
    }
  }

  bool LevelWithImu(const ros::Time& stamp, Cloud* cloud,
                    double* base_heading_rad) {
    if (base_heading_rad) {
      *base_heading_rad = std::numeric_limits<double>::quiet_NaN();
    }
    if (!enable_imu_leveling_) return true;
    ImuSample sample;
    bool found = false;
    {
      std::lock_guard<std::mutex> lock(imu_mutex_);
      double best_age = std::numeric_limits<double>::infinity();
      for (const auto& candidate : imu_samples_) {
        const double age = std::abs(candidate.stamp_s - stamp.toSec());
        if (age < best_age) {
          best_age = age;
          sample = candidate;
          found = true;
        }
      }
      if (!found || best_age > imu_fresh_timeout_s_) found = false;
    }
    if (!found) {
      ROS_WARN_THROTTLE(2.0,
                        "[localization] fresh IMU unavailable for GICP leveling");
      return false;
    }

    geometry_msgs::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(base_frame_, sample.frame_id, stamp,
                                             ros::Duration(0.10));
    } catch (const tf2::TransformException& exception) {
      ROS_WARN_THROTTLE(1.0,
                        "[localization] GICP IMU extrinsic unavailable: %s",
                        exception.what());
      return false;
    }
    const auto& rotation = transform.transform.rotation;
    Eigen::Quaternionf base_from_imu(rotation.w, rotation.x, rotation.y,
                                     rotation.z);
    if (!std::isfinite(base_from_imu.norm()) || base_from_imu.norm() < 1e-6F) {
      return false;
    }
    base_from_imu.normalize();
    Eigen::Quaternionf world_from_base =
        sample.world_from_imu * base_from_imu.conjugate();
    world_from_base.normalize();
    const Eigen::Matrix3f world_rotation = world_from_base.toRotationMatrix();
    const float yaw =
        std::atan2(world_rotation(1, 0), world_rotation(0, 0));
    if (base_heading_rad) *base_heading_rad = static_cast<double>(yaw);
    const Eigen::Matrix3f heading_from_world =
        Eigen::AngleAxisf(-yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    const Eigen::Matrix3f heading_from_base =
        heading_from_world * world_rotation;
    for (auto& point : cloud->points) {
      const Eigen::Vector3f levelled =
          heading_from_base * Eigen::Vector3f(point.x, point.y, point.z);
      point.x = levelled.x();
      point.y = levelled.y();
      point.z = levelled.z();
    }
    return true;
  }

  Cloud::Ptr PrepareCloud(const sensor_msgs::PointCloud& message,
                          double* base_heading_rad) {
    geometry_msgs::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(base_frame_, message.header.frame_id,
                                             message.header.stamp,
                                             ros::Duration(0.10));
    } catch (const tf2::TransformException& exception) {
      ROS_WARN_THROTTLE(1.0,
                        "[localization] lidar extrinsic unavailable: %s",
                        exception.what());
      return Cloud::Ptr();
    }
    const auto& translation = transform.transform.translation;
    const auto& rotation = transform.transform.rotation;
    Eigen::Quaternionf quaternion(rotation.w, rotation.x, rotation.y, rotation.z);
    if (!std::isfinite(quaternion.norm()) || quaternion.norm() < 1e-6F) {
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
      const double range =
          std::sqrt(source.x * source.x + source.y * source.y +
                    source.z * source.z);
      if (range < min_range_ || range > max_range_) continue;
      const Eigen::Vector3f point =
          quaternion * Eigen::Vector3f(source.x, source.y, source.z) + offset;
      unfiltered->emplace_back(point.x(), point.y(), point.z());
    }
    unfiltered->width = unfiltered->size();
    unfiltered->height = 1;
    if (!LevelWithImu(message.header.stamp, unfiltered.get(), base_heading_rad)) {
      return Cloud::Ptr();
    }
    return VoxelFilter(unfiltered);
  }

  bool ImuStationary(const ros::Time& stamp) {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    if (imu_samples_.empty()) return false;
    const auto sample = std::min_element(
        imu_samples_.begin(), imu_samples_.end(),
        [&stamp](const ImuSample& left, const ImuSample& right) {
          return std::abs(left.stamp_s - stamp.toSec()) <
                 std::abs(right.stamp_s - stamp.toSec());
        });
    if (std::abs(sample->stamp_s - stamp.toSec()) > imu_fresh_timeout_s_) {
      return false;
    }
    return stamp.toSec() - last_motion_excitation_stamp_s_ >=
           imu_motion_excitation_hold_s_;
  }

  Cloud::Ptr VoxelFilter(const Cloud::Ptr& input) const {
    Cloud::Ptr filtered(new Cloud());
    pcl::VoxelGrid<Point> voxel_grid;
    voxel_grid.setLeafSize(config_.voxel_size_m, config_.voxel_size_m,
                           config_.voxel_size_m);
    voxel_grid.setInputCloud(input);
    voxel_grid.filter(*filtered);
    return filtered;
  }

  Cloud::Ptr AddObservationFrame(const Cloud::Ptr& current) {
    if (!current || current->empty()) return Cloud::Ptr();
    *observation_cloud_ += *current;
    ++observation_count_;
    if (observation_count_ < observation_scans_) return Cloud::Ptr();
    Cloud::Ptr merged = VoxelFilter(observation_cloud_);
    observation_cloud_.reset(new Cloud());
    observation_count_ = 0;
    if (static_cast<int>(merged->size()) <= observation_max_points_) {
      return merged;
    }
    std::sort(merged->points.begin(), merged->points.end(),
              [](const Point& left, const Point& right) {
                if (left.x != right.x) return left.x < right.x;
                if (left.y != right.y) return left.y < right.y;
                return left.z < right.z;
              });
    Cloud::Ptr limited(new Cloud());
    limited->reserve(observation_max_points_);
    const double stride = static_cast<double>(merged->size()) /
                          static_cast<double>(observation_max_points_);
    for (int index = 0; index < observation_max_points_; ++index) {
      limited->push_back(
          (*merged)[static_cast<std::size_t>(index * stride)]);
    }
    limited->width = limited->size();
    limited->height = 1;
    return limited;
  }

  void ProcessMessage(const sensor_msgs::PointCloud::ConstPtr& message) {
    const ros::WallTime started = ros::WallTime::now();
    if (!message || message->header.frame_id.empty()) return;
    double base_heading_rad = std::numeric_limits<double>::quiet_NaN();
    const Cloud::Ptr current = PrepareCloud(*message, &base_heading_rad);
    const Cloud::Ptr observation = AddObservationFrame(current);
    if (!observation) return;
    const auto result = core_->Process(
        observation, message->header.stamp.toSec(),
        ImuStationary(message->header.stamp),
        std::isfinite(base_heading_rad), base_heading_rad);
    if (result.outcome == LidarOdometryCore::Outcome::kInvalidInput) {
      ROS_WARN_THROTTLE(
          1.0, "[localization] too few or invalid lidar points: %zu/%d",
          observation ? observation->size() : 0U, config_.min_points);
      return;
    }
    if (result.outcome == LidarOdometryCore::Outcome::kInvalidTimestamp) {
      ROS_WARN_THROTTLE(1.0,
                        "[localization] invalid lidar scan timestamp: %s",
                        result.reason.c_str());
      return;
    }
    if (result.outcome == LidarOdometryCore::Outcome::kRejected ||
        result.outcome == LidarOdometryCore::Outcome::kRebaseline) {
      ROS_ERROR_THROTTLE(
          1.0,
          "[localization] GICP rejected (%s): converged=%d fitness=%.3f "
          "correspondence=%.3f translation=%.3f/%.3f rotation=%.3f/%.3f "
          "z=%.3f roll_pitch=%.3f imu_yaw_error=%.3f failures=%d%s",
          result.reason.c_str(), result.registration.converged,
          result.registration.fitness,
          result.registration.correspondence_ratio,
          result.registration.translation, result.translation_limit,
          result.registration.rotation, result.rotation_limit,
          result.registration.z_translation, result.registration.roll_pitch,
          result.registration.imu_yaw_error,
          result.consecutive_failures,
          result.rebuilt_reference ? "; rebuilding reference frame"
                                   : "; retaining trusted reference");
    }
    if (result.publish) {
      PublishPose(message->header.stamp, result.pose,
                  result.registration.fitness, result.healthy);
    }
    const double elapsed_ms =
        (ros::WallTime::now() - started).toSec() * 1000.0;
    if (elapsed_ms > 100.0) {
      ROS_WARN_THROTTLE(
          1.0,
          "[localization] GICP worker exceeded budget: %.1fms target_points=%zu",
          elapsed_ms, result.target_points);
    }
  }

  void PublishPose(const ros::Time& stamp, const Eigen::Isometry3d& pose,
                   double fitness, bool healthy) {
    if (!pose.matrix().allFinite() ||
        pose.translation().head<2>().norm() >
            config_.max_absolute_translation_m) {
      ROS_ERROR_THROTTLE(1.0,
                         "[localization] refusing to publish invalid GICP pose");
      return;
    }
    geometry_msgs::PoseWithCovarianceStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = odom_frame_;
    const Eigen::Vector3d translation = pose.translation();
    Eigen::Quaterniond rotation(pose.rotation());
    if (!std::isfinite(rotation.norm()) || rotation.norm() < 1e-6) return;
    rotation.normalize();
    message.pose.pose.position.x = translation.x();
    message.pose.pose.position.y = translation.y();
    message.pose.pose.position.z = 0.0;
    message.pose.pose.orientation.x = 0.0;
    message.pose.pose.orientation.y = 0.0;
    message.pose.pose.orientation.z = rotation.z();
    message.pose.pose.orientation.w = rotation.w();
    const double safe_fitness = std::isfinite(fitness) ? fitness : 10.0;
    const double xy_variance =
        healthy ? std::max(0.0025, safe_fitness) : 10.0;
    const double yaw_variance =
        healthy ? std::max(0.005, safe_fitness * 2.0) : 10.0;
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
  ros::Subscriber subscriber_;
  ros::Subscriber imu_subscriber_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string input_topic_;
  std::string output_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string imu_topic_;
  double min_range_ = 0.40;
  double max_range_ = 12.0;
  bool enable_imu_leveling_ = true;
  double imu_fresh_timeout_s_ = 0.20;
  double gravity_mps2_ = 9.80665;
  double imu_stationary_accel_tolerance_mps2_ = 0.35;
  double imu_stationary_gyro_threshold_rps_ = 0.12;
  double imu_motion_excitation_hold_s_ = 0.75;
  double last_motion_excitation_stamp_s_ = 0.0;
  int observation_scans_ = 1;
  int observation_max_points_ = 1000;
  int observation_count_ = 0;
  Cloud::Ptr observation_cloud_{new Cloud()};
  LidarOdometryCore::Config config_;
  std::unique_ptr<LidarOdometryCore> core_;

  std::mutex imu_mutex_;
  std::deque<ImuSample> imu_samples_;
  std::mutex pending_mutex_;
  std::condition_variable pending_condition_;
  sensor_msgs::PointCloud::ConstPtr pending_message_;
  bool stop_worker_ = false;
  std::thread worker_;
};

}  // namespace danger_search_localization

int main(int argc, char** argv) {
  ros::init(argc, argv, "lidar_odometry");
  try {
    danger_search_localization::LidarOdometryNode node;
    ros::spin();
  } catch (const std::exception& exception) {
    ROS_FATAL("[localization] lidar odometry startup failed: %s",
              exception.what());
    return 1;
  }
  return 0;
}
