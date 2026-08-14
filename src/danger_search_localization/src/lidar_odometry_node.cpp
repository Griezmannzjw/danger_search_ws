#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>
#include <danger_search_localization/lidar_odometry_core.hpp>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace danger_search_localization {

class LidarOdometryNode {
 public:
  LidarOdometryNode() : private_nh_("~"), tf_listener_(tf_buffer_) {
    private_nh_.param<std::string>("raw_scan_topic", input_topic_, "/scan");
    private_nh_.param<std::string>("gicp_pose_topic", output_topic_, "/localization/raw_pose");
    private_nh_.param<std::string>("odom_frame", odom_frame_, "odom");
    private_nh_.param<std::string>("base_frame", base_frame_, "base");
    private_nh_.param("lidar_odom_voxel_size_m", config_.voxel_size_m, 0.25F);
    private_nh_.param("lidar_odom_min_range_m", min_range_, 0.40);
    private_nh_.param("lidar_odom_max_range_m", max_range_, 12.0);
    private_nh_.param("lidar_odom_max_correspondence_m", config_.max_correspondence_m, 0.60);
    private_nh_.param("lidar_odom_max_iterations", config_.max_iterations, 15);
    private_nh_.param("lidar_odom_min_correspondence_ratio", config_.min_correspondence_ratio, 0.35);
    private_nh_.param("lidar_odom_max_reference_age_s", config_.max_reference_age_s, 3.0);
    private_nh_.param("lidar_odom_submap_scans", config_.submap_scans, 5);
    private_nh_.param("lidar_odom_submap_max_points", config_.submap_max_points, 1200);
    private_nh_.param("lidar_odom_registration_max_points", config_.registration_max_points, 240);
    private_nh_.param("lidar_odom_max_fitness", config_.max_fitness, 0.40);
    private_nh_.param("lidar_odom_max_step_translation_m", config_.max_step_translation_m, 0.90);
    private_nh_.param("lidar_odom_max_step_rotation_rad", config_.max_step_rotation_rad, 0.90);
    private_nh_.param("lidar_odom_max_linear_speed_mps", config_.max_linear_speed_mps, 1.0);
    private_nh_.param("lidar_odom_max_angular_speed_rps", config_.max_angular_speed_rps, 2.0);
    private_nh_.param("lidar_odom_translation_margin_m", config_.translation_margin_m, 0.05);
    private_nh_.param("lidar_odom_rotation_margin_rad", config_.rotation_margin_rad, 0.08);
    private_nh_.param("lidar_odom_translation_deadband_m", config_.translation_deadband_m, 0.015);
    private_nh_.param("lidar_odom_rotation_deadband_rad", config_.rotation_deadband_rad, 0.008);
    private_nh_.param("lidar_odom_min_points", config_.min_points, 100);
    private_nh_.param("lidar_odom_rebaseline_after_failures", config_.rebaseline_after_failures, 3);
    private_nh_.param("gicp_recovery_consecutive_accepts", config_.recovery_consecutive_accepts, 2);
    private_nh_.param("lidar_odom_max_absolute_translation_m", config_.max_absolute_translation_m, 100.0);
    if (min_range_ < 0.0 || max_range_ <= min_range_) {
      throw std::invalid_argument("invalid lidar range configuration");
    }
    core_.reset(new LidarOdometryCore(config_));
    publisher_ = private_nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(output_topic_, 10);
    subscriber_ = private_nh_.subscribe(input_topic_, 1, &LidarOdometryNode::CloudCallback, this);
    ROS_INFO("[localization] 3D GICP odometry: %s -> %s", input_topic_.c_str(), output_topic_.c_str());
  }

 private:
  using Cloud = LidarOdometryCore::Cloud;
  using Point = LidarOdometryCore::Point;

  Cloud::Ptr PrepareCloud(const sensor_msgs::PointCloud& message) {
    geometry_msgs::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(base_frame_, message.header.frame_id,
                                             message.header.stamp, ros::Duration(0.10));
    } catch (const tf2::TransformException& exception) {
      ROS_WARN_THROTTLE(1.0, "[localization] lidar extrinsic unavailable: %s", exception.what());
      return Cloud::Ptr();
    }
    const auto& translation = transform.transform.translation;
    const auto& rotation = transform.transform.rotation;
    Eigen::Quaternionf quaternion(rotation.w, rotation.x, rotation.y, rotation.z);
    if (quaternion.norm() < 1e-6F) return Cloud::Ptr();
    quaternion.normalize();
    const Eigen::Vector3f offset(translation.x, translation.y, translation.z);
    Cloud::Ptr unfiltered(new Cloud());
    unfiltered->reserve(message.points.size());
    for (const auto& source : message.points) {
      if (!std::isfinite(source.x) || !std::isfinite(source.y) || !std::isfinite(source.z)) continue;
      const double range = std::sqrt(source.x * source.x + source.y * source.y + source.z * source.z);
      if (range < min_range_ || range > max_range_) continue;
      const Eigen::Vector3f point = quaternion * Eigen::Vector3f(source.x, source.y, source.z) + offset;
      unfiltered->emplace_back(point.x(), point.y(), point.z());
    }
    unfiltered->width = unfiltered->size();
    unfiltered->height = 1;
    Cloud::Ptr filtered(new Cloud());
    pcl::VoxelGrid<Point> voxel_grid;
    voxel_grid.setLeafSize(config_.voxel_size_m, config_.voxel_size_m, config_.voxel_size_m);
    voxel_grid.setInputCloud(unfiltered);
    voxel_grid.filter(*filtered);
    return filtered;
  }

  void CloudCallback(const sensor_msgs::PointCloud::ConstPtr& message) {
    const ros::WallTime started = ros::WallTime::now();
    if (message->header.frame_id.empty()) return;
    const Cloud::Ptr current = PrepareCloud(*message);
    const auto result = core_->Process(current, message->header.stamp.toSec());
    if (result.outcome == LidarOdometryCore::Outcome::kInvalidInput) {
      ROS_WARN_THROTTLE(1.0, "[localization] too few or invalid lidar points: %zu/%d", current ? current->size() : 0U, config_.min_points);
      return;
    }
    if (result.outcome == LidarOdometryCore::Outcome::kInvalidTimestamp) {
      ROS_WARN_THROTTLE(1.0, "[localization] invalid lidar scan timestamp: %s", result.reason.c_str());
      return;
    }
    if (result.outcome == LidarOdometryCore::Outcome::kRejected || result.outcome == LidarOdometryCore::Outcome::kRebaseline) {
      ROS_ERROR_THROTTLE(1.0, "[localization] GICP rejected (%s): converged=%d fitness=%.3f correspondence=%.3f translation=%.3f/%.3f rotation=%.3f/%.3f failures=%d%s", result.reason.c_str(), result.registration.converged, result.registration.fitness, result.registration.correspondence_ratio, result.registration.translation, result.translation_limit, result.registration.rotation, result.rotation_limit, result.consecutive_failures, result.rebuilt_reference ? "; rebuilding reference frame" : "; retaining trusted reference");
    }
    if (result.publish) PublishPose(message->header.stamp, result.pose, result.registration.fitness, result.healthy);
    const double elapsed_ms = (ros::WallTime::now() - started).toSec() * 1000.0;
    if (elapsed_ms > 100.0) ROS_WARN_THROTTLE(1.0, "[localization] GICP callback exceeded budget: %.1fms target_points=%zu", elapsed_ms, result.target_points);
  }

  void PublishPose(const ros::Time& stamp, const Eigen::Isometry3d& pose, double fitness, bool healthy) {
    if (!pose.matrix().allFinite() || pose.translation().norm() > config_.max_absolute_translation_m) {
      ROS_ERROR_THROTTLE(1.0, "[localization] refusing to publish invalid GICP pose");
      return;
    }
    geometry_msgs::PoseWithCovarianceStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = odom_frame_;
    const Eigen::Vector3d translation = pose.translation();
    Eigen::Quaterniond rotation(pose.rotation());
    if (!std::isfinite(rotation.norm()) || rotation.norm() < 1e-6) return;
    rotation.normalize();
    message.pose.pose.position.x = translation.x(); message.pose.pose.position.y = translation.y(); message.pose.pose.position.z = translation.z();
    message.pose.pose.orientation.x = rotation.x(); message.pose.pose.orientation.y = rotation.y(); message.pose.pose.orientation.z = rotation.z(); message.pose.pose.orientation.w = rotation.w();
    const double safe_fitness = std::isfinite(fitness) ? fitness : 10.0;
    const double xy_variance = healthy ? std::max(0.0025, safe_fitness) : 10.0;
    const double yaw_variance = healthy ? std::max(0.005, safe_fitness * 2.0) : 10.0;
    message.pose.covariance[0] = xy_variance; message.pose.covariance[7] = xy_variance;
    message.pose.covariance[14] = healthy ? xy_variance * 2.0 : 10.0;
    message.pose.covariance[21] = healthy ? yaw_variance * 2.0 : 10.0;
    message.pose.covariance[28] = healthy ? yaw_variance * 2.0 : 10.0;
    message.pose.covariance[35] = yaw_variance;
    publisher_.publish(message);
  }

  ros::NodeHandle private_nh_;
  ros::Publisher publisher_;
  ros::Subscriber subscriber_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string input_topic_, output_topic_, odom_frame_, base_frame_;
  double min_range_ = 0.40, max_range_ = 12.0;
  LidarOdometryCore::Config config_;
  std::unique_ptr<LidarOdometryCore> core_;
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
