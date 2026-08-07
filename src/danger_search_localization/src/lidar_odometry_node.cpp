#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
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
    private_nh_.param<std::string>("gicp_pose_topic", output_topic_,
                                   "/localization/raw_pose");
    private_nh_.param<std::string>("map_frame", map_frame_, "map");
    private_nh_.param<std::string>("base_frame", base_frame_, "base");
    private_nh_.param("lidar_odom_voxel_size_m", voxel_size_, 0.15);
    private_nh_.param("lidar_odom_min_range_m", min_range_, 0.40);
    private_nh_.param("lidar_odom_max_range_m", max_range_, 12.0);
    private_nh_.param("lidar_odom_max_correspondence_m", max_correspondence_, 0.80);
    private_nh_.param("lidar_odom_max_iterations", max_iterations_, 35);
    private_nh_.param("lidar_odom_max_fitness", max_fitness_, 0.20);
    private_nh_.param("lidar_odom_max_step_translation_m", max_step_translation_, 0.35);
    private_nh_.param("lidar_odom_max_step_rotation_rad", max_step_rotation_, 0.45);
    private_nh_.param("lidar_odom_min_points", min_points_, 300);
    if (voxel_size_ <= 0.0 || min_range_ < 0.0 || max_range_ <= min_range_ ||
        max_correspondence_ <= 0.0 || max_iterations_ < 1 || max_fitness_ <= 0.0 ||
        max_step_translation_ <= 0.0 || max_step_rotation_ <= 0.0 || min_points_ < 50) {
      throw std::invalid_argument("invalid lidar odometry configuration");
    }
    publisher_ = private_nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        output_topic_, 10);
    subscriber_ = private_nh_.subscribe(input_topic_, 2,
                                        &LidarOdometryNode::CloudCallback, this);
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
    if (!previous_) {
      previous_ = current;
      last_stamp_ = message->header.stamp;
      PublishPose(message->header.stamp, 0.01, true);
      return;
    }
    const double dt = (message->header.stamp - last_stamp_).toSec();
    if (dt <= 0.0 || dt > 0.5) {
      ROS_WARN_THROTTLE(1.0, "[localization] invalid lidar scan interval %.3f", dt);
      previous_ = current;
      last_stamp_ = message->header.stamp;
      last_delta_.setIdentity();
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
    registration.align(aligned, last_delta_.matrix().cast<float>());
    Eigen::Isometry3d delta = Eigen::Isometry3d::Identity();
    delta.matrix() = registration.getFinalTransformation().cast<double>();
    const double translation = delta.translation().norm();
    const double angle = std::abs(Eigen::AngleAxisd(delta.rotation()).angle());
    const double fitness = registration.hasConverged()
                               ? registration.getFitnessScore(max_correspondence_)
                               : std::numeric_limits<double>::infinity();
    const bool accepted = registration.hasConverged() && std::isfinite(fitness) &&
                          fitness <= max_fitness_ &&
                          translation <= max_step_translation_ &&
                          angle <= max_step_rotation_;
    if (accepted) {
      world_from_base_ = world_from_base_ * delta;
      last_delta_ = delta;
      previous_ = current;
      last_stamp_ = message->header.stamp;
      consecutive_failures_ = 0;
      PublishPose(message->header.stamp, fitness, true);
    } else {
      ++consecutive_failures_;
      ROS_ERROR_THROTTLE(1.0,
                         "[localization] GICP rejected: converged=%d fitness=%.3f "
                         "translation=%.3f rotation=%.3f failures=%d; rebaselining",
                         registration.hasConverged(), fitness, translation, angle,
                         consecutive_failures_);
      // Do not compare later scans with a stale reference after a rejection.
      previous_ = current;
      last_stamp_ = message->header.stamp;
      last_delta_.setIdentity();
    }
  }

  void PublishPose(const ros::Time& stamp, double fitness, bool healthy) {
    geometry_msgs::PoseWithCovarianceStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = map_frame_;
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
  ros::Subscriber subscriber_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string input_topic_, output_topic_, map_frame_, base_frame_;
  double voxel_size_, min_range_, max_range_, max_correspondence_, max_fitness_;
  double max_step_translation_, max_step_rotation_;
  int max_iterations_, min_points_, consecutive_failures_ = 0;
  Cloud::Ptr previous_;
  ros::Time last_stamp_;
  Eigen::Isometry3d world_from_base_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d last_delta_ = Eigen::Isometry3d::Identity();
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
