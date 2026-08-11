#include <algorithm>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

#include <Eigen/Geometry>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace {

class LioOccupancyMapper {
 public:
  LioOccupancyMapper() : private_nh_("~") {
    private_nh_.param("map_resolution", resolution_, 0.05);
    private_nh_.param("map_size", size_, 1024);
    private_nh_.param("map_publish_rate_hz", publish_rate_, 2.0);
    private_nh_.param("map_max_range_m", max_range_, 12.0);
    private_nh_.param("map_obstacle_min_height_m", obstacle_min_height_, -0.12);
    private_nh_.param("map_obstacle_max_height_m", obstacle_max_height_, 1.20);
    private_nh_.param("map_ground_min_height_m", ground_min_height_, -0.55);
    private_nh_.param("map_ground_max_height_m", ground_max_height_, -0.12);
    private_nh_.param("map_voxel_size_m", voxel_size_, 0.12);
    private_nh_.param<std::string>("map_frame", map_frame_, "map");
    log_odds_.assign(static_cast<std::size_t>(size_) * size_, 0);
    publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
    odom_subscriber_ = nh_.subscribe("/localization/lio/odometry", 20,
                                     &LioOccupancyMapper::odomCallback, this);
    cloud_subscriber_ = nh_.subscribe("/localization/lio/cloud_registered", 2,
                                      &LioOccupancyMapper::cloudCallback, this);
    timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                             &LioOccupancyMapper::publish, this);
  }

 private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr& message) {
    std::lock_guard<std::mutex> guard(mutex_);
    const auto& p = message->pose.pose.position;
    const auto& q = message->pose.pose.orientation;
    const Eigen::Quaterniond orientation(q.w, q.x, q.y, q.z);
    if (!initialized_) {
      anchor_position_ = Eigen::Vector3d(p.x, p.y, p.z);
      anchor_inverse_ = orientation.normalized().conjugate();
      initialized_ = true;
    }
    sensor_position_ = anchor_inverse_ * (Eigen::Vector3d(p.x, p.y, p.z) -
                                           anchor_position_);
    last_odom_stamp_ = message->header.stamp;
  }

  bool cell(double x, double y, int& cx, int& cy) const {
    cx = static_cast<int>(std::floor(x / resolution_ + size_ * 0.5));
    cy = static_cast<int>(std::floor(y / resolution_ + size_ * 0.5));
    return cx >= 0 && cy >= 0 && cx < size_ && cy < size_;
  }

  void updateCell(int x, int y, int delta) {
    if (x < 0 || y < 0 || x >= size_ || y >= size_) return;
    auto& value = log_odds_[static_cast<std::size_t>(y) * size_ + x];
    value = static_cast<int8_t>(std::max(-20, std::min(20, static_cast<int>(value) + delta)));
  }

  void trace(int x0, int y0, int x1, int y1, bool occupied) {
    int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int error = dx + dy;
    while (x0 != x1 || y0 != y1) {
      updateCell(x0, y0, -1);
      const int twice = 2 * error;
      if (twice >= dy) { error += dy; x0 += sx; }
      if (twice <= dx) { error += dx; y0 += sy; }
    }
    updateCell(x1, y1, occupied ? 3 : -1);
  }

  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& message) {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*message, cloud);
    std::lock_guard<std::mutex> guard(mutex_);
    if (!initialized_ || cloud.empty()) return;
    int origin_x, origin_y;
    if (!cell(sensor_position_.x(), sensor_position_.y(), origin_x, origin_y)) return;
    std::unordered_set<std::uint64_t> visited;
    const double max_squared = max_range_ * max_range_;
    for (const auto& point : cloud) {
      if (!pcl::isFinite(point)) continue;
      const Eigen::Vector3d mapped = anchor_inverse_ *
          (Eigen::Vector3d(point.x, point.y, point.z) - anchor_position_);
      const Eigen::Vector3d relative = mapped - sensor_position_;
      const double horizontal_squared = relative.x() * relative.x() +
                                        relative.y() * relative.y();
      if (horizontal_squared < 0.16 || horizontal_squared > max_squared) continue;
      const bool obstacle = relative.z() >= obstacle_min_height_ &&
                            relative.z() <= obstacle_max_height_;
      const bool ground = relative.z() >= ground_min_height_ &&
                          relative.z() < ground_max_height_;
      if (!obstacle && !ground) continue;
      const int vx = static_cast<int>(std::floor(mapped.x() / voxel_size_));
      const int vy = static_cast<int>(std::floor(mapped.y() / voxel_size_));
      const std::uint64_t key = (static_cast<std::uint64_t>(static_cast<uint32_t>(vx)) << 32) |
                                static_cast<uint32_t>(vy);
      if (!visited.insert(key).second) continue;
      int endpoint_x, endpoint_y;
      if (cell(mapped.x(), mapped.y(), endpoint_x, endpoint_y))
        trace(origin_x, origin_y, endpoint_x, endpoint_y, obstacle);
    }
    last_cloud_stamp_ = message->header.stamp;
    dirty_ = true;
  }

  void publish(const ros::TimerEvent&) {
    nav_msgs::OccupancyGrid map;
    {
      std::lock_guard<std::mutex> guard(mutex_);
      if (!dirty_) return;
      map.header.stamp = last_cloud_stamp_;
      map.header.frame_id = map_frame_;
      map.info.map_load_time = map.header.stamp;
      map.info.resolution = resolution_;
      map.info.width = map.info.height = size_;
      map.info.origin.position.x = -size_ * resolution_ * 0.5;
      map.info.origin.position.y = -size_ * resolution_ * 0.5;
      map.info.origin.orientation.w = 1.0;
      map.data.resize(log_odds_.size());
      for (std::size_t i = 0; i < log_odds_.size(); ++i) {
        map.data[i] = log_odds_[i] >= 4 ? 100 : (log_odds_[i] <= -2 ? 0 : -1);
      }
      dirty_ = false;
    }
    publisher_.publish(map);
  }

  ros::NodeHandle nh_, private_nh_;
  ros::Subscriber odom_subscriber_, cloud_subscriber_;
  ros::Publisher publisher_;
  ros::Timer timer_;
  std::mutex mutex_;
  std::vector<int8_t> log_odds_;
  bool initialized_ = false, dirty_ = false;
  int size_;
  double resolution_, publish_rate_, max_range_, obstacle_min_height_;
  double obstacle_max_height_, ground_min_height_, ground_max_height_, voxel_size_;
  std::string map_frame_;
  Eigen::Vector3d anchor_position_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond anchor_inverse_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d sensor_position_ = Eigen::Vector3d::Zero();
  ros::Time last_odom_stamp_, last_cloud_stamp_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "lio_occupancy_mapper");
  LioOccupancyMapper mapper;
  ros::spin();
  return 0;
}
