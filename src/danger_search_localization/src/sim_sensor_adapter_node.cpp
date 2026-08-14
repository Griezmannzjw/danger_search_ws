#include <cmath>
#include <string>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace {

class SimSensorAdapter {
 public:
  SimSensorAdapter() : private_nh_("~"), tf_listener_(tf_buffer_) {
    private_nh_.param<std::string>("input_topic", input_topic_, "/scan");
    private_nh_.param<std::string>("output_topic", output_topic_,
                                   "/localization/lio/points");
    private_nh_.param<std::string>("output_frame", output_frame_, "base");
    private_nh_.param("min_range_m", min_range_m_, 0.35);
    private_nh_.param("max_range_m", max_range_m_, 20.0);
    publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 4);
    subscriber_ = nh_.subscribe(input_topic_, 4, &SimSensorAdapter::callback, this);
    ROS_INFO("[localization] SimEnv cloud adapter: %s -> %s (%s)",
             input_topic_.c_str(), output_topic_.c_str(), output_frame_.c_str());
  }

 private:
  void callback(const sensor_msgs::PointCloud::ConstPtr& message) {
    geometry_msgs::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(output_frame_, message->header.frame_id,
                                             ros::Time(0), ros::Duration(0.1));
    } catch (const tf2::TransformException& exception) {
      ROS_WARN_THROTTLE(2.0, "[localization] cloud TF unavailable: %s",
                        exception.what());
      return;
    }

    const auto& t = transform.transform.translation;
    const auto& r = transform.transform.rotation;
    Eigen::Quaternionf rotation(static_cast<float>(r.w), static_cast<float>(r.x),
                                static_cast<float>(r.y), static_cast<float>(r.z));
    if (rotation.norm() < 1e-6F) return;
    rotation.normalize();
    const Eigen::Vector3f translation(static_cast<float>(t.x), static_cast<float>(t.y),
                                      static_cast<float>(t.z));

    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.reserve(message->points.size());
    const float min_squared = static_cast<float>(min_range_m_ * min_range_m_);
    const float max_squared = static_cast<float>(max_range_m_ * max_range_m_);
    for (const auto& input : message->points) {
      const float squared = input.x * input.x + input.y * input.y + input.z * input.z;
      if (!std::isfinite(squared) || squared < min_squared || squared > max_squared) continue;
      const Eigen::Vector3f output = rotation * Eigen::Vector3f(input.x, input.y, input.z) +
                                     translation;
      pcl::PointXYZI point;
      point.x = output.x();
      point.y = output.y();
      point.z = output.z();
      point.intensity = 0.0F;
      cloud.push_back(point);
    }
    cloud.width = cloud.size();
    cloud.height = 1;
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output);
    output.header = message->header;
    output.header.frame_id = output_frame_;
    publisher_.publish(output);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;
  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_;
  double min_range_m_;
  double max_range_m_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "sim_sensor_adapter");
  SimSensorAdapter adapter;
  ros::spin();
  return 0;
}
