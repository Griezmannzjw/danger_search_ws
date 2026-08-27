#ifndef DANGER_SEARCH_NAVIGATION_UNITREE_ESCAPE_RECOVERY_H
#define DANGER_SEARCH_NAVIGATION_UNITREE_ESCAPE_RECOVERY_H

#include <atomic>
#include <string>

#include <actionlib_msgs/GoalID.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <nav_core/recovery_behavior.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <danger_search_navigation/escape_recovery_core.h>

namespace danger_search_navigation
{

class UnitreeEscapeRecovery : public nav_core::RecoveryBehavior
{
public:
  UnitreeEscapeRecovery();
  ~UnitreeEscapeRecovery() override = default;

  void initialize(
      std::string name, tf2_ros::Buffer* tf,
      costmap_2d::Costmap2DROS* global_costmap,
      costmap_2d::Costmap2DROS* local_costmap) override;

  void runBehavior() override;

private:
  bool validConfiguration() const;
  bool interrupted(const RecoveryAttemptLease& lease) const;
  bool currentPose(Pose2D& pose) const;
  void publishCommand(EscapeManeuver maneuver);
  void publishZero();
  void goalCallback(const move_base_msgs::MoveBaseActionGoal::ConstPtr& message);
  void cancelCallback(const actionlib_msgs::GoalID::ConstPtr& message);
  void safetyCallback(const std_msgs::Bool::ConstPtr& message);

  std::string name_;
  costmap_2d::Costmap2DROS* local_costmap_{nullptr};
  ros::Publisher velocity_publisher_;
  ros::Subscriber goal_subscriber_;
  ros::Subscriber cancel_subscriber_;
  ros::Subscriber safety_subscriber_;
  std::atomic<bool> safety_stop_{false};
  bool initialized_{false};

  double backup_distance_{0.35};
  double backup_speed_{-0.30};
  double strafe_distance_{0.30};
  double strafe_speed_{0.20};
  double simulation_step_{0.025};
  double frequency_{20.0};
  double no_progress_timeout_{1.5};
  double timeout_{8.0};
  double progress_epsilon_{0.01};
  int max_attempts_per_goal_{2};
};

}  // namespace danger_search_navigation

#endif  // DANGER_SEARCH_NAVIGATION_UNITREE_ESCAPE_RECOVERY_H
