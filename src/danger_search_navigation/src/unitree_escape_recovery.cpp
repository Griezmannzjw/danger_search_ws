#include <danger_search_navigation/unitree_escape_recovery.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include <boost/thread/locks.hpp>
#include <costmap_2d/costmap_2d.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>

namespace
{

danger_search_navigation::RecoveryAttemptTracker& sharedState()
{
  static danger_search_navigation::RecoveryAttemptTracker state;
  return state;
}

}  // namespace

namespace danger_search_navigation
{

UnitreeEscapeRecovery::UnitreeEscapeRecovery() = default;

void UnitreeEscapeRecovery::initialize(
    std::string name, tf2_ros::Buffer*,
    costmap_2d::Costmap2DROS*,
    costmap_2d::Costmap2DROS* local_costmap)
{
  if (initialized_)
  {
    ROS_ERROR("[%s] initialize called twice", name_.c_str());
    return;
  }
  name_ = std::move(name);
  local_costmap_ = local_costmap;
  ros::NodeHandle private_node("~/" + name_);
  private_node.param("backup_distance", backup_distance_, backup_distance_);
  private_node.param("backup_speed", backup_speed_, backup_speed_);
  private_node.param("enable_arc", enable_arc_, enable_arc_);
  private_node.param("arc_distance", arc_distance_, arc_distance_);
  private_node.param("arc_linear_speed", arc_linear_speed_, arc_linear_speed_);
  private_node.param("arc_angular_speed", arc_angular_speed_, arc_angular_speed_);
  private_node.param("enable_strafe", enable_strafe_, enable_strafe_);
  private_node.param("strafe_distance", strafe_distance_, strafe_distance_);
  private_node.param("strafe_speed", strafe_speed_, strafe_speed_);
  private_node.param("simulation_step", simulation_step_, simulation_step_);
  private_node.param("frequency", frequency_, frequency_);
  private_node.param(
      "no_progress_timeout", no_progress_timeout_, no_progress_timeout_);
  private_node.param("timeout", timeout_, timeout_);
  private_node.param("progress_epsilon", progress_epsilon_, progress_epsilon_);
  private_node.param(
      "max_attempts_per_goal", max_attempts_per_goal_,
      max_attempts_per_goal_);

  if (local_costmap_ == nullptr || !validConfiguration())
  {
    ROS_ERROR("[%s] invalid Unitree escape recovery configuration", name_.c_str());
    return;
  }

  ros::NodeHandle node;
  velocity_publisher_ = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  goal_subscriber_ = node.subscribe(
      "/move_base/goal", 5, &UnitreeEscapeRecovery::goalCallback, this);
  cancel_subscriber_ = node.subscribe(
      "/move_base/cancel", 5, &UnitreeEscapeRecovery::cancelCallback, this);
  safety_subscriber_ = node.subscribe(
      "/danger_search/safety_stop", 5,
      &UnitreeEscapeRecovery::safetyCallback, this);
  initialized_ = true;
  ROS_INFO(
      "[%s] Unitree escape recovery ready: backup=%.2fm@%.2fm/s "
      "arc=%s %.2fm@%.2fm/s,%.2frad/s strafe=%s %.2fm@%.2fm/s "
      "max_attempts=%d",
      name_.c_str(), backup_distance_, backup_speed_,
      enable_arc_ ? "enabled" : "disabled", arc_distance_,
      arc_linear_speed_, arc_angular_speed_,
      enable_strafe_ ? "enabled" : "disabled",
      strafe_distance_, strafe_speed_, max_attempts_per_goal_);
}

bool UnitreeEscapeRecovery::validConfiguration() const
{
  const double values[] = {
      backup_distance_, -backup_speed_, arc_distance_, arc_linear_speed_,
      arc_angular_speed_, strafe_distance_, strafe_speed_,
      simulation_step_, frequency_, no_progress_timeout_, timeout_,
      progress_epsilon_};
  for (const double value : values)
  {
    if (!std::isfinite(value) || value <= 0.0)
    {
      return false;
    }
  }
  return max_attempts_per_goal_ > 0;
}

void UnitreeEscapeRecovery::goalCallback(
    const move_base_msgs::MoveBaseActionGoal::ConstPtr& message)
{
  sharedState().acceptGoal(message->goal_id.id, message->goal_id.stamp);
}

void UnitreeEscapeRecovery::cancelCallback(
    const actionlib_msgs::GoalID::ConstPtr& message)
{
  sharedState().cancelGoal(message->id, message->stamp);
}

void UnitreeEscapeRecovery::safetyCallback(
    const std_msgs::Bool::ConstPtr& message)
{
  safety_stop_.store(message->data);
}

bool UnitreeEscapeRecovery::interrupted(
    const RecoveryAttemptLease& lease) const
{
  return sharedState().interrupted(lease, safety_stop_.load());
}

bool UnitreeEscapeRecovery::currentPose(Pose2D& pose) const
{
  geometry_msgs::PoseStamped message;
  if (local_costmap_ == nullptr ||
      !local_costmap_->isCurrent() ||
      !local_costmap_->getRobotPose(message))
  {
    return false;
  }
  pose.x = message.pose.position.x;
  pose.y = message.pose.position.y;
  pose.yaw = tf2::getYaw(message.pose.orientation);
  return std::isfinite(pose.x) && std::isfinite(pose.y) &&
         std::isfinite(pose.yaw);
}

void UnitreeEscapeRecovery::publishCommand(EscapeManeuver maneuver)
{
  geometry_msgs::Twist command;
  if (maneuver == EscapeManeuver::BACKUP)
  {
    command.linear.x = backup_speed_;
  }
  else if (maneuver == EscapeManeuver::ARC_LEFT ||
           maneuver == EscapeManeuver::ARC_RIGHT)
  {
    command.linear.x = arc_linear_speed_;
    command.angular.z = maneuver == EscapeManeuver::ARC_LEFT
                            ? arc_angular_speed_
                            : -arc_angular_speed_;
  }
  else if (maneuver == EscapeManeuver::STRAFE_LEFT)
  {
    command.linear.y = strafe_speed_;
  }
  else if (maneuver == EscapeManeuver::STRAFE_RIGHT)
  {
    command.linear.y = -strafe_speed_;
  }
  velocity_publisher_.publish(command);
}

void UnitreeEscapeRecovery::publishZero()
{
  velocity_publisher_.publish(geometry_msgs::Twist());
}

void UnitreeEscapeRecovery::runBehavior()
{
  if (!initialized_)
  {
    ROS_ERROR("[%s] recovery was not initialized", name_.c_str());
    return;
  }
  publishZero();
  if (safety_stop_.load())
  {
    ROS_ERROR("[%s] recovery interrupted before execution", name_.c_str());
    return;
  }

  RecoveryAttemptLease lease;
  if (!sharedState().beginAttempt(max_attempts_per_goal_, lease))
  {
    ROS_ERROR(
        "[%s] no active uncanceled goal or recovery budget exhausted",
        name_.c_str());
    return;
  }
  const EscapeManeuver excluded = lease.excluded;
  const int attempt = lease.attempt;

  Pose2D start;
  if (!currentPose(start))
  {
    ROS_ERROR("[%s] current pose or local costmap is stale", name_.c_str());
    return;
  }

  SweepResult backup;
  SweepResult arc_left;
  SweepResult arc_right;
  SweepResult left;
  SweepResult right;
  {
    costmap_2d::Costmap2D* costmap = local_costmap_->getCostmap();
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(
        *costmap->getMutex());
    const std::vector<geometry_msgs::Point>& footprint =
        local_costmap_->getRobotFootprint();
    backup = EscapeRecoveryCore::evaluateSweep(
        *costmap, footprint, start, EscapeManeuver::BACKUP,
        backup_distance_, simulation_step_);
    if (enable_arc_)
    {
      const double curvature = arc_angular_speed_ / arc_linear_speed_;
      arc_left = EscapeRecoveryCore::evaluateSweep(
          *costmap, footprint, start, EscapeManeuver::ARC_LEFT,
          arc_distance_, simulation_step_, curvature);
      arc_right = EscapeRecoveryCore::evaluateSweep(
          *costmap, footprint, start, EscapeManeuver::ARC_RIGHT,
          arc_distance_, simulation_step_, curvature);
    }
    if (enable_strafe_)
    {
      left = EscapeRecoveryCore::evaluateSweep(
          *costmap, footprint, start, EscapeManeuver::STRAFE_LEFT,
          strafe_distance_, simulation_step_);
      right = EscapeRecoveryCore::evaluateSweep(
          *costmap, footprint, start, EscapeManeuver::STRAFE_RIGHT,
          strafe_distance_, simulation_step_);
    }
  }

  const EscapeManeuver maneuver = EscapeRecoveryCore::selectManeuver(
      backup, arc_left, arc_right, left, right, excluded);
  if (maneuver == EscapeManeuver::NONE)
  {
    ROS_ERROR(
        "[%s] no collision-free recovery sweep (attempt %d)",
        name_.c_str(), attempt);
    return;
  }

  const bool arc = maneuver == EscapeManeuver::ARC_LEFT ||
                   maneuver == EscapeManeuver::ARC_RIGHT;
  const double requested_distance = maneuver == EscapeManeuver::BACKUP
                                        ? backup_distance_
                                        : arc ? arc_distance_ : strafe_distance_;
  const double arc_curvature = arc
                                   ? arc_angular_speed_ / arc_linear_speed_
                                   : 1.0;
  ROS_WARN(
      "[%s] recovery attempt %d: %s %.2fm",
      name_.c_str(), attempt, EscapeRecoveryCore::name(maneuver),
      requested_distance);

  const ros::WallTime begin = ros::WallTime::now();
  ros::WallTime last_progress_time = begin;
  double last_progress = 0.0;
  double achieved = 0.0;
  bool succeeded = false;
  ros::WallRate rate(frequency_);
  while (ros::ok())
  {
    if (interrupted(lease))
    {
      ROS_ERROR("[%s] recovery interrupted", name_.c_str());
      break;
    }
    const ros::WallTime now = ros::WallTime::now();
    if ((now - begin).toSec() > timeout_)
    {
      ROS_ERROR("[%s] recovery timed out", name_.c_str());
      break;
    }

    Pose2D current;
    if (!currentPose(current))
    {
      ROS_ERROR("[%s] pose or local costmap became stale", name_.c_str());
      break;
    }
    achieved = EscapeRecoveryCore::progressAlong(
        start, current, maneuver, arc_curvature);
    if (achieved >= requested_distance)
    {
      succeeded = true;
      break;
    }
    if (achieved >= last_progress + progress_epsilon_)
    {
      last_progress = achieved;
      last_progress_time = now;
    }
    else if ((now - last_progress_time).toSec() > no_progress_timeout_)
    {
      ROS_ERROR("[%s] recovery made no progress for %.2fs",
                name_.c_str(), no_progress_timeout_);
      break;
    }

    const double remaining = requested_distance - achieved;
    SweepResult remaining_sweep;
    {
      costmap_2d::Costmap2D* costmap = local_costmap_->getCostmap();
      boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(
          *costmap->getMutex());
      remaining_sweep = EscapeRecoveryCore::evaluateSweep(
          *costmap, local_costmap_->getRobotFootprint(), current,
          maneuver, remaining, simulation_step_, arc_curvature);
    }
    if (!remaining_sweep.safe)
    {
      ROS_ERROR("[%s] a new obstacle blocks the remaining recovery sweep",
                name_.c_str());
      break;
    }
    publishCommand(maneuver);
    rate.sleep();
  }
  publishZero();

  // Do not issue the same physical maneuver twice for one action goal.  The
  // tracker ignores this completion if a new goal arrived during execution.
  sharedState().finishAttempt(lease, maneuver);
  if (succeeded)
  {
    ROS_INFO("[%s] recovery succeeded: %s achieved %.3fm",
             name_.c_str(), EscapeRecoveryCore::name(maneuver), achieved);
  }
  else
  {
    ROS_ERROR("[%s] recovery failed: %s achieved %.3fm",
              name_.c_str(), EscapeRecoveryCore::name(maneuver), achieved);
  }
}

}  // namespace danger_search_navigation

PLUGINLIB_EXPORT_CLASS(
    danger_search_navigation::UnitreeEscapeRecovery,
    nav_core::RecoveryBehavior)
