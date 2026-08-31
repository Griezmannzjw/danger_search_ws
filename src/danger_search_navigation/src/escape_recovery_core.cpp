#include <danger_search_navigation/escape_recovery_core.h>

#include <algorithm>
#include <cmath>

#include <base_local_planner/costmap_model.h>
#include <costmap_2d/cost_values.h>

namespace danger_search_navigation
{

void RecoveryAttemptTracker::acceptGoal(
    const std::string& goal_id, const ros::Time& goal_stamp)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_goal_ || goal_id != goal_id_ || goal_stamp != goal_stamp_)
  {
    ++goal_epoch_;
    goal_id_ = goal_id;
    goal_stamp_ = goal_stamp;
    attempts_ = 0;
    canceled_ = false;
    excluded_ = EscapeManeuver::NONE;
    has_goal_ = true;
  }
}

void RecoveryAttemptTracker::cancelGoal(
    const std::string& goal_id, const ros::Time& cancel_before_stamp)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_goal_)
  {
    return;
  }
  if (!goal_id.empty())
  {
    if (goal_id == goal_id_)
    {
      canceled_ = true;
    }
    return;
  }

  // actionlib GoalID semantics for an empty id: a zero stamp cancels all
  // goals, otherwise cancel only goals accepted at or before that stamp.  A
  // zero active goal stamp cannot be ordered safely, so leave it running;
  // an exact-id cancel or the independent safety-stop path remains available.
  if (cancel_before_stamp.isZero()
      || (!goal_stamp_.isZero() && goal_stamp_ <= cancel_before_stamp))
  {
    canceled_ = true;
  }
}

bool RecoveryAttemptTracker::beginAttempt(
    int max_attempts, RecoveryAttemptLease& lease)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_goal_ || canceled_ || max_attempts <= 0 ||
      attempts_ >= max_attempts)
  {
    return false;
  }
  ++attempts_;
  lease.goal_epoch = goal_epoch_;
  lease.goal_id = goal_id_;
  lease.attempt = attempts_;
  lease.excluded = excluded_;
  return true;
}

bool RecoveryAttemptTracker::interrupted(
    const RecoveryAttemptLease& lease, bool safety_stop) const
{
  if (safety_stop)
  {
    return true;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  return !has_goal_ || canceled_ || lease.goal_epoch != goal_epoch_ ||
         lease.goal_id != goal_id_;
}

void RecoveryAttemptTracker::finishAttempt(
    const RecoveryAttemptLease& lease, EscapeManeuver maneuver)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (has_goal_ && !canceled_ && lease.goal_epoch == goal_epoch_ &&
      lease.goal_id == goal_id_)
  {
    excluded_ = maneuver;
  }
}

Pose2D EscapeRecoveryCore::poseAt(
    const Pose2D& start, EscapeManeuver maneuver, double distance,
    double arc_curvature)
{
  double robot_x = 0.0;
  double robot_y = 0.0;
  double yaw_delta = 0.0;
  switch (maneuver)
  {
    case EscapeManeuver::BACKUP:
      robot_x = -distance;
      break;
    case EscapeManeuver::STRAFE_LEFT:
      robot_y = distance;
      break;
    case EscapeManeuver::STRAFE_RIGHT:
      robot_y = -distance;
      break;
    case EscapeManeuver::ARC_LEFT:
    case EscapeManeuver::ARC_RIGHT:
    {
      const double magnitude =
          std::max(std::abs(arc_curvature), 1e-6);
      const double curvature =
          maneuver == EscapeManeuver::ARC_LEFT ? magnitude : -magnitude;
      yaw_delta = curvature * distance;
      robot_x = std::sin(yaw_delta) / curvature;
      robot_y = (1.0 - std::cos(yaw_delta)) / curvature;
      break;
    }
    case EscapeManeuver::NONE:
      break;
  }
  Pose2D pose = start;
  const double cosine = std::cos(start.yaw);
  const double sine = std::sin(start.yaw);
  pose.x += cosine * robot_x - sine * robot_y;
  pose.y += sine * robot_x + cosine * robot_y;
  pose.yaw += yaw_delta;
  return pose;
}

double EscapeRecoveryCore::progressAlong(
    const Pose2D& start, const Pose2D& current,
    EscapeManeuver maneuver, double arc_curvature)
{
  if (maneuver == EscapeManeuver::ARC_LEFT ||
      maneuver == EscapeManeuver::ARC_RIGHT)
  {
    const double translation = std::hypot(
        current.x - start.x, current.y - start.y);
    double yaw_delta = std::atan2(
        std::sin(current.yaw - start.yaw),
        std::cos(current.yaw - start.yaw));
    if (maneuver == EscapeManeuver::ARC_RIGHT)
    {
      yaw_delta = -yaw_delta;
    }
    const double yaw_progress = std::max(
        0.0, yaw_delta / std::max(std::abs(arc_curvature), 1e-6));
    return std::max(translation, yaw_progress);
  }
  const Pose2D unit_target = poseAt(start, maneuver, 1.0, arc_curvature);
  const double direction_x = unit_target.x - start.x;
  const double direction_y = unit_target.y - start.y;
  const double delta_x = current.x - start.x;
  const double delta_y = current.y - start.y;
  return std::max(0.0, delta_x * direction_x + delta_y * direction_y);
}

SweepResult EscapeRecoveryCore::evaluateSweep(
    const costmap_2d::Costmap2D& costmap,
    const std::vector<geometry_msgs::Point>& footprint,
    const Pose2D& start, EscapeManeuver maneuver,
    double distance, double step, double arc_curvature)
{
  SweepResult result;
  if (maneuver == EscapeManeuver::NONE || footprint.size() < 3 ||
      !std::isfinite(distance) || !std::isfinite(step) ||
      distance <= 0.0 || step <= 0.0)
  {
    return result;
  }

  base_local_planner::CostmapModel world(costmap);
  const int samples = std::max(1, static_cast<int>(std::ceil(distance / step)));
  double worst_cost = 0.0;
  for (int index = 0; index <= samples; ++index)
  {
    const double traveled = distance * static_cast<double>(index) /
                            static_cast<double>(samples);
    const Pose2D pose = poseAt(
        start, maneuver, traveled, arc_curvature);
    unsigned int map_x = 0;
    unsigned int map_y = 0;
    if (!costmap.worldToMap(pose.x, pose.y, map_x, map_y))
    {
      return result;
    }
    const unsigned char center_cost = costmap.getCost(map_x, map_y);
    if (center_cost == costmap_2d::NO_INFORMATION ||
        center_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    {
      return result;
    }
    const double footprint_cost = world.footprintCost(
        pose.x, pose.y, pose.yaw, footprint, 0.0, 0.0);
    if (footprint_cost < 0.0)
    {
      return result;
    }
    worst_cost = std::max(
        worst_cost,
        std::max(static_cast<double>(center_cost), footprint_cost));
  }
  result.safe = true;
  result.worst_cost = worst_cost;
  return result;
}

EscapeManeuver EscapeRecoveryCore::selectManeuver(
    const SweepResult& backup,
    const SweepResult& arc_left,
    const SweepResult& arc_right,
    const SweepResult& left,
    const SweepResult& right,
    EscapeManeuver excluded)
{
  const bool arc_left_safe =
      arc_left.safe && excluded != EscapeManeuver::ARC_LEFT;
  const bool arc_right_safe =
      arc_right.safe && excluded != EscapeManeuver::ARC_RIGHT;
  if (arc_left_safe && arc_right_safe)
  {
    return arc_left.worst_cost <= arc_right.worst_cost
               ? EscapeManeuver::ARC_LEFT
               : EscapeManeuver::ARC_RIGHT;
  }
  if (arc_left_safe)
  {
    return EscapeManeuver::ARC_LEFT;
  }
  if (arc_right_safe)
  {
    return EscapeManeuver::ARC_RIGHT;
  }
  if (backup.safe && excluded != EscapeManeuver::BACKUP)
  {
    return EscapeManeuver::BACKUP;
  }
  const bool left_safe = left.safe && excluded != EscapeManeuver::STRAFE_LEFT;
  const bool right_safe = right.safe && excluded != EscapeManeuver::STRAFE_RIGHT;
  if (left_safe && right_safe)
  {
    return left.worst_cost <= right.worst_cost
               ? EscapeManeuver::STRAFE_LEFT
               : EscapeManeuver::STRAFE_RIGHT;
  }
  if (left_safe)
  {
    return EscapeManeuver::STRAFE_LEFT;
  }
  if (right_safe)
  {
    return EscapeManeuver::STRAFE_RIGHT;
  }
  return EscapeManeuver::NONE;
}

const char* EscapeRecoveryCore::name(EscapeManeuver maneuver)
{
  switch (maneuver)
  {
    case EscapeManeuver::BACKUP:
      return "BACKUP";
    case EscapeManeuver::STRAFE_LEFT:
      return "STRAFE_LEFT";
    case EscapeManeuver::STRAFE_RIGHT:
      return "STRAFE_RIGHT";
    case EscapeManeuver::ARC_LEFT:
      return "ARC_LEFT";
    case EscapeManeuver::ARC_RIGHT:
      return "ARC_RIGHT";
    case EscapeManeuver::NONE:
    default:
      return "NONE";
  }
}

}  // namespace danger_search_navigation
