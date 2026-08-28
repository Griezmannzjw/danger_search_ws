#ifndef DANGER_SEARCH_NAVIGATION_ESCAPE_RECOVERY_CORE_H
#define DANGER_SEARCH_NAVIGATION_ESCAPE_RECOVERY_CORE_H

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>
#include <ros/time.h>

namespace danger_search_navigation
{

enum class EscapeManeuver
{
  NONE = 0,
  BACKUP = 1,
  STRAFE_LEFT = 2,
  STRAFE_RIGHT = 3,
  ARC_LEFT = 4,
  ARC_RIGHT = 5,
};

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct SweepResult
{
  bool safe{false};
  double worst_cost{255.0};
};

struct RecoveryAttemptLease
{
  std::uint64_t goal_epoch{0};
  std::string goal_id;
  int attempt{0};
  EscapeManeuver excluded{EscapeManeuver::NONE};
};

// Thread-safe state shared by both recovery plugin instances.  A lease binds
// one runBehavior invocation to the goal that owned it; old runs cannot mutate
// a newer goal after preemption.
class RecoveryAttemptTracker
{
public:
  // GoalID.id is normally unique, but GoalID.stamp is also part of the
  // actionlib cancellation contract.  Keeping both prevents a late
  // cancel-before-time message from an older goal invalidating a newer epoch.
  void acceptGoal(const std::string& goal_id, const ros::Time& goal_stamp);
  void cancelGoal(
      const std::string& goal_id, const ros::Time& cancel_before_stamp);
  bool beginAttempt(int max_attempts, RecoveryAttemptLease& lease);
  bool interrupted(
      const RecoveryAttemptLease& lease, bool safety_stop) const;
  void finishAttempt(
      const RecoveryAttemptLease& lease, EscapeManeuver maneuver);

private:
  mutable std::mutex mutex_;
  std::uint64_t goal_epoch_{0};
  std::string goal_id_;
  ros::Time goal_stamp_;
  int attempts_{0};
  bool has_goal_{false};
  bool canceled_{false};
  EscapeManeuver excluded_{EscapeManeuver::NONE};
};

class EscapeRecoveryCore
{
public:
  static Pose2D poseAt(
      const Pose2D& start, EscapeManeuver maneuver, double distance,
      double arc_curvature = 1.0);

  static double progressAlong(
      const Pose2D& start, const Pose2D& current,
      EscapeManeuver maneuver, double arc_curvature = 1.0);

  static SweepResult evaluateSweep(
      const costmap_2d::Costmap2D& costmap,
      const std::vector<geometry_msgs::Point>& footprint,
      const Pose2D& start, EscapeManeuver maneuver,
      double distance, double step, double arc_curvature = 1.0);

  static EscapeManeuver selectManeuver(
      const SweepResult& backup,
      const SweepResult& arc_left,
      const SweepResult& arc_right,
      const SweepResult& left,
      const SweepResult& right,
      EscapeManeuver excluded = EscapeManeuver::NONE);

  static const char* name(EscapeManeuver maneuver);
};

}  // namespace danger_search_navigation

#endif  // DANGER_SEARCH_NAVIGATION_ESCAPE_RECOVERY_CORE_H
