#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <costmap_2d/cost_values.h>
#include <danger_search_navigation/escape_recovery_core.h>

namespace
{

using danger_search_navigation::EscapeManeuver;
using danger_search_navigation::EscapeRecoveryCore;
using danger_search_navigation::Pose2D;
using danger_search_navigation::RecoveryAttemptLease;
using danger_search_navigation::RecoveryAttemptTracker;
using danger_search_navigation::SweepResult;

std::vector<geometry_msgs::Point> footprint()
{
  std::vector<geometry_msgs::Point> points(4);
  points[0].x = 0.34;
  points[0].y = 0.19;
  points[1].x = 0.34;
  points[1].y = -0.19;
  points[2].x = -0.39;
  points[2].y = -0.19;
  points[3].x = -0.39;
  points[3].y = 0.19;
  return points;
}

costmap_2d::Costmap2D freeMap()
{
  return costmap_2d::Costmap2D(120, 120, 0.05, -3.0, -3.0, 0);
}

void setWorldCost(
    costmap_2d::Costmap2D& map, double x, double y, unsigned char cost)
{
  unsigned int map_x = 0;
  unsigned int map_y = 0;
  ASSERT_TRUE(map.worldToMap(x, y, map_x, map_y));
  map.setCost(map_x, map_y, cost);
}

TEST(EscapeRecoveryCoreTest, RobotFrameManeuversRespectYaw)
{
  const Pose2D start{1.0, 2.0, M_PI_2};
  const Pose2D backup = EscapeRecoveryCore::poseAt(
      start, EscapeManeuver::BACKUP, 1.0);
  const Pose2D left = EscapeRecoveryCore::poseAt(
      start, EscapeManeuver::STRAFE_LEFT, 1.0);
  EXPECT_NEAR(backup.x, 1.0, 1e-9);
  EXPECT_NEAR(backup.y, 1.0, 1e-9);
  EXPECT_NEAR(left.x, 0.0, 1e-9);
  EXPECT_NEAR(left.y, 2.0, 1e-9);
}

TEST(EscapeRecoveryCoreTest, FreeBackupSweepIsAccepted)
{
  costmap_2d::Costmap2D map = freeMap();
  const SweepResult result = EscapeRecoveryCore::evaluateSweep(
      map, footprint(), Pose2D{}, EscapeManeuver::BACKUP, 0.35, 0.025);
  EXPECT_TRUE(result.safe);
  EXPECT_DOUBLE_EQ(result.worst_cost, 0.0);
}

TEST(EscapeRecoveryCoreTest, InitialFootprintCollisionRejectsMotion)
{
  costmap_2d::Costmap2D map = freeMap();
  setWorldCost(map, 0.0, 0.0, costmap_2d::LETHAL_OBSTACLE);
  const SweepResult result = EscapeRecoveryCore::evaluateSweep(
      map, footprint(), Pose2D{}, EscapeManeuver::BACKUP, 0.35, 0.025);
  EXPECT_FALSE(result.safe);
}

TEST(EscapeRecoveryCoreTest, BlockedBackupSelectsLowerCostStrafe)
{
  costmap_2d::Costmap2D map = freeMap();
  for (double y = -0.35; y <= 0.35; y += 0.05)
  {
    setWorldCost(map, -0.45, y, costmap_2d::LETHAL_OBSTACLE);
  }
  for (double x = -0.30; x <= 0.30; x += 0.05)
  {
    setWorldCost(map, x, 0.42, 120);
  }
  const std::vector<geometry_msgs::Point> shape = footprint();
  const Pose2D start{};
  const SweepResult backup = EscapeRecoveryCore::evaluateSweep(
      map, shape, start, EscapeManeuver::BACKUP, 0.35, 0.025);
  const SweepResult left = EscapeRecoveryCore::evaluateSweep(
      map, shape, start, EscapeManeuver::STRAFE_LEFT, 0.30, 0.025);
  const SweepResult right = EscapeRecoveryCore::evaluateSweep(
      map, shape, start, EscapeManeuver::STRAFE_RIGHT, 0.30, 0.025);
  EXPECT_FALSE(backup.safe);
  EXPECT_TRUE(left.safe);
  EXPECT_TRUE(right.safe);
  EXPECT_EQ(
      EscapeRecoveryCore::selectManeuver(backup, left, right),
      EscapeManeuver::STRAFE_RIGHT);
}

TEST(EscapeRecoveryCoreTest, ExcludedDirectionIsNotRetried)
{
  const SweepResult safe{true, 0.0};
  const SweepResult blocked{false, 255.0};
  EXPECT_EQ(
      EscapeRecoveryCore::selectManeuver(
          safe, blocked, blocked, EscapeManeuver::BACKUP),
      EscapeManeuver::NONE);
}

TEST(EscapeRecoveryCoreTest, ProgressUsesSelectedRobotAxis)
{
  const Pose2D start{0.0, 0.0, 0.0};
  EXPECT_NEAR(
      EscapeRecoveryCore::progressAlong(
          start, Pose2D{-0.31, 0.04, 0.0}, EscapeManeuver::BACKUP),
      0.31, 1e-9);
  EXPECT_DOUBLE_EQ(
      EscapeRecoveryCore::progressAlong(
          start, Pose2D{0.10, 0.0, 0.0}, EscapeManeuver::BACKUP),
      0.0);
}

TEST(RecoveryAttemptTrackerTest, NewGoalInterruptsOldLeaseAndResetsBudget)
{
  RecoveryAttemptTracker tracker;
  tracker.acceptGoal("goal-a");
  RecoveryAttemptLease old_lease;
  ASSERT_TRUE(tracker.beginAttempt(2, old_lease));
  EXPECT_EQ(old_lease.attempt, 1);

  tracker.acceptGoal("goal-b");
  EXPECT_TRUE(tracker.interrupted(old_lease, false));
  tracker.finishAttempt(old_lease, EscapeManeuver::BACKUP);

  RecoveryAttemptLease new_lease;
  ASSERT_TRUE(tracker.beginAttempt(2, new_lease));
  EXPECT_EQ(new_lease.attempt, 1);
  EXPECT_EQ(new_lease.excluded, EscapeManeuver::NONE);
}

TEST(RecoveryAttemptTrackerTest, CancelOnlyInterruptsMatchingGoal)
{
  RecoveryAttemptTracker tracker;
  tracker.acceptGoal("goal-a");
  RecoveryAttemptLease lease;
  ASSERT_TRUE(tracker.beginAttempt(2, lease));
  tracker.cancelGoal("another-goal");
  EXPECT_FALSE(tracker.interrupted(lease, false));
  tracker.cancelGoal("goal-a");
  EXPECT_TRUE(tracker.interrupted(lease, false));
  RecoveryAttemptLease blocked;
  EXPECT_FALSE(tracker.beginAttempt(2, blocked));
}

TEST(RecoveryAttemptTrackerTest, EmptyCancelAndSafetyStopAreImmediate)
{
  RecoveryAttemptTracker tracker;
  tracker.acceptGoal("goal-a");
  RecoveryAttemptLease lease;
  ASSERT_TRUE(tracker.beginAttempt(2, lease));
  EXPECT_TRUE(tracker.interrupted(lease, true));
  EXPECT_FALSE(tracker.interrupted(lease, false));
  tracker.cancelGoal("");
  EXPECT_TRUE(tracker.interrupted(lease, false));
}

TEST(RecoveryAttemptTrackerTest, TwoInstancesShareAttemptAndExclusionState)
{
  RecoveryAttemptTracker tracker;
  tracker.acceptGoal("goal-a");
  RecoveryAttemptLease first;
  ASSERT_TRUE(tracker.beginAttempt(2, first));
  tracker.finishAttempt(first, EscapeManeuver::BACKUP);

  RecoveryAttemptLease second;
  ASSERT_TRUE(tracker.beginAttempt(2, second));
  EXPECT_EQ(second.attempt, 2);
  EXPECT_EQ(second.excluded, EscapeManeuver::BACKUP);
  RecoveryAttemptLease exhausted;
  EXPECT_FALSE(tracker.beginAttempt(2, exhausted));
}

}  // namespace

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
