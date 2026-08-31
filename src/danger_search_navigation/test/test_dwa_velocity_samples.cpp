#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <base_local_planner/local_planner_limits.h>
#include <base_local_planner/simple_trajectory_generator.h>
#include <base_local_planner/trajectory.h>

namespace
{

struct VelocitySample
{
  double x;
  double theta;
};

std::vector<VelocitySample> generatedSamples(
    double current_x, double min_x = 0.0, int vx_samples = 2)
{
  base_local_planner::LocalPlannerLimits limits(
      0.30, 0.30,
      0.30, min_x,
      0.0, 0.0,
      0.40, 0.40,
      3.0, 2.0, 8.0, 3.0,
      0.15, 0.80,
      true, 0.05, 0.10);

  base_local_planner::SimpleTrajectoryGenerator generator;
  generator.setParameters(2.0, 0.025, 0.025, true, 0.10);
  generator.initialise(
      Eigen::Vector3f::Zero(),
      Eigen::Vector3f(static_cast<float>(current_x), 0.0F, 0.0F),
      Eigen::Vector3f::Zero(),
      &limits,
      Eigen::Vector3f(
          static_cast<float>(vx_samples), 1.0F, 9.0F));

  std::vector<VelocitySample> samples;
  base_local_planner::Trajectory trajectory;
  while (generator.hasMoreTrajectories())
  {
    if (generator.nextTrajectory(trajectory))
    {
      samples.push_back({trajectory.xv_, trajectory.thetav_});
    }
  }
  return samples;
}

void expectTwoModeDomain(double current_x)
{
  const std::vector<VelocitySample> samples = generatedSamples(current_x);
  ASSERT_EQ(samples.size(), 11U);

  int rotations = 0;
  int forward = 0;
  for (const VelocitySample& sample : samples)
  {
    EXPECT_FALSE(
        std::abs(sample.x) < 1e-6 && std::abs(sample.theta) < 1e-6);
    if (std::abs(sample.x) < 1e-6)
    {
      ++rotations;
      EXPECT_NEAR(std::abs(sample.theta), 0.40, 1e-6);
    }
    else
    {
      ++forward;
      EXPECT_NEAR(sample.x, 0.30, 1e-6);
    }
    EXPECT_FALSE(sample.x > 1e-6 && sample.x < 0.30 - 1e-6);
  }
  EXPECT_EQ(rotations, 2);
  EXPECT_EQ(forward, 9);
}

TEST(DwaVelocitySamplesTest, StationaryWindowContainsOnlyTwoExecutableModes)
{
  expectTwoModeDomain(0.0);
}

TEST(DwaVelocitySamplesTest, IntermediateWindowContainsOnlyTwoExecutableModes)
{
  expectTwoModeDomain(0.15);
}

TEST(DwaVelocitySamplesTest, ForwardWindowContainsOnlyTwoExecutableModes)
{
  expectTwoModeDomain(0.30);
}

TEST(DwaVelocitySamplesTest, PositiveMinimumRemovesInPlaceRotation)
{
  const std::vector<VelocitySample> samples = generatedSamples(0.0, 0.30, 2);
  ASSERT_FALSE(samples.empty());
  for (const VelocitySample& sample : samples)
  {
    EXPECT_GT(sample.x, 0.0);
  }
}

}  // namespace

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
