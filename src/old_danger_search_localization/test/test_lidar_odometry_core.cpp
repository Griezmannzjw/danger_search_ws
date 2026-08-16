#include <cmath>
#include <random>

#include <Eigen/Geometry>
#include <danger_search_localization/lidar_odometry_core.hpp>
#include <gtest/gtest.h>
#include <pcl/common/transforms.h>

namespace {
using Core = danger_search_localization::LidarOdometryCore;

Core::Cloud::Ptr MakeScene(int points = 350) {
  Core::Cloud::Ptr cloud(new Core::Cloud());
  std::mt19937 generator(7);
  std::uniform_real_distribution<float> x(-3.0F, 3.0F), y(-2.0F, 2.0F), z(-0.7F, 1.4F);
  for (int i = 0; i < points; ++i) {
    const float px = x(generator), py = y(generator);
    cloud->emplace_back(px, py, z(generator) + 0.08F * px * px + 0.04F * py);
  }
  cloud->width = cloud->size(); cloud->height = 1; return cloud;
}

Core::Cloud::Ptr TransformCloud(const Core::Cloud::Ptr& cloud, double x) {
  Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
  transform.translate(Eigen::Vector3f(static_cast<float>(x), 0.0F, 0.0F));
  Core::Cloud::Ptr transformed(new Core::Cloud());
  pcl::transformPointCloud(*cloud, *transformed, transform.matrix()); return transformed;
}

Core::Config TestConfig() {
  Core::Config config; config.voxel_size_m = 0.05F; config.max_correspondence_m = 0.50;
  config.max_iterations = 30; config.min_correspondence_ratio = 0.80; config.max_fitness = 0.05;
  config.max_step_translation_m = 0.25; config.max_step_rotation_rad = 0.30;
  config.max_linear_speed_mps = 2.0; config.max_angular_speed_rps = 3.0;
  config.translation_margin_m = 0.02; config.rotation_margin_rad = 0.02;
  config.translation_deadband_m = 0.0; config.rotation_deadband_rad = 0.0;
  config.min_points = 100; config.rebaseline_after_failures = 3; config.recovery_consecutive_accepts = 2;
  return config;
}

TEST(LidarOdometryCore, IdenticalCloudIsAcceptedWithoutDrift) {
  Core core(TestConfig()); const auto scene = MakeScene(); core.Process(scene, 1.0);
  const auto result = core.Process(scene, 1.1); EXPECT_EQ(result.outcome, Core::Outcome::kAccepted); EXPECT_TRUE(result.healthy); EXPECT_NEAR(result.pose.translation().norm(), 0.0, 1e-3);
}
TEST(LidarOdometryCore, LegalTranslationIsAcceptedContinuously) {
  Core core(TestConfig()); const auto scene = MakeScene(); core.Process(scene, 1.0);
  const auto result = core.Process(TransformCloud(scene, -0.10), 1.1); ASSERT_EQ(result.outcome, Core::Outcome::kAccepted); EXPECT_NEAR(result.pose.translation().x(), 0.10, 0.02);
}
TEST(LidarOdometryCore, MetreScaleMismatchIsRejectedAndPoseIsHeld) {
  Core core(TestConfig()); const auto scene = MakeScene(); core.Process(scene, 1.0); const auto before = core.pose();
  const auto result = core.Process(TransformCloud(scene, -1.0), 1.1); EXPECT_EQ(result.outcome, Core::Outcome::kRejected); EXPECT_TRUE(result.pose.matrix().isApprox(before.matrix(), 1e-9));
}
TEST(LidarOdometryCore, RebuildsOnlyOnThirdConsecutiveRejection) {
  Core core(TestConfig()); const auto scene = MakeScene(), mismatch = TransformCloud(scene, -1.0); core.Process(scene, 1.0);
  EXPECT_EQ(core.Process(mismatch, 1.1).outcome, Core::Outcome::kRejected); EXPECT_EQ(core.Process(mismatch, 1.2).outcome, Core::Outcome::kRejected); EXPECT_EQ(core.Process(mismatch, 1.3).outcome, Core::Outcome::kRebaseline);
}
TEST(LidarOdometryCore, RequiresTwoAcceptsAfterRebaselineToRecover) {
  Core core(TestConfig()); const auto scene = MakeScene(), mismatch = TransformCloud(scene, -1.0); core.Process(scene, 1.0); core.Process(mismatch, 1.1); core.Process(mismatch, 1.2); core.Process(mismatch, 1.3);
  EXPECT_FALSE(core.Process(mismatch, 1.4).healthy); EXPECT_TRUE(core.Process(mismatch, 1.5).healthy);
}
TEST(LidarOdometryCore, RejectedFramesNeverEnterTrustedSubmap) {
  Core core(TestConfig()); const auto scene = MakeScene(); core.Process(scene, 1.0); const auto trusted = core.history_size(); core.Process(TransformCloud(scene, -1.0), 1.1); core.Process(TransformCloud(scene, -1.0), 1.2); EXPECT_EQ(core.history_size(), trusted);
}
TEST(LidarOdometryCore, InvalidTimestampAndPointCountDoNotPolluteState) {
  Core core(TestConfig()); const auto scene = MakeScene(); core.Process(scene, 1.0); Core::Cloud::Ptr sparse(new Core::Cloud()); sparse->emplace_back(0,0,0);
  EXPECT_EQ(core.Process(scene, 1.0).outcome, Core::Outcome::kInvalidTimestamp); EXPECT_EQ(core.Process(sparse, 1.1).outcome, Core::Outcome::kInvalidInput); EXPECT_EQ(core.consecutive_failures(), 0);
}
TEST(LidarOdometryCore, SubmapNeverExceedsConfiguredPointLimit) {
  auto config = TestConfig(); config.submap_max_points = 150; Core core(config); const auto scene = MakeScene(); core.Process(scene, 1.0);
  EXPECT_LE(core.BuildSubmapForTest()->size(), 150U); EXPECT_LE(core.history_size(), static_cast<std::size_t>(config.submap_scans));
}
TEST(LidarOdometryCore, RegistrationCloudNeverExceedsConfiguredPointLimit) {
  auto config = TestConfig();
  config.registration_max_points = 150;
  Core core(config);

  const auto limited = core.LimitRegistrationCloudForTest(MakeScene(500));

  ASSERT_TRUE(limited);
  EXPECT_EQ(limited->size(), 150U);
}
TEST(LidarOdometryCore, RejectsNonFiniteCloudWithoutPollutingState) {
  Core core(TestConfig()); const auto scene = MakeScene(); core.Process(scene, 1.0); const auto trusted = core.history_size(); auto invalid = MakeScene(); invalid->points[0].x = std::numeric_limits<float>::infinity(); EXPECT_EQ(core.Process(invalid, 1.1).outcome, Core::Outcome::kInvalidInput); EXPECT_EQ(core.history_size(), trusted);
}
}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
