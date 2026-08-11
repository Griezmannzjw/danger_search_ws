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

Core::Cloud::Ptr TransformCloud2d(const Core::Cloud::Ptr& cloud, double x,
                                  double y, double yaw) {
  Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
  transform.linear() = Eigen::AngleAxisf(static_cast<float>(yaw),
                                         Eigen::Vector3f::UnitZ())
                           .toRotationMatrix();
  transform.translation() =
      Eigen::Vector3f(static_cast<float>(x), static_cast<float>(y), 0.0F);
  Core::Cloud::Ptr transformed(new Core::Cloud());
  pcl::transformPointCloud(*cloud, *transformed, transform.matrix());
  return transformed;
}

Core::Cloud::Ptr ReplaceHeights(const Core::Cloud::Ptr& cloud) {
  Core::Cloud::Ptr changed(new Core::Cloud(*cloud));
  for (std::size_t index = 0; index < changed->size(); ++index) {
    changed->points[index].z =
        static_cast<float>(1.5 * std::sin(static_cast<double>(index)));
  }
  return changed;
}

Core::Config TestConfig() {
  Core::Config config; config.voxel_size_m = 0.05F; config.max_correspondence_m = 0.50;
  config.max_iterations = 30; config.min_correspondence_ratio = 0.80; config.max_fitness = 0.05;
  config.max_step_translation_m = 0.25; config.max_step_rotation_rad = 0.30;
  config.max_linear_speed_mps = 2.0; config.max_angular_speed_rps = 3.0;
  config.translation_margin_m = 0.02; config.rotation_margin_rad = 0.02;
  config.translation_deadband_m = 0.0; config.rotation_deadband_rad = 0.0;
  config.min_points = 100; config.registration_max_points = 120;
  config.rebaseline_after_failures = 3; config.recovery_consecutive_accepts = 2;
  return config;
}

TEST(LidarOdometryCore, IdenticalCloudIsAcceptedWithoutDrift) {
  Core core(TestConfig()); const auto scene = MakeScene(); core.Process(scene, 1.0);
  const auto recovering = core.Process(scene, 1.1);
  EXPECT_EQ(recovering.outcome, Core::Outcome::kAccepted);
  EXPECT_FALSE(recovering.healthy);
  const auto result = core.Process(scene, 1.2); EXPECT_EQ(result.outcome, Core::Outcome::kAccepted); EXPECT_TRUE(result.healthy); EXPECT_NEAR(result.pose.translation().norm(), 0.0, 1e-3);
}
TEST(LidarOdometryCore, LegalTranslationIsAcceptedContinuously) {
  Core core(TestConfig()); const auto scene = MakeScene(); core.Process(scene, 1.0);
  const auto moved = TransformCloud(scene, -0.10);
  const auto result = core.Process(moved, 1.1);
  ASSERT_EQ(result.outcome, Core::Outcome::kAccepted);
  EXPECT_NEAR(result.pose.translation().x(), 0.10, 0.02);
  EXPECT_TRUE(core.Process(moved, 1.2).healthy);
}
TEST(LidarOdometryCore, DifferentVerticalSamplesDoNotInventPlanarMotion) {
  Core core(TestConfig());
  const auto scene = MakeScene();
  core.Process(scene, 1.0, true);
  const auto different_heights = ReplaceHeights(scene);

  const auto recovering = core.Process(different_heights, 1.1, true);
  const auto healthy = core.Process(different_heights, 1.2, true);

  EXPECT_EQ(recovering.outcome, Core::Outcome::kAccepted);
  EXPECT_EQ(healthy.outcome, Core::Outcome::kAccepted);
  EXPECT_TRUE(healthy.healthy);
  EXPECT_LT(healthy.registration.z_translation, TestConfig().max_step_z_m);
  EXPECT_LT(healthy.registration.roll_pitch,
            TestConfig().max_step_roll_pitch_rad);
  EXPECT_LT(healthy.pose.translation().norm(), 0.02);
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
TEST(LidarOdometryCore, SpeedGateTracksElapsedTimeFromTrustedReference) {
  auto config = TestConfig();
  config.max_linear_speed_mps = 0.30;
  config.translation_margin_m = 0.01;
  config.max_gate_dt_s = 0.75;
  Core core(config); const auto scene = MakeScene(); core.Process(scene, 1.0);
  const auto mismatch = TransformCloud(scene, -0.20);
  const auto first = core.Process(mismatch, 1.1);
  const auto second = core.Process(mismatch, 1.5);
  EXPECT_EQ(first.outcome, Core::Outcome::kRejected);
  EXPECT_EQ(second.outcome, Core::Outcome::kRejected);
  EXPECT_NEAR(first.translation_limit, 0.04, 1e-9);
  EXPECT_NEAR(second.translation_limit, 0.16, 1e-9);
}
TEST(LidarOdometryCore, ConstantImuYawDoesNotSuppressTranslation) {
  Core core(TestConfig()); const auto scene = MakeScene();
  core.Process(scene, 1.0, false, true, 0.0);
  const auto moved = TransformCloud(scene, -0.10);
  const auto result = core.Process(moved, 1.1, false, true, 0.0);
  EXPECT_EQ(result.outcome, Core::Outcome::kAccepted);
  EXPECT_NEAR(result.pose.translation().x(), 0.10, 0.02);
}

TEST(LidarOdometryCore, ImuStationarySuppressesPlausibleFalseTranslation) {
  Core core(TestConfig());
  const auto scene = MakeScene();
  core.Process(scene, 1.0, true);

  const auto result = core.Process(TransformCloud(scene, -0.10), 1.1, true);

  EXPECT_EQ(result.outcome, Core::Outcome::kAccepted);
  EXPECT_NEAR(result.pose.translation().norm(), 0.0, 1e-6);
}

TEST(LidarOdometryCore, ImuStationaryConflictBypassesOnlyDynamicGate) {
  auto config = TestConfig();
  config.max_linear_speed_mps = 0.30;
  config.translation_margin_m = 0.01;
  Core core(config);
  const auto scene = MakeScene();
  core.Process(scene, 1.0, true);

  const auto result = core.Process(TransformCloud(scene, -0.20), 1.1, true);

  EXPECT_EQ(result.outcome, Core::Outcome::kAccepted);
  EXPECT_EQ(result.reason, "RECOVERING_IMU_STATIONARY_CONFLICT_HOLD");
  EXPECT_NEAR(result.pose.translation().norm(), 0.0, 1e-6);
}

TEST(LidarOdometryCore, TranslationDeadbandDoesNotAccumulateInternally) {
  auto config = TestConfig();
  config.translation_deadband_m = 0.05;
  Core core(config);
  const auto scene = MakeScene();
  core.Process(scene, 1.0, true);
  const auto small_shift = TransformCloud(scene, -0.02);

  const auto first = core.Process(small_shift, 1.1, true);
  const auto second = core.Process(small_shift, 1.2, true);

  EXPECT_EQ(first.outcome, Core::Outcome::kAccepted);
  EXPECT_EQ(second.outcome, Core::Outcome::kAccepted);
  EXPECT_NEAR(second.pose.translation().norm(), 0.0, 1e-6);
}

TEST(LidarOdometryCore, RecoversLegalMotionAgainstRetainedReference) {
  auto config = TestConfig();
  config.max_linear_speed_mps = 0.30;
  config.translation_margin_m = 0.01;
  config.max_gate_dt_s = 0.75;
  Core core(config);
  const auto scene = MakeScene();
  core.Process(scene, 1.0);
  EXPECT_EQ(core.Process(TransformCloud(scene, -1.0), 1.1).outcome,
            Core::Outcome::kRejected);
  const auto moved = TransformCloud(scene, -0.10);
  const auto accepted = core.Process(moved, 1.4);
  EXPECT_EQ(accepted.outcome, Core::Outcome::kAccepted);
  EXPECT_NEAR(accepted.translation_limit, 0.13, 1e-9);
  EXPECT_NEAR(accepted.pose.translation().x(), 0.10, 0.02);
}

TEST(LidarOdometryCore, ImuHeadingRejectsInconsistentScanYaw) {
  auto config = TestConfig();
  config.imu_yaw_tolerance_rad = 0.05;
  Core core(config);
  const auto scene = MakeScene();
  core.Process(scene, 1.0, false, true, 0.0);

  const auto rotated = TransformCloud2d(scene, 0.0, 0.0, -0.15);
  const auto result = core.Process(rotated, 1.1, false, true, 0.0);

  EXPECT_EQ(result.outcome, Core::Outcome::kRejected);
  EXPECT_EQ(result.reason, "IMU_YAW_CONSISTENCY_LIMIT");
  EXPECT_NEAR(result.pose.translation().norm(), 0.0, 1e-6);
}

TEST(LidarOdometryCore, ImuHeadingConstrainsAcceptedScanYaw) {
  auto config = TestConfig();
  config.imu_yaw_tolerance_rad = 0.08;
  Core core(config);
  const auto scene = MakeScene();
  core.Process(scene, 1.0, false, true, 0.0);

  const auto moved = TransformCloud2d(scene, -0.04, 0.0, -0.10);
  const auto accepted = core.Process(moved, 1.1, false, true, 0.10);

  EXPECT_EQ(accepted.outcome, Core::Outcome::kAccepted);
  EXPECT_NEAR(std::atan2(accepted.pose.rotation()(1, 0),
                         accepted.pose.rotation()(0, 0)),
              0.10, 0.02);
}
}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
