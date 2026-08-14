#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>

namespace danger_search_localization {

class LidarOdometryCore {
 public:
  using Point = pcl::PointXYZ;
  using Cloud = pcl::PointCloud<Point>;

  struct Config {
    float voxel_size_m = 0.25F;
    double max_correspondence_m = 0.60;
    int max_iterations = 15;
    double min_correspondence_ratio = 0.35;
    double max_reference_age_s = 3.0;
    int submap_scans = 5;
    int submap_max_points = 1200;
    int registration_max_points = 240;
    double max_fitness = 0.40;
    double max_step_translation_m = 0.90;
    double max_step_rotation_rad = 0.90;
    double max_linear_speed_mps = 1.0;
    double max_angular_speed_rps = 2.0;
    double translation_margin_m = 0.05;
    double rotation_margin_rad = 0.08;
    double translation_deadband_m = 0.015;
    double rotation_deadband_rad = 0.008;
    int min_points = 100;
    int rebaseline_after_failures = 3;
    int recovery_consecutive_accepts = 2;
    double max_absolute_translation_m = 100.0;
  };

  struct RegistrationResult {
    bool converged = false;
    bool transform_valid = false;
    bool quality_valid = false;
    double fitness = std::numeric_limits<double>::infinity();
    double translation = 0.0;
    double rotation = 0.0;
    double correspondence_ratio = 0.0;
    Eigen::Isometry3d delta = Eigen::Isometry3d::Identity();
  };

  enum class Outcome { kBootstrap, kAccepted, kRejected, kRebaseline,
                       kInvalidInput, kInvalidTimestamp };

  struct ProcessResult {
    Outcome outcome = Outcome::kInvalidInput;
    bool publish = false;
    bool healthy = false;
    bool rebuilt_reference = false;
    int consecutive_failures = 0;
    int recovery_accepts = 0;
    std::size_t target_points = 0;
    double translation_limit = 0.0;
    double rotation_limit = 0.0;
    RegistrationResult registration;
    std::string reason;
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  };

  explicit LidarOdometryCore(const Config& config) : config_(config) { ValidateConfig(); }

  ProcessResult Process(const Cloud::Ptr& current, double stamp_s) {
    ProcessResult output;
    output.pose = published_pose_;
    if (!current || static_cast<int>(current->size()) < config_.min_points ||
        !CloudFinite(current)) { output.reason = "TOO_FEW_OR_INVALID_POINTS"; return output; }
    if (!std::isfinite(stamp_s)) { output.reason = "INVALID_TIMESTAMP"; output.outcome = Outcome::kInvalidTimestamp; return output; }
    if (!reference_) {
      Rebaseline(current, stamp_s); recovery_accepts_ = config_.recovery_consecutive_accepts;
      history_.push_back(HistoryEntry{current, matching_pose_});
      output.outcome = Outcome::kBootstrap; output.publish = true; output.healthy = true;
      output.registration.converged = output.registration.transform_valid = output.registration.quality_valid = true;
      output.registration.fitness = 0.01; output.registration.correspondence_ratio = 1.0;
      output.reason = "BOOTSTRAP"; PopulateState(&output); return output;
    }
    const double dt = stamp_s - reference_stamp_s_;
    if (!std::isfinite(dt) || dt <= 0.0) { output.outcome = Outcome::kInvalidTimestamp; output.reason = "NON_INCREASING_TIMESTAMP"; PopulateState(&output); return output; }
    if (dt > config_.max_reference_age_s) { Rebaseline(current, stamp_s); output.outcome = Outcome::kRebaseline; output.publish = true; output.rebuilt_reference = true; output.reason = "REFERENCE_TOO_OLD"; PopulateState(&output); return output; }
    const Cloud::Ptr target = BuildSubmap(); output.target_points = target->size();
    output.translation_limit = std::min(config_.max_step_translation_m, config_.translation_margin_m + config_.max_linear_speed_mps * dt);
    output.rotation_limit = std::min(config_.max_step_rotation_rad, config_.rotation_margin_rad + config_.max_angular_speed_rps * dt);
    RegistrationResult best = Register(current, target, last_increment_);
    if (!PassesAllGates(best, output.translation_limit, output.rotation_limit) &&
        (!best.quality_valid || !best.transform_valid)) {
      const RegistrationResult identity = Register(current, target, Eigen::Isometry3d::Identity());
      if (PassesAllGates(identity, output.translation_limit, output.rotation_limit) ||
          ((!best.quality_valid && identity.quality_valid) || (best.quality_valid == identity.quality_valid && identity.fitness < best.fitness))) best = identity;
    }
    output.registration = best;
    if (PassesAllGates(best, output.translation_limit, output.rotation_limit)) {
      const Eigen::Isometry3d previous_matching_pose = matching_pose_;
      const Eigen::Isometry3d increment = best.delta;
      matching_pose_ = previous_matching_pose * increment;
      pending_delta_ = pending_delta_ * increment;
      const double pending_translation = pending_delta_.translation().norm();
      const double pending_rotation = std::abs(Eigen::AngleAxisd(pending_delta_.rotation()).angle());
      const bool valid_pose = IsPoseValid(matching_pose_) && IsPoseValid(published_pose_ * pending_delta_);
      if (!valid_pose) { ++consecutive_failures_; recovery_accepts_ = 0; output.outcome = Outcome::kRejected; output.publish = true; output.reason = "ABSOLUTE_POSE_LIMIT"; pending_delta_.setIdentity(); matching_pose_ = previous_matching_pose; PopulateState(&output); return output; }
      if (pending_translation > config_.translation_deadband_m || pending_rotation > config_.rotation_deadband_rad) { published_pose_ = published_pose_ * pending_delta_; pending_delta_.setIdentity(); }
      last_increment_ = increment; reference_ = current; reference_stamp_s_ = stamp_s; consecutive_failures_ = 0;
      recovery_accepts_ = std::min(config_.recovery_consecutive_accepts, recovery_accepts_ + 1);
      history_.push_back(HistoryEntry{current, matching_pose_}); while (static_cast<int>(history_.size()) > config_.submap_scans) history_.pop_front();
      output.outcome = Outcome::kAccepted; output.publish = true; output.healthy = recovery_accepts_ >= config_.recovery_consecutive_accepts; output.reason = output.healthy ? "ACCEPTED" : "RECOVERING"; PopulateState(&output); return output;
    }
    ++consecutive_failures_; recovery_accepts_ = 0; output.publish = true; output.outcome = Outcome::kRejected; output.reason = RejectionReason(best, output.translation_limit, output.rotation_limit);
    if (consecutive_failures_ >= config_.rebaseline_after_failures) { Rebaseline(current, stamp_s); output.outcome = Outcome::kRebaseline; output.rebuilt_reference = true; } else last_increment_.setIdentity();
    PopulateState(&output); return output;
  }

  RegistrationResult RegisterForTest(const Cloud::Ptr& source, const Cloud::Ptr& target, const Eigen::Isometry3d& guess = Eigen::Isometry3d::Identity()) const { return Register(source, target, guess); }
  Cloud::Ptr BuildSubmapForTest() const { return BuildSubmap(); }
  Cloud::Ptr LimitRegistrationCloudForTest(const Cloud::Ptr& cloud) const {
    return LimitRegistrationCloud(cloud);
  }
  const Eigen::Isometry3d& pose() const { return published_pose_; }
  int consecutive_failures() const { return consecutive_failures_; }
  int recovery_accepts() const { return recovery_accepts_; }
  std::size_t history_size() const { return history_.size(); }

 private:
  struct HistoryEntry { Cloud::Ptr cloud; Eigen::Isometry3d pose; };
  static bool CloudFinite(const Cloud::Ptr& cloud) { for (const auto& p : cloud->points) if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return false; return true; }
  bool IsPoseValid(const Eigen::Isometry3d& pose) const { return pose.matrix().allFinite() && pose.translation().norm() <= config_.max_absolute_translation_m && (pose.rotation().transpose() * pose.rotation() - Eigen::Matrix3d::Identity()).norm() < 1e-3 && std::abs(pose.rotation().determinant() - 1.0) < 1e-3; }
  void ValidateConfig() const { if (config_.voxel_size_m <= 0.0F || config_.max_correspondence_m <= 0.0 || config_.max_iterations < 1 || config_.max_fitness <= 0.0 || config_.min_correspondence_ratio <= 0.0 || config_.min_correspondence_ratio > 1.0 || config_.max_reference_age_s <= 0.0 || config_.max_step_translation_m <= 0.0 || config_.max_step_rotation_rad <= 0.0 || config_.max_linear_speed_mps <= 0.0 || config_.max_angular_speed_rps <= 0.0 || config_.translation_margin_m < 0.0 || config_.rotation_margin_rad < 0.0 || config_.translation_deadband_m < 0.0 || config_.rotation_deadband_rad < 0.0 || config_.min_points < 3 || config_.submap_scans < 2 || config_.submap_max_points < config_.min_points || config_.registration_max_points < config_.min_points || config_.rebaseline_after_failures < 1 || config_.recovery_consecutive_accepts < 1 || config_.max_absolute_translation_m <= 0.0) throw std::invalid_argument("invalid lidar odometry configuration"); }
  void Rebaseline(const Cloud::Ptr& cloud, double stamp_s) { reference_ = cloud; reference_stamp_s_ = stamp_s; last_increment_.setIdentity(); pending_delta_.setIdentity(); consecutive_failures_ = 0; recovery_accepts_ = 0; history_.clear(); }
  Cloud::Ptr BuildSubmap() const { Cloud::Ptr submap(new Cloud()); if (history_.empty()) { if (reference_) *submap = *reference_; return submap; } for (const auto& entry : history_) { const Eigen::Isometry3d old_to_latest = matching_pose_.inverse() * entry.pose; Cloud transformed; pcl::transformPointCloud(*entry.cloud, transformed, old_to_latest.matrix().cast<float>()); *submap += transformed; } pcl::VoxelGrid<Point> voxel_grid; voxel_grid.setLeafSize(config_.voxel_size_m, config_.voxel_size_m, config_.voxel_size_m); voxel_grid.setInputCloud(submap); Cloud::Ptr filtered(new Cloud()); voxel_grid.filter(*filtered); if (static_cast<int>(filtered->size()) <= config_.submap_max_points) return filtered; Cloud::Ptr limited(new Cloud()); limited->reserve(config_.submap_max_points); const double stride = static_cast<double>(filtered->size()) / static_cast<double>(config_.submap_max_points); for (int i = 0; i < config_.submap_max_points; ++i) limited->push_back((*filtered)[static_cast<std::size_t>(i * stride)]); return limited; }
  Cloud::Ptr LimitRegistrationCloud(const Cloud::Ptr& cloud) const {
    if (!cloud || static_cast<int>(cloud->size()) <= config_.registration_max_points) {
      return cloud;
    }
    Cloud canonical = *cloud;
    std::sort(
        canonical.points.begin(), canonical.points.end(),
        [](const Point& left, const Point& right) {
          if (left.x != right.x) return left.x < right.x;
          if (left.y != right.y) return left.y < right.y;
          return left.z < right.z;
        });
    Cloud::Ptr limited(new Cloud());
    limited->reserve(config_.registration_max_points);
    const double stride = static_cast<double>(canonical.size()) /
                          static_cast<double>(config_.registration_max_points);
    for (int index = 0; index < config_.registration_max_points; ++index) {
      limited->push_back(canonical[static_cast<std::size_t>(index * stride)]);
    }
    limited->width = limited->size();
    limited->height = 1;
    return limited;
  }
  RegistrationResult Register(const Cloud::Ptr& source, const Cloud::Ptr& target, const Eigen::Isometry3d& guess) const { RegistrationResult result; const Cloud::Ptr registration_source = LimitRegistrationCloud(source); const Cloud::Ptr registration_target = LimitRegistrationCloud(target); if (!registration_source || !registration_target || registration_source->empty() || registration_target->empty()) return result; pcl::GeneralizedIterativeClosestPoint<Point, Point> registration; registration.setInputSource(registration_source); registration.setInputTarget(registration_target); registration.setMaxCorrespondenceDistance(config_.max_correspondence_m); registration.setMaximumIterations(config_.max_iterations); registration.setMaximumOptimizerIterations(5); registration.setCorrespondenceRandomness(10); registration.setTransformationEpsilon(1e-5); registration.setEuclideanFitnessEpsilon(1e-5); Cloud aligned; registration.align(aligned, guess.matrix().cast<float>()); result.converged = registration.hasConverged(); result.delta.matrix() = registration.getFinalTransformation().cast<double>(); result.fitness = result.converged ? registration.getFitnessScore(config_.max_correspondence_m) : std::numeric_limits<double>::infinity(); if (!result.delta.matrix().allFinite() || !std::isfinite(result.fitness)) return result; result.translation = result.delta.translation().norm(); result.rotation = std::abs(Eigen::AngleAxisd(result.delta.rotation()).angle()); const Eigen::Matrix3d rotation = result.delta.rotation(); const double rotation_error = (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm(); const double determinant = rotation.determinant(); result.transform_valid = std::isfinite(result.translation) && std::isfinite(result.rotation) && rotation_error <= 1e-3 && std::isfinite(determinant) && std::abs(determinant - 1.0) <= 1e-3; if (!result.transform_valid) return result; pcl::KdTreeFLANN<Point> tree; tree.setInputCloud(registration_target); int matched = 0; std::vector<int> indices(1); std::vector<float> distances(1); for (const auto& point : aligned.points) if (tree.nearestKSearch(point, 1, indices, distances) > 0 && distances[0] <= config_.max_correspondence_m * config_.max_correspondence_m) ++matched; result.correspondence_ratio = aligned.empty() ? 0.0 : static_cast<double>(matched) / static_cast<double>(aligned.size()); result.quality_valid = result.converged && result.fitness <= config_.max_fitness && result.correspondence_ratio >= config_.min_correspondence_ratio; return result; }
  static bool PassesAllGates(const RegistrationResult& result, double translation_limit, double rotation_limit) { return result.transform_valid && result.quality_valid && result.translation <= translation_limit && result.rotation <= rotation_limit; }
  std::string RejectionReason(const RegistrationResult& result, double translation_limit, double rotation_limit) const { if (!result.converged) return "NOT_CONVERGED"; if (!std::isfinite(result.fitness)) return "NON_FINITE_FITNESS"; if (!result.transform_valid) return "INVALID_TRANSFORM"; if (result.fitness > config_.max_fitness) return "FITNESS_LIMIT"; if (result.correspondence_ratio < config_.min_correspondence_ratio) return "CORRESPONDENCE_RATIO_LIMIT"; if (result.translation > translation_limit) return "TRANSLATION_LIMIT"; if (result.rotation > rotation_limit) return "ROTATION_LIMIT"; return "UNKNOWN_REJECTION"; }
  void PopulateState(ProcessResult* result) const { result->pose = published_pose_; result->consecutive_failures = consecutive_failures_; result->recovery_accepts = recovery_accepts_; }
  Config config_; Cloud::Ptr reference_; double reference_stamp_s_ = 0.0; int consecutive_failures_ = 0; int recovery_accepts_ = 0; Eigen::Isometry3d matching_pose_ = Eigen::Isometry3d::Identity(); Eigen::Isometry3d published_pose_ = Eigen::Isometry3d::Identity(); Eigen::Isometry3d last_increment_ = Eigen::Isometry3d::Identity(); Eigen::Isometry3d pending_delta_ = Eigen::Isometry3d::Identity(); std::deque<HistoryEntry> history_;
};

}  // namespace danger_search_localization
