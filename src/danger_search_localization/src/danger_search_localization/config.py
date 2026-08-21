"""Validated configuration for localization nodes."""

from dataclasses import dataclass
import math


def _require(condition, message):
    if not condition:
        raise ValueError(message)


@dataclass(frozen=True)
class ScanProjectionConfig:
    angle_min: float = -3.141592653589793
    angle_max: float = 3.141592653589793
    angle_increment: float = 0.008726646259972
    range_min: float = 0.20
    range_max: float = 12.0
    min_height: float = 0.05
    max_height: float = 1.20
    self_exclusion_min_x: float = -0.55
    self_exclusion_max_x: float = 0.55
    self_exclusion_half_width_y: float = 0.40
    min_returns_per_bin: int = 2
    max_intra_bin_range_gap: float = 0.20
    enable_isolated_hit_filter: bool = True
    neighbor_window_bins: int = 8
    min_neighbor_support: int = 1
    max_neighbor_range_jump: float = 0.75
    tf_timeout_s: float = 0.10
    imu_topic: str = "/livox/imu"
    enable_imu_leveling: bool = True
    imu_fresh_timeout_s: float = 0.05
    max_abs_roll_rad: float = 0.55
    max_abs_pitch_rad: float = 0.55
    max_angular_speed_rps: float = 1.50
    enable_ground_clearance_gate: bool = True
    ground_candidate_max_z: float = 0.05
    ground_candidate_min_range: float = 0.40
    ground_candidate_max_range: float = 3.00
    min_ground_candidate_points: int = 30
    min_ground_clearance_m: float = 0.18
    scan_accumulation_frames: int = 3
    scan_accumulation_min_samples_per_bin: int = 1
    scan_accumulation_max_age_s: float = 0.6
    min_valid_scan_bins: int = 8
    min_angular_coverage_rad: float = 0.05
    drop_unstable_scans: bool = True

    def __post_init__(self):
        _require(self.angle_min < self.angle_max, "scan angle range is invalid")
        _require(self.angle_increment > 0.0, "angle_increment must be positive")
        _require(0.0 <= self.range_min < self.range_max, "scan range is invalid")
        _require(self.min_height < self.max_height, "height range is invalid")
        _require(
            self.self_exclusion_min_x < self.self_exclusion_max_x,
            "self exclusion x range is invalid",
        )
        _require(
            self.self_exclusion_half_width_y >= 0.0,
            "self_exclusion_half_width_y cannot be negative",
        )
        _require(
            self.min_returns_per_bin >= 1,
            "min_returns_per_bin must be at least one",
        )
        _require(
            self.max_intra_bin_range_gap > 0.0,
            "max_intra_bin_range_gap must be positive",
        )
        _require(
            self.neighbor_window_bins >= 1,
            "neighbor_window_bins must be at least one",
        )
        _require(
            self.min_neighbor_support >= 1,
            "min_neighbor_support must be at least one",
        )
        _require(
            self.max_neighbor_range_jump > 0.0,
            "max_neighbor_range_jump must be positive",
        )
        _require(self.tf_timeout_s >= 0.0, "tf_timeout_s cannot be negative")
        _require(bool(self.imu_topic), "imu_topic cannot be empty")
        _require(self.imu_fresh_timeout_s > 0.0, "IMU timeout must be positive")
        _require(self.max_abs_roll_rad > 0.0, "maximum roll must be positive")
        _require(self.max_abs_pitch_rad > 0.0, "maximum pitch must be positive")
        _require(self.max_angular_speed_rps > 0.0, "maximum angular speed must be positive")
        _require(
            0.0 <= self.ground_candidate_min_range
            < self.ground_candidate_max_range,
            "ground candidate range is invalid",
        )
        _require(
            self.min_ground_candidate_points >= 1,
            "minimum ground candidate count must be positive",
        )
        _require(self.min_ground_clearance_m > 0.0, "minimum clearance must be positive")
        _require(
            self.scan_accumulation_frames >= 1,
            "scan accumulation frame count must be positive",
        )
        _require(
            1 <= self.scan_accumulation_min_samples_per_bin
            <= self.scan_accumulation_frames,
            "scan accumulation minimum samples must be within the window",
        )
        _require(
            math.isfinite(self.scan_accumulation_max_age_s)
            and self.scan_accumulation_max_age_s > 0.0,
            "scan accumulation maximum age must be positive",
        )
        _require(
            self.min_valid_scan_bins >= 1,
            "min_valid_scan_bins must be at least one",
        )
        _require(
            self.min_angular_coverage_rad > 0.0,
            "min_angular_coverage_rad must be positive",
        )

    @property
    def bin_count(self):
        return int(round((self.angle_max - self.angle_min) / self.angle_increment)) + 1


@dataclass(frozen=True)
class AdapterConfig:
    pose_fresh_timeout_s: float = 2.0
    map_fresh_timeout_s: float = 5.0
    min_map_updates_for_stable: int = 2
    pose_publish_rate_hz: float = 20.0
    tf_publish_future_tolerance_s: float = 0.5
    status_publish_rate_hz: float = 2.0
    fallback_xy_variance: float = 0.10
    fallback_yaw_variance: float = 0.15
    fallback_unobserved_variance: float = 0.25
    use_backend_covariance: bool = False
    covariance_warning_trace: float = 2.0
    current_floor: int = 0
    pose_position_deadband_m: float = 0.015
    pose_yaw_deadband_rad: float = 0.010
    pose_filter_time_constant_s: float = 0.15
    pose_max_linear_speed_mps: float = 1.0
    pose_max_angular_speed_rps: float = 2.0
    pose_jump_translation_margin_m: float = 0.08
    pose_jump_yaw_margin_rad: float = 0.10
    pose_gate_max_dt_s: float = 0.50
    pose_rejections_before_lost: int = 3
    fusion_local_history_size: int = 30
    fusion_max_pose_pair_age_s: float = 0.80
    fusion_initial_correction_translation_m: float = 0.30
    fusion_initial_correction_yaw_rad: float = 0.35
    fusion_max_correction_translation_m: float = 0.30
    fusion_max_correction_yaw_rad: float = 0.25
    fusion_correction_time_constant_s: float = 0.75
    fusion_stationary_translation_m: float = 0.025
    fusion_stationary_yaw_rad: float = 0.020
    fusion_stationary_correction_deadband_m: float = 0.025
    fusion_stationary_correction_deadband_yaw_rad: float = 0.020
    gicp_unhealthy_variance_threshold: float = 1.0
    fusion_max_absolute_pose_translation_m: float = 100.0
    gicp_healthy_fresh_timeout_s: float = 1.0
    gicp_healthy_lost_timeout_s: float = 3.0
    hector_rejections_before_degraded: int = 3
    hector_pose_fresh_timeout_s: float = 2.0
    vertical_estimation_enabled: bool = False
    imu_topic: str = "/livox/imu"
    vertical_imu_fresh_timeout_s: float = 0.20
    vertical_integration_rate_hz: float = 200.0
    gravity_mps2: float = 9.80665
    stationary_accel_tolerance_mps2: float = 0.30
    stationary_gyro_threshold_rps: float = 0.08
    stationary_hold_s: float = 0.40
    vertical_bias_learning_rate: float = 0.01
    max_vertical_acceleration_mps2: float = 5.0
    max_vertical_speed_mps: float = 1.5
    vertical_max_abs_roll_rad: float = 0.26
    vertical_max_abs_pitch_rad: float = 0.26
    vertical_velocity_damping_per_s: float = 2.0
    vertical_acceleration_deadband_mps2: float = 0.15
    floor_heights: tuple = (0.0, 2.6, 5.2)
    floor_switch_hysteresis_m: float = 0.20
    floor_snap_tolerance_m: float = 0.35

    def __post_init__(self):
        _require(self.pose_fresh_timeout_s > 0.0, "pose timeout must be positive")
        _require(self.map_fresh_timeout_s > 0.0, "map timeout must be positive")
        _require(self.min_map_updates_for_stable >= 1, "min_map_updates_for_stable must be positive")
        _require(self.pose_publish_rate_hz >= 10.0, "pose publish rate must be at least 10 Hz")
        _require(
            0.0 <= self.tf_publish_future_tolerance_s <= 1.0,
            "TF publish future tolerance must be within [0, 1] seconds",
        )
        _require(self.status_publish_rate_hz > 0.0, "status publish rate must be positive")
        _require(self.fallback_xy_variance > 0.0, "fallback_xy_variance must be positive")
        _require(self.fallback_yaw_variance > 0.0, "fallback_yaw_variance must be positive")
        _require(
            self.fallback_unobserved_variance > 0.0,
            "fallback_unobserved_variance must be positive",
        )
        _require(self.covariance_warning_trace > 0.0, "covariance warning trace must be positive")
        _require(self.current_floor >= 0, "current_floor cannot be negative")
        _require(self.pose_position_deadband_m >= 0.0, "pose deadband cannot be negative")
        _require(self.pose_yaw_deadband_rad >= 0.0, "yaw deadband cannot be negative")
        _require(self.pose_filter_time_constant_s > 0.0, "pose filter time constant must be positive")
        _require(self.pose_max_linear_speed_mps > 0.0, "maximum pose speed must be positive")
        _require(self.pose_max_angular_speed_rps > 0.0, "maximum pose yaw rate must be positive")
        _require(self.pose_jump_translation_margin_m >= 0.0, "pose jump margin cannot be negative")
        _require(self.pose_jump_yaw_margin_rad >= 0.0, "pose yaw margin cannot be negative")
        _require(self.pose_gate_max_dt_s > 0.0, "pose gate dt must be positive")
        _require(self.pose_rejections_before_lost >= 1, "pose rejection limit must be positive")
        _require(self.fusion_local_history_size >= 2, "fusion history must contain at least two poses")
        _require(self.fusion_max_pose_pair_age_s > 0.0, "fusion pose pair timeout must be positive")
        _require(
            self.fusion_initial_correction_translation_m > 0.0
            and self.fusion_initial_correction_yaw_rad > 0.0,
            "initial fusion correction limits must be positive",
        )
        _require(
            self.fusion_max_correction_translation_m > 0.0
            and self.fusion_max_correction_yaw_rad > 0.0,
            "fusion correction limits must be positive",
        )
        _require(
            self.fusion_correction_time_constant_s > 0.0,
            "fusion correction time constant must be positive",
        )
        _require(
            self.fusion_stationary_translation_m >= 0.0
            and self.fusion_stationary_yaw_rad >= 0.0,
            "fusion stationary thresholds cannot be negative",
        )
        _require(
            self.fusion_stationary_correction_deadband_m >= 0.0
            and self.fusion_stationary_correction_deadband_yaw_rad >= 0.0,
            "fusion stationary correction deadbands cannot be negative",
        )
        _require(
            self.gicp_unhealthy_variance_threshold > 0.0,
            "GICP unhealthy variance threshold must be positive",
        )
        _require(
            self.fusion_max_absolute_pose_translation_m > 0.0,
            "fusion absolute pose limit must be positive",
        )
        _require(
            0.0 < self.gicp_healthy_fresh_timeout_s
            < self.gicp_healthy_lost_timeout_s,
            "GICP healthy-pose timeouts are invalid",
        )
        _require(
            self.hector_rejections_before_degraded >= 1,
            "Hector rejection threshold must be positive",
        )
        _require(
            self.hector_pose_fresh_timeout_s > 0.0,
            "Hector pose timeout must be positive",
        )
        _require(bool(self.imu_topic), "imu_topic cannot be empty")
        _require(
            self.vertical_imu_fresh_timeout_s > 0.0,
            "vertical IMU timeout must be positive",
        )
        _require(
            self.vertical_integration_rate_hz > 0.0,
            "vertical integration rate must be positive",
        )
        _require(self.gravity_mps2 > 0.0, "gravity must be positive")
        _require(
            self.stationary_accel_tolerance_mps2 > 0.0,
            "stationary acceleration tolerance must be positive",
        )
        _require(
            self.stationary_gyro_threshold_rps > 0.0,
            "stationary gyro threshold must be positive",
        )
        _require(self.stationary_hold_s >= 0.0, "stationary hold cannot be negative")
        _require(
            0.0 < self.vertical_bias_learning_rate <= 1.0,
            "vertical bias learning rate must be in (0, 1]",
        )
        _require(
            self.max_vertical_acceleration_mps2 > 0.0,
            "maximum vertical acceleration must be positive",
        )
        _require(self.max_vertical_speed_mps > 0.0, "maximum vertical speed must be positive")
        _require(self.vertical_max_abs_roll_rad > 0.0, "vertical roll limit must be positive")
        _require(self.vertical_max_abs_pitch_rad > 0.0, "vertical pitch limit must be positive")
        _require(
            self.vertical_velocity_damping_per_s >= 0.0,
            "vertical velocity damping cannot be negative",
        )
        _require(
            self.vertical_acceleration_deadband_mps2 >= 0.0,
            "vertical acceleration deadband cannot be negative",
        )
        heights = tuple(float(value) for value in self.floor_heights)
        _require(bool(heights), "floor_heights cannot be empty")
        _require(
            all(b > a for a, b in zip(heights, heights[1:])),
            "floor_heights must be strictly increasing",
        )
        _require(self.current_floor < len(heights), "current_floor is outside floor_heights")
        _require(
            self.floor_switch_hysteresis_m >= 0.0,
            "floor switch hysteresis cannot be negative",
        )
        _require(self.floor_snap_tolerance_m >= 0.0, "floor snap tolerance cannot be negative")
        object.__setattr__(self, "floor_heights", heights)
