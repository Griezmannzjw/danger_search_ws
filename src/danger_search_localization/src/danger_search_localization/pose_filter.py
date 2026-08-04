"""Guard a scan-matching pose before exposing it to navigation.

The backend may jitter while stationary or occasionally converge to a wrong
local minimum.  This module anchors the first valid pose at the mission
origin, suppresses sub-centimetre jitter, smooths valid motion, and rejects
physically impossible discontinuities.  It is intentionally independent of
ROS so its safety behaviour can be unit-tested.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class FilteredPose:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class PoseFilterResult:
    accepted: bool
    initialized: bool
    pose: FilteredPose
    reason: str
    consecutive_rejections: int


class PoseStabilizer:
    """SE(2) pose anchor, low-pass filter, and discontinuity gate."""

    def __init__(self, config):
        self.config = config
        self.initialized = False
        self.anchor = FilteredPose(0.0, 0.0, 0.0)
        self.previous_raw = FilteredPose(0.0, 0.0, 0.0)
        self.output = FilteredPose(0.0, 0.0, 0.0)
        self.last_accepted_stamp_s = 0.0
        self.consecutive_rejections = 0
        self.total_rejections = 0
        self.last_reason = "WAITING_FOR_FIRST_POSE"

    def update(self, stamp_s, x, y, yaw):
        values = tuple(float(value) for value in (stamp_s, x, y, yaw))
        if not all(math.isfinite(value) for value in values):
            return self._reject("NON_FINITE_RAW_POSE")
        stamp_s, x, y, yaw = values
        yaw = normalize_angle(yaw)

        if not self.initialized:
            self.initialized = True
            self.anchor = FilteredPose(x, y, yaw)
            self.previous_raw = FilteredPose(0.0, 0.0, 0.0)
            self.output = FilteredPose(0.0, 0.0, 0.0)
            self.last_accepted_stamp_s = stamp_s
            self.consecutive_rejections = 0
            self.last_reason = "INITIALIZED_AT_MISSION_ORIGIN"
            return self.snapshot(True)

        target = self._relative_to_anchor(x, y, yaw)
        dt = stamp_s - self.last_accepted_stamp_s
        if dt <= 0.0:
            return self._reject("NON_INCREASING_RAW_POSE_STAMP")

        gate_dt = min(dt, self.config.pose_gate_max_dt_s)
        translation_step = math.hypot(
            target.x - self.previous_raw.x,
            target.y - self.previous_raw.y,
        )
        yaw_step = abs(normalize_angle(target.yaw - self.previous_raw.yaw))
        allowed_translation = (
            self.config.pose_jump_translation_margin_m
            + self.config.pose_max_linear_speed_mps * gate_dt
        )
        allowed_yaw = (
            self.config.pose_jump_yaw_margin_rad
            + self.config.pose_max_angular_speed_rps * gate_dt
        )
        if translation_step > allowed_translation:
            return self._reject("RAW_POSE_TRANSLATION_JUMP")
        if yaw_step > allowed_yaw:
            return self._reject("RAW_POSE_YAW_JUMP")

        self.previous_raw = target
        self.last_accepted_stamp_s = stamp_s
        self.consecutive_rejections = 0

        residual_x = target.x - self.output.x
        residual_y = target.y - self.output.y
        residual_yaw = normalize_angle(target.yaw - self.output.yaw)
        if (
            math.hypot(residual_x, residual_y)
            <= self.config.pose_position_deadband_m
            and abs(residual_yaw) <= self.config.pose_yaw_deadband_rad
        ):
            self.last_reason = "STATIONARY_JITTER_SUPPRESSED"
            return self.snapshot(True)

        alpha = 1.0 - math.exp(
            -dt / self.config.pose_filter_time_constant_s
        )
        self.output = FilteredPose(
            self.output.x + alpha * residual_x,
            self.output.y + alpha * residual_y,
            normalize_angle(self.output.yaw + alpha * residual_yaw),
        )
        self.last_reason = "TRACKING_FILTERED_SCAN_MATCHING"
        return self.snapshot(True)

    def _relative_to_anchor(self, x, y, yaw):
        dx = x - self.anchor.x
        dy = y - self.anchor.y
        cosine = math.cos(self.anchor.yaw)
        sine = math.sin(self.anchor.yaw)
        return FilteredPose(
            cosine * dx + sine * dy,
            -sine * dx + cosine * dy,
            normalize_angle(yaw - self.anchor.yaw),
        )

    def _reject(self, reason):
        self.consecutive_rejections += 1
        self.total_rejections += 1
        self.last_reason = reason
        return self.snapshot(False)

    def snapshot(self, accepted=None):
        if accepted is None:
            accepted = self.consecutive_rejections == 0
        return PoseFilterResult(
            accepted=accepted,
            initialized=self.initialized,
            pose=self.output,
            reason=self.last_reason,
            consecutive_rejections=self.consecutive_rejections,
        )


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))
