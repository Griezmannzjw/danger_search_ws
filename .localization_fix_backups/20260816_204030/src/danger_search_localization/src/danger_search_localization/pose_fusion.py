"""Fuse smooth local lidar odometry with bounded Hector map corrections.

The local pose is expressed in ``odom`` and is responsible for continuous
motion.  Hector supplies a global ``map`` pose for the same robot.  Their
difference is the ``map -> odom`` correction.  Corrections are deliberately
bounded and low-pass filtered so a bad scan match cannot teleport navigation.

This module has no ROS dependency and is covered by deterministic tests.
"""

from collections import deque
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class FusionResult:
    accepted: bool
    initialized: bool
    pose: Pose2D
    correction: Pose2D
    reason: str
    consecutive_global_rejections: int


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def compose(first, second):
    """Return the SE(2) composition ``first * second``."""
    cosine = math.cos(first.yaw)
    sine = math.sin(first.yaw)
    return Pose2D(
        first.x + cosine * second.x - sine * second.y,
        first.y + sine * second.x + cosine * second.y,
        normalize_angle(first.yaw + second.yaw),
    )


def inverse(pose):
    cosine = math.cos(pose.yaw)
    sine = math.sin(pose.yaw)
    return Pose2D(
        -cosine * pose.x - sine * pose.y,
        sine * pose.x - cosine * pose.y,
        normalize_angle(-pose.yaw),
    )


def relative(reference, pose):
    return compose(inverse(reference), pose)


class HectorGicpFusion:
    """Maintain a smooth map correction over a local GICP trajectory."""

    def __init__(self, config):
        self.config = config
        self.local_history = deque(maxlen=config.fusion_local_history_size)
        self.latest_local = None
        self.correction = Pose2D(0.0, 0.0, 0.0)
        self.initialized = False
        self.last_global_stamp_s = None
        self.last_correction_local = None
        self.consecutive_global_rejections = 0
        self.last_reason = "WAITING_FOR_LOCAL_ODOMETRY"

    def update_local(self, stamp_s, x, y, yaw):
        sample = self._sample(stamp_s, x, y, yaw)
        if self.latest_local is not None and sample[0] <= self.latest_local[0]:
            return self.snapshot(False, "NON_INCREASING_LOCAL_POSE_STAMP")
        self.latest_local = sample
        self.local_history.append(sample)
        self.last_reason = (
            "TRACKING_FUSED_POSE" if self.initialized else "WAITING_FOR_HECTOR_POSE"
        )
        return self.snapshot(True)

    def update_global(self, stamp_s, x, y, yaw):
        global_pose = self._sample(stamp_s, x, y, yaw)
        if (
            self.last_global_stamp_s is not None
            and global_pose[0] <= self.last_global_stamp_s
        ):
            return self._reject("NON_INCREASING_HECTOR_POSE_STAMP")
        local_sample = self._nearest_local(global_pose[0])
        if local_sample is None:
            # ROS callback ordering is not deterministic.  This is a timing
            # condition, not a failed scan match, so it must not poison the
            # correction health counter; the adapter retries after local odom
            # for the same scan arrives.
            return self.snapshot(
                False, "HECTOR_POSE_HAS_NO_SYNCHRONIZED_LOCAL_POSE"
            )

        _, local_pose = local_sample
        candidate = compose(global_pose[1], inverse(local_pose))
        if not self.initialized:
            if (
                math.hypot(candidate.x, candidate.y)
                > self.config.fusion_initial_correction_translation_m
                or abs(candidate.yaw)
                > self.config.fusion_initial_correction_yaw_rad
            ):
                return self._reject("UNSAFE_INITIAL_HECTOR_CORRECTION")
            self.correction = candidate
            self.initialized = True
            self.last_global_stamp_s = global_pose[0]
            self.last_correction_local = local_pose
            self.consecutive_global_rejections = 0
            self.last_reason = "INITIALIZED_HECTOR_MAP_CORRECTION"
            return self.snapshot(True)

        innovation = relative(self.correction, candidate)
        innovation_translation = math.hypot(innovation.x, innovation.y)
        innovation_yaw = abs(innovation.yaw)

        local_motion = relative(self.last_correction_local, local_pose)
        local_stationary = (
            math.hypot(local_motion.x, local_motion.y)
            <= self.config.fusion_stationary_translation_m
            and abs(local_motion.yaw) <= self.config.fusion_stationary_yaw_rad
        )
        correction_outside_deadband = (
            innovation_translation
            > self.config.fusion_stationary_correction_deadband_m
            or innovation_yaw
            > self.config.fusion_stationary_correction_deadband_yaw_rad
        )
        if local_stationary and correction_outside_deadband:
            return self._reject("HECTOR_DRIFT_WHILE_LOCAL_ODOMETRY_STATIONARY")
        if local_stationary:
            # Even sub-threshold Hector noise must not accumulate into metres
            # over a long stationary period.  Treat it as an observation that
            # confirms the existing correction, but do not integrate it.
            self.last_global_stamp_s = global_pose[0]
            self.last_correction_local = local_pose
            self.consecutive_global_rejections = 0
            self.last_reason = "HECTOR_CORRECTION_HELD_WHILE_STATIONARY"
            return self.snapshot(True)
        if (
            innovation_translation
            > self.config.fusion_max_correction_translation_m
        ):
            return self._reject("HECTOR_CORRECTION_TRANSLATION_JUMP")
        if innovation_yaw > self.config.fusion_max_correction_yaw_rad:
            return self._reject("HECTOR_CORRECTION_YAW_JUMP")

        dt = max(0.0, global_pose[0] - self.last_global_stamp_s)
        alpha = 1.0 - math.exp(
            -dt / self.config.fusion_correction_time_constant_s
        )
        self.correction = compose(
            self.correction,
            Pose2D(
                alpha * innovation.x,
                alpha * innovation.y,
                alpha * innovation.yaw,
            ),
        )
        self.last_global_stamp_s = global_pose[0]
        self.last_correction_local = local_pose
        self.consecutive_global_rejections = 0
        self.last_reason = "TRACKING_BOUNDED_HECTOR_CORRECTION"
        return self.snapshot(True)

    def snapshot(self, accepted=True, reason=None):
        local_pose = (
            self.latest_local[1]
            if self.latest_local is not None
            else Pose2D(0.0, 0.0, 0.0)
        )
        return FusionResult(
            accepted=accepted,
            initialized=self.initialized,
            pose=compose(self.correction, local_pose),
            correction=self.correction,
            reason=reason or self.last_reason,
            consecutive_global_rejections=self.consecutive_global_rejections,
        )

    def update_known_global(self, stamp_s):
        """Accept a map built at the trusted local pose with identity correction."""
        stamp_s = float(stamp_s)
        if self.latest_local is None:
            return self.snapshot(
                False, "KNOWN_POSE_HAS_NO_SYNCHRONIZED_LOCAL_POSE"
            )
        self.correction = Pose2D(0.0, 0.0, 0.0)
        self.initialized = True
        self.last_global_stamp_s = stamp_s
        self.last_correction_local = self.latest_local[1]
        self.consecutive_global_rejections = 0
        self.last_reason = "TRACKING_KNOWN_POSE_MAPPING"
        return self.snapshot(True)

    def _nearest_local(self, stamp_s):
        if not self.local_history:
            return None
        sample = min(self.local_history, key=lambda item: abs(item[0] - stamp_s))
        if abs(sample[0] - stamp_s) > self.config.fusion_max_pose_pair_age_s:
            return None
        return sample

    def _reject(self, reason):
        self.consecutive_global_rejections += 1
        self.last_reason = reason
        return self.snapshot(False, reason)

    @staticmethod
    def _sample(stamp_s, x, y, yaw):
        values = tuple(float(value) for value in (stamp_s, x, y, yaw))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("pose contains a non-finite value")
        return values[0], Pose2D(values[1], values[2], normalize_angle(values[3]))
