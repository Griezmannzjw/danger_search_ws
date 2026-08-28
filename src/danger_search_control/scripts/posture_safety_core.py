#!/usr/bin/env python3
"""ROS-independent, fail-safe posture safety state machine."""

import math


def quaternion_roll_pitch(x, y, z, w):
    values = tuple(float(v) for v in (x, y, z, w))
    if not all(math.isfinite(v) for v in values):
        raise ValueError("orientation contains non-finite values")
    norm = math.sqrt(sum(v * v for v in values))
    if norm < 1e-6:
        raise ValueError("orientation quaternion has zero norm")
    x, y, z, w = (v / norm for v in values)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    return roll, pitch


class PostureSafetyState:
    def __init__(self, tilt_limit_rad, trigger_hold_s, recovery_limit_rad,
                 recovery_hold_s, imu_timeout_s):
        values = (tilt_limit_rad, trigger_hold_s, recovery_limit_rad,
                  recovery_hold_s, imu_timeout_s)
        if not all(math.isfinite(float(v)) and float(v) > 0.0 for v in values):
            raise ValueError("posture safety limits and timeouts must be positive")
        if recovery_limit_rad >= tilt_limit_rad:
            raise ValueError("recovery tilt must be below trigger tilt")
        self.tilt_limit = float(tilt_limit_rad)
        self.trigger_hold = float(trigger_hold_s)
        self.recovery_limit = float(recovery_limit_rad)
        self.recovery_hold = float(recovery_hold_s)
        self.imu_timeout = float(imu_timeout_s)
        self.latched = True
        self.reason = "imu_not_received"
        self.last_sample_time = None
        self.over_limit_since = None
        self.stable_since = None
        self.last_tilt = None
        self.last_sample_valid = False
        self.fallen = False

    def observe(self, now, quaternion):
        now = float(now)
        try:
            roll, pitch = quaternion_roll_pitch(*quaternion)
        except (TypeError, ValueError) as exc:
            self._trip("invalid_imu:" + str(exc))
            self.last_sample_time = now
            self.last_sample_valid = False
            return self.latched, self.reason
        self.last_sample_time = now
        self.last_sample_valid = True
        tilt = max(abs(roll), abs(pitch))
        self.last_tilt = tilt
        if tilt > self.tilt_limit:
            self.stable_since = None
            if self.over_limit_since is None:
                self.over_limit_since = now
            if now - self.over_limit_since >= self.trigger_hold:
                self._trip("excessive_tilt:%.1fdeg" % math.degrees(tilt))
        else:
            self.over_limit_since = None
            if self.latched and tilt <= self.recovery_limit:
                if self.stable_since is None:
                    self.stable_since = now
                elif now - self.stable_since >= self.recovery_hold:
                    self.latched = False
                    self.fallen = False
                    self.reason = "stable_recovery"
            else:
                self.stable_since = None
        return self.latched, self.reason

    def tick(self, now):
        now = float(now)
        if (self.last_sample_time is None
                or now - self.last_sample_time > self.imu_timeout
                or now < self.last_sample_time):
            self._trip("imu_stale")
        return self.latched, self.reason

    def reset(self, now):
        now = float(now)
        self.tick(now)
        if (not self.last_sample_valid
                or self.last_sample_time is None or self.last_tilt is None
                or now < self.last_sample_time
                or now - self.last_sample_time > self.imu_timeout):
            return False, "fresh valid IMU required"
        if self.last_tilt > self.recovery_limit:
            return False, "posture is outside recovery limit"
        self.latched = False
        self.fallen = False
        self.reason = "explicit_reset"
        self.over_limit_since = None
        self.stable_since = None
        return True, self.reason

    def _trip(self, reason):
        self.latched = True
        self.reason = str(reason)
        if self.reason.startswith("excessive_tilt:"):
            self.fallen = True
        self.stable_since = None
