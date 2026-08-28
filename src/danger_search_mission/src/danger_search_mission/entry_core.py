"""Pure helpers for the guarded short-range entrance crossing."""

import math


def normalize_angle(value):
    """Return an angle in ``[-pi, pi]``."""
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("angle must be finite")
    return math.atan2(math.sin(value), math.cos(value))


def quaternion_roll_pitch(x, y, z, w):
    """Return roll and pitch for a finite, non-zero quaternion."""
    values = tuple(float(value) for value in (x, y, z, w))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("quaternion must be finite")
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-9:
        raise ValueError("quaternion norm is zero")
    x, y, z, w = (value / norm for value in values)
    roll = math.atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    pitch_term = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    return roll, math.asin(pitch_term)


def crossing_command(
        lateral_error_m, yaw_error_rad, speed_mps, yaw_gain,
        lateral_gain, max_angular_speed_rps):
    """Compute a bounded forward command that holds the captured entry line.

    Positive lateral error means the robot is left of the captured entry line,
    so both positive heading and lateral errors request a negative yaw command.
    """
    values = tuple(float(value) for value in (
        lateral_error_m,
        yaw_error_rad,
        speed_mps,
        yaw_gain,
        lateral_gain,
        max_angular_speed_rps,
    ))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("crossing command inputs must be finite")
    lateral_error_m, yaw_error_rad, speed_mps, yaw_gain, lateral_gain, limit = values
    if speed_mps <= 0.0 or yaw_gain < 0.0 or lateral_gain < 0.0 or limit < 0.0:
        raise ValueError("crossing gains and limits are invalid")
    angular = -yaw_gain * yaw_error_rad - lateral_gain * lateral_error_m
    angular = max(-limit, min(limit, angular))
    return speed_mps, angular


def margin_avoidance_command(
        linear_speed_mps, angular_speed_rps, obstacle_y_m,
        avoidance_speed_mps, avoidance_yaw_rps, max_angular_speed_rps):
    """Slow down and steer away from a point in the lateral safety margin.

    This helper is only for a point outside the physical footprint corridor;
    callers must still treat any hit in the uninflated swept footprint as a
    hard stop.  Positive laser ``y`` is to the robot's left and therefore
    requests a negative (right-turning) yaw correction.
    """
    values = tuple(float(value) for value in (
        linear_speed_mps,
        angular_speed_rps,
        obstacle_y_m,
        avoidance_speed_mps,
        avoidance_yaw_rps,
        max_angular_speed_rps,
    ))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("margin avoidance inputs must be finite")
    linear, angular, obstacle_y, avoidance_speed, avoidance_yaw, limit = values
    if (linear < 0.0 or avoidance_speed <= 0.0 or avoidance_yaw <= 0.0
            or limit <= 0.0):
        raise ValueError("margin avoidance configuration is invalid")
    linear = min(linear, avoidance_speed)
    if abs(obstacle_y) >= 1e-9:
        angular -= math.copysign(avoidance_yaw, obstacle_y)
        # The line follower may currently request a turn toward the nearby
        # side wall.  Keep its contribution when already safe, but never let
        # it reverse the avoidance direction.
        if angular * obstacle_y >= 0.0:
            angular = -math.copysign(avoidance_yaw, obstacle_y)
    angular = max(-limit, min(limit, angular))
    return linear, angular


class CrossingReference:
    """Frozen local frame for one guarded entrance crossing.

    The task-start pose remains the mission's return-home reference.  This
    separate frame is intentionally captured only when the short-range lease
    takes control, so small pose changes while standing or entering gait do
    not turn the apron crossing into a correction back to the task origin.
    """

    def __init__(self, x_m, y_m, yaw_rad):
        values = tuple(float(value) for value in (x_m, y_m, yaw_rad))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("crossing reference must be finite")
        self.x_m, self.y_m, self.yaw_rad = values

    def errors(self, x_m, y_m, yaw_rad):
        """Return forward, left-lateral and yaw error in this local frame."""
        values = tuple(float(value) for value in (x_m, y_m, yaw_rad))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("crossing pose must be finite")
        x_m, y_m, yaw_rad = values
        dx = x_m - self.x_m
        dy = y_m - self.y_m
        cosine = math.cos(self.yaw_rad)
        sine = math.sin(self.yaw_rad)
        return (
            cosine * dx + sine * dy,
            -sine * dx + cosine * dy,
            normalize_angle(yaw_rad - self.yaw_rad),
        )


class YawErrorFilter:
    """Deadband then exponentially smooth a local heading error.

    The exact zero inside the deadband prevents a stationary robot's pose
    jitter from commanding alternating yaw.  Outside it, the low-pass filter
    limits the first correction and avoids a one-tick sign flip when gait
    estimation moves around the desired heading.
    """

    def __init__(self, deadband_rad, alpha):
        self.deadband_rad = float(deadband_rad)
        self.alpha = float(alpha)
        if (not math.isfinite(self.deadband_rad)
                or self.deadband_rad < 0.0
                or not math.isfinite(self.alpha)
                or not 0.0 < self.alpha <= 1.0):
            raise ValueError("yaw filter parameters are invalid")
        self.reset()

    def reset(self):
        self.filtered_error_rad = 0.0

    def update(self, yaw_error_rad):
        yaw_error_rad = float(yaw_error_rad)
        if not math.isfinite(yaw_error_rad):
            raise ValueError("yaw error must be finite")
        if abs(yaw_error_rad) <= self.deadband_rad:
            # Do not leave a residual turn command after the robot is aligned.
            self.filtered_error_rad = 0.0
            return self.filtered_error_rad
        target = math.copysign(
            abs(yaw_error_rad) - self.deadband_rad,
            yaw_error_rad,
        )
        self.filtered_error_rad = (
            self.alpha * target
            + (1.0 - self.alpha) * self.filtered_error_rad
        )
        return self.filtered_error_rad


def command_is_zero(command, linear_tolerance=0.01, angular_tolerance=0.02):
    """Check the three planar components of a Twist-like object."""
    if command is None:
        return False
    return (
        math.hypot(float(command.linear.x), float(command.linear.y))
        <= float(linear_tolerance)
        and abs(float(command.angular.z)) <= float(angular_tolerance)
    )


def rolling_sweep_distance(remaining_distance_m, lookahead_distance_m):
    """Bound a straight footprint sweep to the controller's local horizon.

    The entrance controller continuously corrects heading and lateral error.
    Treating the complete remaining path as a straight segment in the current
    base frame therefore predicts collisions that the closed loop will never
    reach (most visibly, an open sliding-door panel during a small yaw error).
    A fresh scan is checked every control tick, so the conservative geometry
    is retained over a finite rolling horizon instead.
    """
    remaining_distance_m = float(remaining_distance_m)
    lookahead_distance_m = float(lookahead_distance_m)
    if (not math.isfinite(remaining_distance_m)
            or not math.isfinite(lookahead_distance_m)
            or remaining_distance_m < 0.0
            or lookahead_distance_m <= 0.0):
        raise ValueError("rolling sweep distances are invalid")
    return min(remaining_distance_m, lookahead_distance_m)


class ProgressWatchdog:
    """Detect a commanded crossing that has stopped making forward progress."""

    def __init__(self, minimum_increment_m, timeout_s):
        self.minimum_increment_m = float(minimum_increment_m)
        self.timeout_s = float(timeout_s)
        if (not math.isfinite(self.minimum_increment_m)
                or self.minimum_increment_m <= 0.0
                or not math.isfinite(self.timeout_s)
                or self.timeout_s <= 0.0):
            raise ValueError("progress watchdog parameters must be positive")
        self.best_progress_m = 0.0
        self.last_progress_time_s = None

    def reset(self, progress_m, now_s):
        progress_m = float(progress_m)
        now_s = float(now_s)
        if not math.isfinite(progress_m) or not math.isfinite(now_s):
            raise ValueError("progress watchdog state must be finite")
        self.best_progress_m = progress_m
        self.last_progress_time_s = now_s

    def update(self, progress_m, now_s):
        progress_m = float(progress_m)
        now_s = float(now_s)
        if not math.isfinite(progress_m) or not math.isfinite(now_s):
            return False
        if self.last_progress_time_s is None:
            self.reset(progress_m, now_s)
            return True
        if progress_m >= self.best_progress_m + self.minimum_increment_m:
            self.best_progress_m = progress_m
            self.last_progress_time_s = now_s
            return True
        return now_s - self.last_progress_time_s <= self.timeout_s
