"""ROS-independent planar pose handling for the Gazebo truth test source."""

from dataclasses import dataclass
import math


def normalize_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    if not math.isfinite(float(angle)):
        raise ValueError("angle must be finite")
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def quaternion_yaw(x, y, z, w):
    values = tuple(float(value) for value in (x, y, z, w))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("quaternion must be finite")
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-9:
        raise ValueError("quaternion norm is zero")
    x, y, z, w = (value / norm for value in values)
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def relative_planar_pose(origin, current):
    """Express a world-frame SE(2) pose relative to the captured origin."""
    values = tuple(float(value) for value in (*origin, *current))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("poses must be finite")
    origin_x, origin_y, origin_yaw = values[:3]
    current_x, current_y, current_yaw = values[3:]
    dx = current_x - origin_x
    dy = current_y - origin_y
    cosine = math.cos(origin_yaw)
    sine = math.sin(origin_yaw)
    return (
        cosine * dx + sine * dy,
        -sine * dx + cosine * dy,
        normalize_angle(current_yaw - origin_yaw),
    )


@dataclass(frozen=True)
class TruthSample:
    stamp_s: float
    x: float
    y: float
    yaw: float


class GazeboTruthCore:
    """Capture a start frame and serve fresh relative planar truth poses."""

    def __init__(self, max_age_s=0.20, max_future_s=0.05):
        self.max_age_s = float(max_age_s)
        self.max_future_s = float(max_future_s)
        if not math.isfinite(self.max_age_s) or self.max_age_s <= 0.0:
            raise ValueError("max_age_s must be positive and finite")
        if not math.isfinite(self.max_future_s) or self.max_future_s < 0.0:
            raise ValueError("max_future_s must be non-negative and finite")
        self.origin = None
        self.latest = None

    def reset(self):
        self.origin = None
        self.latest = None

    def update(self, stamp_s, x, y, yaw):
        values = tuple(float(value) for value in (stamp_s, x, y, yaw))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("truth sample must be finite")
        stamp_s, x, y, yaw = values
        if self.latest is not None and stamp_s < self.latest.stamp_s:
            self.reset()
        yaw = normalize_angle(yaw)
        if self.origin is None:
            self.origin = (x, y, yaw)
        self.latest = TruthSample(stamp_s, x, y, yaw)

    def pose_at(self, stamp_s):
        stamp_s = float(stamp_s)
        if not math.isfinite(stamp_s) or self.latest is None or self.origin is None:
            return None
        age = stamp_s - self.latest.stamp_s
        epsilon = 1e-9
        if age > self.max_age_s + epsilon or age < -self.max_future_s - epsilon:
            return None
        return relative_planar_pose(
            self.origin,
            (self.latest.x, self.latest.y, self.latest.yaw),
        )
