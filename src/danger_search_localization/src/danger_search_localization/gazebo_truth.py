"""ROS-independent pose handling for the Gazebo truth integration source."""

from collections import deque
from dataclasses import dataclass
import math

from .scan_projection import interpolate_planar_pose


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
    z: float
    yaw: float


class GazeboTruthCore:
    """Capture a start frame and serve fresh relative planar truth poses."""

    def __init__(self, max_age_s=0.20, max_future_s=0.05, history_size=1000):
        self.max_age_s = float(max_age_s)
        self.max_future_s = float(max_future_s)
        if not math.isfinite(self.max_age_s) or self.max_age_s <= 0.0:
            raise ValueError("max_age_s must be positive and finite")
        if not math.isfinite(self.max_future_s) or self.max_future_s < 0.0:
            raise ValueError("max_future_s must be non-negative and finite")
        if int(history_size) != history_size or int(history_size) < 2:
            raise ValueError("history_size must be an integer of at least two")
        self.origin = None
        self.latest = None
        self.history = deque(maxlen=int(history_size))

    def reset(self):
        self.origin = None
        self.latest = None
        self.history.clear()

    def update(self, stamp_s, x, y, yaw, z=0.0):
        values = tuple(float(value) for value in (stamp_s, x, y, z, yaw))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("truth sample must be finite")
        stamp_s, x, y, z, yaw = values
        if self.latest is not None and stamp_s < self.latest.stamp_s:
            self.reset()
        yaw = normalize_angle(yaw)
        if self.origin is None:
            self.origin = (x, y, z, yaw)
        sample = TruthSample(stamp_s, x, y, z, yaw)
        if self.history and abs(stamp_s - self.history[-1].stamp_s) <= 1e-9:
            self.history[-1] = sample
        else:
            self.history.append(sample)
        self.latest = sample

    def pose_at(self, stamp_s):
        pose = self.pose_with_height_at(stamp_s)
        if pose is None:
            return None
        return pose[0], pose[1], pose[3]

    def pose_with_height_at(self, stamp_s):
        """Return mission-relative ``x, y, z, yaw`` at a sensor timestamp."""

        stamp_s = float(stamp_s)
        if (not math.isfinite(stamp_s) or self.latest is None
                or self.origin is None or not self.history):
            return None
        epsilon = 1e-9
        oldest = self.history[0]
        latest = self.history[-1]
        if stamp_s > latest.stamp_s + self.max_age_s + epsilon:
            return None
        if stamp_s < oldest.stamp_s - self.max_future_s - epsilon:
            return None

        if stamp_s <= oldest.stamp_s + epsilon:
            selected = (oldest.x, oldest.y, oldest.z, oldest.yaw)
        elif stamp_s >= latest.stamp_s - epsilon:
            selected = (latest.x, latest.y, latest.z, latest.yaw)
        else:
            selected = None
            previous = oldest
            for following in tuple(self.history)[1:]:
                if following.stamp_s + epsilon >= stamp_s:
                    span = following.stamp_s - previous.stamp_s
                    ratio = 0.0 if span <= epsilon else (
                        stamp_s - previous.stamp_s
                    ) / span
                    planar = interpolate_planar_pose(
                        (previous.x, previous.y, previous.yaw),
                        (following.x, following.y, following.yaw),
                        ratio,
                    )
                    selected = (
                        planar[0],
                        planar[1],
                        previous.z + (following.z - previous.z) * ratio,
                        planar[2],
                    )
                    break
                previous = following
            if selected is None:
                return None
        relative_xy_yaw = relative_planar_pose(
            (self.origin[0], self.origin[1], self.origin[3]),
            (selected[0], selected[1], selected[3]),
        )
        return (
            relative_xy_yaw[0],
            relative_xy_yaw[1],
            selected[2] - self.origin[2],
            relative_xy_yaw[2],
        )
