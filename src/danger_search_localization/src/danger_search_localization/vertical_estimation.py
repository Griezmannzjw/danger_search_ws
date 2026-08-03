"""P0 vertical inertial estimator and floor-state machine.

This is deliberately isolated behind the localization interface so a LIO
backend can replace it later.  It never reads Gazebo truth.  The estimator
anchors Z at the robot start, removes gravity using the IMU orientation,
applies zero-velocity updates while stationary, and uses known floor heights
only to classify/snap completed floor transitions.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class VerticalState:
    initialized: bool
    stamp_s: float
    z: float
    velocity_z: float
    roll: float
    pitch: float
    current_floor: int
    stationary: bool


class VerticalEstimator:
    def __init__(self, config):
        self.config = config
        self.initialized = False
        self.last_stamp_s = 0.0
        self.z = float(config.floor_heights[config.current_floor])
        self.velocity_z = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.current_floor = int(config.current_floor)
        self.vertical_accel_bias = 0.0
        self.stationary_elapsed_s = 0.0
        self.stationary = False

    def update(self, stamp_s, quaternion, angular_velocity, linear_acceleration):
        stamp_s = float(stamp_s)
        qx, qy, qz, qw = _normalized_quaternion(quaternion)
        self.roll, self.pitch, _ = quaternion_to_rpy((qx, qy, qz, qw))
        ax, ay, az = (float(value) for value in linear_acceleration)
        gx, gy, gz = (float(value) for value in angular_velocity)
        acceleration_norm = math.sqrt(ax * ax + ay * ay + az * az)
        gyro_norm = math.sqrt(gx * gx + gy * gy + gz * gz)
        vertical_specific_force = _world_vertical_component(
            (ax, ay, az), (qx, qy, qz, qw)
        )
        raw_vertical_acceleration = vertical_specific_force - self.config.gravity_mps2

        if not self.initialized:
            self.initialized = True
            self.last_stamp_s = stamp_s
            self.vertical_accel_bias = raw_vertical_acceleration
            return self.snapshot()

        dt = stamp_s - self.last_stamp_s
        minimum_dt = 1.0 / self.config.vertical_integration_rate_hz
        if dt < minimum_dt:
            return self.snapshot()
        self.last_stamp_s = stamp_s
        if dt <= 0.0 or dt > 0.1:
            self.velocity_z = 0.0
            self.stationary_elapsed_s = 0.0
            self.stationary = False
            return self.snapshot()

        if (
            abs(self.roll) > self.config.vertical_max_abs_roll_rad
            or abs(self.pitch) > self.config.vertical_max_abs_pitch_rad
        ):
            # A fallen/strongly tilted quadruped makes pure IMU integration
            # unusable. Freeze height rather than publishing unbounded drift.
            self.velocity_z = 0.0
            self.stationary_elapsed_s = 0.0
            self.stationary = False
            return self.snapshot()

        stationary_candidate = (
            abs(acceleration_norm - self.config.gravity_mps2)
            <= self.config.stationary_accel_tolerance_mps2
            and gyro_norm <= self.config.stationary_gyro_threshold_rps
        )
        if stationary_candidate:
            self.stationary_elapsed_s += dt
        else:
            self.stationary_elapsed_s = 0.0
        self.stationary = (
            self.stationary_elapsed_s >= self.config.stationary_hold_s
        )

        corrected_acceleration = raw_vertical_acceleration - self.vertical_accel_bias
        if (
            abs(corrected_acceleration)
            < self.config.vertical_acceleration_deadband_mps2
        ):
            corrected_acceleration = 0.0
        corrected_acceleration = max(
            -self.config.max_vertical_acceleration_mps2,
            min(self.config.max_vertical_acceleration_mps2, corrected_acceleration),
        )
        if self.stationary:
            learning_rate = self.config.vertical_bias_learning_rate
            self.vertical_accel_bias = (
                (1.0 - learning_rate) * self.vertical_accel_bias
                + learning_rate * raw_vertical_acceleration
            )
            self.velocity_z = 0.0
        else:
            self.velocity_z *= math.exp(
                -self.config.vertical_velocity_damping_per_s * dt
            )
            self.z += self.velocity_z * dt + 0.5 * corrected_acceleration * dt * dt
            self.velocity_z += corrected_acceleration * dt
            self.velocity_z = max(
                -self.config.max_vertical_speed_mps,
                min(self.config.max_vertical_speed_mps, self.velocity_z),
            )
            self.z = max(
                float(self.config.floor_heights[0]),
                min(float(self.config.floor_heights[-1]), self.z),
            )

        self._update_floor()
        if self.stationary:
            floor_height = self.config.floor_heights[self.current_floor]
            if abs(self.z - floor_height) <= self.config.floor_snap_tolerance_m:
                self.z = float(floor_height)
        return self.snapshot()

    def _update_floor(self):
        heights = self.config.floor_heights
        hysteresis = self.config.floor_switch_hysteresis_m
        while self.current_floor + 1 < len(heights):
            boundary = 0.5 * (
                heights[self.current_floor] + heights[self.current_floor + 1]
            )
            if self.z <= boundary + hysteresis:
                break
            self.current_floor += 1
        while self.current_floor > 0:
            boundary = 0.5 * (
                heights[self.current_floor - 1] + heights[self.current_floor]
            )
            if self.z >= boundary - hysteresis:
                break
            self.current_floor -= 1

    def snapshot(self):
        return VerticalState(
            initialized=self.initialized,
            stamp_s=self.last_stamp_s,
            z=self.z,
            velocity_z=self.velocity_z,
            roll=self.roll,
            pitch=self.pitch,
            current_floor=self.current_floor,
            stationary=self.stationary,
        )


def quaternion_to_rpy(quaternion):
    qx, qy, qz, qw = _normalized_quaternion(quaternion)
    sin_roll = 2.0 * (qw * qx + qy * qz)
    cos_roll = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sin_roll, cos_roll)
    sin_pitch = 2.0 * (qw * qy - qz * qx)
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
    sin_yaw = 2.0 * (qw * qz + qx * qy)
    cos_yaw = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(sin_yaw, cos_yaw)
    return roll, pitch, yaw


def quaternion_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quaternion_multiply(left, right):
    lx, ly, lz, lw = _normalized_quaternion(left)
    rx, ry, rz, rw = _normalized_quaternion(right)
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def quaternion_inverse(quaternion):
    qx, qy, qz, qw = _normalized_quaternion(quaternion)
    return -qx, -qy, -qz, qw


def rotate_vector(vector, quaternion):
    """Rotate a three-dimensional vector without applying translation."""
    vx, vy, vz = (float(value) for value in vector)
    qx, qy, qz, qw = _normalized_quaternion(quaternion)
    # Equivalent to q * [v, 0] * inverse(q), expanded to avoid normalizing v.
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def _world_vertical_component(vector, quaternion):
    ax, ay, az = vector
    qx, qy, qz, qw = quaternion
    return (
        2.0 * (qx * qz - qy * qw) * ax
        + 2.0 * (qy * qz + qx * qw) * ay
        + (1.0 - 2.0 * (qx * qx + qy * qy)) * az
    )


def _normalized_quaternion(quaternion):
    qx, qy, qz, qw = (float(value) for value in quaternion)
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        raise ValueError("IMU quaternion has zero norm")
    return qx / norm, qy / norm, qz / norm, qw / norm
