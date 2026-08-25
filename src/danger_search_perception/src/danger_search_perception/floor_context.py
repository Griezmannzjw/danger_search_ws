"""Thread-safe mapping gate and ROS-independent floor-height validation."""

from dataclasses import dataclass
import math
import threading


@dataclass(frozen=True)
class MappingSnapshot:
    """Semantic mapping state captured for one synchronized sensor frame."""

    epoch: int
    floor_id: int
    allowed: bool
    reason: str


class MappingGate:
    """Reject observations while the active floor map is unavailable.

    Periodic MappingStatus refreshes do not change ``epoch``.  A semantic
    change (floor, ready/stable/lost or validity) does, which lets a sensor
    callback detect that its floor context changed while it was processing.
    """

    def __init__(self, status_timeout_s, fallback_floor_id=0):
        if not math.isfinite(float(status_timeout_s)) or status_timeout_s <= 0.0:
            raise ValueError("status_timeout_s must be positive and finite")
        if int(fallback_floor_id) < 0:
            raise ValueError("fallback_floor_id cannot be negative")
        self.status_timeout_s = float(status_timeout_s)
        self._fallback_floor_id = int(fallback_floor_id)
        self._lock = threading.RLock()
        self._seen = False
        self._ready = False
        self._stable = False
        self._lost = True
        self._floor_id = self._fallback_floor_id
        self._valid_floor = True
        self._received_s = float("-inf")
        self._epoch = 0

    def update(self, ready, stable, lost, floor_id, received_s):
        received_s = float(received_s)
        floor_id = int(floor_id)
        if not math.isfinite(received_s):
            raise ValueError("received_s must be finite")
        semantic = (
            bool(ready),
            bool(stable),
            bool(lost),
            floor_id,
            floor_id >= 0,
        )
        with self._lock:
            previous = (
                self._ready,
                self._stable,
                self._lost,
                self._floor_id,
                self._valid_floor,
            )
            self._seen = True
            self._ready = semantic[0]
            self._stable = semantic[1]
            self._lost = semantic[2]
            self._floor_id = floor_id
            self._valid_floor = semantic[4]
            self._received_s = received_s
            if semantic != previous:
                self._epoch += 1

    def snapshot(self, now_s, required=True):
        now_s = float(now_s)
        if not math.isfinite(now_s):
            raise ValueError("now_s must be finite")
        with self._lock:
            if not required:
                floor_id = (
                    self._floor_id if self._valid_floor
                    else self._fallback_floor_id
                )
                return MappingSnapshot(self._epoch, floor_id, True, "OK")
            return self._snapshot_required(now_s)

    def is_current(self, snapshot, now_s, required=True):
        current = self.snapshot(now_s, required=required)
        return (
            current.allowed
            and current.epoch == snapshot.epoch
            and current.floor_id == snapshot.floor_id
        )

    def _snapshot_required(self, now_s):
        if not self._seen:
            return MappingSnapshot(
                self._epoch,
                self._fallback_floor_id,
                False,
                "WAITING_FOR_MAPPING_STATUS",
            )
        if now_s - self._received_s > self.status_timeout_s:
            return MappingSnapshot(
                self._epoch,
                max(self._floor_id, self._fallback_floor_id),
                False,
                "MAPPING_STATUS_STALE",
            )
        if not self._valid_floor:
            return MappingSnapshot(
                self._epoch,
                self._fallback_floor_id,
                False,
                "MAPPING_FLOOR_INVALID",
            )
        if self._lost:
            return MappingSnapshot(
                self._epoch, self._floor_id, False, "MAPPING_LOST"
            )
        if not self._ready:
            return MappingSnapshot(
                self._epoch, self._floor_id, False, "MAPPING_NOT_READY"
            )
        if not self._stable:
            return MappingSnapshot(
                self._epoch, self._floor_id, False, "MAPPING_UNSTABLE"
            )
        return MappingSnapshot(self._epoch, self._floor_id, True, "OK")


class FloorHeightClassifier:
    """Classify a mission-relative base height into a configured floor."""

    def __init__(self, floor_heights, tolerance_m):
        self.floor_heights = tuple(float(value) for value in floor_heights)
        self.tolerance_m = float(tolerance_m)
        if not self.floor_heights:
            raise ValueError("floor_heights cannot be empty")
        if not all(math.isfinite(value) for value in self.floor_heights):
            raise ValueError("floor_heights must be finite")
        if any(
            upper <= lower
            for lower, upper in zip(
                self.floor_heights, self.floor_heights[1:]
            )
        ):
            raise ValueError("floor_heights must be strictly increasing")
        if not math.isfinite(self.tolerance_m) or self.tolerance_m <= 0.0:
            raise ValueError("tolerance_m must be positive and finite")
        if len(self.floor_heights) > 1:
            minimum_gap = min(
                upper - lower
                for lower, upper in zip(
                    self.floor_heights, self.floor_heights[1:]
                )
            )
            if 2.0 * self.tolerance_m >= minimum_gap:
                raise ValueError(
                    "tolerance_m must leave an unclassified gap between floors"
                )

    def classify(self, height_m):
        height_m = float(height_m)
        if not math.isfinite(height_m):
            return None
        nearest = min(
            range(len(self.floor_heights)),
            key=lambda index: abs(height_m - self.floor_heights[index]),
        )
        if abs(height_m - self.floor_heights[nearest]) > self.tolerance_m:
            return None
        return nearest

    def matches(self, floor_id, height_m):
        return self.classify(height_m) == int(floor_id)
