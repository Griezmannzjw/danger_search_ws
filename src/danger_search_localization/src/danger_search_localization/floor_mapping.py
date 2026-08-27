"""ROS-independent floor classification and per-floor occupancy storage."""

from dataclasses import dataclass
import math

from .occupancy_mapping import OccupancyMapperCore


@dataclass(frozen=True)
class FloorAssignment:
    """Result of assigning a synchronized pose height to a floor."""

    floor_id: int
    floor_height: float
    height_error: float


@dataclass(frozen=True)
class FloorSwitchDecision:
    """Pure result of an idempotent explicit floor-switch request."""

    success: bool
    changed: bool
    current_floor: int
    map_epoch: int
    floor_z_m: float
    message: str


class FloorSwitchState:
    """Track active-floor identity independently of ROS service retries.

    ``transition_id`` is the idempotency key. Replaying a completed or active
    request returns its original epoch; reusing an id for another target is a
    contract error. ``map_epoch`` identifies the active map, whereas each
    floor's ``map_version`` continues to identify occupancy updates within it.
    """

    def __init__(self, floor_heights, initial_floor=0, initial_epoch=1):
        self.floor_heights = tuple(float(value) for value in floor_heights)
        self.current_floor = int(initial_floor)
        self.map_epoch = int(initial_epoch)
        self.transitioning = False
        self._requests = {}
        if not self.floor_heights:
            raise ValueError("floor_heights cannot be empty")
        if self.current_floor < 0 or self.current_floor >= len(self.floor_heights):
            raise ValueError("initial_floor is outside floor_heights")
        if self.map_epoch < 1:
            raise ValueError("initial_epoch must be positive")

    @property
    def floor_z_m(self):
        return self.floor_heights[self.current_floor]

    def replay(self, transition_id, target_floor):
        """Return a prior decision, a conflict failure, or ``None`` if new."""
        transition_id = str(transition_id).strip()
        target_floor = int(target_floor)
        previous = self._requests.get(transition_id)
        if previous is None:
            return None
        previous_target, decision = previous
        if previous_target != target_floor:
            return self._failure(
                "transition_id already belongs to floor %d" % previous_target
            )
        return decision

    def validate(self, transition_id, target_floor):
        transition_id = str(transition_id).strip()
        target_floor = int(target_floor)
        if not transition_id:
            return self._failure("transition_id cannot be empty")
        replay = self.replay(transition_id, target_floor)
        if replay is not None:
            return replay
        if target_floor < 0 or target_floor >= len(self.floor_heights):
            return self._failure("target_floor is outside floor_heights")
        return None

    def request(self, transition_id, target_floor):
        transition_id = str(transition_id).strip()
        target_floor = int(target_floor)
        validation = self.validate(transition_id, target_floor)
        if validation is not None:
            return validation

        changed = target_floor != self.current_floor
        if changed:
            self.current_floor = target_floor
            self.map_epoch += 1
            self.transitioning = True
            message = "active floor changed to %d" % target_floor
        else:
            message = "floor %d is already active" % target_floor
        decision = FloorSwitchDecision(
            success=True,
            changed=changed,
            current_floor=self.current_floor,
            map_epoch=self.map_epoch,
            floor_z_m=self.floor_z_m,
            message=message,
        )
        self._requests[transition_id] = (target_floor, decision)
        return decision

    def force_floor(self, target_floor):
        """Apply a trusted automatic floor observation (test/truth backend)."""
        target_floor = int(target_floor)
        if target_floor < 0 or target_floor >= len(self.floor_heights):
            raise ValueError("target_floor is outside floor_heights")
        changed = target_floor != self.current_floor
        if changed:
            self.current_floor = target_floor
            self.map_epoch += 1
            self.transitioning = True
        return changed

    def reset_map(self):
        self.map_epoch += 1
        self.transitioning = True
        self._requests.clear()
        return self.map_epoch

    def mark_stable(self):
        self.transitioning = False

    def _failure(self, message):
        return FloorSwitchDecision(
            success=False,
            changed=False,
            current_floor=self.current_floor,
            map_epoch=self.map_epoch,
            floor_z_m=self.floor_z_m,
            message=message,
        )


class FloorHeightClassifier:
    """Assign poses near known landings and reject between-floor poses.

    Rejecting transition heights is important: scans collected while an
    elevator or the robot is between landings must not be projected into
    either adjacent two-dimensional map.
    """

    def __init__(self, floor_heights, assignment_tolerance_m=0.45):
        self.floor_heights = tuple(float(value) for value in floor_heights)
        self.assignment_tolerance_m = float(assignment_tolerance_m)
        if not self.floor_heights:
            raise ValueError("floor_heights cannot be empty")
        if not all(math.isfinite(value) for value in self.floor_heights):
            raise ValueError("floor_heights must be finite")
        if any(
            following <= previous
            for previous, following in zip(
                self.floor_heights, self.floor_heights[1:]
            )
        ):
            raise ValueError("floor_heights must be strictly increasing")
        if (
            not math.isfinite(self.assignment_tolerance_m)
            or self.assignment_tolerance_m <= 0.0
        ):
            raise ValueError("assignment_tolerance_m must be positive")
        if len(self.floor_heights) > 1:
            minimum_gap = min(
                following - previous
                for previous, following in zip(
                    self.floor_heights, self.floor_heights[1:]
                )
            )
            if 2.0 * self.assignment_tolerance_m >= minimum_gap:
                raise ValueError(
                    "assignment_tolerance_m must leave a between-floor gap"
                )

    def classify(self, relative_height_m):
        height = float(relative_height_m)
        if not math.isfinite(height):
            return None
        floor_id = min(
            range(len(self.floor_heights)),
            key=lambda index: abs(height - self.floor_heights[index]),
        )
        floor_height = self.floor_heights[floor_id]
        error = abs(height - floor_height)
        if error > self.assignment_tolerance_m:
            return None
        return FloorAssignment(floor_id, floor_height, error)


class MultiFloorOccupancyStore:
    """Keep one independent occupancy mapper and version per visited floor."""

    def __init__(self, mapping_config, valid_floor_ids, initial_floor=0):
        self.mapping_config = mapping_config
        self.valid_floor_ids = tuple(
            sorted({int(value) for value in valid_floor_ids})
        )
        self.initial_floor = int(initial_floor)
        if not self.valid_floor_ids:
            raise ValueError("valid_floor_ids cannot be empty")
        if self.valid_floor_ids[0] < 0:
            raise ValueError("floor ids cannot be negative")
        if self.initial_floor not in self.valid_floor_ids:
            raise ValueError("initial_floor is not a valid floor")
        self._cores = {}
        self._ensure_floor(self.initial_floor)

    @property
    def floor_ids(self):
        return tuple(sorted(self._cores))

    def has_floor(self, floor_id):
        return int(floor_id) in self._cores

    def core(self, floor_id):
        floor_id = self._validate_floor(floor_id)
        return self._ensure_floor(floor_id)

    def version(self, floor_id):
        return int(self.core(floor_id).update_count)

    def update(self, floor_id, pose, scan):
        return self.core(floor_id).update(pose, scan)

    def reset_all(self):
        known_floors = self.floor_ids or (self.initial_floor,)
        self._cores = {
            floor_id: OccupancyMapperCore(self.mapping_config)
            for floor_id in known_floors
        }

    def _validate_floor(self, floor_id):
        floor_id = int(floor_id)
        if floor_id not in self.valid_floor_ids:
            raise ValueError(
                "floor %d is outside configured floor_heights" % floor_id
            )
        return floor_id

    def _ensure_floor(self, floor_id):
        floor_id = self._validate_floor(floor_id)
        core = self._cores.get(floor_id)
        if core is None:
            core = OccupancyMapperCore(self.mapping_config)
            self._cores[floor_id] = core
        return core
