"""ROS-independent mission state, result-path and danger-fusion logic."""

from dataclasses import dataclass
import math
import os


class MissionLifecycle:
    """Small explicit state machine used by the ROS mission manager."""

    IDLE = "IDLE"
    ENTERING = "ENTERING"
    EXPLORING = "EXPLORING"
    RETURNING = "RETURNING"
    FINISHED = "FINISHED"
    ERROR = "ERROR"

    def __init__(self):
        self.state = self.IDLE

    def start(self):
        if self.state not in (self.IDLE, self.FINISHED, self.ERROR):
            return False
        self.state = self.ENTERING
        return True

    def begin_exploration(self):
        if self.state != self.ENTERING:
            return False
        self.state = self.EXPLORING
        return True

    def begin_return(self):
        if self.state not in (self.ENTERING, self.EXPLORING):
            return False
        self.state = self.RETURNING
        return True

    def finish(self):
        if self.state not in (self.ENTERING, self.EXPLORING, self.RETURNING):
            return False
        self.state = self.FINISHED
        return True

    def fail(self):
        self.state = self.ERROR
        return True


@dataclass
class DangerTrack:
    """Running mean for repeated observations of one spatial danger source."""

    x: float
    y: float
    z: float
    floor_id: int
    count: int = 1
    max_confidence: float = 0.0

    def update(self, x, y, z, confidence):
        next_count = self.count + 1
        weight = 1.0 / float(next_count)
        self.x += (x - self.x) * weight
        self.y += (y - self.y) * weight
        self.z += (z - self.z) * weight
        self.count = next_count
        self.max_confidence = max(self.max_confidence, confidence)


class DangerTrackStore:
    """Reject weak samples, merge spatial duplicates and confirm across frames."""

    def __init__(self, dedup_distance_m, min_detections, min_confidence):
        if not math.isfinite(dedup_distance_m) or dedup_distance_m <= 0.0:
            raise ValueError("dedup_distance_m must be positive and finite")
        if int(min_detections) < 1:
            raise ValueError("min_detections must be at least one")
        if not math.isfinite(min_confidence) or not 0.0 <= min_confidence <= 1.0:
            raise ValueError("min_confidence must be in [0, 1]")
        self.dedup_distance_m = float(dedup_distance_m)
        self.min_detections = int(min_detections)
        self.min_confidence = float(min_confidence)
        self.reset()

    def reset(self):
        self.tracks = []
        self.seen_detection_ids = set()

    def add(self, detection_id, x, y, z, floor_id, confidence):
        values = (x, y, z, confidence)
        if not detection_id or detection_id in self.seen_detection_ids:
            return None
        if not all(math.isfinite(float(value)) for value in values):
            return None
        confidence = float(confidence)
        if confidence < self.min_confidence or not 0.0 <= confidence <= 1.0:
            return None
        self.seen_detection_ids.add(detection_id)

        nearest = None
        nearest_distance = float("inf")
        for track in self.tracks:
            if track.floor_id != int(floor_id):
                continue
            distance = math.sqrt(
                (float(x) - track.x) ** 2
                + (float(y) - track.y) ** 2
                + (float(z) - track.z) ** 2
            )
            if distance < self.dedup_distance_m and distance < nearest_distance:
                nearest = track
                nearest_distance = distance

        if nearest is None:
            nearest = DangerTrack(
                x=float(x),
                y=float(y),
                z=float(z),
                floor_id=int(floor_id),
                max_confidence=confidence,
            )
            self.tracks.append(nearest)
        else:
            nearest.update(float(x), float(y), float(z), confidence)
        return nearest

    def confirmed_tracks(self):
        return [
            track for track in self.tracks
            if track.count >= self.min_detections
        ]


def task_relative_position(x, y, z, home_x, home_y, home_z, home_yaw):
    """Express a map point in the task-start frame required by evaluation."""
    values = (x, y, z, home_x, home_y, home_z, home_yaw)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("task-relative transform requires finite values")
    dx = float(x) - float(home_x)
    dy = float(y) - float(home_y)
    cosine = math.cos(float(home_yaw))
    sine = math.sin(float(home_yaw))
    return (
        cosine * dx + sine * dy,
        -sine * dx + cosine * dy,
        float(z) - float(home_z),
    )


def entry_progress(current_x, current_y, home_x, home_y, home_yaw):
    """Return forward and lateral displacement in the captured entry frame."""
    values = (current_x, current_y, home_x, home_y, home_yaw)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("entry progress requires finite values")
    dx = float(current_x) - float(home_x)
    dy = float(current_y) - float(home_y)
    cosine = math.cos(float(home_yaw))
    sine = math.sin(float(home_yaw))
    return (
        cosine * dx + sine * dy,
        -sine * dx + cosine * dy,
    )


def next_entry_target(home_x, home_y, home_yaw, current_progress, distance, step):
    """Build the next center-line target for incremental entrance traversal."""
    values = (home_x, home_y, home_yaw, current_progress, distance, step)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("entry target requires finite values")
    if float(distance) <= 0.0 or float(step) <= 0.0:
        raise ValueError("entry distance and step must be positive")
    target_progress = min(
        float(distance),
        max(0.0, float(current_progress)) + float(step),
    )
    return (
        float(home_x) + target_progress * math.cos(float(home_yaw)),
        float(home_y) + target_progress * math.sin(float(home_yaw)),
        target_progress,
    )


def normalize_result_file(path):
    """Expand and normalize a configured absolute result path."""
    expanded = os.path.expandvars(os.path.expanduser(str(path or "").strip()))
    if not expanded:
        raise ValueError("result_file is empty")
    if not os.path.isabs(expanded):
        raise ValueError("result_file must resolve to an absolute path")
    normalized = os.path.abspath(expanded)
    if os.path.basename(normalized) != "detected_danger.json":
        raise ValueError("result_file must end with detected_danger.json")
    return normalized


def build_result_document(tracks, home, elapsed_s):
    """Build the exact evaluator-facing JSON document."""
    if not math.isfinite(float(elapsed_s)) or float(elapsed_s) < 0.0:
        raise ValueError("elapsed_s must be non-negative and finite")
    if len(home) != 4:
        raise ValueError("home must contain x, y, z and yaw")
    positions = [
        task_relative_position(
            track.x,
            track.y,
            track.z,
            home[0],
            home[1],
            home[2],
            home[3],
        )
        for track in tracks
    ]
    return {
        "exploration_time": round(float(elapsed_s), 2),
        "detected_danger_sources": [
            {"position": [round(x, 2), round(y, 2), round(z, 2)]}
            for x, y, z in positions
        ],
    }
