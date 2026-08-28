"""ROS-independent mission state, result-path and danger-fusion logic."""

from dataclasses import dataclass
import math
import os


DEFAULT_ENTRY_COMPLETION_TOLERANCE_M = 0.45


class PostureSafetyGate:
    """Classify posture stops without coupling sensor faults to true falls."""

    def __init__(self, recoverable_abort_s):
        recoverable_abort_s = float(recoverable_abort_s)
        if not math.isfinite(recoverable_abort_s) or recoverable_abort_s <= 0.0:
            raise ValueError("recoverable safety abort must be positive and finite")
        self.recoverable_abort_s = recoverable_abort_s
        self.active = False
        self.fallen = False
        self.reason = "unknown"
        self.active_since = None

    def update_stop(self, active, now):
        active = bool(active)
        now = float(now)
        if active and not self.active:
            self.active_since = now
        elif not active:
            self.active_since = None
        self.active = active

    def update_fallen(self, fallen):
        self.fallen = bool(fallen)

    def update_reason(self, reason):
        normalized = str(reason or "").strip()
        if normalized:
            self.reason = normalized

    def abort_detail(self, now, mission_active):
        """Return a stable abort detail, or ``None`` while recovery is allowed."""
        if not mission_active:
            return None
        if self.fallen:
            return "posture_fallen"
        if not self.active or self.active_since is None:
            return None
        elapsed = float(now) - self.active_since
        if elapsed < self.recoverable_abort_s:
            return None
        reason = self.reason.replace(":", "_").replace(" ", "_")
        return "persistent_" + reason


def allocate_return_attempt_budget(
        remaining_s, attempt_cap_s, retry_reserve_s,
        terminal_reserve_s, attempts_left):
    """Allocate one home-goal attempt without consuming the terminal budget.

    ``attempts_left`` is the number of retries still available *after* the
    attempt being allocated.  A first attempt therefore cannot occupy the
    complete return window and starve a short, useful retry near home.
    """
    values = (
        remaining_s,
        attempt_cap_s,
        retry_reserve_s,
        terminal_reserve_s,
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("return budget values must be finite")
    if float(remaining_s) < 0.0:
        raise ValueError("remaining return budget cannot be negative")
    if float(attempt_cap_s) <= 0.0:
        raise ValueError("return attempt cap must be positive")
    if float(retry_reserve_s) < 0.0 or float(terminal_reserve_s) < 0.0:
        raise ValueError("return reserves cannot be negative")
    if int(attempts_left) < 0:
        raise ValueError("attempts_left cannot be negative")
    reserve = float(terminal_reserve_s)
    if int(attempts_left) > 0:
        reserve += float(retry_reserve_s)
    return max(
        0.0,
        min(float(attempt_cap_s), float(remaining_s) - reserve),
    )


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


def task_to_world_position(x, y, z, start_x, start_y, start_z, start_yaw):
    """Transform a task-start-relative point into the public world frame."""
    values = (x, y, z, start_x, start_y, start_z, start_yaw)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("world transform requires finite values")
    cosine = math.cos(float(start_yaw))
    sine = math.sin(float(start_yaw))
    return (
        float(start_x) + cosine * float(x) - sine * float(y),
        float(start_y) + sine * float(x) + cosine * float(y),
        float(start_z) + float(z),
    )


def resolve_result_coordinate_frame(requested, scene_frame=None):
    """Resolve the evaluator-facing frame without consulting forbidden files."""
    requested = str(requested or "auto").strip().lower()
    if requested not in ("auto", "world", "start_relative"):
        raise ValueError("result_coordinate_frame must be auto, world or start_relative")
    if requested != "auto":
        return requested
    normalized_scene = str(scene_frame or "").strip().lower()
    return "world" if normalized_scene == "world" else "start_relative"


def parse_public_scene_contract(document):
    """Extract only the explicitly public fields used by the mission runtime."""
    if not isinstance(document, dict):
        raise ValueError("team scene info must be a JSON object")
    if document.get("schema") != "team_scene_info_v1":
        raise ValueError("unsupported team scene info schema")
    coordinate_frame = str(document.get("coordinate_frame", "")).strip().lower()
    if coordinate_frame not in ("", "world", "start_relative"):
        raise ValueError("unsupported public coordinate frame")
    start = document.get("robot_start")
    if not isinstance(start, dict):
        raise ValueError("team scene info is missing robot_start")
    values = tuple(float(start[name]) for name in ("x", "y", "z", "yaw"))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("robot_start must contain finite x/y/z/yaw")
    return {
        "coordinate_frame": coordinate_frame,
        "robot_start": values,
        "public_scene": document.get("public_scene", {}),
    }


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


RUN_PROFILE_FORMAL = "formal"
RUN_PROFILE_SIMULATION_TRUTH = "simulation_truth"
RESULT_FILENAME_BY_RUN_PROFILE = {
    RUN_PROFILE_FORMAL: "detected_danger.json",
    RUN_PROFILE_SIMULATION_TRUTH: "detected_danger.simulation_truth.json",
}


def normalize_run_profile(run_profile):
    """Return one of the explicitly supported runtime profiles."""
    normalized = str(run_profile or RUN_PROFILE_FORMAL).strip().lower()
    if normalized not in RESULT_FILENAME_BY_RUN_PROFILE:
        raise ValueError("run_profile must be formal or simulation_truth")
    return normalized


def official_eligibility(run_profile):
    """Only the fail-closed formal profile can produce an official result."""
    return normalize_run_profile(run_profile) == RUN_PROFILE_FORMAL


def result_profile_errors(document, official=False):
    """Return profile/terminal-state violations for an evaluator result."""
    if not isinstance(document, dict):
        return ["result document must be a JSON object"]

    errors = []
    mission_status = document.get("mission_status")
    localization_backend = document.get("localization_backend")
    eligible = document.get("official_eligible")
    raw_run_profile = document.get("run_profile")
    if not isinstance(raw_run_profile, str) or not raw_run_profile.strip():
        run_profile = None
        errors.append("run_profile must be a non-empty string")
    else:
        try:
            run_profile = normalize_run_profile(raw_run_profile)
        except ValueError as exc:
            run_profile = None
            errors.append(str(exc))

    if not isinstance(mission_status, str) or not mission_status.strip():
        errors.append("mission_status must be a non-empty string")
    if (not isinstance(localization_backend, str)
            or not localization_backend.strip()):
        errors.append("localization_backend must be a non-empty string")
    if not isinstance(eligible, bool):
        errors.append("official_eligible must be a boolean")
    elif run_profile is not None and eligible != official_eligibility(run_profile):
        errors.append("official_eligible is inconsistent with run_profile")

    if official:
        if mission_status != "FINISHED":
            errors.append("official result requires mission_status=FINISHED")
        if run_profile != RUN_PROFILE_FORMAL:
            errors.append("official result requires run_profile=formal")
        if localization_backend != "gicp":
            errors.append("official result requires localization_backend=gicp")
        if eligible is not True:
            errors.append("official result requires official_eligible=true")
    return errors


def normalize_result_file(path, run_profile=RUN_PROFILE_FORMAL):
    """Expand and normalize the profile-specific absolute result path."""
    expanded = os.path.expandvars(os.path.expanduser(str(path or "").strip()))
    if not expanded:
        raise ValueError("result_file is empty")
    if not os.path.isabs(expanded):
        raise ValueError("result_file must resolve to an absolute path")
    normalized = os.path.abspath(expanded)
    expected_basename = RESULT_FILENAME_BY_RUN_PROFILE[
        normalize_run_profile(run_profile)
    ]
    if os.path.basename(normalized) != expected_basename:
        raise ValueError("result_file must end with %s" % expected_basename)
    return normalized


def build_result_document(
    tracks,
    home,
    elapsed_s,
    coordinate_frame="start_relative",
    robot_start=None,
    mission_status="FINISHED",
    run_profile=RUN_PROFILE_FORMAL,
    localization_backend="gicp",
    finish_reason="",
):
    """Build the exact evaluator-facing JSON document."""
    if not math.isfinite(float(elapsed_s)) or float(elapsed_s) < 0.0:
        raise ValueError("elapsed_s must be non-negative and finite")
    if len(home) != 4:
        raise ValueError("home must contain x, y, z and yaw")
    coordinate_frame = resolve_result_coordinate_frame(coordinate_frame)
    run_profile = normalize_run_profile(run_profile)
    relative_positions = [
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
    if coordinate_frame == "world":
        if robot_start is None or len(robot_start) != 4:
            raise ValueError("world output requires public robot_start x/y/z/yaw")
        positions = [
            task_to_world_position(*position, *robot_start)
            for position in relative_positions
        ]
    else:
        positions = relative_positions
    return {
        "exploration_time": round(float(elapsed_s), 2),
        "coordinate_frame": coordinate_frame,
        "mission_status": str(mission_status),
        "finish_reason": str(finish_reason),
        "run_profile": run_profile,
        "localization_backend": str(localization_backend),
        "official_eligible": official_eligibility(run_profile),
        "detected_danger_sources": [
            {"position": [round(x, 2), round(y, 2), round(z, 2)]}
            for x, y, z in positions
        ],
    }
