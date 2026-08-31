"""ROS-independent, floor-aware tracking for static danger sources."""

from dataclasses import dataclass
import math
import threading


@dataclass(frozen=True)
class DetectionObservation:
    detection_id: str
    floor_id: int
    position: tuple
    confidence: float
    stamp_s: float
    # A map epoch is a coordinate-system identity, not just a monotonic
    # timestamp.  A floor can be reloaded or reset while retaining its floor
    # id, so association must never smooth points from two map instances.
    map_epoch: int = 0


@dataclass(frozen=True)
class TrackAssignment:
    track_id: str
    position: tuple
    position_covariance: tuple
    confirmed: bool
    hit_count: int
    possible_duplicate_track_ids: tuple


class _Track:
    def __init__(self, track_id, observation):
        self.track_id = str(track_id)
        self.floor_id = int(observation.floor_id)
        self.map_epoch = int(observation.map_epoch)
        self.mean = [float(value) for value in observation.position]
        self.m2 = [0.0, 0.0, 0.0]
        self.hit_count = 1
        self.first_seen_s = float(observation.stamp_s)
        self.last_seen_s = float(observation.stamp_s)
        self.max_confidence = float(observation.confidence)

    def update(self, observation):
        self.hit_count += 1
        for axis, value in enumerate(observation.position):
            value = float(value)
            delta = value - self.mean[axis]
            self.mean[axis] += delta / float(self.hit_count)
            delta_after = value - self.mean[axis]
            self.m2[axis] += delta * delta_after
        self.last_seen_s = max(self.last_seen_s, float(observation.stamp_s))
        self.max_confidence = max(
            self.max_confidence, float(observation.confidence)
        )

    def distance_to(self, observation):
        return math.sqrt(
            sum(
                (float(value) - self.mean[axis]) ** 2
                for axis, value in enumerate(observation.position)
            )
        )

    def covariance(self, config):
        diagonal = []
        for m2 in self.m2:
            sample_variance = (
                m2 / float(self.hit_count - 1)
                if self.hit_count > 1 else 0.0
            )
            mean_variance = sample_variance / float(self.hit_count)
            prior_variance = (
                config.initial_position_variance_m2
                / float(self.hit_count)
            )
            diagonal.append(
                max(
                    config.minimum_position_variance_m2,
                    mean_variance,
                    prior_variance,
                )
            )
        return (
            diagonal[0], 0.0, 0.0,
            0.0, diagonal[1], 0.0,
            0.0, 0.0, diagonal[2],
        )


class MultiFrameDangerTracker:
    """Associate repeated 3-D observations without mixing floors.

    Only detections present in the current sensor frame are returned.  A track
    therefore supplies identity and a filtered position but never fabricates a
    detection during an occlusion.  Confirmed tracks can be retained for the
    entire mission so returning to a previously visited floor keeps identity.
    """

    def __init__(self, config):
        self.config = config
        self._lock = threading.RLock()
        self._tracks = {}
        self._next_sequence = 1

    def update(self, observations):
        observations = tuple(observations)
        if not observations:
            return []
        for observation in observations:
            self._validate_observation(observation)
        now_s = max(float(item.stamp_s) for item in observations)

        with self._lock:
            self._prune_unlocked(now_s)
            assignments = [None] * len(observations)
            available_tracks = tuple(self._tracks.values())
            candidate_pairs = []
            candidate_ids = [set() for _ in observations]
            for observation_index, observation in enumerate(observations):
                for track in available_tracks:
                    if (
                        track.floor_id != int(observation.floor_id)
                        or track.map_epoch != int(observation.map_epoch)
                    ):
                        continue
                    distance = track.distance_to(observation)
                    if distance <= self.config.association_distance_m:
                        candidate_pairs.append(
                            (distance, observation_index, track.track_id)
                        )
                        candidate_ids[observation_index].add(track.track_id)

            used_observations = set()
            used_tracks = set()
            for _distance, observation_index, track_id in sorted(
                candidate_pairs, key=lambda item: (item[0], item[1], item[2])
            ):
                if (
                    observation_index in used_observations
                    or track_id in used_tracks
                ):
                    continue
                track = self._tracks[track_id]
                track.update(observations[observation_index])
                used_observations.add(observation_index)
                used_tracks.add(track_id)
                assignments[observation_index] = self._assignment(
                    track,
                    candidate_ids[observation_index] - {track_id},
                )

            for observation_index, observation in enumerate(observations):
                if assignments[observation_index] is not None:
                    continue
                track = self._new_track(observation)
                assignments[observation_index] = self._assignment(track, ())

            return assignments

    def counts(self, now_s):
        now_s = float(now_s)
        if not math.isfinite(now_s):
            raise ValueError("now_s must be finite")
        with self._lock:
            self._prune_unlocked(now_s)
            confirmed = sum(
                self._is_confirmed(track) for track in self._tracks.values()
            )
            return confirmed, len(self._tracks) - confirmed

    def reset(self):
        with self._lock:
            self._tracks.clear()
            self._next_sequence = 1

    def _new_track(self, observation):
        track_id = "danger-f{}-e{}-{:04d}".format(
            int(observation.floor_id),
            int(observation.map_epoch),
            self._next_sequence,
        )
        self._next_sequence += 1
        track = _Track(track_id, observation)
        self._tracks[track_id] = track
        return track

    def _assignment(self, track, possible_duplicates):
        return TrackAssignment(
            track_id=track.track_id,
            position=tuple(track.mean),
            position_covariance=track.covariance(self.config),
            confirmed=self._is_confirmed(track),
            hit_count=track.hit_count,
            possible_duplicate_track_ids=tuple(sorted(possible_duplicates)),
        )

    def _is_confirmed(self, track):
        return track.hit_count >= int(self.config.confirmation_hits)

    def _prune_unlocked(self, now_s):
        retained = {}
        for track_id, track in self._tracks.items():
            age_s = max(0.0, float(now_s) - track.last_seen_s)
            if self._is_confirmed(track):
                timeout_s = float(self.config.confirmed_timeout_s)
                if timeout_s <= 0.0 or age_s <= timeout_s:
                    retained[track_id] = track
            elif age_s <= float(self.config.tentative_timeout_s):
                retained[track_id] = track
        self._tracks = retained

    @staticmethod
    def _validate_observation(observation):
        if not observation.detection_id:
            raise ValueError("detection_id cannot be empty")
        if int(observation.floor_id) < 0:
            raise ValueError("floor_id cannot be negative")
        if int(observation.map_epoch) < 0:
            raise ValueError("map_epoch cannot be negative")
        if len(observation.position) != 3:
            raise ValueError("position must contain x, y and z")
        values = tuple(observation.position) + (
            observation.confidence,
            observation.stamp_s,
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("observation values must be finite")
        if not 0.0 <= float(observation.confidence) <= 1.0:
            raise ValueError("confidence must be in [0, 1]")
