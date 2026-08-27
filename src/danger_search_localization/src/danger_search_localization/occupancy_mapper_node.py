"""ROS occupancy mapper using synchronized trusted GICP poses and scans."""

import copy
import math
import threading

import rospy
from danger_search_common.msg import FloorOccupancyGrid
from danger_search_common.srv import SwitchFloor, SwitchFloorResponse
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty, EmptyResponse

from .occupancy_mapping import OccupancyMapperCore, OccupancyMappingConfig
from .floor_mapping import (
    FloorHeightClassifier,
    FloorSwitchState,
    MultiFloorOccupancyStore,
)


class OccupancyMapperNode:
    def __init__(self):
        rospy.init_node("local_occupancy_mapper", anonymous=False)
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base")
        self.scan_topic = rospy.get_param(
            "~mapping_scan_topic", "/localization/mapping_scan"
        )
        self.pose_topic = rospy.get_param(
            "~validated_gicp_pose_topic", "/localization/validated_pose"
        )
        self.map_topic = rospy.get_param(
            "~raw_map_topic", "/localization/raw_map"
        )
        self.raw_floor_map_topic = rospy.get_param(
            "~raw_floor_map_topic", "/localization/raw_floor_map"
        )
        self.floor_map_topic_prefix = rospy.get_param(
            "~floor_map_topic_prefix", "/mapping/floors"
        ).rstrip("/")
        self.multifloor_enabled = bool(
            rospy.get_param("~multifloor_enabled", False)
        )
        self.localization_backend = rospy.get_param(
            "~localization_backend", "gicp"
        )
        self.explicit_floor_switching = bool(rospy.get_param(
            "~explicit_floor_switching",
            self.multifloor_enabled and self.localization_backend == "gicp",
        ))
        self.reset_map_service = rospy.get_param(
            "~reset_map_service", "/localization/reset_map"
        )
        self.switch_floor_service = rospy.get_param(
            "~mapper_switch_floor_service", "/localization/mapper_switch_floor"
        )
        self.unhealthy_variance = float(
            rospy.get_param("~gicp_unhealthy_variance_threshold", 1.0)
        )
        self.publish_period = float(rospy.get_param("~map_pub_period", 1.0))
        self.config = OccupancyMappingConfig(
            resolution=float(rospy.get_param("~map_resolution", 0.05)),
            size=int(rospy.get_param("~map_size", 1024)),
            start_x=float(rospy.get_param("~map_start_x", 0.5)),
            start_y=float(rospy.get_param("~map_start_y", 0.5)),
            max_rays=int(rospy.get_param("~occupancy_mapper_max_rays", 360)),
            free_update=int(rospy.get_param("~occupancy_mapper_free_update", 1)),
            occupied_update=int(
                rospy.get_param("~occupancy_mapper_occupied_update", 4)
            ),
            min_score=int(rospy.get_param("~occupancy_mapper_min_score", -20)),
            max_score=int(rospy.get_param("~occupancy_mapper_max_score", 20)),
            occupied_score=int(
                rospy.get_param("~occupancy_mapper_occupied_score", 2)
            ),
            clear_radius_m=float(
                rospy.get_param("~occupancy_mapper_clear_radius_m", 0.35)
            ),
        )
        if self.publish_period <= 0.0:
            raise ValueError("map publication period must be positive")
        self.initial_floor = int(rospy.get_param("~current_floor", 0))
        self.floor_heights = tuple(
            float(value) for value in rospy.get_param(
                "~floor_heights", [0.0, 2.6, 5.2]
            )
        )
        self.floor_classifier = FloorHeightClassifier(
            self.floor_heights,
            float(rospy.get_param("~floor_map_assignment_tolerance_m", 0.45)),
        )
        self.floor_store = MultiFloorOccupancyStore(
            self.config,
            range(len(self.floor_heights)),
            initial_floor=self.initial_floor,
        )
        self.floor_switch = FloorSwitchState(
            self.floor_heights, initial_floor=self.initial_floor
        )
        self.current_floor = self.initial_floor
        self.core = self.floor_store.core(self.current_floor)
        self.lock = threading.RLock()
        self.pose_cache = {}
        self.scan_cache = {}
        self.last_scan_stamp = rospy.Time(0)
        self.map_dirty = False
        self.map_load_time = rospy.Time.now()
        self.map_load_times = {self.current_floor: self.map_load_time}
        self.last_scan_stamps = {self.current_floor: self.last_scan_stamp}
        self.latest_floor_key = None

        self.publisher = rospy.Publisher(
            self.map_topic, OccupancyGrid, queue_size=1, latch=True
        )
        self.floor_publisher = None
        self.floor_publishers = {}
        if self.multifloor_enabled:
            self.floor_publisher = rospy.Publisher(
                self.raw_floor_map_topic,
                FloorOccupancyGrid,
                queue_size=1,
                latch=True,
            )
            self.floor_publishers = {
                floor_id: rospy.Publisher(
                    "%s/%d/map" % (self.floor_map_topic_prefix, floor_id),
                    OccupancyGrid,
                    queue_size=1,
                    latch=True,
                )
                for floor_id in range(len(self.floor_heights))
            }
        self.pose_subscriber = rospy.Subscriber(
            self.pose_topic,
            PoseWithCovarianceStamped,
            self._pose_callback,
            queue_size=20,
        )
        self.scan_subscriber = rospy.Subscriber(
            self.scan_topic, LaserScan, self._scan_callback, queue_size=10
        )
        self.reset_service = rospy.Service(
            self.reset_map_service, Empty, self._reset_map_callback
        )
        self.floor_switch_service = rospy.Service(
            self.switch_floor_service,
            SwitchFloor,
            self._switch_floor_callback,
        )
        self.timer = rospy.Timer(
            rospy.Duration(self.publish_period), self._publish_map
        )
        rospy.loginfo(
            "[localization] canonical occupancy mapper: %s + %s -> %s "
            "(multifloor=%s)",
            self.pose_topic,
            self.scan_topic,
            self.map_topic,
            self.multifloor_enabled,
        )

    @staticmethod
    def _key(stamp):
        return int(stamp.secs), int(stamp.nsecs)

    def _pose_callback(self, message):
        if message.header.frame_id != self.odom_frame:
            return
        covariance = message.pose.covariance
        if not all(
            math.isfinite(covariance[index])
            and covariance[index] < self.unhealthy_variance
            for index in (0, 7, 35)
        ):
            return
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0
            * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0
            - 2.0
            * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )
        position = message.pose.pose.position
        pose = (float(position.x), float(position.y), float(yaw))
        height = float(position.z)
        if not all(math.isfinite(value) for value in (*pose, height)):
            return
        if self.multifloor_enabled and self.explicit_floor_switching:
            floor_id = self.current_floor
        else:
            assignment = (
                self.floor_classifier.classify(height)
                if self.multifloor_enabled
                else None
            )
            floor_id = (
                assignment.floor_id
                if assignment is not None
                else (None if self.multifloor_enabled else self.initial_floor)
            )
        with self.lock:
            self.pose_cache[self._key(message.header.stamp)] = (pose, floor_id)
            self._consume(self._key(message.header.stamp))
            self._prune_caches()

    def _scan_callback(self, message):
        if message.header.frame_id != self.base_frame:
            return
        with self.lock:
            self.scan_cache[self._key(message.header.stamp)] = copy.deepcopy(message)
            self._consume(self._key(message.header.stamp))
            self._prune_caches()

    def _consume(self, key):
        if key not in self.pose_cache or key not in self.scan_cache:
            return
        pose, floor_id = self.pose_cache.pop(key)
        scan = self.scan_cache.pop(key)
        if floor_id is None:
            rospy.loginfo_throttle(
                1.0,
                "[localization] dropping mapping scan while between floors",
            )
            return
        self._ensure_floor_runtime(floor_id)
        try:
            updated = self.floor_store.update(floor_id, pose, scan)
        except ValueError as exc:
            rospy.logwarn_throttle(
                1.0, "[localization] occupancy update rejected: %s", str(exc)
            )
            updated = False
        if updated:
            self.last_scan_stamps[floor_id] = scan.header.stamp
            if self.latest_floor_key is None or key >= self.latest_floor_key:
                if floor_id != self.current_floor:
                    rospy.loginfo(
                        "[localization] occupancy map switched floor %d -> %d",
                        self.current_floor,
                        floor_id,
                    )
                    self.floor_switch.force_floor(floor_id)
                self.current_floor = floor_id
                self.latest_floor_key = key
                self._sync_current_floor_compatibility()
            self.map_dirty = True

    def _prune_caches(self):
        for cache in (self.pose_cache, self.scan_cache):
            if len(cache) <= 50:
                continue
            for key in sorted(cache)[:-50]:
                cache.pop(key, None)

    def _publish_map(self, _event=None):
        with self.lock:
            if self.core.update_count == 0:
                return
            floor_id = self.current_floor
            message = self._map_message_locked(
                self.last_scan_stamps[floor_id], floor_id
            )
            version = self.floor_store.version(floor_id)
            self.map_dirty = False
        self._publish_floor_messages(floor_id, version, message)

    def _reset_map_callback(self, _request):
        if getattr(self, "multifloor_enabled", False):
            return self._reset_all_floor_maps()
        now = rospy.Time.now()
        with self.lock:
            if hasattr(self, "floor_switch"):
                self.floor_switch.reset_map()
            if now <= self.map_load_time:
                now = self.map_load_time + rospy.Duration.from_sec(1e-9)
            self.core = OccupancyMapperCore(self.config)
            self.pose_cache.clear()
            self.scan_cache.clear()
            self.last_scan_stamp = now
            self.map_load_time = now
            self.map_dirty = False
            message = self._map_message_locked(now)
        self.publisher.publish(message)
        rospy.logwarn("[localization] occupancy map reset by service request")
        return EmptyResponse()

    def _reset_all_floor_maps(self):
        now = rospy.Time.now()
        with self.lock:
            if hasattr(self, "floor_switch"):
                self.floor_switch.reset_map()
            known_floors = self.floor_store.floor_ids
            latest_load_time = max(
                self.map_load_times.values(), default=rospy.Time(0)
            )
            if now <= latest_load_time:
                now = latest_load_time + rospy.Duration.from_sec(1e-9)
            self.floor_store.reset_all()
            self.pose_cache.clear()
            self.scan_cache.clear()
            self.map_load_times = {floor_id: now for floor_id in known_floors}
            self.last_scan_stamps = {floor_id: now for floor_id in known_floors}
            self.latest_floor_key = None
            self.map_dirty = False
            self._sync_current_floor_compatibility()
            messages = {
                floor_id: self._map_message_locked(now, floor_id)
                for floor_id in known_floors
            }
        # Publish every reset epoch through the atomic envelope so the adapter
        # also clears metadata for floors that are not currently selected.
        for floor_id, message in messages.items():
            archive_publisher = self.floor_publishers.get(floor_id)
            if archive_publisher is not None:
                archive_publisher.publish(message)
            self._publish_floor_envelope(floor_id, 0, message)
        current_message = messages[self.current_floor]
        self._publish_floor_messages(self.current_floor, 0, current_message)
        rospy.logwarn("[localization] all per-floor occupancy maps reset")
        return EmptyResponse()

    def _switch_floor_callback(self, request):
        """Atomically select a per-floor store with retry-safe semantics."""
        publish = None
        with self.lock:
            if not self.multifloor_enabled:
                return SwitchFloorResponse(
                    success=False,
                    map_epoch=self.floor_switch.map_epoch,
                    message="multifloor mapping is disabled",
                )
            decision = self.floor_switch.request(
                request.transition_id, request.target_floor
            )
            if not decision.success:
                return SwitchFloorResponse(
                    success=False,
                    map_epoch=decision.map_epoch,
                    message=decision.message,
                )
            if decision.changed:
                previous_floor = self.current_floor
                self.current_floor = decision.current_floor
                self.pose_cache.clear()
                self.scan_cache.clear()
                self.latest_floor_key = None
                self._sync_current_floor_compatibility()
                stamp = self.last_scan_stamps[self.current_floor]
                publish = (
                    self.current_floor,
                    self.floor_store.version(self.current_floor),
                    self._map_message_locked(stamp, self.current_floor),
                )
                rospy.loginfo(
                    "[localization] explicit occupancy map switch %d -> %d "
                    "(transition=%s epoch=%d)",
                    previous_floor,
                    self.current_floor,
                    request.transition_id,
                    decision.map_epoch,
                )
            elif self.floor_store.has_floor(self.current_floor):
                stamp = self.last_scan_stamps[self.current_floor]
                publish = (
                    self.current_floor,
                    self.floor_store.version(self.current_floor),
                    self._map_message_locked(stamp, self.current_floor),
                )
        if publish is not None:
            self._publish_floor_messages(*publish)
        return SwitchFloorResponse(
            success=True,
            map_epoch=decision.map_epoch,
            message=decision.message,
        )

    def _ensure_floor_runtime(self, floor_id):
        self.floor_store.core(floor_id)
        if floor_id not in self.map_load_times:
            self.map_load_times[floor_id] = rospy.Time.now()
        self.last_scan_stamps.setdefault(floor_id, rospy.Time(0))

    def _sync_current_floor_compatibility(self):
        self._ensure_floor_runtime(self.current_floor)
        self.core = self.floor_store.core(self.current_floor)
        self.map_load_time = self.map_load_times[self.current_floor]
        self.last_scan_stamp = self.last_scan_stamps[self.current_floor]

    def _publish_floor_messages(self, floor_id, version, message):
        self.publisher.publish(message)
        if not getattr(self, "multifloor_enabled", False):
            return
        self._publish_floor_envelope(floor_id, version, message)
        self.floor_publishers[floor_id].publish(message)

    def _publish_floor_envelope(self, floor_id, version, message):
        envelope = FloorOccupancyGrid()
        envelope.header = message.header
        envelope.floor_id = int(floor_id)
        envelope.map_version = int(version)
        envelope.occupancy_grid = message
        self.floor_publisher.publish(envelope)

    def _map_message_locked(self, stamp, floor_id=None):
        floor_id = (
            getattr(self, "current_floor", 0)
            if floor_id is None
            else int(floor_id)
        )
        core = (
            self.floor_store.core(floor_id)
            if hasattr(self, "floor_store")
            else self.core
        )
        map_load_time = (
            self.map_load_times[floor_id]
            if hasattr(self, "map_load_times")
            else self.map_load_time
        )
        message = OccupancyGrid()
        message.header.stamp = stamp
        message.header.frame_id = self.map_frame
        message.info.map_load_time = map_load_time
        message.info.resolution = self.config.resolution
        message.info.width = self.config.size
        message.info.height = self.config.size
        message.info.origin.position.x = core.origin_x
        message.info.origin.position.y = core.origin_y
        message.info.origin.orientation.w = 1.0
        message.data = core.occupancy_data()
        return message

    @staticmethod
    def run():
        rospy.spin()
