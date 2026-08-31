"""ROS integration for the danger source perception pipeline."""

import message_filters
import math
import rospy
import tf2_ros
import threading
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_point

from danger_search_common.msg import (
    DangerSource,
    DangerSourceArray,
    DetectionStatus,
    LocalizationStatus,
    MappingStatus,
)

from .color_detector import RedCandidateDetector
from .confidence import observation_confidence
from .config import (
    ColorDetectionConfig,
    GeometryConfig,
    PipelineConfig,
    TrackingConfig,
)
from .depth_geometry import DepthGeometryValidator
from .floor_context import (
    FloorHeightClassifier,
    LocalizationCorrectionGate,
    MappingGate,
)
from .tracking import DetectionObservation, MultiFrameDangerTracker


class DangerDetectorNode:
    """Run the detector and publish observations through the v1.1-P0 API."""

    def __init__(self):
        rospy.init_node("danger_detector", anonymous=False)

        self.rgb_topic = rospy.get_param(
            "~rgb_topic", "/real_sense/rgb/image_raw"
        )
        self.depth_topic = rospy.get_param(
            "~depth_topic", "/real_sense/depth/image_raw"
        )
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic", "/real_sense/rgb/camera_info"
        )
        self.detections_topic = rospy.get_param(
            "~detections_topic", "/danger_detector/detections"
        )
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.target_frame = rospy.get_param(
            "~target_frame", self.map_frame
        )
        self.base_frame = rospy.get_param("~base_frame", "base")
        self.status_topic = rospy.get_param(
            "~status_topic", "/danger_detector/status"
        )
        self.floor_id = int(rospy.get_param("~floor_id", 0))
        self.current_floor = self.floor_id
        self.floor_lock = threading.Lock()
        self.mapping_status_topic = rospy.get_param(
            "~mapping_status_topic", "/mapping/status"
        )
        self.localization_status_topic = rospy.get_param(
            "~localization_status_topic", "/localization/status"
        )
        self.require_stable_mapping = bool(
            rospy.get_param(
                "~require_stable_mapping",
                self.target_frame == self.map_frame,
            )
        )
        self.mapping_status_timeout_s = float(
            rospy.get_param("~mapping_status_timeout_s", 1.5)
        )
        self.verify_floor_height = bool(
            rospy.get_param(
                "~verify_floor_height",
                # Floor identity is supplied by the explicit elevator/map
                # transition contract.  A continuous pose z is at most a
                # diagnostic cross-check, especially in Gazebo truth mode;
                # it must not become an implicit floor-switching input.
                False,
            )
        )
        self.floor_height_classifier = FloorHeightClassifier(
            rospy.get_param("~floor_heights", [0.0, 2.6, 5.2]),
            rospy.get_param("~floor_height_tolerance_m", 0.45),
        )
        self.mapping_gate = MappingGate(
            self.mapping_status_timeout_s,
            fallback_floor_id=self.floor_id,
        )
        self.require_localization_status = bool(
            rospy.get_param("~require_localization_status", True)
        )
        self.localization_status_timeout_s = float(
            rospy.get_param("~localization_status_timeout_s", 1.5)
        )
        self.correction_gate = LocalizationCorrectionGate(
            self.localization_status_timeout_s
        )
        self.input_fresh_timeout_s = float(
            rospy.get_param("~input_fresh_timeout_s", 1.0)
        )
        self.capability_version = int(
            rospy.get_param("~capability_version", 1)
        )
        sync_queue_size = int(rospy.get_param("~sync_queue_size", 10))
        sync_slop_s = float(rospy.get_param("~sync_slop_s", 0.05))

        self.color_config = self._load_color_config()
        self.geometry_config = self._load_geometry_config()
        self.pipeline_config = self._load_pipeline_config()
        self.tracking_enabled = bool(
            rospy.get_param("~tracking_enabled", True)
        )
        self.tracking_config = self._load_tracking_config()
        self.tracker = (
            MultiFrameDangerTracker(self.tracking_config)
            if self.tracking_enabled else None
        )
        self.color_detector = RedCandidateDetector(self.color_config)
        self.geometry_validator = DepthGeometryValidator(
            self.geometry_config
        )

        self.bridge = CvBridge()
        self.camera_model = PinholeCameraModel()
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.last_input_stamp = rospy.Time(0)
        self.last_detection_count = 0
        self.has_synchronized_input = False
        self.last_tf_available = False
        self.last_camera_valid = False
        self.last_floor_context_valid = not self.verify_floor_height
        self.last_floor_context_epoch = -1
        self.last_floor_context_reason = "WAITING_FOR_FLOOR_CONTEXT"
        self.state_lock = threading.Lock()
        # A queued synchronizer/timer callback can run while roslaunch is
        # closing publishers.  Keep a node-local latch in addition to
        # rospy.is_shutdown() so those callbacks never publish through an
        # already unregistered endpoint.
        self._shutdown_requested = False
        self.status_timer = None

        self.detections_pub = rospy.Publisher(
            self.detections_topic, DangerSourceArray, queue_size=10
        )
        self.status_pub = rospy.Publisher(
            self.status_topic, DetectionStatus, queue_size=10
        )
        self.mapping_status_sub = rospy.Subscriber(
            self.mapping_status_topic,
            MappingStatus,
            self._mapping_status_callback,
            queue_size=5,
        )
        self.localization_status_sub = rospy.Subscriber(
            self.localization_status_topic,
            LocalizationStatus,
            self._localization_status_callback,
            queue_size=5,
        )
        self.rgb_sub = message_filters.Subscriber(self.rgb_topic, Image)
        self.depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        self.camera_info_sub = message_filters.Subscriber(
            self.camera_info_topic, CameraInfo
        )
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub, self.camera_info_sub],
            queue_size=sync_queue_size,
            slop=sync_slop_s,
            allow_headerless=False,
        )
        self.synchronizer.registerCallback(self._sensor_callback)
        self.status_timer = rospy.Timer(
            rospy.Duration(0.5), self._publish_status
        )
        rospy.on_shutdown(self._on_shutdown)

        rospy.loginfo(
            "[perception] danger_detector started: RGB=%s depth=%s "
            "detections=%s status=%s frame=%s tracking=%s map_gate=%s",
            self.rgb_topic,
            self.depth_topic,
            self.detections_topic,
            self.status_topic,
            self.target_frame,
            self.tracking_enabled,
            self.require_stable_mapping,
        )

    def _is_shutting_down(self):
        """Return whether callbacks must no longer touch ROS endpoints."""
        return bool(getattr(self, "_shutdown_requested", False)) or rospy.is_shutdown()

    def _on_shutdown(self):
        """Stop the timer before ROS unregisters this node's publishers."""
        self._shutdown_requested = True
        timer = getattr(self, "status_timer", None)
        if timer is None:
            return
        try:
            timer.shutdown()
        except rospy.ROSException:
            # rospy may already have removed the timer while invoking this
            # hook.  The local latch still prevents a queued callback publish.
            pass

    def _publish_if_running(self, publisher, message):
        """Publish unless shutdown owns the endpoint.

        Only the known race where shutdown begins after the initial check is
        suppressed.  Transport failures during normal operation must remain
        visible instead of being hidden as a shutdown condition.
        """
        if self._is_shutting_down():
            return False
        try:
            publisher.publish(message)
        except rospy.ROSException:
            if self._is_shutting_down():
                return False
            raise
        return True

    def _sensor_callback(self, rgb_msg, depth_msg, camera_info_msg):
        if self._is_shutting_down():
            return
        with self.state_lock:
            self.has_synchronized_input = True
            self.last_input_stamp = rgb_msg.header.stamp
            self.last_detection_count = 0
            self.last_camera_valid = False
            self.last_floor_context_valid = not self.verify_floor_height
            self.last_floor_context_reason = "WAITING_FOR_FLOOR_CONTEXT"

        output = DangerSourceArray()
        output.header.stamp = rgb_msg.header.stamp
        output.header.frame_id = self.target_frame

        mapping_snapshot = self.mapping_gate.snapshot(
            rospy.Time.now().to_sec(),
            required=self.require_stable_mapping,
        )
        if not mapping_snapshot.allowed:
            self._set_floor_context(
                False, mapping_snapshot.epoch, mapping_snapshot.reason
            )
            self._publish(output)
            return
        correction_snapshot = self.correction_gate.snapshot(
            rospy.Time.now().to_sec(),
            required=self.require_localization_status,
        )
        if not correction_snapshot.allowed:
            self._set_floor_context(
                False, mapping_snapshot.epoch, correction_snapshot.reason
            )
            self._publish(output)
            return

        images = self._convert_images(rgb_msg, depth_msg)
        if images is None:
            self._publish(output)
            return
        bgr, depth_m = images
        if bgr.shape[:2] != depth_m.shape[:2]:
            rospy.logwarn_throttle(
                2.0,
                "[perception] RGB/depth size mismatch: %s vs %s",
                str(bgr.shape[:2]),
                str(depth_m.shape[:2]),
            )
            self._publish(output)
            return

        self.camera_model.fromCameraInfo(camera_info_msg)
        if not self._camera_model_is_valid(self.camera_model):
            rospy.logwarn_throttle(
                2.0, "[perception] CameraInfo has invalid intrinsics"
            )
            self._publish(output)
            return
        camera_frame = self._camera_frame(
            rgb_msg, depth_msg, camera_info_msg
        )
        if not camera_frame:
            rospy.logwarn_throttle(
                2.0, "[perception] Camera messages have no frame_id"
            )
            self._publish(output)
            return

        transform = self._lookup_transform(
            camera_frame, rgb_msg.header.stamp
        )
        if transform is None:
            with self.state_lock:
                self.last_tf_available = False
            self._publish(output)
            return

        # RGB/depth, intrinsics and the target-frame transform are valid at
        # this point.  Record that before the independent floor-height gate so
        # a rejected floor is not misreported as an invalid camera input.
        with self.state_lock:
            self.last_camera_valid = True
            self.last_tf_available = True

        if self.verify_floor_height:
            base_transform = self._lookup_transform(
                self.base_frame, rgb_msg.header.stamp
            )
            if base_transform is None:
                with self.state_lock:
                    self.last_tf_available = False
                self._set_floor_context(
                    False,
                    mapping_snapshot.epoch,
                    "FLOOR_HEIGHT_TF_UNAVAILABLE",
                )
                self._publish(output)
                return
            base_height = float(base_transform.transform.translation.z)
            classified_floor = self.floor_height_classifier.classify(
                base_height
            )
            if classified_floor != mapping_snapshot.floor_id:
                rospy.logwarn_throttle(
                    1.0,
                    "[perception] sensor-time floor mismatch: status=%d "
                    "base_z=%.3f classified=%s",
                    mapping_snapshot.floor_id,
                    base_height,
                    str(classified_floor),
                )
                self._set_floor_context(
                    False,
                    mapping_snapshot.epoch,
                    "FLOOR_HEIGHT_MISMATCH",
                )
                self._publish(output)
                return
        self._set_floor_context(True, mapping_snapshot.epoch, "OK")

        _, candidates = self.color_detector.detect(bgr)
        for candidate_index, candidate in enumerate(candidates):
            inner_mask = self.color_detector.make_inner_mask(
                candidate, bgr.shape
            )
            geometry = self.geometry_validator.validate(
                candidate, inner_mask, depth_m, self.camera_model
            )
            if geometry is None:
                continue
            if not self.pipeline_config.is_reliable_range(
                geometry.center_camera
            ):
                continue

            confidence = observation_confidence(
                candidate, geometry, self.geometry_config
            )
            if confidence < self.pipeline_config.confidence_threshold:
                continue

            danger = self._to_danger_message(
                geometry, confidence, camera_frame, rgb_msg.header.stamp,
                transform, candidate_index, mapping_snapshot.floor_id,
                mapping_snapshot.epoch, correction_snapshot.version,
            )
            if danger is not None:
                output.dangers.append(danger)

        now_s = rospy.Time.now().to_sec()
        if not self.mapping_gate.is_current(
            mapping_snapshot,
            now_s,
            required=self.require_stable_mapping,
        ) or not self.correction_gate.is_current(
            correction_snapshot,
            now_s,
            required=self.require_localization_status,
        ):
            output.dangers = []
            self._set_floor_context(
                False,
                mapping_snapshot.epoch,
                "LOCALIZATION_CONTEXT_CHANGED_DURING_FRAME",
            )
        elif self.tracker is not None and output.dangers:
            self._annotate_tracks(output.dangers, rgb_msg.header.stamp)

        with self.state_lock:
            self.last_detection_count = len(output.dangers)
        self._publish(output)

    def _convert_images(self, rgb_msg, depth_msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            depth = self.bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding="passthrough"
            )
            depth_m = self.geometry_validator.depth_to_metres(
                depth, depth_msg.encoding
            )
            return bgr, depth_m
        except (CvBridgeError, ValueError) as exc:
            rospy.logwarn_throttle(
                1.0, "[perception] Image conversion failed: %s", str(exc)
            )
            return None

    def _to_danger_message(
        self, geometry, confidence, camera_frame, stamp, transform,
        candidate_index, floor_id=None, map_epoch=0, correction_version=0
    ):
        point_camera = PointStamped()
        point_camera.header.stamp = stamp
        point_camera.header.frame_id = camera_frame
        point_camera.point.x = float(geometry.center_camera[0])
        point_camera.point.y = float(geometry.center_camera[1])
        point_camera.point.z = float(geometry.center_camera[2])
        try:
            point_target = do_transform_point(point_camera, transform)
        except Exception as exc:
            rospy.logwarn_throttle(
                1.0, "[perception] Point transform failed: %s", str(exc)
            )
            return None
        # tf2 copies the transform header. The P0 contract requires the
        # original sensor acquisition time on every detection position.
        point_target.header.stamp = stamp
        point_target.header.frame_id = self.target_frame

        danger = DangerSource()
        danger.detection_id = "m{}-c{}-{}.{}-{}".format(
            int(map_epoch), int(correction_version),
            stamp.secs, stamp.nsecs, candidate_index
        )
        danger.class_id = DangerSource.CLASS_DANGER_RED_SPHERE
        danger.position = point_target
        if floor_id is None:
            with self.floor_lock:
                floor_id = self.current_floor
        danger.floor_id = int(floor_id)
        danger.map_epoch = int(map_epoch)
        danger.confidence = float(confidence)
        danger.localization_correction_version = int(correction_version)
        danger.source_time = stamp
        return danger

    def _mapping_status_callback(self, message):
        if self._is_shutting_down():
            return
        self.mapping_gate.update(
            message.ready,
            message.stable,
            message.lost,
            message.current_floor,
            rospy.Time.now().to_sec(),
            transitioning=bool(getattr(message, "transitioning", False)),
            map_epoch=int(getattr(message, "map_epoch", 0)),
        )
        if message.current_floor < 0:
            rospy.logwarn_throttle(
                2.0,
                "[perception] blocking invalid floor id %d",
                message.current_floor,
            )
            return
        with self.floor_lock:
            self.current_floor = int(message.current_floor)

    def _localization_status_callback(self, message):
        if self._is_shutting_down():
            return
        changed = self.correction_gate.update(
            message.correction_version,
            rospy.Time.now().to_sec(),
        )
        if changed and self.tracker is not None:
            # Existing track coordinates belong to an older corrected map.
            # Mission-level confirmed results remain preserved independently.
            self.tracker.reset()

    def _annotate_tracks(self, dangers, stamp):
        observations = [
            DetectionObservation(
                detection_id=danger.detection_id,
                floor_id=danger.floor_id,
                position=(
                    danger.position.point.x,
                    danger.position.point.y,
                    danger.position.point.z,
                ),
                confidence=danger.confidence,
                stamp_s=stamp.to_sec(),
                map_epoch=int(danger.map_epoch),
            )
            for danger in dangers
        ]
        assignments = self.tracker.update(observations)
        for danger, assignment in zip(dangers, assignments):
            danger.track_id = assignment.track_id
            danger.position.point.x = assignment.position[0]
            danger.position.point.y = assignment.position[1]
            danger.position.point.z = assignment.position[2]
            danger.position_covariance = list(
                assignment.position_covariance
            )
            danger.confirmed = assignment.confirmed
            danger.verification_required = (
                not assignment.confirmed
                or bool(assignment.possible_duplicate_track_ids)
            )
            danger.possible_duplicate_track_ids = list(
                assignment.possible_duplicate_track_ids
            )

    def _set_floor_context(self, valid, epoch, reason):
        with self.state_lock:
            self.last_floor_context_valid = bool(valid)
            self.last_floor_context_epoch = int(epoch)
            self.last_floor_context_reason = str(reason)

    def _lookup_transform(self, source_frame, stamp):
        try:
            return self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                stamp,
                rospy.Duration(self.pipeline_config.tf_timeout_s),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            rospy.logwarn_throttle(
                1.0,
                "[perception] TF %s <- %s unavailable: %s",
                self.target_frame,
                source_frame,
                str(exc),
            )
            return None

    def _publish(self, output):
        if self._is_shutting_down():
            return False
        if self.pipeline_config.publish_empty_array or output.dangers:
            return self._publish_if_running(self.detections_pub, output)
        return False

    def _publish_status(self, _event=None):
        if self._is_shutting_down():
            return False
        now = rospy.Time.now()
        status = DetectionStatus()
        status.header.stamp = now
        status.header.frame_id = self.target_frame

        with self.state_lock:
            last_input_stamp = self.last_input_stamp
            has_synchronized_input = self.has_synchronized_input
            last_tf_available = self.last_tf_available
            last_camera_valid = self.last_camera_valid
            last_detection_count = self.last_detection_count
            last_floor_context_valid = self.last_floor_context_valid
            last_floor_context_epoch = self.last_floor_context_epoch
            last_floor_context_reason = self.last_floor_context_reason

        mapping_snapshot = self.mapping_gate.snapshot(
            now.to_sec(), required=self.require_stable_mapping
        )
        correction_snapshot = self.correction_gate.snapshot(
            now.to_sec(), required=self.require_localization_status
        )
        floor_context_current = (
            not self.verify_floor_height
            or (
                last_floor_context_valid
                and last_floor_context_epoch == mapping_snapshot.epoch
            )
        )

        input_age_s = float("inf")
        if last_input_stamp != rospy.Time(0):
            input_age_s = max(0.0, (now - last_input_stamp).to_sec())

        status.input_fresh = (
            has_synchronized_input
            and input_age_s < self.input_fresh_timeout_s
        )
        status.ready = (
            status.input_fresh
            and mapping_snapshot.allowed
            and correction_snapshot.allowed
            and last_camera_valid
            and last_tf_available
            and floor_context_current
        )
        status.input_latency_ms = (
            float(input_age_s * 1000.0)
            if input_age_s != float("inf")
            else -1.0
        )
        status.total_detections = last_detection_count
        if self.tracker is not None:
            confirmed_count, tentative_count = self.tracker.counts(
                now.to_sec()
            )
        else:
            confirmed_count, tentative_count = 0, 0
        status.confirmed_count = confirmed_count
        status.pending_verification = tentative_count
        status.capability_version = self.capability_version

        if not has_synchronized_input:
            status.status_reason = "WAITING_FOR_SYNCHRONIZED_INPUT"
        elif not status.input_fresh:
            status.status_reason = "INPUT_STALE"
        elif not mapping_snapshot.allowed:
            status.status_reason = mapping_snapshot.reason
        elif not correction_snapshot.allowed:
            status.status_reason = correction_snapshot.reason
        elif (
            self.verify_floor_height
            and last_floor_context_epoch == mapping_snapshot.epoch
            and not last_floor_context_valid
            and last_floor_context_reason.startswith("FLOOR_")
        ):
            status.status_reason = last_floor_context_reason
        elif not last_camera_valid:
            status.status_reason = "CAMERA_INPUT_INVALID"
        elif not last_tf_available:
            status.status_reason = "TARGET_FRAME_TF_UNAVAILABLE"
        elif not floor_context_current:
            status.status_reason = last_floor_context_reason
        else:
            status.status_reason = "OK"

        return self._publish_if_running(self.status_pub, status)

    @staticmethod
    def _camera_frame(rgb_msg, depth_msg, camera_info_msg):
        return (
            camera_info_msg.header.frame_id
            or rgb_msg.header.frame_id
            or depth_msg.header.frame_id
        )

    @staticmethod
    def _camera_model_is_valid(camera_model):
        try:
            parameters = (
                float(camera_model.fx()),
                float(camera_model.fy()),
                float(camera_model.cx()),
                float(camera_model.cy()),
            )
        except (AttributeError, TypeError, ValueError):
            return False
        return (
            all(math.isfinite(value) for value in parameters)
            and parameters[0] > 1e-9
            and parameters[1] > 1e-9
        )

    @staticmethod
    def _load_color_config():
        defaults = ColorDetectionConfig()
        return ColorDetectionConfig(
            **{
                name: rospy.get_param("~" + name, value)
                for name, value in vars(defaults).items()
            }
        )

    @staticmethod
    def _load_geometry_config():
        defaults = GeometryConfig()
        return GeometryConfig(
            **{
                name: rospy.get_param("~" + name, value)
                for name, value in vars(defaults).items()
            }
        )

    @staticmethod
    def _load_pipeline_config():
        defaults = PipelineConfig()
        return PipelineConfig(
            **{
                name: rospy.get_param("~" + name, value)
                for name, value in vars(defaults).items()
            }
        )

    @staticmethod
    def _load_tracking_config():
        defaults = TrackingConfig()
        return TrackingConfig(
            **{
                name: rospy.get_param("~track_" + name, value)
                for name, value in vars(defaults).items()
            }
        )

    @staticmethod
    def run():
        rospy.spin()
