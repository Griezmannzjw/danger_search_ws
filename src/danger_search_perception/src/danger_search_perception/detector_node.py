"""ROS integration for the danger source perception pipeline."""

import message_filters
import rospy
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_point

from danger_search_common.msg import (
    DangerSource,
    DangerSourceArray,
    DetectionStatus,
)

from .color_detector import RedCandidateDetector
from .confidence import observation_confidence
from .config import ColorDetectionConfig, GeometryConfig, PipelineConfig
from .depth_geometry import DepthGeometryValidator


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
        map_frame = rospy.get_param("~map_frame", "map")
        self.target_frame = rospy.get_param("~target_frame", map_frame)
        self.status_topic = rospy.get_param(
            "~status_topic", "/danger_detector/status"
        )
        self.floor_id = int(rospy.get_param("~floor_id", 0))
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

        self.detections_pub = rospy.Publisher(
            self.detections_topic, DangerSourceArray, queue_size=10
        )
        self.status_pub = rospy.Publisher(
            self.status_topic, DetectionStatus, queue_size=10
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

        rospy.loginfo(
            "[perception] danger_detector started: RGB=%s depth=%s "
            "detections=%s status=%s frame=%s",
            self.rgb_topic,
            self.depth_topic,
            self.detections_topic,
            self.status_topic,
            self.target_frame,
        )

    def _sensor_callback(self, rgb_msg, depth_msg, camera_info_msg):
        self.has_synchronized_input = True
        self.last_input_stamp = rgb_msg.header.stamp
        self.last_detection_count = 0

        output = DangerSourceArray()
        output.header.stamp = rgb_msg.header.stamp
        output.header.frame_id = self.target_frame

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
            self.last_tf_available = False
            self._publish(output)
            return
        self.last_tf_available = True

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

            confidence = observation_confidence(
                candidate, geometry, self.geometry_config
            )
            if confidence < self.pipeline_config.confidence_threshold:
                continue

            danger = self._to_danger_message(
                geometry, confidence, camera_frame, rgb_msg.header.stamp,
                transform, candidate_index
            )
            if danger is not None:
                output.dangers.append(danger)

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
        candidate_index
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
        danger.detection_id = "{}.{}-{}".format(
            stamp.secs, stamp.nsecs, candidate_index
        )
        danger.class_id = DangerSource.CLASS_DANGER_RED_SPHERE
        danger.position = point_target
        danger.floor_id = self.floor_id
        danger.confidence = float(confidence)
        danger.source_time = stamp
        return danger

    def _lookup_transform(self, camera_frame, stamp):
        try:
            return self.tf_buffer.lookup_transform(
                self.target_frame,
                camera_frame,
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
                camera_frame,
                str(exc),
            )
            return None

    def _publish(self, output):
        if self.pipeline_config.publish_empty_array or output.dangers:
            self.detections_pub.publish(output)

    def _publish_status(self, _event=None):
        now = rospy.Time.now()
        status = DetectionStatus()
        status.header.stamp = now
        status.header.frame_id = self.target_frame

        input_age_s = float("inf")
        if self.last_input_stamp != rospy.Time(0):
            input_age_s = max(0.0, (now - self.last_input_stamp).to_sec())

        status.input_fresh = (
            self.has_synchronized_input
            and input_age_s < self.input_fresh_timeout_s
        )
        status.ready = status.input_fresh and self.last_tf_available
        status.input_latency_ms = (
            float(input_age_s * 1000.0)
            if input_age_s != float("inf")
            else -1.0
        )
        status.total_detections = self.last_detection_count
        # P0 confirmation and de-duplication are owned by mission.
        status.confirmed_count = 0
        status.pending_verification = 0
        status.capability_version = self.capability_version

        if not self.has_synchronized_input:
            status.status_reason = "WAITING_FOR_SYNCHRONIZED_INPUT"
        elif not status.input_fresh:
            status.status_reason = "INPUT_STALE"
        elif not self.last_tf_available:
            status.status_reason = "TARGET_FRAME_TF_UNAVAILABLE"
        else:
            status.status_reason = "OK"

        self.status_pub.publish(status)

    @staticmethod
    def _camera_frame(rgb_msg, depth_msg, camera_info_msg):
        return (
            depth_msg.header.frame_id
            or rgb_msg.header.frame_id
            or camera_info_msg.header.frame_id
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
    def run():
        rospy.spin()
