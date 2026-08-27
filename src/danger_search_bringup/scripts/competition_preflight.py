#!/usr/bin/env python3
"""Fail-closed runtime contract check for formal competition launches."""

import json
import os
import sys
import time

import rosgraph
import rosservice
import rospy
import tf2_ros
from std_msgs.msg import Bool, String

from building_generator_interfaces.srv import CallElevator, SetDoorState
from danger_search_common.srv import SwitchFloor


FORBIDDEN_TOPICS = {
    "/gazebo/link_states",
    "/Odometry_gazebo",
    "/ground_truth/base_w",
    "/ground_truth/base_trunk",
    "/ground_truth/FL_foot",
    "/ground_truth/FR_foot",
    "/ground_truth/RL_foot",
    "/ground_truth/RR_foot",
}
FORBIDDEN_FILE_BASENAMES = {
    "layout_metadata.json",
    "building_config.json",
    "scene_manifest.json",
    "danger_truth.json",
}
FORMAL_FALSE_ENVIRONMENT = (
    "ENABLE_REFEREE_ODOM",
    "ENABLE_GROUND_TRUTH",
    "POINTCLOUD_USE_GROUND_TRUTH_ODOM",
)
ALGORITHM_NODES = {
    "/competition_preflight",
    "/control",
    "/entrance_door",
    "/exploration",
    "/lidar_odometry",
    "/local_occupancy_mapper",
    "/local_scan_projector",
    "/localization_adapter",
    "/mission",
    "/move_base",
    "/navigation_config_guard",
    "/navigation_monitor",
    "/perception",
}


def validate_runtime_contract(competition_mode, multifloor_enabled,
                              localization_backend, environment):
    errors = []
    if competition_mode and not multifloor_enabled:
        errors.append("competition_mode requires multifloor_enabled=true")
    if competition_mode and localization_backend != "gicp":
        errors.append("competition_mode requires localization_backend=gicp")
    if competition_mode:
        for name in FORMAL_FALSE_ENVIRONMENT:
            value = str(environment.get(name, "0")).strip().lower()
            if value not in {"0", "false", "off", "no"}:
                errors.append("%s must be disabled in competition mode" % name)
    return errors


def load_public_scene_contract(path):
    normalized = os.path.abspath(os.path.expanduser(os.path.expandvars(str(path))))
    if os.path.basename(normalized) != "team_scene_info.json":
        raise ValueError("only team_scene_info.json is allowed")
    with open(normalized, encoding="utf-8") as stream:
        document = json.load(stream)
    if document.get("schema") != "team_scene_info_v1":
        raise ValueError("unsupported public scene schema")
    public_scene = document.get("public_scene")
    if not isinstance(public_scene, dict) or not public_scene.get("elevators"):
        raise ValueError("public scene has no elevator topology")
    return normalized, document


def effective_forbidden_topics(public_scene_contract=None):
    """Combine fail-closed built-ins with optional referee additions."""
    topics = set(FORBIDDEN_TOPICS)
    if not public_scene_contract:
        return topics
    declared = public_scene_contract.get("referee_only", {}).get(
        "forbidden_topics", []
    )
    if declared is None:
        return topics
    if not isinstance(declared, list) or not all(
            isinstance(topic, str) and topic.startswith("/")
            for topic in declared):
        raise ValueError("referee_only.forbidden_topics must be a topic list")
    topics.update(declared)
    return topics


def forbidden_parameter_references(parameter_tree, forbidden_topics=None):
    violations = []
    forbidden_topics = set(forbidden_topics or FORBIDDEN_TOPICS)

    def visit(path, value):
        if isinstance(value, dict):
            for key, child in value.items():
                visit(path + "/" + str(key), child)
            return
        if isinstance(value, (list, tuple)):
            for index, child in enumerate(value):
                visit(path + "/" + str(index), child)
            return
        if not isinstance(value, str):
            return
        normalized = value.strip()
        if normalized in forbidden_topics:
            violations.append("%s references forbidden topic %s" % (path, normalized))
        basename = os.path.basename(normalized)
        if basename in FORBIDDEN_FILE_BASENAMES:
            violations.append("%s references forbidden file %s" % (path, basename))

    visit("", parameter_tree)
    return violations


def command_owner_error(publishers, expected_owner):
    owners = sorted(set(publishers))
    if owners == [expected_owner]:
        return ""
    return "%s publishers must be [%s], got %s" % (
        "/cmd_vel", expected_owner, owners
    )


def normalize_system_state(response):
    """Accept rosgraph's unwrapped value and the raw XML-RPC envelope."""
    state = response
    if (isinstance(response, (list, tuple)) and len(response) == 3
            and isinstance(response[0], int)
            and isinstance(response[1], str)):
        if response[0] != 1:
            raise RuntimeError("ROS master getSystemState failed: %s" % response[1])
        state = response[2]
    if not isinstance(state, (list, tuple)) or len(state) != 3:
        raise RuntimeError("unexpected ROS master system-state response")
    publishers, subscribers, services = state
    return dict(publishers), dict(subscribers), dict(services)


def forbidden_algorithm_subscriptions(subscribers, forbidden_topics,
                                      algorithm_nodes=None):
    algorithm_nodes = set(algorithm_nodes or ALGORITHM_NODES)
    violations = []
    for topic in forbidden_topics:
        owners = sorted(
            owner for owner in subscribers.get(topic, [])
            if owner in algorithm_nodes
        )
        if owners:
            violations.append(
                "forbidden subscription %s by %s" % (topic, owners)
            )
    return violations


class CompetitionPreflight:
    def __init__(self):
        rospy.init_node("competition_preflight", anonymous=False)
        self.competition_mode = bool(rospy.get_param("~competition_mode", True))
        self.multifloor_enabled = bool(rospy.get_param("~multifloor_enabled", True))
        self.localization_backend = str(rospy.get_param(
            "~localization_backend", "gicp"
        ))
        self.scene_info_file = str(rospy.get_param("~scene_info_file", ""))
        self.result_file = os.path.abspath(os.path.expanduser(os.path.expandvars(
            str(rospy.get_param("~result_file", ""))
        )))
        self.timeout_s = float(rospy.get_param("~timeout_s", 45.0))
        self.map_frame = str(rospy.get_param("~map_frame", "map"))
        self.base_frame = str(rospy.get_param("~base_frame", "base"))
        self.cmd_topic = str(rospy.get_param("~cmd_topic", "/cmd_vel"))
        self.cmd_owner = str(rospy.get_param("~cmd_owner", "/control"))
        self.forbidden_topics = set(FORBIDDEN_TOPICS)
        self.ready_pub = rospy.Publisher(
            "/danger_search/preflight_ready", Bool, queue_size=1, latch=True
        )
        self.status_pub = rospy.Publisher(
            "/danger_search/preflight_status", String, queue_size=1, latch=True
        )
        self.ready_pub.publish(Bool(data=False))

    def _static_checks(self):
        errors = validate_runtime_contract(
            self.competition_mode,
            self.multifloor_enabled,
            self.localization_backend,
            os.environ,
        )
        try:
            _path, scene = load_public_scene_contract(self.scene_info_file)
            self.forbidden_topics = effective_forbidden_topics(scene)
        except (OSError, ValueError, TypeError, json.JSONDecodeError) as exc:
            errors.append("invalid team_scene_info.json: %s" % exc)
        if not self.result_file:
            errors.append("result_file is empty")
        else:
            directory = os.path.dirname(self.result_file)
            if not os.path.isdir(directory):
                errors.append("result directory does not exist: %s" % directory)
            elif not os.access(directory, os.W_OK):
                errors.append("result directory is not writable: %s" % directory)
        errors.extend(forbidden_parameter_references(
            rospy.get_param("/", {}), self.forbidden_topics
        ))
        # Importing generated classes above proves the source order; checking
        # _type also guards against a shadow package with the same module name.
        if CallElevator._type != "building_generator_interfaces/CallElevator":
            errors.append("unexpected CallElevator service type")
        if SetDoorState._type != "building_generator_interfaces/SetDoorState":
            errors.append("unexpected SetDoorState service type")
        if SwitchFloor._type != "danger_search_common/SwitchFloor":
            errors.append("unexpected SwitchFloor service type")
        return errors

    @staticmethod
    def _system_state():
        response = rosgraph.Master(rospy.get_name()).getSystemState()
        return normalize_system_state(response)

    def _dynamic_errors(self, tf_buffer):
        errors = []
        publishers, subscribers, _services = self._system_state()
        errors.extend(forbidden_algorithm_subscriptions(
            subscribers, self.forbidden_topics
        ))
        owner_error = command_owner_error(
            publishers.get(self.cmd_topic, []), self.cmd_owner
        )
        if owner_error:
            errors.append(owner_error)

        required_services = {
            "/call_elevator": "building_generator_interfaces/CallElevator",
            "/set_door_state": "building_generator_interfaces/SetDoorState",
            "/localization/switch_floor": "danger_search_common/SwitchFloor",
            "/move_base/make_plan": "nav_msgs/GetPlan",
            "/move_base/clear_costmaps": "std_srvs/Empty",
        }
        for service, expected_type in required_services.items():
            actual_type = rosservice.get_service_type(service)
            if actual_type != expected_type:
                errors.append(
                    "%s type is %s, expected %s" % (
                        service, actual_type or "unavailable", expected_type
                    )
                )

        topic_types = dict(rospy.get_published_topics())
        required_topics = {
            "/scan": "sensor_msgs/PointCloud",
            "/trunk_imu": "sensor_msgs/Imu",
            "/real_sense/rgb/image_raw": "sensor_msgs/Image",
            "/real_sense/depth/image_raw": "sensor_msgs/Image",
            "/real_sense/rgb/camera_info": "sensor_msgs/CameraInfo",
            "/mapping/status": "danger_search_common/MappingStatus",
        }
        for topic, expected_type in required_topics.items():
            actual_type = topic_types.get(topic)
            if actual_type != expected_type:
                errors.append(
                    "%s type is %s, expected %s" % (
                        topic, actual_type or "unavailable", expected_type
                    )
                )
        try:
            tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rospy.Time(0),
                rospy.Duration(0.05),
            )
        except Exception as exc:
            errors.append("TF %s->%s unavailable: %s" % (
                self.map_frame, self.base_frame, exc
            ))
        return errors

    def run(self):
        static_errors = self._static_checks()
        if static_errors:
            self._fail(static_errors)
        tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        _listener = tf2_ros.TransformListener(tf_buffer)
        deadline = time.monotonic() + self.timeout_s
        last_errors = ["runtime checks have not run"]
        rate = rospy.Rate(2.0)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            last_errors = self._dynamic_errors(tf_buffer)
            if not last_errors:
                self.ready_pub.publish(Bool(data=True))
                self.status_pub.publish(String(data="READY"))
                rospy.loginfo("[preflight] competition runtime contract READY")
                rospy.spin()
                return
            self.status_pub.publish(String(data="WAITING: " + "; ".join(last_errors)))
            rate.sleep()
        self._fail(last_errors)

    def _fail(self, errors):
        message = "; ".join(errors)
        self.ready_pub.publish(Bool(data=False))
        self.status_pub.publish(String(data="FAILED: " + message))
        rospy.logfatal("[preflight] %s", message)
        raise RuntimeError(message)


if __name__ == "__main__":
    try:
        CompetitionPreflight().run()
    except (rospy.ROSInterruptException, RuntimeError) as exc:
        if str(exc):
            rospy.logerr("[preflight] exiting: %s", exc)
        sys.exit(1)
