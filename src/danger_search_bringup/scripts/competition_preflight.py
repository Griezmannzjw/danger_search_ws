#!/usr/bin/env python3
"""Fail-closed runtime contract check for formal competition launches."""

import json
import math
import os
import sys
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
import rosgraph
import rosservice
import rospy
import tf2_ros
from sensor_msgs.msg import Imu
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
RUN_PROFILE_FORMAL = "formal"
RUN_PROFILE_SIMULATION_TRUTH = "simulation_truth"
TRUTH_LINK_STATES_TOPIC = "/gazebo/link_states"
TRUTH_ODOMETRY_NODE = "/gazebo_truth_odometry"
TRUTH_LINK_STATES_TYPE = "gazebo_msgs/LinkStates"
TRUTH_GAZEBO_BASE_LINK = "a1_gazebo::base"
TRUTH_PARAMETER_PATH = "/gazebo_truth_odometry/gazebo_link_states_topic"
TRUTH_RAW_POSE_TOPIC = "/localization/raw_pose"
TRUTH_RAW_POSE_TYPE = "geometry_msgs/PoseWithCovarianceStamped"
RESULT_FILENAME_BY_RUN_PROFILE = {
    RUN_PROFILE_FORMAL: "detected_danger.json",
    RUN_PROFILE_SIMULATION_TRUTH: "detected_danger.simulation_truth.json",
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
REQUIRED_TOPIC_TYPES = {
    "/scan": "sensor_msgs/PointCloud",
    "/trunk_imu": "sensor_msgs/Imu",
    "/real_sense/rgb/image_raw": "sensor_msgs/Image",
    "/real_sense/depth/image_raw": "sensor_msgs/Image",
    "/real_sense/depth/camera_info": "sensor_msgs/CameraInfo",
    "/real_sense/rgb/camera_info": "sensor_msgs/CameraInfo",
    "/localization/depth_obstacle_scan": "sensor_msgs/LaserScan",
    "/mapping/status": "danger_search_common/MappingStatus",
    # These make the floor/map/navigation envelope observable before mission
    # can consume a preflight READY latch.
    "/mapping/active_map": "danger_search_common/FloorOccupancyGrid",
    "/navigation/health": "danger_search_common/NavigationHealth",
}
ALGORITHM_NODES = {
    "/competition_preflight",
    "/control",
    "/entrance_door",
    "/exploration",
    "/lidar_odometry",
    "/local_occupancy_mapper",
    "/depth_obstacle_projector",
    "/local_scan_projector",
    "/localization_adapter",
    TRUTH_ODOMETRY_NODE,
    "/mission",
    "/move_base",
    "/navigation_config_guard",
    "/navigation_monitor",
    "/perception",
    "/posture_safety_monitor",
}
RUNTIME_IDENTITY_NODES = {
    "/control",
    "/exploration",
    "/local_occupancy_mapper",
    "/local_scan_projector",
    "/localization_adapter",
    "/mission",
    "/move_base",
    "/navigation_config_guard",
    "/navigation_monitor",
    "/perception",
}


def normalize_run_profile(run_profile):
    normalized = str(run_profile or RUN_PROFILE_FORMAL).strip().lower()
    if normalized not in {RUN_PROFILE_FORMAL, RUN_PROFILE_SIMULATION_TRUTH}:
        raise ValueError("run_profile must be formal or simulation_truth")
    return normalized


def expected_result_filename(run_profile):
    return RESULT_FILENAME_BY_RUN_PROFILE[normalize_run_profile(run_profile)]


def validate_runtime_contract(competition_mode, multifloor_enabled,
                              localization_backend, environment,
                              run_profile=RUN_PROFILE_FORMAL):
    errors = []
    try:
        run_profile = normalize_run_profile(run_profile)
    except ValueError as exc:
        return [str(exc)]
    if run_profile == RUN_PROFILE_FORMAL:
        if not competition_mode:
            errors.append("formal profile requires competition_mode=true")
        if not multifloor_enabled:
            errors.append("formal profile requires multifloor_enabled=true")
        if localization_backend != "gicp":
            errors.append("formal profile requires localization_backend=gicp")
        for name in FORMAL_FALSE_ENVIRONMENT:
            value = str(environment.get(name, "0")).strip().lower()
            if value not in {"0", "false", "off", "no"}:
                errors.append("%s must be disabled in formal profile" % name)
    else:
        if competition_mode:
            errors.append("simulation_truth profile requires competition_mode=false")
        if not multifloor_enabled:
            errors.append("simulation_truth profile requires multifloor_enabled=true")
        if localization_backend != "gazebo_truth":
            errors.append(
                "simulation_truth profile requires localization_backend=gazebo_truth"
            )
    return errors


def validate_truth_base_link(run_profile, gazebo_base_link):
    """Keep the simulation truth adapter scoped to the competition robot.

    ``system.launch`` is deliberately an internal assembly entry point, but
    it still must not become a generic ``/gazebo/link_states`` adapter when
    invoked directly.  The formal profile does not consume this parameter.
    """
    if normalize_run_profile(run_profile) != RUN_PROFILE_SIMULATION_TRUTH:
        return []
    if str(gazebo_base_link).strip() != TRUTH_GAZEBO_BASE_LINK:
        return [
            "simulation_truth profile requires gazebo_base_link=%s"
            % TRUTH_GAZEBO_BASE_LINK
        ]
    return []


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


def effective_forbidden_topics(public_scene_contract=None,
                               run_profile=RUN_PROFILE_FORMAL):
    """Combine fail-closed built-ins with optional referee additions."""
    topics = set(FORBIDDEN_TOPICS)
    if public_scene_contract:
        declared = public_scene_contract.get("referee_only", {}).get(
            "forbidden_topics", []
        )
        if declared is not None:
            if not isinstance(declared, list) or not all(
                    isinstance(topic, str) and topic.startswith("/")
                    for topic in declared):
                raise ValueError("referee_only.forbidden_topics must be a topic list")
            topics.update(declared)
    if normalize_run_profile(run_profile) == RUN_PROFILE_SIMULATION_TRUTH:
        topics.discard(TRUTH_LINK_STATES_TOPIC)
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


def truth_parameter_reference_violations(parameter_tree):
    """Allow the Gazebo link-state parameter only on the dedicated test node."""
    violations = []

    def visit(path, value):
        if isinstance(value, dict):
            for key, child in value.items():
                visit(path + "/" + str(key), child)
            return
        if isinstance(value, (list, tuple)):
            for index, child in enumerate(value):
                visit(path + "/" + str(index), child)
            return
        if isinstance(value, str) and value.strip() == TRUTH_LINK_STATES_TOPIC:
            if path != TRUTH_PARAMETER_PATH:
                violations.append(
                    "%s references %s outside %s" % (
                        path, TRUTH_LINK_STATES_TOPIC, TRUTH_PARAMETER_PATH
                    )
                )

    visit("", parameter_tree)
    return violations


def runtime_identity_parameter_violations(
        parameter_tree, run_profile, competition_mode,
        multifloor_enabled, localization_backend):
    """Require every assembled algorithm component to share one identity."""
    run_profile = normalize_run_profile(run_profile)
    nodes = set(RUNTIME_IDENTITY_NODES)
    nodes.add(
        TRUTH_ODOMETRY_NODE
        if run_profile == RUN_PROFILE_SIMULATION_TRUTH
        else "/lidar_odometry"
    )
    expected = {
        "run_profile": run_profile,
        "competition_mode": bool(competition_mode),
        "multifloor_enabled": bool(multifloor_enabled),
        "localization_backend": str(localization_backend),
    }
    violations = []
    for node in sorted(nodes):
        params = parameter_tree.get(node.lstrip("/"), {})
        if not isinstance(params, dict):
            params = {}
        for key, expected_value in expected.items():
            actual = params.get(key, None)
            if actual != expected_value:
                violations.append(
                    "%s/%s is %r, expected %r" % (
                        node, key, actual, expected_value
                    )
                )
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


def truth_subscription_violations(subscribers, algorithm_nodes=None):
    """The simulation profile gives exactly one algorithm node truth access."""
    algorithm_nodes = set(algorithm_nodes or ALGORITHM_NODES)
    owners = set(subscribers.get(TRUTH_LINK_STATES_TOPIC, []))
    algorithm_owners = owners.intersection(algorithm_nodes)
    violations = []
    unexpected = sorted(algorithm_owners - {TRUTH_ODOMETRY_NODE})
    if unexpected:
        violations.append(
            "simulation truth subscription %s by %s" % (
                TRUTH_LINK_STATES_TOPIC, unexpected
            )
        )
    if TRUTH_ODOMETRY_NODE not in algorithm_owners:
        violations.append(
            "%s must subscribe to %s" % (
                TRUTH_ODOMETRY_NODE, TRUTH_LINK_STATES_TOPIC
            )
        )
    return violations


class CompetitionPreflight:
    def __init__(self):
        rospy.init_node("competition_preflight", anonymous=False)
        try:
            self.run_profile = normalize_run_profile(
                rospy.get_param("~run_profile", RUN_PROFILE_FORMAL)
            )
        except ValueError as exc:
            raise rospy.ROSInitException(str(exc))
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
        self.gazebo_base_link = str(rospy.get_param(
            "~gazebo_base_link", "a1_gazebo::base"
        ))
        self.truth_link_max_age_s = float(rospy.get_param(
            "~truth_link_max_age_s", 1.0
        ))
        if (not math.isfinite(self.truth_link_max_age_s)
                or self.truth_link_max_age_s <= 0.0):
            raise rospy.ROSInitException("~truth_link_max_age_s must be positive")
        self.imu_topic = str(rospy.get_param("~imu_topic", "/trunk_imu"))
        self.imu_max_age_s = float(rospy.get_param("~imu_max_age_s", 1.0))
        if (not self.imu_topic.startswith("/") or
                not math.isfinite(self.imu_max_age_s) or
                self.imu_max_age_s <= 0.0):
            raise rospy.ROSInitException("invalid IMU preflight parameters")
        self._imu_received_at = None
        self._imu_valid = False
        self._imu_subscriber = rospy.Subscriber(
            self.imu_topic, Imu, self._imu_callback, queue_size=5,
        )
        self._truth_raw_pose_received_at = None
        self._truth_raw_pose_subscriber = None
        if self.run_profile == RUN_PROFILE_SIMULATION_TRUTH:
            # Do not subscribe to LinkStates here: only gazebo_truth_odometry
            # is allowed to consume it. Its raw-pose output proves that the
            # configured link is present and fresh for the current scan.
            self._truth_raw_pose_subscriber = rospy.Subscriber(
                TRUTH_RAW_POSE_TOPIC,
                PoseWithCovarianceStamped,
                self._truth_raw_pose_callback,
                queue_size=1,
            )
        self.forbidden_topics = set(FORBIDDEN_TOPICS)
        self.ready_pub = rospy.Publisher(
            "/danger_search/preflight_ready", Bool, queue_size=1, latch=True
        )
        self.status_pub = rospy.Publisher(
            "/danger_search/preflight_status", String, queue_size=1, latch=True
        )
        self.ready_pub.publish(Bool(data=False))

    def _truth_raw_pose_callback(self, _message):
        self._truth_raw_pose_received_at = time.monotonic()

    def _imu_callback(self, message):
        q = message.orientation
        values = (q.x, q.y, q.z, q.w)
        self._imu_valid = all(math.isfinite(float(value)) for value in values)
        self._imu_received_at = time.monotonic()

    def _static_checks(self):
        errors = validate_runtime_contract(
            self.competition_mode,
            self.multifloor_enabled,
            self.localization_backend,
            os.environ,
            self.run_profile,
        )
        errors.extend(validate_truth_base_link(
            self.run_profile, self.gazebo_base_link
        ))
        try:
            _path, scene = load_public_scene_contract(self.scene_info_file)
            self.forbidden_topics = effective_forbidden_topics(
                scene, self.run_profile
            )
        except (OSError, ValueError, TypeError, json.JSONDecodeError) as exc:
            errors.append("invalid team_scene_info.json: %s" % exc)
        if not self.result_file:
            errors.append("result_file is empty")
        else:
            try:
                expected_basename = expected_result_filename(self.run_profile)
            except ValueError:
                expected_basename = None
            if (expected_basename is not None
                    and os.path.basename(self.result_file) != expected_basename):
                errors.append(
                    "result_file must end with %s for %s profile" % (
                        expected_basename, self.run_profile
                    )
                )
            directory = os.path.dirname(self.result_file)
            if not os.path.isdir(directory):
                errors.append("result directory does not exist: %s" % directory)
            elif not os.access(directory, os.W_OK):
                errors.append("result directory is not writable: %s" % directory)
        parameter_tree = rospy.get_param("/", {})
        errors.extend(runtime_identity_parameter_violations(
            parameter_tree,
            self.run_profile,
            self.competition_mode,
            self.multifloor_enabled,
            self.localization_backend,
        ))
        errors.extend(forbidden_parameter_references(
            parameter_tree, self.forbidden_topics
        ))
        if self.run_profile == RUN_PROFILE_SIMULATION_TRUTH:
            errors.extend(truth_parameter_reference_violations(parameter_tree))
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
        if self.run_profile == RUN_PROFILE_SIMULATION_TRUTH:
            errors.extend(truth_subscription_violations(subscribers))
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
        for topic, expected_type in REQUIRED_TOPIC_TYPES.items():
            actual_type = topic_types.get(topic)
            if actual_type != expected_type:
                errors.append(
                    "%s type is %s, expected %s" % (
                        topic, actual_type or "unavailable", expected_type
                    )
                )
        if self._imu_received_at is None:
            errors.append("%s has no received IMU sample" % self.imu_topic)
        elif time.monotonic() - self._imu_received_at > self.imu_max_age_s:
            errors.append("%s IMU sample is stale" % self.imu_topic)
        elif not self._imu_valid:
            errors.append("%s IMU sample is invalid" % self.imu_topic)
        if self.run_profile == RUN_PROFILE_SIMULATION_TRUTH:
            actual_type = topic_types.get(TRUTH_LINK_STATES_TOPIC)
            if actual_type != TRUTH_LINK_STATES_TYPE:
                errors.append(
                    "%s type is %s, expected %s" % (
                        TRUTH_LINK_STATES_TOPIC,
                        actual_type or "unavailable",
                        TRUTH_LINK_STATES_TYPE,
                    )
                )
            raw_pose_type = topic_types.get(TRUTH_RAW_POSE_TOPIC)
            if raw_pose_type != TRUTH_RAW_POSE_TYPE:
                errors.append(
                    "%s type is %s, expected %s" % (
                        TRUTH_RAW_POSE_TOPIC,
                        raw_pose_type or "unavailable",
                        TRUTH_RAW_POSE_TYPE,
                    )
                )
            received_at = self._truth_raw_pose_received_at
            if received_at is None or (
                    time.monotonic() - received_at > self.truth_link_max_age_s):
                errors.append(
                    "%s has no fresh pose from Gazebo link %s" % (
                        TRUTH_RAW_POSE_TOPIC, self.gazebo_base_link
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
                rospy.loginfo("[preflight] %s runtime contract READY", self.run_profile)
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
