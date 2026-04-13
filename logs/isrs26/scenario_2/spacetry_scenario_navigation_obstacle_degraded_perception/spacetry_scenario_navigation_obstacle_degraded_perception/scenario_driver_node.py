#!/usr/bin/env python3
import json
import math
import os
import signal
import subprocess
import time
from collections import deque
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Tuple

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(q: Any) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def distance_xy(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def sector_min(scan: LaserScan, min_angle: float, max_angle: float) -> float:
    if not scan.ranges or scan.angle_increment == 0.0:
        return float("inf")

    if max_angle < min_angle:
        min_angle, max_angle = max_angle, min_angle

    start_index = max(
        0,
        min(
            len(scan.ranges) - 1,
            int(round((min_angle - scan.angle_min) / scan.angle_increment)),
        ),
    )
    end_index = max(
        0,
        min(
            len(scan.ranges) - 1,
            int(round((max_angle - scan.angle_min) / scan.angle_increment)),
        ),
    )

    best = float("inf")
    for index in range(min(start_index, end_index), max(start_index, end_index) + 1):
        value = scan.ranges[index]
        if not math.isfinite(value) or value <= 0.0:
            continue
        if value < scan.range_min or value > scan.range_max:
            continue
        best = min(best, float(value))
    return best


def line_progress_ratio(
    point: Tuple[float, float],
    start: Tuple[float, float],
    goal: Tuple[float, float],
) -> float:
    dx = goal[0] - start[0]
    dy = goal[1] - start[1]
    denom = dx * dx + dy * dy
    if denom <= 1e-9:
        return 0.0
    px = point[0] - start[0]
    py = point[1] - start[1]
    return clamp((px * dx + py * dy) / denom, 0.0, 1.0)


def point_to_route_distance(
    point: Tuple[float, float],
    start: Tuple[float, float],
    goal: Tuple[float, float],
) -> float:
    progress = line_progress_ratio(point, start, goal)
    projected = (
        start[0] + (goal[0] - start[0]) * progress,
        start[1] + (goal[1] - start[1]) * progress,
    )
    return distance_xy(point, projected)


def load_yaml_file(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


class ScenarioDriver(Node):
    def __init__(self) -> None:
        super().__init__("scenario_navigation_obstacle_degraded_perception_driver")

        self.declare_parameter("scenario_contract_file", "")
        self.declare_parameter("scenario_config_file", "")
        self.declare_parameter("output_root", "/ws/log")
        self.declare_parameter("rosbag_enabled", True)

        contract_file = Path(str(self.get_parameter("scenario_contract_file").value))
        config_file = Path(str(self.get_parameter("scenario_config_file").value))
        output_root = Path(str(self.get_parameter("output_root").value))
        self.rosbag_enabled = bool(self.get_parameter("rosbag_enabled").value)

        self.contract = load_yaml_file(contract_file)
        self.config = load_yaml_file(config_file)

        self.scenario_name = str(self.config["scenario_name"])
        self.output_dir = output_root / f"spacetry_scenario_{self.scenario_name}"
        self.metrics_dir = self.output_dir / "metrics"
        self.runtime_dir = self.output_dir / "runtime"
        self.rosbag_dir = self.output_dir / "rosbags"
        for directory in (self.output_dir, self.metrics_dir, self.runtime_dir, self.rosbag_dir):
            directory.mkdir(parents=True, exist_ok=True)

        self.report_path = (
            self.output_dir
            / f"spacetry_scenario_{self.scenario_name}_report.md"
        )
        self.metrics_path = self.metrics_dir / "scenario_metrics.json"
        self.timeline_path = self.runtime_dir / "timeline.jsonl"
        self.timeline_handle = self.timeline_path.open("a", encoding="utf-8")
        self.rosbag_log_path = self.runtime_dir / "rosbag_record.log"

        self._load_runtime_config()
        self._init_state()
        self._init_interfaces()
        self._register_signal_handlers()
        self._start_rosbag()

        self.log_event(
            "scenario_started",
            contract_file=str(contract_file),
            config_file=str(config_file),
            output_dir=str(self.output_dir),
            rosbag_enabled=self.rosbag_enabled,
        )
        self.timer = self.create_timer(self.loop_period_s, self.on_timer)

    def _load_runtime_config(self) -> None:
        topics = self.config["topics"]
        trigger = self.config["trigger"]
        injection = self.config["injection"]
        degradation = self.config["degradation"]
        recovery = self.config["recovery"]
        geometry = self.config["geometry_checks"]

        self.world_name = str(self.config["world_name"])
        self.timeout_s = float(self.config["timeout_s"])
        self.nominal_linear_speed_mps = float(self.config["nominal_linear_speed_mps"])
        self.goal_tolerance_m = float(self.config["goal_tolerance_m"])
        self.minimum_post_window_s = float(
            self.contract["minimum_post_encounter_observation_window_s"]
        )
        self.publish_at_science_rock = bool(self.config["publish_at_science_rock"])

        self.start_xy = (
            float(self.config["start_pose"]["x"]),
            float(self.config["start_pose"]["y"]),
        )
        self.goal_xy = (
            float(self.config["goal_pose"]["x"]),
            float(self.config["goal_pose"]["y"]),
        )
        self.baseline_hazard_xy = (
            float(self.config["baseline_hazard"]["x"]),
            float(self.config["baseline_hazard"]["y"]),
        )
        self.baseline_hazard_radius_m = float(
            self.config["baseline_hazard"]["radius_m"]
        )
        self.route_length_m = distance_xy(self.start_xy, self.goal_xy)

        self.injection_reference_xy = (
            float(injection["reference_pose"]["x"]),
            float(injection["reference_pose"]["y"]),
        )
        self.injection_xy = self.injection_reference_xy
        self.injection_z = float(injection["reference_pose"]["z"])
        self.injected_entity_name = str(injection["entity_name"])
        self.dynamic_pose_from_goal_bearing = bool(
            injection["dynamic_pose_from_goal_bearing"]
        )
        self.injection_spawn_distance_ahead_m = float(
            injection["spawn_distance_ahead_m"]
        )
        self.injection_detection_range_m = float(injection["detection_range_m"])
        self.injection_encounter_radius_m = float(injection["encounter_radius_m"])
        self.injection_attribution_distance_m = float(injection["attribution_distance_m"])
        self.injection_collision_distance_m = float(injection["collision_distance_m"])
        self.injection_verification_delay_s = float(injection["verification_delay_s"])
        models_share = Path(get_package_share_directory(str(injection["model_package"])))
        self.injected_model_file = models_share / str(injection["model_relative_path"])
        self.model_root = models_share / "models"

        self.trigger_min_progress_ratio = float(trigger["min_progress_ratio"])
        self.trigger_radius_m = float(trigger["trigger_radius_m"])
        self.trigger_recent_progress_window_s = float(trigger["recent_progress_window_s"])
        self.trigger_min_recent_progress_m = float(trigger["min_recent_progress_m"])
        self.trigger_min_remaining_time_s = float(trigger["min_remaining_time_s"])
        self.trigger_obstacle_clear_hold_s = float(trigger["obstacle_clear_hold_s"])
        self.trigger_require_baseline_reaction_observed = bool(
            trigger["require_baseline_reaction_observed"]
        )

        self.degradation_duration_s = float(degradation["duration_s"])
        self.degradation_publish_rate_hz = float(degradation["publish_rate_hz"])
        self.degradation_base_false_duty_cycle = float(
            degradation["base_false_duty_cycle"]
        )
        self.degradation_max_false_duty_cycle = float(
            degradation["max_false_duty_cycle"]
        )
        self.degradation_base_segment_s = float(degradation["base_segment_s"])

        self.recovery_hold_s = float(recovery["hold_s"])
        self.recovery_resumed_linear_speed_mps = float(
            recovery["resumed_linear_speed_mps"]
        )
        self.recovery_resumed_progress_delta_m = float(
            recovery["resumed_progress_delta_m"]
        )
        self.stall_timeout_s = float(recovery["stall_timeout_s"])
        self.stall_progress_window_s = float(recovery["stall_progress_window_s"])
        self.stall_min_progress_m = float(recovery["stall_min_progress_m"])

        self.front_bearing_rad = math.radians(float(geometry["front_bearing_deg"]))
        self.misclassification_front_bearing_rad = math.radians(
            float(geometry["misclassification_front_bearing_deg"])
        )
        self.misclassification_distance_m = float(
            geometry["misclassification_distance_m"]
        )

        self.odom_topic = str(topics["odom"])
        self.scan_topic = str(topics["scan"])
        self.cmd_vel_topic = str(topics["cmd_vel"])
        self.obstacle_front_topic = str(topics["obstacle_front"])
        self.obstacle_left_topic = str(topics["obstacle_left"])
        self.obstacle_right_topic = str(topics["obstacle_right"])
        self.obstacle_state_topic = str(topics["obstacle_state"])
        self.battery_soc_topic = str(topics["battery_soc"])
        self.battery_near_outpost_topic = str(topics["battery_near_outpost"])
        self.monitor_events_topic = str(topics["monitor_events"])
        self.monitor_mr009_topic = str(topics["monitor_mr009"])
        self.monitor_mr011_topic = str(topics["monitor_mr011"])
        self.at_science_rock_topic = str(topics["at_science_rock"])
        self.rosbag_topics = list(self.config["rosbag_topics"])

        self.loop_period_s = 1.0 / max(5.0, self.degradation_publish_rate_hz)

    def _init_state(self) -> None:
        self.finalized = False
        self.termination_reason: Optional[str] = None
        self.start_time_s: Optional[float] = None
        self.end_time_s: Optional[float] = None

        self.current_pose: Optional[Tuple[float, float, float]] = None
        self.current_progress_ratio = 0.0
        self.path_samples: List[Tuple[float, float, float]] = []
        self.odom_history: Deque[Tuple[float, float]] = deque(maxlen=6000)
        self.max_route_deviation_m = 0.0
        self.max_post_injection_route_deviation_m = 0.0
        self.min_distance_to_injected_m = float("inf")

        self.latest_cmd = (0.0, 0.0)
        self.last_logged_cmd: Optional[Tuple[float, float]] = None
        self.latest_scan_front_min = float("inf")
        self.latest_scan_left_min = float("inf")
        self.latest_scan_right_min = float("inf")
        self.latest_obstacle_front = False
        self.latest_obstacle_left = False
        self.latest_obstacle_right = False
        self.latest_obstacle_state = "CLEAR"
        self.latest_battery_soc: Optional[float] = None
        self.latest_battery_near_outpost: Optional[bool] = None

        self.monitor_violations = {"MR_009": False, "MR_011": False}
        self.monitor_events: List[str] = []

        self.baseline_uncertainty_exercised = False
        self.baseline_hazard_encountered = False
        self.baseline_reaction_time_s: Optional[float] = None

        self.injection_gate_diagnostics: Dict[str, Any] = {}
        self.injection_attempted = False
        self.injection_succeeded = False
        self.injection_failed_reason: Optional[str] = None
        self.injection_time_s: Optional[float] = None
        self.degradation_end_time_s: Optional[float] = None
        self.last_published_noise_state: Optional[str] = None

        self.encounter_time_s: Optional[float] = None
        self.detection_time_s: Optional[float] = None
        self.detection_signal_source: Optional[str] = None
        self.raw_scan_detection_time_s: Optional[float] = None

        self.first_reaction_time_s: Optional[float] = None
        self.first_reaction_rationale: Optional[str] = None
        self.first_reaction_scope: Optional[str] = None
        self.first_reaction_context: Optional[Dict[str, Any]] = None
        self.attributed_reaction_time_s: Optional[float] = None
        self.attributed_reaction_scope: Optional[str] = None
        self.attributed_reaction_context: Optional[Dict[str, Any]] = None
        self.recovery_time_s: Optional[float] = None
        self.recovery_candidate_start_s: Optional[float] = None
        self.reaction_progress_ratio: Optional[float] = None

        self.false_state_samples = 0
        self.total_state_samples = 0

        self.rosbag_process: Optional[subprocess.Popen[Any]] = None
        self.rosbag_log_handle = None
        self.clear_path_since_s: Optional[float] = None

    def _init_interfaces(self) -> None:
        odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        obstacle_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.create_subscription(Odometry, self.odom_topic, self.on_odom, odom_qos)
        self.create_subscription(LaserScan, self.scan_topic, self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, self.cmd_vel_topic, self.on_cmd_vel, 10)
        self.create_subscription(
            Bool, self.obstacle_front_topic, self.on_obstacle_front, obstacle_qos
        )
        self.create_subscription(
            Bool, self.obstacle_left_topic, self.on_obstacle_left, obstacle_qos
        )
        self.create_subscription(
            Bool, self.obstacle_right_topic, self.on_obstacle_right, obstacle_qos
        )
        self.create_subscription(
            String, self.obstacle_state_topic, self.on_obstacle_state, obstacle_qos
        )
        self.create_subscription(Float32, self.battery_soc_topic, self.on_battery_soc, 10)
        self.create_subscription(
            Bool, self.battery_near_outpost_topic, self.on_battery_near_outpost, 10
        )
        self.create_subscription(String, self.monitor_events_topic, self.on_monitor_event, 10)
        self.create_subscription(Bool, self.monitor_mr009_topic, self.on_monitor_mr009, 10)
        self.create_subscription(Bool, self.monitor_mr011_topic, self.on_monitor_mr011, 10)

        self.pub_obstacle_left = self.create_publisher(Bool, self.obstacle_left_topic, 10)
        self.pub_obstacle_right = self.create_publisher(Bool, self.obstacle_right_topic, 10)
        self.pub_obstacle_front = self.create_publisher(Bool, self.obstacle_front_topic, 10)
        self.pub_obstacle_state = self.create_publisher(String, self.obstacle_state_topic, 10)
        self.pub_at_science_rock = self.create_publisher(Bool, self.at_science_rock_topic, 10)

    def _register_signal_handlers(self) -> None:
        signal.signal(signal.SIGINT, self._handle_signal)
        signal.signal(signal.SIGTERM, self._handle_signal)

    def _start_rosbag(self) -> None:
        if not self.rosbag_enabled:
            self.log_event("rosbag_disabled")
            return

        bag_output = self.rosbag_dir / f"scenario_bag_{int(time.time())}"
        command = [
            "ros2",
            "bag",
            "record",
            "--use-sim-time",
            "-o",
            str(bag_output),
            *self.rosbag_topics,
        ]
        try:
            self.rosbag_log_handle = self.rosbag_log_path.open("a", encoding="utf-8")
            self.rosbag_process = subprocess.Popen(
                command,
                stdout=self.rosbag_log_handle,
                stderr=subprocess.STDOUT,
                env=os.environ.copy(),
            )
            self.log_event("rosbag_started", command=command, output=str(bag_output))
        except Exception as exc:
            self.log_event("rosbag_failed_to_start", error=str(exc), command=command)
            self.rosbag_process = None

    def _stop_rosbag(self) -> None:
        if self.rosbag_process is None:
            if self.rosbag_log_handle is not None:
                self.rosbag_log_handle.close()
                self.rosbag_log_handle = None
            return

        if self.rosbag_process.poll() is None:
            self.rosbag_process.send_signal(signal.SIGINT)
            try:
                self.rosbag_process.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                self.rosbag_process.terminate()
                try:
                    self.rosbag_process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    self.rosbag_process.kill()
                    self.rosbag_process.wait(timeout=5.0)
        self.log_event("rosbag_stopped", returncode=self.rosbag_process.returncode)
        self.rosbag_process = None
        if self.rosbag_log_handle is not None:
            self.rosbag_log_handle.close()
            self.rosbag_log_handle = None

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def elapsed_s(self) -> float:
        if self.start_time_s is None:
            return 0.0
        return max(0.0, self.now_s() - self.start_time_s)

    def goal_distance_m(self) -> Optional[float]:
        if self.current_pose is None:
            return None
        return distance_xy((self.current_pose[0], self.current_pose[1]), self.goal_xy)

    def distance_to_injected_m(self) -> Optional[float]:
        if self.current_pose is None:
            return None
        return distance_xy((self.current_pose[0], self.current_pose[1]), self.injection_xy)

    def distance_to_baseline_hazard_m(self) -> Optional[float]:
        if self.current_pose is None:
            return None
        return distance_xy(
            (self.current_pose[0], self.current_pose[1]), self.baseline_hazard_xy
        )

    def obstacle_front_active(self) -> bool:
        return bool(self.latest_obstacle_front) or (
            self.latest_scan_front_min < self.injection_detection_range_m
        )

    def lateral_obstacle_active(self) -> bool:
        return bool(self.latest_obstacle_left or self.latest_obstacle_right)

    def degradation_active(self) -> bool:
        return (
            self.injection_succeeded
            and self.degradation_end_time_s is not None
            and self.now_s() <= self.degradation_end_time_s
        )

    def goal_reached(self) -> bool:
        goal_distance = self.goal_distance_m()
        return goal_distance is not None and goal_distance <= self.goal_tolerance_m

    def recent_route_progress_m(self, window_s: float) -> float:
        if not self.odom_history:
            return 0.0

        now = self.now_s()
        earliest = None
        for sample_time, progress_ratio in reversed(self.odom_history):
            if now - sample_time > window_s:
                break
            earliest = (sample_time, progress_ratio)

        if earliest is None:
            earliest = self.odom_history[0]

        latest = self.odom_history[-1]
        return max(0.0, latest[1] - earliest[1]) * self.route_length_m

    def injected_bearing_error_rad(self) -> Optional[float]:
        if self.current_pose is None:
            return None
        dx = self.injection_xy[0] - self.current_pose[0]
        dy = self.injection_xy[1] - self.current_pose[1]
        target_yaw = math.atan2(dy, dx)
        return abs(wrap_pi(target_yaw - self.current_pose[2]))

    def near_injected_front_context(
        self, max_distance_m: float, front_bearing_rad: float
    ) -> bool:
        if not self.injection_succeeded or self.current_pose is None:
            return False
        distance_to_fault = self.distance_to_injected_m()
        bearing_error = self.injected_bearing_error_rad()
        if distance_to_fault is None or bearing_error is None:
            return False
        return (
            distance_to_fault <= max_distance_m and bearing_error <= front_bearing_rad
        )

    def baseline_context_active(self) -> bool:
        distance_to_hazard = self.distance_to_baseline_hazard_m()
        if not self.injection_succeeded and (
            self.latest_obstacle_front
            or self.latest_obstacle_left
            or self.latest_obstacle_right
            or self.baseline_reaction_time_s is not None
        ):
            return True
        return bool(
            (distance_to_hazard is not None and distance_to_hazard <= self.baseline_hazard_radius_m)
            or self.monitor_violations["MR_009"]
            or self.monitor_violations["MR_011"]
        )

    def injected_context_active(self) -> bool:
        return self.near_injected_front_context(
            self.injection_attribution_distance_m, self.front_bearing_rad
        ) or self.degradation_active()

    def active_context(self) -> Dict[str, Any]:
        return {
            "progress_ratio": round(self.current_progress_ratio, 4),
            "goal_distance_m": self._rounded_or_none(self.goal_distance_m()),
            "distance_to_injected_m": self._rounded_or_none(self.distance_to_injected_m()),
            "distance_to_baseline_hazard_m": self._rounded_or_none(
                self.distance_to_baseline_hazard_m()
            ),
            "battery_soc": self._rounded_or_none(self.latest_battery_soc),
            "battery_near_outpost": self.latest_battery_near_outpost,
            "obstacle_front": self.latest_obstacle_front,
            "obstacle_left": self.latest_obstacle_left,
            "obstacle_right": self.latest_obstacle_right,
            "obstacle_state": self.latest_obstacle_state,
            "degradation_active": self.degradation_active(),
            "monitor_violations": self.monitor_violations.copy(),
        }

    def classify_reaction_scope(self) -> str:
        baseline_active = self.baseline_context_active()
        injected_active = self.injected_context_active()
        if baseline_active and injected_active:
            return "baseline_and_injected"
        if injected_active:
            return "injected_only"
        if baseline_active:
            return "baseline_only"
        return "indeterminate"

    def classify_control_rationale(self, linear_x: float, angular_z: float) -> str:
        if any(self.monitor_violations.values()) and abs(linear_x) < 0.05:
            return "monitor_enforcement"

        obstacle_context = self.obstacle_front_active() or self.lateral_obstacle_active()
        if obstacle_context and (
            linear_x < self.nominal_linear_speed_mps * 0.75
            or abs(angular_z) > 0.35
            or linear_x < 0.0
        ):
            return "obstacle_avoidance"

        if abs(angular_z) > 0.25 or linear_x < self.nominal_linear_speed_mps * 0.9:
            return "goal_alignment"

        return "unknown"

    def is_detection_attributable(self) -> bool:
        return self.near_injected_front_context(
            self.injection_attribution_distance_m, self.front_bearing_rad
        )

    def is_reaction_attributable(self, rationale: str, scope: str) -> bool:
        return rationale == "obstacle_avoidance" and scope in {
            "injected_only",
            "baseline_and_injected",
        }

    def on_odom(self, msg: Odometry) -> None:
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        if self.start_time_s is None:
            self.start_time_s = self.now_s()

        self.current_pose = (x, y, yaw)
        self.current_progress_ratio = line_progress_ratio((x, y), self.start_xy, self.goal_xy)
        timestamp = self.now_s()
        self.odom_history.append((timestamp, self.current_progress_ratio))
        self.path_samples.append((timestamp, x, y))

        route_deviation = point_to_route_distance((x, y), self.start_xy, self.goal_xy)
        self.max_route_deviation_m = max(self.max_route_deviation_m, route_deviation)
        if self.injection_time_s is not None:
            self.max_post_injection_route_deviation_m = max(
                self.max_post_injection_route_deviation_m, route_deviation
            )

        if self.distance_to_baseline_hazard_m() is not None:
            if self.distance_to_baseline_hazard_m() <= self.baseline_hazard_radius_m:
                if not self.baseline_hazard_encountered:
                    self.baseline_hazard_encountered = True
                    self.baseline_uncertainty_exercised = True
                    self.log_event("baseline_reaction_observed", source="baseline_hazard_zone")

        if self.injection_succeeded:
            distance_to_fault = self.distance_to_injected_m()
            if distance_to_fault is not None:
                self.min_distance_to_injected_m = min(
                    self.min_distance_to_injected_m, distance_to_fault
                )
                if (
                    self.encounter_time_s is None
                    and distance_to_fault <= self.injection_encounter_radius_m
                ):
                    self.encounter_time_s = timestamp
                    self.log_event(
                        "fault_encountered",
                        distance_to_fault_m=round(distance_to_fault, 3),
                    )

    def on_scan(self, msg: LaserScan) -> None:
        front = sector_min(msg, math.radians(-20.0), math.radians(20.0))
        left = sector_min(msg, math.radians(20.0), math.radians(75.0))
        right = sector_min(msg, math.radians(-75.0), math.radians(-20.0))

        self.latest_scan_front_min = front
        self.latest_scan_left_min = left
        self.latest_scan_right_min = right

        if (
            self.injection_succeeded
            and self.raw_scan_detection_time_s is None
            and front < self.injection_detection_range_m
            and self.is_detection_attributable()
        ):
            self.raw_scan_detection_time_s = self.now_s()
            self.log_event(
                "raw_scan_detection_attributed",
                obstacle_distance_m=round(front, 3),
            )

    def on_cmd_vel(self, msg: Twist) -> None:
        self.latest_cmd = (float(msg.linear.x), float(msg.angular.z))

    def on_obstacle_front(self, msg: Bool) -> None:
        self.latest_obstacle_front = bool(msg.data)
        if self.latest_obstacle_front and self.detection_time_s is None and self.is_detection_attributable():
            self.detection_time_s = self.now_s()
            self.detection_signal_source = self.obstacle_front_topic
            self.log_event(
                "fault_detection_attributed",
                source=self.obstacle_front_topic,
                distance_to_fault_m=self._rounded_or_none(self.distance_to_injected_m()),
            )

    def on_obstacle_left(self, msg: Bool) -> None:
        self.latest_obstacle_left = bool(msg.data)

    def on_obstacle_right(self, msg: Bool) -> None:
        self.latest_obstacle_right = bool(msg.data)

    def on_obstacle_state(self, msg: String) -> None:
        self.latest_obstacle_state = str(msg.data)

        if (
            self.detection_time_s is None
            and self.latest_obstacle_state in {"FRONT", "FRONT_LEFT", "FRONT_RIGHT"}
            and self.is_detection_attributable()
        ):
            self.detection_time_s = self.now_s()
            self.detection_signal_source = self.obstacle_state_topic
            self.log_event(
                "fault_detection_attributed",
                source=self.obstacle_state_topic,
                state=self.latest_obstacle_state,
                distance_to_fault_m=self._rounded_or_none(self.distance_to_injected_m()),
            )

        if (
            self.degradation_active()
            and self.near_injected_front_context(
                self.misclassification_distance_m,
                self.misclassification_front_bearing_rad,
            )
        ):
            self.total_state_samples += 1
            if self.latest_obstacle_state in {"LEFT", "RIGHT"}:
                self.false_state_samples += 1

    def on_battery_soc(self, msg: Float32) -> None:
        self.latest_battery_soc = float(msg.data)

    def on_battery_near_outpost(self, msg: Bool) -> None:
        self.latest_battery_near_outpost = bool(msg.data)

    def on_monitor_event(self, msg: String) -> None:
        event = str(msg.data)
        self.monitor_events.append(event)
        self.log_event("baseline_monitor_violation_active", event=event)
        if "MR_009" in event:
            self.monitor_violations["MR_009"] = True
        if "MR_011" in event:
            self.monitor_violations["MR_011"] = True

    def on_monitor_mr009(self, msg: Bool) -> None:
        if msg.data:
            self.monitor_violations["MR_009"] = True
            self.log_event("baseline_monitor_violation_active", event="MR_009")

    def on_monitor_mr011(self, msg: Bool) -> None:
        if msg.data:
            self.monitor_violations["MR_011"] = True
            self.log_event("baseline_monitor_violation_active", event="MR_011")

    def on_timer(self) -> None:
        if self.finalized:
            return

        self.update_clear_path_state()
        self.publish_science_rock_status()
        self.track_control_changes()
        self.maybe_inject_fault()
        self.publish_degradation_noise()
        self.maybe_record_recovery()

        if self.goal_reached():
            self.finalize("goal_reached")
            return

        if self.elapsed_s() >= self.timeout_s:
            self.finalize("timeout")
            return

        if (
            self.encounter_time_s is not None
            and self.now_s() - self.encounter_time_s >= self.stall_timeout_s
            and self.recent_route_progress_m(self.stall_progress_window_s)
            < self.stall_min_progress_m
        ):
            self.finalize("stalled_after_encounter")

    def publish_science_rock_status(self) -> None:
        if not self.publish_at_science_rock:
            return
        self.pub_at_science_rock.publish(Bool(data=self.goal_reached()))

    def update_clear_path_state(self) -> None:
        path_clear = (
            not self.latest_obstacle_front
            and not self.latest_obstacle_left
            and not self.latest_obstacle_right
            and self.latest_obstacle_state == "CLEAR"
        )
        if path_clear:
            if self.clear_path_since_s is None:
                self.clear_path_since_s = self.now_s()
        else:
            self.clear_path_since_s = None

    def track_control_changes(self) -> None:
        linear_x, angular_z = self.latest_cmd
        if self.last_logged_cmd is None:
            self.last_logged_cmd = (linear_x, angular_z)
            return

        lin_delta = abs(linear_x - self.last_logged_cmd[0])
        ang_delta = abs(angular_z - self.last_logged_cmd[1])
        if lin_delta < 0.15 and ang_delta < 0.2:
            return

        rationale = self.classify_control_rationale(linear_x, angular_z)
        scope = self.classify_reaction_scope()
        attributed = self.is_reaction_attributable(rationale, scope)
        context = self.active_context()

        self.log_event(
            "cmd_vel_changed",
            linear_x=round(linear_x, 3),
            angular_z=round(angular_z, 3),
        )
        self.log_event(
            "control_rationale_classified",
            observed_control_rationale=rationale,
            reaction_scope=scope,
            reaction_attribution_status=attributed,
            active_context_at_reaction=context,
        )

        if rationale == "obstacle_avoidance":
            if self.injection_time_s is None and self.baseline_reaction_time_s is None:
                self.baseline_reaction_time_s = self.now_s()
                self.baseline_uncertainty_exercised = True
                self.log_event("baseline_reaction_observed", source="cmd_vel")

            if self.injection_time_s is not None:
                if self.first_reaction_time_s is None:
                    self.first_reaction_time_s = self.now_s()
                    self.first_reaction_rationale = rationale
                    self.first_reaction_scope = scope
                    self.first_reaction_context = context
                    self.reaction_progress_ratio = self.current_progress_ratio
                    self.log_event(
                        "reaction_candidate",
                        observed_control_rationale=rationale,
                        reaction_scope=scope,
                        reaction_attribution_status=attributed,
                    )
                if attributed and self.attributed_reaction_time_s is None:
                    self.attributed_reaction_time_s = self.now_s()
                    self.attributed_reaction_scope = scope
                    self.attributed_reaction_context = context
                    self.log_event(
                        "reaction_attributed",
                        observed_control_rationale=rationale,
                        reaction_scope=scope,
                    )
                elif not attributed:
                    self.log_event(
                        "reaction_not_attributed",
                        observed_control_rationale=rationale,
                        reaction_scope=scope,
                    )

        self.last_logged_cmd = (linear_x, angular_z)

    def maybe_record_recovery(self) -> None:
        if self.first_reaction_time_s is None or self.recovery_time_s is not None:
            return

        linear_x, _ = self.latest_cmd
        recovery_ready = (
            linear_x >= self.recovery_resumed_linear_speed_mps
            and not self.obstacle_front_active()
            and self.reaction_progress_ratio is not None
            and (
                self.current_progress_ratio - self.reaction_progress_ratio
            )
            * self.route_length_m
            >= self.recovery_resumed_progress_delta_m
        )

        if recovery_ready:
            if self.recovery_candidate_start_s is None:
                self.recovery_candidate_start_s = self.now_s()
            elif self.now_s() - self.recovery_candidate_start_s >= self.recovery_hold_s:
                self.recovery_time_s = self.now_s()
                self.log_event(
                    "meaningful_evaluation_window_satisfied",
                    recovery_time_s=round(self.recovery_time_s, 3),
                )
        else:
            self.recovery_candidate_start_s = None

    def maybe_inject_fault(self) -> None:
        if self.injection_attempted or self.current_pose is None or self.start_time_s is None:
            return

        recent_progress_m = self.recent_route_progress_m(self.trigger_recent_progress_window_s)
        distance_to_fault = distance_xy(
            (self.current_pose[0], self.current_pose[1]), self.injection_reference_xy
        )
        remaining_time_s = max(0.0, self.timeout_s - self.elapsed_s())
        baseline_monitor_active = any(self.monitor_violations.values())
        clear_path_hold_s = (
            0.0 if self.clear_path_since_s is None else self.now_s() - self.clear_path_since_s
        )

        self.injection_gate_diagnostics = {
            "progress_ratio": round(self.current_progress_ratio, 4),
            "distance_to_fault_m": self._rounded_or_none(distance_to_fault),
            "recent_progress_m": round(recent_progress_m, 3),
            "remaining_time_s": round(remaining_time_s, 3),
            "baseline_monitor_active": baseline_monitor_active,
            "baseline_reaction_observed": self.baseline_reaction_time_s is not None,
            "clear_path_hold_s": round(clear_path_hold_s, 3),
        }

        if (
            self.current_progress_ratio < self.trigger_min_progress_ratio
            or distance_to_fault is None
            or distance_to_fault > self.trigger_radius_m
            or recent_progress_m < self.trigger_min_recent_progress_m
            or remaining_time_s < self.trigger_min_remaining_time_s
            or clear_path_hold_s < self.trigger_obstacle_clear_hold_s
            or (
                self.trigger_require_baseline_reaction_observed
                and self.baseline_reaction_time_s is None
            )
        ):
            return

        self.injection_attempted = True
        self.injection_xy = self.compute_injection_pose()
        self.log_event(
            "fault_injection_gate_passed",
            actual_injection_pose={"x": round(self.injection_xy[0], 3), "y": round(self.injection_xy[1], 3)},
            **self.injection_gate_diagnostics,
        )
        self.spawn_injected_obstacle()

    def compute_injection_pose(self) -> Tuple[float, float]:
        if self.current_pose is None or not self.dynamic_pose_from_goal_bearing:
            return self.injection_reference_xy
        goal_bearing = math.atan2(
            self.goal_xy[1] - self.current_pose[1],
            self.goal_xy[0] - self.current_pose[0],
        )
        return (
            self.current_pose[0] + self.injection_spawn_distance_ahead_m * math.cos(goal_bearing),
            self.current_pose[1] + self.injection_spawn_distance_ahead_m * math.sin(goal_bearing),
        )

    def spawn_injected_obstacle(self) -> None:
        env = os.environ.copy()
        resource_roots = [
            env.get("GZ_SIM_RESOURCE_PATH", ""),
            str(self.model_root),
        ]
        env["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join(
            [entry for entry in resource_roots if entry]
        )
        sdf_roots = [
            env.get("SDF_PATH", ""),
            str(self.model_root),
        ]
        env["SDF_PATH"] = os.pathsep.join([entry for entry in sdf_roots if entry])

        command = [
            "ros2",
            "run",
            "ros_gz_sim",
            "create",
            "-world",
            self.world_name,
            "-name",
            self.injected_entity_name,
            "-allow_renaming",
            "true",
            "-file",
            str(self.injected_model_file),
            "-x",
            str(self.injection_xy[0]),
            "-y",
            str(self.injection_xy[1]),
            "-z",
            str(self.injection_z),
        ]

        try:
            result = subprocess.run(
                command,
                env=env,
                check=False,
                capture_output=True,
                text=True,
                timeout=20.0,
            )
        except Exception as exc:
            self.injection_failed_reason = f"spawn command failed: {exc}"
            self.log_event("fault_injected", success=False, error=self.injection_failed_reason)
            return

        time.sleep(self.injection_verification_delay_s)
        verified = self.verify_injected_obstacle(env)
        if result.returncode == 0 and verified:
            self.injection_succeeded = True
            self.injection_time_s = self.now_s()
            self.degradation_end_time_s = self.injection_time_s + self.degradation_duration_s
            self.log_event(
                "fault_injected",
                success=True,
                command=command,
                stdout=result.stdout.strip(),
                stderr=result.stderr.strip(),
                effective_resource_path=env["GZ_SIM_RESOURCE_PATH"],
            )
            return

        self.injection_failed_reason = (
            f"spawn_returncode={result.returncode} verified={verified}"
        )
        self.log_event(
            "fault_injected",
            success=False,
            command=command,
            stdout=result.stdout.strip(),
            stderr=result.stderr.strip(),
            effective_resource_path=env["GZ_SIM_RESOURCE_PATH"],
            verification=verified,
        )

    def verify_injected_obstacle(self, env: Dict[str, str]) -> bool:
        commands = [
            ["gz", "model", "--list", "-w", self.world_name],
            ["gz", "model", "--list"],
        ]
        for command in commands:
            try:
                result = subprocess.run(
                    command,
                    env=env,
                    check=False,
                    capture_output=True,
                    text=True,
                    timeout=10.0,
                )
            except Exception as exc:
                self.log_event(
                    "fault_detection_rejected",
                    reason="verification_command_error",
                    command=command,
                    error=str(exc),
                )
                continue

            if result.returncode == 0 and self.injected_entity_name in result.stdout:
                return True
        return False

    def publish_degradation_noise(self) -> None:
        if not self.degradation_active():
            return

        assert self.injection_time_s is not None
        elapsed_since_injection = max(0.0, self.now_s() - self.injection_time_s)
        intensity = clamp(elapsed_since_injection / self.degradation_duration_s, 0.0, 1.0)
        duty_cycle = self.degradation_base_false_duty_cycle + intensity * (
            self.degradation_max_false_duty_cycle
            - self.degradation_base_false_duty_cycle
        )
        segment_s = max(
            0.12, self.degradation_base_segment_s - 0.2 * intensity
        )
        phase_position = (elapsed_since_injection / segment_s) % 4.0
        front_visible = self.near_injected_front_context(
            self.injection_detection_range_m + 2.0,
            self.front_bearing_rad * 1.25,
        )

        noise_state = "CLEAR"
        if front_visible and phase_position < duty_cycle * 2.8:
            front_states = ("FRONT_LEFT", "FRONT", "FRONT_RIGHT", "FRONT")
            noise_state = front_states[int(phase_position) % len(front_states)]
        elif phase_position < duty_cycle * 2.0:
            noise_state = "LEFT" if int(phase_position) % 2 == 0 else "RIGHT"

        front = noise_state in {"FRONT", "FRONT_LEFT", "FRONT_RIGHT"}
        left = noise_state in {"LEFT", "FRONT_LEFT"}
        right = noise_state in {"RIGHT", "FRONT_RIGHT"}

        self.pub_obstacle_front.publish(Bool(data=front))
        self.pub_obstacle_left.publish(Bool(data=left))
        self.pub_obstacle_right.publish(Bool(data=right))
        self.pub_obstacle_state.publish(String(data=noise_state))

        # Keep attribution analysis aligned with the injected observation values.
        self.latest_obstacle_front = front
        self.latest_obstacle_left = left
        self.latest_obstacle_right = right
        self.latest_obstacle_state = noise_state

        if self.detection_time_s is None and front and front_visible:
            self.detection_time_s = self.now_s()
            self.detection_signal_source = self.obstacle_front_topic
            self.log_event(
                "fault_detection_attributed",
                source=self.obstacle_front_topic,
                state=noise_state,
                distance_to_fault_m=self._rounded_or_none(self.distance_to_injected_m()),
            )

        if noise_state != self.last_published_noise_state:
            self.last_published_noise_state = noise_state
            self.log_event(
                "fault_detection_candidate",
                injected_uncertainty_source="degraded_obstacle_classification",
                published_state=noise_state,
                duty_cycle=round(duty_cycle, 3),
            )

    def finalize(self, reason: str) -> None:
        if self.finalized:
            return

        self.finalized = True
        self.termination_reason = reason
        self.end_time_s = self.now_s()
        evaluation_window_s = self.compute_evaluation_window_after_encounter()
        meaningful_window_satisfied = self.meaningful_evaluation_window_satisfied(
            evaluation_window_s
        )

        if meaningful_window_satisfied:
            self.log_event(
                "meaningful_evaluation_window_satisfied",
                evaluation_window_after_encounter_s=self._rounded_or_none(
                    evaluation_window_s
                ),
            )
        else:
            self.log_event(
                "meaningful_evaluation_window_not_satisfied",
                evaluation_window_after_encounter_s=self._rounded_or_none(
                    evaluation_window_s
                ),
            )

        metrics = self.build_metrics(reason, evaluation_window_s, meaningful_window_satisfied)
        self.write_metrics(metrics)
        self.write_report(metrics)
        self._stop_rosbag()
        self.timeline_handle.close()
        self.get_logger().info(
            f"Scenario finalised with reason={reason}, report={self.report_path}"
        )
        rclpy.try_shutdown()

    def build_metrics(
        self,
        reason: str,
        evaluation_window_s: Optional[float],
        meaningful_window_satisfied: bool,
    ) -> Dict[str, Any]:
        goal_reached = self.goal_reached()
        collision_with_injected_obstacle = (
            self.min_distance_to_injected_m <= self.injection_collision_distance_m
            if math.isfinite(self.min_distance_to_injected_m)
            else False
        )
        injected_encountered = self.encounter_time_s is not None
        baseline_confound_status = self.classify_reaction_scope()

        safety_preservation = {
            "MR_009_RETURN_TO_OUTPOST_ON_LOW_BATTERY": not self.monitor_violations["MR_009"],
            "MR_011_MAINTAIN_SPEED_WHEN_FULL_BATTERY": not self.monitor_violations["MR_011"],
            "collision_with_injected_obstacle": not collision_with_injected_obstacle,
        }
        goal_viability = {
            "science_rock_01_reached": goal_reached,
            "mission_deadline_met": reason != "timeout" and goal_reached,
        }

        injected_outcome = self.classify_injected_outcome(
            meaningful_window_satisfied=meaningful_window_satisfied,
            goal_reached=goal_reached,
            collision_with_injected_obstacle=collision_with_injected_obstacle,
            injected_encountered=injected_encountered,
        )
        baseline_outcome = self.classify_baseline_outcome()

        return {
            "scenario_name": self.scenario_name,
            "termination_reason": reason,
            "started_at_sim_s": self._rounded_or_none(self.start_time_s),
            "finished_at_sim_s": self._rounded_or_none(self.end_time_s),
            "runtime_s": self._rounded_or_none(
                None if self.start_time_s is None or self.end_time_s is None else self.end_time_s - self.start_time_s
            ),
            "injection_succeeded": self.injection_succeeded,
            "injection_failed_reason": self.injection_failed_reason,
            "progress_ratio_at_injection": self.injection_gate_diagnostics.get("progress_ratio"),
            "core_metrics": {
                "adaptation_speed_ms": self.delta_ms(
                    self.injection_time_s, self.attributed_reaction_time_s
                ),
                "obstacle_detection_latency_ms": self.delta_ms(
                    self.injection_time_s, self.detection_time_s
                ),
                "autonomy_reaction_status": self.first_reaction_time_s is not None,
                "detection_signal_source": self.detection_signal_source,
                "observed_control_rationale": self.first_reaction_rationale,
                "reaction_scope": self.attributed_reaction_scope or self.first_reaction_scope,
                "reaction_attribution_status": self.attributed_reaction_time_s is not None,
                "active_context_at_reaction": self.attributed_reaction_context
                or self.first_reaction_context,
                "safety_preservation": safety_preservation,
                "goal_viability": goal_viability,
                "recovery_rate_ms": self.delta_ms(
                    self.first_reaction_time_s, self.recovery_time_s
                ),
                "route_deviation_m": round(self.max_route_deviation_m, 3),
                "detection_attribution_status": self.detection_time_s is not None,
                "minimum_fault_distance_at_detection_m": self._rounded_or_none(
                    self.min_distance_to_injected_m
                ),
                "baseline_confound_status": baseline_confound_status,
                "injected_uncertainty_encounter_status": injected_encountered,
                "evaluation_window_after_encounter_s": self._rounded_or_none(
                    evaluation_window_s
                ),
                "baseline_uncertainty_exercised_status": self.baseline_uncertainty_exercised,
                "attribution_scope": baseline_confound_status,
                "injected_uncertainty_source": [
                    "runtime_blocking_rock",
                    "degraded_obstacle_classification",
                ],
            },
            "additional_metrics": {
                "raw_scan_detection_latency_ms": self.delta_ms(
                    self.injection_time_s, self.raw_scan_detection_time_s
                ),
                "false_obstacle_rate": self.compute_false_obstacle_rate(),
                "post_injection_route_deviation_m": round(
                    self.max_post_injection_route_deviation_m, 3
                ),
                "progress_ratio_at_injection": self.injection_gate_diagnostics.get(
                    "progress_ratio"
                ),
            },
            "events": {
                "injection_time_s": self._rounded_or_none(self.injection_time_s),
                "encounter_time_s": self._rounded_or_none(self.encounter_time_s),
                "detection_time_s": self._rounded_or_none(self.detection_time_s),
                "raw_scan_detection_time_s": self._rounded_or_none(
                    self.raw_scan_detection_time_s
                ),
                "first_reaction_time_s": self._rounded_or_none(self.first_reaction_time_s),
                "attributed_reaction_time_s": self._rounded_or_none(
                    self.attributed_reaction_time_s
                ),
                "recovery_time_s": self._rounded_or_none(self.recovery_time_s),
            },
            "outcomes": {
                "goal_status": "PASS" if goal_reached else "FAIL",
                "safety_status": self.classify_safety_status(collision_with_injected_obstacle),
                "autonomy_assessment": injected_outcome,
                "baseline_outcome_assessment": baseline_outcome,
                "injected_outcome_assessment": injected_outcome,
                "outcome_assessment": injected_outcome,
            },
            "observability_gaps": [
                "Collision is inferred from rover-to-injected-rock distance; there is no direct baseline collision topic in the stack.",
                "The baseline stack does not publish /at_science_rock, so the scenario driver emits a derived status topic for monitor observability.",
            ],
            "artifact_paths": {
                "report": str(self.report_path),
                "metrics": str(self.metrics_path),
                "timeline": str(self.timeline_path),
                "rosbags": str(self.rosbag_dir),
            },
        }

    def classify_safety_status(self, collision_with_injected_obstacle: bool) -> str:
        if collision_with_injected_obstacle:
            return "FAIL"
        if any(self.monitor_violations.values()):
            return "DEGRADED"
        return "PASS"

    def classify_baseline_outcome(self) -> str:
        if self.monitor_violations["MR_009"] or self.monitor_violations["MR_011"]:
            return "DEGRADED"
        if self.baseline_uncertainty_exercised:
            return "PASS"
        return "NOT_EVALUATED"

    def classify_injected_outcome(
        self,
        meaningful_window_satisfied: bool,
        goal_reached: bool,
        collision_with_injected_obstacle: bool,
        injected_encountered: bool,
    ) -> str:
        if not self.injection_succeeded:
            return "UNTESTED"
        if not injected_encountered:
            return "UNTESTED"
        if not meaningful_window_satisfied:
            return "INCONCLUSIVE"
        if collision_with_injected_obstacle:
            return "FAIL"
        if self.attributed_reaction_time_s is None:
            return "FAIL"
        if goal_reached and not any(self.monitor_violations.values()):
            return "PASS"
        if goal_reached:
            return "DEGRADED"
        return "FAIL"

    def compute_evaluation_window_after_encounter(self) -> Optional[float]:
        if self.encounter_time_s is None or self.end_time_s is None:
            return None
        return max(0.0, self.end_time_s - self.encounter_time_s)

    def meaningful_evaluation_window_satisfied(
        self, evaluation_window_s: Optional[float]
    ) -> bool:
        if evaluation_window_s is None:
            return False
        return (
            evaluation_window_s >= self.minimum_post_window_s
            or self.recovery_time_s is not None
        )

    def compute_false_obstacle_rate(self) -> Optional[float]:
        if self.total_state_samples <= 0:
            return None
        return round(self.false_state_samples / self.total_state_samples, 4)

    def delta_ms(
        self, start_time_s: Optional[float], end_time_s: Optional[float]
    ) -> Optional[float]:
        if start_time_s is None or end_time_s is None:
            return None
        return round((end_time_s - start_time_s) * 1000.0, 3)

    def write_metrics(self, metrics: Dict[str, Any]) -> None:
        with self.metrics_path.open("w", encoding="utf-8") as handle:
            json.dump(metrics, handle, indent=2, sort_keys=False)
            handle.write("\n")

    def write_report(self, metrics: Dict[str, Any]) -> None:
        lines = [
            f"# SpaceTry Scenario Report: {self.scenario_name}",
            "",
            "## Scenario Summary",
            f"- Termination reason: {metrics['termination_reason']}",
            f"- Injection succeeded: {metrics['injection_succeeded']}",
            f"- Goal status: {metrics['outcomes']['goal_status']}",
            f"- Safety status: {metrics['outcomes']['safety_status']}",
            f"- Autonomy assessment: {metrics['outcomes']['autonomy_assessment']}",
            f"- Baseline outcome assessment: {metrics['outcomes']['baseline_outcome_assessment']}",
            f"- Injected outcome assessment: {metrics['outcomes']['injected_outcome_assessment']}",
            "",
            "## Baseline And Injected Uncertainty",
            f"- Baseline hazard exercised: {metrics['core_metrics']['baseline_uncertainty_exercised_status']}",
            f"- Injection encounter status: {metrics['core_metrics']['injected_uncertainty_encounter_status']}",
            f"- Reaction scope: {metrics['core_metrics']['reaction_scope']}",
            f"- Reaction attribution status: {metrics['core_metrics']['reaction_attribution_status']}",
            "",
            "## Key Metrics",
            f"- Obstacle detection latency (ms): {metrics['core_metrics']['obstacle_detection_latency_ms']}",
            f"- Raw scan detection latency (ms): {metrics['additional_metrics']['raw_scan_detection_latency_ms']}",
            f"- Adaptation speed (ms): {metrics['core_metrics']['adaptation_speed_ms']}",
            f"- Recovery rate (ms): {metrics['core_metrics']['recovery_rate_ms']}",
            f"- False obstacle rate: {metrics['additional_metrics']['false_obstacle_rate']}",
            f"- Route deviation (m): {metrics['core_metrics']['route_deviation_m']}",
            f"- Post-injection route deviation (m): {metrics['additional_metrics']['post_injection_route_deviation_m']}",
            f"- Evaluation window after encounter (s): {metrics['core_metrics']['evaluation_window_after_encounter_s']}",
            "",
            "## Safety And Goal Viability",
            f"- Safety preservation: {json.dumps(metrics['core_metrics']['safety_preservation'])}",
            f"- Goal viability: {json.dumps(metrics['core_metrics']['goal_viability'])}",
            "",
            "## Traceability Notes",
            f"- Detection signal source: {metrics['core_metrics']['detection_signal_source']}",
            f"- Active context at reaction: {json.dumps(metrics['core_metrics']['active_context_at_reaction'])}",
            f"- Injection gate diagnostics: {json.dumps(self.injection_gate_diagnostics)}",
            "",
            "## Observability Gaps",
            *[f"- {gap}" for gap in metrics["observability_gaps"]],
            "",
            "## Artifact Paths",
            f"- Metrics JSON: {self.metrics_path}",
            f"- Runtime timeline: {self.timeline_path}",
            f"- Rosbags: {self.rosbag_dir}",
        ]
        with self.report_path.open("w", encoding="utf-8") as handle:
            handle.write("\n".join(lines) + "\n")

    def log_event(self, event_name: str, **details: Any) -> None:
        entry = {
            "event": event_name,
            "sim_time_s": round(self.now_s(), 3),
            "wall_time_s": round(time.time(), 3),
            "details": details,
        }
        self.timeline_handle.write(json.dumps(entry, sort_keys=False) + "\n")
        self.timeline_handle.flush()

    def _handle_signal(self, signum: int, _frame: Any) -> None:
        signame = signal.Signals(signum).name.lower()
        self.finalize(f"interrupted_{signame}")

    def _rounded_or_none(self, value: Optional[float]) -> Optional[float]:
        if value is None:
            return None
        return round(value, 3)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = ScenarioDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    except AttributeError as exc:
        if "trigger" not in str(exc):
            raise
    finally:
        if not node.finalized:
            node.finalize("shutdown")
        node.destroy_node()


if __name__ == "__main__":
    main()
