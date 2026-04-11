#!/usr/bin/env python3
import json
import math
import os
import signal
import subprocess
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Tuple

import rclpy
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def yaw_from_quaternion(msg: Any) -> float:
    siny_cosp = 2.0 * (msg.w * msg.z + msg.x * msg.y)
    cosy_cosp = 1.0 - 2.0 * (msg.y * msg.y + msg.z * msg.z)
    return math.atan2(siny_cosp, cosy_cosp)


def point_distance(ax: float, ay: float, bx: float, by: float) -> float:
    return math.hypot(ax - bx, ay - by)


def iso_utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def safe_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


class ScenarioDriverNode(Node):
    def __init__(self) -> None:
        super().__init__("scenario_driver")

        self.declare_parameter("scenario_contract_file", "")
        self.declare_parameter("scenario_config_file", "")
        self.declare_parameter("output_root", "/ws/log/scenario_obstacle_sensing_stress")
        self.declare_parameter("run_timeout_s", 1200.0)
        self.declare_parameter("goal_name", "science_rock_01")
        self.declare_parameter("goal_x", 50.10)
        self.declare_parameter("goal_y", -80.0)
        self.declare_parameter("goal_tolerance_m", 1.5)

        self.scenario_contract_file = str(self.get_parameter("scenario_contract_file").value)
        self.scenario_config_file = str(self.get_parameter("scenario_config_file").value)
        self.output_root = Path(str(self.get_parameter("output_root").value))
        self.run_timeout_s = safe_float(self.get_parameter("run_timeout_s").value, 1200.0)
        self.goal_name = str(self.get_parameter("goal_name").value)
        self.goal_x = safe_float(self.get_parameter("goal_x").value, 50.10)
        self.goal_y = safe_float(self.get_parameter("goal_y").value, -80.0)
        self.goal_tolerance_m = safe_float(self.get_parameter("goal_tolerance_m").value, 1.5)

        self.contract = self._load_yaml(self.scenario_contract_file)
        self.config = self._load_yaml(self.scenario_config_file)

        self.scenario_name = str(self.config.get("scenario_name", "obstacle_sensing_stress"))
        self.output_cfg = self.config.get("output", {})
        self.topics = self.config.get("topics", {})
        self.metrics_cfg = self.config.get("metrics", {})
        self.trigger_cfg = self.config.get("trigger", {})
        self.injection_cfg = self.config.get("injection", {})
        self.encounter_cfg = self.config.get("encounter", {})
        self.evaluation_cfg = self.config.get("evaluation", {})
        self.baseline_map_cfg = self.config.get("baseline_map", {})
        self.mission_cfg = self.config.get("mission", {})

        self.start_x = safe_float(self.mission_cfg.get("start", {}).get("x"), 0.0)
        self.start_y = safe_float(self.mission_cfg.get("start", {}).get("y"), 0.0)
        self.goal_x = safe_float(self.mission_cfg.get("goal", {}).get("x"), self.goal_x)
        self.goal_y = safe_float(self.mission_cfg.get("goal", {}).get("y"), self.goal_y)
        self.goal_tolerance_m = safe_float(
            self.mission_cfg.get("goal", {}).get("tolerance_m"), self.goal_tolerance_m
        )
        self.run_timeout_s = safe_float(self.mission_cfg.get("timeout_s"), self.run_timeout_s)

        self.block_island_x = safe_float(self.baseline_map_cfg.get("block_island", {}).get("x"), 20.0)
        self.block_island_y = safe_float(self.baseline_map_cfg.get("block_island", {}).get("y"), -40.5)
        self.block_island_radius_m = safe_float(
            self.baseline_map_cfg.get("block_island", {}).get("confound_radius_m"), 16.0
        )
        self.outpost_x = safe_float(self.baseline_map_cfg.get("outpost", {}).get("x"), 66.0)
        self.outpost_y = safe_float(self.baseline_map_cfg.get("outpost", {}).get("y"), 0.0)

        self.obstacle_threshold_m = safe_float(self.metrics_cfg.get("obstacle_threshold_m"), 9.0)
        self.route_sample_distance_m = safe_float(self.metrics_cfg.get("route_sample_distance_m"), 0.75)
        self.post_goal_observation_s = safe_float(self.mission_cfg.get("post_goal_observation_s"), 5.0)

        self.trigger_min_progress_ratio = safe_float(self.trigger_cfg.get("min_progress_ratio"), 0.42)
        self.trigger_min_progress_delta = safe_float(self.trigger_cfg.get("min_progress_delta"), 0.03)
        self.trigger_progress_window_s = safe_float(self.trigger_cfg.get("progress_window_s"), 6.0)
        self.trigger_clear_window_s = safe_float(self.trigger_cfg.get("clear_window_s"), 4.0)
        self.trigger_min_remaining_time_s = safe_float(self.trigger_cfg.get("min_remaining_time_s"), 120.0)
        self.trigger_min_distance_to_goal_m = safe_float(self.trigger_cfg.get("min_distance_to_goal_m"), 18.0)
        self.trigger_min_distance_from_block_island_m = safe_float(
            self.trigger_cfg.get("min_distance_from_block_island_m"), 14.0
        )

        obstacle_cfg = self.injection_cfg.get("obstacle", {})
        degradation_cfg = self.injection_cfg.get("sensing_degradation", {})
        self.spawn_distance_ahead_m = safe_float(obstacle_cfg.get("spawn_distance_ahead_m"), 12.0)
        self.spawn_z = safe_float(obstacle_cfg.get("spawn_z"), 0.0)
        self.min_sep_goal_m = safe_float(obstacle_cfg.get("min_separation_from_goal_m"), 8.0)
        self.min_sep_outpost_m = safe_float(obstacle_cfg.get("min_separation_from_outpost_m"), 12.0)
        self.min_sep_block_island_m = safe_float(
            obstacle_cfg.get("min_separation_from_block_island_m"), 14.0
        )
        self.injected_model_key = str(obstacle_cfg.get("model_name", "rock_5"))

        self.degradation_publish_hz = safe_float(degradation_cfg.get("publish_hz"), 20.0)
        self.suppression_max_s = safe_float(degradation_cfg.get("suppression_max_s"), 6.0)
        self.degradation_activation_distance_m = safe_float(
            degradation_cfg.get("activation_distance_m"), 9.5
        )
        self.noise_duration_s = safe_float(degradation_cfg.get("noise_duration_s"), 12.0)
        self.alternation_period_s = safe_float(degradation_cfg.get("alternation_period_s"), 0.75)

        self.encounter_distance_m = safe_float(self.encounter_cfg.get("distance_m"), 14.0)
        self.encounter_bearing_deg = safe_float(self.encounter_cfg.get("bearing_deg"), 35.0)
        self.minimum_post_encounter_observation_window_s = safe_float(
            self.evaluation_cfg.get("minimum_post_encounter_observation_window_s"), 30.0
        )
        self.deadlock_window_s = safe_float(self.evaluation_cfg.get("deadlock_window_s"), 45.0)
        self.deadlock_progress_delta = safe_float(self.evaluation_cfg.get("deadlock_progress_delta"), 0.01)
        self.degraded_recovery_threshold_ms = safe_float(
            self.evaluation_cfg.get("degraded_recovery_threshold_ms"), 25000.0
        )
        self.reaction_cmd_delta_lin = safe_float(self.evaluation_cfg.get("reaction_cmd_delta_lin"), 0.20)
        self.reaction_cmd_delta_ang = safe_float(self.evaluation_cfg.get("reaction_cmd_delta_ang"), 0.35)

        self.output_root.mkdir(parents=True, exist_ok=True)
        (self.output_root / "metrics").mkdir(exist_ok=True)
        (self.output_root / "rosbags").mkdir(exist_ok=True)
        (self.output_root / "runtime").mkdir(exist_ok=True)
        self.report_path = self.output_root / str(
            self.output_cfg.get("report_file", "spacetry_scenario_obstacle_sensing_stress_report.md")
        )
        self.metrics_path = self.output_root / str(
            self.output_cfg.get(
                "metrics_file", "metrics/scenario_obstacle_sensing_stress_metrics.json"
            )
        )
        self.timeline_path = self.output_root / str(
            self.output_cfg.get(
                "timeline_file", "runtime/scenario_obstacle_sensing_stress_timeline.jsonl"
            )
        )
        self.metrics_path.parent.mkdir(parents=True, exist_ok=True)
        self.timeline_path.parent.mkdir(parents=True, exist_ok=True)
        self.timeline_handle = self.timeline_path.open("a", encoding="utf-8")

        self.best_effort_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.pose_x: Optional[float] = None
        self.pose_y: Optional[float] = None
        self.pose_yaw: Optional[float] = None
        self.last_pose_time_s: Optional[float] = None
        self.last_path_point: Optional[Tuple[float, float]] = None
        self.progress_history: Deque[Tuple[float, float]] = deque(maxlen=600)
        self.cmd_vel: Optional[Tuple[float, float]] = None
        self.prev_cmd_vel: Optional[Tuple[float, float]] = None
        self.last_cmd_classification: Optional[str] = None

        self.battery_soc: Optional[float] = None
        self.battery_near_outpost: Optional[bool] = None

        self.obstacle_front = False
        self.obstacle_left = False
        self.obstacle_right = False
        self.obstacle_state = "CLEAR"
        self.last_non_clear_obstacle_time_s: Optional[float] = None

        self.scan_front_min: Optional[float] = None
        self.scan_left_min: Optional[float] = None
        self.scan_right_min: Optional[float] = None

        self.goal_reached = False
        self.goal_reached_time_s: Optional[float] = None
        self.at_science_rock_pub = self.create_publisher(
            Bool, self.topics.get("at_science_rock", "/at_science_rock"), 10
        )

        self.override_front_pub = self.create_publisher(
            Bool, self.topics.get("obstacle_front", "/obstacle/front"), self.best_effort_qos
        )
        self.override_left_pub = self.create_publisher(
            Bool, self.topics.get("obstacle_left", "/obstacle/left"), self.best_effort_qos
        )
        self.override_right_pub = self.create_publisher(
            Bool, self.topics.get("obstacle_right", "/obstacle/right"), self.best_effort_qos
        )
        self.override_state_pub = self.create_publisher(
            String, self.topics.get("obstacle_state", "/obstacle/state"), self.best_effort_qos
        )

        self.monitor_mr009_active = False
        self.monitor_mr011_active = False
        self.monitor_mr009_time_s: Optional[float] = None
        self.monitor_mr011_time_s: Optional[float] = None
        self.monitor_events: List[str] = []

        self.start_time_s: Optional[float] = None
        self.injected_model_name: Optional[str] = None
        self.injected_fault_pose: Optional[Dict[str, float]] = None
        self.injection_gate_diagnostics: Dict[str, Any] = {}
        self.injection_triggered = False
        self.injection_verified = False
        self.injection_failed = False
        self.injection_time_s: Optional[float] = None
        self.degradation_mode = "inactive"
        self.degradation_noise_start_s: Optional[float] = None
        self.degradation_last_state: Optional[str] = None
        self.override_message_count = 0
        self.false_obstacle_count = 0

        self.encounter_time_s: Optional[float] = None
        self.detection_time_s: Optional[float] = None
        self.detection_signal_source: Optional[str] = None
        self.detection_attribution_status = False
        self.raw_scan_detection_time_s: Optional[float] = None
        self.minimum_fault_distance_at_detection_m: Optional[float] = None

        self.first_reaction_time_s: Optional[float] = None
        self.reaction_cmd_snapshot: Optional[Dict[str, float]] = None
        self.observed_control_rationale: Optional[str] = None
        self.reaction_scope: Optional[str] = None
        self.reaction_attribution_status = False
        self.active_context_at_reaction: Optional[Dict[str, Any]] = None

        self.recovery_time_s: Optional[float] = None
        self.route_deviation_m = 0.0
        self.post_injection_route_deviation_m = 0.0
        self.baseline_uncertainty_exercised = False
        self.baseline_reaction_observed = False
        self.baseline_confound_status = "clear"
        self.collision_with_obstacle_observed = False

        self.termination_reason: Optional[str] = None
        self.shutdown_requested = False
        self.finalized = False

        self.create_subscription(
            Odometry,
            self.topics.get("odom", "/mobile_base_controller/odom"),
            self.on_odom,
            self.best_effort_qos,
        )
        self.create_subscription(Twist, self.topics.get("cmd_vel", "/cmd_vel"), self.on_cmd_vel, 10)
        self.create_subscription(
            LaserScan, self.topics.get("scan", "/scan"), self.on_scan, qos_profile_sensor_data
        )
        self.create_subscription(
            Bool, self.topics.get("obstacle_front", "/obstacle/front"), self.on_obstacle_front, self.best_effort_qos
        )
        self.create_subscription(
            Bool, self.topics.get("obstacle_left", "/obstacle/left"), self.on_obstacle_left, self.best_effort_qos
        )
        self.create_subscription(
            Bool, self.topics.get("obstacle_right", "/obstacle/right"), self.on_obstacle_right, self.best_effort_qos
        )
        self.create_subscription(
            String, self.topics.get("obstacle_state", "/obstacle/state"), self.on_obstacle_state, self.best_effort_qos
        )
        self.create_subscription(
            Float32, self.topics.get("battery_soc", "/battery/soc"), self.on_battery_soc, 10
        )
        self.create_subscription(
            Bool,
            self.topics.get("battery_near_outpost", "/battery/near_outpost"),
            self.on_battery_near_outpost,
            10,
        )
        self.create_subscription(
            String, self.topics.get("monitor_events", "/monitor/events"), self.on_monitor_event, 10
        )
        self.create_subscription(
            Bool, self.topics.get("monitor_mr009", "/monitor/handlerMR_009_RETURN_TO_OUTPOST_ON_LOW_BATTERY"),
            self.on_monitor_mr009, 10
        )
        self.create_subscription(
            Bool, self.topics.get("monitor_mr011", "/monitor/handlerMR_011_MAINTAIN_SPEED_WHEN_FULL_BATTERY"),
            self.on_monitor_mr011, 10
        )

        self.create_timer(1.0 / max(self.degradation_publish_hz, 1.0), self.on_fast_timer)
        self.create_timer(0.5, self.on_slow_timer)

        self.write_metadata_snapshot()
        self.log_event(
            "scenario_started",
            {
                "contract_file": self.scenario_contract_file,
                "config_file": self.scenario_config_file,
                "output_root": str(self.output_root),
                "goal": {"name": self.goal_name, "x": self.goal_x, "y": self.goal_y},
            },
        )
        self.get_logger().info(
            "Scenario driver ready: obstacle blocker plus degraded obstacle interpretation."
        )

    def _load_yaml(self, path: str) -> Dict[str, Any]:
        if not path:
            return {}
        with open(path, "r", encoding="utf-8") as handle:
            return yaml.safe_load(handle) or {}

    def sim_time_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def write_metadata_snapshot(self) -> None:
        metadata = {
            "scenario_name": self.scenario_name,
            "created_at_utc": iso_utc_now(),
            "contract_file": self.scenario_contract_file,
            "config_file": self.scenario_config_file,
            "runtime_parameter_interface": self.contract.get("runtime_parameter_interface", []),
        }
        metadata_path = self.output_root / "runtime" / "scenario_metadata.json"
        with metadata_path.open("w", encoding="utf-8") as handle:
            json.dump(metadata, handle, indent=2, sort_keys=True)

    def log_event(self, event: str, details: Optional[Dict[str, Any]] = None) -> None:
        payload = {
            "event": event,
            "sim_time_s": round(self.sim_time_s(), 3),
            "wall_time_utc": iso_utc_now(),
            "details": details or {},
        }
        self.timeline_handle.write(json.dumps(payload, sort_keys=True) + "\n")
        self.timeline_handle.flush()

    def on_odom(self, msg: Odometry) -> None:
        now_s = self.sim_time_s()
        if self.start_time_s is None and now_s > 0.0:
            self.start_time_s = now_s
            self.log_event("sim_time_started", {"sim_time_s": round(now_s, 3)})
        self.pose_x = float(msg.pose.pose.position.x)
        self.pose_y = float(msg.pose.pose.position.y)
        self.pose_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.last_pose_time_s = now_s

        progress = self.compute_progress_ratio(self.pose_x, self.pose_y)
        self.progress_history.append((now_s, progress))
        self.update_route_deviation(self.pose_x, self.pose_y)
        self.publish_goal_flag()

        if not self.goal_reached:
            distance_to_goal = point_distance(self.pose_x, self.pose_y, self.goal_x, self.goal_y)
            if distance_to_goal <= self.goal_tolerance_m:
                self.goal_reached = True
                self.goal_reached_time_s = now_s
                self.log_event(
                    "goal_reached",
                    {
                        "goal_name": self.goal_name,
                        "distance_to_goal_m": round(distance_to_goal, 3),
                    },
                )

        if self.injection_verified:
            self.update_encounter_status()
            self.update_detection_candidates()

    def on_cmd_vel(self, msg: Twist) -> None:
        now_s = self.sim_time_s()
        current = (float(msg.linear.x), float(msg.angular.z))
        self.cmd_vel = current
        if self.prev_cmd_vel is None:
            self.prev_cmd_vel = current
            return

        delta_lin = abs(current[0] - self.prev_cmd_vel[0])
        delta_ang = abs(current[1] - self.prev_cmd_vel[1])
        if delta_lin >= self.reaction_cmd_delta_lin or delta_ang >= self.reaction_cmd_delta_ang:
            rationale, scope, attribution, context = self.classify_reaction(current)
            self.last_cmd_classification = rationale
            self.log_event(
                "cmd_vel_changed",
                {
                    "linear_x": round(current[0], 3),
                    "angular_z": round(current[1], 3),
                    "observed_control_rationale": rationale,
                    "reaction_scope": scope,
                    "reaction_attribution_status": attribution,
                    "active_context_at_reaction": context,
                },
            )
            if scope in {"baseline_only", "baseline_and_injected"}:
                self.baseline_reaction_observed = True
            if scope != "indeterminate":
                self.baseline_uncertainty_exercised = self.baseline_uncertainty_exercised or (
                    scope in {"baseline_only", "baseline_and_injected"}
                )
            if (
                self.first_reaction_time_s is None
                and rationale != "goal_alignment"
                and rationale != "unknown"
                and self.encounter_time_s is not None
            ):
                self.first_reaction_time_s = now_s
                self.observed_control_rationale = rationale
                self.reaction_scope = scope
                self.reaction_attribution_status = attribution
                self.active_context_at_reaction = context
                self.reaction_cmd_snapshot = {
                    "linear_x": round(current[0], 3),
                    "angular_z": round(current[1], 3),
                }
                event_name = "reaction_attributed" if attribution else "reaction_not_attributed"
                self.log_event(
                    event_name,
                    {
                        "observed_control_rationale": rationale,
                        "reaction_scope": scope,
                        "reaction_attribution_status": attribution,
                        "active_context_at_reaction": context,
                    },
                )
            if (
                self.recovery_time_s is None
                and self.first_reaction_time_s is not None
                and current[0] > 0.3
                and abs(current[1]) < 0.25
                and self.progress_delta_over(self.trigger_progress_window_s) > self.trigger_min_progress_delta
            ):
                self.recovery_time_s = now_s
                self.log_event(
                    "recovery_observed",
                    {
                        "time_since_reaction_s": round(now_s - self.first_reaction_time_s, 3),
                        "progress_delta": round(self.progress_delta_over(self.trigger_progress_window_s), 4),
                    },
                )

        self.prev_cmd_vel = current

    def on_scan(self, msg: LaserScan) -> None:
        self.scan_front_min = self._sector_min(msg, math.radians(-20.0), math.radians(20.0))
        self.scan_left_min = self._sector_min(msg, math.radians(20.0), math.radians(75.0))
        self.scan_right_min = self._sector_min(msg, math.radians(-75.0), math.radians(-20.0))

    def _sector_min(self, msg: LaserScan, min_angle: float, max_angle: float) -> Optional[float]:
        if not msg.ranges or msg.angle_increment == 0.0:
            return None
        i0 = max(0, int(round((min_angle - msg.angle_min) / msg.angle_increment)))
        i1 = min(len(msg.ranges) - 1, int(round((max_angle - msg.angle_min) / msg.angle_increment)))
        best = math.inf
        for index in range(min(i0, i1), max(i0, i1) + 1):
            distance = msg.ranges[index]
            if not math.isfinite(distance) or distance <= 0.0:
                continue
            if distance < msg.range_min or distance > msg.range_max:
                continue
            best = min(best, float(distance))
        return None if not math.isfinite(best) else best

    def on_obstacle_front(self, msg: Bool) -> None:
        self.obstacle_front = bool(msg.data)
        if self.obstacle_front:
            self.last_non_clear_obstacle_time_s = self.sim_time_s()

    def on_obstacle_left(self, msg: Bool) -> None:
        self.obstacle_left = bool(msg.data)
        if self.obstacle_left:
            self.last_non_clear_obstacle_time_s = self.sim_time_s()

    def on_obstacle_right(self, msg: Bool) -> None:
        self.obstacle_right = bool(msg.data)
        if self.obstacle_right:
            self.last_non_clear_obstacle_time_s = self.sim_time_s()

    def on_obstacle_state(self, msg: String) -> None:
        self.obstacle_state = msg.data or "CLEAR"
        if self.obstacle_state != "CLEAR":
            self.last_non_clear_obstacle_time_s = self.sim_time_s()

    def on_battery_soc(self, msg: Float32) -> None:
        self.battery_soc = float(msg.data)

    def on_battery_near_outpost(self, msg: Bool) -> None:
        self.battery_near_outpost = bool(msg.data)

    def on_monitor_event(self, msg: String) -> None:
        event = msg.data
        self.monitor_events.append(event)
        self.log_event("baseline_monitor_violation_active", {"event": event})

    def on_monitor_mr009(self, msg: Bool) -> None:
        if msg.data and not self.monitor_mr009_active:
            self.monitor_mr009_active = True
            self.monitor_mr009_time_s = self.sim_time_s()
            self.baseline_uncertainty_exercised = True
            self.log_event("monitor_mr009_triggered", {"topic": self.topics.get("monitor_mr009")})

    def on_monitor_mr011(self, msg: Bool) -> None:
        if msg.data and not self.monitor_mr011_active:
            self.monitor_mr011_active = True
            self.monitor_mr011_time_s = self.sim_time_s()
            self.baseline_uncertainty_exercised = True
            self.log_event("monitor_mr011_triggered", {"topic": self.topics.get("monitor_mr011")})

    def publish_goal_flag(self) -> None:
        at_goal = False
        if self.pose_x is not None and self.pose_y is not None:
            at_goal = point_distance(self.pose_x, self.pose_y, self.goal_x, self.goal_y) <= self.goal_tolerance_m
        msg = Bool()
        msg.data = at_goal
        self.at_science_rock_pub.publish(msg)

    def compute_progress_ratio(self, x: float, y: float) -> float:
        goal_dx = self.goal_x - self.start_x
        goal_dy = self.goal_y - self.start_y
        denom = goal_dx * goal_dx + goal_dy * goal_dy
        if denom <= 1e-6:
            return 0.0
        progress = ((x - self.start_x) * goal_dx + (y - self.start_y) * goal_dy) / denom
        return clamp(progress, 0.0, 1.0)

    def point_to_nominal_route_distance(self, x: float, y: float) -> float:
        x1, y1 = self.start_x, self.start_y
        x2, y2 = self.goal_x, self.goal_y
        dx = x2 - x1
        dy = y2 - y1
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return point_distance(x, y, x1, y1)
        numerator = abs(dy * x - dx * y + x2 * y1 - y2 * x1)
        denominator = math.hypot(dx, dy)
        return numerator / denominator

    def update_route_deviation(self, x: float, y: float) -> None:
        deviation = self.point_to_nominal_route_distance(x, y)
        self.route_deviation_m = max(self.route_deviation_m, deviation)
        if self.injection_time_s is not None:
            self.post_injection_route_deviation_m = max(self.post_injection_route_deviation_m, deviation)

    def progress_delta_over(self, window_s: float) -> float:
        if not self.progress_history:
            return 0.0
        now_s = self.sim_time_s()
        current_progress = self.progress_history[-1][1]
        reference_progress = self.progress_history[0][1]
        for timestamp, progress in self.progress_history:
            if now_s - timestamp <= window_s:
                reference_progress = progress
                break
        return current_progress - reference_progress

    def distance_to_goal(self) -> Optional[float]:
        if self.pose_x is None or self.pose_y is None:
            return None
        return point_distance(self.pose_x, self.pose_y, self.goal_x, self.goal_y)

    def distance_to_injected_fault(self) -> Optional[float]:
        if self.injected_fault_pose is None or self.pose_x is None or self.pose_y is None:
            return None
        return point_distance(
            self.pose_x,
            self.pose_y,
            self.injected_fault_pose["x"],
            self.injected_fault_pose["y"],
        )

    def bearing_to_injected_fault_deg(self) -> Optional[float]:
        if (
            self.injected_fault_pose is None
            or self.pose_x is None
            or self.pose_y is None
            or self.pose_yaw is None
        ):
            return None
        target_yaw = math.atan2(
            self.injected_fault_pose["y"] - self.pose_y,
            self.injected_fault_pose["x"] - self.pose_x,
        )
        delta = target_yaw - self.pose_yaw
        while delta > math.pi:
            delta -= 2.0 * math.pi
        while delta < -math.pi:
            delta += 2.0 * math.pi
        return math.degrees(delta)

    def geometry_expected_state(self) -> str:
        distance = self.distance_to_injected_fault()
        bearing = self.bearing_to_injected_fault_deg()
        if distance is None or bearing is None or distance > self.obstacle_threshold_m:
            return "CLEAR"
        if -20.0 <= bearing <= 20.0:
            return "FRONT"
        if 20.0 < bearing <= 75.0:
            return "LEFT"
        if -75.0 <= bearing < -20.0:
            return "RIGHT"
        return "CLEAR"

    def block_island_near(self) -> bool:
        if self.pose_x is None or self.pose_y is None:
            return False
        return (
            point_distance(self.pose_x, self.pose_y, self.block_island_x, self.block_island_y)
            <= self.block_island_radius_m
        )

    def on_fast_timer(self) -> None:
        if self.finalized or self.shutdown_requested:
            return
        self.publish_goal_flag()
        if self.injection_triggered and self.injection_verified:
            self.drive_degradation_publishers()

    def on_slow_timer(self) -> None:
        if self.finalized or self.shutdown_requested:
            return
        if self.start_time_s is None or self.pose_x is None or self.pose_y is None:
            return

        if not self.injection_triggered:
            self.try_inject_fault()

        if self.encounter_time_s is not None and self.first_reaction_time_s is None:
            self.check_deadlock_candidate()

        now_s = self.sim_time_s()
        if now_s - self.start_time_s >= self.run_timeout_s:
            self.request_shutdown("timeout")
            return

        if self.goal_reached and self.goal_reached_time_s is not None:
            if now_s - self.goal_reached_time_s >= self.post_goal_observation_s:
                self.request_shutdown("goal_reached")

    def try_inject_fault(self) -> None:
        now_s = self.sim_time_s()
        progress = self.compute_progress_ratio(self.pose_x, self.pose_y)
        progress_delta = self.progress_delta_over(self.trigger_progress_window_s)
        distance_to_goal = self.distance_to_goal()
        distance_to_block_island = point_distance(
            self.pose_x, self.pose_y, self.block_island_x, self.block_island_y
        )
        remaining_time = self.run_timeout_s - (now_s - self.start_time_s)
        clear_window_s = (
            now_s - self.last_non_clear_obstacle_time_s
            if self.last_non_clear_obstacle_time_s is not None
            else now_s - self.start_time_s
        )
        monitors_active = self.monitor_mr009_active or self.monitor_mr011_active
        diagnostics = {
            "progress_ratio": round(progress, 4),
            "progress_delta": round(progress_delta, 4),
            "remaining_time_s": round(remaining_time, 2),
            "distance_to_goal_m": round(distance_to_goal, 3) if distance_to_goal is not None else -1.0,
            "distance_to_block_island_m": round(distance_to_block_island, 3),
            "obstacle_state": self.obstacle_state,
            "baseline_monitors_active": {
                "MR_009": self.monitor_mr009_active,
                "MR_011": self.monitor_mr011_active,
            },
            "clear_window_s": round(clear_window_s, 3),
        }

        gate_passed = (
            progress >= self.trigger_min_progress_ratio
            and progress_delta >= self.trigger_min_progress_delta
            and remaining_time >= self.trigger_min_remaining_time_s
            and distance_to_goal is not None
            and distance_to_goal >= self.trigger_min_distance_to_goal_m
            and distance_to_block_island >= self.trigger_min_distance_from_block_island_m
            and clear_window_s >= self.trigger_clear_window_s
            and not monitors_active
        )
        if not gate_passed:
            return

        self.injection_gate_diagnostics = diagnostics
        self.log_event("fault_injection_gate_passed", diagnostics)
        candidate = self.select_injection_pose()
        if candidate is None:
            self.injection_failed = True
            self.log_event("fault_injection_rejected", {"reason": "no_valid_pose"})
            self.request_shutdown("injection_pose_unavailable")
            return

        spawn_ok, verification_ok, spawn_result = self.spawn_fault(candidate)
        self.injected_fault_pose = candidate
        self.injected_model_name = str(spawn_result.get("model_name"))
        self.injection_triggered = True
        self.injection_time_s = now_s
        self.injection_verified = spawn_ok and verification_ok
        self.injection_failed = not self.injection_verified
        self.degradation_mode = "suppression" if self.injection_verified else "inactive"
        self.log_event(
            "fault_injected",
            {
                "fault_pose": candidate,
                "spawn_ok": spawn_ok,
                "verification_ok": verification_ok,
                "spawn_result": spawn_result,
                "remaining_time_s": round(remaining_time, 2),
                "progress_ratio": round(progress, 4),
                "baseline_monitors_active": {
                    "MR_009": self.monitor_mr009_active,
                    "MR_011": self.monitor_mr011_active,
                },
            },
        )
        if not self.injection_verified:
            self.request_shutdown("injection_failed")

    def select_injection_pose(self) -> Optional[Dict[str, float]]:
        if self.pose_x is None or self.pose_y is None:
            return None
        vec_x = self.goal_x - self.pose_x
        vec_y = self.goal_y - self.pose_y
        length = math.hypot(vec_x, vec_y)
        if length <= 1e-6:
            return None
        unit_x = vec_x / length
        unit_y = vec_y / length
        yaw = math.atan2(unit_y, unit_x)

        for offset in (self.spawn_distance_ahead_m, self.spawn_distance_ahead_m + 2.0, self.spawn_distance_ahead_m + 4.0):
            candidate_x = self.pose_x + unit_x * offset
            candidate_y = self.pose_y + unit_y * offset
            if point_distance(candidate_x, candidate_y, self.goal_x, self.goal_y) < self.min_sep_goal_m:
                continue
            if point_distance(candidate_x, candidate_y, self.outpost_x, self.outpost_y) < self.min_sep_outpost_m:
                continue
            if (
                point_distance(candidate_x, candidate_y, self.block_island_x, self.block_island_y)
                < self.min_sep_block_island_m
            ):
                continue
            return {
                "x": round(candidate_x, 3),
                "y": round(candidate_y, 3),
                "z": round(self.spawn_z, 3),
                "yaw": round(yaw, 3),
            }
        return None

    def spawn_fault(self, pose: Dict[str, float]) -> Tuple[bool, bool, Dict[str, Any]]:
        env = os.environ.copy()
        model_root = self.resolve_model_root()
        model_file = model_root / self.injected_model_key / "model.sdf"
        resource_paths = [env.get("GZ_SIM_RESOURCE_PATH", ""), str(model_root)]
        sdf_paths = [env.get("SDF_PATH", ""), str(model_root)]
        env["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join([value for value in resource_paths if value])
        env["SDF_PATH"] = os.pathsep.join([value for value in sdf_paths if value])

        model_name = f"injected_{self.injected_model_key}_{int(self.sim_time_s() * 1000)}"
        command = [
            "ros2",
            "run",
            "ros_gz_sim",
            "create",
            "-world",
            "mars_outpost",
            "-name",
            model_name,
            "-allow_renaming",
            "false",
            "-file",
            str(model_file),
            "-x",
            f"{pose['x']:.3f}",
            "-y",
            f"{pose['y']:.3f}",
            "-z",
            f"{pose['z']:.3f}",
            "-Y",
            f"{pose['yaw']:.3f}",
        ]

        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=30,
            env=env,
            check=False,
        )
        time.sleep(1.0)
        verify = subprocess.run(
            ["gz", "model", "--list"],
            capture_output=True,
            text=True,
            timeout=15,
            env=env,
            check=False,
        )
        verification_ok = verify.returncode == 0 and model_name in verify.stdout
        return (
            result.returncode == 0,
            verification_ok,
            {
                "command": command,
                "model_name": model_name,
                "model_file": str(model_file),
                "gz_sim_resource_path": env["GZ_SIM_RESOURCE_PATH"],
                "sdf_path": env["SDF_PATH"],
                "spawn_stdout": result.stdout[-2000:],
                "spawn_stderr": result.stderr[-2000:],
                "verify_stdout": verify.stdout[-2000:],
                "verify_stderr": verify.stderr[-2000:],
            },
        )

    def resolve_model_root(self) -> Path:
        try:
            return Path(get_package_share_directory("spacetry_models")) / "models"
        except PackageNotFoundError:
            return Path("/ws/src/spacetry_models/models")

    def drive_degradation_publishers(self) -> None:
        if self.injection_time_s is None:
            return
        now_s = self.sim_time_s()
        distance = self.distance_to_injected_fault()
        if distance is None:
            return

        if self.degradation_mode == "suppression":
            elapsed = now_s - self.injection_time_s
            if distance <= self.degradation_activation_distance_m or elapsed >= self.suppression_max_s:
                self.degradation_mode = "noise"
                self.degradation_noise_start_s = now_s
                self.log_event(
                    "fault_detection_candidate",
                    {
                        "distance_to_fault_m": round(distance, 3),
                        "candidate_source": "obstacle_override_phase_transition",
                        "phase": "noise",
                    },
                )
            else:
                self.publish_override_state("CLEAR")
                return

        if self.degradation_mode == "noise":
            if self.degradation_noise_start_s is None:
                self.degradation_noise_start_s = now_s
            noise_elapsed = now_s - self.degradation_noise_start_s
            if noise_elapsed >= self.noise_duration_s:
                self.degradation_mode = "recovered"
                self.log_event("fault_degradation_recovered", {"noise_duration_s": round(noise_elapsed, 3)})
                return
            phase_index = int(noise_elapsed / max(self.alternation_period_s, 0.1))
            state = "FRONT_LEFT" if phase_index % 2 == 0 else "FRONT_RIGHT"
            self.publish_override_state(state)

    def publish_override_state(self, state: str) -> None:
        if state != self.degradation_last_state:
            self.log_event("fault_state_applied", {"state": state})
            self.degradation_last_state = state

        front = state in {"FRONT", "FRONT_LEFT", "FRONT_RIGHT"}
        left = state in {"LEFT", "FRONT_LEFT"}
        right = state in {"RIGHT", "FRONT_RIGHT"}

        front_msg = Bool()
        front_msg.data = front
        left_msg = Bool()
        left_msg.data = left
        right_msg = Bool()
        right_msg.data = right
        state_msg = String()
        state_msg.data = state

        self.override_front_pub.publish(front_msg)
        self.override_left_pub.publish(left_msg)
        self.override_right_pub.publish(right_msg)
        self.override_state_pub.publish(state_msg)

        self.override_message_count += 1
        if state != "CLEAR":
            expected_state = self.geometry_expected_state()
            if not self.is_supported_classification(expected_state, state):
                self.false_obstacle_count += 1

    def is_supported_classification(self, expected_state: str, published_state: str) -> bool:
        if published_state == "CLEAR":
            return expected_state == "CLEAR"
        if expected_state == "FRONT":
            return published_state in {"FRONT", "FRONT_LEFT", "FRONT_RIGHT"}
        if expected_state == "LEFT":
            return published_state == "LEFT"
        if expected_state == "RIGHT":
            return published_state == "RIGHT"
        return False

    def update_encounter_status(self) -> None:
        if self.encounter_time_s is not None or self.injected_fault_pose is None:
            return
        distance = self.distance_to_injected_fault()
        bearing = self.bearing_to_injected_fault_deg()
        if distance is None or bearing is None:
            return
        if distance <= self.encounter_distance_m and abs(bearing) <= self.encounter_bearing_deg:
            self.encounter_time_s = self.sim_time_s()
            self.log_event(
                "fault_encountered",
                {
                    "distance_to_fault_m": round(distance, 3),
                    "bearing_deg": round(bearing, 3),
                },
            )

    def update_detection_candidates(self) -> None:
        if self.injected_fault_pose is None or self.detection_time_s is not None:
            return
        distance = self.distance_to_injected_fault()
        bearing = self.bearing_to_injected_fault_deg()
        if distance is None or bearing is None:
            return
        if abs(bearing) > self.encounter_bearing_deg:
            return

        baseline_confounded = self.block_island_near() or self.monitor_mr009_active or self.monitor_mr011_active
        if self.scan_front_min is not None and distance <= self.obstacle_threshold_m and self.scan_front_min <= self.obstacle_threshold_m:
            if self.raw_scan_detection_time_s is None:
                self.raw_scan_detection_time_s = self.sim_time_s()
                self.log_event(
                    "raw_scan_detection_attributed",
                    {
                        "distance_to_fault_m": round(distance, 3),
                        "scan_front_min_m": round(self.scan_front_min, 3),
                    },
                )

        non_clear = self.obstacle_state != "CLEAR" or self.obstacle_front or self.obstacle_left or self.obstacle_right
        if not non_clear or distance > self.obstacle_threshold_m:
            return

        self.detection_time_s = self.sim_time_s()
        self.detection_signal_source = "obstacle/front|left|right"
        self.detection_attribution_status = not baseline_confounded
        self.minimum_fault_distance_at_detection_m = distance
        self.baseline_confound_status = "active" if baseline_confounded else "clear"
        event_name = "fault_detection_attributed" if self.detection_attribution_status else "fault_detection_rejected"
        self.log_event(
            event_name,
            {
                "distance_to_fault_m": round(distance, 3),
                "bearing_deg": round(bearing, 3),
                "baseline_confound_status": self.baseline_confound_status,
                "detection_signal_source": self.detection_signal_source,
                "obstacle_state": self.obstacle_state,
            },
        )

    def classify_reaction(
        self, current_cmd: Tuple[float, float]
    ) -> Tuple[str, str, bool, Dict[str, Any]]:
        distance = self.distance_to_injected_fault()
        bearing = self.bearing_to_injected_fault_deg()
        injected_context = (
            self.injection_verified
            and distance is not None
            and distance <= self.encounter_distance_m
            and bearing is not None
            and abs(bearing) <= self.encounter_bearing_deg
        )
        baseline_context = self.block_island_near() or self.monitor_mr009_active or self.monitor_mr011_active

        linear_x, angular_z = current_cmd
        if self.monitor_mr009_active or self.monitor_mr011_active:
            rationale = "monitor_enforcement"
        elif linear_x < -0.05:
            rationale = "obstacle_avoidance"
        elif abs(angular_z) > 0.45 and abs(linear_x) < 0.15 and (injected_context or self.obstacle_front):
            rationale = "obstacle_avoidance"
        elif abs(angular_z) > 0.35 and linear_x > 0.15 and injected_context:
            rationale = "replan_execution"
        elif linear_x > 0.15:
            rationale = "goal_alignment"
        else:
            rationale = "unknown"

        if injected_context and baseline_context:
            scope = "baseline_and_injected"
        elif injected_context:
            scope = "injected_only"
        elif baseline_context:
            scope = "baseline_only"
        else:
            scope = "indeterminate"

        attribution = scope in {"injected_only", "baseline_and_injected"} and rationale in {
            "obstacle_avoidance",
            "replan_execution",
        }
        context = {
            "distance_to_fault_m": None if distance is None else round(distance, 3),
            "bearing_to_fault_deg": None if bearing is None else round(bearing, 3),
            "obstacle_state": self.obstacle_state,
            "scan_front_min_m": None if self.scan_front_min is None else round(self.scan_front_min, 3),
            "block_island_near": self.block_island_near(),
            "monitor_mr009_active": self.monitor_mr009_active,
            "monitor_mr011_active": self.monitor_mr011_active,
            "degradation_mode": self.degradation_mode,
            "progress_ratio": round(self.compute_progress_ratio(self.pose_x or 0.0, self.pose_y or 0.0), 4),
        }
        return rationale, scope, attribution, context

    def check_deadlock_candidate(self) -> None:
        if self.encounter_time_s is None or self.goal_reached:
            return
        now_s = self.sim_time_s()
        if now_s - self.encounter_time_s < self.deadlock_window_s:
            return
        if self.progress_delta_over(self.deadlock_window_s) <= self.deadlock_progress_delta:
            self.log_event(
                "meaningful_evaluation_window_satisfied",
                {
                    "evaluation_window_after_encounter_s": round(now_s - self.encounter_time_s, 3),
                    "progress_delta": round(self.progress_delta_over(self.deadlock_window_s), 4),
                },
            )
            self.request_shutdown("deadlock")

    def request_shutdown(self, reason: str) -> None:
        if self.termination_reason is None:
            self.termination_reason = reason
            self.log_event("shutdown_requested", {"reason": reason})
        self.shutdown_requested = True

    def note_external_shutdown(self, reason: str) -> None:
        if self.termination_reason is None:
            self.termination_reason = reason
            self.log_event("shutdown_requested", {"reason": reason})
        self.shutdown_requested = True

    def finalize(self) -> None:
        if self.finalized:
            return
        self.finalized = True
        evaluation_window_after_encounter_s = 0.0
        if self.encounter_time_s is not None:
            end_time = self.sim_time_s()
            evaluation_window_after_encounter_s = max(0.0, end_time - self.encounter_time_s)

        meaningful_window_satisfied = (
            self.encounter_time_s is not None
            and evaluation_window_after_encounter_s >= self.minimum_post_encounter_observation_window_s
        )
        if self.encounter_time_s is not None:
            self.log_event(
                "meaningful_evaluation_window_satisfied"
                if meaningful_window_satisfied
                else "meaningful_evaluation_window_not_satisfied",
                {
                    "evaluation_window_after_encounter_s": round(evaluation_window_after_encounter_s, 3),
                    "minimum_post_encounter_observation_window_s": self.minimum_post_encounter_observation_window_s,
                },
            )

        route_deviation_m = round(self.route_deviation_m, 3)
        post_injection_route_deviation_m = round(self.post_injection_route_deviation_m, 3)
        recovery_rate_ms = (
            round((self.recovery_time_s - self.first_reaction_time_s) * 1000.0, 1)
            if self.recovery_time_s is not None and self.first_reaction_time_s is not None
            else None
        )
        adaptation_speed_ms = (
            round((self.first_reaction_time_s - self.injection_time_s) * 1000.0, 1)
            if self.first_reaction_time_s is not None and self.injection_time_s is not None
            else None
        )
        obstacle_detection_latency_ms = (
            round((self.detection_time_s - self.injection_time_s) * 1000.0, 1)
            if self.detection_time_s is not None and self.injection_time_s is not None
            else None
        )
        raw_scan_detection_latency_ms = (
            round((self.raw_scan_detection_time_s - self.injection_time_s) * 1000.0, 1)
            if self.raw_scan_detection_time_s is not None and self.injection_time_s is not None
            else None
        )
        false_obstacle_rate_pct = (
            round((self.false_obstacle_count / self.override_message_count) * 100.0, 2)
            if self.override_message_count > 0
            else 0.0
        )

        safety_preservation = {
            "MR_009": not self.monitor_mr009_active,
            "MR_011": not self.monitor_mr011_active,
            "collision_free": not self.collision_with_obstacle_observed,
        }
        goal_viability = {
            "science_rock_01_reached": self.goal_reached,
            "mission_deadline_met": self.goal_reached and self.termination_reason != "timeout",
        }

        if not self.injection_verified:
            injected_outcome = "INCONCLUSIVE"
        elif self.encounter_time_s is None:
            injected_outcome = "UNTESTED"
        elif not meaningful_window_satisfied:
            injected_outcome = "INCONCLUSIVE"
        elif (
            self.reaction_attribution_status
            and self.goal_reached
            and all(safety_preservation.values())
            and recovery_rate_ms is not None
        ):
            injected_outcome = "PASS"
        elif self.first_reaction_time_s is not None:
            injected_outcome = "DEGRADED"
        else:
            injected_outcome = "FAIL"

        if self.baseline_uncertainty_exercised or self.baseline_reaction_observed or self.block_island_near():
            baseline_outcome = "DEGRADED" if (self.monitor_mr009_active or self.monitor_mr011_active) else "PASS"
        else:
            baseline_outcome = "NOT_EVALUATED"

        if all(safety_preservation.values()):
            safety_status = "PASS"
        elif self.monitor_mr009_active or self.collision_with_obstacle_observed:
            safety_status = "FAIL"
        else:
            safety_status = "DEGRADED"

        goal_status = "PASS" if self.goal_reached else "FAIL"
        overall_outcome = "FAIL" if safety_status == "FAIL" else injected_outcome

        metrics = {
            "scenario_name": self.scenario_name,
            "termination_reason": self.termination_reason or "shutdown",
            "outcome_assessment": overall_outcome,
            "baseline_outcome_assessment": baseline_outcome,
            "injected_outcome_assessment": injected_outcome,
            "adaptation_speed_ms": adaptation_speed_ms,
            "obstacle_detection_latency_ms": obstacle_detection_latency_ms,
            "raw_scan_detection_latency_ms": raw_scan_detection_latency_ms,
            "autonomy_reaction_status": self.first_reaction_time_s is not None,
            "detection_signal_source": self.detection_signal_source,
            "observed_control_rationale": self.observed_control_rationale,
            "reaction_scope": self.reaction_scope,
            "reaction_attribution_status": self.reaction_attribution_status,
            "active_context_at_reaction": self.active_context_at_reaction,
            "safety_preservation": safety_preservation,
            "goal_viability": goal_viability,
            "recovery_rate_ms": recovery_rate_ms,
            "route_deviation_m": route_deviation_m,
            "post_injection_route_deviation_m": post_injection_route_deviation_m,
            "detection_attribution_status": self.detection_attribution_status,
            "minimum_fault_distance_at_detection_m": self.minimum_fault_distance_at_detection_m,
            "baseline_confound_status": self.baseline_confound_status,
            "injected_uncertainty_encounter_status": self.encounter_time_s is not None,
            "evaluation_window_after_encounter_s": round(evaluation_window_after_encounter_s, 3),
            "baseline_uncertainty_exercised_status": self.baseline_uncertainty_exercised
            or self.baseline_reaction_observed,
            "attribution_scope": self.reaction_scope,
            "injected_uncertainty_source": [
                "injected_route_blocker",
                "obstacle_topic_degradation",
            ],
            "false_obstacle_rate_pct": false_obstacle_rate_pct,
            "false_obstacle_classification_count": self.false_obstacle_count,
            "mission_deadline_met": goal_viability["mission_deadline_met"],
            "science_rock_01_reached": self.goal_reached,
            "collision_with_obstacle_observed": self.collision_with_obstacle_observed,
            "meaningful_evaluation_rule_satisfied": meaningful_window_satisfied,
            "injection_gate_diagnostics": self.injection_gate_diagnostics,
            "fault_pose": self.injected_fault_pose,
            "fault_model_name": self.injected_model_name,
            "report": str(self.report_path),
        }

        with self.metrics_path.open("w", encoding="utf-8") as handle:
            json.dump(metrics, handle, indent=2, sort_keys=True)

        report_lines = [
            f"# Scenario Report: {self.scenario_name}",
            "",
            f"- Termination reason: `{self.termination_reason or 'shutdown'}`",
            f"- Goal status: `{goal_status}`",
            f"- Safety status: `{safety_status}`",
            f"- Overall injected-autonomy outcome: `{injected_outcome}`",
            f"- Baseline autonomy outcome: `{baseline_outcome}`",
            "",
            "## Scenario Summary",
            "",
            self.contract.get(
                "interaction_hypothesis",
                "Runtime obstacle blocking with degraded obstacle interpretation.",
            ),
            "",
            "## Runtime Facts",
            "",
            f"- Injected fault verified in Gazebo: `{self.injection_verified}`",
            f"- Injected uncertainty encountered: `{self.encounter_time_s is not None}`",
            f"- Meaningful evaluation rule satisfied: `{meaningful_window_satisfied}`",
            f"- Baseline uncertainty exercised: `{self.baseline_uncertainty_exercised or self.baseline_reaction_observed}`",
            f"- Injected fault pose: `{self.injected_fault_pose}`",
            "",
            "## Metrics",
            "",
            f"- adaptation_speed_ms: `{adaptation_speed_ms}`",
            f"- obstacle_detection_latency_ms: `{obstacle_detection_latency_ms}`",
            f"- raw_scan_detection_latency_ms: `{raw_scan_detection_latency_ms}`",
            f"- recovery_rate_ms: `{recovery_rate_ms}`",
            f"- route_deviation_m: `{route_deviation_m}`",
            f"- post_injection_route_deviation_m: `{post_injection_route_deviation_m}`",
            f"- false_obstacle_rate_pct: `{false_obstacle_rate_pct}`",
            f"- false_obstacle_classification_count: `{self.false_obstacle_count}`",
            f"- observed_control_rationale: `{self.observed_control_rationale}`",
            f"- reaction_scope: `{self.reaction_scope}`",
            f"- reaction_attribution_status: `{self.reaction_attribution_status}`",
            f"- evaluation_window_after_encounter_s: `{round(evaluation_window_after_encounter_s, 3)}`",
            "",
            "## Safety And Goal Viability",
            "",
            f"- safety_preservation: `{safety_preservation}`",
            f"- goal_viability: `{goal_viability}`",
            "",
            "## Observability Notes",
            "",
            "- Collision outcome is based on explicit observed evidence only; this scenario does not add a new contact sensor, so collision observability remains limited.",
            "- Detection attribution is based on geometry, runtime topic state, and baseline-hazard proximity rather than a dedicated perception provenance hook.",
            "",
            "## Artifacts",
            "",
            f"- Metrics JSON: `{self.metrics_path}`",
            f"- Runtime timeline: `{self.timeline_path}`",
            f"- Rosbag directory: `{self.output_root / 'rosbags'}`",
        ]
        self.report_path.write_text("\n".join(report_lines) + "\n", encoding="utf-8")
        self.log_event("scenario_finalized", {"termination_reason": self.termination_reason or "shutdown"})
        self.timeline_handle.close()


def main() -> None:
    rclpy.init()
    node = ScenarioDriverNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def handle_signal(signum: int, _frame: Any) -> None:
        signal_name = signal.Signals(signum).name.lower()
        node.note_external_shutdown(signal_name)

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        while rclpy.ok() and not node.shutdown_requested:
            executor.spin_once(timeout_sec=0.2)
    except KeyboardInterrupt:
        node.note_external_shutdown("keyboard_interrupt")
    finally:
        try:
            node.finalize()
        finally:
            executor.remove_node(node)
            node.destroy_node()
            executor.shutdown()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
