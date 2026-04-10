#!/usr/bin/env python3
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import random
import signal
import subprocess
import time
from typing import Any, Deque, Dict, Optional, Tuple

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String

import yaml


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def sector_min(scan: LaserScan, angle_min: float, angle_max: float) -> float:
    if not scan.ranges or scan.angle_increment == 0.0:
        return float("inf")
    if angle_max < angle_min:
        angle_min, angle_max = angle_max, angle_min
    start_index = max(
        0,
        min(
            len(scan.ranges) - 1,
            int(round((angle_min - scan.angle_min) / scan.angle_increment)),
        ),
    )
    end_index = max(
        0,
        min(
            len(scan.ranges) - 1,
            int(round((angle_max - scan.angle_min) / scan.angle_increment)),
        ),
    )
    minimum = float("inf")
    for index in range(min(start_index, end_index), max(start_index, end_index) + 1):
        value = scan.ranges[index]
        if not math.isfinite(value) or value <= 0.0:
            continue
        if value < scan.range_min or value > scan.range_max:
            continue
        minimum = min(minimum, float(value))
    return minimum


def iso_timestamp() -> str:
    return datetime.now(timezone.utc).isoformat()


@dataclass
class PoseState:
    x: float
    y: float
    z: float
    yaw: float


class ScenarioDriver(Node):
    FRONT_MIN = math.radians(-20.0)
    FRONT_MAX = math.radians(20.0)
    LEFT_MIN = math.radians(20.0)
    LEFT_MAX = math.radians(75.0)
    RIGHT_MIN = math.radians(-75.0)
    RIGHT_MAX = math.radians(-20.0)

    def __init__(self) -> None:
        super().__init__(
            "scenario_driver",
            automatically_declare_parameters_from_overrides=True,
        )

        self._ensure_parameter("scenario_name", "")
        self._ensure_parameter("scenario_config_file", "")
        self._ensure_parameter("scenario_contract_file", "")
        self._ensure_parameter("log_root_dir", "/ws/log")
        self._ensure_parameter("rosbag_output_dir", "")
        self._ensure_parameter("run_id", iso_timestamp().replace(":", "").replace("+00:00", "Z"))

        self.scenario_name = str(self.get_parameter("scenario_name").value)
        self.config_path = Path(str(self.get_parameter("scenario_config_file").value))
        self.contract_path = Path(str(self.get_parameter("scenario_contract_file").value))
        self.log_root_dir = Path(str(self.get_parameter("log_root_dir").value))
        self.rosbag_output_dir = Path(str(self.get_parameter("rosbag_output_dir").value))
        self.run_id = str(self.get_parameter("run_id").value)

        if not self.scenario_name:
            raise RuntimeError("scenario_name parameter is required")
        if not self.config_path.is_file():
            raise RuntimeError(f"scenario config file not found: {self.config_path}")
        if not self.contract_path.is_file():
            raise RuntimeError(f"scenario contract file not found: {self.contract_path}")

        self.config = self._load_yaml(self.config_path)
        self.contract = self._load_yaml(self.contract_path)

        self.route_start = self.config["route"]["start"]
        self.route_goal = self.config["route"]["goal"]
        self.goal_tolerance_m = float(self.route_goal["tolerance_m"])
        self.world_name = str(self.config["world_name"])
        self.timeout_s = float(self.config["evaluation"]["scenario_timeout_s"])
        self.minimum_post_encounter_window_s = float(
            self.config["evaluation"]["minimum_post_encounter_observation_window_s"]
        )
        self.trigger_config = self.config["trigger"]
        self.obstacle_config = self.config["injection"]["obstacle"]
        self.degradation_config = self.config["injection"]["perception_degradation"]
        self.attribution_config = self.config["attribution"]
        self.baseline_hazard = self.config["baseline_world"]["hazards"][0]
        self.random = random.Random(int(self.degradation_config["random_seed"]))

        self._prepare_output_paths()

        self.best_effort_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            Odometry,
            "/mobile_base_controller/odom",
            self._on_odom,
            self.best_effort_qos,
        )
        self.create_subscription(
            LaserScan,
            "/scan",
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Twist,
            "/cmd_vel",
            self._on_cmd_vel,
            self.best_effort_qos,
        )
        self.create_subscription(
            Float32,
            "/battery/soc",
            self._on_battery_soc,
            self.best_effort_qos,
        )
        self.create_subscription(
            Bool,
            "/battery/near_outpost",
            self._on_battery_near_outpost,
            self.best_effort_qos,
        )
        self.create_subscription(String, "/monitor/events", self._on_monitor_event, 10)
        self.create_subscription(
            Bool,
            "/monitor/handlerMR_009_RETURN_TO_OUTPOST_ON_LOW_BATTERY",
            self._on_mr009,
            10,
        )
        self.create_subscription(
            Bool,
            "/monitor/handlerMR_011_MAINTAIN_SPEED_WHEN_FULL_BATTERY",
            self._on_mr011,
            10,
        )

        self.degraded_front_pub = self.create_publisher(Bool, "/obstacle/front", self.best_effort_qos)
        self.degraded_left_pub = self.create_publisher(Bool, "/obstacle/left", self.best_effort_qos)
        self.degraded_right_pub = self.create_publisher(Bool, "/obstacle/right", self.best_effort_qos)
        self.degraded_state_pub = self.create_publisher(String, "/obstacle/state", self.best_effort_qos)

        self.main_timer = self.create_timer(0.1, self._on_timer)
        publish_period = 1.0 / max(1.0, float(self.degradation_config["publish_hz"]))
        self.degradation_timer = self.create_timer(publish_period, self._on_degradation_timer)

        self.pose: Optional[PoseState] = None
        self.pose_time = None
        self.raw_scan = {
            "front_min": float("inf"),
            "left_min": float("inf"),
            "right_min": float("inf"),
            "state": "CLEAR",
        }
        self.battery_soc = 0.0
        self.near_outpost = False
        self.last_cmd: Optional[Tuple[float, float, float]] = None
        self.progress_history: Deque[Tuple[float, float]] = deque(maxlen=300)
        self.distance_to_goal_history: Deque[Tuple[float, float]] = deque(maxlen=300)
        self.route_deviation_max_m = 0.0
        self.post_injection_route_deviation_max_m = 0.0
        self.minimum_injected_fault_distance_m = float("inf")
        self.minimum_fault_distance_at_detection_m = None
        self.baseline_uncertainty_exercised = False
        self.baseline_reaction_time_s = None
        self.first_post_injection_reaction: Optional[Dict[str, Any]] = None
        self.attributed_reaction: Optional[Dict[str, Any]] = None
        self.attributed_detection: Optional[Dict[str, Any]] = None
        self.raw_scan_attributed_detection: Optional[Dict[str, Any]] = None
        self.encounter_time_s: Optional[float] = None
        self.recovery_time_s: Optional[float] = None
        self.injection_time_s: Optional[float] = None
        self.injection_entity_name: Optional[str] = None
        self.injection_pose: Optional[Dict[str, float]] = None
        self.injection_failure_reason: Optional[str] = None
        self.degradation_active = False
        self.degradation_end_time_s: Optional[float] = None
        self.last_degraded_state = "CLEAR"
        self.degradation_publications = 0
        self.unsupported_classification_publications = 0
        self.monitor_events = []
        self.mr009_fired = False
        self.mr011_fired = False
        self.start_sim_time_s: Optional[float] = None
        self.start_wall_time = iso_timestamp()
        self.goal_reached_time_s: Optional[float] = None
        self.shutdown_requested = False
        self.finalized = False
        self.termination_reason: Optional[str] = None
        self.last_trigger_snapshot_s = -1.0
        self.report_notes = [
            "Baseline BT, mission config, perception node, monitors, and world file were evaluated unchanged.",
            "The scenario launch starts with battery=0.79 to avoid unrelated MR_011 dominance from the baseline full-speed constraint.",
        ]

        self._log_event(
            "scenario_started",
            scenario_name=self.scenario_name,
            config_file=str(self.config_path),
            contract_file=str(self.contract_path),
            rosbag_output_dir=str(self.rosbag_output_dir),
        )

    def _ensure_parameter(self, name: str, default: Any) -> None:
        if not self.has_parameter(name):
            self.declare_parameter(name, default)

    def _load_yaml(self, path: Path) -> Dict[str, Any]:
        with path.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
        if not isinstance(data, dict):
            raise RuntimeError(f"expected a mapping in {path}")
        return data

    def _prepare_output_paths(self) -> None:
        self.scenario_output_dir = self.log_root_dir / self.scenario_name
        self.metrics_dir = self.scenario_output_dir / "metrics"
        self.runtime_dir = self.scenario_output_dir / "runtime"
        self.rosbag_dir = self.scenario_output_dir / "rosbags"
        for directory in (
            self.scenario_output_dir,
            self.metrics_dir,
            self.runtime_dir,
            self.rosbag_dir,
        ):
            directory.mkdir(parents=True, exist_ok=True)
        self.report_path = self.scenario_output_dir / f"{self.scenario_name}_report.md"
        self.metrics_path = self.metrics_dir / f"{self.run_id}_metrics.json"
        self.timeline_path = self.runtime_dir / f"{self.run_id}_timeline.jsonl"
        self.timeline_handle = self.timeline_path.open("a", encoding="utf-8")

    def _sim_time_s(self) -> float:
        now_s = self.get_clock().now().nanoseconds / 1e9
        if self.start_sim_time_s is None:
            self.start_sim_time_s = now_s
        return max(0.0, now_s - self.start_sim_time_s)

    def _log_event(self, event: str, **data: Any) -> None:
        payload = {
            "event": event,
            "sim_time_s": round(self._sim_time_s(), 3),
            "wall_time": iso_timestamp(),
        }
        payload.update(self._jsonify(data))
        self.timeline_handle.write(json.dumps(payload, sort_keys=True) + "\n")
        self.timeline_handle.flush()

    def _jsonify(self, value: Any) -> Any:
        if isinstance(value, Path):
            return str(value)
        if isinstance(value, dict):
            return {str(key): self._jsonify(item) for key, item in value.items()}
        if isinstance(value, (list, tuple)):
            return [self._jsonify(item) for item in value]
        if isinstance(value, float):
            if math.isfinite(value):
                return round(value, 4)
            return None
        return value

    def _on_odom(self, msg: Odometry) -> None:
        self.pose = PoseState(
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            z=float(msg.pose.pose.position.z),
            yaw=yaw_from_quaternion(msg),
        )
        self.pose_time = self.get_clock().now()
        sim_time = self._sim_time_s()
        progress_ratio = self._route_progress_ratio(self.pose.x, self.pose.y)
        distance_to_goal = self._distance_to_goal()
        self.progress_history.append((sim_time, progress_ratio))
        self.distance_to_goal_history.append((sim_time, distance_to_goal))
        self.route_deviation_max_m = max(
            self.route_deviation_max_m,
            self._distance_to_route_line(self.pose.x, self.pose.y),
        )
        if self.injection_time_s is not None:
            self.post_injection_route_deviation_max_m = max(
                self.post_injection_route_deviation_max_m,
                self._distance_to_route_line(self.pose.x, self.pose.y),
            )
            distance_to_fault, _ = self._injected_fault_geometry()
            if distance_to_fault is not None:
                self.minimum_injected_fault_distance_m = min(
                    self.minimum_injected_fault_distance_m, distance_to_fault
                )
        if self.goal_reached_time_s is None and distance_to_goal <= self.goal_tolerance_m:
            self.goal_reached_time_s = sim_time
            self._log_event(
                "goal_reached",
                distance_to_goal_m=distance_to_goal,
                progress_ratio=progress_ratio,
            )

    def _on_scan(self, msg: LaserScan) -> None:
        front_min = sector_min(msg, self.FRONT_MIN, self.FRONT_MAX)
        left_min = sector_min(msg, self.LEFT_MIN, self.LEFT_MAX)
        right_min = sector_min(msg, self.RIGHT_MIN, self.RIGHT_MAX)
        obstacle_threshold = float(self.attribution_config["obstacle_threshold_m"])
        if front_min < obstacle_threshold:
            state = "FRONT"
        elif left_min < obstacle_threshold:
            state = "LEFT"
        elif right_min < obstacle_threshold:
            state = "RIGHT"
        else:
            state = "CLEAR"
        self.raw_scan = {
            "front_min": front_min,
            "left_min": left_min,
            "right_min": right_min,
            "state": state,
        }
        if self.injection_time_s is not None and self.attributed_detection is None:
            self._maybe_record_detection()

    def _on_cmd_vel(self, msg: Twist) -> None:
        sim_time = self._sim_time_s()
        linear = float(msg.linear.x)
        angular = float(msg.angular.z)
        last_linear = None
        last_angular = None
        if self.last_cmd is not None:
            _, last_linear, last_angular = self.last_cmd
        self.last_cmd = (sim_time, linear, angular)

        if last_linear is None or last_angular is None:
            return

        significant_change = (
            abs(linear - last_linear) >= 0.2
            or abs(angular - last_angular) >= 0.2
            or (linear < -0.05 <= last_linear)
            or (last_linear < -0.05 <= linear)
        )
        if not significant_change:
            return

        rationale = self._classify_control_rationale(linear, angular, last_linear)
        reaction_scope = self._classify_reaction_scope()
        attributable, rejection_reason = self._reaction_attribution_decision(rationale, reaction_scope)
        context = self._active_context_snapshot()
        self._log_event(
            "cmd_vel_changed",
            linear_x=linear,
            angular_z=angular,
            previous_linear_x=last_linear,
            previous_angular_z=last_angular,
        )
        self._log_event(
            "control_rationale_classified",
            observed_control_rationale=rationale,
            reaction_scope=reaction_scope,
            reaction_attribution_status=attributable,
            active_context_at_reaction=context,
        )

        if self.injection_time_s is None and reaction_scope == "baseline_only" and rationale == "obstacle_avoidance":
            if not self.baseline_uncertainty_exercised:
                self.baseline_uncertainty_exercised = True
                self.baseline_reaction_time_s = sim_time
                self._log_event(
                    "baseline_reaction_observed",
                    observed_control_rationale=rationale,
                    reaction_scope=reaction_scope,
                    active_context_at_reaction=context,
                )
            return

        if self.injection_time_s is None:
            return

        if self.first_post_injection_reaction is None and rationale != "unknown":
            self.first_post_injection_reaction = {
                "time_s": sim_time,
                "observed_control_rationale": rationale,
                "reaction_scope": reaction_scope,
                "reaction_attribution_status": attributable,
                "active_context_at_reaction": context,
            }

        if rationale == "obstacle_avoidance":
            self._log_event(
                "reaction_candidate",
                observed_control_rationale=rationale,
                reaction_scope=reaction_scope,
                reaction_attribution_status=attributable,
                active_context_at_reaction=context,
            )
            if attributable and self.attributed_reaction is None:
                self.attributed_reaction = {
                    "time_s": sim_time,
                    "observed_control_rationale": rationale,
                    "reaction_scope": reaction_scope,
                    "reaction_attribution_status": True,
                    "active_context_at_reaction": context,
                }
                self._log_event(
                    "reaction_attributed",
                    injected_uncertainty_source=self._active_injected_sources(),
                    active_context_at_reaction=context,
                )
            elif not attributable:
                self._log_event(
                    "reaction_not_attributed",
                    rejection_reason=rejection_reason,
                    active_context_at_reaction=context,
                )

    def _on_battery_soc(self, msg: Float32) -> None:
        self.battery_soc = float(msg.data)

    def _on_battery_near_outpost(self, msg: Bool) -> None:
        self.near_outpost = bool(msg.data)

    def _on_monitor_event(self, msg: String) -> None:
        self.monitor_events.append(msg.data)
        self._log_event("baseline_monitor_violation_active", event_message=msg.data)

    def _on_mr009(self, msg: Bool) -> None:
        if msg.data:
            self.mr009_fired = True
            self._log_event("baseline_monitor_violation_active", monitor="MR_009")

    def _on_mr011(self, msg: Bool) -> None:
        if msg.data:
            self.mr011_fired = True
            self._log_event("baseline_monitor_violation_active", monitor="MR_011")

    def _distance_to_goal(self) -> float:
        if self.pose is None:
            return float("inf")
        return math.hypot(
            self.route_goal["x"] - self.pose.x,
            self.route_goal["y"] - self.pose.y,
        )

    def _route_progress_ratio(self, x: float, y: float) -> float:
        start_x = float(self.route_start["x"])
        start_y = float(self.route_start["y"])
        goal_x = float(self.route_goal["x"])
        goal_y = float(self.route_goal["y"])
        vx = goal_x - start_x
        vy = goal_y - start_y
        denom = max(1e-6, vx * vx + vy * vy)
        wx = x - start_x
        wy = y - start_y
        return clamp((wx * vx + wy * vy) / denom, 0.0, 1.0)

    def _distance_to_route_line(self, x: float, y: float) -> float:
        start_x = float(self.route_start["x"])
        start_y = float(self.route_start["y"])
        goal_x = float(self.route_goal["x"])
        goal_y = float(self.route_goal["y"])
        vx = goal_x - start_x
        vy = goal_y - start_y
        denom = math.hypot(vx, vy)
        if denom <= 1e-6:
            return 0.0
        return abs(vy * x - vx * y + goal_x * start_y - goal_y * start_x) / denom

    def _distance_to_baseline_hazard(self) -> Optional[float]:
        if self.pose is None:
            return None
        return math.hypot(
            float(self.baseline_hazard["x"]) - self.pose.x,
            float(self.baseline_hazard["y"]) - self.pose.y,
        )

    def _injected_fault_geometry(self) -> Tuple[Optional[float], Optional[float]]:
        if self.pose is None or self.injection_pose is None:
            return None, None
        dx = float(self.injection_pose["x"]) - self.pose.x
        dy = float(self.injection_pose["y"]) - self.pose.y
        distance = math.hypot(dx, dy)
        relative_angle = wrap_pi(math.atan2(dy, dx) - self.pose.yaw)
        return distance, relative_angle

    def _trigger_ready(self) -> Tuple[bool, Dict[str, Any]]:
        if self.pose is None or self.last_cmd is None:
            return False, {"reason": "awaiting_pose_or_cmd"}
        sim_time = self._sim_time_s()
        progress_ratio = self._route_progress_ratio(self.pose.x, self.pose.y)
        distance_to_goal = self._distance_to_goal()
        hazard_distance = self._distance_to_baseline_hazard()
        _, current_linear, _ = self.last_cmd
        quiet_required = bool(self.trigger_config["require_monitor_quiet"])
        progress_ok = progress_ratio >= float(self.trigger_config["min_progress_ratio"])
        progress_not_too_late = progress_ratio <= float(self.trigger_config["max_progress_ratio"])
        forward_speed_ok = current_linear >= float(self.trigger_config["min_forward_speed_mps"])
        hazard_clear = (
            hazard_distance is not None
            and hazard_distance >= float(self.trigger_config["min_distance_from_baseline_hazard_m"])
        )
        remaining_goal_ok = distance_to_goal >= float(self.trigger_config["min_remaining_goal_distance_m"])
        time_ok = sim_time >= float(self.trigger_config["min_trigger_time_s"])
        monitor_quiet = not (self.mr009_fired or self.mr011_fired)
        progress_window_gain = self._progress_over_window(
            float(self.trigger_config["sustained_progress_window_s"])
        )
        sustained_progress_ok = progress_window_gain > 0.005
        obstacle_context_clear = self.raw_scan["state"] == "CLEAR"

        gates = {
            "progress_ok": progress_ok,
            "progress_not_too_late": progress_not_too_late,
            "forward_speed_ok": forward_speed_ok,
            "hazard_clear": hazard_clear,
            "remaining_goal_ok": remaining_goal_ok,
            "time_ok": time_ok,
            "monitor_quiet": monitor_quiet,
            "sustained_progress_ok": sustained_progress_ok,
            "obstacle_context_clear": obstacle_context_clear,
            "progress_ratio": progress_ratio,
            "distance_to_goal_m": distance_to_goal,
            "hazard_distance_m": hazard_distance,
            "current_linear_speed_mps": current_linear,
            "progress_window_gain": progress_window_gain,
        }
        ready = all(
            (
                progress_ok,
                progress_not_too_late,
                forward_speed_ok,
                hazard_clear,
                remaining_goal_ok,
                time_ok,
                sustained_progress_ok,
                obstacle_context_clear,
                monitor_quiet if quiet_required else True,
            )
        )
        return ready, gates

    def _progress_over_window(self, window_s: float) -> float:
        if len(self.progress_history) < 2:
            return 0.0
        now_s = self._sim_time_s()
        earliest = None
        latest = None
        for time_s, progress in self.progress_history:
            if time_s >= now_s - window_s:
                if earliest is None:
                    earliest = progress
                latest = progress
        if earliest is None or latest is None:
            return 0.0
        return latest - earliest

    def _log_trigger_snapshot_if_needed(self) -> None:
        sim_time = self._sim_time_s()
        if sim_time - self.last_trigger_snapshot_s < 5.0:
            return
        self.last_trigger_snapshot_s = sim_time
        ready, gates = self._trigger_ready()
        self._log_event(
            "trigger_state_snapshot",
            trigger_ready=ready,
            trigger_state=gates,
        )

    def _inject_faults(self) -> None:
        if self.pose is None:
            return
        distance_to_goal = self._distance_to_goal()
        goal_heading = math.atan2(
            float(self.route_goal["y"]) - self.pose.y,
            float(self.route_goal["x"]) - self.pose.x,
        )
        ahead = float(self.obstacle_config["spawn_ahead_distance_m"])
        lateral = float(self.obstacle_config["lateral_offset_m"])
        spawn_x = self.pose.x + ahead * math.cos(goal_heading) - lateral * math.sin(goal_heading)
        spawn_y = self.pose.y + ahead * math.sin(goal_heading) + lateral * math.cos(goal_heading)
        spawn_z = self.pose.z + float(self.obstacle_config["z_offset_from_rover_m"])
        spawn_yaw = goal_heading + float(self.obstacle_config["yaw_offset_rad"])
        entity_name = f"{self.obstacle_config['entity_name_prefix']}_{self.run_id.lower()}"
        model_share = Path(get_package_share_directory("spacetry_models"))
        model_file = model_share / "models" / str(self.obstacle_config["model_name"]) / "model.sdf"
        models_root = model_share / "models"
        environment = os.environ.copy()
        environment["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join(
            [str(models_root), environment.get("GZ_SIM_RESOURCE_PATH", "")]
        ).strip(os.pathsep)
        environment["SDF_PATH"] = os.pathsep.join(
            [str(models_root), environment.get("SDF_PATH", "")]
        ).strip(os.pathsep)

        self.injection_entity_name = entity_name
        self.injection_pose = {
            "x": spawn_x,
            "y": spawn_y,
            "z": spawn_z,
            "yaw": spawn_yaw,
        }

        self._log_event(
            "fault_injection_gate_passed",
            why_injection_allowed="route progress past baseline hazard with quiet monitors",
            remaining_mission_time_s=self.timeout_s - self._sim_time_s(),
            progress_ratio=self._route_progress_ratio(self.pose.x, self.pose.y),
            baseline_monitors_active=self.mr009_fired or self.mr011_fired,
            distance_to_goal_m=distance_to_goal,
            injection_pose=self.injection_pose,
        )

        command = [
            "ros2",
            "run",
            "ros_gz_sim",
            "create",
            "-world",
            self.world_name,
            "-name",
            entity_name,
            "-allow_renaming",
            "true",
            "-file",
            str(model_file),
            "-x",
            f"{spawn_x:.3f}",
            "-y",
            f"{spawn_y:.3f}",
            "-z",
            f"{spawn_z:.3f}",
            "-Y",
            f"{spawn_yaw:.3f}",
        ]
        try:
            result = subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
                env=environment,
                timeout=30,
            )
        except Exception as exc:
            self.injection_failure_reason = str(exc)
            self._log_event("fault_injection_failed", error=str(exc), command=command)
            return

        if result.returncode != 0:
            self.injection_failure_reason = result.stderr.strip() or result.stdout.strip() or "spawn failed"
            self._log_event(
                "fault_injection_failed",
                command=command,
                returncode=result.returncode,
                stdout=result.stdout,
                stderr=result.stderr,
            )
            return

        verified = False
        retries = int(self.obstacle_config["verification_retries"])
        retry_period_s = float(self.obstacle_config["verification_retry_period_s"])
        for _ in range(retries):
            query = subprocess.run(
                ["gz", "model", "--list"],
                check=False,
                capture_output=True,
                text=True,
                env=environment,
                timeout=10,
            )
            if entity_name in query.stdout.split():
                verified = True
                break
            time.sleep(retry_period_s)

        if not verified:
            self.injection_failure_reason = "spawn command succeeded but Gazebo entity verification failed"
            self._log_event(
                "fault_injection_failed",
                command=command,
                stdout=result.stdout,
                stderr=result.stderr,
                verification_output=query.stdout,
            )
            return

        self.injection_time_s = self._sim_time_s()
        self.degradation_active = bool(self.degradation_config["enabled"])
        self.degradation_end_time_s = self.injection_time_s + float(self.degradation_config["duration_s"])
        self._log_event(
            "fault_injected",
            injected_fault_pose=self.injection_pose,
            injected_fault_entity=entity_name,
            model_file=str(model_file),
            gz_sim_resource_path=environment.get("GZ_SIM_RESOURCE_PATH"),
            sdf_path=environment.get("SDF_PATH"),
        )

    def _encounter_rule_satisfied(self) -> bool:
        if self.injection_time_s is None:
            return False
        distance, relative_angle = self._injected_fault_geometry()
        if distance is None or relative_angle is None:
            return False
        max_distance = float(self.attribution_config["encounter_distance_m"])
        max_angle = math.radians(float(self.attribution_config["encounter_heading_half_angle_deg"]))
        return distance <= max_distance and abs(relative_angle) <= max_angle

    def _pose_snapshot(self) -> Optional[Dict[str, float]]:
        if self.pose is None:
            return None
        return {
            "x": self.pose.x,
            "y": self.pose.y,
            "z": self.pose.z,
            "yaw": self.pose.yaw,
        }

    def _baseline_hazard_confound(self, distance_to_fault: float) -> Tuple[bool, Optional[float]]:
        hazard_distance = self._distance_to_baseline_hazard()
        margin = float(self.attribution_config["fault_distance_margin_from_baseline_hazard_m"])
        return (
            hazard_distance is not None and hazard_distance + margin < distance_to_fault,
            hazard_distance,
        )

    def _maybe_record_autonomy_stack_detection(self, actual_state: str, degraded_state: str) -> None:
        if self.attributed_detection is not None or self.injection_time_s is None:
            return
        if actual_state == "CLEAR" or degraded_state == "CLEAR":
            return
        distance, relative_angle = self._injected_fault_geometry()
        if distance is None or relative_angle is None:
            return
        baseline_confound, hazard_distance = self._baseline_hazard_confound(distance)
        if baseline_confound:
            self._log_event(
                "fault_detection_rejected",
                rejection_reason="baseline hazard closer than injected obstacle",
                sensor_signal="/obstacle/state",
                actual_state=actual_state,
                published_obstacle_state=degraded_state,
                injected_fault_distance_m=distance,
                baseline_hazard_distance_m=hazard_distance,
                raw_scan=self.raw_scan,
                pose=self._pose_snapshot(),
            )
            return
        self.minimum_fault_distance_at_detection_m = distance
        self.attributed_detection = {
            "time_s": self._sim_time_s(),
            "distance_m": distance,
            "relative_angle_rad": relative_angle,
            "signal_source": "/obstacle/state",
            "actual_state": actual_state,
            "published_obstacle_state": degraded_state,
            "pose": self._pose_snapshot(),
        }
        self._log_event(
            "fault_detection_attributed",
            injected_uncertainty_source=self._active_injected_sources(),
            rover_to_fault_distance_m=distance,
            sensor_signal="/obstacle/state",
            actual_state=actual_state,
            published_obstacle_state=degraded_state,
            raw_scan=self.raw_scan,
            pose=self._pose_snapshot(),
        )

    def _maybe_record_detection(self) -> None:
        distance, relative_angle = self._injected_fault_geometry()
        if (
            distance is None
            or relative_angle is None
            or self.injection_time_s is None
            or self.raw_scan_attributed_detection is not None
        ):
            return
        candidate = self.raw_scan["state"] != "CLEAR"
        if not candidate:
            return
        if self._encounter_rule_satisfied():
            baseline_confound, hazard_distance = self._baseline_hazard_confound(distance)
            if baseline_confound:
                self._log_event(
                    "fault_detection_rejected",
                    rejection_reason="baseline hazard closer than injected obstacle",
                    sensor_signal="/scan",
                    raw_scan=self.raw_scan,
                    injected_fault_distance_m=distance,
                    baseline_hazard_distance_m=hazard_distance,
                    pose=self._pose_snapshot(),
                )
                return
            self.raw_scan_attributed_detection = {
                "time_s": self._sim_time_s(),
                "distance_m": distance,
                "relative_angle_rad": relative_angle,
                "raw_scan_state": self.raw_scan["state"],
                "signal_source": "/scan",
                "pose": self._pose_snapshot(),
            }
            self._log_event(
                "raw_scan_detection_attributed",
                injected_uncertainty_source=self._active_injected_sources(),
                rover_to_fault_distance_m=distance,
                sensor_signal="scan_sector_min",
                raw_scan=self.raw_scan,
                pose=self._pose_snapshot(),
            )
        else:
            self._log_event(
                "fault_detection_candidate",
                rover_to_fault_distance_m=distance,
                relative_angle_rad=relative_angle,
                sensor_signal="/scan",
                raw_scan=self.raw_scan,
                pose=self._pose_snapshot(),
            )

    def _current_actual_obstacle_state(self) -> str:
        distance, relative_angle = self._injected_fault_geometry()
        if distance is None or relative_angle is None:
            return "CLEAR"
        if distance > float(self.attribution_config["obstacle_threshold_m"]):
            return "CLEAR"
        if self.RIGHT_MAX <= relative_angle < self.FRONT_MIN:
            return "RIGHT"
        if self.FRONT_MIN <= relative_angle <= self.FRONT_MAX:
            return "FRONT"
        if self.LEFT_MIN < relative_angle <= self.LEFT_MAX:
            return "LEFT"
        return "CLEAR"

    def _degraded_state(self, actual_state: str) -> str:
        if actual_state == "CLEAR":
            if self.random.random() < float(self.degradation_config["lateral_flip_bias"]) * 0.3:
                return self.random.choice(["LEFT", "RIGHT"])
            return "CLEAR"
        elapsed = max(0.0, self._sim_time_s() - float(self.injection_time_s or 0.0))
        ramp = clamp(
            elapsed / max(0.1, float(self.degradation_config["ramp_up_s"])),
            0.0,
            1.0,
        )
        clear_bias = (
            float(self.degradation_config["clear_bias_start"])
            + (
                float(self.degradation_config["clear_bias_end"])
                - float(self.degradation_config["clear_bias_start"])
            )
            * ramp
        )
        lateral_flip_bias = float(self.degradation_config["lateral_flip_bias"])
        hold_last_bias = float(self.degradation_config["hold_last_bias"])
        draw = self.random.random()
        if draw < clear_bias:
            return "CLEAR"
        if draw < clear_bias + lateral_flip_bias:
            if actual_state == "FRONT":
                return self.random.choice(["LEFT", "RIGHT"])
            return "RIGHT" if actual_state == "LEFT" else "LEFT"
        if draw < clear_bias + lateral_flip_bias + hold_last_bias:
            return self.last_degraded_state
        return actual_state

    def _publish_state(self, state: str) -> None:
        front = state == "FRONT"
        left = state == "LEFT"
        right = state == "RIGHT"
        self.degraded_front_pub.publish(Bool(data=front))
        self.degraded_left_pub.publish(Bool(data=left))
        self.degraded_right_pub.publish(Bool(data=right))
        self.degraded_state_pub.publish(String(data=state))

    def _on_degradation_timer(self) -> None:
        if not self.degradation_active or self.injection_time_s is None:
            return
        if self.degradation_end_time_s is not None and self._sim_time_s() >= self.degradation_end_time_s:
            self.degradation_active = False
            self._log_event("fault_degradation_completed")
            return
        actual_state = self._current_actual_obstacle_state()
        degraded_state = self._degraded_state(actual_state)
        self._publish_state(degraded_state)
        self._maybe_record_autonomy_stack_detection(actual_state, degraded_state)
        self.degradation_publications += 1
        if degraded_state != actual_state:
            self.unsupported_classification_publications += 1
        self.last_degraded_state = degraded_state
        self._log_event(
            "fault_state_applied",
            actual_state=actual_state,
            degraded_state=degraded_state,
            unsupported=degraded_state != actual_state,
        )

    def _classify_control_rationale(self, linear: float, angular: float, previous_linear: float) -> str:
        obstacle_context = self.raw_scan["state"] != "CLEAR" or self._encounter_rule_satisfied()
        if (self.mr009_fired or self.mr011_fired) and abs(linear) < 0.05:
            return "monitor_enforcement"
        if linear < -0.05:
            return "obstacle_avoidance"
        if obstacle_context and (
            abs(angular) >= float(self.attribution_config["cmd_vel_turn_threshold_radps"])
            or previous_linear - linear >= float(self.attribution_config["cmd_vel_drop_threshold_mps"])
        ):
            return "obstacle_avoidance"
        if abs(angular) >= float(self.attribution_config["cmd_vel_turn_threshold_radps"]):
            return "goal_alignment"
        return "unknown"

    def _classify_reaction_scope(self) -> str:
        distance_to_fault, _ = self._injected_fault_geometry()
        hazard_distance = self._distance_to_baseline_hazard()
        hazard_context_radius = float(self.baseline_hazard["context_radius_m"])
        injected_active = distance_to_fault is not None and distance_to_fault <= float(
            self.attribution_config["encounter_distance_m"]
        ) + 3.0
        baseline_active = hazard_distance is not None and hazard_distance <= hazard_context_radius
        if injected_active and baseline_active:
            return "baseline_and_injected"
        if injected_active:
            return "injected_only"
        if baseline_active:
            return "baseline_only"
        return "indeterminate"

    def _reaction_attribution_decision(
        self,
        rationale: str,
        reaction_scope: str,
    ) -> Tuple[bool, Optional[str]]:
        if rationale != "obstacle_avoidance":
            return False, "non_avoidance_rationale"
        if self.injection_time_s is None:
            return False, "injection_not_active"
        if self.encounter_time_s is None:
            return False, "encounter_not_confirmed"
        if self._sim_time_s() - self.encounter_time_s > float(self.attribution_config["reaction_window_s"]):
            return False, "reaction_outside_window"
        if self.mr009_fired or self.mr011_fired:
            return False, "monitor_violation_active"
        distance_to_fault, _ = self._injected_fault_geometry()
        hazard_distance = self._distance_to_baseline_hazard()
        margin = float(self.attribution_config["fault_distance_margin_from_baseline_hazard_m"])
        if distance_to_fault is None:
            return False, "fault_distance_unavailable"
        if hazard_distance is not None and hazard_distance + margin < distance_to_fault:
            return False, "baseline_hazard_closer_than_injected_fault"
        if reaction_scope not in ("injected_only", "baseline_and_injected"):
            return False, "reaction_scope_not_injected"
        return True, None

    def _active_context_snapshot(self) -> Dict[str, Any]:
        distance_to_fault, relative_angle = self._injected_fault_geometry()
        return {
            "progress_ratio": self._route_progress_ratio(self.pose.x, self.pose.y) if self.pose else None,
            "distance_to_goal_m": self._distance_to_goal(),
            "distance_to_injected_fault_m": distance_to_fault,
            "distance_to_baseline_hazard_m": self._distance_to_baseline_hazard(),
            "relative_angle_to_injected_fault_rad": relative_angle,
            "raw_scan_state": self.raw_scan["state"],
            "raw_scan_front_min_m": self.raw_scan["front_min"],
            "monitor_mr009_active": self.mr009_fired,
            "monitor_mr011_active": self.mr011_fired,
            "degradation_active": self.degradation_active,
            "injection_active": self.injection_time_s is not None,
        }

    def _active_injected_sources(self) -> list[str]:
        sources = []
        if self.injection_time_s is not None:
            sources.append("runtime_blocking_rock")
        if self.degradation_active or self.degradation_publications > 0:
            sources.append("degraded_obstacle_interpretation")
        return sources

    def _stable_progress_resumed(self) -> bool:
        if self.attributed_reaction is None or self.last_cmd is None:
            return False
        if self.raw_scan["state"] != "CLEAR":
            return False
        _, current_linear, _ = self.last_cmd
        if current_linear < 0.35:
            return False
        progress_gain = self._progress_over_window(float(self.attribution_config["stable_progress_window_s"]))
        return progress_gain > 0.02

    def _on_timer(self) -> None:
        if self.finalized:
            return

        if self.injection_time_s is None:
            self._log_trigger_snapshot_if_needed()
            ready, _ = self._trigger_ready()
            if ready:
                self._inject_faults()
                if self.injection_failure_reason is not None:
                    self._finalize("injection_failed")
                    return

        if self.injection_time_s is not None and self.encounter_time_s is None and self._encounter_rule_satisfied():
            self.encounter_time_s = self._sim_time_s()
            self._log_event(
                "fault_encountered",
                rover_to_fault_distance_m=self._injected_fault_geometry()[0],
                evaluation_window_after_encounter_s=self.timeout_s - self.encounter_time_s,
            )

        if self.recovery_time_s is None and self.attributed_reaction is not None and self._stable_progress_resumed():
            self.recovery_time_s = self._sim_time_s()
            self._log_event(
                "recovery_observed",
                recovery_time_ms=(self.recovery_time_s - self.attributed_reaction["time_s"]) * 1000.0,
            )

        if self.goal_reached_time_s is not None:
            self._finalize("goal_reached")
            return

        if self._sim_time_s() >= self.timeout_s:
            self._finalize("timeout")

    def _evaluation_window_after_encounter_s(self, end_time_s: float) -> float:
        if self.encounter_time_s is None:
            return 0.0
        return max(0.0, end_time_s - self.encounter_time_s)

    def _meaningful_evaluation(self, end_time_s: float) -> bool:
        return self._evaluation_window_after_encounter_s(end_time_s) >= self.minimum_post_encounter_window_s

    def _collision_proxy(self) -> bool:
        return self.minimum_injected_fault_distance_m <= float(self.attribution_config["collision_distance_m"])

    def _compute_metrics(self, reason: str) -> Dict[str, Any]:
        end_time_s = self._sim_time_s()
        evaluation_window_after_encounter_s = self._evaluation_window_after_encounter_s(end_time_s)
        meaningful_evaluation = self._meaningful_evaluation(end_time_s)
        goal_reached = self.goal_reached_time_s is not None
        mission_deadline_met = goal_reached and self.goal_reached_time_s <= self.timeout_s
        collision_with_obstacle = self._collision_proxy()
        safety_preservation = {
            "MR_009": not self.mr009_fired,
            "MR_011": not self.mr011_fired,
            "collision_with_obstacle": not collision_with_obstacle,
        }
        safety_status = "PASS"
        if collision_with_obstacle:
            safety_status = "FAIL"
        elif not all(safety_preservation.values()):
            safety_status = "DEGRADED"

        detection_latency_ms = None
        if self.attributed_detection is not None and self.injection_time_s is not None:
            detection_latency_ms = (self.attributed_detection["time_s"] - self.injection_time_s) * 1000.0

        raw_scan_detection_latency_ms = None
        if self.raw_scan_attributed_detection is not None and self.injection_time_s is not None:
            raw_scan_detection_latency_ms = (
                self.raw_scan_attributed_detection["time_s"] - self.injection_time_s
            ) * 1000.0

        adaptation_speed_ms = None
        if self.attributed_reaction is not None and self.injection_time_s is not None:
            adaptation_speed_ms = (self.attributed_reaction["time_s"] - self.injection_time_s) * 1000.0

        recovery_rate_ms = None
        if self.recovery_time_s is not None and self.attributed_reaction is not None:
            recovery_rate_ms = (self.recovery_time_s - self.attributed_reaction["time_s"]) * 1000.0

        false_obstacle_rate = None
        if self.degradation_publications > 0:
            false_obstacle_rate = (
                self.unsupported_classification_publications / self.degradation_publications
            )

        if self.encounter_time_s is None:
            injected_outcome = "UNTESTED"
        elif not meaningful_evaluation:
            injected_outcome = "INCONCLUSIVE"
        elif collision_with_obstacle or self.attributed_reaction is None:
            injected_outcome = "FAIL"
        elif goal_reached and safety_status == "PASS":
            injected_outcome = "PASS"
        else:
            injected_outcome = "DEGRADED"

        if not self.baseline_uncertainty_exercised:
            baseline_outcome = "NOT_EVALUATED"
        elif collision_with_obstacle:
            baseline_outcome = "FAIL"
        else:
            baseline_outcome = "PASS"

        autonomy_status = "PASS"
        if injected_outcome == "FAIL":
            autonomy_status = "FAIL"
        elif injected_outcome in ("DEGRADED", "INCONCLUSIVE"):
            autonomy_status = "DEGRADED"
        elif injected_outcome == "UNTESTED":
            autonomy_status = "UNTESTED"

        first_reaction = self.attributed_reaction or self.first_post_injection_reaction or {}
        baseline_confound_status = first_reaction.get("reaction_scope") in (
            "baseline_only",
            "baseline_and_injected",
        )
        metrics = {
            "scenario_name": self.scenario_name,
            "run_id": self.run_id,
            "termination_reason": reason,
            "start_wall_time": self.start_wall_time,
            "end_wall_time": iso_timestamp(),
            "scenario_elapsed_s": end_time_s,
            "goal_status": "PASS" if goal_reached else "FAIL",
            "safety_status": safety_status,
            "autonomy_assessment": autonomy_status,
            "baseline_outcome_assessment": baseline_outcome,
            "injected_outcome_assessment": injected_outcome,
            "outcome_assessment": injected_outcome,
            "autonomy_reaction_status": bool(self.first_post_injection_reaction or self.attributed_reaction),
            "observed_control_rationale": first_reaction.get("observed_control_rationale", "unknown"),
            "reaction_scope": first_reaction.get("reaction_scope", "indeterminate"),
            "reaction_attribution_status": bool(self.attributed_reaction),
            "active_context_at_reaction": first_reaction.get("active_context_at_reaction"),
            "adaptation_speed_ms": adaptation_speed_ms,
            "detection_attribution_status": bool(self.attributed_detection),
            "minimum_fault_distance_at_detection_m": self.minimum_fault_distance_at_detection_m,
            "detection_signal_source": (
                self.attributed_detection.get("signal_source")
                if self.attributed_detection is not None
                else None
            ),
            "baseline_confound_status": baseline_confound_status,
            "injected_uncertainty_encounter_status": self.encounter_time_s is not None,
            "evaluation_window_after_encounter_s": evaluation_window_after_encounter_s,
            "meaningful_evaluation_rule_satisfied": meaningful_evaluation,
            "baseline_uncertainty_exercised_status": self.baseline_uncertainty_exercised,
            "attribution_scope": first_reaction.get("reaction_scope", "indeterminate"),
            "injected_uncertainty_source": self._active_injected_sources(),
            "obstacle_detection_latency_ms": detection_latency_ms,
            "raw_scan_detection_latency_ms": raw_scan_detection_latency_ms,
            "false_obstacle_rate": false_obstacle_rate,
            "recovery_rate_ms": recovery_rate_ms,
            "route_deviation_m": self.route_deviation_max_m,
            "post_injection_route_deviation_m": (
                self.post_injection_route_deviation_max_m
                if self.injection_time_s is not None
                else None
            ),
            "safety_preservation": safety_preservation,
            "goal_viability": {
                "science_rock_01_reached": goal_reached,
                "mission_deadline_met": mission_deadline_met,
            },
            "injection_pose": self.injection_pose,
            "injection_entity_name": self.injection_entity_name,
            "injection_failure_reason": self.injection_failure_reason,
            "rosbag_output_dir": str(self.rosbag_output_dir),
            "monitor_events": self.monitor_events,
        }
        return self._jsonify(metrics)

    def _build_report(self, metrics: Dict[str, Any]) -> str:
        lines = [
            f"# {self.scenario_name}",
            "",
            f"- Run ID: {self.run_id}",
            f"- Termination reason: {metrics['termination_reason']}",
            f"- Start wall time: {metrics['start_wall_time']}",
            f"- End wall time: {metrics['end_wall_time']}",
            f"- Scenario elapsed: {metrics['scenario_elapsed_s']} s",
            "",
            "## Scenario Contract Summary",
            "",
            f"- Primary evaluation target: {self.contract['primary_evaluation_target']}",
            f"- Secondary injected uncertainties: {', '.join(self.contract['secondary_injected_uncertainties'])}",
            f"- Interaction hypothesis: {self.contract['interaction_hypothesis']}",
            f"- Encounter rule: {self.contract['encounter_rule']}",
            f"- Meaningful evaluation rule: {self.contract['meaningful_evaluation_rule']}",
            "",
            "## Outcomes",
            "",
            f"- Goal status: {metrics['goal_status']}",
            f"- Safety status: {metrics['safety_status']}",
            f"- Autonomy assessment: {metrics['autonomy_assessment']}",
            f"- Baseline outcome assessment: {metrics['baseline_outcome_assessment']}",
            f"- Injected outcome assessment: {metrics['injected_outcome_assessment']}",
            "",
            "## Metrics",
            "",
            f"- Adaptation speed (ms): {metrics['adaptation_speed_ms']}",
            f"- Obstacle detection latency (ms): {metrics['obstacle_detection_latency_ms']}",
            f"- Detection signal source: {metrics['detection_signal_source']}",
            f"- Raw scan detection latency (ms): {metrics['raw_scan_detection_latency_ms']}",
            f"- Recovery rate (ms): {metrics['recovery_rate_ms']}",
            f"- False obstacle rate: {metrics['false_obstacle_rate']}",
            f"- Route deviation (m): {metrics['route_deviation_m']}",
            f"- Post-injection route deviation (m): {metrics['post_injection_route_deviation_m']}",
            f"- Evaluation window after encounter (s): {metrics['evaluation_window_after_encounter_s']}",
            f"- Meaningful evaluation satisfied: {metrics['meaningful_evaluation_rule_satisfied']}",
            f"- Observed control rationale: {metrics['observed_control_rationale']}",
            f"- Reaction scope: {metrics['reaction_scope']}",
            f"- Reaction attribution status: {metrics['reaction_attribution_status']}",
            f"- Detection attribution status: {metrics['detection_attribution_status']}",
            f"- Baseline uncertainty exercised: {metrics['baseline_uncertainty_exercised_status']}",
            "",
            "## Safety Preservation",
            "",
            f"- MR_009 preserved: {metrics['safety_preservation']['MR_009']}",
            f"- MR_011 preserved: {metrics['safety_preservation']['MR_011']}",
            f"- Collision proxy preserved: {metrics['safety_preservation']['collision_with_obstacle']}",
            "",
            "## Goal Viability",
            "",
            f"- science_rock_01 reached: {metrics['goal_viability']['science_rock_01_reached']}",
            f"- Mission deadline met: {metrics['goal_viability']['mission_deadline_met']}",
            "",
            "## Traceability",
            "",
            f"- Injection entity: {metrics['injection_entity_name']}",
            f"- Injection pose: {metrics['injection_pose']}",
            f"- Active injected uncertainty sources: {metrics['injected_uncertainty_source']}",
            f"- Rosbag output dir: {metrics['rosbag_output_dir']}",
            "",
            "## Notes",
            "",
        ]
        for note in self.report_notes:
            lines.append(f"- {note}")
        if metrics["monitor_events"]:
            lines.append(f"- Monitor events observed: {metrics['monitor_events']}")
        if metrics["injection_failure_reason"]:
            lines.append(f"- Injection failure reason: {metrics['injection_failure_reason']}")
        return "\n".join(lines) + "\n"

    def _finalize(self, reason: str) -> None:
        if self.finalized:
            return
        self.finalized = True
        self.termination_reason = reason
        metrics = self._compute_metrics(reason)
        if metrics["injected_uncertainty_encounter_status"]:
            if metrics["meaningful_evaluation_rule_satisfied"]:
                self._log_event("meaningful_evaluation_window_satisfied")
            else:
                self._log_event("meaningful_evaluation_window_not_satisfied")
        self.metrics_path.write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        self.report_path.write_text(self._build_report(metrics), encoding="utf-8")
        self._log_event(
            "scenario_finalized",
            metrics_path=self.metrics_path,
            report_path=self.report_path,
            termination_reason=reason,
        )
        self.timeline_handle.flush()
        self.timeline_handle.close()
        self.shutdown_requested = True
        try:
            rclpy.shutdown()
        except Exception:
            pass


def main() -> None:
    rclpy.init()
    node = ScenarioDriver()

    def handle_signal(signum: int, _frame: Any) -> None:
        signal_name = signal.Signals(signum).name.lower()
        if not node.finalized:
            node._finalize(f"signal_{signal_name}")

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if not node.finalized:
            node._finalize("interrupted")
    finally:
        if not node.finalized:
            node._finalize(node.termination_reason or "shutdown")
        node.destroy_node()


if __name__ == "__main__":
    main()
