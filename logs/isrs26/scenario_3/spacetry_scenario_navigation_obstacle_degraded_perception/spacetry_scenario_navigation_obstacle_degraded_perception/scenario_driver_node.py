#!/usr/bin/env python3
import json
import math
import os
import re
import signal
import subprocess
from collections import deque
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Tuple

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String


FRONT_MIN_RAD = math.radians(-20.0)
FRONT_MAX_RAD = math.radians(20.0)


def load_yaml(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def sanitize_label(value: str) -> str:
    text = re.sub(r"[^A-Za-z0-9_.-]+", "_", value.strip())
    return text or "run"


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def distance_xy(ax: float, ay: float, bx: float, by: float) -> float:
    return math.hypot(ax - bx, ay - by)


def sector_min(scan: LaserScan, min_angle: float, max_angle: float) -> float:
    minimum = math.inf
    for index, raw_range in enumerate(scan.ranges):
        if not math.isfinite(raw_range) or raw_range <= 0.0:
            continue
        if raw_range < scan.range_min or raw_range > scan.range_max:
            continue
        angle = scan.angle_min + index * scan.angle_increment
        if min_angle < angle < max_angle:
            minimum = min(minimum, float(raw_range))
    return minimum


def project_point_to_route(
    point_x: float,
    point_y: float,
    start_x: float,
    start_y: float,
    goal_x: float,
    goal_y: float,
) -> Tuple[float, float]:
    route_dx = goal_x - start_x
    route_dy = goal_y - start_y
    route_length_sq = route_dx * route_dx + route_dy * route_dy
    if route_length_sq <= 1e-9:
        return 0.0, 0.0

    rel_x = point_x - start_x
    rel_y = point_y - start_y
    progress = clamp((rel_x * route_dx + rel_y * route_dy) / route_length_sq, 0.0, 1.0)
    proj_x = start_x + progress * route_dx
    proj_y = start_y + progress * route_dy
    deviation = distance_xy(point_x, point_y, proj_x, proj_y)
    return progress, deviation


def json_ready(value: Any) -> Any:
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): json_ready(item) for key, item in value.items()}
    if isinstance(value, list):
        return [json_ready(item) for item in value]
    if isinstance(value, float):
        if math.isnan(value) or math.isinf(value):
            return None
        return value
    return value


class ScenarioDriver(Node):
    def __init__(self) -> None:
        super().__init__("scenario_navigation_obstacle_degraded_perception_driver")

        self.declare_parameter("scenario_contract_file", "")
        self.declare_parameter("scenario_config_file", "")
        self.declare_parameter("output_root", "/ws/log/spacetry_scenario_navigation_obstacle_degraded_perception")
        self.declare_parameter("run_label", "manual_run")
        self.declare_parameter("record_rosbag", True)

        self.contract_path = Path(str(self.get_parameter("scenario_contract_file").value))
        self.config_path = Path(str(self.get_parameter("scenario_config_file").value))
        self.output_root = Path(str(self.get_parameter("output_root").value))
        self.run_label = sanitize_label(str(self.get_parameter("run_label").value))
        self.record_rosbag = bool(self.get_parameter("record_rosbag").value)

        self.contract = load_yaml(self.contract_path)
        self.config = load_yaml(self.config_path)
        self.scenario_name = str(self.config["scenario_name"])

        self.run_dir = self.output_root / self.run_label
        self.metrics_dir = self.run_dir / "metrics"
        self.runtime_dir = self.run_dir / "runtime"
        self.rosbag_dir = self.run_dir / "rosbags"
        for directory in (self.run_dir, self.metrics_dir, self.runtime_dir, self.rosbag_dir):
            directory.mkdir(parents=True, exist_ok=True)

        self.report_path = self.run_dir / f"{self.scenario_name}_report.md"
        self.metrics_path = self.metrics_dir / f"{self.scenario_name}_metrics.json"
        self.metadata_path = self.runtime_dir / "scenario_metadata.json"
        self.timeline_path = self.runtime_dir / f"{self.scenario_name}_timeline.jsonl"
        self.timeline_handle = self.timeline_path.open("w", encoding="utf-8", buffering=1)

        self.runtime_cfg = self.config["runtime"]
        self.metric_cfg = self.config["metrics"]
        self.degradation_cfg = self.config["degradation"]
        self.obstacle_cfg = self.config["injected_obstacle"]
        self.topics = self.config["topics"]
        self.goal_cfg = self.config["goal"]
        self.start_cfg = self.config["start"]
        self.baseline_hazard_cfg = self.config["baseline_hazard"]

        self.best_effort_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.pub_front = self.create_publisher(Bool, self.topics["obstacle_front"], 10)
        self.pub_left = self.create_publisher(Bool, self.topics["obstacle_left"], 10)
        self.pub_right = self.create_publisher(Bool, self.topics["obstacle_right"], 10)
        self.pub_state = self.create_publisher(String, self.topics["obstacle_state"], 10)

        self.create_subscription(
            Odometry, self.topics["odom"], self.on_odom, self.best_effort_qos
        )
        self.create_subscription(
            LaserScan, self.topics["scan"], self.on_scan, qos_profile_sensor_data
        )
        self.create_subscription(Twist, self.topics["cmd_vel"], self.on_cmd_vel, 10)
        self.create_subscription(Float32, self.topics["battery_soc"], self.on_battery_soc, 10)
        self.create_subscription(
            Bool, self.topics["battery_near_outpost"], self.on_battery_near_outpost, 10
        )
        self.create_subscription(
            Bool, self.topics["obstacle_front"], self.on_obstacle_front, self.best_effort_qos
        )
        self.create_subscription(
            Bool, self.topics["obstacle_left"], self.on_obstacle_left, self.best_effort_qos
        )
        self.create_subscription(
            Bool, self.topics["obstacle_right"], self.on_obstacle_right, self.best_effort_qos
        )
        self.create_subscription(
            String, self.topics["obstacle_state"], self.on_obstacle_state, self.best_effort_qos
        )
        self.create_subscription(
            Bool, self.topics["handler_mr_009"], self.on_handler_mr_009, 10
        )
        self.create_subscription(
            Bool, self.topics["handler_mr_011"], self.on_handler_mr_011, 10
        )

        self.timer = self.create_timer(0.1, self.on_timer)

        self.rosbag_process: Optional[subprocess.Popen] = None
        self.rosbag_log_handle = None

        self.run_started = False
        self.finalized = False
        self.termination_reason = "running"

        self.run_start_s: Optional[float] = None
        self.goal_reached_time_s: Optional[float] = None
        self.latest_pose: Optional[Dict[str, float]] = None
        self.latest_scan_front_min = math.inf
        self.latest_cmd_vel = {"linear_x": 0.0, "angular_z": 0.0, "time_s": None}
        self.latest_soc = 1.0
        self.latest_near_outpost = False
        self.latest_obstacle = {
            "front": False,
            "left": False,
            "right": False,
            "state": "CLEAR",
        }
        self.latest_obstacle_time = {
            "front": None,
            "left": None,
            "right": None,
            "state": None,
        }

        self.monitor_status = {"MR_009": True, "MR_011": True}
        self.monitor_events: List[Dict[str, Any]] = []

        self.path_samples: List[Dict[str, float]] = []
        self.progress_history: Deque[Tuple[float, float, float]] = deque()
        self.max_route_deviation_m = 0.0
        self.max_post_injection_route_deviation_m = 0.0

        self.injection_gate_passed = False
        self.injection_time_s: Optional[float] = None
        self.injection_status = "pending"
        self.spawn_command: List[str] = []
        self.spawn_stdout = ""
        self.spawn_stderr = ""
        self.spawn_verified = False
        self.injected_entity_name = (
            f"{self.obstacle_cfg['entity_name_prefix']}_{self.run_label}"
        )
        self.degradation_mode = "inactive"
        self.degradation_end_time_s: Optional[float] = None
        self.next_degradation_publish_s: Optional[float] = None
        self.last_degradation_side = "left"
        self.total_degradation_publications = 0
        self.false_obstacle_publications = 0

        self.encounter_time_s: Optional[float] = None
        self.raw_scan_detection_time_s: Optional[float] = None
        self.detection_time_s: Optional[float] = None
        self.detection_signal_source: Optional[str] = None
        self.detection_attribution_status = False
        self.minimum_fault_distance_at_detection_m: Optional[float] = None
        self.detection_rejection_counts: Dict[str, int] = {}

        self.reaction_time_s: Optional[float] = None
        self.reaction_scope = "indeterminate"
        self.reaction_attribution_status = False
        self.observed_control_rationale = "unknown"
        self.active_context_at_reaction: Dict[str, Any] = {}
        self.injected_uncertainty_source: List[str] = []
        self.baseline_reaction_observed = False
        self.last_logged_rationale = "unknown"

        self.recovery_time_s: Optional[float] = None
        self.resumed_progress_candidate_s: Optional[float] = None

        self.min_distance_to_injected_obstacle_m = math.inf
        self.min_distance_to_baseline_hazard_m = math.inf

        self.write_metadata()
        self.record_event(
            "scenario_initialized",
            scenario_name=self.scenario_name,
            run_label=self.run_label,
            contract_file=str(self.contract_path),
            config_file=str(self.config_path),
            output_root=str(self.output_root),
            report_path=str(self.report_path),
        )

        if self.record_rosbag:
            self.start_rosbag_recording()

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def write_metadata(self) -> None:
        metadata = {
            "scenario_name": self.scenario_name,
            "run_label": self.run_label,
            "contract_file": str(self.contract_path),
            "config_file": str(self.config_path),
            "output_root": str(self.output_root),
            "report_path": str(self.report_path),
            "metrics_path": str(self.metrics_path),
            "timeline_path": str(self.timeline_path),
            "termination_reason": self.termination_reason,
            "injection_status": self.injection_status,
        }
        self.metadata_path.write_text(
            json.dumps(json_ready(metadata), indent=2),
            encoding="utf-8",
        )

    def record_event(self, event_name: str, **fields: Any) -> None:
        event = {
            "event": event_name,
            "sim_time_s": round(self.now_s(), 3),
            "elapsed_s": None
            if self.run_start_s is None
            else round(self.now_s() - self.run_start_s, 3),
            "fields": json_ready(fields),
        }
        self.timeline_handle.write(json.dumps(event) + "\n")

    def start_rosbag_recording(self) -> None:
        bag_output = self.rosbag_dir / f"{self.scenario_name}_bag"
        rosbag_log = self.runtime_dir / "rosbag_record.log"
        self.rosbag_log_handle = rosbag_log.open("w", encoding="utf-8")
        cmd = [
            "ros2",
            "bag",
            "record",
            "-o",
            str(bag_output),
            *self.topics["rosbag_topics"],
        ]
        self.rosbag_process = subprocess.Popen(
            cmd,
            stdout=self.rosbag_log_handle,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        self.record_event(
            "rosbag_started",
            command=cmd,
            output=str(bag_output),
        )

    def stop_rosbag_recording(self) -> None:
        if self.rosbag_process is None:
            return

        if self.rosbag_process.poll() is None:
            os.killpg(self.rosbag_process.pid, signal.SIGINT)
            try:
                self.rosbag_process.wait(timeout=20.0)
            except subprocess.TimeoutExpired:
                os.killpg(self.rosbag_process.pid, signal.SIGTERM)
                self.rosbag_process.wait(timeout=10.0)

        self.record_event(
            "rosbag_stopped",
            returncode=self.rosbag_process.returncode,
        )
        if self.rosbag_log_handle is not None:
            self.rosbag_log_handle.close()
            self.rosbag_log_handle = None

    def on_battery_soc(self, msg: Float32) -> None:
        self.latest_soc = float(msg.data)

    def on_battery_near_outpost(self, msg: Bool) -> None:
        self.latest_near_outpost = bool(msg.data)

    def on_handler_mr_009(self, msg: Bool) -> None:
        if not msg.data:
            return
        self.monitor_status["MR_009"] = False
        event = {"monitor": "MR_009", "time_s": self.now_s()}
        self.monitor_events.append(event)
        self.record_event("baseline_monitor_violation_active", **event)

    def on_handler_mr_011(self, msg: Bool) -> None:
        if not msg.data:
            return
        self.monitor_status["MR_011"] = False
        event = {"monitor": "MR_011", "time_s": self.now_s()}
        self.monitor_events.append(event)
        self.record_event("baseline_monitor_violation_active", **event)

    def on_obstacle_front(self, msg: Bool) -> None:
        now = self.now_s()
        self.latest_obstacle["front"] = bool(msg.data)
        self.latest_obstacle_time["front"] = now
        if msg.data:
            self.evaluate_detection_candidate(self.topics["obstacle_front"], "front", now)

    def on_obstacle_left(self, msg: Bool) -> None:
        now = self.now_s()
        self.latest_obstacle["left"] = bool(msg.data)
        self.latest_obstacle_time["left"] = now
        if msg.data:
            self.evaluate_detection_candidate(self.topics["obstacle_left"], "left", now)

    def on_obstacle_right(self, msg: Bool) -> None:
        now = self.now_s()
        self.latest_obstacle["right"] = bool(msg.data)
        self.latest_obstacle_time["right"] = now
        if msg.data:
            self.evaluate_detection_candidate(self.topics["obstacle_right"], "right", now)

    def on_obstacle_state(self, msg: String) -> None:
        now = self.now_s()
        self.latest_obstacle["state"] = str(msg.data)
        self.latest_obstacle_time["state"] = now
        if msg.data != "CLEAR":
            self.evaluate_detection_candidate(self.topics["obstacle_state"], str(msg.data), now)

    def on_scan(self, msg: LaserScan) -> None:
        self.latest_scan_front_min = sector_min(msg, FRONT_MIN_RAD, FRONT_MAX_RAD)
        if (
            self.raw_scan_detection_time_s is None
            and self.injection_time_s is not None
            and self.spawn_verified
            and self.latest_scan_front_min < float(self.metric_cfg["obstacle_threshold_m"])
            and self.passes_detection_attribution_rule()
        ):
            self.raw_scan_detection_time_s = self.now_s()
            self.record_event(
                "raw_scan_detection_attributed",
                source=self.topics["scan"],
                front_min_m=self.latest_scan_front_min,
                distance_to_fault_m=self.current_distance_to_injected_obstacle(),
            )

    def on_cmd_vel(self, msg: Twist) -> None:
        now = self.now_s()
        linear_x = float(msg.linear.x)
        angular_z = float(msg.angular.z)

        rationale = self.classify_control_rationale(linear_x, angular_z)
        previous_linear = float(self.latest_cmd_vel["linear_x"])
        previous_angular = float(self.latest_cmd_vel["angular_z"])
        rationale_changed = rationale != self.last_logged_rationale
        changed = (
            abs(linear_x - previous_linear) > 0.15
            or abs(angular_z - previous_angular) > 0.15
            or rationale_changed
        )

        self.latest_cmd_vel = {
            "linear_x": linear_x,
            "angular_z": angular_z,
            "time_s": now,
        }

        if changed:
            context = self.active_context_snapshot()
            self.record_event(
                "cmd_vel_changed",
                linear_x=linear_x,
                angular_z=angular_z,
                observed_control_rationale=rationale,
                active_context=context,
            )
            self.record_event(
                "control_rationale_classified",
                observed_control_rationale=rationale,
                reaction_scope=self.classify_reaction_scope(),
            )
            self.last_logged_rationale = rationale

        if (
            not self.baseline_reaction_observed
            and self.injection_time_s is None
            and rationale == "obstacle_avoidance"
            and self.current_distance_to_baseline_hazard() <= float(self.baseline_hazard_cfg["confound_radius_m"])
        ):
            self.baseline_reaction_observed = True
            self.record_event(
                "baseline_reaction_observed",
                observed_control_rationale=rationale,
                distance_to_block_island_m=self.current_distance_to_baseline_hazard(),
            )

        if (
            self.reaction_time_s is None
            and self.injection_time_s is not None
            and rationale == "obstacle_avoidance"
            and self.passes_reaction_attribution_rule()
        ):
            self.reaction_time_s = now
            self.observed_control_rationale = rationale
            self.reaction_scope = self.classify_reaction_scope()
            self.reaction_attribution_status = True
            self.active_context_at_reaction = self.active_context_snapshot()
            self.injected_uncertainty_source = self.classify_injected_sources()
            self.record_event(
                "reaction_attributed",
                observed_control_rationale=self.observed_control_rationale,
                reaction_scope=self.reaction_scope,
                reaction_attribution_status=True,
                active_context_at_reaction=self.active_context_at_reaction,
                injected_uncertainty_source=self.injected_uncertainty_source,
            )

    def on_odom(self, msg: Odometry) -> None:
        now = self.now_s()
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        self.latest_pose = {"x": x, "y": y}

        if not self.run_started:
            self.run_started = True
            self.run_start_s = now
            self.record_event(
                "scenario_started",
                start_pose=self.latest_pose,
                goal_id=self.goal_cfg["id"],
            )

        progress_ratio, route_deviation = project_point_to_route(
            x,
            y,
            float(self.start_cfg["x"]),
            float(self.start_cfg["y"]),
            float(self.goal_cfg["x"]),
            float(self.goal_cfg["y"]),
        )
        distance_to_goal = distance_xy(
            x,
            y,
            float(self.goal_cfg["x"]),
            float(self.goal_cfg["y"]),
        )
        distance_to_injected = self.current_distance_to_injected_obstacle()
        if math.isfinite(distance_to_injected):
            self.min_distance_to_injected_obstacle_m = min(
                self.min_distance_to_injected_obstacle_m,
                distance_to_injected,
            )

        distance_to_baseline = self.current_distance_to_baseline_hazard()
        self.min_distance_to_baseline_hazard_m = min(
            self.min_distance_to_baseline_hazard_m,
            distance_to_baseline,
        )

        self.max_route_deviation_m = max(self.max_route_deviation_m, route_deviation)
        if self.injection_time_s is not None:
            self.max_post_injection_route_deviation_m = max(
                self.max_post_injection_route_deviation_m,
                route_deviation,
            )

        self.path_samples.append(
            {
                "time_s": now,
                "x": x,
                "y": y,
                "progress_ratio": progress_ratio,
                "route_deviation_m": route_deviation,
                "distance_to_goal_m": distance_to_goal,
            }
        )
        self.progress_history.append((now, progress_ratio, distance_to_goal))
        self.trim_progress_history(now)

        if (
            self.encounter_time_s is None
            and self.spawn_verified
            and distance_to_injected <= float(self.obstacle_cfg["encounter_distance_m"])
        ):
            self.encounter_time_s = now
            self.record_event(
                "fault_encountered",
                distance_to_fault_m=distance_to_injected,
                progress_ratio=progress_ratio,
            )

        if (
            self.goal_reached_time_s is None
            and distance_to_goal <= float(self.goal_cfg["tolerance_m"])
        ):
            self.goal_reached_time_s = now
            self.record_event(
                "goal_reached",
                goal_id=self.goal_cfg["id"],
                distance_to_goal_m=distance_to_goal,
                progress_ratio=progress_ratio,
            )
            self.request_shutdown("goal_reached")

        if self.reaction_time_s is not None and self.recovery_time_s is None:
            if self.has_resumed_progress(now, distance_to_goal):
                if self.resumed_progress_candidate_s is None:
                    self.resumed_progress_candidate_s = now
                elif (
                    now - self.resumed_progress_candidate_s
                    >= float(self.metric_cfg["resumed_progress_window_s"])
                ):
                    self.recovery_time_s = self.resumed_progress_candidate_s
                    self.record_event(
                        "meaningful_evaluation_window_satisfied",
                        recovery_time_s=self.recovery_time_s,
                    )
            else:
                self.resumed_progress_candidate_s = None

    def trim_progress_history(self, now_s: float) -> None:
        window_s = float(self.runtime_cfg["progress_window_s"])
        while self.progress_history and (now_s - self.progress_history[0][0]) > window_s:
            self.progress_history.popleft()

    def current_distance_to_injected_obstacle(self) -> float:
        if self.latest_pose is None:
            return math.inf
        return distance_xy(
            self.latest_pose["x"],
            self.latest_pose["y"],
            float(self.obstacle_cfg["x"]),
            float(self.obstacle_cfg["y"]),
        )

    def current_distance_to_baseline_hazard(self) -> float:
        if self.latest_pose is None:
            return math.inf
        return distance_xy(
            self.latest_pose["x"],
            self.latest_pose["y"],
            float(self.baseline_hazard_cfg["x"]),
            float(self.baseline_hazard_cfg["y"]),
        )

    def recent_progress_distance(self) -> float:
        if len(self.progress_history) < 2:
            return 0.0
        return self.progress_history[0][2] - self.progress_history[-1][2]

    def classify_control_rationale(self, linear_x: float, angular_z: float) -> str:
        if not all(self.monitor_status.values()):
            return "monitor_enforcement"

        distance_to_goal = math.inf
        if self.latest_pose is not None:
            distance_to_goal = distance_xy(
                self.latest_pose["x"],
                self.latest_pose["y"],
                float(self.goal_cfg["x"]),
                float(self.goal_cfg["y"]),
            )

        obstacle_context = (
            self.latest_obstacle["front"]
            or self.latest_obstacle["left"]
            or self.latest_obstacle["right"]
            or self.latest_obstacle["state"] != "CLEAR"
            or self.degradation_mode != "inactive"
        )
        if (
            abs(angular_z) >= float(self.metric_cfg["reaction_angular_threshold_radps"])
            or linear_x <= float(self.metric_cfg["slow_linear_threshold_mps"])
        ):
            if obstacle_context:
                return "obstacle_avoidance"
            if distance_to_goal <= 6.0 and abs(linear_x) <= 0.1 and abs(angular_z) > 0.2:
                return "goal_alignment"

        if distance_to_goal <= 6.0 and abs(linear_x) <= 0.1 and abs(angular_z) > 0.2:
            return "goal_alignment"
        return "unknown"

    def classify_reaction_scope(self) -> str:
        baseline_active = (
            self.current_distance_to_baseline_hazard()
            <= float(self.baseline_hazard_cfg["confound_radius_m"])
        )
        injected_active = self.injection_time_s is not None and (
            self.current_distance_to_injected_obstacle()
            <= float(self.obstacle_cfg["attribution_distance_m"])
            or self.degradation_mode != "inactive"
        )

        if baseline_active and injected_active:
            return "baseline_and_injected"
        if injected_active:
            return "injected_only"
        if baseline_active:
            return "baseline_only"
        return "indeterminate"

    def classify_injected_sources(self) -> List[str]:
        sources: List[str] = []
        if (
            self.spawn_verified
            and self.current_distance_to_injected_obstacle()
            <= float(self.obstacle_cfg["attribution_distance_m"])
        ):
            sources.append("runtime_route_blocking_rock")
        if self.degradation_mode != "inactive":
            sources.append("degraded_obstacle_interpretation")
        return sources

    def passes_detection_attribution_rule(self) -> bool:
        if self.injection_time_s is None or not self.spawn_verified:
            return False
        if (
            self.current_distance_to_injected_obstacle()
            > float(self.obstacle_cfg["attribution_distance_m"])
        ):
            return False
        if (
            self.current_distance_to_baseline_hazard()
            <= float(self.baseline_hazard_cfg["confound_radius_m"])
        ):
            return False
        return True

    def passes_reaction_attribution_rule(self) -> bool:
        if self.injection_time_s is None:
            return False
        if (
            self.current_distance_to_baseline_hazard()
            <= float(self.baseline_hazard_cfg["confound_radius_m"])
        ):
            return False
        if self.degradation_mode != "inactive":
            return True
        if not self.spawn_verified:
            return False
        return (
            self.current_distance_to_injected_obstacle()
            <= float(self.obstacle_cfg["attribution_distance_m"])
        )

    def evaluate_detection_candidate(
        self, source: str, signal_value: str, timestamp_s: float
    ) -> None:
        self.record_event(
            "fault_detection_candidate",
            source=source,
            signal_value=signal_value,
            distance_to_fault_m=self.current_distance_to_injected_obstacle(),
            distance_to_block_island_m=self.current_distance_to_baseline_hazard(),
        )

        if self.detection_time_s is not None:
            return

        if not self.passes_detection_attribution_rule():
            reason = "outside_injected_attribution_window"
            self.detection_rejection_counts[reason] = (
                self.detection_rejection_counts.get(reason, 0) + 1
            )
            self.record_event(
                "fault_detection_rejected",
                source=source,
                rejection_reason=reason,
                signal_value=signal_value,
            )
            return

        self.detection_time_s = timestamp_s
        self.detection_signal_source = source
        self.detection_attribution_status = True
        self.minimum_fault_distance_at_detection_m = self.current_distance_to_injected_obstacle()
        self.record_event(
            "fault_detection_attributed",
            source=source,
            signal_value=signal_value,
            distance_to_fault_m=self.minimum_fault_distance_at_detection_m,
            injected_uncertainty_source=self.classify_injected_sources(),
        )

    def active_context_snapshot(self) -> Dict[str, Any]:
        context = {
            "battery_soc": round(self.latest_soc, 4),
            "battery_near_outpost": self.latest_near_outpost,
            "distance_to_goal_m": None,
            "distance_to_injected_fault_m": self.current_distance_to_injected_obstacle(),
            "distance_to_block_island_m": self.current_distance_to_baseline_hazard(),
            "degradation_mode": self.degradation_mode,
            "latest_obstacle": dict(self.latest_obstacle),
            "monitor_status": dict(self.monitor_status),
        }
        if self.latest_pose is not None:
            distance_to_goal = distance_xy(
                self.latest_pose["x"],
                self.latest_pose["y"],
                float(self.goal_cfg["x"]),
                float(self.goal_cfg["y"]),
            )
            progress_ratio, _ = project_point_to_route(
                self.latest_pose["x"],
                self.latest_pose["y"],
                float(self.start_cfg["x"]),
                float(self.start_cfg["y"]),
                float(self.goal_cfg["x"]),
                float(self.goal_cfg["y"]),
            )
            context["pose"] = dict(self.latest_pose)
            context["distance_to_goal_m"] = distance_to_goal
            context["progress_ratio"] = progress_ratio
        return json_ready(context)

    def has_resumed_progress(self, now_s: float, current_distance_to_goal: float) -> bool:
        if self.reaction_time_s is None or len(self.progress_history) < 2:
            return False
        recent_distance_reduction = self.recent_progress_distance()
        front_clear = not self.latest_obstacle["front"]
        far_from_fault = (
            self.current_distance_to_injected_obstacle()
            >= float(self.obstacle_cfg["clear_distance_m"])
        )
        return (
            self.latest_cmd_vel["linear_x"] >= float(self.metric_cfg["resumed_linear_threshold_mps"])
            and recent_distance_reduction >= float(self.runtime_cfg["min_progress_distance_m_over_window"])
            and (front_clear or far_from_fault)
        )

    def on_timer(self) -> None:
        if self.finalized or self.run_start_s is None:
            return

        now = self.now_s()
        elapsed = now - self.run_start_s

        if self.record_rosbag and self.rosbag_process is not None and self.rosbag_process.poll() is not None:
            self.record_event(
                "rosbag_recording_failed",
                returncode=self.rosbag_process.returncode,
            )
            self.record_rosbag = False

        if not self.injection_gate_passed and self.should_inject_fault(now):
            self.injection_gate_passed = True
            self.record_event(
                "fault_injection_gate_passed",
                elapsed_s=elapsed,
                progress_ratio=self.path_samples[-1]["progress_ratio"],
                distance_to_goal_m=self.path_samples[-1]["distance_to_goal_m"],
                distance_to_fault_pose_m=self.current_distance_to_injected_obstacle(),
                baseline_monitor_active=not all(self.monitor_status.values()),
                remaining_mission_time_s=float(self.runtime_cfg["timeout_s"]) - elapsed,
            )
            self.inject_fault(now)

        if self.injection_time_s is not None and self.degradation_mode != "inactive":
            self.publish_degradation_state(now)

        if elapsed >= float(self.runtime_cfg["timeout_s"]):
            self.request_shutdown("timeout")

    def should_inject_fault(self, now_s: float) -> bool:
        if self.latest_pose is None or not self.path_samples:
            return False

        elapsed = now_s - self.run_start_s
        latest = self.path_samples[-1]
        return (
            elapsed >= float(self.runtime_cfg["start_guard_s"])
            and latest["progress_ratio"] >= float(self.runtime_cfg["min_progress_ratio_before_injection"])
            and latest["progress_ratio"] <= float(self.runtime_cfg["max_progress_ratio_for_injection"])
            and self.current_distance_to_injected_obstacle()
            <= float(self.runtime_cfg["max_distance_to_injection_pose_m"])
            and latest["distance_to_goal_m"] >= float(self.runtime_cfg["min_distance_to_goal_after_injection_m"])
            and self.current_distance_to_baseline_hazard()
            >= float(self.runtime_cfg["min_distance_from_baseline_hazard_m"])
            and self.recent_progress_distance() >= float(self.runtime_cfg["min_progress_distance_m_over_window"])
        )

    def gazebo_env(self) -> Dict[str, str]:
        env = dict(os.environ)
        models_root = Path(get_package_share_directory("spacetry_models")) / "models"
        extra_paths = [str(models_root)]
        for key in ("GZ_SIM_RESOURCE_PATH", "SDF_PATH"):
            existing = env.get(key, "")
            parts = [part for part in existing.split(os.pathsep) if part]
            env[key] = os.pathsep.join(parts + extra_paths)
        return env

    def inject_fault(self, now_s: float) -> None:
        model_file = (
            Path(get_package_share_directory("spacetry_models"))
            / "models"
            / str(self.obstacle_cfg["model_relative_path"])
        )
        self.spawn_command = [
            "ros2",
            "run",
            "ros_gz_sim",
            "create",
            "-world",
            str(self.config["world_name"]),
            "-file",
            str(model_file),
            "-name",
            self.injected_entity_name,
            "-x",
            str(self.obstacle_cfg["x"]),
            "-y",
            str(self.obstacle_cfg["y"]),
            "-z",
            str(self.obstacle_cfg["z"]),
            "-R",
            str(self.obstacle_cfg["roll"]),
            "-P",
            str(self.obstacle_cfg["pitch"]),
            "-Y",
            str(self.obstacle_cfg["yaw"]),
        ]

        self.injection_time_s = now_s
        self.injection_status = "spawn_command_started"

        try:
            completed = subprocess.run(
                self.spawn_command,
                env=self.gazebo_env(),
                capture_output=True,
                text=True,
                timeout=60.0,
                check=False,
            )
            self.spawn_stdout = completed.stdout
            self.spawn_stderr = completed.stderr
            returncode = completed.returncode
        except subprocess.TimeoutExpired as exc:
            self.spawn_stdout = exc.stdout or ""
            self.spawn_stderr = (exc.stderr or "") + "\nSpawn command timed out."
            returncode = 124

        self.record_event(
            "fault_injected",
            command=self.spawn_command,
            returncode=returncode,
            stdout=self.spawn_stdout.strip(),
            stderr=self.spawn_stderr.strip(),
            fault_pose={
                "x": self.obstacle_cfg["x"],
                "y": self.obstacle_cfg["y"],
                "z": self.obstacle_cfg["z"],
                "roll": self.obstacle_cfg["roll"],
                "pitch": self.obstacle_cfg["pitch"],
                "yaw": self.obstacle_cfg["yaw"],
            },
            fault_id=self.injected_entity_name,
        )

        self.spawn_verified = returncode == 0 and self.verify_spawn()
        self.injection_status = "verified" if self.spawn_verified else "verification_failed"
        self.degradation_mode = "pre_encounter_noise" if self.spawn_verified else "inactive"
        self.degradation_end_time_s = now_s + float(self.degradation_cfg["max_duration_s"])
        self.next_degradation_publish_s = now_s
        self.write_metadata()

    def verify_spawn(self) -> bool:
        commands = [
            ["gz", "model", "--list", "-w", str(self.config["world_name"])],
            ["gz", "model", "--list"],
        ]
        for command in commands:
            completed = subprocess.run(
                command,
                env=self.gazebo_env(),
                capture_output=True,
                text=True,
                timeout=20.0,
                check=False,
            )
            if self.injected_entity_name in completed.stdout:
                self.record_event(
                    "fault_spawn_verified",
                    command=command,
                    stdout=completed.stdout.strip(),
                )
                return True

        self.record_event(
            "fault_spawn_verification_failed",
            entity_name=self.injected_entity_name,
        )
        return False

    def publish_degradation_state(self, now_s: float) -> None:
        if self.next_degradation_publish_s is None or now_s < self.next_degradation_publish_s:
            return

        if self.encounter_time_s is not None:
            self.degradation_mode = "encounter_noise"

        if self.degradation_end_time_s is not None and now_s >= self.degradation_end_time_s:
            self.publish_obstacle_state(False, False, False, "CLEAR")
            self.degradation_mode = "inactive"
            self.record_event("fault_state_applied", mode="clear", supported=False)
            return

        flip_left = self.last_degradation_side != "left"
        self.last_degradation_side = "left" if flip_left else "right"
        left = flip_left
        right = not flip_left

        if self.degradation_mode == "pre_encounter_noise":
            front = False
            state = "LEFT" if left else "RIGHT"
            supported = False
        else:
            front = True
            state = "FRONT_LEFT" if left else "FRONT_RIGHT"
            supported = True

        self.publish_obstacle_state(front, left, right, state)
        self.total_degradation_publications += 1
        if not supported:
            self.false_obstacle_publications += 1

        self.record_event(
            "fault_state_applied",
            mode=self.degradation_mode,
            front=front,
            left=left,
            right=right,
            state=state,
            supported=supported,
        )

        self.next_degradation_publish_s = now_s + (
            1.0 / float(self.degradation_cfg["publish_hz"])
        )

    def publish_obstacle_state(
        self, front: bool, left: bool, right: bool, state: str
    ) -> None:
        self.pub_front.publish(Bool(data=front))
        self.pub_left.publish(Bool(data=left))
        self.pub_right.publish(Bool(data=right))
        self.pub_state.publish(String(data=state))

    def request_shutdown(self, reason: str) -> None:
        if self.finalized:
            return
        self.termination_reason = reason
        self.record_event("scenario_shutdown_requested", reason=reason)
        self.finalize()
        rclpy.shutdown()

    def outcome_assessments(self) -> Dict[str, str]:
        safety_failed = (
            not all(self.monitor_status.values())
            or self.min_distance_to_injected_obstacle_m
            <= float(self.obstacle_cfg["collision_distance_m"])
        )
        goal_reached = self.goal_reached_time_s is not None
        baseline_exercised = self.baseline_reaction_observed or bool(self.monitor_events)
        encountered = self.encounter_time_s is not None
        evaluation_window = self.evaluation_window_after_encounter_s()
        meaningful = (
            encountered
            and evaluation_window
            >= float(self.runtime_cfg["minimum_post_encounter_observation_window_s"])
        )

        if encountered and meaningful and self.reaction_attribution_status and goal_reached and not safety_failed:
            injected = "PASS"
        elif not encountered:
            injected = "UNTESTED"
        elif encountered and not meaningful:
            injected = "INCONCLUSIVE"
        elif safety_failed:
            injected = "FAIL"
        elif self.reaction_attribution_status:
            injected = "DEGRADED"
        else:
            injected = "FAIL"

        if baseline_exercised and all(self.monitor_status.values()):
            baseline = "PASS"
        elif baseline_exercised:
            baseline = "DEGRADED"
        else:
            baseline = "NOT_EVALUATED"

        if injected == "PASS" and baseline in ("PASS", "NOT_EVALUATED"):
            overall = "PASS"
        elif injected in ("UNTESTED", "INCONCLUSIVE"):
            overall = injected
        elif injected == "DEGRADED" or baseline == "DEGRADED":
            overall = "DEGRADED"
        else:
            overall = "FAIL"

        return {
            "baseline_outcome_assessment": baseline,
            "injected_outcome_assessment": injected,
            "outcome_assessment": overall,
        }

    def evaluation_window_after_encounter_s(self) -> float:
        if self.encounter_time_s is None:
            return 0.0
        end_time = self.goal_reached_time_s if self.goal_reached_time_s is not None else self.now_s()
        return max(0.0, end_time - self.encounter_time_s)

    def finalize(self) -> None:
        if self.finalized:
            return

        if self.termination_reason == "running":
            if self.goal_reached_time_s is not None:
                self.termination_reason = "goal_reached"
            else:
                self.termination_reason = "launch_shutdown"

        self.finalized = True
        if self.detection_time_s is None and self.raw_scan_detection_time_s is not None:
            self.detection_time_s = self.raw_scan_detection_time_s
            self.detection_signal_source = self.topics["scan"]
            self.detection_attribution_status = True
            self.minimum_fault_distance_at_detection_m = self.current_distance_to_injected_obstacle()

        self.stop_rosbag_recording()

        if self.timeline_handle and not self.timeline_handle.closed:
            self.record_event("scenario_finalizing", termination_reason=self.termination_reason)
            self.timeline_handle.close()

        outcomes = self.outcome_assessments()
        evaluation_window = self.evaluation_window_after_encounter_s()
        meaningful = (
            evaluation_window
            >= float(self.runtime_cfg["minimum_post_encounter_observation_window_s"])
            if self.encounter_time_s is not None
            else False
        )
        adaptation_speed_ms = None
        if self.injection_time_s is not None and self.reaction_time_s is not None:
            adaptation_speed_ms = round((self.reaction_time_s - self.injection_time_s) * 1000.0, 1)

        obstacle_detection_latency_ms = None
        if self.injection_time_s is not None and self.detection_time_s is not None:
            obstacle_detection_latency_ms = round(
                (self.detection_time_s - self.injection_time_s) * 1000.0, 1
            )

        raw_scan_detection_latency_ms = None
        if self.injection_time_s is not None and self.raw_scan_detection_time_s is not None:
            raw_scan_detection_latency_ms = round(
                (self.raw_scan_detection_time_s - self.injection_time_s) * 1000.0, 1
            )

        recovery_rate_ms = None
        if self.reaction_time_s is not None and self.recovery_time_s is not None:
            recovery_rate_ms = round((self.recovery_time_s - self.reaction_time_s) * 1000.0, 1)

        false_obstacle_rate_pct = None
        if self.total_degradation_publications > 0:
            false_obstacle_rate_pct = round(
                100.0 * self.false_obstacle_publications / self.total_degradation_publications,
                2,
            )

        baseline_exercised = self.baseline_reaction_observed or bool(self.monitor_events)
        metrics = {
            "scenario_name": self.scenario_name,
            "run_label": self.run_label,
            "termination_reason": self.termination_reason,
            "injection_status": self.injection_status,
            "spawn_verified": self.spawn_verified,
            "adaptation_speed_ms": adaptation_speed_ms,
            "obstacle_detection_latency_ms": obstacle_detection_latency_ms,
            "raw_scan_detection_latency_ms": raw_scan_detection_latency_ms,
            "autonomy_reaction_status": self.reaction_time_s is not None,
            "detection_signal_source": self.detection_signal_source,
            "observed_control_rationale": self.observed_control_rationale,
            "reaction_scope": self.reaction_scope,
            "reaction_attribution_status": self.reaction_attribution_status,
            "active_context_at_reaction": self.active_context_at_reaction,
            "safety_preservation": {
                "MR_009": self.monitor_status["MR_009"],
                "MR_011": self.monitor_status["MR_011"],
                "collision_with_obstacle": (
                    self.min_distance_to_injected_obstacle_m
                    > float(self.obstacle_cfg["collision_distance_m"])
                ),
            },
            "goal_viability": {
                "science_rock_01_reached": self.goal_reached_time_s is not None,
                "mission_deadline_met": (
                    self.goal_reached_time_s is not None
                    and (self.goal_reached_time_s - self.run_start_s)
                    <= float(self.runtime_cfg["timeout_s"])
                ),
            },
            "recovery_rate_ms": recovery_rate_ms,
            "route_deviation_m": round(self.max_route_deviation_m, 3),
            "post_injection_route_deviation_m": round(self.max_post_injection_route_deviation_m, 3),
            "detection_attribution_status": self.detection_attribution_status,
            "minimum_fault_distance_at_detection_m": self.minimum_fault_distance_at_detection_m,
            "baseline_confound_status": (
                "block_island_nearby"
                if self.current_distance_to_baseline_hazard()
                <= float(self.baseline_hazard_cfg["confound_radius_m"])
                else "none"
            ),
            "injected_uncertainty_encounter_status": self.encounter_time_s is not None,
            "evaluation_window_after_encounter_s": round(evaluation_window, 3),
            "meaningful_evaluation_window_satisfied": meaningful,
            "baseline_uncertainty_exercised_status": baseline_exercised,
            "attribution_scope": self.reaction_scope,
            "injected_uncertainty_source": self.injected_uncertainty_source,
            "false_obstacle_rate_count": self.false_obstacle_publications,
            "false_obstacle_rate_pct": false_obstacle_rate_pct,
            "minimum_distance_to_injected_obstacle_m": (
                None
                if math.isinf(self.min_distance_to_injected_obstacle_m)
                else round(self.min_distance_to_injected_obstacle_m, 3)
            ),
            "minimum_distance_to_block_island_m": (
                None
                if math.isinf(self.min_distance_to_baseline_hazard_m)
                else round(self.min_distance_to_baseline_hazard_m, 3)
            ),
            "detection_rejection_counts": self.detection_rejection_counts,
            "monitor_violation_count": len(self.monitor_events),
            "report_path": str(self.report_path),
            "metrics_path": str(self.metrics_path),
            "timeline_path": str(self.timeline_path),
            "rosbag_dir": str(self.rosbag_dir),
            **outcomes,
        }
        self.metrics_path.write_text(
            json.dumps(json_ready(metrics), indent=2),
            encoding="utf-8",
        )

        report_lines = [
            f"# {self.scenario_name} Report",
            "",
            f"- Run label: `{self.run_label}`",
            f"- Termination reason: `{self.termination_reason}`",
            f"- Outcome assessment: `{outcomes['outcome_assessment']}`",
            f"- Baseline outcome assessment: `{outcomes['baseline_outcome_assessment']}`",
            f"- Injected outcome assessment: `{outcomes['injected_outcome_assessment']}`",
            "",
            "## Scenario Summary",
            "",
            "- Primary evaluation target: runtime obstacle blocking on the route to `science_rock_01`.",
            "- Secondary injected uncertainty: degraded obstacle interpretation on the autonomy-facing obstacle topics.",
            "- Baseline autonomy intentionally left untouched: `src/spacetry_bt/trees/base_bt.xml`, `spacetry_bringup`, `spacetry_perception`, `spacetry_battery`, and `spacetry_monitors` remained unchanged and were only observed.",
            "",
            "## Key Metrics",
            "",
            f"- Adaptation speed: `{adaptation_speed_ms}` ms",
            f"- Obstacle detection latency: `{obstacle_detection_latency_ms}` ms via `{self.detection_signal_source}`",
            f"- Raw scan detection latency: `{raw_scan_detection_latency_ms}` ms",
            f"- Recovery rate: `{recovery_rate_ms}` ms",
            f"- Route deviation: `{round(self.max_route_deviation_m, 3)}` m",
            f"- Post-injection route deviation: `{round(self.max_post_injection_route_deviation_m, 3)}` m",
            f"- False obstacle rate: `{self.false_obstacle_publications}` / `{self.total_degradation_publications}` (`{false_obstacle_rate_pct}`%)",
            f"- Evaluation window after encounter: `{round(evaluation_window, 3)}` s",
            "",
            "## Safety And Goal",
            "",
            f"- MR_009 preserved: `{self.monitor_status['MR_009']}`",
            f"- MR_011 preserved: `{self.monitor_status['MR_011']}`",
            f"- Collision with injected obstacle avoided: `{metrics['safety_preservation']['collision_with_obstacle']}`",
            f"- science_rock_01 reached: `{metrics['goal_viability']['science_rock_01_reached']}`",
            f"- Mission deadline met: `{metrics['goal_viability']['mission_deadline_met']}`",
            "",
            "## Attribution",
            "",
            f"- Observed control rationale: `{self.observed_control_rationale}`",
            f"- Reaction scope: `{self.reaction_scope}`",
            f"- Reaction attribution status: `{self.reaction_attribution_status}`",
            f"- Injected uncertainty source: `{self.injected_uncertainty_source}`",
            f"- Baseline uncertainty exercised: `{baseline_exercised}`",
            "",
            "## Runtime Artifacts",
            "",
            f"- Metrics JSON: `{self.metrics_path}`",
            f"- Timeline JSONL: `{self.timeline_path}`",
            f"- Rosbag directory: `{self.rosbag_dir}`",
            "",
            "## Observability Notes",
            "",
            "- Collision status is inferred conservatively from rover-to-fault distance because the baseline stack does not expose a direct contact signal on a ROS topic.",
            "- The primary obstacle detection metric prefers the autonomy-facing obstacle topics, with raw `/scan` timing reported separately.",
        ]
        self.report_path.write_text("\n".join(report_lines) + "\n", encoding="utf-8")

        self.write_metadata()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = ScenarioDriver()

    def handle_signal(signum, _frame) -> None:
        if signum == signal.SIGTERM:
            node.termination_reason = "signal"
        else:
            node.termination_reason = "interrupted"
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.termination_reason = "interrupted"
    finally:
        node.finalize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
