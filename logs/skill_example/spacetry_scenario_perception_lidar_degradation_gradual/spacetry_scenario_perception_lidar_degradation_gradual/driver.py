#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import signal
from collections import deque
from pathlib import Path
from typing import Any, Optional

import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from spacetry_scenario_metrics.io import write_metrics_json
from spacetry_scenario_metrics.report import render_markdown_report
from spacetry_scenario_metrics.schema import ArtifactPaths, ScenarioOutcomes, ScenarioSummaryMetrics
from spacetry_scenario_metrics.session import ScenarioMetricsSession
from std_msgs.msg import Bool, Float32, String


def _get_nested(data: dict[str, Any], path: str, default: Any) -> Any:
    current: Any = data
    for key in path.split("."):
        if isinstance(current, dict):
            if key not in current:
                return default
            current = current[key]
        elif isinstance(current, list):
            try:
                current = current[int(key)]
            except (ValueError, IndexError):
                return default
        else:
            return default
    return current


def _distance(ax: float, ay: float, bx: float, by: float) -> float:
    return math.hypot(ax - bx, ay - by)


def _sector_min(scan: LaserScan, amin: float, amax: float) -> float:
    best = float("inf")
    if not scan.ranges or scan.angle_increment == 0.0:
        return best
    lo = min(amin, amax)
    hi = max(amin, amax)
    for idx, value in enumerate(scan.ranges):
        if not math.isfinite(value) or value <= 0.0:
            continue
        if value < scan.range_min or value > scan.range_max:
            continue
        angle = scan.angle_min + idx * scan.angle_increment
        if lo <= angle <= hi:
            best = min(best, float(value))
    return best


class PerceptionLidarDegradationDriver(Node):
    def __init__(self) -> None:
        super().__init__("perception_lidar_degradation_driver")
        self.declare_parameter("scenario_config_file", "")
        self.declare_parameter("scenario_contract_file", "")
        self.declare_parameter(
            "output_root",
            "/ws/logs/skill_example",
        )
        self.declare_parameter("run_label", "full_run")

        self.config_file = Path(str(self.get_parameter("scenario_config_file").value))
        self.contract_file = Path(str(self.get_parameter("scenario_contract_file").value))
        self.config = self._load_yaml(self.config_file)
        self.contract = self._load_yaml(self.contract_file)

        self.scenario_name = str(self.config.get("scenario_name", self.contract.get("scenario_name")))
        self.run_label = str(self.get_parameter("run_label").value)
        self.output_root = Path(str(self.get_parameter("output_root").value))
        self.wrapper_run_dir = self.output_root / self.run_label
        self.metrics_dir = self.output_root / "metrics"
        self.runtime_dir = self.output_root / "runtime"
        self.rosbags_dir = self.output_root / "rosbags"
        self.wrapper_metrics_dir = self.wrapper_run_dir / "metrics"
        self.report_path = self.output_root / f"{self.scenario_name}_report.md"
        self.metrics_path = self.metrics_dir / f"{self.run_label}_metrics.json"
        self.wrapper_metrics_path = self.wrapper_metrics_dir / f"{self.run_label}_metrics.json"
        self.timeline_path = self.runtime_dir / f"{self.run_label}_timeline.jsonl"
        for path in (
            self.output_root,
            self.metrics_dir,
            self.runtime_dir,
            self.rosbags_dir,
            self.wrapper_metrics_dir,
        ):
            path.mkdir(parents=True, exist_ok=True)

        self.timeout_s = float(self.config.get("scenario_timeout_s", 1200.0))
        self.min_post_encounter_s = float(
            self.config.get("minimum_post_encounter_observation_window_s", 45.0)
        )

        self.goal_x = float(_get_nested(self.config, "mission.goal.x", 50.10))
        self.goal_y = float(_get_nested(self.config, "mission.goal.y", -80.0))
        self.goal_tolerance_m = float(_get_nested(self.config, "mission.goal.tolerance_m", 1.5))
        self.start_x = float(_get_nested(self.config, "mission.start.x", 0.0))
        self.start_y = float(_get_nested(self.config, "mission.start.y", 0.0))
        self.nominal_route_len = max(_distance(self.start_x, self.start_y, self.goal_x, self.goal_y), 1e-6)
        self.block_x = float(_get_nested(self.config, "mission.baseline_hazards.0.x", 20.0))
        self.block_y = float(_get_nested(self.config, "mission.baseline_hazards.0.y", -40.5))
        self.block_radius_m = float(
            _get_nested(self.config, "mission.baseline_hazards.0.approximate_radius_m", 7.5)
        )

        self.inject_duration_s = float(_get_nested(self.config, "injection.duration_s", 75.0))
        self.inject_rate_hz = float(_get_nested(self.config, "injection.publish_rate_hz", 18.0))
        self.pulse_period_s = float(_get_nested(self.config, "injection.pulse_period_s", 3.0))
        self.pulse_initial_s = float(_get_nested(self.config, "injection.true_pulse_s_initial", 0.75))
        self.pulse_final_s = float(_get_nested(self.config, "injection.true_pulse_s_final", 2.2))
        self.clear_messages_remaining = 0
        self.restore_clear_messages = int(_get_nested(self.config, "injection.restore_obstacle_clear_messages", 12))

        self.raw_clear_threshold_m = float(
            _get_nested(self.config, "response_detection.raw_scan_clear_threshold_m", 9.0)
        )
        self.speed_drop_fraction = float(
            _get_nested(self.config, "response_detection.speed_drop_fraction", 0.35)
        )
        self.lateral_cmd_threshold = float(
            _get_nested(self.config, "response_detection.lateral_cmd_threshold_rad_s", 0.45)
        )
        self.reverse_threshold = float(
            _get_nested(self.config, "response_detection.reverse_threshold_m_s", -0.10)
        )
        self.normal_progress_speed = float(
            _get_nested(self.config, "response_detection.normal_progress_speed_m_s", 0.35)
        )
        self.normal_progress_window_s = float(
            _get_nested(self.config, "response_detection.normal_progress_window_s", 6.0)
        )

        self.odom: Optional[Odometry] = None
        self.last_scan: Optional[LaserScan] = None
        self.last_cmd: Optional[Twist] = None
        self.last_obstacle_state = "UNKNOWN"
        self.obstacle_front = False
        self.obstacle_left = False
        self.obstacle_right = False
        self.battery_soc: Optional[float] = None
        self.near_outpost: Optional[bool] = None
        self.mr009_active = False
        self.mr011_active = False
        self.started_sim_s: Optional[float] = None
        self.finished = False
        self.injection_started_s: Optional[float] = None
        self.injection_finished_s: Optional[float] = None
        self.injection_trigger_id: Optional[str] = None
        self.primary_reaction_id: Optional[str] = None
        self.primary_detection_id: Optional[str] = None
        self.recovery_id: Optional[str] = None
        self.max_route_deviation_m = 0.0
        self.forward_motion_since_s: Optional[float] = None
        self.last_progress = 0.0
        self.cmd_history: deque[tuple[float, float, float]] = deque(maxlen=300)
        self.progress_history: deque[tuple[float, float]] = deque(maxlen=500)
        self.timeline: list[dict[str, Any]] = []
        self.false_obstacle_samples = 0
        self.injected_true_samples = 0
        self.missed_detection_samples = 0
        self.raw_hazard_samples = 0
        self.baseline_uncertainty_exercised = False

        self.session = ScenarioMetricsSession(
            scenario_name=self.scenario_name,
            run_label=self.run_label,
            metadata={
                "primary_evaluation_target": self.contract.get("primary_evaluation_target"),
                "scenario_config_file": str(self.config_file),
                "scenario_contract_file": str(self.contract_file),
                "baseline_map_assessment": self.contract.get("baseline_map_assessment"),
            },
        )

        bool_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.pub_front = self.create_publisher(Bool, str(_get_nested(self.config, "topics.obstacle_front", "/obstacle/front")), bool_qos)
        self.pub_left = self.create_publisher(Bool, str(_get_nested(self.config, "topics.obstacle_left", "/obstacle/left")), bool_qos)
        self.pub_right = self.create_publisher(Bool, str(_get_nested(self.config, "topics.obstacle_right", "/obstacle/right")), bool_qos)
        self.create_subscription(Odometry, str(_get_nested(self.config, "topics.odom", "/mobile_base_controller/odom")), self.on_odom, bool_qos)
        self.create_subscription(LaserScan, str(_get_nested(self.config, "topics.scan", "/scan")), self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, str(_get_nested(self.config, "topics.cmd_vel", "/cmd_vel")), self.on_cmd_vel, bool_qos)
        self.create_subscription(Bool, str(_get_nested(self.config, "topics.obstacle_front", "/obstacle/front")), self.on_front, bool_qos)
        self.create_subscription(Bool, str(_get_nested(self.config, "topics.obstacle_left", "/obstacle/left")), self.on_left, bool_qos)
        self.create_subscription(Bool, str(_get_nested(self.config, "topics.obstacle_right", "/obstacle/right")), self.on_right, bool_qos)
        self.create_subscription(String, str(_get_nested(self.config, "topics.obstacle_state", "/obstacle/state")), self.on_obstacle_state, bool_qos)
        self.create_subscription(Float32, str(_get_nested(self.config, "topics.battery_soc", "/battery/soc")), self.on_soc, bool_qos)
        self.create_subscription(Bool, str(_get_nested(self.config, "topics.battery_near_outpost", "/battery/near_outpost")), self.on_near_outpost, bool_qos)
        self.create_subscription(Bool, str(_get_nested(self.config, "topics.monitor_mr009", "/monitor/handlerMR_009_RETURN_TO_OUTPOST_ON_LOW_BATTERY")), self.on_mr009, bool_qos)
        self.create_subscription(Bool, str(_get_nested(self.config, "topics.monitor_mr011", "/monitor/handlerMR_011_MAINTAIN_SPEED_WHEN_FULL_BATTERY")), self.on_mr011, bool_qos)

        self.control_timer = self.create_timer(1.0 / max(self.inject_rate_hz, 1.0), self.on_timer)
        self.get_logger().info(f"{self.scenario_name} ready; output_root={self.output_root}")

    def _load_yaml(self, path: Path) -> dict[str, Any]:
        with path.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
        if not isinstance(data, dict):
            raise ValueError(f"{path} must contain a YAML mapping")
        return data

    def sim_time_s(self) -> float:
        now = self.get_clock().now()
        return now.nanoseconds * 1e-9

    def on_odom(self, msg: Odometry) -> None:
        self.odom = msg

    def on_scan(self, msg: LaserScan) -> None:
        self.last_scan = msg

    def on_cmd_vel(self, msg: Twist) -> None:
        self.last_cmd = msg

    def on_front(self, msg: Bool) -> None:
        self.obstacle_front = bool(msg.data)

    def on_left(self, msg: Bool) -> None:
        self.obstacle_left = bool(msg.data)

    def on_right(self, msg: Bool) -> None:
        self.obstacle_right = bool(msg.data)

    def on_obstacle_state(self, msg: String) -> None:
        self.last_obstacle_state = str(msg.data)

    def on_soc(self, msg: Float32) -> None:
        self.battery_soc = float(msg.data)

    def on_near_outpost(self, msg: Bool) -> None:
        self.near_outpost = bool(msg.data)

    def on_mr009(self, msg: Bool) -> None:
        self.mr009_active = self.mr009_active or bool(msg.data)
        if msg.data:
            self._timeline("baseline_monitor_violation_active", {"monitor": "MR_009"})

    def on_mr011(self, msg: Bool) -> None:
        self.mr011_active = self.mr011_active or bool(msg.data)
        if msg.data:
            self._timeline("baseline_monitor_violation_active", {"monitor": "MR_011"})

    def on_timer(self) -> None:
        if self.finished:
            return
        now_s = self.sim_time_s()
        if now_s <= 0.0:
            return
        if self.started_sim_s is None:
            self.started_sim_s = now_s
            self._timeline("scenario_started", {"run_label": self.run_label})

        self._update_motion_metrics(now_s)
        if self._goal_reached():
            self.finalize("goal_reached")
            return
        if self.started_sim_s is not None and now_s - self.started_sim_s >= self.timeout_s:
            self.finalize("timeout")
            return

        if self.injection_started_s is None and self._trigger_gate_passed(now_s):
            self._start_injection(now_s)

        if self.injection_started_s is not None and self.injection_finished_s is None:
            self._publish_injection(now_s)
            self._evaluate_injection_response(now_s)
        elif self.clear_messages_remaining > 0:
            self._publish_clear()
            self.clear_messages_remaining -= 1
            self._evaluate_recovery(now_s)
        else:
            self._evaluate_recovery(now_s)

    def _update_motion_metrics(self, now_s: float) -> None:
        if self.odom is None:
            return
        x = float(self.odom.pose.pose.position.x)
        y = float(self.odom.pose.pose.position.y)
        progress = self._progress_ratio(x, y)
        self.last_progress = max(self.last_progress, progress)
        self.max_route_deviation_m = max(self.max_route_deviation_m, self._route_deviation(x, y))
        self.progress_history.append((now_s, progress))
        if self.last_cmd is not None:
            lin = float(self.last_cmd.linear.x)
            ang = float(self.last_cmd.angular.z)
            self.cmd_history.append((now_s, lin, ang))
            if lin > self.normal_progress_speed:
                if self.forward_motion_since_s is None:
                    self.forward_motion_since_s = now_s
            else:
                self.forward_motion_since_s = None
        if _distance(x, y, self.block_x, self.block_y) <= self.block_radius_m + 6.0:
            self.baseline_uncertainty_exercised = True

    def _progress_ratio(self, x: float, y: float) -> float:
        vx = self.goal_x - self.start_x
        vy = self.goal_y - self.start_y
        wx = x - self.start_x
        wy = y - self.start_y
        return max(0.0, min(1.0, (wx * vx + wy * vy) / (self.nominal_route_len ** 2)))

    def _route_deviation(self, x: float, y: float) -> float:
        vx = self.goal_x - self.start_x
        vy = self.goal_y - self.start_y
        wx = x - self.start_x
        wy = y - self.start_y
        cross = abs(vx * wy - vy * wx)
        return cross / self.nominal_route_len

    def _goal_reached(self) -> bool:
        if self.odom is None:
            return False
        p = self.odom.pose.pose.position
        return _distance(float(p.x), float(p.y), self.goal_x, self.goal_y) <= self.goal_tolerance_m

    def _trigger_gate_passed(self, now_s: float) -> bool:
        if self.odom is None or self.last_cmd is None or self.started_sim_s is None:
            return False
        elapsed = now_s - self.started_sim_s
        time_remaining = self.timeout_s - elapsed
        min_time = float(_get_nested(self.config, "trigger.min_sim_time_s", 35.0))
        min_progress = float(_get_nested(self.config, "trigger.min_progress_ratio", 0.30))
        min_forward_s = float(_get_nested(self.config, "trigger.require_forward_motion_s", 5.0))
        min_remaining = float(_get_nested(self.config, "trigger.minimum_time_remaining_s", 120.0))
        if self.mr009_active:
            return False
        return (
            elapsed >= min_time
            and self.last_progress >= min_progress
            and self.forward_motion_since_s is not None
            and now_s - self.forward_motion_since_s >= min_forward_s
            and time_remaining >= min_remaining
        )

    def _start_injection(self, now_s: float) -> None:
        self.injection_started_s = now_s
        progress = self.last_progress
        remaining = None if self.started_sim_s is None else self.timeout_s - (now_s - self.started_sim_s)
        self.injection_trigger_id = self.session.note_trigger(
            time_s=now_s,
            kind="uncertainty_injection",
            source="/obstacle/front|left|right",
            description="Runtime intermittent false obstacle classification pulses started.",
            metadata={
                "why_allowed": "progress, forward-motion, and remaining-time gates passed",
                "remaining_mission_time_s": remaining,
                "progress_ratio": progress,
                "mr009_active": self.mr009_active,
                "mr011_active": self.mr011_active,
            },
            evidence_refs=["scenario_driver:trigger_gate"],
        )
        self._timeline(
            "fault_injected",
            {
                "trigger_id": self.injection_trigger_id,
                "progress_ratio": progress,
                "remaining_mission_time_s": remaining,
                "mr009_active": self.mr009_active,
                "mr011_active": self.mr011_active,
            },
        )

    def _publish_injection(self, now_s: float) -> None:
        assert self.injection_started_s is not None
        elapsed = now_s - self.injection_started_s
        if elapsed >= self.inject_duration_s:
            self.injection_finished_s = now_s
            self.clear_messages_remaining = self.restore_clear_messages
            self._timeline("fault_restored", {"injection_duration_s": elapsed})
            self._publish_clear()
            return

        degradation = min(1.0, max(0.0, elapsed / max(self.inject_duration_s, 1e-6)))
        true_pulse_s = self.pulse_initial_s + (self.pulse_final_s - self.pulse_initial_s) * degradation
        phase_s = elapsed % self.pulse_period_s
        pulse_active = phase_s <= true_pulse_s
        if pulse_active:
            self._publish_obstacles(front=True, left=False, right=True)
            self.injected_true_samples += 1
            raw_clear = self._raw_front_clear()
            if raw_clear:
                self.false_obstacle_samples += 1
            if self.primary_detection_id is None and self.injection_trigger_id is not None:
                self.primary_detection_id = self.session.note_detection(
                    time_s=now_s,
                    source="/obstacle/front",
                    attribution_status="supported" if raw_clear else "ambiguous",
                    candidate_sources=["lidar_classification_degradation_001"],
                    linked_trigger_ids=[self.injection_trigger_id],
                    minimum_fault_distance_m=self._fault_distance(),
                    evidence_refs=["scenario_driver:injected_obstacle_pulse"],
                    metadata={
                        "raw_scan_front_clear": raw_clear,
                        "obstacle_state": self.last_obstacle_state,
                    },
                )
                self._timeline(
                    "fault_detection_attributed" if raw_clear else "fault_detection_candidate",
                    {"detection_id": self.primary_detection_id, "raw_scan_front_clear": raw_clear},
                )
        else:
            self._publish_clear()

        if self.last_scan is not None and _sector_min(self.last_scan, math.radians(-20.0), math.radians(20.0)) < self.raw_clear_threshold_m:
            self.raw_hazard_samples += 1
            if not self.obstacle_front:
                self.missed_detection_samples += 1

    def _publish_clear(self) -> None:
        self._publish_obstacles(front=False, left=False, right=False)

    def _publish_obstacles(self, *, front: bool, left: bool, right: bool) -> None:
        self.pub_front.publish(Bool(data=front))
        self.pub_left.publish(Bool(data=left))
        self.pub_right.publish(Bool(data=right))

    def _raw_front_clear(self) -> bool:
        if self.last_scan is None:
            return False
        return _sector_min(self.last_scan, math.radians(-20.0), math.radians(20.0)) >= self.raw_clear_threshold_m

    def _fault_distance(self) -> Optional[float]:
        if self.odom is None:
            return None
        p = self.odom.pose.pose.position
        return _distance(float(p.x), float(p.y), self.block_x, self.block_y)

    def _evaluate_injection_response(self, now_s: float) -> None:
        if self.primary_reaction_id is not None or self.injection_trigger_id is None or self.last_cmd is None:
            return
        if self.injection_started_s is None or now_s - self.injection_started_s < 0.5:
            return
        lin = float(self.last_cmd.linear.x)
        ang = float(self.last_cmd.angular.z)
        raw_clear = self._raw_front_clear()
        near_block = self._fault_distance() is not None and self._fault_distance() <= self.block_radius_m + 6.0
        obstacle_evidence = self.obstacle_front or self.obstacle_left or self.obstacle_right
        if not obstacle_evidence:
            return

        if lin <= self.reverse_threshold or abs(ang) >= self.lateral_cmd_threshold or lin <= 1.6 * self.speed_drop_fraction:
            rationale = "obstacle_avoidance"
            if raw_clear and not near_block and not self.mr009_active:
                scope = "injected_only"
                attribution = "supported"
            elif raw_clear and not self.mr009_active:
                scope = "baseline_and_injected"
                attribution = "ambiguous"
            else:
                scope = "baseline_and_injected"
                attribution = "ambiguous"
            self.primary_reaction_id = self.session.note_reaction(
                time_s=now_s,
                observed_control_rationale=rationale,
                reaction_scope=scope,
                attribution_status=attribution,
                active_context_at_reaction={
                    "cmd_vel_linear_x": lin,
                    "cmd_vel_angular_z": ang,
                    "raw_scan_front_clear": raw_clear,
                    "obstacle_front": self.obstacle_front,
                    "obstacle_left": self.obstacle_left,
                    "obstacle_right": self.obstacle_right,
                    "near_block_island": bool(near_block),
                    "mr009_active": self.mr009_active,
                    "mr011_active": self.mr011_active,
                },
                candidate_sources=["lidar_classification_degradation_001"],
                linked_trigger_ids=[self.injection_trigger_id],
                evidence_refs=["/cmd_vel", "/obstacle/front", "/scan"],
            )
            self.session.note_adaptation(
                trigger_id=self.injection_trigger_id,
                reaction_id=self.primary_reaction_id,
                attribution_scope=scope,
                attribution_confidence=attribution,
                candidate_sources=["lidar_classification_degradation_001"],
            )
            self._timeline(
                "reaction_attributed" if attribution == "supported" else "reaction_candidate",
                {
                    "reaction_id": self.primary_reaction_id,
                    "observed_control_rationale": rationale,
                    "reaction_scope": scope,
                    "reaction_attribution_status": attribution,
                },
            )

    def _evaluate_recovery(self, now_s: float) -> None:
        if self.recovery_id is not None or self.primary_reaction_id is None or self.injection_finished_s is None:
            return
        if now_s - self.injection_finished_s < self.normal_progress_window_s:
            return
        if len(self.progress_history) < 2:
            return
        window_start = now_s - self.normal_progress_window_s
        samples = [item for item in self.progress_history if item[0] >= window_start]
        if len(samples) < 2:
            return
        progress_delta = samples[-1][1] - samples[0][1]
        if progress_delta * self.nominal_route_len >= self.normal_progress_speed * self.normal_progress_window_s:
            self.recovery_id = self.session.note_recovery(
                time_s=now_s,
                linked_reaction_ids=[self.primary_reaction_id],
                evidence_refs=["/mobile_base_controller/odom", "/cmd_vel"],
                metadata={"progress_delta": progress_delta},
            )
            self._timeline("recovery_observed", {"recovery_id": self.recovery_id})

    def _timeline(self, event: str, metadata: dict[str, Any]) -> None:
        record = {"time_s": round(self.sim_time_s(), 3), "event": event, "metadata": metadata}
        self.timeline.append(record)
        self.get_logger().info(f"{event}: {json.dumps(metadata, sort_keys=True)}")

    def finalize(self, reason: str) -> None:
        if self.finished:
            return
        self.finished = True
        now_s = self.sim_time_s()
        self.finished_sim_s = now_s
        self._publish_clear()
        self._timeline("scenario_finalizing", {"termination_reason": reason})

        encountered = self.injection_started_s is not None
        evaluation_window = None
        if self.injection_started_s is not None:
            evaluation_window = max(0.0, now_s - self.injection_started_s)
        meaningful = bool(
            encountered
            and evaluation_window is not None
            and evaluation_window >= self.min_post_encounter_s
        )
        goal_reached = self._goal_reached()
        safety_preservation = {
            "MR_009_RETURN_TO_OUTPOST_ON_LOW_BATTERY": not self.mr009_active,
            "MR_011_MAINTAIN_SPEED_WHEN_FULL_BATTERY": not self.mr011_active,
            "collision_with_obstacle": None,
        }
        goal_viability = {
            "science_rock_01_reached": goal_reached,
            "mission_deadline_met": goal_reached and reason == "goal_reached",
        }
        false_rate = None
        if self.injected_true_samples:
            false_rate = self.false_obstacle_samples / float(self.injected_true_samples)
        missed_rate = None
        if self.raw_hazard_samples:
            missed_rate = self.missed_detection_samples / float(self.raw_hazard_samples)
        recovery_time_ms = None
        if self.injection_finished_s is not None and self.recovery_id is not None:
            recovery_time_ms = round((now_s - self.injection_finished_s) * 1000.0, 3)

        if not encountered:
            injected_outcome = "UNTESTED"
        elif not meaningful:
            injected_outcome = "INCONCLUSIVE"
        elif self.primary_reaction_id and self.recovery_id:
            injected_outcome = "PASS"
        elif self.primary_reaction_id:
            injected_outcome = "DEGRADED"
        else:
            injected_outcome = "FAIL"

        safety_status = "PASS" if all(v is not False for v in safety_preservation.values()) else "DEGRADED"
        autonomy_assessment = injected_outcome
        if reason == "interrupted" or reason == "signal":
            autonomy_assessment = "INCONCLUSIVE"
        summary = ScenarioSummaryMetrics(
            route_deviation_m=round(self.max_route_deviation_m, 3),
            safety_preservation=safety_preservation,
            goal_viability=goal_viability,
            injected_uncertainty_encounter_status=encountered,
            evaluation_window_after_encounter_s=round(evaluation_window, 3) if evaluation_window is not None else None,
            meaningful_evaluation_window_satisfied=meaningful,
            baseline_uncertainty_exercised_status=self.baseline_uncertainty_exercised,
            detection_signal_source="/obstacle/front",
            baseline_confound_status={
                "block_island_near_route": True,
                "mr009_active": self.mr009_active,
                "mr011_active": self.mr011_active,
            },
            injected_uncertainty_source=["lidar_classification_degradation_001"] if encountered else [],
        )
        outcomes = ScenarioOutcomes(
            goal_status="PASS" if goal_reached else "FAIL",
            safety_status=safety_status,
            autonomy_assessment=autonomy_assessment,
            baseline_outcome_assessment="PASS" if self.baseline_uncertainty_exercised else "NOT_EVALUATED",
            injected_outcome_assessment=injected_outcome,
            outcome_assessment=injected_outcome,
        )
        additional_metrics = {
            "degraded_sensing_response_latency_ms": self._first_adaptation_latency_ms(),
            "false_obstacle_rate": false_rate,
            "false_obstacle_count": self.false_obstacle_samples,
            "missed_detection_rate": missed_rate,
            "missed_detection_count": self.missed_detection_samples,
            "recovery_time_ms": recovery_time_ms,
            "post_injection_route_deviation_m": round(self.max_route_deviation_m, 3),
            "injected_true_samples": self.injected_true_samples,
            "raw_hazard_samples": self.raw_hazard_samples,
        }
        observability_gaps = [
            "No dedicated collision topic was found in the baseline stack; collision_with_obstacle is not directly observable and is reported as null.",
            "The scenario injects BT-facing obstacle classification messages; raw /scan is observed for attribution but not degraded directly.",
        ]
        bundle = self.session.build_bundle(
            termination_reason=reason,
            started_at_sim_s=self.started_sim_s,
            finished_at_sim_s=now_s,
            summary_metrics=summary,
            outcomes=outcomes,
            additional_metrics=additional_metrics,
            observability_gaps=observability_gaps,
            artifact_paths=ArtifactPaths(
                report=str(self.report_path),
                metrics=str(self.metrics_path),
                timeline=str(self.timeline_path),
                rosbags=str(self.rosbags_dir),
            ),
            metadata={
                "injection_status": "encountered" if encountered else "not_encountered",
                "primary_observed_control_rationale": self._primary_reaction_field("observed_control_rationale"),
                "primary_reaction_scope": self._primary_reaction_field("reaction_scope"),
                "primary_reaction_attribution_status": self._primary_reaction_field("attribution_status"),
                "minimum_post_encounter_observation_window_s": self.min_post_encounter_s,
                "monitor_handling": self.contract.get("monitor_handling", []),
                "fault_attribution_rule": self.contract.get("fault_attribution_rule"),
            },
        )
        write_metrics_json(bundle, self.metrics_path)
        if self.wrapper_metrics_path != self.metrics_path:
            write_metrics_json(bundle, self.wrapper_metrics_path)
        self.timeline_path.parent.mkdir(parents=True, exist_ok=True)
        self.timeline_path.write_text(
            "\n".join(json.dumps(item, sort_keys=True) for item in self.timeline) + "\n",
            encoding="utf-8",
        )
        self.report_path.write_text(render_markdown_report(bundle), encoding="utf-8")
        self.get_logger().info(f"wrote report={self.report_path} metrics={self.metrics_path}")
        self.destroy_timer(self.control_timer)

    def _first_adaptation_latency_ms(self) -> Optional[float]:
        if self.injection_started_s is None or self.primary_reaction_id is None:
            return None
        for event in self.session.build_bundle(termination_reason=None).adaptation_events:
            if event.adaptation_latency_ms is not None:
                return float(event.adaptation_latency_ms)
        return None

    def _primary_reaction_field(self, field_name: str) -> Optional[Any]:
        if self.primary_reaction_id is None:
            return None
        for event in self.session.build_bundle(termination_reason=None).reaction_events:
            if event.event_id == self.primary_reaction_id:
                return getattr(event, field_name)
        return None


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = PerceptionLidarDegradationDriver()

    def _handle_signal(signum: int, _frame: Any) -> None:
        node.finalize("signal")
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _handle_signal)
    signal.signal(signal.SIGINT, _handle_signal)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        node.finalize("interrupted")
    finally:
        if not node.finished:
            node.finalize("interrupted")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
