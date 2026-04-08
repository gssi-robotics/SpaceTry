#!/usr/bin/env python3
import json
import math
import os
from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict, List, Optional

import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Float32, String


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def distance_xy(a_x: float, a_y: float, b_x: float, b_y: float) -> float:
    return math.hypot(a_x - b_x, a_y - b_y)


@dataclass
class EventRuntime:
    event_id: str
    timing: str
    intensity: str
    autonomy_aspect: str
    uncertainty_location: str
    uncertainty_nature: str
    fault_subject: str
    fault_attribute: str
    manifestation: str
    time_domain: str
    space_domain: str
    trigger: str
    active: bool = False
    completed: bool = False
    start_time_s: Optional[float] = None
    end_time_s: Optional[float] = None
    reaction_time_s: Optional[float] = None
    recovery_time_s: Optional[float] = None
    outcome: str = "pending"
    response_signal: Optional[str] = None
    notes: List[str] = field(default_factory=list)

    def to_dict(self) -> Dict[str, object]:
        return {
            "event_id": self.event_id,
            "timing": self.timing,
            "intensity": self.intensity,
            "autonomy_aspect": self.autonomy_aspect,
            "uncertainty_location": self.uncertainty_location,
            "uncertainty_nature": self.uncertainty_nature,
            "fault_subject": self.fault_subject,
            "fault_attribute": self.fault_attribute,
            "manifestation": self.manifestation,
            "time_domain": self.time_domain,
            "space_domain": self.space_domain,
            "trigger": self.trigger,
            "start_time_s": self.start_time_s,
            "end_time_s": self.end_time_s,
            "reaction_time_s": self.reaction_time_s,
            "recovery_time_s": self.recovery_time_s,
            "outcome": self.outcome,
            "response_signal": self.response_signal,
            "notes": self.notes,
        }


class ScenarioDriver(Node):
    def __init__(self) -> None:
        super().__init__("scenario_driver")

        self.declare_parameter("timer_period_s", 0.1)
        self.declare_parameter("override_publish_hz", 20.0)
        self.declare_parameter("report_dir", "/tmp/spacetry_scenario_uncertainty_stress")
        self.declare_parameter("contract_path", "")
        self.declare_parameter("scenario_timeout_s", 90.0)
        self.declare_parameter("response_timeout_s", 18.0)
        self.declare_parameter("recovery_timeout_s", 25.0)
        self.declare_parameter("movement_commit_linear_threshold", 0.15)
        self.declare_parameter("movement_mid_action_linear_threshold", 0.5)
        self.declare_parameter("reaction_linear_drop_threshold", 0.2)
        self.declare_parameter("reaction_turn_threshold", 0.25)
        self.declare_parameter("cruise_linear_threshold", 0.45)
        self.declare_parameter("settle_turn_threshold", 0.18)
        self.declare_parameter("decision_event_duration_s", 8.0)
        self.declare_parameter("decision_ramp_step_s", 2.0)
        self.declare_parameter("battery_drop_soc", 0.15)
        self.declare_parameter("battery_event_duration_s", 18.0)
        self.declare_parameter("contingency_duration_s", 12.0)
        self.declare_parameter("contingency_pulse_period_s", 1.5)
        self.declare_parameter("science_goal_x", 50.1)
        self.declare_parameter("science_goal_y", -80.0)
        self.declare_parameter("science_goal_tolerance_m", 2.0)
        self.declare_parameter("outpost_x", 66.0)
        self.declare_parameter("outpost_y", 0.0)
        self.declare_parameter("outpost_tolerance_m", 2.0)

        self.report_dir = str(self.get_parameter("report_dir").value)
        self.contract = self.load_contract(str(self.get_parameter("contract_path").value))
        self.scenario_timeout_s = float(self.get_parameter("scenario_timeout_s").value)
        self.response_timeout_s = float(self.get_parameter("response_timeout_s").value)
        self.recovery_timeout_s = float(self.get_parameter("recovery_timeout_s").value)
        self.override_period_s = 1.0 / max(
            1.0, float(self.get_parameter("override_publish_hz").value)
        )

        self.science_goal = (
            float(self.get_parameter("science_goal_x").value),
            float(self.get_parameter("science_goal_y").value),
        )
        self.science_tolerance = float(
            self.get_parameter("science_goal_tolerance_m").value
        )
        self.outpost_goal = (
            float(self.get_parameter("outpost_x").value),
            float(self.get_parameter("outpost_y").value),
        )
        self.outpost_tolerance = float(
            self.get_parameter("outpost_tolerance_m").value
        )

        self.last_soc: Optional[float] = None
        self.last_near_outpost: Optional[bool] = None
        self.last_cmd_vel: Optional[Twist] = None
        self.last_obstacle_state: str = "UNKNOWN"
        self.last_position: Optional[tuple[float, float]] = None
        self.start_position: Optional[tuple[float, float]] = None
        self.monitor_violations: Dict[str, bool] = {
            "MR_009_RETURN_TO_OUTPOST_ON_LOW_BATTERY": False,
            "MR_011_MAINTAIN_SPEED_WHEN_FULL_BATTERY": False,
        }
        self.monitor_events: List[Dict[str, object]] = []
        self.timeline: List[Dict[str, object]] = []
        self.report_written = False

        self.current_obstacle_override = {
            "front": False,
            "left": False,
            "right": False,
            "state": "CLEAR",
        }
        self.current_battery_override: Dict[str, object] = {}
        self.decision_ramp_level = 0
        self.contingency_pulse_index = 0
        self.scenario_started_at_s = self.now_s()

        self.decision_event = EventRuntime(
            event_id="decision_point_perception_ramp",
            timing="decision_point",
            intensity="gradual",
            autonomy_aspect="perception",
            uncertainty_location="adaptation_functions",
            uncertainty_nature="variability",
            fault_subject="robot.sensor",
            fault_attribute="obstacle perception output",
            manifestation="degrading",
            time_domain="transient",
            space_domain="scattered",
            trigger="first sustained navigation command on /cmd_vel",
        )
        self.battery_event = EventRuntime(
            event_id="mid_action_battery_drop",
            timing="mid_action",
            intensity="sudden",
            autonomy_aspect="resource_awareness",
            uncertainty_location="resources",
            uncertainty_nature="epistemic",
            fault_subject="robot.battery",
            fault_attribute="energy level",
            manifestation="stuck",
            time_domain="transient",
            space_domain="isolated",
            trigger="rover travelling away from the outpost at cruise speed",
        )
        self.contingency_event = EventRuntime(
            event_id="contingency_obstacle_cascade",
            timing="during_contingency",
            intensity="cascading",
            autonomy_aspect="behavior_flexibility",
            uncertainty_location="environment",
            uncertainty_nature="variability",
            fault_subject="environment.hazards",
            fault_attribute="traversability perception",
            manifestation="intermittent",
            time_domain="intermittent",
            space_domain="contiguous",
            trigger="battery stress event active and response not yet recovered",
        )
        self.events = [
            self.decision_event,
            self.battery_event,
            self.contingency_event,
        ]

        self.create_subscription(Float32, "/battery/soc", self.on_soc, 10)
        self.create_subscription(Bool, "/battery/near_outpost", self.on_near_outpost, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.create_subscription(String, "/obstacle/state", self.on_obstacle_state, 10)
        self.create_subscription(
            Odometry,
            "/mobile_base_controller/odom",
            self.on_odom,
            qos_profile_sensor_data,
        )
        self.create_subscription(String, "monitor/events", self.on_monitor_event, 10)

        self.pub_soc = self.create_publisher(Float32, "/battery/soc", 10)
        self.pub_near = self.create_publisher(Bool, "/battery/near_outpost", 10)
        self.pub_front = self.create_publisher(Bool, "obstacle/front", 10)
        self.pub_left = self.create_publisher(Bool, "obstacle/left", 10)
        self.pub_right = self.create_publisher(Bool, "obstacle/right", 10)
        self.pub_state = self.create_publisher(String, "obstacle/state", 10)
        self.pub_status = self.create_publisher(
            String, "scenario_driver/status", 10
        )

        self.create_timer(
            float(self.get_parameter("timer_period_s").value),
            self.on_timer,
        )
        self.create_timer(self.override_period_s, self.publish_overrides)

        self.log_timeline("scenario_started", {"report_dir": self.report_dir})
        self.get_logger().info(
            "Scenario driver ready: evaluating baseline autonomy with decision, "
            "mid-action, and contingency uncertainty injections."
        )

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def load_contract(self, contract_path: str) -> Dict[str, object]:
        fallback = {
            "scenario_name": "uncertainty_stress",
            "baseline_under_test": {
                "behavior_tree": "src/spacetry_bt/trees/base_bt.xml",
                "mission": "src/spacetry_mission/MISSION.md",
                "monitors": [
                    "MR_009_RETURN_TO_OUTPOST_ON_LOW_BATTERY",
                    "MR_011_MAINTAIN_SPEED_WHEN_FULL_BATTERY",
                ],
            },
            "traceability_gaps": [
                "Dynamic obstacles are emulated through obstacle topic overrides rather than direct Gazebo model insertion.",
                "Behavior-tree decision points are approximated from /cmd_vel because the BT runner does not expose explicit decision-state topics.",
                "Return-to-outpost intent is inferred from monitor state and pose rather than a dedicated mission-status topic.",
            ],
        }
        if not contract_path:
            return fallback
        try:
            with open(contract_path, "r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle) or {}
            return data.get("scenario_contract", fallback)
        except OSError as exc:
            self.get_logger().warning(
                f"Failed to load contract file '{contract_path}': {exc}. Using fallback contract."
            )
            return fallback

    def log_timeline(self, label: str, payload: Dict[str, object]) -> None:
        entry = {"time_s": round(self.now_s(), 3), "label": label, "payload": payload}
        self.timeline.append(entry)
        msg = String()
        msg.data = json.dumps(entry, sort_keys=True)
        self.pub_status.publish(msg)

    def on_soc(self, msg: Float32) -> None:
        self.last_soc = float(msg.data)

    def on_near_outpost(self, msg: Bool) -> None:
        self.last_near_outpost = bool(msg.data)

    def on_cmd_vel(self, msg: Twist) -> None:
        self.last_cmd_vel = msg

    def on_obstacle_state(self, msg: String) -> None:
        self.last_obstacle_state = msg.data

    def on_odom(self, msg: Odometry) -> None:
        self.last_position = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )
        if self.start_position is None:
            self.start_position = self.last_position

    def on_monitor_event(self, msg: String) -> None:
        event_text = msg.data
        violation_name = None
        if "MR_009" in event_text:
            violation_name = "MR_009_RETURN_TO_OUTPOST_ON_LOW_BATTERY"
        elif "MR_011" in event_text:
            violation_name = "MR_011_MAINTAIN_SPEED_WHEN_FULL_BATTERY"

        record = {"time_s": round(self.now_s(), 3), "message": event_text}
        self.monitor_events.append(record)
        self.log_timeline("monitor_event", record)

        if violation_name is not None:
            self.monitor_violations[violation_name] = True

    def linear_speed(self) -> float:
        if self.last_cmd_vel is None:
            return 0.0
        return abs(float(self.last_cmd_vel.linear.x))

    def angular_speed(self) -> float:
        if self.last_cmd_vel is None:
            return 0.0
        return abs(float(self.last_cmd_vel.angular.z))

    def distance_to_science_goal(self) -> Optional[float]:
        if self.last_position is None:
            return None
        return distance_xy(
            self.last_position[0],
            self.last_position[1],
            self.science_goal[0],
            self.science_goal[1],
        )

    def distance_to_outpost(self) -> Optional[float]:
        if self.last_position is None:
            return None
        return distance_xy(
            self.last_position[0],
            self.last_position[1],
            self.outpost_goal[0],
            self.outpost_goal[1],
        )

    def decision_triggered(self) -> bool:
        return self.linear_speed() >= float(
            self.get_parameter("movement_commit_linear_threshold").value
        )

    def mid_action_triggered(self) -> bool:
        if self.last_position is None:
            return False
        distance = self.distance_to_outpost()
        if distance is None:
            return False
        return (
            self.linear_speed()
            >= float(self.get_parameter("movement_mid_action_linear_threshold").value)
            and distance > self.outpost_tolerance + 3.0
        )

    def record_reaction_if_needed(self, event: EventRuntime, signal: str) -> None:
        if event.start_time_s is None or event.reaction_time_s is not None:
            return
        event.reaction_time_s = self.now_s()
        event.response_signal = signal
        self.log_timeline(
            "reaction_detected",
            {
                "event_id": event.event_id,
                "signal": signal,
                "adaptation_speed_ms": self.metric_ms(
                    event.start_time_s, event.reaction_time_s
                ),
            },
        )

    def metric_ms(self, start_time: Optional[float], end_time: Optional[float]) -> Optional[float]:
        if start_time is None or end_time is None:
            return None
        return round((end_time - start_time) * 1000.0, 1)

    def start_event(self, event: EventRuntime, extra: Optional[Dict[str, object]] = None) -> None:
        event.active = True
        event.start_time_s = self.now_s()
        payload = {"event_id": event.event_id, "trigger": event.trigger}
        if extra is not None:
            payload.update(extra)
        self.log_timeline("injection_started", payload)

    def complete_event(self, event: EventRuntime, outcome: str, note: str) -> None:
        if event.completed:
            return
        event.active = False
        event.completed = True
        event.end_time_s = self.now_s()
        event.outcome = outcome
        event.notes.append(note)
        self.log_timeline(
            "injection_completed",
            {
                "event_id": event.event_id,
                "outcome": outcome,
                "note": note,
                "recovery_rate_ms": self.metric_ms(
                    event.reaction_time_s, event.recovery_time_s
                ),
            },
        )

    def update_decision_event(self) -> None:
        if not self.decision_event.active and not self.decision_event.completed:
            if self.decision_triggered():
                self.start_event(self.decision_event, {"timing_proxy": "cmd_vel"})

        if not self.decision_event.active:
            return

        elapsed = self.now_s() - float(self.decision_event.start_time_s)
        ramp_step = float(self.get_parameter("decision_ramp_step_s").value)
        self.decision_ramp_level = int(elapsed // max(ramp_step, 0.1))
        if self.decision_ramp_level <= 0:
            self.current_obstacle_override = {
                "front": True,
                "left": False,
                "right": False,
                "state": "FRONT",
            }
        elif self.decision_ramp_level == 1:
            self.current_obstacle_override = {
                "front": True,
                "left": True,
                "right": False,
                "state": "FRONT_LEFT",
            }
        else:
            self.current_obstacle_override = {
                "front": True,
                "left": True,
                "right": True,
                "state": "FRONT_BOTH",
            }

        if (
            self.angular_speed()
            >= float(self.get_parameter("reaction_turn_threshold").value)
        ):
            self.record_reaction_if_needed(self.decision_event, "turn_command_detected")
        elif (
            self.linear_speed()
            <= float(self.get_parameter("reaction_linear_drop_threshold").value)
        ):
            self.record_reaction_if_needed(self.decision_event, "linear_speed_drop")

        if (
            self.decision_event.reaction_time_s is not None
            and self.linear_speed()
            >= float(self.get_parameter("cruise_linear_threshold").value)
            and self.angular_speed()
            <= float(self.get_parameter("settle_turn_threshold").value)
        ):
            self.decision_event.recovery_time_s = self.now_s()
            self.current_obstacle_override = {
                "front": False,
                "left": False,
                "right": False,
                "state": "CLEAR",
            }
            self.complete_event(
                self.decision_event,
                "adapted",
                "Rover recovered cruise motion after perception degradation.",
            )
            return

        if elapsed >= float(self.get_parameter("decision_event_duration_s").value):
            self.current_obstacle_override = {
                "front": False,
                "left": False,
                "right": False,
                "state": "CLEAR",
            }
            outcome = (
                "degraded"
                if self.decision_event.reaction_time_s is not None
                else "failed"
            )
            note = (
                "Decision-point event timed out after partial adaptation."
                if self.decision_event.reaction_time_s is not None
                else "No observable reaction during decision-point degradation."
            )
            self.complete_event(self.decision_event, outcome, note)

    def update_battery_event(self) -> None:
        if (
            self.decision_event.completed
            and not self.battery_event.active
            and not self.battery_event.completed
            and self.mid_action_triggered()
        ):
            self.current_battery_override = {
                "soc": clamp(
                    float(self.get_parameter("battery_drop_soc").value),
                    0.0,
                    1.0,
                ),
                "near_outpost": False,
            }
            self.start_event(
                self.battery_event,
                {"forced_soc": self.current_battery_override["soc"]},
            )

        if not self.battery_event.active:
            return

        if (
            self.linear_speed()
            <= float(self.get_parameter("reaction_linear_drop_threshold").value)
        ):
            self.record_reaction_if_needed(self.battery_event, "speed_reduction")

        if self.last_near_outpost:
            self.record_reaction_if_needed(self.battery_event, "outpost_proximity")

        if self.monitor_violations["MR_009_RETURN_TO_OUTPOST_ON_LOW_BATTERY"]:
            self.record_reaction_if_needed(
                self.battery_event, "monitor_violation_low_battery"
            )

        elapsed = self.now_s() - float(self.battery_event.start_time_s)
        reached_outpost = bool(self.last_near_outpost)
        if reached_outpost:
            self.battery_event.recovery_time_s = self.now_s()
            self.current_battery_override = {}
            self.complete_event(
                self.battery_event,
                "adapted",
                "Rover reached the outpost zone while low-battery stress was active.",
            )
            return

        if elapsed >= float(self.get_parameter("battery_event_duration_s").value):
            self.current_battery_override = {}
            outcome = (
                "degraded"
                if self.monitor_violations["MR_009_RETURN_TO_OUTPOST_ON_LOW_BATTERY"]
                else "failed"
            )
            note = (
                "Low-battery event expired with MR_009 violation still present."
                if outcome == "degraded"
                else "Low-battery event expired without a visible recovery action."
            )
            self.complete_event(self.battery_event, outcome, note)

    def update_contingency_event(self) -> None:
        if (
            self.battery_event.completed
            and not self.contingency_event.active
            and not self.contingency_event.completed
            and self.battery_event.reaction_time_s is None
        ):
            self.complete_event(
                self.contingency_event,
                "skipped",
                "Contingency cascade was skipped because no low-battery reaction was observed.",
            )
            return

        if (
            self.battery_event.active
            and not self.contingency_event.active
            and not self.contingency_event.completed
            and self.battery_event.reaction_time_s is not None
        ):
            self.start_event(
                self.contingency_event,
                {"timing_proxy": self.battery_event.response_signal or "reaction"},
            )

        if not self.contingency_event.active:
            return

        elapsed = self.now_s() - float(self.contingency_event.start_time_s)
        pulse_period = float(self.get_parameter("contingency_pulse_period_s").value)
        pulse_slot = int(elapsed // max(pulse_period, 0.1))
        if pulse_slot != self.contingency_pulse_index:
            self.contingency_pulse_index = pulse_slot
            if pulse_slot % 2 == 0:
                self.current_obstacle_override = {
                    "front": True,
                    "left": False,
                    "right": True,
                    "state": "FRONT_RIGHT",
                }
            else:
                self.current_obstacle_override = {
                    "front": True,
                    "left": True,
                    "right": False,
                    "state": "FRONT_LEFT",
                }

        if (
            self.angular_speed()
            >= float(self.get_parameter("reaction_turn_threshold").value)
        ):
            self.record_reaction_if_needed(
                self.contingency_event, "contingency_turn_command"
            )
        elif (
            self.linear_speed()
            <= float(self.get_parameter("reaction_linear_drop_threshold").value)
        ):
            self.record_reaction_if_needed(
                self.contingency_event, "contingency_speed_reduction"
            )

        if self.last_near_outpost:
            self.contingency_event.recovery_time_s = self.now_s()
            self.current_obstacle_override = {
                "front": False,
                "left": False,
                "right": False,
                "state": "CLEAR",
            }
            self.complete_event(
                self.contingency_event,
                "adapted",
                "Rover remained responsive while compounded contingency stress was active.",
            )
            return

        if elapsed >= float(self.get_parameter("contingency_duration_s").value):
            self.current_obstacle_override = {
                "front": False,
                "left": False,
                "right": False,
                "state": "CLEAR",
            }
            outcome = (
                "degraded"
                if self.contingency_event.reaction_time_s is not None
                else "failed"
            )
            note = (
                "Contingency cascade ended without reaching the outpost."
                if outcome == "degraded"
                else "No contingency response observed during cascaded hazard pulses."
            )
            self.complete_event(self.contingency_event, outcome, note)

    def publish_overrides(self) -> None:
        if self.current_battery_override:
            soc_msg = Float32()
            soc_msg.data = float(self.current_battery_override["soc"])
            self.pub_soc.publish(soc_msg)

            near_msg = Bool()
            near_msg.data = bool(self.current_battery_override["near_outpost"])
            self.pub_near.publish(near_msg)

        if self.decision_event.active or self.contingency_event.active:
            self.pub_front.publish(Bool(data=bool(self.current_obstacle_override["front"])))
            self.pub_left.publish(Bool(data=bool(self.current_obstacle_override["left"])))
            self.pub_right.publish(Bool(data=bool(self.current_obstacle_override["right"])))
            self.pub_state.publish(String(data=str(self.current_obstacle_override["state"])))

    def goal_viability(self) -> Dict[str, bool]:
        science_distance = self.distance_to_science_goal()
        outpost_distance = self.distance_to_outpost()
        science_reached = (
            science_distance is not None and science_distance <= self.science_tolerance
        )
        science_progress = False
        if (
            science_distance is not None
            and self.start_position is not None
        ):
            start_distance = distance_xy(
                self.start_position[0],
                self.start_position[1],
                self.science_goal[0],
                self.science_goal[1],
            )
            science_progress = science_distance < start_distance - 5.0

        return {
            "inspect_science_rock_01": bool(science_reached or science_progress),
            "return_to_outpost_habitat_01": bool(
                outpost_distance is not None
                and (
                    outpost_distance <= self.outpost_tolerance
                    or self.monitor_violations["MR_009_RETURN_TO_OUTPOST_ON_LOW_BATTERY"]
                    is False
                )
            ),
        }

    def scenario_outcome(self) -> str:
        if all(not violated for violated in self.monitor_violations.values()):
            if any(event.outcome == "adapted" for event in self.events):
                return "achieved"
        if any(event.outcome in {"adapted", "degraded"} for event in self.events):
            return "degraded"
        return "failed"

    def write_report(self) -> None:
        if self.report_written:
            return
        os.makedirs(self.report_dir, exist_ok=True)
        timestamp = datetime.utcnow().strftime("%Y%m%dT%H%M%SZ")
        report_path = os.path.join(
            self.report_dir, f"scenario_uncertainty_stress_{timestamp}.json"
        )
        report = {
            "scenario_name": self.contract.get("scenario_name", "uncertainty_stress"),
            "generated_at_utc": timestamp,
            "baseline_under_test": self.contract.get("baseline_under_test", {}),
            "measurements": {
                "adaptation_speed_ms": {
                    event.event_id: self.metric_ms(
                        event.start_time_s, event.reaction_time_s
                    )
                    for event in self.events
                },
                "recovery_rate_ms": {
                    event.event_id: self.metric_ms(
                        event.reaction_time_s, event.recovery_time_s
                    )
                    for event in self.events
                },
                "safety_preservation": {
                    key: (not value)
                    for key, value in self.monitor_violations.items()
                },
                "goal_viability": self.goal_viability(),
            },
            "events": [event.to_dict() for event in self.events],
            "monitor_events": self.monitor_events,
            "timeline": self.timeline,
            "outcome": self.scenario_outcome(),
            "traceability_gaps": self.contract.get("traceability_gaps", []),
        }
        with open(report_path, "w", encoding="utf-8") as handle:
            json.dump(report, handle, indent=2, sort_keys=True)
        self.report_written = True
        self.log_timeline("report_written", {"path": report_path})
        self.get_logger().info(f"Scenario report written to {report_path}")

    def on_timer(self) -> None:
        if self.last_cmd_vel is None or self.last_position is None:
            if (self.now_s() - self.scenario_started_at_s) >= self.scenario_timeout_s:
                for event in self.events:
                    if not event.completed:
                        self.complete_event(
                            event,
                            "failed",
                            "Scenario timeout elapsed before the rover exposed the required runtime signals.",
                        )
                self.write_report()
            return

        self.update_decision_event()
        self.update_battery_event()
        self.update_contingency_event()

        finished = all(event.completed for event in self.events)
        any_timeout = any(
            event.start_time_s is not None
            and event.reaction_time_s is None
            and (self.now_s() - event.start_time_s) >= self.response_timeout_s
            for event in self.events
            if event.active
        )
        if any_timeout:
            for event in self.events:
                if (
                    event.active
                    and event.start_time_s is not None
                    and event.reaction_time_s is None
                    and (self.now_s() - event.start_time_s) >= self.response_timeout_s
                ):
                    event.notes.append(
                        "Response timeout reached before a reaction signal was observed."
                    )

        if (self.now_s() - self.scenario_started_at_s) >= self.scenario_timeout_s:
            for event in self.events:
                if not event.completed:
                    outcome = "degraded" if event.reaction_time_s is not None else "failed"
                    self.complete_event(
                        event,
                        outcome,
                        "Scenario timeout reached before the event naturally completed.",
                    )

        if finished:
            self.write_report()
            return

        if all(event.completed for event in self.events):
            self.write_report()


def main() -> None:
    rclpy.init()
    node = ScenarioDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.write_report()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
