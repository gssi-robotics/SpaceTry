#!/usr/bin/env python3
import copy
import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import yaml

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))
@dataclass
class EventRuntime:
    config: Dict
    status: str = "pending"
    trigger_time_s: Optional[float] = None
    reaction_time_s: Optional[float] = None
    recovery_time_s: Optional[float] = None
    completed_time_s: Optional[float] = None
    active_intensity: float = 0.0
    baseline_cmd_linear: float = 0.0
    baseline_cmd_angular: float = 0.0
    baseline_goal_distance: Optional[float] = None
    baseline_outpost_distance: Optional[float] = None
    safety_snapshot: Dict[str, bool] = field(default_factory=dict)
    goal_viability_snapshot: Dict[str, bool] = field(default_factory=dict)
    notes: List[str] = field(default_factory=list)


class ScenarioDriver(Node):
    def __init__(self) -> None:
        super().__init__("spacetry_scenario_driver")

        self.declare_parameter("scenario_config", "")
        self.declare_parameter("report_path", "")
        self.declare_parameter("goal_waypoint", "science_rock_01")
        self.declare_parameter("outpost_waypoint", "outpost_habitat_01")
        self.declare_parameter("odom_topic", "/model/curiosity_mars_rover/odometry")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("battery_soc_topic", "/battery/soc")
        self.declare_parameter("battery_near_outpost_topic", "/battery/near_outpost")
        self.declare_parameter("obstacle_front_topic", "/obstacle/front")
        self.declare_parameter("monitor_events_topic", "monitor/events")
        self.declare_parameter(
            "monitor_handler_topics",
            [
                "monitor/handlerMR_009_RETURN_TO_OUTPOST_ON_LOW_BATTERY",
                "monitor/handlerMR_011_MAINTAIN_SPEED_WHEN_FULL_BATTERY",
            ],
        )

        self.scenario = self._load_scenario_config()
        self.report_path = self._resolve_report_path()

        self.goal_waypoint = str(self.get_parameter("goal_waypoint").value)
        self.outpost_waypoint = str(self.get_parameter("outpost_waypoint").value)
        self.waypoints = self._load_waypoints()

        self.goal_pose = self.waypoints.get(self.goal_waypoint)
        self.outpost_pose = self.waypoints.get(self.outpost_waypoint)
        if self.goal_pose is None or self.outpost_pose is None:
            raise RuntimeError("Scenario driver requires mission waypoints for the goal and outpost")

        monitoring = self.scenario.get("monitoring", {})
        self.tick_hz = float(monitoring.get("tick_hz", 10.0))
        self.min_motion_linear_x = float(monitoring.get("min_motion_linear_x", 0.2))
        self.contingency_turn_rate = float(monitoring.get("contingency_turn_rate", 0.35))
        self.low_battery_threshold = float(monitoring.get("low_battery_threshold", 0.2))
        self.stable_window_s = float(monitoring.get("stable_window_s", 2.0))
        self.max_runtime_s = float(monitoring.get("max_runtime_s", 240.0))

        self.last_odom: Optional[Odometry] = None
        self.last_scan: Optional[LaserScan] = None
        self.last_cmd_vel: Optional[Twist] = None
        self.last_cmd_time_s: Optional[float] = None
        self.battery_soc: Optional[float] = None
        self.near_outpost: Optional[bool] = None
        self.obstacle_front: bool = False
        self.monitor_events: List[Dict] = []
        self.safety_constraints = {
            "MR_009_RETURN_TO_OUTPOST_ON_LOW_BATTERY": True,
            "MR_011_MAINTAIN_SPEED_WHEN_FULL_BATTERY": True,
        }

        self.distance_travelled_m = 0.0
        self.last_xy: Optional[List[float]] = None
        self.first_motion_seen = False
        self.contingency_seen = False
        self.mission_complete = False
        self.started_at_s = self.now_s()
        self.reaction_stable_since_s: Optional[float] = None
        self.global_intensity_bias = 0.0

        self.events: List[EventRuntime] = [EventRuntime(config=cfg) for cfg in self.scenario.get("events", [])]
        self.active_event: Optional[EventRuntime] = None
        self.final_report_written = False
        self._subscriptions = []

        odom_topic = str(self.get_parameter("odom_topic").value)
        scan_topic = str(self.get_parameter("scan_topic").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        battery_soc_topic = str(self.get_parameter("battery_soc_topic").value)
        battery_near_outpost_topic = str(self.get_parameter("battery_near_outpost_topic").value)
        obstacle_front_topic = str(self.get_parameter("obstacle_front_topic").value)

        self._subscriptions.append(self.create_subscription(Odometry, odom_topic, self._on_odom, 10))
        self._subscriptions.append(self.create_subscription(LaserScan, scan_topic, self._on_scan, 10))
        self._subscriptions.append(self.create_subscription(Twist, cmd_vel_topic, self._on_cmd_vel, 10))
        self._subscriptions.append(
            self.create_subscription(Float32, battery_soc_topic, self._on_battery_soc, 10)
        )
        self._subscriptions.append(
            self.create_subscription(Bool, battery_near_outpost_topic, self._on_near_outpost, 10)
        )
        self._subscriptions.append(
            self.create_subscription(Bool, obstacle_front_topic, self._on_obstacle_front, 10)
        )
        self._subscriptions.append(
            self.create_subscription(
                String,
                str(self.get_parameter("monitor_events_topic").value),
                self._on_monitor_event,
                10,
            )
        )

        for topic in self.get_parameter("monitor_handler_topics").value:
            self._subscriptions.append(
                self.create_subscription(Bool, str(topic), self._make_handler_cb(str(topic)), 10)
            )

        self.scan_override_pub = self.create_publisher(LaserScan, scan_topic, 10)
        self.battery_soc_override_pub = self.create_publisher(Float32, battery_soc_topic, 10)
        self.battery_near_override_pub = self.create_publisher(Bool, battery_near_outpost_topic, 10)
        self.events_pub = self.create_publisher(String, "/scenario_driver/events", 10)
        self.report_pub = self.create_publisher(String, "/scenario_driver/report", 10)

        self.tick_timer = self.create_timer(1.0 / max(self.tick_hz, 1.0), self._tick)
        self.get_logger().info(f"Scenario driver loaded {len(self.events)} events from {self.scenario['__source_path']}")

    def _resolve_report_path(self) -> str:
        configured = str(self.get_parameter("report_path").value).strip()
        if configured:
            return configured
        return str(self.scenario.get("report_path", "/tmp/spacetry_autonomy_report.json"))

    def _load_scenario_config(self) -> Dict:
        configured = str(self.get_parameter("scenario_config").value).strip()
        if configured:
            path = Path(configured)
        else:
            path = (
                Path(get_package_share_directory("spacetry_scenario_driver"))
                / "config"
                / "autonomy_scenario.yaml"
            )

        with path.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}

        data["__source_path"] = str(path)
        return data

    def _load_waypoints(self) -> Dict[str, Dict[str, float]]:
        waypoints_path = (
            Path(get_package_share_directory("spacetry_mission")) / "config" / "waypoints.yaml"
        )
        with waypoints_path.open("r", encoding="utf-8") as handle:
            raw = yaml.safe_load(handle) or {}
        return raw.get("waypoints", {})

    def _make_handler_cb(self, topic: str):
        def _cb(msg: Bool) -> None:
            if not msg.data:
                return
            if "MR_009" in topic:
                self.safety_constraints["MR_009_RETURN_TO_OUTPOST_ON_LOW_BATTERY"] = False
            if "MR_011" in topic:
                self.safety_constraints["MR_011_MAINTAIN_SPEED_WHEN_FULL_BATTERY"] = False
            self.monitor_events.append({"time_s": self.now_s(), "event": topic})

        return _cb

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_odom(self, msg: Odometry) -> None:
        self.last_odom = msg
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        if self.last_xy is not None:
            self.distance_travelled_m += math.hypot(x - self.last_xy[0], y - self.last_xy[1])
        self.last_xy = [x, y]

    def _on_scan(self, msg: LaserScan) -> None:
        self.last_scan = msg

    def _on_cmd_vel(self, msg: Twist) -> None:
        self.last_cmd_vel = msg
        self.last_cmd_time_s = self.now_s()
        if abs(msg.linear.x) >= self.min_motion_linear_x:
            self.first_motion_seen = True
        if self.obstacle_front or abs(msg.angular.z) >= self.contingency_turn_rate:
            self.contingency_seen = True

    def _on_battery_soc(self, msg: Float32) -> None:
        self.battery_soc = float(msg.data)

    def _on_near_outpost(self, msg: Bool) -> None:
        self.near_outpost = bool(msg.data)

    def _on_obstacle_front(self, msg: Bool) -> None:
        self.obstacle_front = bool(msg.data)
        if self.obstacle_front:
            self.contingency_seen = True

    def _on_monitor_event(self, msg: String) -> None:
        text = msg.data.strip()
        self.monitor_events.append({"time_s": self.now_s(), "event": text})
        if "MR_009" in text:
            self.safety_constraints["MR_009_RETURN_TO_OUTPOST_ON_LOW_BATTERY"] = False
        if "MR_011" in text:
            self.safety_constraints["MR_011_MAINTAIN_SPEED_WHEN_FULL_BATTERY"] = False

    def _tick(self) -> None:
        now_s = self.now_s()
        if (now_s - self.started_at_s) >= self.max_runtime_s:
            self._finalize("max_runtime_reached")
            return

        if self.last_odom is None or self.last_cmd_vel is None:
            return

        if self._distance_to_goal(self.goal_pose) <= self.goal_pose.get("tolerance", 1.0):
            self.mission_complete = True

        if self.active_event is None:
            next_event = self._next_pending_event()
            if next_event is not None and self._trigger_is_ready(next_event):
                self._activate_event(next_event, now_s)

        if self.active_event is not None:
            self._publish_injection(self.active_event, now_s)
            self._update_event_progress(self.active_event, now_s)

        if self.mission_complete and self.active_event is None and not self.final_report_written:
            self._finalize("mission_complete")
            return

        if all(event.status == "completed" for event in self.events) and self.active_event is None:
            self._finalize("all_events_completed")

    def _next_pending_event(self) -> Optional[EventRuntime]:
        for event in self.events:
            if event.status == "pending":
                return event
        return None

    def _trigger_is_ready(self, event: EventRuntime) -> bool:
        trigger = event.config.get("trigger", {})
        trigger_type = trigger.get("type")

        if trigger_type == "first_motion":
            return self.first_motion_seen
        if trigger_type == "distance_travelled":
            return self.distance_travelled_m >= float(trigger.get("distance_m", 0.0))
        if trigger_type == "contingency_active":
            return self.contingency_seen
        return False

    def _activate_event(self, event: EventRuntime, now_s: float) -> None:
        event.status = "active"
        event.trigger_time_s = now_s
        event.active_intensity = self._initial_intensity(event)
        event.safety_snapshot = dict(self.safety_constraints)
        event.goal_viability_snapshot = self._goal_viability()
        event.baseline_goal_distance = self._distance_to_goal(self.goal_pose)
        event.baseline_outpost_distance = self._distance_to_goal(self.outpost_pose)
        event.notes.append(
            f"Activated at {now_s - self.started_at_s:.2f}s during {event.config.get('injection_timing')}"
        )
        if self.last_cmd_vel is not None:
            event.baseline_cmd_linear = float(self.last_cmd_vel.linear.x)
            event.baseline_cmd_angular = float(self.last_cmd_vel.angular.z)
        self.active_event = event
        self._publish_status(
            f"ACTIVATE {event.config.get('id')} timing={event.config.get('injection_timing')} "
            f"profile={event.config.get('intensity_increase')} intensity={event.active_intensity:.2f}"
        )

    def _initial_intensity(self, event: EventRuntime) -> float:
        injector = event.config.get("injector", {})
        base = float(injector.get("base_intensity", 0.5))
        upper = float(injector.get("max_intensity", 1.0))
        return clamp(base + self.global_intensity_bias, 0.0, upper)

    def _publish_injection(self, event: EventRuntime, now_s: float) -> None:
        injector = event.config.get("injector", {})
        elapsed_s = now_s - float(event.trigger_time_s or now_s)
        duration_s = float(injector.get("duration_s", 1.0))
        if elapsed_s > duration_s:
            return

        intensity = self._current_intensity(event, elapsed_s)
        event.active_intensity = intensity
        injector_type = injector.get("type")

        if injector_type in {"scan_dropout", "scan_obstacle"} and self.last_scan is not None:
            modified = self._build_scan_override(self.last_scan, injector_type, injector, intensity)
            self.scan_override_pub.publish(modified)
        elif injector_type == "battery_override":
            self._publish_battery_override(injector, intensity)

    def _current_intensity(self, event: EventRuntime, elapsed_s: float) -> float:
        injector = event.config.get("injector", {})
        profile = event.config.get("intensity_increase", "sudden")
        base = float(injector.get("base_intensity", 0.5))
        step = float(injector.get("step", 0.1))
        upper = float(injector.get("max_intensity", 1.0))

        if profile == "gradual":
            return clamp(base + self.global_intensity_bias + step * elapsed_s, 0.0, upper)
        if profile == "cascading":
            stage = int(elapsed_s // 2.0)
            return clamp(base + self.global_intensity_bias + step * stage, 0.0, upper)
        return clamp(base + self.global_intensity_bias + step, 0.0, upper)

    def _build_scan_override(
        self,
        source: LaserScan,
        injector_type: str,
        injector: Dict,
        intensity: float,
    ) -> LaserScan:
        scan = copy.deepcopy(source)
        scan.header.stamp = self.get_clock().now().to_msg()
        ranges = list(scan.ranges)
        if not ranges:
            return scan

        front_sector_rad = math.radians(float(injector.get("front_sector_deg", 20.0)))
        min_angle = -front_sector_rad / 2.0
        max_angle = front_sector_rad / 2.0
        for idx in self._sector_indices(scan, min_angle, max_angle):
            if injector_type == "scan_dropout":
                if idx % 2 == 0 or intensity > 0.75:
                    ranges[idx] = float("nan")
                else:
                    ranges[idx] = float(scan.range_max) * max(0.1, 1.0 - intensity)
            else:
                obstacle_distance = float(injector.get("obstacle_distance_m", 0.8))
                ranges[idx] = min(ranges[idx], max(scan.range_min + 0.05, obstacle_distance * (1.1 - intensity)))
        scan.ranges = ranges
        return scan

    def _sector_indices(self, scan: LaserScan, min_angle: float, max_angle: float) -> range:
        if scan.angle_increment == 0.0:
            return range(0, 0)
        start_idx = int((min_angle - scan.angle_min) / scan.angle_increment)
        end_idx = int((max_angle - scan.angle_min) / scan.angle_increment)
        start_idx = max(0, min(start_idx, len(scan.ranges) - 1))
        end_idx = max(0, min(end_idx, len(scan.ranges) - 1))
        if end_idx < start_idx:
            start_idx, end_idx = end_idx, start_idx
        return range(start_idx, end_idx + 1)

    def _publish_battery_override(self, injector: Dict, intensity: float) -> None:
        override_soc = float(injector.get("override_soc", self.low_battery_threshold))
        msg_soc = Float32()
        msg_soc.data = max(0.0, override_soc - 0.05 * intensity)
        self.battery_soc_override_pub.publish(msg_soc)

        msg_near = Bool()
        msg_near.data = bool(injector.get("force_near_outpost", False))
        self.battery_near_override_pub.publish(msg_near)

    def _update_event_progress(self, event: EventRuntime, now_s: float) -> None:
        injector = event.config.get("injector", {})
        reaction = event.config.get("reaction", {})
        duration_s = float(injector.get("duration_s", 0.0))
        elapsed_s = now_s - float(event.trigger_time_s or now_s)

        if event.reaction_time_s is None and self._reaction_detected(event, reaction):
            event.reaction_time_s = now_s
            event.notes.append(
                f"Reaction detected after {(event.reaction_time_s - event.trigger_time_s) * 1000.0:.1f} ms"
            )
            self._publish_status(f"REACTION {event.config.get('id')}")

        if event.reaction_time_s is not None and self._recovery_detected(event):
            if self.reaction_stable_since_s is None:
                self.reaction_stable_since_s = now_s
            if now_s - self.reaction_stable_since_s >= self.stable_window_s:
                event.recovery_time_s = now_s
        else:
            self.reaction_stable_since_s = None

        reaction_timeout_s = float(reaction.get("reaction_timeout_s", 15.0))
        recovery_timeout_s = float(reaction.get("recovery_timeout_s", 25.0))

        reaction_timed_out = event.reaction_time_s is None and elapsed_s >= reaction_timeout_s
        recovery_timed_out = (
            event.reaction_time_s is not None
            and event.recovery_time_s is None
            and (now_s - event.reaction_time_s) >= recovery_timeout_s
        )
        finished_injection = elapsed_s >= duration_s

        if reaction_timed_out:
            event.notes.append("Reaction timeout reached before adaptation was observed")

        if recovery_timed_out:
            event.notes.append("Recovery timeout reached before a stable outcome was observed")

        if finished_injection and (event.recovery_time_s is not None or recovery_timed_out or reaction_timed_out):
            self._complete_event(event, now_s)

    def _reaction_detected(self, event: EventRuntime, reaction: Dict) -> bool:
        if self.last_cmd_vel is None:
            return False

        linear_delta = abs(float(self.last_cmd_vel.linear.x) - event.baseline_cmd_linear)
        angular_delta = abs(float(self.last_cmd_vel.angular.z) - event.baseline_cmd_angular)
        target_linear_delta = float(reaction.get("linear_delta", 0.25))
        target_angular_delta = float(reaction.get("angular_delta", 0.2))

        if linear_delta >= target_linear_delta or angular_delta >= target_angular_delta:
            return True

        injector_type = event.config.get("injector", {}).get("type")
        if injector_type == "battery_override":
            current_outpost_distance = self._distance_to_goal(self.outpost_pose)
            baseline = event.baseline_outpost_distance or current_outpost_distance
            return current_outpost_distance < baseline - 0.75 or bool(self.near_outpost)

        return self.obstacle_front

    def _recovery_detected(self, event: EventRuntime) -> bool:
        safety_ok = all(self.safety_constraints.values())
        goal_viability = self._goal_viability()
        injector_type = event.config.get("injector", {}).get("type")

        if injector_type == "battery_override":
            current_outpost_distance = self._distance_to_goal(self.outpost_pose)
            baseline = event.baseline_outpost_distance or current_outpost_distance
            return safety_ok and (bool(self.near_outpost) or current_outpost_distance < baseline - 1.5)

        current_goal_distance = self._distance_to_goal(self.goal_pose)
        baseline_goal = event.baseline_goal_distance or current_goal_distance
        return safety_ok and goal_viability["science_rock_01"] and current_goal_distance < baseline_goal - 0.75

    def _complete_event(self, event: EventRuntime, now_s: float) -> None:
        event.status = "completed"
        event.completed_time_s = now_s
        event.goal_viability_snapshot = self._goal_viability()
        self.global_intensity_bias = clamp(
            self.global_intensity_bias + 0.05 if event.recovery_time_s is not None else self.global_intensity_bias,
            0.0,
            0.3,
        )
        self._publish_status(
            f"COMPLETE {event.config.get('id')} reaction_ms={self._duration_ms(event.trigger_time_s, event.reaction_time_s)} "
            f"recovery_ms={self._duration_ms(event.reaction_time_s, event.recovery_time_s)}"
        )
        self.active_event = None
        self.reaction_stable_since_s = None

    def _goal_viability(self) -> Dict[str, bool]:
        science_goal_distance = self._distance_to_goal(self.goal_pose)
        outpost_distance = self._distance_to_goal(self.outpost_pose)
        science_viable = science_goal_distance < 120.0 and (
            self.battery_soc is None or self.battery_soc > self.low_battery_threshold * 0.9
        )
        return_home_viable = (
            outpost_distance < 150.0
            and (self.battery_soc is None or self.battery_soc >= 0.0)
            and self.safety_constraints["MR_009_RETURN_TO_OUTPOST_ON_LOW_BATTERY"]
        )
        return {
            "science_rock_01": bool(science_viable),
            "outpost_habitat_01": bool(return_home_viable),
        }

    def _distance_to_goal(self, waypoint: Dict[str, float]) -> float:
        if self.last_odom is None:
            return float("inf")
        x = float(self.last_odom.pose.pose.position.x)
        y = float(self.last_odom.pose.pose.position.y)
        return math.hypot(x - float(waypoint["x"]), y - float(waypoint["y"]))

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.events_pub.publish(msg)
        self.get_logger().info(text)

    def _duration_ms(self, start_s: Optional[float], end_s: Optional[float]) -> Optional[float]:
        if start_s is None or end_s is None:
            return None
        return round((end_s - start_s) * 1000.0, 3)

    def _event_outcome(self, event: EventRuntime) -> str:
        safety_ok = all(event.goal_viability_snapshot.values()) and all(self.safety_constraints.values())
        adapted = event.reaction_time_s is not None
        if adapted and safety_ok:
            return "Autonomy achieved"
        if adapted:
            return "Degraded"
        return "Failed"

    def _build_report(self, reason: str) -> Dict:
        events_payload = []
        for event in self.events:
            safety_state = dict(self.safety_constraints)
            goal_viability = event.goal_viability_snapshot or self._goal_viability()
            events_payload.append(
                {
                    "id": event.config.get("id"),
                    "category": event.config.get("category"),
                    "injection_timing": event.config.get("injection_timing"),
                    "intensity_increase": event.config.get("intensity_increase"),
                    "status": event.status,
                    "adaptation_speed_ms": self._duration_ms(event.trigger_time_s, event.reaction_time_s),
                    "recovery_rate_ms": self._duration_ms(event.reaction_time_s, event.recovery_time_s),
                    "safety_preservation": safety_state,
                    "goal_viability": goal_viability,
                    "outcome": self._event_outcome(event),
                    "notes": event.notes,
                }
            )

        final_safety = dict(self.safety_constraints)
        final_goal_viability = self._goal_viability()
        return {
            "scenario_id": self.scenario.get("scenario_id", "unknown"),
            "mission_goal": self.scenario.get("mission_goal", self.goal_waypoint),
            "reason": reason,
            "runtime_s": round(self.now_s() - self.started_at_s, 3),
            "distance_travelled_m": round(self.distance_travelled_m, 3),
            "safety_preservation": final_safety,
            "goal_viability": final_goal_viability,
            "events": events_payload,
            "monitor_events": self.monitor_events,
        }

    def _finalize(self, reason: str) -> None:
        if self.final_report_written:
            return

        report = self._build_report(reason)
        report_path = Path(self.report_path)
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")

        msg = String()
        msg.data = json.dumps(report)
        self.report_pub.publish(msg)
        self.get_logger().info(f"Scenario report written to {report_path}")
        self.final_report_written = True
        rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = ScenarioDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
