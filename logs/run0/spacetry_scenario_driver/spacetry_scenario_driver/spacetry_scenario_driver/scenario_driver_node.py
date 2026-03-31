#!/usr/bin/env python3
import json
import math
import random
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


@dataclass
class ObstacleSnapshot:
    front: bool = False
    left: bool = False
    right: bool = False
    state: str = "CLEAR"


@dataclass
class ScenarioPhase:
    name: str
    phase_type: str
    trigger: str
    duration_s: float
    intensity: float
    trigger_after_s: Optional[float] = None
    trigger_distance_m: Optional[float] = None
    left_bias: bool = False
    right_bias: bool = False
    notes: str = ""
    activated_at_s: Optional[float] = None
    completed: bool = False

    def should_activate(self, mission_elapsed_s: float, goal_distance_m: Optional[float]) -> bool:
        if self.completed or self.activated_at_s is not None:
            return False
        if self.trigger == "mission_elapsed":
            return mission_elapsed_s >= float(self.trigger_after_s or 0.0)
        if self.trigger == "goal_distance_below":
            return goal_distance_m is not None and goal_distance_m <= float(self.trigger_distance_m or 0.0)
        return False

    def is_active(self, mission_elapsed_s: float) -> bool:
        if self.activated_at_s is None or self.completed:
            return False
        return mission_elapsed_s < self.activated_at_s + self.duration_s


class ScenarioDriver(Node):
    def __init__(self) -> None:
        super().__init__("scenario_driver")
        self.declare_parameter("scenario_file", "")

        self._scenario = self._load_scenario()
        self._rng = random.Random(42)

        self._mission_start_time = self.get_clock().now()
        self._raw_obstacles = ObstacleSnapshot()
        self._final_obstacles = ObstacleSnapshot()
        self._battery_soc = 1.0
        self._position = None
        self._distance_travelled_m = 0.0
        self._goal_reached_at_s: Optional[float] = None
        self._violation_events: List[str] = []
        self._scenario_events: List[str] = []
        self._report_published = False

        scenario = self._scenario["scenario"]
        self._goal_x = float(scenario["goal"]["x"])
        self._goal_y = float(scenario["goal"]["y"])
        self._goal_tolerance_m = float(scenario["goal"]["tolerance_m"])
        self._monitor_events_topic = str(scenario["monitor_events_topic"])
        self._battery_topic = str(scenario["battery_topic"])
        self._odom_topic = str(scenario["odom_topic"])

        raw_topics = scenario["raw_topics"]
        final_topics = scenario["final_topics"]

        self._phases = [
            ScenarioPhase(
                name=str(item["name"]),
                phase_type=str(item["type"]),
                trigger=str(item["trigger"]),
                duration_s=float(item["duration_s"]),
                intensity=float(item.get("intensity", 1.0)),
                trigger_after_s=item.get("trigger_after_s"),
                trigger_distance_m=item.get("trigger_distance_m"),
                left_bias=bool(item.get("left_bias", False)),
                right_bias=bool(item.get("right_bias", False)),
                notes=str(item.get("notes", "")),
            )
            for item in scenario.get("phases", [])
        ]

        self.create_subscription(Bool, str(raw_topics["front"]), self._on_raw_front, 10)
        self.create_subscription(Bool, str(raw_topics["left"]), self._on_raw_left, 10)
        self.create_subscription(Bool, str(raw_topics["right"]), self._on_raw_right, 10)
        self.create_subscription(String, str(raw_topics["state"]), self._on_raw_state, 10)
        self.create_subscription(String, self._monitor_events_topic, self._on_monitor_event, 10)
        self.create_subscription(Float32, self._battery_topic, self._on_battery, 10)
        self.create_subscription(Odometry, self._odom_topic, self._on_odom, 10)

        self._pub_front = self.create_publisher(Bool, str(final_topics["front"]), 10)
        self._pub_left = self.create_publisher(Bool, str(final_topics["left"]), 10)
        self._pub_right = self.create_publisher(Bool, str(final_topics["right"]), 10)
        self._pub_state = self.create_publisher(String, str(final_topics["state"]), 10)
        self._pub_at_goal = self.create_publisher(Bool, str(final_topics["at_goal"]), 10)
        self._pub_events = self.create_publisher(String, str(scenario["events_topic"]), 10)
        self._pub_report = self.create_publisher(String, str(scenario["report_topic"]), 10)

        self.create_timer(0.2, self._on_timer)
        self.get_logger().info(
            f"ScenarioDriver loaded '{scenario['name']}' with {len(self._phases)} injection phases"
        )

    def _load_scenario(self) -> Dict:
        scenario_file = str(self.get_parameter("scenario_file").value)
        if not scenario_file:
            raise RuntimeError("scenario_file parameter is required in scenario mode")

        path = Path(scenario_file)
        if not path.exists():
            raise RuntimeError(f"Scenario file does not exist: {scenario_file}")

        with path.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}

        if "scenario" not in data:
            raise RuntimeError(f"Scenario file missing top-level 'scenario': {scenario_file}")
        return data

    def _mission_elapsed_s(self) -> float:
        return (self.get_clock().now() - self._mission_start_time).nanoseconds * 1e-9

    def _goal_distance_m(self) -> Optional[float]:
        if self._position is None:
            return None
        return math.hypot(self._goal_x - self._position[0], self._goal_y - self._position[1])

    def _publish_event(self, message: str) -> None:
        self._scenario_events.append(message)
        msg = String()
        msg.data = message
        self._pub_events.publish(msg)
        self.get_logger().info(message)

    def _on_raw_front(self, msg: Bool) -> None:
        self._raw_obstacles.front = bool(msg.data)

    def _on_raw_left(self, msg: Bool) -> None:
        self._raw_obstacles.left = bool(msg.data)

    def _on_raw_right(self, msg: Bool) -> None:
        self._raw_obstacles.right = bool(msg.data)

    def _on_raw_state(self, msg: String) -> None:
        self._raw_obstacles.state = msg.data or "CLEAR"

    def _on_monitor_event(self, msg: String) -> None:
        if msg.data:
            self._violation_events.append(msg.data)

    def _on_battery(self, msg: Float32) -> None:
        self._battery_soc = float(msg.data)

    def _on_odom(self, msg: Odometry) -> None:
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        if self._position is not None:
            self._distance_travelled_m += math.hypot(x - self._position[0], y - self._position[1])
        self._position = (x, y)

    def _activate_phase_if_needed(self, phase: ScenarioPhase, mission_elapsed_s: float, goal_distance_m: Optional[float]) -> None:
        if phase.should_activate(mission_elapsed_s, goal_distance_m):
            phase.activated_at_s = mission_elapsed_s
            self._publish_event(
                f"Scenario phase '{phase.name}' activated at t={mission_elapsed_s:.1f}s: {phase.notes}"
            )

    def _apply_injections(self, mission_elapsed_s: float) -> ObstacleSnapshot:
        final = ObstacleSnapshot(
            front=self._raw_obstacles.front,
            left=self._raw_obstacles.left,
            right=self._raw_obstacles.right,
            state=self._raw_obstacles.state,
        )

        for phase in self._phases:
            if not phase.is_active(mission_elapsed_s):
                if phase.activated_at_s is not None and not phase.completed:
                    phase.completed = True
                    self._publish_event(f"Scenario phase '{phase.name}' completed")
                continue

            if phase.phase_type == "front_override" and self._rng.random() <= phase.intensity:
                final.front = True
                final.left = final.left or phase.left_bias
                final.right = final.right or phase.right_bias
                if phase.left_bias:
                    final.state = "LEFT"
                elif phase.right_bias:
                    final.state = "RIGHT"
                else:
                    final.state = "FRONT"

        if final.front and final.state == "CLEAR":
            final.state = "FRONT"
        elif final.left and not final.front:
            final.state = "LEFT"
        elif final.right and not final.front and not final.left:
            final.state = "RIGHT"
        elif not any([final.front, final.left, final.right]):
            final.state = "CLEAR"

        return final

    def _publish_obstacles(self, snapshot: ObstacleSnapshot) -> None:
        self._final_obstacles = snapshot

        front = Bool()
        front.data = snapshot.front
        self._pub_front.publish(front)

        left = Bool()
        left.data = snapshot.left
        self._pub_left.publish(left)

        right = Bool()
        right.data = snapshot.right
        self._pub_right.publish(right)

        state = String()
        state.data = snapshot.state
        self._pub_state.publish(state)

    def _publish_at_goal(self, goal_distance_m: Optional[float], mission_elapsed_s: float) -> None:
        at_goal = goal_distance_m is not None and goal_distance_m <= self._goal_tolerance_m
        msg = Bool()
        msg.data = bool(at_goal)
        self._pub_at_goal.publish(msg)

        if at_goal and self._goal_reached_at_s is None:
            self._goal_reached_at_s = mission_elapsed_s
            self._publish_event(
                f"Goal tolerance reached at t={mission_elapsed_s:.1f}s with battery={self._battery_soc:.2f}"
            )

    def _publish_report_if_ready(self, mission_elapsed_s: float, goal_distance_m: Optional[float]) -> None:
        if self._report_published:
            return

        phases_done = all(phase.completed for phase in self._phases)
        goal_done = self._goal_reached_at_s is not None
        timed_out = mission_elapsed_s >= 180.0
        if not (goal_done or timed_out or (phases_done and mission_elapsed_s > 60.0)):
            return

        autonomy_status = "PASS"
        if self._violation_events:
            autonomy_status = "FAIL"
        elif not goal_done:
            autonomy_status = "DEGRADED"
        elif self._battery_soc < 0.2:
            autonomy_status = "DEGRADED"

        report = {
            "scenario": self._scenario["scenario"]["name"],
            "autonomy_status": autonomy_status,
            "goal_reached": goal_done,
            "goal_reached_at_s": self._goal_reached_at_s,
            "goal_distance_m": goal_distance_m,
            "battery_soc": round(self._battery_soc, 3),
            "distance_travelled_m": round(self._distance_travelled_m, 3),
            "monitor_violations": self._violation_events,
            "scenario_events": self._scenario_events,
            "active_obstacle_state": self._final_obstacles.state,
        }

        msg = String()
        msg.data = json.dumps(report, sort_keys=True)
        self._pub_report.publish(msg)
        self.get_logger().info(f"Scenario report: {msg.data}")
        self._report_published = True

    def _on_timer(self) -> None:
        mission_elapsed_s = self._mission_elapsed_s()
        goal_distance_m = self._goal_distance_m()

        for phase in self._phases:
            self._activate_phase_if_needed(phase, mission_elapsed_s, goal_distance_m)

        final_snapshot = self._apply_injections(mission_elapsed_s)
        self._publish_obstacles(final_snapshot)
        self._publish_at_goal(goal_distance_m, mission_elapsed_s)
        self._publish_report_if_ready(mission_elapsed_s, goal_distance_m)


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
