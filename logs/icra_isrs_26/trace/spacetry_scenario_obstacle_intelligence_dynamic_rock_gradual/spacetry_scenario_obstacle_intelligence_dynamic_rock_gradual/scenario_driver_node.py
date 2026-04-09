#!/usr/bin/env python3
import json
import math
import signal
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32


@dataclass
class Pose2D:
    stamp_s: float
    x: float
    y: float
    yaw: float


def quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def point_to_segment_metrics(
    point: Tuple[float, float],
    start: Tuple[float, float],
    end: Tuple[float, float],
) -> Tuple[float, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    denom = dx * dx + dy * dy
    if denom <= 1e-9:
        return distance(point, start), 0.0
    raw_t = ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / denom
    t = max(0.0, min(1.0, raw_t))
    proj = (start[0] + t * dx, start[1] + t * dy)
    return distance(point, proj), t


def load_yaml(path: Path) -> Dict:
    with path.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


class ScenarioDriverNode(Node):
    def __init__(self) -> None:
        super().__init__("scenario_obstacle_intelligence_dynamic_rock_gradual")

        self.declare_parameter("config_file", "")
        self.declare_parameter("contract_file", "")
        self.declare_parameter("output_root", "/ws/log/scenario_obstacle_intelligence_dynamic_rock_gradual")
        self.declare_parameter("run_stamp", "")

        self.output_root = Path(str(self.get_parameter("output_root").value))
        self.output_root.mkdir(parents=True, exist_ok=True)
        self.metrics_dir = self.output_root / "metrics"
        self.runtime_dir = self.output_root / "runtime"
        self.metrics_dir.mkdir(parents=True, exist_ok=True)
        self.runtime_dir.mkdir(parents=True, exist_ok=True)

        config_path = Path(str(self.get_parameter("config_file").value))
        contract_path = Path(str(self.get_parameter("contract_file").value))
        self.config = load_yaml(config_path)["scenario"]["ros__parameters"]
        self.contract = load_yaml(contract_path).get("scenario_contract", {})
        self.run_stamp = str(self.get_parameter("run_stamp").value)

        mission_waypoints = load_yaml(
            Path(get_package_share_directory("spacetry_mission")) / "config" / "waypoints.yaml"
        )
        waypoint_map = mission_waypoints.get("waypoints", {})

        self.start_waypoint_name = str(self.config["start_waypoint"])
        self.goal_waypoint_name = str(self.config["goal_waypoint"])
        self.start_waypoint = waypoint_map[self.start_waypoint_name]
        self.goal_waypoint = waypoint_map[self.goal_waypoint_name]
        self.start_xy = (float(self.start_waypoint["x"]), float(self.start_waypoint["y"]))
        self.goal_xy = (float(self.goal_waypoint["x"]), float(self.goal_waypoint["y"]))
        self.goal_tolerance_m = float(self.config["goal_tolerance_m"])
        self.route_length_m = distance(self.start_xy, self.goal_xy)

        obstacle_fraction = float(self.config["obstacle_route_fraction"])
        obstacle_x = self.start_xy[0] + obstacle_fraction * (self.goal_xy[0] - self.start_xy[0])
        obstacle_y = self.start_xy[1] + obstacle_fraction * (self.goal_xy[1] - self.start_xy[1])
        self.obstacle_xy = (obstacle_x, obstacle_y)
        self.obstacle_name = str(self.config["obstacle_name"])
        self.obstacle_model = str(self.config["obstacle_model"])
        self.obstacle_z = float(self.config["obstacle_z"])
        self.obstacle_yaw = float(self.config["obstacle_yaw"])
        self.trigger_progress_fraction = float(self.config["trigger_progress_fraction"])
        self.trigger_after_s = float(self.config["trigger_after_s"])
        self.timeout_s = float(self.config["scenario_timeout_s"])
        self.collision_radius_m = float(self.config["collision_radius_m"])
        self.nominal_linear_speed_mps = float(self.config["nominal_linear_speed_mps"])
        self.reaction_angular_threshold = float(self.config["reaction_angular_threshold_radps"])
        self.reaction_linear_drop_fraction = float(self.config["reaction_linear_drop_fraction"])
        self.stable_progress_window_s = float(self.config["stable_progress_window_s"])
        self.stable_progress_delta_m = float(self.config["stable_progress_delta_m"])
        self.degraded_recovery_ms = float(self.config["degraded_recovery_ms"])
        self.baseline_map_assessment = str(self.config["baseline_map_assessment"])

        self.odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.pose: Optional[Pose2D] = None
        self.first_pose_time_s: Optional[float] = None
        self.last_cmd_vel: Optional[Twist] = None
        self.last_soc: Optional[float] = None
        self.front_obstacle = False
        self.left_obstacle = False
        self.right_obstacle = False
        self.monitor_mr009_fired = False
        self.monitor_mr011_fired = False
        self.obstacle_spawned = False
        self.injection_time_s: Optional[float] = None
        self.obstacle_detection_time_s: Optional[float] = None
        self.reaction_time_s: Optional[float] = None
        self.stable_progress_time_s: Optional[float] = None
        self.goal_reached_time_s: Optional[float] = None
        self.finish_reason: Optional[str] = None
        self.path_samples: List[Pose2D] = []
        self.timeline: List[Dict] = []
        self.spawn_output: str = ""
        self.spawn_return_code: Optional[int] = None

        self.create_subscription(Odometry, "/mobile_base_controller/odom", self.on_odom, self.odom_qos)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, self.reliable_qos)
        self.create_subscription(Float32, "/battery/soc", self.on_battery_soc, self.reliable_qos)
        self.create_subscription(Bool, "/obstacle/front", self.on_front_obstacle, self.odom_qos)
        self.create_subscription(Bool, "/obstacle/left", self.on_left_obstacle, self.odom_qos)
        self.create_subscription(Bool, "/obstacle/right", self.on_right_obstacle, self.odom_qos)
        self.create_subscription(
            Bool,
            "/monitor/handlerMR_009_RETURN_TO_OUTPOST_ON_LOW_BATTERY",
            self.on_mr009,
            self.reliable_qos,
        )
        self.create_subscription(
            Bool,
            "/monitor/handlerMR_011_MAINTAIN_SPEED_WHEN_FULL_BATTERY",
            self.on_mr011,
            self.reliable_qos,
        )

        self.create_timer(0.2, self.on_timer)
        self.scenario_start_time_s = self.now_s()

        self.log_event(
            "scenario_initialized",
            {
                "goal_waypoint": self.goal_waypoint_name,
                "start_waypoint": self.start_waypoint_name,
                "obstacle_xy": [round(self.obstacle_xy[0], 3), round(self.obstacle_xy[1], 3)],
                "trigger_progress_fraction": self.trigger_progress_fraction,
                "trigger_after_s": self.trigger_after_s,
            },
        )
        self.get_logger().info(
            "Scenario ready: runtime rock will be inserted near "
            f"({self.obstacle_xy[0]:.2f}, {self.obstacle_xy[1]:.2f}) on the dock-to-science route."
        )

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def log_event(self, event: str, details: Dict) -> None:
        entry = {"time_s": round(self.now_s(), 3), "event": event, "details": details}
        self.timeline.append(entry)

    def on_odom(self, msg: Odometry) -> None:
        stamp_s = self.now_s()
        pose = Pose2D(
            stamp_s=stamp_s,
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            yaw=quat_to_yaw(msg.pose.pose.orientation),
        )
        self.pose = pose
        self.path_samples.append(pose)
        if self.first_pose_time_s is None:
            self.first_pose_time_s = stamp_s
            self.log_event("first_odom_received", {"x": round(pose.x, 3), "y": round(pose.y, 3)})

    def on_cmd_vel(self, msg: Twist) -> None:
        self.last_cmd_vel = msg

    def on_battery_soc(self, msg: Float32) -> None:
        self.last_soc = float(msg.data)

    def on_front_obstacle(self, msg: Bool) -> None:
        self.front_obstacle = bool(msg.data)

    def on_left_obstacle(self, msg: Bool) -> None:
        self.left_obstacle = bool(msg.data)

    def on_right_obstacle(self, msg: Bool) -> None:
        self.right_obstacle = bool(msg.data)

    def on_mr009(self, msg: Bool) -> None:
        if msg.data:
            self.monitor_mr009_fired = True
            self.log_event("monitor_violation", {"monitor": "MR_009_RETURN_TO_OUTPOST_ON_LOW_BATTERY"})

    def on_mr011(self, msg: Bool) -> None:
        if msg.data:
            self.monitor_mr011_fired = True
            self.log_event("monitor_violation", {"monitor": "MR_011_MAINTAIN_SPEED_WHEN_FULL_BATTERY"})

    def route_progress_fraction(self) -> float:
        if self.pose is None:
            return 0.0
        _, t = point_to_segment_metrics((self.pose.x, self.pose.y), self.start_xy, self.goal_xy)
        return t

    def goal_distance_m(self) -> Optional[float]:
        if self.pose is None:
            return None
        return distance((self.pose.x, self.pose.y), self.goal_xy)

    def spawn_obstacle(self) -> None:
        model_file = (
            Path(get_package_share_directory("spacetry_models"))
            / "models"
            / self.obstacle_model
            / "model.sdf"
        )
        cmd = [
            "ros2",
            "run",
            "ros_gz_sim",
            "create",
            "-world",
            str(self.config["world_name"]),
            "-name",
            self.obstacle_name,
            "-allow_renaming",
            "true",
            "-file",
            str(model_file),
            "-x",
            f"{self.obstacle_xy[0]:.3f}",
            "-y",
            f"{self.obstacle_xy[1]:.3f}",
            "-z",
            f"{self.obstacle_z:.3f}",
            "-Y",
            f"{self.obstacle_yaw:.3f}",
        ]
        completed = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=30,
            check=False,
        )
        self.spawn_return_code = completed.returncode
        self.spawn_output = "\n".join([completed.stdout.strip(), completed.stderr.strip()]).strip()
        if completed.returncode != 0:
            self.log_event(
                "obstacle_spawn_failed",
                {"return_code": completed.returncode, "output": self.spawn_output[-400:]},
            )
            self.finish("spawn_failed")
            return

        self.obstacle_spawned = True
        self.injection_time_s = self.now_s()
        self.log_event(
            "obstacle_injected",
            {
                "obstacle_name": self.obstacle_name,
                "obstacle_xy": [round(self.obstacle_xy[0], 3), round(self.obstacle_xy[1], 3)],
                "output": self.spawn_output[-300:],
            },
        )
        self.get_logger().info("Injected runtime obstacle on the nominal route.")

    def infer_collision(self) -> Tuple[bool, Optional[float]]:
        if not self.obstacle_spawned or self.injection_time_s is None:
            return False, None
        min_distance = None
        for pose in self.path_samples:
            if pose.stamp_s < self.injection_time_s:
                continue
            d = distance((pose.x, pose.y), self.obstacle_xy)
            min_distance = d if min_distance is None else min(min_distance, d)
        if min_distance is None:
            return False, None
        return min_distance <= self.collision_radius_m, min_distance

    def compute_post_injection_metrics(self) -> Tuple[float, float]:
        if self.injection_time_s is None:
            return 0.0, 0.0

        samples = [pose for pose in self.path_samples if pose.stamp_s >= self.injection_time_s]
        if len(samples) < 2:
            return 0.0, 0.0

        path_len = 0.0
        max_deviation = 0.0
        for current, nxt in zip(samples, samples[1:]):
            path_len += distance((current.x, current.y), (nxt.x, nxt.y))
        for sample in samples:
            deviation, _ = point_to_segment_metrics((sample.x, sample.y), self.start_xy, self.goal_xy)
            max_deviation = max(max_deviation, deviation)

        nominal_remaining = distance((samples[0].x, samples[0].y), self.goal_xy)
        detour = max(0.0, path_len - nominal_remaining)
        return detour, max_deviation

    def finish(self, reason: str) -> None:
        if self.finish_reason is not None:
            return
        self.finish_reason = reason

        collision, min_distance = self.infer_collision()
        detour_distance_m, route_deviation_m = self.compute_post_injection_metrics()
        elapsed_s = self.now_s() - self.scenario_start_time_s

        goal_reached = self.goal_reached_time_s is not None
        mission_deadline_met = goal_reached and elapsed_s <= self.timeout_s
        detection_latency_ms = None
        if self.injection_time_s is not None and self.obstacle_detection_time_s is not None:
            detection_latency_ms = (self.obstacle_detection_time_s - self.injection_time_s) * 1000.0
        adaptation_speed_ms = None
        if self.injection_time_s is not None and self.reaction_time_s is not None:
            adaptation_speed_ms = (self.reaction_time_s - self.injection_time_s) * 1000.0
        recovery_rate_ms = None
        if self.reaction_time_s is not None and self.stable_progress_time_s is not None:
            recovery_rate_ms = (self.stable_progress_time_s - self.reaction_time_s) * 1000.0

        safety_status = "PASS"
        if collision:
            safety_status = "FAIL"
        elif self.monitor_mr009_fired or self.monitor_mr011_fired:
            safety_status = "DEGRADED"

        autonomy_status = "PASS"
        if collision or not goal_reached or self.spawn_return_code not in (None, 0):
            autonomy_status = "FAIL"
        elif self.reaction_time_s is None or self.obstacle_detection_time_s is None:
            autonomy_status = "FAIL"
        elif recovery_rate_ms is None or recovery_rate_ms > self.degraded_recovery_ms:
            autonomy_status = "DEGRADED"
        elif self.last_soc is not None and self.last_soc <= 0.05:
            autonomy_status = "FAIL"

        goal_status = "PASS" if mission_deadline_met else "FAIL"

        metrics = {
            "scenario_name": str(self.config["scenario_name"]),
            "run_stamp": self.run_stamp,
            "finish_reason": reason,
            "baseline_map_assessment": self.baseline_map_assessment,
            "start_waypoint": self.start_waypoint_name,
            "goal_waypoint": self.goal_waypoint_name,
            "obstacle_xy": {"x": self.obstacle_xy[0], "y": self.obstacle_xy[1]},
            "timing": {
                "elapsed_s": elapsed_s,
                "obstacle_detection_latency_ms": detection_latency_ms,
                "adaptation_speed_ms": adaptation_speed_ms,
                "recovery_rate_ms": recovery_rate_ms,
            },
            "safety_preservation": {
                "MR_009_RETURN_TO_OUTPOST_ON_LOW_BATTERY": not self.monitor_mr009_fired,
                "MR_011_MAINTAIN_SPEED_WHEN_FULL_BATTERY": not self.monitor_mr011_fired,
                "collision_with_dynamic_obstacle": collision,
            },
            "goal_viability": {
                "science_rock_01_reached": goal_reached,
                "mission_deadline_met": mission_deadline_met,
            },
            "additional_metrics": {
                "detour_distance_m": detour_distance_m,
                "route_deviation_m": route_deviation_m,
                "minimum_obstacle_clearance_m": min_distance,
                "final_battery_soc": self.last_soc,
            },
            "outcome_assessment": {
                "goal_status": goal_status,
                "safety_status": safety_status,
                "autonomy_assessment": autonomy_status,
            },
            "spawn_result": {
                "return_code": self.spawn_return_code,
                "output_tail": self.spawn_output[-1000:],
            },
            "contract": self.contract,
        }

        metrics_path = self.metrics_dir / "scenario_obstacle_intelligence_dynamic_rock_gradual_metrics.json"
        timeline_path = self.runtime_dir / "scenario_obstacle_intelligence_dynamic_rock_gradual_timeline.json"
        report_path = self.output_root / "scenario_obstacle_intelligence_dynamic_rock_gradual_report.md"

        self.log_event("scenario_finished", {"reason": reason, "report_path": str(report_path)})

        metrics_path.write_text(json.dumps(metrics, indent=2), encoding="utf-8")
        timeline_path.write_text(json.dumps(self.timeline, indent=2), encoding="utf-8")

        report_lines = [
            "# Scenario Report",
            "",
            "## Scenario",
            f"- Scenario: `{self.config['scenario_name']}`",
            f"- Start waypoint: `{self.start_waypoint_name}`",
            f"- Goal waypoint: `{self.goal_waypoint_name}`",
            f"- Runtime obstacle pose: `({self.obstacle_xy[0]:.2f}, {self.obstacle_xy[1]:.2f}, {self.obstacle_z:.2f})`",
            f"- Baseline map assessment: {self.baseline_map_assessment}",
            "",
            "## Outcomes",
            f"- Goal status: `{goal_status}`",
            f"- Safety status: `{safety_status}`",
            f"- Autonomy assessment: `{autonomy_status}`",
            f"- Finish reason: `{reason}`",
            "",
            "## Metrics",
            f"- Obstacle detection latency (ms): `{detection_latency_ms}`",
            f"- Adaptation speed (ms): `{adaptation_speed_ms}`",
            f"- Recovery rate (ms): `{recovery_rate_ms}`",
            f"- Detour distance (m): `{detour_distance_m:.3f}`",
            f"- Route deviation (m): `{route_deviation_m:.3f}`",
            f"- Minimum obstacle clearance (m): `{min_distance}`",
            f"- Final battery SOC: `{self.last_soc}`",
            "",
            "## Safety Preservation",
            f"- `MR_009_RETURN_TO_OUTPOST_ON_LOW_BATTERY`: `{not self.monitor_mr009_fired}`",
            f"- `MR_011_MAINTAIN_SPEED_WHEN_FULL_BATTERY`: `{not self.monitor_mr011_fired}`",
            f"- `collision_with_dynamic_obstacle`: `{collision}`",
            "",
            "## Goal Viability",
            f"- `science_rock_01_reached`: `{goal_reached}`",
            f"- `mission_deadline_met`: `{mission_deadline_met}`",
            "",
            "## Observability Notes",
            "- Obstacle-detection latency is derived from the first asserted `obstacle/front|left|right` topic after injection.",
            "- Adaptation speed is derived from the first post-injection `cmd_vel` reaction that exceeds the angular threshold or drops linear speed below the configured fraction of nominal cruise speed.",
            "- Collision with the dynamic obstacle is inferred geometrically from rover odometry and obstacle center distance because the baseline stack does not expose a dedicated collision topic.",
        ]
        report_path.write_text("\n".join(report_lines) + "\n", encoding="utf-8")

        self.get_logger().info(f"Scenario finished with reason={reason}. Report written to {report_path}")
        self.destroy_node()
        rclpy.shutdown()

    def on_timer(self) -> None:
        if self.finish_reason is not None or self.pose is None:
            return

        now_s = self.now_s()
        elapsed_s = now_s - self.scenario_start_time_s
        if elapsed_s >= self.timeout_s:
            self.finish("timeout")
            return

        goal_distance = self.goal_distance_m()
        if goal_distance is not None and goal_distance <= self.goal_tolerance_m:
            if self.goal_reached_time_s is None:
                self.goal_reached_time_s = now_s
                self.log_event(
                    "goal_reached",
                    {"distance_to_goal_m": round(goal_distance, 3), "elapsed_s": round(elapsed_s, 3)},
                )
            self.finish("goal_reached")
            return

        route_progress = self.route_progress_fraction()
        if not self.obstacle_spawned and elapsed_s >= self.trigger_after_s:
            self.log_event(
                "obstacle_injection_triggered",
                {
                    "elapsed_s": round(elapsed_s, 3),
                    "route_progress_fraction": round(route_progress, 3),
                },
            )
            self.spawn_obstacle()
            return

        if not self.obstacle_spawned or self.injection_time_s is None:
            return

        if (
            self.obstacle_detection_time_s is None
            and (self.front_obstacle or self.left_obstacle or self.right_obstacle)
        ):
            self.obstacle_detection_time_s = now_s
            self.log_event(
                "obstacle_detected",
                {
                    "front": self.front_obstacle,
                    "left": self.left_obstacle,
                    "right": self.right_obstacle,
                },
            )

        if self.reaction_time_s is None and self.last_cmd_vel is not None:
            linear_x = float(self.last_cmd_vel.linear.x)
            angular_z = float(self.last_cmd_vel.angular.z)
            reaction_detected = (
                abs(angular_z) >= self.reaction_angular_threshold
                or linear_x <= (self.nominal_linear_speed_mps * self.reaction_linear_drop_fraction)
            )
            if reaction_detected:
                self.reaction_time_s = now_s
                self.log_event(
                    "motion_reaction_detected",
                    {"linear_x": round(linear_x, 3), "angular_z": round(angular_z, 3)},
                )

        if self.reaction_time_s is not None and self.stable_progress_time_s is None:
            samples = [
                pose
                for pose in self.path_samples
                if pose.stamp_s >= max(self.reaction_time_s, now_s - self.stable_progress_window_s)
            ]
            if len(samples) >= 2:
                progress_delta = distance((samples[0].x, samples[0].y), (samples[-1].x, samples[-1].y))
                if progress_delta >= self.stable_progress_delta_m and not self.front_obstacle:
                    self.stable_progress_time_s = now_s
                    self.log_event(
                        "stable_progress_detected",
                        {"progress_delta_m": round(progress_delta, 3)},
                    )


def main() -> None:
    rclpy.init()
    node = ScenarioDriverNode()

    def handle_signal(signum, _frame) -> None:
        if node.finish_reason is None:
            node.finish(f"signal_{signum}")

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.finish("interrupted")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
