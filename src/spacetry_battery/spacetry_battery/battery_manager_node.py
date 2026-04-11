#!/usr/bin/env python3
import math
import fnmatch
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, JointState
from std_msgs.msg import Bool, Float32


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class BatteryManager(Node):
    """
    Standalone battery manager:
      - Drains based on cmd_vel and joint motion
      - Recharges slowly when idle and faster near outpost
      - Publishes BatteryState + SOC convenience topics
    """

    def __init__(self) -> None:
        super().__init__("battery_manager")

        # Parameters
        self.declare_parameter("capacity_wh", 120.0)
        self.declare_parameter("initial_soc", 1.0)
        self.declare_parameter("update_rate_hz", 10.0)

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("odom_topic", "/model/curiosity_mars_rover/odometry")

        self.declare_parameter("battery_state_topic", "/battery_state")
        self.declare_parameter("soc_topic", "/battery/soc")
        self.declare_parameter("near_outpost_topic", "/battery/near_outpost")

        self.declare_parameter("outpost_x", 0.0)
        self.declare_parameter("outpost_y", 0.0)
        self.declare_parameter("outpost_radius_m", 2.0)

        self.declare_parameter("vel_eps", 0.01)
        self.declare_parameter("joint_vel_eps", 0.01)
        self.declare_parameter("stale_timeout_s", 0.75)

        self.declare_parameter("arm_joint_patterns", ["*arm*"])
        self.declare_parameter("mast_joint_patterns", ["*mast*"])

        self.declare_parameter("drain_base_w", 4.0)
        self.declare_parameter("drain_move_w_per_mps", 55.0)
        self.declare_parameter("drain_turn_w_per_radps", 25.0)
        self.declare_parameter("drain_arm_w", 30.0)
        self.declare_parameter("drain_mast_w", 18.0)

        self.declare_parameter("idle_recharge_w", 0.6)
        self.declare_parameter("outpost_recharge_w", 90.0)

        # State
        self.capacity_wh = float(self.get_parameter("capacity_wh").value)
        init_soc = clamp(float(self.get_parameter("initial_soc").value), 0.0, 1.0)
        self.energy_wh = init_soc * self.capacity_wh

        self.last_update_time: Time = self.get_clock().now()

        self.last_twist: Optional[Twist] = None
        self.last_twist_time: Optional[Time] = None

        self.last_odom_xy: Optional[Tuple[float, float]] = None
        self.last_odom_time: Optional[Time] = None

        self.last_joint: Optional[JointState] = None
        self.last_joint_time: Optional[Time] = None
        self.prev_joint_pos: Dict[str, float] = {}
        self.prev_joint_time: Optional[Time] = None

        # I/O
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.create_subscription(Twist, cmd_vel_topic, self.on_twist, 10)
        self.create_subscription(JointState, joint_states_topic, self.on_joint, 10)
        self.create_subscription(Odometry, odom_topic, self.on_odom, odom_qos)

        self.pub_battery = self.create_publisher(
            BatteryState, str(self.get_parameter("battery_state_topic").value), 10
        )
        self.pub_soc = self.create_publisher(
            Float32, str(self.get_parameter("soc_topic").value), 10
        )
        self.pub_near = self.create_publisher(
            Bool, str(self.get_parameter("near_outpost_topic").value), 10
        )

        rate_hz = max(1.0, float(self.get_parameter("update_rate_hz").value))
        self.create_timer(1.0 / rate_hz, self.on_timer)

        self.get_logger().info(
            f"BatteryManager: capacity={self.capacity_wh:.1f}Wh init={init_soc*100:.0f}% "
            f"cmd_vel={cmd_vel_topic} joint_states={joint_states_topic} odom={odom_topic}"
        )

    def on_twist(self, msg: Twist) -> None:
        self.last_twist = msg
        self.last_twist_time = self.get_clock().now()

    def on_odom(self, msg: Odometry) -> None:
        self.last_odom_xy = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
        self.last_odom_time = self.get_clock().now()

    def on_joint(self, msg: JointState) -> None:
        self.last_joint = msg
        self.last_joint_time = self.get_clock().now()

    def is_stale(self, t: Optional[Time], now: Time) -> bool:
        if t is None:
            return True
        timeout_s = float(self.get_parameter("stale_timeout_s").value)
        age = (now - t).nanoseconds * 1e-9
        return age > timeout_s

    def moving_activity(self, now: Time) -> Tuple[float, float, bool]:
        vel_eps = float(self.get_parameter("vel_eps").value)
        if self.last_twist is None or self.is_stale(self.last_twist_time, now):
            return (0.0, 0.0, False)
        v = abs(float(self.last_twist.linear.x)) + abs(float(self.last_twist.linear.y))
        w = abs(float(self.last_twist.angular.z))
        return (v, w, (v > vel_eps) or (w > vel_eps))

    def joint_activity(self, now: Time) -> Tuple[bool, bool]:
        if self.last_joint is None or self.is_stale(self.last_joint_time, now):
            return (False, False)

        vel_eps = float(self.get_parameter("joint_vel_eps").value)
        arm_patterns: List[str] = list(self.get_parameter("arm_joint_patterns").value)
        mast_patterns: List[str] = list(self.get_parameter("mast_joint_patterns").value)

        msg = self.last_joint
        names = list(msg.name)
        speeds: Dict[str, float] = {}

        if msg.velocity and len(msg.velocity) == len(names):
            for n, v in zip(names, msg.velocity):
                speeds[n] = abs(float(v))
        elif msg.position and len(msg.position) == len(names):
            if self.prev_joint_time is not None:
                dt = (now - self.prev_joint_time).nanoseconds * 1e-9
                if dt > 1e-6:
                    for n, p in zip(names, msg.position):
                        p = float(p)
                        if n in self.prev_joint_pos:
                            speeds[n] = abs((p - self.prev_joint_pos[n]) / dt)
                        self.prev_joint_pos[n] = p
            else:
                for n, p in zip(names, msg.position):
                    self.prev_joint_pos[n] = float(p)
            self.prev_joint_time = now

        def matches_any(name: str, patterns: List[str]) -> bool:
            return any(fnmatch.fnmatchcase(name, pat) for pat in patterns)

        arm_active = any(matches_any(n, arm_patterns) and speeds.get(n, 0.0) > vel_eps for n in names)
        mast_active = any(matches_any(n, mast_patterns) and speeds.get(n, 0.0) > vel_eps for n in names)
        return (arm_active, mast_active)

    def near_outpost(self, now: Time) -> bool:
        if self.last_odom_xy is None or self.is_stale(self.last_odom_time, now):
            return False
        ox = float(self.get_parameter("outpost_x").value)
        oy = float(self.get_parameter("outpost_y").value)
        r = float(self.get_parameter("outpost_radius_m").value)
        x, y = self.last_odom_xy
        return math.hypot(x - ox, y - oy) <= r

    def on_timer(self) -> None:
        now = self.get_clock().now()
        dt_s = (now - self.last_update_time).nanoseconds * 1e-9
        if dt_s <= 0.0:
            return
        dt_s = min(dt_s, 0.5)
        self.last_update_time = now

        v, w, is_moving = self.moving_activity(now)
        arm_active, mast_active = self.joint_activity(now)
        is_near = self.near_outpost(now)

        drain_w = float(self.get_parameter("drain_base_w").value)
        drain_w += float(self.get_parameter("drain_move_w_per_mps").value) * v
        drain_w += float(self.get_parameter("drain_turn_w_per_radps").value) * w
        drain_w += float(self.get_parameter("drain_arm_w").value) if arm_active else 0.0
        drain_w += float(self.get_parameter("drain_mast_w").value) if mast_active else 0.0

        stopped = (not is_moving) and (not arm_active) and (not mast_active)

        recharge_w = 0.0
        if stopped:
            recharge_w += float(self.get_parameter("idle_recharge_w").value)
        if is_near:
            recharge_w += float(self.get_parameter("outpost_recharge_w").value)

        net_w = recharge_w - drain_w
        self.energy_wh = clamp(self.energy_wh + (net_w * dt_s) / 3600.0, 0.0, self.capacity_wh)
        soc = 0.0 if self.capacity_wh <= 1e-9 else clamp(self.energy_wh / self.capacity_wh, 0.0, 1.0)

        bs = BatteryState()
        bs.header.stamp = now.to_msg()
        bs.present = True
        bs.percentage = float(soc)
        if recharge_w > drain_w + 1e-6:
            bs.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif self.energy_wh <= 1e-6:
            bs.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
        else:
            bs.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

        self.pub_battery.publish(bs)

        s = Float32()
        s.data = float(soc)
        self.pub_soc.publish(s)

        b = Bool()
        b.data = bool(is_near)
        self.pub_near.publish(b)


def main() -> None:
    rclpy.init()
    node = BatteryManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
