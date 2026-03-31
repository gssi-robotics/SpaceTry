#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener


def wrap_to_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def quat_to_rotmat(x: float, y: float, z: float, w: float):
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    return [
        [1.0 - 2.0*(yy + zz), 2.0*(xy - wz),       2.0*(xz + wy)],
        [2.0*(xy + wz),       1.0 - 2.0*(xx + zz), 2.0*(yz - wx)],
        [2.0*(xz - wy),       2.0*(yz + wx),       1.0 - 2.0*(xx + yy)],
    ]


def rot_apply(R, v):
    return [
        R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2],
        R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2],
        R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2],
    ]


class ObstacleDirectionNode(Node):
    """
    Subscribes:  /scan (sensor_msgs/LaserScan)
    Publishes:
      - obstacle/front (std_msgs/Bool)
      - obstacle/left  (std_msgs/Bool)
      - obstacle/right (std_msgs/Bool)
      - obstacle/state (std_msgs/String)  FRONT/LEFT/RIGHT/CLEAR
    """

    def __init__(self):
        super().__init__('obstacle_direction_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('threshold_m', 7.0)

        # sector bounds in radians (in base_frame)
        self.declare_parameter('front_min_rad', math.radians(-20.0))
        self.declare_parameter('front_max_rad', math.radians(20.0))
        self.declare_parameter('left_min_rad', math.radians(20.0))
        self.declare_parameter('left_max_rad', math.radians(75.0))
        self.declare_parameter('right_min_rad', math.radians(-75.0))
        self.declare_parameter('right_max_rad', math.radians(-20.0))

        self.declare_parameter('publish_state_string', True)
        self.declare_parameter('log_period_s', 0.5)

        scan_topic = self.get_parameter('scan_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.threshold = float(self.get_parameter('threshold_m').value)

        self.front_min = float(self.get_parameter('front_min_rad').value)
        self.front_max = float(self.get_parameter('front_max_rad').value)
        self.left_min  = float(self.get_parameter('left_min_rad').value)
        self.left_max  = float(self.get_parameter('left_max_rad').value)
        self.right_min = float(self.get_parameter('right_min_rad').value)
        self.right_max = float(self.get_parameter('right_max_rad').value)

        self.publish_state_string = bool(self.get_parameter('publish_state_string').value)
        self.log_period_s = float(self.get_parameter('log_period_s').value)
        self._last_log_time = self.get_clock().now()

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.pub_front = self.create_publisher(Bool, 'obstacle/front', 10)
        self.pub_left  = self.create_publisher(Bool, 'obstacle/left', 10)
        self.pub_right = self.create_publisher(Bool, 'obstacle/right', 10)
        self.pub_state = self.create_publisher(String, 'obstacle/state', 10) if self.publish_state_string else None

        # Subscriber
        self.create_subscription(LaserScan, scan_topic, self.on_scan, qos_profile_sensor_data)

        self.get_logger().info(
            f"Listening {scan_topic} (base_frame={self.base_frame}) thr={self.threshold:.2f}m -> "
            "publishing obstacle/front|left|right (+ obstacle/state)"
        )

    def on_scan(self, msg: LaserScan):
        scan_frame = msg.header.frame_id

        # TF: base <- scan
        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, scan_frame, rclpy.time.Time())
            q = t.transform.rotation
            R = quat_to_rotmat(q.x, q.y, q.z, q.w)
        except Exception:
            R = [[1,0,0],[0,1,0],[0,0,1]]

        min_front = float('inf')
        min_left  = float('inf')
        min_right = float('inf')
        min_any   = float('inf')
        valid = 0

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= 0.0:
                continue
            if r < msg.range_min or r > msg.range_max:
                continue

            valid += 1
            min_any = min(min_any, r)

            angle_scan = msg.angle_min + i * msg.angle_increment
            v_scan = [math.cos(angle_scan), math.sin(angle_scan), 0.0]
            v_base = rot_apply(R, v_scan)
            angle_base = wrap_to_pi(math.atan2(v_base[1], v_base[0]))

            if self.front_min < angle_base < self.front_max:
                min_front = min(min_front, r)
            elif self.left_min < angle_base < self.left_max:
                min_left = min(min_left, r)
            elif self.right_min < angle_base < self.right_max:
                min_right = min(min_right, r)

        front = False
        left = False
        right = False
        state = "CLEAR"

        sector_ranges = {
            "FRONT": min_front,
            "LEFT": min_left,
            "RIGHT": min_right,
        }
        state, state_range = min(sector_ranges.items(), key=lambda item: item[1])
        if state_range < self.threshold:
            front = (state == "FRONT")
            left = (state == "LEFT")
            right = (state == "RIGHT")
        else:
            state = "CLEAR"

        self.pub_front.publish(Bool(data=front))
        self.pub_left.publish(Bool(data=left))
        self.pub_right.publish(Bool(data=right))

        if self.pub_state is not None:
            self.pub_state.publish(String(data=state))

        now = self.get_clock().now()
        if (now - self._last_log_time).nanoseconds * 1e-9 >= self.log_period_s:
            self._last_log_time = now
            self.get_logger().info(
                f"state={state if self.pub_state is not None else 'N/A'} "
                f"scan_frame={scan_frame} valid={valid}/{len(msg.ranges)} "
                f"range=[{msg.range_min:.2f},{msg.range_max:.2f}] thr={self.threshold:.2f} "
                f"min_any={min_any:.2f} front={min_front:.2f} left={min_left:.2f} right={min_right:.2f}"
            )

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDirectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
