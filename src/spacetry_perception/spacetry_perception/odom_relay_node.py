#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry


class OdomRelayNode(Node):
    def __init__(self):
        super().__init__('odom_relay_node')
        self.declare_parameter('input_topic', '/model/curiosity_mars_rover/odometry')
        self.declare_parameter('output_topic', '/mobile_base_controller/odom')

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.pub = self.create_publisher(Odometry, output_topic, qos)
        self.sub = self.create_subscription(Odometry, input_topic, self.on_odom, qos)

        self.get_logger().info(f"Relaying odom {input_topic} -> {output_topic}")

    def on_odom(self, msg: Odometry):
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomRelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
