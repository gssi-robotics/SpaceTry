#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math
import time

class NavigateToWaypoint(Node):
    def __init__(self, waypoint):
        super().__init__('navigate_to_waypoint')
        self.waypoint = waypoint  # (x, y)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/model/curiosity_mars_rover/odometry', self.odom_callback, 10)
        self.arrived = False

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        dx = self.waypoint[0] - x
        dy = self.waypoint[1] - y
        dist = math.hypot(dx, dy)
        if dist < 0.5:
            self.arrived = True
            self.cmd_pub.publish(Twist())  # Stop
        else:
            twist = Twist()
            twist.linear.x = 0.3
            self.cmd_pub.publish(twist)

    def run(self):
        while rclpy.ok() and not self.arrived:
            rclpy.spin_once(self)
        self.get_logger().info('Arrived at waypoint')

class ObstacleFront(Node):
    def __init__(self):
        super().__init__('obstacle_front')
        self.obstacle = False
        self.sub = self.create_subscription(Bool, '/obstacle/front', self.cb, 10)

    def cb(self, msg):
        self.obstacle = msg.data

    def run(self):
        rclpy.spin_once(self)
        return self.obstacle

class AvoidObstacle(Node):
    def __init__(self):
        super().__init__('avoid_obstacle')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def run(self):
        twist = Twist()
        twist.angular.z = 0.5  # Turn
        self.cmd_pub.publish(twist)
        time.sleep(2)
        self.cmd_pub.publish(Twist())  # Stop

class AlignWithGoal(Node):
    def __init__(self, waypoint):
        super().__init__('align_with_goal')
        self.waypoint = waypoint
        self.odom_sub = self.create_subscription(Odometry, '/model/curiosity_mars_rover/odometry', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.aligned = False

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        dx = self.waypoint[0] - x
        dy = self.waypoint[1] - y
        goal_yaw = math.atan2(dy, dx)
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        if abs(goal_yaw - yaw) < 0.1:
            self.aligned = True
            self.cmd_pub.publish(Twist())
        else:
            twist = Twist()
            twist.angular.z = 0.3 if (goal_yaw - yaw) > 0 else -0.3
            self.cmd_pub.publish(twist)

    def run(self):
        while rclpy.ok() and not self.aligned:
            rclpy.spin_once(self)
        self.get_logger().info('Aligned with goal')

class StopAndObserve(Node):
    def __init__(self, duration_s=5.0):
        super().__init__('stop_and_observe')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.duration_s = duration_s

    def run(self):
        self.cmd_pub.publish(Twist())
        self.get_logger().info('Observing...')
        time.sleep(self.duration_s)

class LogMessage(Node):
    def __init__(self, message):
        super().__init__('log_message')
        self.message = message

    def run(self):
        self.get_logger().info(self.message)

# Example usage:
# rclpy.init()
# nav = NavigateToWaypoint((5.0, 3.0))
# nav.run()
# obs = ObstacleFront()
# if obs.run():
#     AvoidObstacle().run()
# align = AlignWithGoal((5.0, 3.0))
# align.run()
# StopAndObserve(5.0).run()
# LogMessage('Science rock reached').run()
# rclpy.shutdown()
