#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>

#include <limits>
#include <mutex>
#include <string>
#include <vector>

namespace spacetry_bt {

// Lightweight 2D goal used on the Blackboard
struct Goal2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

// === Mission base nodes (Groot2-friendly, ROS2 /cmd_vel) ===

// SetGoal: read a named waypoint from ROS params and write Goal2D to blackboard
class SetGoal : public BT::SyncActionNode
{
public:
  SetGoal(const std::string& name, const BT::NodeConfiguration& cfg);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("waypoint"),
      BT::OutputPort<Goal2D>("goal"),
    };
  }

  BT::NodeStatus tick() override;

  static void setNode(const rclcpp::Node::SharedPtr& node) { node_ = node; }

private:
  static rclcpp::Node::SharedPtr node_;
  bool getWaypoint(const std::string& name, Goal2D& out) const;
};

// NavigateWithAvoidance: go-to-goal using odom and simple obstacle avoidance
class NavigateWithAvoidance : public BT::StatefulActionNode
{
public:
  NavigateWithAvoidance(const std::string& name, const BT::NodeConfiguration& cfg);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<Goal2D>("goal"),

      BT::InputPort<double>("v_lin", 0.3, "Linear speed (m/s)"),
      BT::InputPort<double>("v_ang", 0.6, "Max angular speed (rad/s)"),
      BT::InputPort<double>("dist_tol", 0.6, "Goal distance tolerance (m)"),
      BT::InputPort<double>("kp_yaw", 1.5, "Yaw P gain"),
      BT::InputPort<double>("yaw_slow_deg", 25.0, "Stop forward motion above this yaw error (deg)"),

      BT::InputPort<std::string>("odom_topic", "/model/curiosity_mars_rover/odometry"),
      BT::InputPort<std::string>("scan_topic", "/scan"),
      BT::InputPort<std::string>("obstacle_front_topic", "/obstacle/front"),

      BT::InputPort<double>("obstacle_threshold_m", 1.0, "Front obstacle threshold (m)"),
      BT::InputPort<double>("odom_timeout_s", 1.0, "Fail if odom older than this"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static void setNode(const rclcpp::Node::SharedPtr& node) { node_ = node; }

private:
  static rclcpp::Node::SharedPtr node_;

  // ROS I/O
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_front_sub_;

  // State
  Goal2D goal_;
  bool have_goal_{false};

  bool have_odom_{false};
  double x_{0.0}, y_{0.0}, yaw_{0.0};
  rclcpp::Time last_odom_time_;

  bool have_scan_{false};
  rclcpp::Time last_scan_time_;
  double scan_front_min_{std::numeric_limits<double>::infinity()};
  double scan_left_min_{std::numeric_limits<double>::infinity()};
  double scan_right_min_{std::numeric_limits<double>::infinity()};

  bool have_obstacle_front_{false};
  bool obstacle_front_{false};
  rclcpp::Time last_obstacle_front_time_;

  std::mutex mtx_;
  bool avoiding_{false};
  int turn_sign_{+1};

  // Cached params
  double v_lin_{0.3};
  double v_ang_{0.6};
  double dist_tol_{0.6};
  double kp_yaw_{1.5};
  double yaw_slow_rad_{0.44};
  double obstacle_threshold_{1.0};
  double odom_timeout_s_{1.0};

  std::string odom_topic_;
  std::string scan_topic_;
  std::string obstacle_front_topic_;

  void ensureInterfaces();
  void publishStop();
  void publishCmd(double lin, double ang);

  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg);
  void scanCb(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void obstacleFrontCb(const std_msgs::msg::Bool::SharedPtr msg);
};

// AlignToGoal: rotate in place until pointing to goal
class AlignToGoal : public BT::StatefulActionNode
{
public:
  AlignToGoal(const std::string& name, const BT::NodeConfiguration& cfg);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<Goal2D>("goal"),
      BT::InputPort<double>("v_ang", 0.4, "Max angular speed (rad/s)"),
      BT::InputPort<double>("kp_yaw", 2.0, "Yaw P gain"),
      BT::InputPort<double>("yaw_tol_deg", 10.0, "Yaw tolerance (deg)"),
      BT::InputPort<std::string>("odom_topic", "/model/curiosity_mars_rover/odometry"),
      BT::InputPort<double>("odom_timeout_s", 1.0, "Fail if odom older than this"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static void setNode(const rclcpp::Node::SharedPtr& node) { node_ = node; }

private:
  static rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  Goal2D goal_;
  bool have_goal_{false};

  bool have_odom_{false};
  double x_{0.0}, y_{0.0}, yaw_{0.0};
  rclcpp::Time last_odom_time_;
  std::mutex mtx_;

  double v_ang_{0.4};
  double kp_yaw_{2.0};
  double yaw_tol_rad_{0.1745};
  double odom_timeout_s_{1.0};
  std::string odom_topic_;

  void ensureInterfaces();
  void publishStop();
  void publishCmd(double ang);

  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg);
};

// StopAndObserve: publish /cmd_vel=0 and wait N seconds (no camera)
class StopAndObserve : public BT::StatefulActionNode
{
public:
  StopAndObserve(const std::string& name, const BT::NodeConfiguration& cfg);

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<double>("seconds", 3.0, "Seconds to wait") };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static void setNode(const rclcpp::Node::SharedPtr& node) { node_ = node; }

private:
  static rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Time start_;
  double seconds_{3.0};

  void publishStop();
};

// LogMessage: print a message
class LogMessage : public BT::SyncActionNode
{
public:
  LogMessage(const std::string& name, const BT::NodeConfiguration& cfg);

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("message") };
  }

  BT::NodeStatus tick() override;

  static void setNode(const rclcpp::Node::SharedPtr& node) { node_ = node; }

private:
  static rclcpp::Node::SharedPtr node_;
};

}  // namespace spacetry_bt