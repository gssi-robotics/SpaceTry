#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/condition_node.h>
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

struct Goal2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

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

class NavigateWithAvoidance : public BT::StatefulActionNode
{
public:
  NavigateWithAvoidance(const std::string& name, const BT::NodeConfiguration& cfg);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<Goal2D>("goal"),

      BT::InputPort<double>("v_lin", 1.5, "Desired linear speed (m/s)"),
      BT::InputPort<double>("min_lin", 0.2, "Minimum linear speed while RUNNING (m/s)"),
      BT::InputPort<double>("v_ang", 1.2, "Max angular speed (rad/s)"),

      BT::InputPort<double>("dist_tol", 0.6, "Goal distance tolerance (m)"),
      BT::InputPort<double>("kp_yaw", 1.5, "Yaw P gain"),
      BT::InputPort<double>("yaw_slow_deg", 25.0, "Start scaling down lin above this yaw error (deg)"),

      BT::InputPort<std::string>("odom_topic", "/mobile_base_controller/odom"),
      BT::InputPort<std::string>("scan_topic", "/scan"),
      BT::InputPort<std::string>("obstacle_front_topic", "/obstacle/front"),
      BT::InputPort<std::string>("obstacle_left_topic", "/obstacle/left"),
      BT::InputPort<std::string>("obstacle_right_topic", "/obstacle/right"),

      BT::InputPort<double>("obstacle_threshold_m", 1.0, "Front obstacle threshold (m)"),
      BT::InputPort<double>("odom_timeout_s", 3.0, "Fail if odom older than this"),
      BT::InputPort<double>("reverse_speed", 0.6, "Reverse speed during avoidance (m/s)"),
      BT::InputPort<double>("reverse_seconds", 1.0, "Reverse time when boxed in (s)"),
      BT::InputPort<double>("avoid_arc_seconds", 1.2, "Short arc duration after clearing the obstacle (s)"),
      BT::InputPort<double>("memory_seconds", 3.0, "How long to remember the last blocked direction (s)"),
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
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_front_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_left_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_right_sub_;

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
  bool have_obstacle_left_{false};
  bool obstacle_left_{false};
  rclcpp::Time last_obstacle_left_time_;
  bool have_obstacle_right_{false};
  bool obstacle_right_{false};
  rclcpp::Time last_obstacle_right_time_;

  std::mutex mtx_;
  enum class AvoidancePhase
  {
    None,
    Reverse,
    Turn,
    Arc
  };
  AvoidancePhase avoidance_phase_{AvoidancePhase::None};
  int turn_sign_{+1};
  rclcpp::Time phase_start_time_;
  rclcpp::Time last_turn_memory_time_;
  int last_turn_memory_sign_{0};

  double v_lin_{1.5};
  double min_lin_{0.2};
  double v_ang_{1.2};
  double dist_tol_{0.6};
  double kp_yaw_{1.5};
  double yaw_slow_rad_{0.44};
  double obstacle_threshold_{1.0};
  double odom_timeout_s_{3.0};
  double reverse_speed_{0.6};
  double reverse_seconds_{1.0};
  double avoid_arc_seconds_{1.2};
  double memory_seconds_{3.0};

  std::string odom_topic_;
  std::string scan_topic_;
  std::string obstacle_front_topic_;
  std::string obstacle_left_topic_;
  std::string obstacle_right_topic_;

  void ensureInterfaces();
  void publishStop();
  void publishCmd(double lin, double ang);

  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg);
  void scanCb(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void obstacleFrontCb(const std_msgs::msg::Bool::SharedPtr msg);
  void obstacleLeftCb(const std_msgs::msg::Bool::SharedPtr msg);
  void obstacleRightCb(const std_msgs::msg::Bool::SharedPtr msg);
};

class GoalReached : public BT::ConditionNode
{
public:
  GoalReached(const std::string& name, const BT::NodeConfiguration& cfg);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<Goal2D>("goal"),
      BT::InputPort<double>("dist_tol", 0.6, "Goal distance tolerance (m)"),
      BT::InputPort<std::string>("odom_topic", "/mobile_base_controller/odom"),
      BT::InputPort<double>("odom_timeout_s", 3.0, "Fail if odom older than this"),
    };
  }

  BT::NodeStatus tick() override;

  static void setNode(const rclcpp::Node::SharedPtr& node) { node_ = node; }

private:
  static rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::mutex mtx_;
  bool have_odom_{false};
  double x_{0.0};
  double y_{0.0};
  rclcpp::Time last_odom_time_;
  std::string odom_topic_;
  double odom_timeout_s_{3.0};

  void ensureInterfaces();
  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg);
};

class ObstacleInDirection : public BT::ConditionNode
{
public:
  ObstacleInDirection(const std::string& name, const BT::NodeConfiguration& cfg);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("direction", "front", "Obstacle direction to monitor"),
      BT::InputPort<std::string>("obstacle_front_topic", "/obstacle/front"),
      BT::InputPort<std::string>("obstacle_left_topic", "/obstacle/left"),
      BT::InputPort<std::string>("obstacle_right_topic", "/obstacle/right"),
      BT::InputPort<std::string>("scan_topic", "/scan"),
      BT::InputPort<double>("obstacle_threshold_m", 9.0, "Obstacle threshold when classifying directly from scan"),
      BT::InputPort<double>("freshness_s", 1.0, "Topic age before considering obstacle state stale"),
    };
  }

  BT::NodeStatus tick() override;

  static void setNode(const rclcpp::Node::SharedPtr& node) { node_ = node; }

private:
  static rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr front_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr left_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr right_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  std::mutex mtx_;
  bool front_{false};
  bool left_{false};
  bool right_{false};
  bool have_front_{false};
  bool have_left_{false};
  bool have_right_{false};
  bool have_scan_{false};
  rclcpp::Time front_time_;
  rclcpp::Time left_time_;
  rclcpp::Time right_time_;
  rclcpp::Time scan_time_;
  double scan_front_min_{std::numeric_limits<double>::infinity()};
  double scan_left_min_{std::numeric_limits<double>::infinity()};
  double scan_right_min_{std::numeric_limits<double>::infinity()};
  std::string obstacle_front_topic_;
  std::string obstacle_left_topic_;
  std::string obstacle_right_topic_;
  std::string scan_topic_;
  double obstacle_threshold_{9.0};
  double freshness_s_{1.0};

  void ensureInterfaces();
  void frontCb(const std_msgs::msg::Bool::SharedPtr msg);
  void leftCb(const std_msgs::msg::Bool::SharedPtr msg);
  void rightCb(const std_msgs::msg::Bool::SharedPtr msg);
  void scanCb(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};

class SelectAvoidanceDirection : public BT::SyncActionNode
{
public:
  SelectAvoidanceDirection(const std::string& name, const BT::NodeConfiguration& cfg);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("obstacle_front_topic", "/obstacle/front"),
      BT::InputPort<std::string>("obstacle_left_topic", "/obstacle/left"),
      BT::InputPort<std::string>("obstacle_right_topic", "/obstacle/right"),
      BT::InputPort<std::string>("scan_topic", "/scan"),
      BT::InputPort<double>("obstacle_threshold_m", 9.0, "Obstacle threshold when classifying directly from scan"),
      BT::InputPort<double>("memory_seconds", 3.0, "How long to avoid repeating the same blocked maneuver"),
      BT::InputPort<double>("freshness_s", 1.0, "Topic age before considering obstacle state stale"),
      BT::OutputPort<std::string>("turn_direction"),
      BT::OutputPort<bool>("reverse_first"),
    };
  }

  BT::NodeStatus tick() override;

  static void setNode(const rclcpp::Node::SharedPtr& node) { node_ = node; }

private:
  static rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr front_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr left_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr right_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  std::mutex mtx_;
  bool front_{false};
  bool left_{false};
  bool right_{false};
  bool have_front_{false};
  bool have_left_{false};
  bool have_right_{false};
  bool have_scan_{false};
  rclcpp::Time front_time_;
  rclcpp::Time left_time_;
  rclcpp::Time right_time_;
  rclcpp::Time scan_time_;
  double scan_front_min_{std::numeric_limits<double>::infinity()};
  double scan_left_min_{std::numeric_limits<double>::infinity()};
  double scan_right_min_{std::numeric_limits<double>::infinity()};
  std::string obstacle_front_topic_;
  std::string obstacle_left_topic_;
  std::string obstacle_right_topic_;
  std::string scan_topic_;
  double obstacle_threshold_{9.0};
  double freshness_s_{1.0};
  double memory_seconds_{3.0};
  std::string last_direction_{"left"};
  bool have_last_direction_{false};
  rclcpp::Time last_direction_time_;

  void ensureInterfaces();
  void frontCb(const std_msgs::msg::Bool::SharedPtr msg);
  void leftCb(const std_msgs::msg::Bool::SharedPtr msg);
  void rightCb(const std_msgs::msg::Bool::SharedPtr msg);
  void scanCb(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};

class DriveTowardGoal : public BT::StatefulActionNode
{
public:
  DriveTowardGoal(const std::string& name, const BT::NodeConfiguration& cfg);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<Goal2D>("goal"),
      BT::InputPort<double>("v_lin", 1.5, "Desired linear speed (m/s)"),
      BT::InputPort<double>("min_lin", 0.2, "Minimum linear speed while RUNNING (m/s)"),
      BT::InputPort<double>("v_ang", 1.2, "Max angular speed (rad/s)"),
      BT::InputPort<double>("dist_tol", 0.6, "Goal distance tolerance (m)"),
      BT::InputPort<double>("kp_yaw", 1.5, "Yaw P gain"),
      BT::InputPort<double>("yaw_slow_deg", 25.0, "Start scaling down lin above this yaw error (deg)"),
      BT::InputPort<std::string>("odom_topic", "/mobile_base_controller/odom"),
      BT::InputPort<double>("odom_timeout_s", 3.0, "Fail if odom older than this"),
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
  std::mutex mtx_;
  Goal2D goal_;
  bool have_goal_{false};
  bool have_odom_{false};
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  rclcpp::Time last_odom_time_;
  double v_lin_{1.5};
  double min_lin_{0.2};
  double v_ang_{1.2};
  double dist_tol_{0.6};
  double kp_yaw_{1.5};
  double yaw_slow_rad_{0.44};
  double odom_timeout_s_{3.0};
  std::string odom_topic_;

  void ensureInterfaces();
  void publishStop();
  void publishCmd(double lin, double ang);
  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg);
};

class AvoidObstacle : public BT::StatefulActionNode
{
public:
  AvoidObstacle(const std::string& name, const BT::NodeConfiguration& cfg);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("turn_direction", "left", "Direction to use while avoiding"),
      BT::InputPort<bool>("reverse_first", false, "Reverse before turning when boxed in"),
      BT::InputPort<std::string>("scan_topic", "/scan"),
      BT::InputPort<std::string>("obstacle_front_topic", "/obstacle/front"),
      BT::InputPort<std::string>("obstacle_left_topic", "/obstacle/left"),
      BT::InputPort<std::string>("obstacle_right_topic", "/obstacle/right"),
      BT::InputPort<double>("obstacle_threshold_m", 1.0, "Front obstacle threshold (m)"),
      BT::InputPort<double>("reverse_speed", 0.6, "Reverse speed during avoidance (m/s)"),
      BT::InputPort<double>("reverse_seconds", 1.0, "Reverse time when boxed in (s)"),
      BT::InputPort<double>("turn_speed", 1.2, "Angular turn speed while clearing obstacle"),
      BT::InputPort<double>("turn_timeout_s", 2.5, "Maximum time to spend turning in place"),
      BT::InputPort<double>("arc_speed", 0.7, "Forward speed while arcing around obstacle"),
      BT::InputPort<double>("arc_seconds", 1.4, "How long to keep the avoidance arc active"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static void setNode(const rclcpp::Node::SharedPtr& node) { node_ = node; }

private:
  static rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr front_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr left_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr right_sub_;
  std::mutex mtx_;
  enum class Phase
  {
    Reverse,
    Turn,
    Arc
  };
  Phase phase_{Phase::Turn};
  rclcpp::Time phase_start_time_;
  std::string turn_direction_{"left"};
  bool reverse_first_{false};
  bool have_scan_{false};
  double scan_front_min_{std::numeric_limits<double>::infinity()};
  double scan_left_min_{std::numeric_limits<double>::infinity()};
  double scan_right_min_{std::numeric_limits<double>::infinity()};
  rclcpp::Time last_scan_time_;
  bool front_{false};
  bool left_{false};
  bool right_{false};
  bool have_front_{false};
  bool have_left_{false};
  bool have_right_{false};
  rclcpp::Time front_time_;
  rclcpp::Time left_time_;
  rclcpp::Time right_time_;
  std::string scan_topic_;
  std::string obstacle_front_topic_;
  std::string obstacle_left_topic_;
  std::string obstacle_right_topic_;
  double obstacle_threshold_{1.0};
  double reverse_speed_{0.6};
  double reverse_seconds_{1.0};
  double turn_speed_{1.2};
  double turn_timeout_s_{2.5};
  double arc_speed_{0.7};
  double arc_seconds_{1.4};

  void ensureInterfaces();
  void publishStop();
  void publishCmd(double lin, double ang);
  void scanCb(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void frontCb(const std_msgs::msg::Bool::SharedPtr msg);
  void leftCb(const std_msgs::msg::Bool::SharedPtr msg);
  void rightCb(const std_msgs::msg::Bool::SharedPtr msg);
};

class KeepRunning : public BT::StatefulActionNode
{
public:
  KeepRunning(const std::string& name, const BT::NodeConfiguration& cfg);

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};

class AlignToGoal : public BT::StatefulActionNode
{
public:
  AlignToGoal(const std::string& name, const BT::NodeConfiguration& cfg);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<Goal2D>("goal"),

      BT::InputPort<double>("v_lin", 2.0, "Desired linear speed while aligning (m/s)"),
      BT::InputPort<double>("min_lin", 2.0, "Minimum linear speed while RUNNING (m/s)"),

      BT::InputPort<double>("v_ang", 0.4, "Max angular speed (rad/s)"),
      BT::InputPort<double>("kp_yaw", 2.0, "Yaw P gain"),
      BT::InputPort<double>("yaw_tol_deg", 10.0, "Yaw tolerance (deg)"),
      BT::InputPort<std::string>("odom_topic", "/mobile_base_controller/odom"),
      BT::InputPort<double>("odom_timeout_s", 3.0, "Fail if odom older than this"),
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

  double v_lin_{2.0};
  double min_lin_{2.0};
  double v_ang_{0.4};
  double kp_yaw_{2.0};
  double yaw_tol_rad_{0.1745};
  double odom_timeout_s_{3.0};
  std::string odom_topic_;

  void ensureInterfaces();
  void publishStop();
  void publishCmd(double lin, double ang);
  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg);
};

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
