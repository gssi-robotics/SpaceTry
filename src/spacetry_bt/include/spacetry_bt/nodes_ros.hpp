#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/empty.hpp>

namespace spacetry_bt {

// Call std_srvs/Empty service once and return SUCCESS/FAILURE
class CallEmptyService : public BT::SyncActionNode {
public:
  CallEmptyService(const std::string& name, const BT::NodeConfiguration& cfg);
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("service") };
  }
  BT::NodeStatus tick() override;

  static void setNode(const rclcpp::Node::SharedPtr& node) { node_ = node; }
private:
  static rclcpp::Node::SharedPtr node_;
};

// Sleep for duration_s (returns RUNNING until elapsed)
class Sleep : public BT::StatefulActionNode {
public:
  Sleep(const std::string& name, const BT::NodeConfiguration& cfg);
  static BT::PortsList providedPorts() {
    return { BT::InputPort<double>("duration_s", 0.5, "Duration in seconds") };
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Time start_;
  double duration_{0.5};
  static rclcpp::Node::SharedPtr node_;
public:
  static void setNode(const rclcpp::Node::SharedPtr& node) { node_ = node; }
};

// Timed motion: start_service for duration_s, then /move_stop
class TimedMotion : public BT::StatefulActionNode {
public:
  TimedMotion(const std::string& name, const BT::NodeConfiguration& cfg);
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("start_service"),
      BT::InputPort<std::string>("stop_service", "/move_stop", "ROS name (topic/service)"),
      BT::InputPort<double>("duration_s", 5.0, "Duration in seconds")
    };
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static void setNode(const rclcpp::Node::SharedPtr& node) { node_ = node; }
private:
  static rclcpp::Node::SharedPtr node_;
  rclcpp::Time start_;
  double duration_{5.0};
  std::string start_srv_, stop_srv_;
  bool started_{false};

  bool callEmpty(const std::string& srv);
};

// Condition: obstacle too close using LaserScan min range
class ObstacleTooClose : public BT::ConditionNode {
public:
  ObstacleTooClose(const std::string& name, const BT::NodeConfiguration& cfg);
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("topic", "/scan", "ROS name (topic/service)"),
      BT::InputPort<double>("threshold_m", 1.0, "Obstacle threshold (meters)")
    };
  }
  BT::NodeStatus tick() override;

  static void setNode(const rclcpp::Node::SharedPtr& node) { node_ = node; }
private:
  static rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  std::mutex mtx_;
  double last_min_{std::numeric_limits<double>::infinity()};
  double threshold_{1.0};
  std::string topic_;
  bool subscribed_{false};

  void ensureSub();
  void cb(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};

// Random turn direction: outputs "left" or "right"
class PickRandomTurn : public BT::SyncActionNode {
public:
  PickRandomTurn(const std::string& name, const BT::NodeConfiguration& cfg);
  static BT::PortsList providedPorts() {
    return { BT::OutputPort<std::string>("dir") };
  }
  BT::NodeStatus tick() override;
};

// Random success (sample found stub): SUCCESS with probability p
class RandomSuccess : public BT::ConditionNode {
public:
  RandomSuccess(const std::string& name, const BT::NodeConfiguration& cfg);
  static BT::PortsList providedPorts() {
    return { BT::InputPort<double>("probability", 0.1, "Success probability [0..1]") };
  }
  BT::NodeStatus tick() override;
};

} // namespace spacetry_bt
