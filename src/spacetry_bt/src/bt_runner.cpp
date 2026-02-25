#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <vector>

#include "spacetry_bt/nodes_ros.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("spacetry_bt_runner");

  // Params
  const auto tree_file = node->declare_parameter<std::string>("tree_file", "");
  const double tick_hz = node->declare_parameter<double>("tick_hz", 10.0);
  const double max_runtime_s =
      node->declare_parameter<double>("max_runtime_s", 0.0);  // 0 = run forever

  // Waypoints consumed by SetGoal (and read by Navigate/Align via blackboard goal)
  (void)node->declare_parameter<std::vector<double>>(
      "waypoints.science_rock", std::vector<double>{5.0, 3.0, 0.0});
  (void)node->declare_parameter<std::vector<double>>(
      "waypoints.outpost", std::vector<double>{0.0, 0.0, 0.0});

  if (tree_file.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Parameter tree_file is required.");
    rclcpp::shutdown();
    return 2;
  }

  // Provide ROS node handle to the BT nodes that need it
  spacetry_bt::SetGoal::setNode(node);
  spacetry_bt::NavigateWithAvoidance::setNode(node);
  spacetry_bt::AlignToGoal::setNode(node);
  spacetry_bt::StopAndObserve::setNode(node);
  spacetry_bt::LogMessage::setNode(node);

  // If your remaining XML still uses these generic helpers, keep them;
  // otherwise delete these two lines too.
  // spacetry_bt::Sleep::setNode(node);
  // spacetry_bt::CallEmptyService::setNode(node);

  BT::BehaviorTreeFactory factory;

  // Register only the nodes you actually ship/use
  factory.registerNodeType<spacetry_bt::SetGoal>("SetGoal");
  factory.registerNodeType<spacetry_bt::NavigateWithAvoidance>("NavigateWithAvoidance");
  factory.registerNodeType<spacetry_bt::AlignToGoal>("AlignToGoal");
  factory.registerNodeType<spacetry_bt::StopAndObserve>("StopAndObserve");
  factory.registerNodeType<spacetry_bt::LogMessage>("LogMessage");

  BT::Tree tree = factory.createTreeFromFile(tree_file);
  BT::StdCoutLogger logger(tree);

  const auto start = node->now();
  rclcpp::Rate rate(std::max(1.0, tick_hz));

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);

    const auto status = tree.tickOnce();
    if (status == BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(node->get_logger(), "Tree finished with SUCCESS");
      break;
    }
    if (status == BT::NodeStatus::FAILURE) {
      RCLCPP_WARN(node->get_logger(), "Tree returned FAILURE (continuing)");
    }

    if (max_runtime_s > 0.0 && (node->now() - start).seconds() >= max_runtime_s) {
      RCLCPP_INFO(node->get_logger(), "Max runtime reached, stopping");
      break;
    }

    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}