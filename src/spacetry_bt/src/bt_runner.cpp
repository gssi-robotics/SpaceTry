#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <rclcpp/rclcpp.hpp>

#include "spacetry_bt/nodes_ros.hpp"

static std::string choose_turn_service(const std::string& dir) {
  return (dir == "right") ? "/turn_right" : "/turn_left";
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("spacetry_bt_runner");

  // Params
  const auto tree_file = node->declare_parameter<std::string>("tree_file", "");
  const double tick_hz = node->declare_parameter<double>("tick_hz", 10.0);
  const double max_runtime_s = node->declare_parameter<double>("max_runtime_s", 0.0); // 0=run forever

  if (tree_file.empty()) {
    RCLCPP_ERROR(node->get_logger(), "Parameter tree_file is required.");
    return 2;
  }

  // Register ROS-backed nodes
  spacetry_bt::CallEmptyService::setNode(node);
  spacetry_bt::Sleep::setNode(node);
  spacetry_bt::TimedMotion::setNode(node);
  spacetry_bt::ObstacleTooClose::setNode(node);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<spacetry_bt::CallEmptyService>("CallEmptyService");
  factory.registerNodeType<spacetry_bt::Sleep>("Sleep");
  factory.registerNodeType<spacetry_bt::TimedMotion>("TimedMotion");
  factory.registerNodeType<spacetry_bt::ObstacleTooClose>("ObstacleTooClose");
  factory.registerNodeType<spacetry_bt::PickRandomTurn>("PickRandomTurn");
  factory.registerNodeType<spacetry_bt::RandomSuccess>("RandomSuccess");

  // Small helper: map dir -> turn service via a script-like approach
  factory.registerSimpleAction("SelectTurnService",
    [&](BT::TreeNode& self) -> BT::NodeStatus {
      auto dir = self.getInput<std::string>("dir");
      if (!dir) return BT::NodeStatus::FAILURE;
      self.setOutput("service", choose_turn_service(dir.value()));
      return BT::NodeStatus::SUCCESS;
    },
    { BT::InputPort<std::string>("dir"), BT::OutputPort<std::string>("service") }
  );

  BT::Tree tree = factory.createTreeFromFile(tree_file);
  BT::StdCoutLogger logger(tree);

  const auto start = node->now();
  rclcpp::Rate rate(std::max(1.0, tick_hz));

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);

    auto status = tree.tickOnce();
    if (status == BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(node->get_logger(), "Tree finished with SUCCESS");
      break;
    }
    if (status == BT::NodeStatus::FAILURE) {
      RCLCPP_WARN(node->get_logger(), "Tree returned FAILURE (continuing)");
    }

    if (max_runtime_s > 0.0) {
      if ((node->now() - start).seconds() >= max_runtime_s) {
        RCLCPP_INFO(node->get_logger(), "Max runtime reached, stopping");
        break;
      }
    }
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
