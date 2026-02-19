#include "spacetry_bt/nodes_ros.hpp"

#include <chrono>
#include <random>

using namespace std::chrono_literals;

namespace spacetry_bt {

rclcpp::Node::SharedPtr CallEmptyService::node_;
rclcpp::Node::SharedPtr Sleep::node_;
rclcpp::Node::SharedPtr TimedMotion::node_;
rclcpp::Node::SharedPtr ObstacleTooClose::node_;

CallEmptyService::CallEmptyService(const std::string& name, const BT::NodeConfiguration& cfg)
: BT::SyncActionNode(name, cfg) {}

BT::NodeStatus CallEmptyService::tick() {
  if (!node_) throw BT::RuntimeError("CallEmptyService: node not set");
  auto srv = getInput<std::string>("service");
  if (!srv) throw BT::RuntimeError("CallEmptyService missing input [service]");
  auto client = node_->create_client<std_srvs::srv::Empty>(srv.value());
  if (!client->wait_for_service(2s)) {
    RCLCPP_ERROR(node_->get_logger(), "Service not available: %s", srv.value().c_str());
    return BT::NodeStatus::FAILURE;
  }
  auto req = std::make_shared<std_srvs::srv::Empty::Request>();
  auto fut = client->async_send_request(req);
  if (rclcpp::spin_until_future_complete(node_, fut, 2s) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Service call failed: %s", srv.value().c_str());
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

Sleep::Sleep(const std::string& name, const BT::NodeConfiguration& cfg)
: BT::StatefulActionNode(name, cfg) {}

BT::NodeStatus Sleep::onStart() {
  if (!node_) throw BT::RuntimeError("Sleep: node not set");
  duration_ = getInput<double>("duration_s").value_or(0.5);
  start_ = node_->now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Sleep::onRunning() {
  auto elapsed = (node_->now() - start_).seconds();
  return (elapsed >= duration_) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}

void Sleep::onHalted() {}

TimedMotion::TimedMotion(const std::string& name, const BT::NodeConfiguration& cfg)
: BT::StatefulActionNode(name, cfg) {}

bool TimedMotion::callEmpty(const std::string& srv) {
  auto client = node_->create_client<std_srvs::srv::Empty>(srv);
  if (!client->wait_for_service(2s)) {
    RCLCPP_ERROR(node_->get_logger(), "Service not available: %s", srv.c_str());
    return false;
  }
  auto req = std::make_shared<std_srvs::srv::Empty::Request>();
  auto fut = client->async_send_request(req);
  return rclcpp::spin_until_future_complete(node_, fut, 2s) == rclcpp::FutureReturnCode::SUCCESS;
}

BT::NodeStatus TimedMotion::onStart() {
  if (!node_) throw BT::RuntimeError("TimedMotion: node not set");
  start_srv_ = getInput<std::string>("start_service").value_or("");
  stop_srv_  = getInput<std::string>("stop_service").value_or("/move_stop");
  duration_  = getInput<double>("duration_s").value_or(5.0);

  if (start_srv_.empty()) throw BT::RuntimeError("TimedMotion missing [start_service]");

  started_ = callEmpty(start_srv_);
  if (!started_) return BT::NodeStatus::FAILURE;

  start_ = node_->now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TimedMotion::onRunning() {
  auto elapsed = (node_->now() - start_).seconds();
  if (elapsed < duration_) return BT::NodeStatus::RUNNING;

  (void)callEmpty(stop_srv_);
  return BT::NodeStatus::SUCCESS;
}

void TimedMotion::onHalted() {
  if (node_ && !stop_srv_.empty()) (void)callEmpty(stop_srv_);
}

ObstacleTooClose::ObstacleTooClose(const std::string& name, const BT::NodeConfiguration& cfg)
: BT::ConditionNode(name, cfg) {}

void ObstacleTooClose::ensureSub() {
  if (subscribed_) return;
  if (!node_) throw BT::RuntimeError("ObstacleTooClose: node not set");
  topic_ = getInput<std::string>("topic").value_or("/scan");
  threshold_ = getInput<double>("threshold_m").value_or(1.0);

  sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
    topic_, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::LaserScan::SharedPtr msg){ cb(msg); }
  );
  subscribed_ = true;
}

void ObstacleTooClose::cb(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  double m = std::numeric_limits<double>::infinity();
  for (auto r : msg->ranges) {
    if (std::isfinite(r)) m = std::min<double>(m, r);
  }
  std::lock_guard<std::mutex> lk(mtx_);
  last_min_ = m;
}

BT::NodeStatus ObstacleTooClose::tick() {
  ensureSub();
  double m;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    m = last_min_;
  }
  return (m < threshold_) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

PickRandomTurn::PickRandomTurn(const std::string& name, const BT::NodeConfig& config)
: BT::SyncActionNode(name, config)
{}

BT::NodeStatus PickRandomTurn::tick() {
  static thread_local std::mt19937 rng{std::random_device{}()};
  std::uniform_int_distribution<int> d(0, 1);
  setOutput("dir", d(rng) == 0 ? std::string("left") : std::string("right"));
  return BT::NodeStatus::SUCCESS;
}

RandomSuccess::RandomSuccess(const std::string& name, const BT::NodeConfig& config)
: BT::ConditionNode(name, config)
{}

BT::NodeStatus RandomSuccess::tick() {
  double p = getInput<double>("probability").value_or(0.1);
  p = std::max(0.0, std::min(1.0, p));
  static thread_local std::mt19937 rng{std::random_device{}()};
  std::uniform_real_distribution<double> u(0.0, 1.0);
  return (u(rng) < p) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

} // namespace spacetry_bt
