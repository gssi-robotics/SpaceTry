#include "spacetry_bt/nodes_ros.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace spacetry_bt {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;
constexpr double kFrontSectorMin = -20.0 * kDeg;
constexpr double kFrontSectorMax = +20.0 * kDeg;
constexpr double kLeftSectorMin = +20.0 * kDeg;
constexpr double kLeftSectorMax = +75.0 * kDeg;
constexpr double kRightSectorMin = -75.0 * kDeg;
constexpr double kRightSectorMax = -20.0 * kDeg;

double clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

double wrapPi(double a)
{
  while (a > kPi) a -= 2.0 * kPi;
  while (a < -kPi) a += 2.0 * kPi;
  return a;
}

double yawFromQuat(const geometry_msgs::msg::Quaternion& q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double sectorMin(const sensor_msgs::msg::LaserScan& scan, double a0, double a1)
{
  if (scan.ranges.empty() || scan.angle_increment == 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  if (a1 < a0) {
    std::swap(a0, a1);
  }

  const auto clampIndex = [&](int i) {
    return std::max(0, std::min(i, static_cast<int>(scan.ranges.size()) - 1));
  };

  const int i0 =
      clampIndex(static_cast<int>(std::round((a0 - scan.angle_min) / scan.angle_increment)));
  const int i1 =
      clampIndex(static_cast<int>(std::round((a1 - scan.angle_min) / scan.angle_increment)));

  const float rmin = scan.range_min;
  const float rmax = scan.range_max;

  double m = std::numeric_limits<double>::infinity();
  for (int i = std::min(i0, i1); i <= std::max(i0, i1); ++i) {
    const float r = scan.ranges[static_cast<size_t>(i)];
    if (!std::isfinite(r) || r <= 0.0f) {
      continue;
    }
    if (std::isfinite(rmin) && r < rmin) {
      continue;
    }
    if (std::isfinite(rmax) && r > rmax) {
      continue;
    }
    m = std::min<double>(m, r);
  }
  return m;
}

std::string oppositeDirection(const std::string& direction)
{
  return (direction == "right") ? "left" : "right";
}

int directionToSign(const std::string& direction)
{
  return (direction == "right") ? -1 : +1;
}

}  // namespace

rclcpp::Node::SharedPtr SetGoal::node_;
rclcpp::Node::SharedPtr NavigateWithAvoidance::node_;
rclcpp::Node::SharedPtr GoalReached::node_;
rclcpp::Node::SharedPtr ObstacleInDirection::node_;
rclcpp::Node::SharedPtr SelectAvoidanceDirection::node_;
rclcpp::Node::SharedPtr DriveTowardGoal::node_;
rclcpp::Node::SharedPtr AvoidObstacle::node_;
rclcpp::Node::SharedPtr AlignToGoal::node_;
rclcpp::Node::SharedPtr StopAndObserve::node_;
rclcpp::Node::SharedPtr LogMessage::node_;

// =========================
// SetGoal
// =========================

SetGoal::SetGoal(const std::string& name, const BT::NodeConfiguration& cfg)
: BT::SyncActionNode(name, cfg) {}

bool SetGoal::getWaypoint(const std::string& name, Goal2D& out) const
{
  if (!node_) {
    return false;
  }

  const std::string key = std::string("waypoints.") + name;
  std::vector<double> v;
  if (!node_->get_parameter(key, v) || v.size() < 2) {
    return false;
  }

  out.x = v[0];
  out.y = v[1];
  out.yaw = (v.size() >= 3) ? v[2] : 0.0;
  return true;
}

BT::NodeStatus SetGoal::tick()
{
  if (!node_) {
    throw BT::RuntimeError("SetGoal: node not set");
  }

  const auto wp = getInput<std::string>("waypoint");
  if (!wp) {
    throw BT::RuntimeError("SetGoal missing input [waypoint]");
  }

  Goal2D g;
  if (!getWaypoint(wp.value(), g)) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "SetGoal: unknown waypoint '%s' (expected param waypoints.%s: [x,y,yaw])",
        wp.value().c_str(), wp.value().c_str());
    return BT::NodeStatus::FAILURE;
  }

  setOutput("goal", g);
  return BT::NodeStatus::SUCCESS;
}

// =========================
// NavigateWithAvoidance
// =========================

NavigateWithAvoidance::NavigateWithAvoidance(
    const std::string& name, const BT::NodeConfiguration& cfg)
: BT::StatefulActionNode(name, cfg) {}

void NavigateWithAvoidance::ensureInterfaces()
{
  if (!node_) {
    throw BT::RuntimeError("NavigateWithAvoidance: node not set");
  }

  if (!cmd_pub_) {
    cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

  const auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  if (!odom_sub_) {
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, odom_qos,
        [this](nav_msgs::msg::Odometry::SharedPtr msg) { odomCb(std::move(msg)); });
  }

  if (!scan_sub_) {
    scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::SharedPtr msg) { scanCb(std::move(msg)); });
  }

  const auto obstacle_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
  if (!obstacle_front_sub_) {
    obstacle_front_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        obstacle_front_topic_, obstacle_qos,
        [this](std_msgs::msg::Bool::SharedPtr msg) { obstacleFrontCb(std::move(msg)); });
  }
  if (!obstacle_left_sub_) {
    obstacle_left_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        obstacle_left_topic_, obstacle_qos,
        [this](std_msgs::msg::Bool::SharedPtr msg) { obstacleLeftCb(std::move(msg)); });
  }
  if (!obstacle_right_sub_) {
    obstacle_right_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        obstacle_right_topic_, obstacle_qos,
        [this](std_msgs::msg::Bool::SharedPtr msg) { obstacleRightCb(std::move(msg)); });
  }
}

void NavigateWithAvoidance::publishCmd(double lin, double ang)
{
  if (!cmd_pub_) {
    return;
  }
  geometry_msgs::msg::Twist t;
  t.linear.x = lin;
  t.angular.z = ang;
  cmd_pub_->publish(t);
}

void NavigateWithAvoidance::publishStop()
{
  publishCmd(0.0, 0.0);
}

void NavigateWithAvoidance::odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  x_ = msg->pose.pose.position.x;
  y_ = msg->pose.pose.position.y;
  yaw_ = yawFromQuat(msg->pose.pose.orientation);
  have_odom_ = true;
  last_odom_time_ = node_->now();
}

void NavigateWithAvoidance::scanCb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  const double front = sectorMin(*msg, kFrontSectorMin, kFrontSectorMax);
  const double left = sectorMin(*msg, kLeftSectorMin, kLeftSectorMax);
  const double right = sectorMin(*msg, kRightSectorMin, kRightSectorMax);

  std::lock_guard<std::mutex> lk(mtx_);
  scan_front_min_ = front;
  scan_left_min_ = left;
  scan_right_min_ = right;
  have_scan_ = true;
  last_scan_time_ = node_->now();
}

void NavigateWithAvoidance::obstacleFrontCb(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  obstacle_front_ = msg->data;
  have_obstacle_front_ = true;
  last_obstacle_front_time_ = node_->now();
}

void NavigateWithAvoidance::obstacleLeftCb(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  obstacle_left_ = msg->data;
  have_obstacle_left_ = true;
  last_obstacle_left_time_ = node_->now();
}

void NavigateWithAvoidance::obstacleRightCb(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  obstacle_right_ = msg->data;
  have_obstacle_right_ = true;
  last_obstacle_right_time_ = node_->now();
}

BT::NodeStatus NavigateWithAvoidance::onStart()
{
  if (!node_) {
    throw BT::RuntimeError("NavigateWithAvoidance: node not set");
  }

  const auto g = getInput<Goal2D>("goal");
  if (!g) {
    throw BT::RuntimeError("NavigateWithAvoidance missing input [goal]");
  }
  goal_ = g.value();
  have_goal_ = true;

  v_lin_ = getInput<double>("v_lin").value_or(1.5);
  min_lin_ = getInput<double>("min_lin").value_or(0.2);
  v_ang_ = getInput<double>("v_ang").value_or(1.2);
  dist_tol_ = getInput<double>("dist_tol").value_or(0.6);
  kp_yaw_ = getInput<double>("kp_yaw").value_or(1.5);
  yaw_slow_rad_ =
      (getInput<double>("yaw_slow_deg").value_or(25.0)) * (kPi / 180.0);
  obstacle_threshold_ = getInput<double>("obstacle_threshold_m").value_or(1.0);
  odom_timeout_s_ = getInput<double>("odom_timeout_s").value_or(1.0);
  reverse_speed_ = std::fabs(getInput<double>("reverse_speed").value_or(0.6));
  reverse_seconds_ = std::max(0.0, getInput<double>("reverse_seconds").value_or(1.0));
  avoid_arc_seconds_ =
      std::max(0.0, getInput<double>("avoid_arc_seconds").value_or(1.2));
  memory_seconds_ = std::max(0.0, getInput<double>("memory_seconds").value_or(3.0));

  odom_topic_ =
      getInput<std::string>("odom_topic").value_or("/model/curiosity_mars_rover/odometry");
  scan_topic_ = getInput<std::string>("scan_topic").value_or("/scan");
  obstacle_front_topic_ =
      getInput<std::string>("obstacle_front_topic").value_or("/obstacle/front");
  obstacle_left_topic_ =
      getInput<std::string>("obstacle_left_topic").value_or("/obstacle/left");
  obstacle_right_topic_ =
      getInput<std::string>("obstacle_right_topic").value_or("/obstacle/right");

  if (min_lin_ < 0.0) {
    RCLCPP_WARN(
        node_->get_logger(),
        "NavigateWithAvoidance: min_lin < 0.0; forcing to 0.0");
    min_lin_ = 0.0;
  }

  ensureInterfaces();
  avoidance_phase_ = AvoidancePhase::None;
  turn_sign_ = +1;
  last_turn_memory_sign_ = 0;
  last_odom_time_ = node_->now();
  phase_start_time_ = node_->now();
  last_turn_memory_time_ = node_->now();

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateWithAvoidance::onRunning()
{
  if (!have_goal_) {
    return BT::NodeStatus::FAILURE;
  }

  double x, y, yaw;
  double front_scan, left_scan, right_scan;
  bool have_odom, have_scan;
  bool front_topic, left_topic, right_topic;
  bool have_front_topic, have_left_topic, have_right_topic;
  rclcpp::Time t_odom, t_scan, t_front, t_left, t_right;

  {
    std::lock_guard<std::mutex> lk(mtx_);
    x = x_;
    y = y_;
    yaw = yaw_;
    front_scan = scan_front_min_;
    left_scan = scan_left_min_;
    right_scan = scan_right_min_;
    have_odom = have_odom_;
    have_scan = have_scan_;
    front_topic = obstacle_front_;
    left_topic = obstacle_left_;
    right_topic = obstacle_right_;
    have_front_topic = have_obstacle_front_;
    have_left_topic = have_obstacle_left_;
    have_right_topic = have_obstacle_right_;
    t_odom = last_odom_time_;
    t_scan = last_scan_time_;
    t_front = last_obstacle_front_time_;
    t_left = last_obstacle_left_time_;
    t_right = last_obstacle_right_time_;
  }

  const auto now = node_->now();

  if (!have_odom) {
    publishStop();
    if ((now - t_odom).seconds() < odom_timeout_s_) {
      return BT::NodeStatus::RUNNING;
    }
    RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "NavigateWithAvoidance: odom not received");
    return BT::NodeStatus::FAILURE;
  }

  if ((now - t_odom).seconds() > odom_timeout_s_) {
    publishStop();
    RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "NavigateWithAvoidance: odom stale");
    return BT::NodeStatus::FAILURE;
  }

  const double dx = goal_.x - x;
  const double dy = goal_.y - y;
  const double dist = std::hypot(dx, dy);
  if (dist <= dist_tol_) {
    publishStop();
    return BT::NodeStatus::SUCCESS;
  }

  const bool scan_fresh = have_scan && (now - t_scan).seconds() < 1.0;
  const bool front_fresh = have_front_topic && (now - t_front).seconds() < 1.0;
  const bool left_fresh = have_left_topic && (now - t_left).seconds() < 1.0;
  const bool right_fresh = have_right_topic && (now - t_right).seconds() < 1.0;

  const bool front_blocked =
      front_fresh ? front_topic : (scan_fresh && front_scan < obstacle_threshold_);
  const bool left_blocked =
      left_fresh ? left_topic : (scan_fresh && left_scan < obstacle_threshold_);
  const bool right_blocked =
      right_fresh ? right_topic : (scan_fresh && right_scan < obstacle_threshold_);

  if (avoidance_phase_ == AvoidancePhase::None && front_blocked) {
    const bool memory_fresh =
        last_turn_memory_sign_ != 0 &&
        (now - last_turn_memory_time_).seconds() < memory_seconds_;

    if (left_blocked && !right_blocked) {
      turn_sign_ = -1;
    } else if (right_blocked && !left_blocked) {
      turn_sign_ = +1;
    } else if (!left_blocked && !right_blocked) {
      if (memory_fresh) {
        turn_sign_ = -last_turn_memory_sign_;
      } else if (scan_fresh && std::isfinite(left_scan) && std::isfinite(right_scan)) {
        turn_sign_ = (left_scan >= right_scan) ? +1 : -1;
      } else {
        turn_sign_ = +1;
      }
    } else {
      turn_sign_ = memory_fresh ? -last_turn_memory_sign_ : +1;
      avoidance_phase_ = AvoidancePhase::Reverse;
      phase_start_time_ = now;
    }

    if (avoidance_phase_ == AvoidancePhase::None) {
      avoidance_phase_ = AvoidancePhase::Turn;
      phase_start_time_ = now;
    }

    last_turn_memory_sign_ = turn_sign_;
    last_turn_memory_time_ = now;
  }

  if (avoidance_phase_ == AvoidancePhase::Reverse) {
    publishCmd(-reverse_speed_, 0.0);
    if ((now - phase_start_time_).seconds() >= reverse_seconds_) {
      avoidance_phase_ = AvoidancePhase::Turn;
      phase_start_time_ = now;
    }
    return BT::NodeStatus::RUNNING;
  }

  if (avoidance_phase_ == AvoidancePhase::Turn) {
    publishCmd(0.0, static_cast<double>(turn_sign_) * v_ang_);
    const bool chosen_side_blocked = (turn_sign_ > 0) ? left_blocked : right_blocked;
    if ((!front_blocked && !chosen_side_blocked) ||
        (now - phase_start_time_).seconds() >= 2.0) {
      avoidance_phase_ = AvoidancePhase::Arc;
      phase_start_time_ = now;
    }
    return BT::NodeStatus::RUNNING;
  }

  if (avoidance_phase_ == AvoidancePhase::Arc) {
    const double arc_lin = std::max(min_lin_, std::min(v_lin_, 0.8));
    publishCmd(arc_lin, static_cast<double>(turn_sign_) * v_ang_ * 0.6);
    if ((now - phase_start_time_).seconds() >= avoid_arc_seconds_) {
      avoidance_phase_ = AvoidancePhase::None;
    }
    return BT::NodeStatus::RUNNING;
  }

  const double desired_yaw = std::atan2(dy, dx);
  const double yaw_err = wrapPi(desired_yaw - yaw);
  const double ang = clamp(kp_yaw_ * yaw_err, -v_ang_, +v_ang_);

  double lin = v_lin_;
  const double yaw_abs = std::fabs(yaw_err);
  if (yaw_abs > yaw_slow_rad_) {
    const double denom = std::max(1e-6, (kPi - yaw_slow_rad_));
    const double scale = 1.0 - (yaw_abs - yaw_slow_rad_) / denom;
    lin *= clamp(scale, 0.0, 1.0);
  }

  lin = std::max(lin, min_lin_);
  publishCmd(lin, ang);
  return BT::NodeStatus::RUNNING;
}

void NavigateWithAvoidance::onHalted()
{
  avoidance_phase_ = AvoidancePhase::None;
  publishStop();
}

// =========================
// GoalReached
// =========================

GoalReached::GoalReached(const std::string& name, const BT::NodeConfiguration& cfg)
: BT::ConditionNode(name, cfg) {}

void GoalReached::ensureInterfaces()
{
  if (!node_) {
    throw BT::RuntimeError("GoalReached: node not set");
  }

  const auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  if (!odom_sub_) {
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, odom_qos,
        [this](nav_msgs::msg::Odometry::SharedPtr msg) { odomCb(std::move(msg)); });
  }
}

void GoalReached::odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  x_ = msg->pose.pose.position.x;
  y_ = msg->pose.pose.position.y;
  have_odom_ = true;
  last_odom_time_ = node_->now();
}

BT::NodeStatus GoalReached::tick()
{
  if (!node_) {
    throw BT::RuntimeError("GoalReached: node not set");
  }

  const auto goal = getInput<Goal2D>("goal");
  if (!goal) {
    throw BT::RuntimeError("GoalReached missing input [goal]");
  }

  const double dist_tol = getInput<double>("dist_tol").value_or(0.6);
  odom_timeout_s_ = getInput<double>("odom_timeout_s").value_or(1.0);
  odom_topic_ =
      getInput<std::string>("odom_topic").value_or("/model/curiosity_mars_rover/odometry");
  ensureInterfaces();

  double x, y;
  bool have_odom;
  rclcpp::Time t_odom;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    x = x_;
    y = y_;
    have_odom = have_odom_;
    t_odom = last_odom_time_;
  }

  if (!have_odom || (node_->now() - t_odom).seconds() > odom_timeout_s_) {
    return BT::NodeStatus::FAILURE;
  }

  const double dx = goal->x - x;
  const double dy = goal->y - y;
  return (std::hypot(dx, dy) <= dist_tol) ? BT::NodeStatus::SUCCESS
                                          : BT::NodeStatus::FAILURE;
}

// =========================
// ObstacleInDirection
// =========================

ObstacleInDirection::ObstacleInDirection(
    const std::string& name, const BT::NodeConfiguration& cfg)
: BT::ConditionNode(name, cfg) {}

void ObstacleInDirection::ensureInterfaces()
{
  if (!node_) {
    throw BT::RuntimeError("ObstacleInDirection: node not set");
  }

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
  if (!front_sub_) {
    front_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        obstacle_front_topic_, qos,
        [this](std_msgs::msg::Bool::SharedPtr msg) { frontCb(std::move(msg)); });
  }
  if (!left_sub_) {
    left_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        obstacle_left_topic_, qos,
        [this](std_msgs::msg::Bool::SharedPtr msg) { leftCb(std::move(msg)); });
  }
  if (!right_sub_) {
    right_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        obstacle_right_topic_, qos,
        [this](std_msgs::msg::Bool::SharedPtr msg) { rightCb(std::move(msg)); });
  }
}

void ObstacleInDirection::frontCb(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  front_ = msg->data;
  have_front_ = true;
  front_time_ = node_->now();
}

void ObstacleInDirection::leftCb(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  left_ = msg->data;
  have_left_ = true;
  left_time_ = node_->now();
}

void ObstacleInDirection::rightCb(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  right_ = msg->data;
  have_right_ = true;
  right_time_ = node_->now();
}

BT::NodeStatus ObstacleInDirection::tick()
{
  if (!node_) {
    throw BT::RuntimeError("ObstacleInDirection: node not set");
  }

  const auto direction = getInput<std::string>("direction").value_or("front");
  obstacle_front_topic_ =
      getInput<std::string>("obstacle_front_topic").value_or("/obstacle/front");
  obstacle_left_topic_ =
      getInput<std::string>("obstacle_left_topic").value_or("/obstacle/left");
  obstacle_right_topic_ =
      getInput<std::string>("obstacle_right_topic").value_or("/obstacle/right");
  freshness_s_ = getInput<double>("freshness_s").value_or(1.0);
  ensureInterfaces();

  const auto now = node_->now();
  bool value = false;
  bool fresh = false;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (direction == "left") {
      value = left_;
      fresh = have_left_ && (now - left_time_).seconds() < freshness_s_;
    } else if (direction == "right") {
      value = right_;
      fresh = have_right_ && (now - right_time_).seconds() < freshness_s_;
    } else {
      value = front_;
      fresh = have_front_ && (now - front_time_).seconds() < freshness_s_;
    }
  }

  return (fresh && value) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// =========================
// SelectAvoidanceDirection
// =========================

SelectAvoidanceDirection::SelectAvoidanceDirection(
    const std::string& name, const BT::NodeConfiguration& cfg)
: BT::SyncActionNode(name, cfg) {}

void SelectAvoidanceDirection::ensureInterfaces()
{
  if (!node_) {
    throw BT::RuntimeError("SelectAvoidanceDirection: node not set");
  }

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
  if (!front_sub_) {
    front_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        obstacle_front_topic_, qos,
        [this](std_msgs::msg::Bool::SharedPtr msg) { frontCb(std::move(msg)); });
  }
  if (!left_sub_) {
    left_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        obstacle_left_topic_, qos,
        [this](std_msgs::msg::Bool::SharedPtr msg) { leftCb(std::move(msg)); });
  }
  if (!right_sub_) {
    right_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        obstacle_right_topic_, qos,
        [this](std_msgs::msg::Bool::SharedPtr msg) { rightCb(std::move(msg)); });
  }
}

void SelectAvoidanceDirection::frontCb(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  front_ = msg->data;
  have_front_ = true;
  front_time_ = node_->now();
}

void SelectAvoidanceDirection::leftCb(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  left_ = msg->data;
  have_left_ = true;
  left_time_ = node_->now();
}

void SelectAvoidanceDirection::rightCb(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  right_ = msg->data;
  have_right_ = true;
  right_time_ = node_->now();
}

BT::NodeStatus SelectAvoidanceDirection::tick()
{
  if (!node_) {
    throw BT::RuntimeError("SelectAvoidanceDirection: node not set");
  }

  obstacle_front_topic_ =
      getInput<std::string>("obstacle_front_topic").value_or("/obstacle/front");
  obstacle_left_topic_ =
      getInput<std::string>("obstacle_left_topic").value_or("/obstacle/left");
  obstacle_right_topic_ =
      getInput<std::string>("obstacle_right_topic").value_or("/obstacle/right");
  freshness_s_ = getInput<double>("freshness_s").value_or(1.0);
  memory_seconds_ = getInput<double>("memory_seconds").value_or(3.0);
  ensureInterfaces();

  const auto now = node_->now();
  bool front = false;
  bool left = false;
  bool right = false;
  bool front_fresh = false;
  bool left_fresh = false;
  bool right_fresh = false;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    front = front_;
    left = left_;
    right = right_;
    front_fresh = have_front_ && (now - front_time_).seconds() < freshness_s_;
    left_fresh = have_left_ && (now - left_time_).seconds() < freshness_s_;
    right_fresh = have_right_ && (now - right_time_).seconds() < freshness_s_;
  }

  if (!front_fresh || !left_fresh || !right_fresh) {
    return BT::NodeStatus::FAILURE;
  }

  std::string direction = "left";
  bool reverse_first = false;
  const bool memory_fresh =
      have_last_direction_ &&
      (now - last_direction_time_).seconds() < memory_seconds_;

  if (left && !right) {
    direction = "right";
  } else if (right && !left) {
    direction = "left";
  } else if (left && right) {
    reverse_first = true;
    direction = memory_fresh ? oppositeDirection(last_direction_) : "left";
  } else if (front && memory_fresh) {
    direction = oppositeDirection(last_direction_);
  }

  setOutput("turn_direction", direction);
  setOutput("reverse_first", reverse_first);
  last_direction_ = direction;
  have_last_direction_ = true;
  last_direction_time_ = now;
  return BT::NodeStatus::SUCCESS;
}

// =========================
// DriveTowardGoal
// =========================

DriveTowardGoal::DriveTowardGoal(
    const std::string& name, const BT::NodeConfiguration& cfg)
: BT::StatefulActionNode(name, cfg) {}

void DriveTowardGoal::ensureInterfaces()
{
  if (!node_) {
    throw BT::RuntimeError("DriveTowardGoal: node not set");
  }

  if (!cmd_pub_) {
    cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

  const auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  if (!odom_sub_) {
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, odom_qos,
        [this](nav_msgs::msg::Odometry::SharedPtr msg) { odomCb(std::move(msg)); });
  }
}

void DriveTowardGoal::publishCmd(double lin, double ang)
{
  if (!cmd_pub_) {
    return;
  }
  geometry_msgs::msg::Twist t;
  t.linear.x = lin;
  t.angular.z = ang;
  cmd_pub_->publish(t);
}

void DriveTowardGoal::publishStop()
{
  publishCmd(0.0, 0.0);
}

void DriveTowardGoal::odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  x_ = msg->pose.pose.position.x;
  y_ = msg->pose.pose.position.y;
  yaw_ = yawFromQuat(msg->pose.pose.orientation);
  have_odom_ = true;
  last_odom_time_ = node_->now();
}

BT::NodeStatus DriveTowardGoal::onStart()
{
  if (!node_) {
    throw BT::RuntimeError("DriveTowardGoal: node not set");
  }

  const auto g = getInput<Goal2D>("goal");
  if (!g) {
    throw BT::RuntimeError("DriveTowardGoal missing input [goal]");
  }
  goal_ = g.value();
  have_goal_ = true;

  v_lin_ = getInput<double>("v_lin").value_or(1.5);
  min_lin_ = std::max(0.0, getInput<double>("min_lin").value_or(0.2));
  v_ang_ = getInput<double>("v_ang").value_or(1.2);
  dist_tol_ = getInput<double>("dist_tol").value_or(0.6);
  kp_yaw_ = getInput<double>("kp_yaw").value_or(1.5);
  yaw_slow_rad_ =
      (getInput<double>("yaw_slow_deg").value_or(25.0)) * (kPi / 180.0);
  odom_timeout_s_ = getInput<double>("odom_timeout_s").value_or(1.0);
  odom_topic_ =
      getInput<std::string>("odom_topic").value_or("/model/curiosity_mars_rover/odometry");
  last_odom_time_ = node_->now();

  ensureInterfaces();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DriveTowardGoal::onRunning()
{
  if (!have_goal_) {
    return BT::NodeStatus::FAILURE;
  }

  double x, y, yaw;
  bool have_odom;
  rclcpp::Time t_odom;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    x = x_;
    y = y_;
    yaw = yaw_;
    have_odom = have_odom_;
    t_odom = last_odom_time_;
  }

  const auto now = node_->now();
  if (!have_odom) {
    publishStop();
    if ((now - t_odom).seconds() < odom_timeout_s_) {
      return BT::NodeStatus::RUNNING;
    }
    RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "DriveTowardGoal: odom not received");
    return BT::NodeStatus::FAILURE;
  }

  if ((now - t_odom).seconds() > odom_timeout_s_) {
    publishStop();
    RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "DriveTowardGoal: odom stale");
    return BT::NodeStatus::FAILURE;
  }

  const double dx = goal_.x - x;
  const double dy = goal_.y - y;
  const double dist = std::hypot(dx, dy);
  if (dist <= dist_tol_) {
    publishStop();
    return BT::NodeStatus::SUCCESS;
  }

  const double desired_yaw = std::atan2(dy, dx);
  const double yaw_err = wrapPi(desired_yaw - yaw);
  const double ang = clamp(kp_yaw_ * yaw_err, -v_ang_, +v_ang_);

  double lin = v_lin_;
  const double yaw_abs = std::fabs(yaw_err);
  if (yaw_abs > yaw_slow_rad_) {
    const double denom = std::max(1e-6, (kPi - yaw_slow_rad_));
    const double scale = 1.0 - (yaw_abs - yaw_slow_rad_) / denom;
    lin *= clamp(scale, 0.0, 1.0);
  }

  publishCmd(std::max(lin, min_lin_), ang);
  return BT::NodeStatus::RUNNING;
}

void DriveTowardGoal::onHalted()
{
  publishStop();
}

// =========================
// AvoidObstacle
// =========================

AvoidObstacle::AvoidObstacle(const std::string& name, const BT::NodeConfiguration& cfg)
: BT::StatefulActionNode(name, cfg) {}

void AvoidObstacle::ensureInterfaces()
{
  if (!node_) {
    throw BT::RuntimeError("AvoidObstacle: node not set");
  }

  if (!cmd_pub_) {
    cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

  if (!scan_sub_) {
    scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::SharedPtr msg) { scanCb(std::move(msg)); });
  }

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
  if (!front_sub_) {
    front_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        obstacle_front_topic_, qos,
        [this](std_msgs::msg::Bool::SharedPtr msg) { frontCb(std::move(msg)); });
  }
  if (!left_sub_) {
    left_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        obstacle_left_topic_, qos,
        [this](std_msgs::msg::Bool::SharedPtr msg) { leftCb(std::move(msg)); });
  }
  if (!right_sub_) {
    right_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        obstacle_right_topic_, qos,
        [this](std_msgs::msg::Bool::SharedPtr msg) { rightCb(std::move(msg)); });
  }
}

void AvoidObstacle::publishCmd(double lin, double ang)
{
  if (!cmd_pub_) {
    return;
  }
  geometry_msgs::msg::Twist t;
  t.linear.x = lin;
  t.angular.z = ang;
  cmd_pub_->publish(t);
}

void AvoidObstacle::publishStop()
{
  publishCmd(0.0, 0.0);
}

void AvoidObstacle::scanCb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  const double front = sectorMin(*msg, kFrontSectorMin, kFrontSectorMax);
  const double left = sectorMin(*msg, kLeftSectorMin, kLeftSectorMax);
  const double right = sectorMin(*msg, kRightSectorMin, kRightSectorMax);

  std::lock_guard<std::mutex> lk(mtx_);
  scan_front_min_ = front;
  scan_left_min_ = left;
  scan_right_min_ = right;
  have_scan_ = true;
  last_scan_time_ = node_->now();
}

void AvoidObstacle::frontCb(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  front_ = msg->data;
  have_front_ = true;
  front_time_ = node_->now();
}

void AvoidObstacle::leftCb(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  left_ = msg->data;
  have_left_ = true;
  left_time_ = node_->now();
}

void AvoidObstacle::rightCb(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  right_ = msg->data;
  have_right_ = true;
  right_time_ = node_->now();
}

BT::NodeStatus AvoidObstacle::onStart()
{
  if (!node_) {
    throw BT::RuntimeError("AvoidObstacle: node not set");
  }

  turn_direction_ = getInput<std::string>("turn_direction").value_or("left");
  reverse_first_ = getInput<bool>("reverse_first").value_or(false);
  scan_topic_ = getInput<std::string>("scan_topic").value_or("/scan");
  obstacle_front_topic_ =
      getInput<std::string>("obstacle_front_topic").value_or("/obstacle/front");
  obstacle_left_topic_ =
      getInput<std::string>("obstacle_left_topic").value_or("/obstacle/left");
  obstacle_right_topic_ =
      getInput<std::string>("obstacle_right_topic").value_or("/obstacle/right");
  obstacle_threshold_ = getInput<double>("obstacle_threshold_m").value_or(1.0);
  reverse_speed_ = std::fabs(getInput<double>("reverse_speed").value_or(0.6));
  reverse_seconds_ = std::max(0.0, getInput<double>("reverse_seconds").value_or(1.0));
  turn_speed_ = std::fabs(getInput<double>("turn_speed").value_or(1.2));
  turn_timeout_s_ = std::max(0.2, getInput<double>("turn_timeout_s").value_or(2.5));
  arc_speed_ = std::max(0.0, getInput<double>("arc_speed").value_or(0.7));
  arc_seconds_ = std::max(0.1, getInput<double>("arc_seconds").value_or(1.4));

  ensureInterfaces();
  phase_ = reverse_first_ ? Phase::Reverse : Phase::Turn;
  phase_start_time_ = node_->now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AvoidObstacle::onRunning()
{
  const auto now = node_->now();
  bool have_scan;
  double scan_front, scan_left, scan_right;
  bool front, left, right;
  bool have_front, have_left, have_right;
  rclcpp::Time t_scan, t_front, t_left, t_right;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    have_scan = have_scan_;
    scan_front = scan_front_min_;
    scan_left = scan_left_min_;
    scan_right = scan_right_min_;
    front = front_;
    left = left_;
    right = right_;
    have_front = have_front_;
    have_left = have_left_;
    have_right = have_right_;
    t_scan = last_scan_time_;
    t_front = front_time_;
    t_left = left_time_;
    t_right = right_time_;
  }

  const bool scan_fresh = have_scan && (now - t_scan).seconds() < 1.0;
  const bool front_fresh = have_front && (now - t_front).seconds() < 1.0;
  const bool left_fresh = have_left && (now - t_left).seconds() < 1.0;
  const bool right_fresh = have_right && (now - t_right).seconds() < 1.0;

  const bool front_blocked =
      front_fresh ? front : (scan_fresh && scan_front < obstacle_threshold_);
  const bool left_blocked =
      left_fresh ? left : (scan_fresh && scan_left < obstacle_threshold_);
  const bool right_blocked =
      right_fresh ? right : (scan_fresh && scan_right < obstacle_threshold_);

  const int turn_sign = directionToSign(turn_direction_);
  const bool chosen_side_blocked = (turn_sign > 0) ? left_blocked : right_blocked;

  if (phase_ == Phase::Reverse) {
    publishCmd(-reverse_speed_, 0.0);
    if ((now - phase_start_time_).seconds() >= reverse_seconds_) {
      phase_ = Phase::Turn;
      phase_start_time_ = now;
    }
    return BT::NodeStatus::RUNNING;
  }

  if (phase_ == Phase::Turn) {
    publishCmd(0.0, static_cast<double>(turn_sign) * turn_speed_);
    if ((!front_blocked && !chosen_side_blocked) ||
        (now - phase_start_time_).seconds() >= turn_timeout_s_) {
      phase_ = Phase::Arc;
      phase_start_time_ = now;
    }
    return BT::NodeStatus::RUNNING;
  }

  publishCmd(arc_speed_, static_cast<double>(turn_sign) * turn_speed_ * 0.5);
  if ((now - phase_start_time_).seconds() >= arc_seconds_) {
    publishStop();
    return BT::NodeStatus::SUCCESS;
  }
  if (front_blocked && chosen_side_blocked && (now - phase_start_time_).seconds() > 0.5) {
    publishStop();
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void AvoidObstacle::onHalted()
{
  publishStop();
}

// =========================
// KeepRunning
// =========================

KeepRunning::KeepRunning(const std::string& name, const BT::NodeConfiguration& cfg)
: BT::StatefulActionNode(name, cfg) {}

BT::NodeStatus KeepRunning::onStart()
{
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus KeepRunning::onRunning()
{
  return BT::NodeStatus::RUNNING;
}

void KeepRunning::onHalted() {}

// =========================
// AlignToGoal
// =========================

AlignToGoal::AlignToGoal(const std::string& name, const BT::NodeConfiguration& cfg)
: BT::StatefulActionNode(name, cfg) {}

void AlignToGoal::ensureInterfaces()
{
  if (!node_) {
    throw BT::RuntimeError("AlignToGoal: node not set");
  }

  if (!cmd_pub_) {
    cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

  const auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  if (!odom_sub_) {
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, odom_qos,
        [this](nav_msgs::msg::Odometry::SharedPtr msg) { odomCb(std::move(msg)); });
  }
}

void AlignToGoal::publishCmd(double lin, double ang)
{
  if (!cmd_pub_) {
    return;
  }
  geometry_msgs::msg::Twist t;
  t.linear.x = lin;
  t.angular.z = ang;
  cmd_pub_->publish(t);
}

void AlignToGoal::publishStop()
{
  publishCmd(0.0, 0.0);
}

void AlignToGoal::odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mtx_);
  x_ = msg->pose.pose.position.x;
  y_ = msg->pose.pose.position.y;
  yaw_ = yawFromQuat(msg->pose.pose.orientation);
  have_odom_ = true;
  last_odom_time_ = node_->now();
}

BT::NodeStatus AlignToGoal::onStart()
{
  if (!node_) {
    throw BT::RuntimeError("AlignToGoal: node not set");
  }

  const auto g = getInput<Goal2D>("goal");
  if (!g) {
    throw BT::RuntimeError("AlignToGoal missing input [goal]");
  }
  goal_ = g.value();
  have_goal_ = true;

  v_lin_ = getInput<double>("v_lin").value_or(0.0);
  min_lin_ = std::max(0.0, getInput<double>("min_lin").value_or(0.0));
  v_ang_ = getInput<double>("v_ang").value_or(1.2);
  kp_yaw_ = getInput<double>("kp_yaw").value_or(2.0);
  yaw_tol_rad_ =
      (getInput<double>("yaw_tol_deg").value_or(10.0)) * (kPi / 180.0);
  odom_timeout_s_ = getInput<double>("odom_timeout_s").value_or(1.0);
  odom_topic_ =
      getInput<std::string>("odom_topic").value_or("/model/curiosity_mars_rover/odometry");

  last_odom_time_ = node_->now();
  ensureInterfaces();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AlignToGoal::onRunning()
{
  if (!have_goal_) {
    return BT::NodeStatus::FAILURE;
  }

  double x, y, yaw;
  bool have_odom;
  rclcpp::Time t_odom;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    x = x_;
    y = y_;
    yaw = yaw_;
    have_odom = have_odom_;
    t_odom = last_odom_time_;
  }

  const auto now = node_->now();
  if (!have_odom) {
    publishStop();
    if ((now - t_odom).seconds() < odom_timeout_s_) {
      return BT::NodeStatus::RUNNING;
    }
    RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "AlignToGoal: odom not received");
    return BT::NodeStatus::FAILURE;
  }

  if ((now - t_odom).seconds() > odom_timeout_s_) {
    publishStop();
    RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "AlignToGoal: odom stale");
    return BT::NodeStatus::FAILURE;
  }

  const double desired_yaw = std::atan2(goal_.y - y, goal_.x - x);
  const double yaw_err = wrapPi(desired_yaw - yaw);
  if (std::fabs(yaw_err) <= yaw_tol_rad_) {
    publishStop();
    return BT::NodeStatus::SUCCESS;
  }

  const double ang = clamp(kp_yaw_ * yaw_err, -v_ang_, +v_ang_);
  publishCmd(std::max(v_lin_, min_lin_), ang);
  return BT::NodeStatus::RUNNING;
}

void AlignToGoal::onHalted()
{
  publishStop();
}

// =========================
// StopAndObserve
// =========================

StopAndObserve::StopAndObserve(const std::string& name, const BT::NodeConfiguration& cfg)
: BT::StatefulActionNode(name, cfg) {}

void StopAndObserve::publishStop()
{
  if (!cmd_pub_) {
    if (!node_) {
      throw BT::RuntimeError("StopAndObserve: node not set");
    }
    cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }
  geometry_msgs::msg::Twist t;
  cmd_pub_->publish(t);
}

BT::NodeStatus StopAndObserve::onStart()
{
  if (!node_) {
    throw BT::RuntimeError("StopAndObserve: node not set");
  }
  seconds_ = getInput<double>("seconds").value_or(3.0);
  start_ = node_->now();
  publishStop();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus StopAndObserve::onRunning()
{
  publishStop();
  return ((node_->now() - start_).seconds() >= seconds_) ? BT::NodeStatus::SUCCESS
                                                          : BT::NodeStatus::RUNNING;
}

void StopAndObserve::onHalted()
{
  publishStop();
}

// =========================
// LogMessage
// =========================

LogMessage::LogMessage(const std::string& name, const BT::NodeConfiguration& cfg)
: BT::SyncActionNode(name, cfg) {}

BT::NodeStatus LogMessage::tick()
{
  if (!node_) {
    throw BT::RuntimeError("LogMessage: node not set");
  }
  const auto msg = getInput<std::string>("message");
  if (!msg) {
    throw BT::RuntimeError("LogMessage missing input [message]");
  }
  RCLCPP_INFO(node_->get_logger(), "%s", msg.value().c_str());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace spacetry_bt
