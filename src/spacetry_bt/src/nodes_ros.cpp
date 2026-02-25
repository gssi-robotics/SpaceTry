#include "spacetry_bt/nodes_ros.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <vector>

namespace spacetry_bt {

namespace {

constexpr double kPi = 3.14159265358979323846;

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

// Minimum finite range in a sector [a0, a1] (radians, LaserScan frame)
double sectorMin(const sensor_msgs::msg::LaserScan& scan, double a0, double a1)
{
  if (scan.ranges.empty() || scan.angle_increment == 0.0) {
    return std::numeric_limits<double>::infinity();
  }

  if (a1 < a0) std::swap(a0, a1);

  const auto clampIndex = [&](int i) {
    return std::max(0, std::min(i, static_cast<int>(scan.ranges.size()) - 1));
  };

  const int i0 = clampIndex(static_cast<int>(std::round((a0 - scan.angle_min) / scan.angle_increment)));
  const int i1 = clampIndex(static_cast<int>(std::round((a1 - scan.angle_min) / scan.angle_increment)));

  double m = std::numeric_limits<double>::infinity();
  for (int i = std::min(i0, i1); i <= std::max(i0, i1); ++i) {
    const float r = scan.ranges[static_cast<size_t>(i)];
    if (std::isfinite(r)) m = std::min<double>(m, r);
  }
  return m;
}

}  // namespace

rclcpp::Node::SharedPtr SetGoal::node_;
rclcpp::Node::SharedPtr NavigateWithAvoidance::node_;
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
  if (!node_) return false;

  const std::string key = std::string("waypoints.") + name;
  std::vector<double> v;
  if (!node_->get_parameter(key, v)) {
    return false;
  }
  if (v.size() < 2) return false;

  out.x = v[0];
  out.y = v[1];
  out.yaw = (v.size() >= 3) ? v[2] : 0.0;
  return true;
}

BT::NodeStatus SetGoal::tick()
{
  if (!node_) throw BT::RuntimeError("SetGoal: node not set");
  const auto wp = getInput<std::string>("waypoint");
  if (!wp) throw BT::RuntimeError("SetGoal missing input [waypoint]");

  Goal2D g;
  if (!getWaypoint(wp.value(), g)) {
    RCLCPP_ERROR(node_->get_logger(),
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

NavigateWithAvoidance::NavigateWithAvoidance(const std::string& name, const BT::NodeConfiguration& cfg)
: BT::StatefulActionNode(name, cfg) {}

void NavigateWithAvoidance::ensureInterfaces()
{
  if (!node_) throw BT::RuntimeError("NavigateWithAvoidance: node not set");

  if (!cmd_pub_) {
    cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

  // Many Gazebo bridges publish odom as BEST_EFFORT; default Reliable subscriber may receive nothing.
  const auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

  if (!odom_sub_) {
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, odom_qos,
      [this](nav_msgs::msg::Odometry::SharedPtr msg){ odomCb(std::move(msg)); }
    );
  }

  if (!scan_sub_) {
    scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg){ scanCb(std::move(msg)); }
    );
  }

  // Same idea: depending on bridge/publisher, BEST_EFFORT avoids QoS mismatch.
  const auto obstacle_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

  if (!obstacle_front_sub_) {
    obstacle_front_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      obstacle_front_topic_, obstacle_qos,
      [this](std_msgs::msg::Bool::SharedPtr msg){ obstacleFrontCb(std::move(msg)); }
    );
  }
}

void NavigateWithAvoidance::publishCmd(double lin, double ang)
{
  if (!cmd_pub_) return;
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
  // Sectors: front [-15,+15], left [15,60], right [-60,-15]
  const double deg = kPi / 180.0;
  const double front = sectorMin(*msg, -15.0 * deg, +15.0 * deg);
  const double left  = sectorMin(*msg, +15.0 * deg, +60.0 * deg);
  const double right = sectorMin(*msg, -60.0 * deg, -15.0 * deg);

  std::lock_guard<std::mutex> lk(mtx_);
  scan_front_min_ = front;
  scan_left_min_  = left;
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

BT::NodeStatus NavigateWithAvoidance::onStart()
{
  if (!node_) throw BT::RuntimeError("NavigateWithAvoidance: node not set");

  const auto g = getInput<Goal2D>("goal");
  if (!g) throw BT::RuntimeError("NavigateWithAvoidance missing input [goal]");
  goal_ = g.value();
  have_goal_ = true;

  v_lin_ = getInput<double>("v_lin").value_or(0.3);
  v_ang_ = getInput<double>("v_ang").value_or(0.6);
  dist_tol_ = getInput<double>("dist_tol").value_or(0.6);
  kp_yaw_ = getInput<double>("kp_yaw").value_or(1.5);

  yaw_slow_rad_ = (getInput<double>("yaw_slow_deg").value_or(25.0)) * (kPi / 180.0);
  obstacle_threshold_ = getInput<double>("obstacle_threshold_m").value_or(1.0);
  odom_timeout_s_ = getInput<double>("odom_timeout_s").value_or(1.0);

  odom_topic_ = getInput<std::string>("odom_topic").value_or("/model/curiosity_mars_rover/odometry");
  scan_topic_ = getInput<std::string>("scan_topic").value_or("/scan");
  obstacle_front_topic_ = getInput<std::string>("obstacle_front_topic").value_or("/obstacle/front");

  // Use last_odom_time_ as a "startup timer" so we can wait for the first odom message.
  last_odom_time_ = node_->now();

  ensureInterfaces();
  avoiding_ = false;
  turn_sign_ = +1;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateWithAvoidance::onRunning()
{
  if (!have_goal_) return BT::NodeStatus::FAILURE;

  // Copy latest state
  double x, y, yaw;
  double front, left, right;
  bool have_odom, have_scan;
  bool obs_front, have_obs_front;
  rclcpp::Time t_odom, t_scan, t_obs;

  {
    std::lock_guard<std::mutex> lk(mtx_);
    x = x_; y = y_; yaw = yaw_;
    front = scan_front_min_;
    left  = scan_left_min_;
    right = scan_right_min_;
    have_odom = have_odom_;
    have_scan = have_scan_;
    obs_front = obstacle_front_;
    have_obs_front = have_obstacle_front_;
    t_odom = last_odom_time_;
    t_scan = last_scan_time_;
    t_obs  = last_obstacle_front_time_;
  }

  const auto now = node_->now();

  // --- ODOM handling ---
  // If we haven't received odom yet, don't instantly fail: wait up to odom_timeout_s_.
  if (!have_odom) {
    publishStop();
    if ((now - t_odom).seconds() < odom_timeout_s_) {
      return BT::NodeStatus::RUNNING;
    }
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                          "NavigateWithAvoidance: odom not received");
    return BT::NodeStatus::FAILURE;
  }

  // If we have odom, but it's stale, fail.
  if ((now - t_odom).seconds() > odom_timeout_s_) {
    publishStop();
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
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

  // Prefer /obstacle/front if fresh; else fallback to /scan
  bool front_blocked = false;
  if (have_obs_front && (now - t_obs).seconds() < 1.0) {
    front_blocked = obs_front;
  } else if (have_scan && (now - t_scan).seconds() < 1.0) {
    front_blocked = (front < obstacle_threshold_);
  }

  if (front_blocked) {
    if (!avoiding_) {
      // Choose the more free side if scan is available, else default left
      if (have_scan && std::isfinite(left) && std::isfinite(right)) {
        turn_sign_ = (left >= right) ? +1 : -1;
      } else {
        turn_sign_ = +1;
      }
      avoiding_ = true;
    }
    publishCmd(0.0, static_cast<double>(turn_sign_) * v_ang_);
    return BT::NodeStatus::RUNNING;
  }

  avoiding_ = false;

  // Nominal go-to-goal
  const double desired_yaw = std::atan2(dy, dx);
  const double yaw_err = wrapPi(desired_yaw - yaw);
  const double ang = clamp(kp_yaw_ * yaw_err, -v_ang_, +v_ang_);

  double lin = v_lin_;
  if (std::fabs(yaw_err) > yaw_slow_rad_) {
    lin = 0.0;
  }

  publishCmd(lin, ang);
  return BT::NodeStatus::RUNNING;
}

void NavigateWithAvoidance::onHalted()
{
  publishStop();
}

// =========================
// AlignToGoal
// =========================

AlignToGoal::AlignToGoal(const std::string& name, const BT::NodeConfiguration& cfg)
: BT::StatefulActionNode(name, cfg) {}

void AlignToGoal::ensureInterfaces()
{
  if (!node_) throw BT::RuntimeError("AlignToGoal: node not set");

  if (!cmd_pub_) {
    cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

  // Match Gazebo/bridge odom QoS (often BEST_EFFORT).
  const auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

  if (!odom_sub_) {
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, odom_qos,
      [this](nav_msgs::msg::Odometry::SharedPtr msg){ odomCb(std::move(msg)); }
    );
  }
}

void AlignToGoal::publishCmd(double ang)
{
  if (!cmd_pub_) return;
  geometry_msgs::msg::Twist t;
  t.angular.z = ang;
  cmd_pub_->publish(t);
}

void AlignToGoal::publishStop()
{
  publishCmd(0.0);
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
  if (!node_) throw BT::RuntimeError("AlignToGoal: node not set");

  const auto g = getInput<Goal2D>("goal");
  if (!g) throw BT::RuntimeError("AlignToGoal missing input [goal]");
  goal_ = g.value();
  have_goal_ = true;

  v_ang_ = getInput<double>("v_ang").value_or(0.4);
  kp_yaw_ = getInput<double>("kp_yaw").value_or(2.0);
  yaw_tol_rad_ = (getInput<double>("yaw_tol_deg").value_or(10.0)) * (kPi / 180.0);
  odom_timeout_s_ = getInput<double>("odom_timeout_s").value_or(1.0);
  odom_topic_ = getInput<std::string>("odom_topic").value_or("/model/curiosity_mars_rover/odometry");

  // Startup timer/grace period for first odom message.
  last_odom_time_ = node_->now();

  ensureInterfaces();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AlignToGoal::onRunning()
{
  if (!have_goal_) return BT::NodeStatus::FAILURE;

  double x, y, yaw;
  bool have_odom;
  rclcpp::Time t_odom;

  {
    std::lock_guard<std::mutex> lk(mtx_);
    x = x_; y = y_; yaw = yaw_;
    have_odom = have_odom_;
    t_odom = last_odom_time_;
  }

  const auto now = node_->now();

  // If no odom yet, wait (RUNNING) up to odom_timeout_s_.
  if (!have_odom) {
    publishStop();
    if ((now - t_odom).seconds() < odom_timeout_s_) {
      return BT::NodeStatus::RUNNING;
    }
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                          "AlignToGoal: odom not received");
    return BT::NodeStatus::FAILURE;
  }

  // If odom stale, fail.
  if ((now - t_odom).seconds() > odom_timeout_s_) {
    publishStop();
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                          "AlignToGoal: odom stale");
    return BT::NodeStatus::FAILURE;
  }

  const double dx = goal_.x - x;
  const double dy = goal_.y - y;
  const double desired_yaw = std::atan2(dy, dx);
  const double yaw_err = wrapPi(desired_yaw - yaw);

  if (std::fabs(yaw_err) <= yaw_tol_rad_) {
    publishStop();
    return BT::NodeStatus::SUCCESS;
  }

  const double ang = clamp(kp_yaw_ * yaw_err, -v_ang_, +v_ang_);
  publishCmd(ang);
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
    if (!node_) throw BT::RuntimeError("StopAndObserve: node not set");
    cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }
  geometry_msgs::msg::Twist t;  // zeros
  cmd_pub_->publish(t);
}

BT::NodeStatus StopAndObserve::onStart()
{
  if (!node_) throw BT::RuntimeError("StopAndObserve: node not set");

  seconds_ = getInput<double>("seconds").value_or(3.0);
  start_ = node_->now();

  publishStop();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus StopAndObserve::onRunning()
{
  publishStop();
  const double elapsed = (node_->now() - start_).seconds();
  return (elapsed >= seconds_) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
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
  if (!node_) throw BT::RuntimeError("LogMessage: node not set");

  const auto msg = getInput<std::string>("message");
  if (!msg) throw BT::RuntimeError("LogMessage missing input [message]");

  RCLCPP_INFO(node_->get_logger(), "%s", msg.value().c_str());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace spacetry_bt