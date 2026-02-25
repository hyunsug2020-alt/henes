#include "jeju_mpc/mpc_path_tracker_cpp.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace jeju_mpc
{

MPCPathTrackerCpp::MPCPathTrackerCpp()
: Node("mpc_path_tracker_cpp")
{
  this->declare_parameter("control_rate_hz", 20.0);
  this->declare_parameter("wheelbase_m", 1.04);
  this->declare_parameter("forward_speed_kmh", 8.0);
  this->declare_parameter("prediction_horizon", 15);
  this->declare_parameter("dt", 0.1);
  this->declare_parameter("w_d", 1.0);
  this->declare_parameter("w_theta", 0.5);
  this->declare_parameter("w_kappa", 0.05);
  this->declare_parameter("w_u", 0.2);
  this->declare_parameter("kappa_min", -0.9);
  this->declare_parameter("kappa_max", 0.9);
  this->declare_parameter("kappa_samples", 31);
  this->declare_parameter("max_steer_deg", 55.0);
  this->declare_parameter("goal_tolerance_m", 1.0);

  control_rate_hz_ = this->get_parameter("control_rate_hz").as_double();
  forward_speed_kmh_ = this->get_parameter("forward_speed_kmh").as_double();
  goal_tolerance_m_ = this->get_parameter("goal_tolerance_m").as_double();

  controller_params_.wheelbase_m = this->get_parameter("wheelbase_m").as_double();
  controller_params_.prediction_horizon = this->get_parameter("prediction_horizon").as_int();
  controller_params_.dt = this->get_parameter("dt").as_double();
  controller_params_.w_d = this->get_parameter("w_d").as_double();
  controller_params_.w_theta = this->get_parameter("w_theta").as_double();
  controller_params_.w_kappa = this->get_parameter("w_kappa").as_double();
  controller_params_.w_u = this->get_parameter("w_u").as_double();
  controller_params_.kappa_min = this->get_parameter("kappa_min").as_double();
  controller_params_.kappa_max = this->get_parameter("kappa_max").as_double();
  controller_params_.kappa_samples = this->get_parameter("kappa_samples").as_int();
  controller_params_.max_steer_deg = this->get_parameter("max_steer_deg").as_double();

  controller_.updateParameters(controller_params_);

  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/filtered", 20,
    std::bind(&MPCPathTrackerCpp::odomCb, this, std::placeholders::_1));

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/global_path", 10,
    std::bind(&MPCPathTrackerCpp::pathCb, this, std::placeholders::_1));

  teleop_mode_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/teleop_mode", 10,
    std::bind(&MPCPathTrackerCpp::teleopModeCb, this, std::placeholders::_1));

  obstacle_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/obstacle_stop", 10,
    std::bind(&MPCPathTrackerCpp::obstacleStopCb, this, std::placeholders::_1));

  slope_factor_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/slope_factor", 10,
    std::bind(&MPCPathTrackerCpp::slopeFactorCb, this, std::placeholders::_1));

  auto period = std::chrono::duration<double>(1.0 / std::max(control_rate_hz_, 1.0));
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&MPCPathTrackerCpp::controlLoop, this));

  RCLCPP_INFO(
    this->get_logger(),
    "Single-vehicle MPC tracker ready. Goal stop enabled (tol=%.2fm)", goal_tolerance_m_);
}

void MPCPathTrackerCpp::odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  state_.x = msg->pose.pose.position.x;
  state_.y = msg->pose.pose.position.y;
  state_.yaw = yawFromQuaternion(msg->pose.pose.orientation);
  has_odom_ = true;
}

void MPCPathTrackerCpp::pathCb(const nav_msgs::msg::Path::SharedPtr msg)
{
  path_ = pathFromMsg(*msg);
  has_path_ = !path_.empty();
  goal_reached_latched_ = false;
  RCLCPP_INFO(this->get_logger(), "MPC path updated: %zu points", path_.size());
}

void MPCPathTrackerCpp::teleopModeCb(const std_msgs::msg::Bool::SharedPtr msg)
{
  teleop_mode_ = msg->data;
}

void MPCPathTrackerCpp::obstacleStopCb(const std_msgs::msg::Bool::SharedPtr msg)
{
  obstacle_stop_ = msg->data;
}

void MPCPathTrackerCpp::slopeFactorCb(const std_msgs::msg::Float64::SharedPtr msg)
{
  slope_factor_ = msg->data;
}

double MPCPathTrackerCpp::yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

std::vector<RefPoint> MPCPathTrackerCpp::pathFromMsg(const nav_msgs::msg::Path & msg)
{
  std::vector<RefPoint> out;
  out.reserve(msg.poses.size());

  for (const auto & ps : msg.poses) {
    RefPoint p;
    p.x = ps.pose.position.x;
    p.y = ps.pose.position.y;
    out.push_back(p);
  }

  if (out.size() >= 2) {
    for (size_t i = 0; i < out.size(); ++i) {
      const size_t prev = (i == 0) ? 0 : i - 1;
      const size_t next = (i + 1 >= out.size()) ? out.size() - 1 : i + 1;
      out[i].yaw = std::atan2(out[next].y - out[prev].y, out[next].x - out[prev].x);
    }
  }

  return out;
}

int MPCPathTrackerCpp::findNearestIndex(const VehicleState & state) const
{
  int best_idx = 0;
  double best_d2 = std::numeric_limits<double>::max();
  for (size_t i = 0; i < path_.size(); ++i) {
    const double dx = state.x - path_[i].x;
    const double dy = state.y - path_[i].y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best_idx = static_cast<int>(i);
    }
  }
  return best_idx;
}

bool MPCPathTrackerCpp::isGoalReached(const VehicleState & state) const
{
  if (path_.empty()) {
    return false;
  }
  const auto & g = path_.back();
  return std::hypot(state.x - g.x, state.y - g.y) <= goal_tolerance_m_;
}

void MPCPathTrackerCpp::controlLoop()
{
  geometry_msgs::msg::Twist cmd;

  if (teleop_mode_ || obstacle_stop_ || !has_odom_ || !has_path_ || path_.size() < 2) {
    if (goal_reached_latched_) {
      cmd_pub_->publish(cmd);
    }
    return;
  }

  if (goal_reached_latched_ || isGoalReached(state_)) {
    goal_reached_latched_ = true;
    cmd_pub_->publish(cmd);
    return;
  }

  const int nearest_idx = findNearestIndex(state_);
  const double speed_mps = std::max(0.1, forward_speed_kmh_ / 3.6);
  const auto out = controller_.computeControl(state_, path_, nearest_idx, speed_mps);

  cmd.linear.x = std::abs(forward_speed_kmh_) * (255.0 / 25.5) * slope_factor_;
  cmd.angular.z = out.steer_deg;
  cmd_pub_->publish(cmd);
}

}  // namespace jeju_mpc
