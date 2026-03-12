// ============================================================
// mpc_path_tracker_cpp.cpp
//
// 변경사항 (3단계):
//   odomCb: world→body frame 속도 변환 (vx, vy, omega)
//   pathCb: direction_profile 분석 (path_follower_node.py 동일 로직)
//   controlLoop: 전진/후진 speed_mps 부호 자동 처리
//   DBM 파라미터 전체 로딩
// ============================================================
#include "jeju_mpc/mpc_path_tracker_cpp.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace jeju_mpc
{

MPCPathTrackerCpp::MPCPathTrackerCpp()
: Node("mpc_path_tracker_cpp")
{
  // ── 기존 파라미터 ────────────────────────────────
  this->declare_parameter("control_rate_hz", 20.0);
  this->declare_parameter("wheelbase_m", 1.04);
  this->declare_parameter("vehicle_width_m", 0.70);
  this->declare_parameter("vehicle_length_m", 1.30);
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

  // ── DBM 파라미터 ─────────────────────────────────
  this->declare_parameter("use_dbm", true);
  this->declare_parameter("mass_kg", 150.0);
  this->declare_parameter("inertia_kgm2", 120.0);
  this->declare_parameter("lf_m", 0.52);
  this->declare_parameter("lr_m", 0.52);
  this->declare_parameter("Ca_N_rad", 25000.0);
  this->declare_parameter("Cx_N", 30000.0);
  this->declare_parameter("Jw_kgm2", 0.05);
  this->declare_parameter("r_w_m", 0.08);
  this->declare_parameter("v_delta_max", 0.5);
  this->declare_parameter("v_min_mps", 0.3);
  this->declare_parameter("mu", 0.8);
  this->declare_parameter("w_xy", 2.0);
  this->declare_parameter("w_psi", 1.0);
  this->declare_parameter("w_vx", 0.5);
  this->declare_parameter("w_vy", 0.3);
  this->declare_parameter("w_omega", 0.3);
  this->declare_parameter("w_delta_c", 0.1);
  this->declare_parameter("w_wheel", 0.01);
  this->declare_parameter("w_u_delta", 0.2);
  this->declare_parameter("w_u_torque", 1e-5);

  // ── 기존 파라미터 로딩 ───────────────────────────
  control_rate_hz_ = this->get_parameter("control_rate_hz").as_double();
  forward_speed_kmh_ = this->get_parameter("forward_speed_kmh").as_double();
  goal_tolerance_m_ = this->get_parameter("goal_tolerance_m").as_double();
  vehicle_width_m_ = this->get_parameter("vehicle_width_m").as_double();
  vehicle_length_m_ = this->get_parameter("vehicle_length_m").as_double();
  wheelbase_m_ = this->get_parameter("wheelbase_m").as_double();

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

  // ── DBM 파라미터 로딩 ────────────────────────────
  controller_params_.use_dbm = this->get_parameter("use_dbm").as_bool();
  controller_params_.mass_kg = this->get_parameter("mass_kg").as_double();
  controller_params_.inertia_kgm2 = this->get_parameter("inertia_kgm2").as_double();
  controller_params_.lf_m = this->get_parameter("lf_m").as_double();
  controller_params_.lr_m = this->get_parameter("lr_m").as_double();
  controller_params_.Ca_N_rad = this->get_parameter("Ca_N_rad").as_double();
  controller_params_.Cx_N = this->get_parameter("Cx_N").as_double();
  controller_params_.Jw_kgm2 = this->get_parameter("Jw_kgm2").as_double();
  controller_params_.r_w_m = this->get_parameter("r_w_m").as_double();
  controller_params_.v_delta_max = this->get_parameter("v_delta_max").as_double();
  controller_params_.v_min_mps = this->get_parameter("v_min_mps").as_double();
  controller_params_.mu = this->get_parameter("mu").as_double();
  controller_params_.w_xy = this->get_parameter("w_xy").as_double();
  controller_params_.w_psi = this->get_parameter("w_psi").as_double();
  controller_params_.w_vx = this->get_parameter("w_vx").as_double();
  controller_params_.w_vy = this->get_parameter("w_vy").as_double();
  controller_params_.w_omega = this->get_parameter("w_omega").as_double();
  controller_params_.w_delta_c = this->get_parameter("w_delta_c").as_double();
  controller_params_.w_wheel = this->get_parameter("w_wheel").as_double();
  controller_params_.w_u_delta = this->get_parameter("w_u_delta").as_double();
  controller_params_.w_u_torque = this->get_parameter("w_u_torque").as_double();
  r_w_m_ = controller_params_.r_w_m;

  controller_.updateParameters(controller_params_);

  // ── 토픽 (기존과 동일) ───────────────────────────
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

  steering_angle_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/steering_angle", 10,
    std::bind(&MPCPathTrackerCpp::steeringAngleCb, this, std::placeholders::_1));

  auto period = std::chrono::duration<double>(1.0 / std::max(control_rate_hz_, 1.0));
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&MPCPathTrackerCpp::controlLoop, this));

  RCLCPP_INFO(
    this->get_logger(),
    "MPC tracker ready. wheelbase=%.2f use_dbm=%s",
    wheelbase_m_,
    controller_params_.use_dbm ? "true" : "false");
}

void MPCPathTrackerCpp::odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const double yaw = yawFromQuaternion(msg->pose.pose.orientation);

  state_.x = msg->pose.pose.position.x;
  state_.y = msg->pose.pose.position.y;
  state_.yaw = yaw;

  const double steer_rad = current_steer_deg_ * 3.14159265358979323846 / 180.0;
  state_.kappa = std::tan(steer_rad) / std::max(wheelbase_m_, 0.01);

  // world → body frame 변환
  // status_info_node.py: twist.linear.x/y = GPS world frame 속도
  const double vwx = msg->twist.twist.linear.x;
  const double vwy = msg->twist.twist.linear.y;
  current_vx_world_ = vwx;
  current_vy_world_ = vwy;

  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);

  state_.vx = vwx * cy + vwy * sy;
  state_.vy = -vwx * sy + vwy * cy;

  const double omega_twist = msg->twist.twist.angular.z;
  state_.omega = (std::abs(omega_twist) > 1e-6) ?
    omega_twist : state_.vx * state_.kappa;

  state_.delta_c = steer_rad;

  const double rw = std::max(r_w_m_, 0.01);
  state_.omega_f = state_.vx / rw;
  state_.omega_r = state_.vx / rw;

  has_odom_ = true;
}

void MPCPathTrackerCpp::pathCb(const nav_msgs::msg::Path::SharedPtr msg)
{
  path_ = pathFromMsg(*msg);
  has_path_ = !path_.empty();
  goal_reached_latched_ = false;
  has_prev_output_ = false;
  speed_integral_ = 0.0;
  nearest_idx_prev_ = 0;
  current_direction_ = 1;

  // direction_profile 분석 (path_follower_node.py 동일 로직)
  const int n = static_cast<int>(path_.size());
  direction_profile_.assign(n, 1);

  if (n >= 3) {
    constexpr double kCuspThreshold = -0.5;
    std::vector<int> cusp_indices;
    for (int i = 1; i < n - 1; ++i) {
      const double v1x = path_[i].x - path_[i - 1].x;
      const double v1y = path_[i].y - path_[i - 1].y;
      const double v2x = path_[i + 1].x - path_[i].x;
      const double v2y = path_[i + 1].y - path_[i].y;
      const double m1 = std::hypot(v1x, v1y);
      const double m2 = std::hypot(v2x, v2y);
      if (m1 < 0.01 || m2 < 0.01) {
        continue;
      }
      const double cosA = (v1x * v2x + v1y * v2y) / (m1 * m2);
      if (cosA < kCuspThreshold) {
        cusp_indices.push_back(i);
      }
    }

    int dir = 1;
    int ptr = 0;
    for (int i = 0; i < n; ++i) {
      if (ptr < static_cast<int>(cusp_indices.size()) && i >= cusp_indices[ptr]) {
        dir *= -1;
        ++ptr;
      }
      direction_profile_[i] = dir;
    }
  }

  for (int i = 0; i < n; ++i) {
    if (direction_profile_[i] == -1) {
      path_[i].yaw = MPCControllerCpp::normalizeAngle(path_[i].yaw + M_PI);
    }
  }

  RCLCPP_INFO(this->get_logger(), "MPC path: %d points", n);
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

void MPCPathTrackerCpp::steeringAngleCb(const std_msgs::msg::Float64::SharedPtr msg)
{
  current_steer_deg_ = msg->data;
}

double MPCPathTrackerCpp::yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

std::vector<RefPoint> MPCPathTrackerCpp::pathFromMsg(const nav_msgs::msg::Path & msg)
{
  std::vector<RefPoint> out;
  out.reserve(msg.poses.size());

  for (const auto & ps : msg.poses) {
    RefPoint p;
    p.x = ps.pose.position.x;
    p.y = ps.pose.position.y;
    p.kappa_r = ps.pose.orientation.x;
    out.push_back(p);
  }

  for (size_t i = 0; i < out.size(); ++i) {
    const auto & q = msg.poses[i].pose.orientation;
    const double q_norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    const bool has_orientation = std::abs(q_norm - 1.0) < 0.01;

    if (has_orientation) {
      out[i].yaw = yawFromQuaternion(q);
      continue;
    }

    if (out.size() >= 2) {
      const size_t prev = (i == 0) ? 0 : i - 1;
      const size_t next = (i + 1 < out.size()) ? i + 1 : out.size() - 1;
      out[i].yaw = std::atan2(out[next].y - out[prev].y, out[next].x - out[prev].x);
    }
  }

  return out;
}

int MPCPathTrackerCpp::findNearestIndex(const VehicleState & state) const
{
  int best = 0;
  double best_d2 = std::numeric_limits<double>::max();
  for (size_t i = 0; i < path_.size(); ++i) {
    const double dx = state.x - path_[i].x;
    const double dy = state.y - path_[i].y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best = static_cast<int>(i);
    }
  }
  return best;
}

bool MPCPathTrackerCpp::isGoalReached(const VehicleState & state) const
{
  if (path_.empty()) {
    return false;
  }
  return std::hypot(state.x - path_.back().x, state.y - path_.back().y) <= goal_tolerance_m_;
}

void MPCPathTrackerCpp::controlLoop()
{
  geometry_msgs::msg::Twist cmd;

  if (teleop_mode_ || obstacle_stop_ || !has_odom_ || !has_path_ || path_.size() < 2) {
    has_prev_output_ = false;
    speed_integral_ = 0.0;
    if (goal_reached_latched_) {
      cmd_pub_->publish(cmd);
    }
    return;
  }

  if (goal_reached_latched_ || isGoalReached(state_)) {
    goal_reached_latched_ = true;
    has_prev_output_ = false;
    speed_integral_ = 0.0;
    cmd_pub_->publish(cmd);
    return;
  }

  const int nearest_idx = findNearestIndex(state_);

  int dir = 1;
  if (!direction_profile_.empty() &&
    nearest_idx < static_cast<int>(direction_profile_.size()))
  {
    dir = direction_profile_[nearest_idx];
  }
  current_direction_ = dir;

  // cusp point 도달 시 정지 후 다음 스텝에서 방향 전환
  if (nearest_idx > 0 &&
    nearest_idx < static_cast<int>(direction_profile_.size()) &&
    nearest_idx_prev_ < nearest_idx)
  {
    if (direction_profile_[nearest_idx - 1] != direction_profile_[nearest_idx]) {
      cmd_pub_->publish(cmd);
      nearest_idx_prev_ = nearest_idx;
      return;
    }
  }
  nearest_idx_prev_ = nearest_idx;

  const double kmh = std::abs(forward_speed_kmh_);
  const double speed_mps = static_cast<double>(dir) * kmh / 3.6;
  const double v_actual = state_.vx;
  const double v_target = speed_mps;
  const double v_error = v_target - v_actual;
  const double dt = 1.0 / std::max(control_rate_hz_, 1.0);
  constexpr double kSpeedKp = 10.0;
  constexpr double kSpeedKi = 2.0;
  speed_integral_ += v_error * dt;
  speed_integral_ = std::clamp(speed_integral_, -100.0, 100.0);
  const double speed_pwm = std::clamp(
    (kSpeedKp * v_error + kSpeedKi * speed_integral_) * slope_factor_,
    -255.0, 255.0);

  // 1-sample computation delay:
  // execute the previously optimized input, then compute the next one.
  if (has_prev_output_) {
    cmd.linear.x = speed_pwm;
    cmd.angular.z = prev_output_.steer_deg;
    cmd.linear.y = prev_output_.torque_cmd;
    cmd_pub_->publish(cmd);
  }

  prev_output_ = controller_.computeControl(state_, path_, nearest_idx, speed_mps);
  has_prev_output_ = true;
}

}  // namespace jeju_mpc
