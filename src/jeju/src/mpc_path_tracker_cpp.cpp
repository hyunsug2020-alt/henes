// ============================================================
// mpc_path_tracker_cpp.cpp
//
// bisa-style 3-state LTV-MPC 경로 추종 노드 (ROS2 Humble)
//
// 개선사항 (구 3-state 대비):
//   - κ 하드 제약, VelocityPlanner, α-블렌딩 (ltv_mpc/mpc_controller)
//   - path_hold: |e_y| > threshold 시 정지
//   - oscillation guard: 조향 진동 감지 시 속도 반감
//   - odom 헤딩 직접 사용 (gps_odom_node 가 RTK 위치차분 헤딩 발행)
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
    // ── 차량 / 제어 파라미터 ─────────────────────────────
    declare_parameter("control_rate_hz",    20.0);
    declare_parameter("wheelbase_m",         1.04);
    declare_parameter("vehicle_width_m",     0.70);
    declare_parameter("vehicle_length_m",    1.30);
    declare_parameter("forward_speed_kmh",   3.0);
    declare_parameter("goal_tolerance_m",    1.0);
    declare_parameter("max_steer_deg",       55.0);

    // ── bisa-style LTV-MPC 파라미터 ──────────────────────
    declare_parameter("prediction_horizon",  15);
    declare_parameter("dt",                  0.1);
    declare_parameter("Q_ey",              100.0);
    declare_parameter("Q_epsi",             10.0);
    declare_parameter("Q_kappa",             1.0);
    declare_parameter("QN_ey",            200.0);
    declare_parameter("QN_epsi",           20.0);
    declare_parameter("QN_kappa",           2.0);
    declare_parameter("R_dkappa",            0.5);
    declare_parameter("kappa_max",           0.0);   // 0 = max_steer_deg 로 자동 계산
    declare_parameter("dkappa_max",          1.0);
    declare_parameter("a_lat_max",           1.0);
    declare_parameter("kappa_alpha",         0.40);

    // ── 속도 PI 파라미터 ─────────────────────────────────
    declare_parameter("speed_kp",           30.0);
    declare_parameter("speed_ki",            2.0);
    declare_parameter("speed_integral_max", 15.0);

    // ── nearest 탐색 / cusp ───────────────────────────────
    declare_parameter("nearest_search_window", 80);
    declare_parameter("cusp_cos_threshold",   -0.9999);

    // ── 복구 동작 ─────────────────────────────────────────
    declare_parameter("path_hold_ey_m",       2.0);
    declare_parameter("osc_steer_thresh_deg", 20.0);
    declare_parameter("osc_window",            6);

    // ── 값 로딩 ──────────────────────────────────────────
    control_rate_hz_   = get_parameter("control_rate_hz").as_double();
    forward_speed_kmh_ = get_parameter("forward_speed_kmh").as_double();
    goal_tolerance_m_  = get_parameter("goal_tolerance_m").as_double();
    vehicle_width_m_   = get_parameter("vehicle_width_m").as_double();
    vehicle_length_m_  = get_parameter("vehicle_length_m").as_double();
    wheelbase_m_       = get_parameter("wheelbase_m").as_double();

    controller_params_.wheelbase_m        = wheelbase_m_;
    controller_params_.prediction_horizon = get_parameter("prediction_horizon").as_int();
    controller_params_.dt                 = get_parameter("dt").as_double();
    controller_params_.max_steer_deg      = get_parameter("max_steer_deg").as_double();
    controller_params_.Q_ey               = get_parameter("Q_ey").as_double();
    controller_params_.Q_epsi             = get_parameter("Q_epsi").as_double();
    controller_params_.Q_kappa            = get_parameter("Q_kappa").as_double();
    controller_params_.QN_ey              = get_parameter("QN_ey").as_double();
    controller_params_.QN_epsi            = get_parameter("QN_epsi").as_double();
    controller_params_.QN_kappa           = get_parameter("QN_kappa").as_double();
    controller_params_.R_dkappa           = get_parameter("R_dkappa").as_double();
    controller_params_.kappa_max          = get_parameter("kappa_max").as_double();
    controller_params_.dkappa_max         = get_parameter("dkappa_max").as_double();
    controller_params_.a_lat_max          = get_parameter("a_lat_max").as_double();
    controller_params_.kappa_alpha        = get_parameter("kappa_alpha").as_double();

    speed_kp_             = get_parameter("speed_kp").as_double();
    speed_ki_             = get_parameter("speed_ki").as_double();
    speed_integral_max_   = get_parameter("speed_integral_max").as_double();
    nearest_search_window_= get_parameter("nearest_search_window").as_int();
    cusp_cos_threshold_   = get_parameter("cusp_cos_threshold").as_double();
    path_hold_ey_m_       = get_parameter("path_hold_ey_m").as_double();
    osc_steer_thresh_deg_ = get_parameter("osc_steer_thresh_deg").as_double();
    osc_window_           = get_parameter("osc_window").as_int();

    controller_.updateParameters(controller_params_);

    // ── 토픽 ─────────────────────────────────────────────
    cmd_pub_   = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    debug_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/mpc/debug", 20);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 20,
        std::bind(&MPCPathTrackerCpp::odomCb, this, std::placeholders::_1));

    {
        // /local_path : LocalPathPubCpp 가 슬라이딩 윈도우로 발행
        // /global_path fallback: local_path_pub 미사용 시 직접 구독 가능
        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/local_path", rclcpp::QoS(10),
            std::bind(&MPCPathTrackerCpp::pathCb, this, std::placeholders::_1));
    }

    teleop_mode_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/teleop_mode", 10,
        std::bind(&MPCPathTrackerCpp::teleopModeCb, this, std::placeholders::_1));

    obstacle_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/obstacle_stop", 10,
        std::bind(&MPCPathTrackerCpp::obstacleStopCb, this, std::placeholders::_1));

    slope_factor_sub_ = create_subscription<std_msgs::msg::Float64>(
        "/slope_factor", 10,
        std::bind(&MPCPathTrackerCpp::slopeFactorCb, this, std::placeholders::_1));

    steering_angle_sub_ = create_subscription<std_msgs::msg::Float64>(
        "/steering_angle", 10,
        std::bind(&MPCPathTrackerCpp::steeringAngleCb, this, std::placeholders::_1));

    auto period = std::chrono::duration<double>(1.0 / std::max(control_rate_hz_, 1.0));
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&MPCPathTrackerCpp::controlLoop, this));

    RCLCPP_INFO(get_logger(),
        "bisa-style LTV-MPC tracker 시작  wheelbase=%.2f m  speed=%.1f km/h  "
        "N=%d  dt=%.3f s  a_lat_max=%.2f m/s²  α=%.2f",
        wheelbase_m_, forward_speed_kmh_,
        controller_params_.prediction_horizon, controller_params_.dt,
        controller_params_.a_lat_max, controller_params_.kappa_alpha);
}

// ─────────────────────────────────────────────────────────────
//  오도메트리 콜백
// ─────────────────────────────────────────────────────────────
void MPCPathTrackerCpp::odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    state_.x = msg->pose.pose.position.x;
    state_.y = msg->pose.pose.position.y;

    // gps_odom_node 가 RTK 위치차분 헤딩을 quaternion 으로 발행
    // → odom quaternion 직접 사용
    state_.yaw = yawFromQuaternion(msg->pose.pose.orientation);

    // 현재 조향각 → 곡률
    const double steer_rad = current_steer_deg_ * M_PI / 180.0;
    state_.kappa   = std::tan(steer_rad) / std::max(wheelbase_m_, 0.01);
    state_.delta_c = steer_rad;

    // world → body frame 속도 변환
    const double vwx = msg->twist.twist.linear.x;
    const double vwy = msg->twist.twist.linear.y;
    const double cy  = std::cos(state_.yaw);
    const double sy  = std::sin(state_.yaw);
    state_.vx = vwx * cy + vwy * sy;
    state_.vy = -vwx * sy + vwy * cy;

    // GPS 속도 크기 (부호: direction profile 로 결정)
    gps_speed_mps_ = std::hypot(vwx, vwy);

    const double omega_twist = msg->twist.twist.angular.z;
    state_.omega = (std::abs(omega_twist) > 1e-6) ?
        omega_twist : state_.vx * state_.kappa;

    has_odom_ = true;
}

// ─────────────────────────────────────────────────────────────
//  경로 콜백
// ─────────────────────────────────────────────────────────────
void MPCPathTrackerCpp::pathCb(const nav_msgs::msg::Path::SharedPtr msg)
{
    auto new_path = pathFromMsg(*msg);

    // LocalPathPubCpp 는 항상 같은 크기(local_path_size)의 슬라이딩 윈도우를 발행
    // → 크기가 같으면 슬라이딩 업데이트: nearest_idx 만 0 리셋 (PI 상태 유지)
    // → 크기가 다르면 진짜 새 경로: 모든 상태 리셋
    const bool same_size = has_path_ && (new_path.size() == path_.size());
    path_     = std::move(new_path);
    has_path_ = !path_.empty();
    goal_reached_latched_ = false;

    if (!same_size) {
        // 진짜 새 경로 (글로벌 경로 변경 등) → 전체 리셋
        has_prev_output_  = false;
        speed_integral_   = 0.0;
        nearest_idx_prev_ = 0;
        current_direction_= 1;
        steer_history_.clear();
    } else {
        // 슬라이딩 업데이트: 로컬 윈도우가 차량 위치 기준으로 재시작됐으므로
        // nearest_idx 를 0 으로 리셋 (윈도우 앞쪽에서 탐색 시작)
        nearest_idx_prev_ = 0;
    }

    // direction_profile 구성 (cusp 감지)
    const int n = static_cast<int>(path_.size());
    direction_profile_.assign(n, 1);

    if (n >= 3) {
        std::vector<int> cusps;
        for (int i = 1; i < n - 1; ++i) {
            const double v1x = path_[i].x - path_[i-1].x;
            const double v1y = path_[i].y - path_[i-1].y;
            const double v2x = path_[i+1].x - path_[i].x;
            const double v2y = path_[i+1].y - path_[i].y;
            const double m1  = std::hypot(v1x, v1y);
            const double m2  = std::hypot(v2x, v2y);
            if (m1 < 0.01 || m2 < 0.01) continue;
            const double cosA = (v1x*v2x + v1y*v2y) / (m1 * m2);
            if (cosA < cusp_cos_threshold_) cusps.push_back(i);
        }

        int dir = 1, ptr = 0;
        for (int i = 0; i < n; ++i) {
            if (ptr < static_cast<int>(cusps.size()) && i >= cusps[ptr]) {
                dir *= -1; ++ptr;
            }
            direction_profile_[i] = dir;
        }
    }

    for (int i = 0; i < n; ++i) {
        if (direction_profile_[i] == -1) {
            path_[i].yaw = MPCControllerCpp::normalizeAngle(path_[i].yaw + M_PI);
        }
    }

    RCLCPP_INFO(get_logger(), "MPC 경로 수신: %d 포인트", n);
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

// ─────────────────────────────────────────────────────────────
//  유틸
// ─────────────────────────────────────────────────────────────
double MPCPathTrackerCpp::yawFromQuaternion(
    const geometry_msgs::msg::Quaternion & q)
{
    return std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

std::vector<RefPoint> MPCPathTrackerCpp::pathFromMsg(
    const nav_msgs::msg::Path & msg)
{
    std::vector<RefPoint> out;
    out.reserve(msg.poses.size());

    for (const auto & ps : msg.poses) {
        RefPoint p;
        p.x       = ps.pose.position.x;
        p.y       = ps.pose.position.y;
        p.kappa_r = ps.pose.orientation.x;   // 관례: kappa_r은 orientation.x 에 저장
        out.push_back(p);
    }

    for (size_t i = 0; i < out.size(); ++i) {
        const auto & q     = msg.poses[i].pose.orientation;
        const double q_norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
        if (std::abs(q_norm - 1.0) < 0.01) {
            out[i].yaw = yawFromQuaternion(q);
            continue;
        }
        if (out.size() >= 2) {
            const size_t prev = (i == 0) ? 0 : i - 1;
            const size_t next = (i + 1 < out.size()) ? i + 1 : out.size() - 1;
            out[i].yaw = std::atan2(out[next].y - out[prev].y,
                                    out[next].x - out[prev].x);
        }
    }
    return out;
}

int MPCPathTrackerCpp::findNearestIndex(
    const VehicleState & state, int prev_idx) const
{
    const int n = static_cast<int>(path_.size());
    const double veh_cos = std::cos(state.yaw);
    const double veh_sin = std::sin(state.yaw);

    const int start = prev_idx;
    const int end   = std::min(n - 1, prev_idx + nearest_search_window_);

    int best_hdg  = prev_idx;
    int best_dist = prev_idx;
    double best_d2_hdg  = std::numeric_limits<double>::max();
    double best_d2_dist = std::numeric_limits<double>::max();

    for (int i = start; i <= end; ++i) {
        const double dx = state.x - path_[i].x;
        const double dy = state.y - path_[i].y;
        const double d2 = dx*dx + dy*dy;

        if (d2 < best_d2_dist) { best_d2_dist = d2; best_dist = i; }

        const double path_cos = std::cos(path_[i].yaw);
        const double path_sin = std::sin(path_[i].yaw);
        if (veh_cos * path_cos + veh_sin * path_sin < 0.0) continue;

        if (d2 < best_d2_hdg) { best_d2_hdg = d2; best_hdg = i; }
    }

    return (best_d2_hdg < std::numeric_limits<double>::max()) ? best_hdg : best_dist;
}

bool MPCPathTrackerCpp::isGoalReached(const VehicleState & state) const
{
    if (path_.empty()) return false;
    return std::hypot(state.x - path_.back().x,
                      state.y - path_.back().y) <= goal_tolerance_m_;
}

// ─────────────────────────────────────────────────────────────
//  제어 루프
// ─────────────────────────────────────────────────────────────
void MPCPathTrackerCpp::controlLoop()
{
    geometry_msgs::msg::Twist cmd;   // 기본: 0 (정지)

    if (teleop_mode_ || obstacle_stop_ || !has_odom_ || !has_path_ || path_.size() < 2) {
        has_prev_output_ = false;
        speed_integral_  = 0.0;
        steer_history_.clear();
        if (goal_reached_latched_) cmd_pub_->publish(cmd);
        return;
    }

    if (goal_reached_latched_ || isGoalReached(state_)) {
        goal_reached_latched_ = true;
        has_prev_output_ = false;
        speed_integral_  = 0.0;
        steer_history_.clear();
        cmd_pub_->publish(cmd);
        return;
    }

    const int nearest_idx = findNearestIndex(state_, nearest_idx_prev_);

    // direction profile
    int dir = 1;
    if (!direction_profile_.empty() &&
        nearest_idx < static_cast<int>(direction_profile_.size()))
    {
        dir = direction_profile_[nearest_idx];
    }
    current_direction_ = dir;

    // cusp point 정지
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

    // ── MPC 계산 (이전 출력 먼저 발행: 1-sample delay 보상) ──
    const double kmh       = std::abs(forward_speed_kmh_);
    const double speed_mps = static_cast<double>(dir) * kmh / 3.6;

    // ── MPC 계산 ────────────────────────────────────────
    const auto cur_out = controller_.computeControl(
        state_, path_, nearest_idx, speed_mps);

    // ── path_hold 복구: 횡오차 초과 시 정지 ─────────────
    const bool path_hold = (std::abs(cur_out.e_y) > path_hold_ey_m_);

    // ── oscillation guard: 조향 진동 감지 ───────────────
    steer_history_.push_back(cur_out.steer_deg);
    if (static_cast<int>(steer_history_.size()) > osc_window_) {
        steer_history_.pop_front();
    }
    bool oscillating = false;
    if (static_cast<int>(steer_history_.size()) >= osc_window_) {
        int sign_changes = 0;
        for (int i = 1; i < static_cast<int>(steer_history_.size()); ++i) {
            if (std::abs(steer_history_[i] - steer_history_[i-1]) > osc_steer_thresh_deg_) {
                ++sign_changes;
            }
        }
        oscillating = (sign_changes >= osc_window_ / 2);
    }

    // ── 속도 PI 제어 ─────────────────────────────────────
    const double v_actual = static_cast<double>(dir) * gps_speed_mps_;
    const double v_error  = speed_mps - v_actual;
    const double dt_ctrl  = 1.0 / std::max(control_rate_hz_, 1.0);
    speed_integral_ += v_error * dt_ctrl;
    speed_integral_  = std::clamp(speed_integral_, -speed_integral_max_, speed_integral_max_);
    double speed_pwm = std::clamp(
        (speed_kp_ * v_error + speed_ki_ * speed_integral_) * slope_factor_,
        -255.0, 255.0);

    // path_hold 적용: 정지
    if (path_hold) {
        speed_pwm = 0.0;
        speed_integral_ = 0.0;
    }
    // oscillation guard: 속도 반감
    if (oscillating) {
        speed_pwm *= 0.5;
    }

    // 1-sample delay: 이전 계산 결과 발행
    if (has_prev_output_) {
        cmd.linear.x  = speed_pwm;
        cmd.angular.z = prev_output_.steer_deg;
        cmd.linear.y  = prev_output_.torque_cmd;
        cmd_pub_->publish(cmd);
    }

    // 진단 로그 (2초마다)
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "[MPC] idx=%d  pos=(%.2f,%.2f)  yaw=%.1f°  "
        "e_y=%.3fm  e_psi=%.1f°  κ=%.3f  steer=%.1f°  "
        "spd_pwm=%.1f  v=%.2fm/s  %s%s",
        cur_out.nearest_idx,
        state_.x, state_.y, state_.yaw * 180.0 / M_PI,
        cur_out.e_y, cur_out.e_psi * 180.0 / M_PI,
        cur_out.curvature, cur_out.steer_deg,
        speed_pwm, v_actual,
        path_hold  ? "[PATH_HOLD] " : "",
        oscillating ? "[OSC_GUARD]"  : "");

    prev_output_     = cur_out;
    has_prev_output_ = true;

    // /mpc/debug 발행
    // [0]=e_y  [1]=e_psi(deg)  [2]=target_v  [3]=speed_pwm
    // [4]=nearest_idx  [5]=direction  [6]=steer_cmd  [7]=goal  [8]=has_path
    // [9]=curvature  [10]=path_hold  [11]=oscillating
    {
        std_msgs::msg::Float64MultiArray dbg;
        dbg.data = {
            cur_out.e_y,
            cur_out.e_psi * 180.0 / M_PI,
            speed_mps,
            speed_pwm,
            static_cast<double>(cur_out.nearest_idx),
            static_cast<double>(current_direction_),
            cur_out.steer_deg,
            goal_reached_latched_ ? 1.0 : 0.0,
            has_path_ ? 1.0 : 0.0,
            cur_out.curvature,
            path_hold  ? 1.0 : 0.0,
            oscillating ? 1.0 : 0.0,
        };
        debug_pub_->publish(dbg);
    }
}

}  // namespace jeju_mpc
