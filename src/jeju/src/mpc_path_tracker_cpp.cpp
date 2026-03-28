// ============================================================
// mpc_path_tracker_cpp.cpp
//
// Frenet-frame 3-state LTV-MPC 경로 추종 노드
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
    this->declare_parameter("gps_vel_topic",       std::string("/gnss_left/fix_velocity"));
    this->declare_parameter("gps_heading_min_mps", 0.3);
    this->declare_parameter("control_rate_hz",    20.0);
    this->declare_parameter("wheelbase_m",         1.04);
    this->declare_parameter("vehicle_width_m",     0.70);
    this->declare_parameter("vehicle_length_m",    1.30);
    this->declare_parameter("forward_speed_kmh",   8.0);
    this->declare_parameter("goal_tolerance_m",    1.0);
    this->declare_parameter("max_steer_deg",       90.0);

    // ── LTV-MPC 파라미터 ─────────────────────────────────
    this->declare_parameter("prediction_horizon",  10);
    this->declare_parameter("dt",                  0.05);
    this->declare_parameter("Q_ey",                10.0);
    this->declare_parameter("Q_epsi",               8.0);
    this->declare_parameter("Q_v",                  1.0);
    this->declare_parameter("QN_ey",               20.0);
    this->declare_parameter("QN_epsi",             16.0);
    this->declare_parameter("QN_v",                 2.0);
    this->declare_parameter("R_delta",              0.5);
    this->declare_parameter("R_a",                  0.1);
    this->declare_parameter("a_max",                2.0);
    this->declare_parameter("a_min",               -3.0);
    this->declare_parameter("kappa_speed_factor",   3.0);

    // ── 속도 PI 파라미터 ─────────────────────────────────
    this->declare_parameter("speed_kp",            30.0);
    this->declare_parameter("speed_ki",             2.0);
    this->declare_parameter("speed_integral_max",  15.0);

    // ── 값 로딩 ──────────────────────────────────────────
    control_rate_hz_   = this->get_parameter("control_rate_hz").as_double();
    forward_speed_kmh_ = this->get_parameter("forward_speed_kmh").as_double();
    goal_tolerance_m_  = this->get_parameter("goal_tolerance_m").as_double();
    vehicle_width_m_   = this->get_parameter("vehicle_width_m").as_double();
    vehicle_length_m_  = this->get_parameter("vehicle_length_m").as_double();
    wheelbase_m_       = this->get_parameter("wheelbase_m").as_double();

    controller_params_.wheelbase_m        = wheelbase_m_;
    controller_params_.prediction_horizon = this->get_parameter("prediction_horizon").as_int();
    controller_params_.dt                 = this->get_parameter("dt").as_double();
    controller_params_.max_steer_deg      = this->get_parameter("max_steer_deg").as_double();
    controller_params_.Q_ey               = this->get_parameter("Q_ey").as_double();
    controller_params_.Q_epsi             = this->get_parameter("Q_epsi").as_double();
    controller_params_.Q_v                = this->get_parameter("Q_v").as_double();
    controller_params_.QN_ey              = this->get_parameter("QN_ey").as_double();
    controller_params_.QN_epsi            = this->get_parameter("QN_epsi").as_double();
    controller_params_.QN_v               = this->get_parameter("QN_v").as_double();
    controller_params_.R_delta            = this->get_parameter("R_delta").as_double();
    controller_params_.R_a                = this->get_parameter("R_a").as_double();
    controller_params_.a_max              = this->get_parameter("a_max").as_double();
    controller_params_.a_min              = this->get_parameter("a_min").as_double();
    controller_params_.kappa_speed_factor = this->get_parameter("kappa_speed_factor").as_double();

    speed_kp_           = this->get_parameter("speed_kp").as_double();
    speed_ki_           = this->get_parameter("speed_ki").as_double();
    speed_integral_max_ = this->get_parameter("speed_integral_max").as_double();

    controller_.updateParameters(controller_params_);

    const double gps_hdg_min = this->get_parameter("gps_heading_min_mps").as_double();

    // ── 토픽 ─────────────────────────────────────────────
    cmd_pub_   = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    debug_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/mpc/debug", 20);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 20,
        std::bind(&MPCPathTrackerCpp::odomCb, this, std::placeholders::_1));

    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
            .reliable()
            .transient_local();
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/global_path", qos,
            std::bind(&MPCPathTrackerCpp::pathCb, this, std::placeholders::_1));
    }

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

    // GPS 속도 기반 헤딩 구독 (ESKF 헤딩 대체)
    const auto gps_vel_topic = this->get_parameter("gps_vel_topic").as_string();
    gps_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        gps_vel_topic, 20,
        [this, gps_hdg_min](const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
            const double vx = msg->twist.twist.linear.x;
            const double vy = msg->twist.twist.linear.y;
            gps_speed_mps_ = std::hypot(vx, vy);
            if (gps_speed_mps_ >= gps_hdg_min) {
                gps_yaw_     = std::atan2(vy, vx);
                has_gps_yaw_ = true;
            }
        });

    auto period = std::chrono::duration<double>(1.0 / std::max(control_rate_hz_, 1.0));
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&MPCPathTrackerCpp::controlLoop, this));

    RCLCPP_INFO(
        this->get_logger(),
        "LTV-MPC tracker ready. wheelbase=%.2f m  speed=%.1f km/h  N=%d  dt=%.3f s",
        wheelbase_m_, forward_speed_kmh_,
        controller_params_.prediction_horizon, controller_params_.dt);
}

void MPCPathTrackerCpp::odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // 위치: ESKF (GPS RTK 융합)
    state_.x = msg->pose.pose.position.x;
    state_.y = msg->pose.pose.position.y;

    // 헤딩: GPS 속도 기반 (이동 중) / ESKF fallback (정지 시)
    const double yaw = has_gps_yaw_ ? gps_yaw_
                                    : yawFromQuaternion(msg->pose.pose.orientation);
    state_.yaw = yaw;

    const double steer_rad = current_steer_deg_ * M_PI / 180.0;
    state_.kappa   = std::tan(steer_rad) / std::max(wheelbase_m_, 0.01);
    state_.delta_c = steer_rad;

    // world → body frame 속도 변환
    const double vwx = msg->twist.twist.linear.x;
    const double vwy = msg->twist.twist.linear.y;
    const double cy  = std::cos(yaw);
    const double sy  = std::sin(yaw);
    state_.vx = vwx * cy + vwy * sy;
    state_.vy = -vwx * sy + vwy * cy;

    const double omega_twist = msg->twist.twist.angular.z;
    state_.omega = (std::abs(omega_twist) > 1e-6) ?
        omega_twist : state_.vx * state_.kappa;

    has_odom_ = true;
}

void MPCPathTrackerCpp::pathCb(const nav_msgs::msg::Path::SharedPtr msg)
{
    auto new_path = pathFromMsg(*msg);
    // 경로 포인트 수가 같으면 동일 경로 재발행 → PI 상태 유지 (끊김 방지)
    const bool same_path = has_path_ && (new_path.size() == path_.size());
    path_ = std::move(new_path);
    has_path_ = !path_.empty();
    goal_reached_latched_ = false;
    if (!same_path) {
        has_prev_output_ = false;
        speed_integral_ = 0.0;
        nearest_idx_prev_ = 0;
        current_direction_ = 1;
    }

    // direction_profile 분석 (후진 구간 감지)
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
            if (m1 < 0.01 || m2 < 0.01) { continue; }
            const double cosA = (v1x * v2x + v1y * v2y) / (m1 * m2);
            if (cosA < kCuspThreshold) { cusp_indices.push_back(i); }
        }

        int dir = 1, ptr = 0;
        for (int i = 0; i < n; ++i) {
            if (ptr < static_cast<int>(cusp_indices.size()) && i >= cusp_indices[ptr]) {
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
        p.x       = ps.pose.position.x;
        p.y       = ps.pose.position.y;
        p.kappa_r = ps.pose.orientation.x;
        out.push_back(p);
    }

    for (size_t i = 0; i < out.size(); ++i) {
        const auto & q = msg.poses[i].pose.orientation;
        const double q_norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
        const bool has_orientation = std::abs(q_norm - 1.0) < 0.01;

        if (has_orientation) {
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

int MPCPathTrackerCpp::findNearestIndex(const VehicleState & state, int prev_idx) const
{
    const int n = static_cast<int>(path_.size());

    // 차량 헤딩 벡터
    const double veh_cos = std::cos(state.yaw);
    const double veh_sin = std::sin(state.yaw);

    // 헤딩 일치 필터 + 전방 탐색:
    //   - 전체 전방 경로를 탐색하되 (LTV-MPC 특성 유지)
    //   - 차량과 경로 방향이 반대인 포인트는 제외 (U자/루프 지름길 방지)
    //   - 헤딩 필터로 후보가 없으면 순수 거리 최소 포인트 사용 (fallback)
    const int start = std::max(0,     prev_idx - 5);
    const int end   = n - 1;

    int best_hdg  = prev_idx;
    int best_dist = prev_idx;
    double best_d2_hdg  = std::numeric_limits<double>::max();
    double best_d2_dist = std::numeric_limits<double>::max();

    for (int i = start; i <= end; ++i) {
        const double dx = state.x - path_[i].x;
        const double dy = state.y - path_[i].y;
        const double d2 = dx*dx + dy*dy;

        // 전체 거리 최소 (fallback용)
        if (d2 < best_d2_dist) { best_d2_dist = d2; best_dist = i; }

        // 경로 방향과 차량 방향의 내적 > 0 → 같은 방향 포인트만 허용
        const double path_cos = std::cos(path_[i].yaw);
        const double path_sin = std::sin(path_[i].yaw);
        if (veh_cos * path_cos + veh_sin * path_sin < 0.0) { continue; }

        if (d2 < best_d2_hdg) { best_d2_hdg = d2; best_hdg = i; }
    }

    // 헤딩 일치 후보가 있으면 사용, 없으면 거리 최소 fallback
    return (best_d2_hdg < std::numeric_limits<double>::max()) ? best_hdg : best_dist;
}

bool MPCPathTrackerCpp::isGoalReached(const VehicleState & state) const
{
    if (path_.empty()) { return false; }
    return std::hypot(state.x - path_.back().x,
                      state.y - path_.back().y) <= goal_tolerance_m_;
}

void MPCPathTrackerCpp::controlLoop()
{
    geometry_msgs::msg::Twist cmd;

    if (teleop_mode_ || obstacle_stop_ || !has_odom_ || !has_path_ || path_.size() < 2) {
        has_prev_output_ = false;
        speed_integral_  = 0.0;
        if (goal_reached_latched_) { cmd_pub_->publish(cmd); }
        return;
    }

    if (goal_reached_latched_ || isGoalReached(state_)) {
        goal_reached_latched_ = true;
        has_prev_output_  = false;
        speed_integral_   = 0.0;
        cmd_pub_->publish(cmd);
        return;
    }

    const int nearest_idx = findNearestIndex(state_, nearest_idx_prev_);

    int dir = 1;
    if (!direction_profile_.empty() &&
        nearest_idx < static_cast<int>(direction_profile_.size()))
    {
        dir = direction_profile_[nearest_idx];
    }
    current_direction_ = dir;

    // cusp point에서 일시 정지
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

    // 속도 PI 제어기
    const double kmh       = std::abs(forward_speed_kmh_);
    const double speed_mps = static_cast<double>(dir) * kmh / 3.6;
    const double v_actual  = state_.vx;
    const double v_error   = speed_mps - v_actual;
    const double dt        = 1.0 / std::max(control_rate_hz_, 1.0);
    speed_integral_ += v_error * dt;
    speed_integral_  = std::clamp(speed_integral_, -speed_integral_max_, speed_integral_max_);
    const double speed_pwm = std::clamp(
        (speed_kp_ * v_error + speed_ki_ * speed_integral_) * slope_factor_,
        -255.0, 255.0);

    // 1-sample computation delay: 이전 최적해 실행, 현재 최적해 계산
    if (has_prev_output_) {
        cmd.linear.x  = speed_pwm;
        cmd.angular.z = prev_output_.steer_deg;
        cmd.linear.y  = prev_output_.torque_cmd;
        cmd_pub_->publish(cmd);
    }

    const auto cur_out = controller_.computeControl(state_, path_, nearest_idx, speed_mps);

    // ── 2초마다 진단 로그 ─────────────────────────────────
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "[MPC] idx=%d  pos=(%.2f,%.2f)  yaw=%.1f°  "
        "e_y=%.3f m  e_psi=%.1f°  steer=%.1f°  spd_pwm=%.1f  v=%.2f m/s",
        cur_out.nearest_idx,
        state_.x, state_.y,
        state_.yaw * 180.0 / M_PI,
        cur_out.e_y,
        cur_out.e_psi * 180.0 / M_PI,
        cur_out.steer_deg,
        speed_pwm,
        v_actual);

    prev_output_     = cur_out;
    has_prev_output_ = true;

    // ── /mpc/debug 발행 ──────────────────────────────────
    // [0]=e_y(m)  [1]=e_psi(deg)  [2]=target_v(m/s signed)
    // [3]=speed_pwm  [4]=nearest_idx  [5]=direction(1/-1)
    // [6]=steer_cmd(deg)  [7]=goal_reached  [8]=has_path
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
        };
        debug_pub_->publish(dbg);
    }
}

}  // namespace jeju_mpc
