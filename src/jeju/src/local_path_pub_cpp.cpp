// ============================================================
// local_path_pub_cpp.cpp
//
// bisa LocalPathPubCpp → HENES ROS2 Humble 이식
//
// 핵심 로직:
//   1) 초기화: 전체 경로에서 차량 위치 기준 최근점 탐색
//   2) 슬라이딩: 이전 인덱스 기준 ±search_window 범위 탐색
//               forward dot > 0 우선, fallback 시 순수 거리
//   3) 안정화: 후방 점프 억제, max_index_step 으로 전방 점프 제한
//   4) 순환: (wp + i) % total (랩 랩 추적)
//   5) 스무딩: 이동평균, 코너에서 윈도우 축소
//   6) kappa_r: 3점 menger 곡률 계산 → orientation.x 에 pack
//   7) 발행: orientation = (kappa_r, 0, sin(yaw/2), cos(yaw/2))
// ============================================================
#include "jeju_mpc/local_path_pub_cpp.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace jeju_mpc
{

// ─────────────────────────────────────────────────────────────
//  3점 부호 있는 menger 곡률
//  κ > 0 : 좌회전(반시계),  κ < 0 : 우회전(시계)
// ─────────────────────────────────────────────────────────────
double LocalPathPubCpp::menger_kappa(
    double x0, double y0,
    double x1, double y1,
    double x2, double y2)
{
    const double dx1 = x1 - x0,  dy1 = y1 - y0;
    const double dx2 = x2 - x1,  dy2 = y2 - y1;
    const double d1  = std::hypot(dx1, dy1);
    const double d2  = std::hypot(dx2, dy2);
    const double d01 = std::hypot(x2 - x0, y2 - y0);
    if (d1 < 1e-6 || d2 < 1e-6 || d01 < 1e-6) return 0.0;
    const double cross = dx1 * dy2 - dy1 * dx2;           // 부호 보존
    return 2.0 * cross / (d1 * d2 * d01 + 1e-9);
}

// ─────────────────────────────────────────────────────────────
LocalPathPubCpp::LocalPathPubCpp()
: Node("local_path_pub_cpp"),
  lap_start_time_(this->now())
{
    declare_parameter("local_path_size",        120);
    declare_parameter("search_window",          150);
    declare_parameter("max_index_step",          20);
    declare_parameter("reinit_dist_thresh",       2.0);
    declare_parameter("smooth_window",            2);
    declare_parameter("corner_smooth_window",     0);
    declare_parameter("corner_kappa_thresh",      0.08);

    local_path_size_      = static_cast<int>(std::max<int64_t>(10, get_parameter("local_path_size").as_int()));
    search_window_        = static_cast<int>(std::max<int64_t>(10, get_parameter("search_window").as_int()));
    max_index_step_       = static_cast<int>(std::max<int64_t>(1,  get_parameter("max_index_step").as_int()));
    reinit_dist_thresh_   = std::max(0.5, get_parameter("reinit_dist_thresh").as_double());
    smooth_window_        = static_cast<int>(std::max<int64_t>(0,  get_parameter("smooth_window").as_int()));
    corner_smooth_window_ = static_cast<int>(std::max<int64_t>(0,  get_parameter("corner_smooth_window").as_int()));
    corner_kappa_thresh_  = get_parameter("corner_kappa_thresh").as_double();

    // ── 구독 ──────────────────────────────────────────────
    auto qos_tl = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    global_sub_ = create_subscription<nav_msgs::msg::Path>(
        "/global_path", qos_tl,
        std::bind(&LocalPathPubCpp::globalPathCb, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", rclcpp::SensorDataQoS(),
        std::bind(&LocalPathPubCpp::odomCb, this, std::placeholders::_1));

    // ── 발행 ──────────────────────────────────────────────
    local_pub_ = create_publisher<nav_msgs::msg::Path>("/local_path", 10);
    lap_pub_   = create_publisher<std_msgs::msg::Float64MultiArray>("/mpc/lap_info", 10);

    // 20 Hz
    timer_ = create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&LocalPathPubCpp::publishLocalPath, this));

    RCLCPP_INFO(get_logger(),
        "LocalPathPub 시작  local_path_size=%d  search_window=%d  smooth=%d",
        local_path_size_, search_window_, smooth_window_);
}

// ─────────────────────────────────────────────────────────────
void LocalPathPubCpp::globalPathCb(const nav_msgs::msg::Path::SharedPtr msg)
{
    global_path_   = msg;
    is_initialized_ = false;   // 새 글로벌 경로 → 재초기화
    current_wp_    = 0;
    RCLCPP_INFO(get_logger(), "글로벌 경로 수신: %zu 포인트", msg->poses.size());
}

void LocalPathPubCpp::odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    const double x = msg->pose.pose.position.x;
    const double y = msg->pose.pose.position.y;
    const auto & q = msg->pose.pose.orientation;

    cur_x_ = x;
    cur_y_ = y;
    cur_yaw_ = std::atan2(2.0*(q.w*q.z + q.x*q.y),
                          1.0 - 2.0*(q.y*q.y + q.z*q.z));

    // 총 주행 거리
    if (prev_odom_pos_) {
        const double dx = x - prev_odom_pos_->first;
        const double dy = y - prev_odom_pos_->second;
        total_dist_m_ += std::hypot(dx, dy);
    }
    prev_odom_pos_ = {x, y};
}

// ─────────────────────────────────────────────────────────────
void LocalPathPubCpp::publishLocalPath()
{
    if (!global_path_ || !cur_x_ || !cur_y_ || !cur_yaw_) return;

    const size_t total = global_path_->poses.size();
    if (total == 0) return;

    const double x   = *cur_x_;
    const double y   = *cur_y_;
    const double yaw = *cur_yaw_;

    // ── 1. 최근 인덱스 탐색 ────────────────────────────
    double min_d = std::numeric_limits<double>::infinity();
    size_t closest = current_wp_;

    bool need_global = !is_initialized_;

    if (!need_global) {
        // 슬라이딩 탐색: ±search_window (순환)
        // forward dot > 0 인 후보를 우선
        double   min_d_fwd = std::numeric_limits<double>::infinity();
        size_t   best_fwd  = current_wp_;
        double   min_d_any = std::numeric_limits<double>::infinity();
        size_t   best_any  = current_wp_;

        for (int i = 0; i <= search_window_; ++i) {
            long ri = static_cast<long>(current_wp_) + i;
            if (ri >= static_cast<long>(total)) ri %= total;
            const size_t idx = static_cast<size_t>(ri);

            const double dx = global_path_->poses[idx].pose.position.x - x;
            const double dy = global_path_->poses[idx].pose.position.y - y;
            const double d  = std::hypot(dx, dy);
            const double dot = dx * std::cos(yaw) + dy * std::sin(yaw);

            if (d < min_d_any) { min_d_any = d; best_any = idx; }
            if (dot > 0.0 && d < min_d_fwd) { min_d_fwd = d; best_fwd = idx; }
        }

        min_d   = std::isfinite(min_d_fwd) ? min_d_fwd : min_d_any;
        closest = std::isfinite(min_d_fwd) ? best_fwd  : best_any;

        // 너무 멀면 전역 재탐색
        if (min_d > reinit_dist_thresh_) need_global = true;
    }

    // ── 2. 전역 탐색 (초기화 / 재배치) ─────────────────
    if (need_global) {
        double   min_d_fwd = std::numeric_limits<double>::infinity();
        size_t   best_fwd  = 0;
        double   min_d_any = std::numeric_limits<double>::infinity();
        size_t   best_any  = 0;

        for (size_t i = 0; i < total; ++i) {
            const double dx = global_path_->poses[i].pose.position.x - x;
            const double dy = global_path_->poses[i].pose.position.y - y;
            const double d  = std::hypot(dx, dy);
            const double dot = dx * std::cos(yaw) + dy * std::sin(yaw);
            if (d < min_d_any) { min_d_any = d; best_any = i; }
            if (dot > 0.0 && d < min_d_fwd) { min_d_fwd = d; best_fwd = i; }
        }
        closest = std::isfinite(min_d_fwd) ? best_fwd : best_any;
        min_d   = std::isfinite(min_d_fwd) ? min_d_fwd : min_d_any;
        is_initialized_ = true;
    }

    // ── 3. 인덱스 안정화 (역방향 점프 억제) ────────────
    if (is_initialized_ && total > 0) {
        const size_t prev = current_wp_;
        const size_t fwd_delta  = (closest + total - prev) % total;
        const size_t bwd_delta  = (prev + total - closest) % total;
        const bool   lap_wrap   = (prev > total * 9 / 10) && (closest < total / 10);

        if (!lap_wrap && bwd_delta < fwd_delta) {
            // 역방향 → 현재 유지
            closest = prev;
        } else if (fwd_delta > static_cast<size_t>(max_index_step_) &&
                   min_d < reinit_dist_thresh_) {
            // 과도 전방 점프 억제
            closest = (prev + max_index_step_) % total;
        }
    }

    // ── 4. 랩 감지 ─────────────────────────────────────
    if (closest < current_wp_ &&
        current_wp_ > total * 9 / 10 &&
        closest < total / 10)
    {
        ++lap_count_;
        lap_start_time_ = this->now();
        RCLCPP_INFO(get_logger(), "랩 %d 완료! 총 주행 %.1f m", lap_count_, total_dist_m_);
    }
    current_wp_ = closest;

    // ── 5. 로컬 윈도우 추출 (순환) ─────────────────────
    std::vector<double> raw_x(local_path_size_);
    std::vector<double> raw_y(local_path_size_);
    std::vector<double> raw_kr(local_path_size_);   // 원본 kappa_r 보존

    for (int i = 0; i < local_path_size_; ++i) {
        const size_t idx = (current_wp_ + i) % total;
        raw_x[i]  = global_path_->poses[idx].pose.position.x;
        raw_y[i]  = global_path_->poses[idx].pose.position.y;
        // 원본 kappa_r (mpc_path_maker_node 가 orientation.x에 저장)
        raw_kr[i] = global_path_->poses[idx].pose.orientation.x;
    }

    // ── 6. 곡률 적응형 스무딩 ──────────────────────────
    // 3점 곡률 계산 (스무딩 윈도우 결정용)
    std::vector<double> kappa_abs(local_path_size_, 0.0);
    for (int i = 1; i < local_path_size_ - 1; ++i) {
        kappa_abs[i] = std::abs(menger_kappa(
            raw_x[i-1], raw_y[i-1],
            raw_x[i],   raw_y[i],
            raw_x[i+1], raw_y[i+1]));
    }

    std::vector<double> sm_x = raw_x;
    std::vector<double> sm_y = raw_y;

    for (int i = 0; i < local_path_size_; ++i) {
        int win = smooth_window_;
        if (kappa_abs[i] > corner_kappa_thresh_) {
            win = corner_smooth_window_;
        }
        if (win <= 0) continue;

        const int i0 = std::max(0, i - win);
        const int i1 = std::min(local_path_size_ - 1, i + win);
        double sx = 0.0, sy = 0.0;
        int cnt = 0;
        for (int j = i0; j <= i1; ++j) { sx += raw_x[j]; sy += raw_y[j]; ++cnt; }
        if (cnt > 0) { sm_x[i] = sx / cnt; sm_y[i] = sy / cnt; }
    }

    // ── 7. 경로 메시지 구성 ─────────────────────────────
    // 스무딩된 위치 기반으로 yaw, kappa_r 재계산
    // 저장 형식 (pathFromMsg 관례):
    //   orientation.x = kappa_r (3점 menger)
    //   orientation.y = 0
    //   orientation.z = sin(yaw/2)
    //   orientation.w = cos(yaw/2)
    auto local = nav_msgs::msg::Path();
    local.header.frame_id = "odom";
    local.header.stamp    = this->now();
    local.poses.reserve(local_path_size_);

    for (int i = 0; i < local_path_size_; ++i) {
        // yaw: 접선 방향
        int from_i = (i + 1 < local_path_size_) ? i : i - 1;
        int to_i   = (i + 1 < local_path_size_) ? i + 1 : i;
        const double dx  = sm_x[to_i] - sm_x[from_i];
        const double dy  = sm_y[to_i] - sm_y[from_i];
        const double yaw_k = (std::hypot(dx, dy) > 1e-9)
                             ? std::atan2(dy, dx) : 0.0;

        // kappa_r: 스무딩된 3점 곡률 (경계는 인접 2점)
        double kr = 0.0;
        if (i > 0 && i < local_path_size_ - 1) {
            kr = menger_kappa(
                sm_x[i-1], sm_y[i-1],
                sm_x[i],   sm_y[i],
                sm_x[i+1], sm_y[i+1]);
        } else if (raw_kr[i] != 0.0) {
            kr = raw_kr[i];   // 원본 fallback (경계점)
        }

        geometry_msgs::msg::PoseStamped ps;
        ps.header = local.header;
        ps.pose.position.x  = sm_x[i];
        ps.pose.position.y  = sm_y[i];
        ps.pose.position.z  = 0.0;
        // kappa_r + yaw 패킹
        ps.pose.orientation.x = kr;
        ps.pose.orientation.y = 0.0;
        ps.pose.orientation.z = std::sin(yaw_k * 0.5);
        ps.pose.orientation.w = std::cos(yaw_k * 0.5);
        local.poses.push_back(ps);
    }

    local_pub_->publish(local);

    // ── 8. 랩 정보 발행 ────────────────────────────────
    const double progress    = static_cast<double>(current_wp_) / total * 100.0;
    const double elapsed_sec = (this->now() - lap_start_time_).seconds();

    std_msgs::msg::Float64MultiArray lap;
    lap.data = {
        static_cast<double>(lap_count_),
        progress,
        static_cast<double>(current_wp_),
        static_cast<double>(total),
        elapsed_sec,
        total_dist_m_,
    };
    lap_pub_->publish(lap);
}

}  // namespace jeju_mpc

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<jeju_mpc::LocalPathPubCpp>());
    rclcpp::shutdown();
    return 0;
}
