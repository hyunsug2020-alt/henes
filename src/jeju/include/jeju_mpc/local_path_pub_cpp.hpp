#pragma once
// ============================================================
// local_path_pub_cpp.hpp
//
// bisa LocalPathPubCpp → HENES ROS2 Humble 이식
//
// 기능:
//   - /global_path 에서 차량 위치 기준 로컬 윈도우 추출
//   - 순환 경로 지원 (랩 추적)
//   - 곡률 적응형 이동평균 스무딩
//   - kappa_r 자동 계산 (3점 곡률)
//   - 인덱스 안정화 (역방향 점프 방지, max_index_step)
//   - /local_path 발행 (nav_msgs/Path)
//
// 구독:
//   /global_path   (nav_msgs/Path, transient_local)
//   /odometry/filtered (nav_msgs/Odometry)
//
// 발행:
//   /local_path    (nav_msgs/Path, 20 Hz)
//   /mpc/lap_info  (std_msgs/Float64MultiArray)
//     [0]=lap_count [1]=progress% [2]=current_wp [3]=total_wp
//     [4]=elapsed_sec [5]=total_distance_m
// ============================================================
#include <deque>
#include <optional>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace jeju_mpc
{

class LocalPathPubCpp : public rclcpp::Node
{
public:
    LocalPathPubCpp();

private:
    void globalPathCb(const nav_msgs::msg::Path::SharedPtr msg);
    void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg);
    void publishLocalPath();

    // ── 내부 유틸 ─────────────────────────────────────────
    // 3점 부호 있는 곡률: 양=좌회전, 음=우회전
    static double menger_kappa(
        double x0, double y0,
        double x1, double y1,
        double x2, double y2);

    // ── 구독 / 발행 ───────────────────────────────────────
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr     global_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr        local_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr lap_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ── 전역 경로 / 자세 ─────────────────────────────────
    nav_msgs::msg::Path::SharedPtr global_path_;
    std::optional<double> cur_x_, cur_y_, cur_yaw_;

    // ── 파라미터 ─────────────────────────────────────────
    int    local_path_size_            {120};
    int    search_window_              {150};
    int    max_index_step_             { 20};
    double reinit_dist_thresh_         {2.0};
    int    smooth_window_              {  2};
    int    corner_smooth_window_       {  0};   // 코너에서는 스무딩 감소
    double corner_kappa_thresh_        {0.08};

    // ── 상태 ─────────────────────────────────────────────
    size_t current_wp_                 {0};
    bool   is_initialized_             {false};
    int    lap_count_                  {0};
    double total_dist_m_               {0.0};
    rclcpp::Time lap_start_time_;
    std::optional<std::pair<double,double>> prev_odom_pos_;
};

}  // namespace jeju_mpc
