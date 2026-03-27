// mpc_logger_node.cpp
// MPC 경로추종 실행 중 모든 상태/오차/파라미터를 CSV로 저장
// 저장 경로: ~/henes_ws_ros2/log/{YYYYMMDD_HHMMSS}_mpc.csv
//
// CSV 컬럼:
//   시각, 차량 위치/자세/속도, 최근접 참조점, 횡방향/헤딩 오차,
//   조향 명령, 속도 명령, MPC 비용, 경로 진행률,
//   현재 적용 파라미터 전체

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace
{

struct RefPoint { double x, y, yaw, kappa_r; };

struct VehicleState {
  double x{0}, y{0}, yaw{0};
  double vx{0}, vy{0}, omega{0};
  double delta_c_deg{0};
};

struct CmdState {
  double steer_deg{0};
  double speed_pwm{0};
  double torque_cmd{0};
  double mpc_cost{0};
};

double normalizeAngle(double a) {
  while (a >  M_PI) a -= 2*M_PI;
  while (a < -M_PI) a += 2*M_PI;
  return a;
}

double yawFromQ(double qx, double qy, double qz, double qw) {
  return std::atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz));
}

} // namespace

class MpcLoggerNode : public rclcpp::Node
{
public:
  MpcLoggerNode() : Node("mpc_logger_node")
  {
    // ── 파라미터 ─────────────────────────────────────
    declare_parameter("log_dir",        std::string(std::getenv("HOME")) + "/henes_ws_ros2/check");
    declare_parameter("log_hz",         20.0);
    declare_parameter("odom_topic",     "/odometry/filtered");
    declare_parameter("path_topic",     "/global_path");
    declare_parameter("cmd_topic",      "/cmd_vel");
    declare_parameter("steer_topic",    "/steering_angle");
    declare_parameter("mpc_cost_topic", "/mpc_cost");

    // MPC 파라미터 (로그 기록용)
    declare_parameter("wheelbase_m",        1.04);
    declare_parameter("forward_speed_kmh",  3.0);
    declare_parameter("prediction_horizon", 15);
    declare_parameter("dt",                 0.1);
    declare_parameter("use_dbm",            true);
    declare_parameter("w_d",                1.0);
    declare_parameter("w_theta",            0.5);
    declare_parameter("w_kappa",            0.05);
    declare_parameter("w_u",                0.2);
    declare_parameter("w_xy",               2.0);
    declare_parameter("w_psi",              1.0);
    declare_parameter("w_vx",               0.5);
    declare_parameter("w_vy",               0.3);
    declare_parameter("w_omega",            0.3);
    declare_parameter("w_delta_c",          0.1);
    declare_parameter("w_u_delta",          0.2);
    declare_parameter("w_u_torque",         1e-5);
    declare_parameter("mass_kg",            150.0);
    declare_parameter("inertia_kgm2",       120.0);
    declare_parameter("lf_m",               0.52);
    declare_parameter("lr_m",               0.52);
    declare_parameter("Ca_N_rad",           25000.0);
    declare_parameter("Cx_N",               30000.0);
    declare_parameter("mu",                 0.8);

    log_dir_   = get_parameter("log_dir").as_string();
    double hz  = std::max(1.0, get_parameter("log_hz").as_double());

    // ── 구독 ─────────────────────────────────────────
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("odom_topic").as_string(), 20,
      [this](nav_msgs::msg::Odometry::SharedPtr m){ odomCb(m); });

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      get_parameter("path_topic").as_string(), 10,
      [this](nav_msgs::msg::Path::SharedPtr m){ pathCb(m); });

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      get_parameter("cmd_topic").as_string(), 20,
      [this](geometry_msgs::msg::Twist::SharedPtr m){ cmdCb(m); });

    steer_sub_ = create_subscription<std_msgs::msg::Float64>(
      get_parameter("steer_topic").as_string(), 20,
      [this](std_msgs::msg::Float64::SharedPtr m){
        std::lock_guard<std::mutex> g(mu_);
        cmd_.steer_deg = m->data;
      });

    cost_sub_ = create_subscription<std_msgs::msg::Float64>(
      get_parameter("mpc_cost_topic").as_string(), 20,
      [this](std_msgs::msg::Float64::SharedPtr m){
        std::lock_guard<std::mutex> g(mu_);
        cmd_.mpc_cost = m->data;
      });

    // ── CSV 생성 ──────────────────────────────────────
    openCsv();

    // ── 로깅 타이머 ───────────────────────────────────
    using namespace std::chrono_literals;
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0/hz)),
      [this](){ logRow(); });

    RCLCPP_INFO(get_logger(), "MPC logger started → %s", csv_path_.c_str());
  }

  ~MpcLoggerNode() override
  {
    if (ofs_.is_open()) ofs_.close();
    RCLCPP_INFO(get_logger(), "MPC logger closed. rows=%zu  path=%s",
      row_count_, csv_path_.c_str());
  }

private:
  // ── 콜백 ─────────────────────────────────────────────
  void odomCb(const nav_msgs::msg::Odometry::SharedPtr m)
  {
    std::lock_guard<std::mutex> g(mu_);
    const auto & p = m->pose.pose;
    const auto & t = m->twist.twist;
    state_.x   = p.position.x;
    state_.y   = p.position.y;
    state_.yaw = yawFromQ(p.orientation.x, p.orientation.y,
                          p.orientation.z, p.orientation.w);
    const double cy = std::cos(state_.yaw), sy = std::sin(state_.yaw);
    state_.vx    =  t.linear.x * cy + t.linear.y * sy;
    state_.vy    = -t.linear.x * sy + t.linear.y * cy;
    state_.omega = t.angular.z;
    has_odom_    = true;
  }

  void pathCb(const nav_msgs::msg::Path::SharedPtr m)
  {
    std::vector<RefPoint> tmp;
    tmp.reserve(m->poses.size());
    for (size_t i = 0; i < m->poses.size(); ++i) {
      const auto & ps = m->poses[i];
      RefPoint rp;
      rp.x       = ps.pose.position.x;
      rp.y       = ps.pose.position.y;
      rp.kappa_r = ps.pose.orientation.x;
      const double qn = ps.pose.orientation.x * ps.pose.orientation.x
                      + ps.pose.orientation.y * ps.pose.orientation.y
                      + ps.pose.orientation.z * ps.pose.orientation.z
                      + ps.pose.orientation.w * ps.pose.orientation.w;
      if (std::abs(qn - 1.0) < 0.01) {
        rp.yaw = yawFromQ(ps.pose.orientation.x, ps.pose.orientation.y,
                          ps.pose.orientation.z, ps.pose.orientation.w);
      } else {
        rp.yaw = 0.0;
      }
      tmp.push_back(rp);
    }
    // yaw 보완
    for (size_t i = 0; i < tmp.size(); ++i) {
      if (std::abs(tmp[i].yaw) < 1e-9 && tmp.size() >= 2) {
        const size_t prev = (i == 0) ? 0 : i-1;
        const size_t next = (i+1 < tmp.size()) ? i+1 : tmp.size()-1;
        tmp[i].yaw = std::atan2(tmp[next].y - tmp[prev].y,
                                tmp[next].x - tmp[prev].x);
      }
    }
    std::lock_guard<std::mutex> g(mu_);
    path_      = std::move(tmp);
    path_len_m_ = computePathLength(path_);
    has_path_   = true;
  }

  void cmdCb(const geometry_msgs::msg::Twist::SharedPtr m)
  {
    std::lock_guard<std::mutex> g(mu_);
    cmd_.speed_pwm  = m->linear.x;
    cmd_.steer_deg  = m->angular.z;
    cmd_.torque_cmd = m->linear.y;
    has_cmd_        = true;
  }

  // ── 메인 로깅 ─────────────────────────────────────────
  void logRow()
  {
    std::lock_guard<std::mutex> g(mu_);
    if (!has_odom_ || !has_path_ || !ofs_.is_open()) return;

    const double now_sec = this->now().seconds();

    // 최근접 참조점
    const int ni = nearestIdx(state_, path_);
    const RefPoint & ref = path_[ni];

    // 횡방향 오차 (CTE, 부호: 좌=+, 우=-)
    const double ex   = state_.x - ref.x;
    const double ey   = state_.y - ref.y;
    const double cte  = -std::sin(ref.yaw) * ex + std::cos(ref.yaw) * ey;

    // 헤딩 오차 [deg]
    const double heading_err_deg =
      normalizeAngle(state_.yaw - ref.yaw) * 180.0 / M_PI;

    // 속도
    const double speed_mps = std::hypot(state_.vx, state_.vy);

    // 경로 진행률
    const double progress = (path_len_m_ > 0.01)
      ? arcLengthTo(path_, ni) / path_len_m_ : 0.0;

    // 경로 잔여거리
    const double dist_to_goal = (path_.empty()) ? 0.0 :
      std::hypot(state_.x - path_.back().x, state_.y - path_.back().y);

    char buf[1024];
    std::snprintf(buf, sizeof(buf),
      // 시각
      "%.4f,"
      // 차량 위치/자세
      "%.6f,%.6f,%.6f,"
      // 속도 (body)
      "%.4f,%.4f,%.4f,"
      // 참조점
      "%.6f,%.6f,%.6f,%.6f,"
      // 오차
      "%.6f,%.4f,"
      // 명령
      "%.4f,%.4f,%.6f,"
      // MPC 비용
      "%.6f,"
      // 경로 상태
      "%d,%d,%.4f,%.4f,%.4f\n",
      now_sec,
      state_.x, state_.y, state_.yaw * 180.0/M_PI,
      state_.vx, state_.vy, state_.omega,
      ref.x, ref.y, ref.yaw * 180.0/M_PI, ref.kappa_r,
      cte, heading_err_deg,
      cmd_.steer_deg, cmd_.speed_pwm, cmd_.torque_cmd,
      cmd_.mpc_cost,
      ni, static_cast<int>(path_.size()),
      progress, dist_to_goal, path_len_m_
    );

    ofs_ << buf;
    if (++row_count_ % 200 == 0) ofs_.flush();
  }

  // ── 헬퍼 ─────────────────────────────────────────────
  static int nearestIdx(const VehicleState & st, const std::vector<RefPoint> & path)
  {
    int best = 0;
    double best_d2 = std::numeric_limits<double>::max();
    for (int i = 0; i < static_cast<int>(path.size()); ++i) {
      const double d2 = (st.x-path[i].x)*(st.x-path[i].x)
                      + (st.y-path[i].y)*(st.y-path[i].y);
      if (d2 < best_d2) { best_d2 = d2; best = i; }
    }
    return best;
  }

  static double computePathLength(const std::vector<RefPoint> & path)
  {
    double s = 0.0;
    for (size_t i = 1; i < path.size(); ++i)
      s += std::hypot(path[i].x - path[i-1].x, path[i].y - path[i-1].y);
    return s;
  }

  static double arcLengthTo(const std::vector<RefPoint> & path, int idx)
  {
    double s = 0.0;
    for (int i = 1; i <= idx && i < static_cast<int>(path.size()); ++i)
      s += std::hypot(path[i].x - path[i-1].x, path[i].y - path[i-1].y);
    return s;
  }

  void openCsv()
  {
    // 디렉터리 생성
    std::system(("mkdir -p " + log_dir_).c_str());

    // 타임스탬프 파일명
    std::time_t t = std::time(nullptr);
    char ts[32];
    std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", std::localtime(&t));
    csv_path_ = log_dir_ + "/" + std::string(ts) + "_mpc.csv";

    ofs_.open(csv_path_);
    if (!ofs_.is_open()) {
      RCLCPP_ERROR(get_logger(), "CSV 열기 실패: %s", csv_path_.c_str());
      return;
    }

    // 헤더 행 1: MPC 파라미터 스냅샷 (주석 형태)
    ofs_ << "# MPC_PARAMS"
         << " wheelbase_m="        << get_parameter("wheelbase_m").as_double()
         << " forward_speed_kmh="  << get_parameter("forward_speed_kmh").as_double()
         << " prediction_horizon=" << get_parameter("prediction_horizon").as_int()
         << " dt="                 << get_parameter("dt").as_double()
         << " use_dbm="            << (get_parameter("use_dbm").as_bool() ? 1 : 0)
         << " w_d="                << get_parameter("w_d").as_double()
         << " w_theta="            << get_parameter("w_theta").as_double()
         << " w_kappa="            << get_parameter("w_kappa").as_double()
         << " w_u="                << get_parameter("w_u").as_double()
         << " w_xy="               << get_parameter("w_xy").as_double()
         << " w_psi="              << get_parameter("w_psi").as_double()
         << " w_vx="               << get_parameter("w_vx").as_double()
         << " w_vy="               << get_parameter("w_vy").as_double()
         << " w_omega="            << get_parameter("w_omega").as_double()
         << " w_delta_c="          << get_parameter("w_delta_c").as_double()
         << " w_u_delta="          << get_parameter("w_u_delta").as_double()
         << " w_u_torque="         << get_parameter("w_u_torque").as_double()
         << " mass_kg="            << get_parameter("mass_kg").as_double()
         << " inertia_kgm2="       << get_parameter("inertia_kgm2").as_double()
         << " lf_m="               << get_parameter("lf_m").as_double()
         << " lr_m="               << get_parameter("lr_m").as_double()
         << " Ca_N_rad="           << get_parameter("Ca_N_rad").as_double()
         << " Cx_N="               << get_parameter("Cx_N").as_double()
         << " mu="                 << get_parameter("mu").as_double()
         << "\n";

    // 헤더 행 2: 컬럼명
    ofs_ << "timestamp_sec,"
            "x_m,y_m,yaw_deg,"
            "vx_mps,vy_mps,omega_rads,"
            "ref_x_m,ref_y_m,ref_yaw_deg,ref_kappa,"
            "cte_m,heading_err_deg,"
            "cmd_steer_deg,cmd_speed_pwm,cmd_torque,"
            "mpc_cost,"
            "nearest_idx,path_size,path_progress,dist_to_goal_m,path_len_m\n";
  }

  // ── 멤버 ─────────────────────────────────────────────
  std::mutex mu_;

  VehicleState state_;
  CmdState     cmd_;
  std::vector<RefPoint> path_;
  double path_len_m_ {0.0};

  bool has_odom_ {false};
  bool has_path_ {false};
  bool has_cmd_  {false};

  std::ofstream ofs_;
  std::string   csv_path_;
  std::string   log_dir_;
  size_t        row_count_ {0};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr     path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr  steer_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr  cost_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MpcLoggerNode>());
  rclcpp::shutdown();
  return 0;
}
