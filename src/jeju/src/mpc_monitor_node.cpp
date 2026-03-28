// mpc_monitor_node.cpp  –  MPC 실시간 디버그 GUI (OpenCV)
//
// 레이아웃 (1280 × 720):
//   왼쪽 800px  : 지도 (경로·차량·오차선·진행바)
//   오른쪽 480px:
//     ① 상단 200px  : MPC 내부값 패널 (e_y / e_psi / 방향 / idx / 속도 / PWM / steer)
//     ② 중단 360px  : 5개 그래프 (CTE / HdgErr / Speed_signed / cmd_steer / cmd_pwm)
//     ③ 하단 160px  : /rosout 로그 패널
//
// 토픽 구독:
//   /global_path       (nav_msgs/Path,   transient_local)
//   /odometry/filtered (nav_msgs/Odometry)
//   /cmd_vel           (geometry_msgs/Twist)
//   /steering_angle    (std_msgs/Float64)
//   /mpc/debug         (std_msgs/Float64MultiArray)
//   /rosout            (rcl_interfaces/Log)
//
// /mpc/debug 인덱스:
//   [0]=e_y(m)  [1]=e_psi(deg)  [2]=target_v(m/s signed)
//   [3]=speed_pwm  [4]=nearest_idx  [5]=direction(1/-1)
//   [6]=steer_cmd(deg)  [7]=goal_reached  [8]=has_path

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

// ──────────────────────────────────────────────────────────────
// 헬퍼
// ──────────────────────────────────────────────────────────────
static double yawFromQ(double qx, double qy, double qz, double qw)
{
    return std::atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz));
}
static double normAngle(double a)
{
    while (a >  M_PI) a -= 2*M_PI;
    while (a < -M_PI) a += 2*M_PI;
    return a;
}
struct Pt { double x, y, yaw; };
struct LogEntry {
    uint8_t     level;
    std::string name;
    std::string msg;
};

// ──────────────────────────────────────────────────────────────
// 노드
// ──────────────────────────────────────────────────────────────
class MpcMonitorNode : public rclcpp::Node
{
public:
    // ── 레이아웃 상수 ─────────────────────────────────────────
    static constexpr int WIN_W    = 1280;
    static constexpr int WIN_H    = 720;
    static constexpr int MAP_W    = 800;
    static constexpr int SIDE_W   = WIN_W - MAP_W;   // 480

    static constexpr int STATE_H  = 200;              // MPC 내부값 패널
    static constexpr int LOG_H    = 160;              // 로그 패널
    static constexpr int GRAPH_AREA_H = WIN_H - STATE_H - LOG_H;  // 360

    static constexpr int N_GRAPH  = 5;
    static constexpr int GRAPH_H  = GRAPH_AREA_H / N_GRAPH;       // 72
    static constexpr int N_HIST   = 400;
    static constexpr int MAX_LOGS = 8;

    MpcMonitorNode() : Node("mpc_monitor_node")
    {
        auto qos_tl = rclcpp::QoS(1).reliable().transient_local();
        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/global_path", qos_tl,
            [this](nav_msgs::msg::Path::SharedPtr m){ pathCb(m); });

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 20,
            [this](nav_msgs::msg::Odometry::SharedPtr m){ odomCb(m); });

        cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 20,
            [this](geometry_msgs::msg::Twist::SharedPtr m){
                std::lock_guard<std::mutex> g(mu_);
                cmd_steer_ = m->angular.z;
                cmd_pwm_   = m->linear.x;
            });

        steer_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/steering_angle", 20,
            [this](std_msgs::msg::Float64::SharedPtr m){
                std::lock_guard<std::mutex> g(mu_);
                steer_hw_ = m->data;
            });

        debug_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/mpc/debug", 20,
            [this](std_msgs::msg::Float64MultiArray::SharedPtr m){ debugCb(m); });

        rosout_sub_ = create_subscription<rcl_interfaces::msg::Log>(
            "/rosout", rclcpp::QoS(50),
            [this](rcl_interfaces::msg::Log::SharedPtr m){ logCb(m); });

        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            [this](){ render(); });

        cv::namedWindow("MPC Debug Monitor", cv::WINDOW_NORMAL);
        cv::resizeWindow("MPC Debug Monitor", WIN_W, WIN_H);
        RCLCPP_INFO(get_logger(), "MPC Debug Monitor 시작");
    }

private:
    // ── 콜백 ──────────────────────────────────────────────────
    void pathCb(const nav_msgs::msg::Path::SharedPtr m)
    {
        std::vector<Pt> tmp;
        tmp.reserve(m->poses.size());
        for (const auto & ps : m->poses) {
            Pt p;
            p.x   = ps.pose.position.x;
            p.y   = ps.pose.position.y;
            p.yaw = yawFromQ(ps.pose.orientation.x, ps.pose.orientation.y,
                             ps.pose.orientation.z, ps.pose.orientation.w);
            tmp.push_back(p);
        }
        std::lock_guard<std::mutex> g(mu_);
        path_      = std::move(tmp);
        map_dirty_ = true;
    }

    void odomCb(const nav_msgs::msg::Odometry::SharedPtr m)
    {
        std::lock_guard<std::mutex> g(mu_);
        veh_x_   = m->pose.pose.position.x;
        veh_y_   = m->pose.pose.position.y;
        veh_yaw_ = yawFromQ(m->pose.pose.orientation.x,
                            m->pose.pose.orientation.y,
                            m->pose.pose.orientation.z,
                            m->pose.pose.orientation.w);
        const double cy = std::cos(veh_yaw_), sy = std::sin(veh_yaw_);
        veh_vx_   = m->twist.twist.linear.x * cy + m->twist.twist.linear.y * sy;
        has_odom_ = true;
        if (traveled_.empty() ||
            std::hypot(veh_x_ - traveled_.back().x,
                       veh_y_ - traveled_.back().y) > 0.1) {
            traveled_.push_back({veh_x_, veh_y_, veh_yaw_});
            if (traveled_.size() > 20000) traveled_.erase(traveled_.begin());
        }
    }

    void debugCb(const std_msgs::msg::Float64MultiArray::SharedPtr m)
    {
        if (m->data.size() < 9) return;
        std::lock_guard<std::mutex> g(mu_);
        dbg_e_y_        = m->data[0];
        dbg_e_psi_      = m->data[1];     // deg
        dbg_target_v_   = m->data[2];     // m/s signed
        dbg_speed_pwm_  = m->data[3];     // signed
        dbg_nearest_    = (int)m->data[4];
        dbg_direction_  = (int)m->data[5]; // 1 or -1
        dbg_steer_cmd_  = m->data[6];     // deg
        dbg_goal_       = m->data[7] > 0.5;
        has_debug_      = true;

        // 히스토리
        pushH(cte_hist_,  dbg_e_y_);
        pushH(hdg_hist_,  dbg_e_psi_);
        pushH(spd_hist_,  dbg_target_v_);      // signed
        pushH(str_hist_,  dbg_steer_cmd_);
        pushH(pwm_hist_,  dbg_speed_pwm_);
    }

    void logCb(const rcl_interfaces::msg::Log::SharedPtr m)
    {
        if (m->level < 20) return;
        std::lock_guard<std::mutex> g(mu_);
        logs_.push_back({m->level, m->name, m->msg});
        while ((int)logs_.size() > MAX_LOGS * 3) logs_.pop_front();
    }

    // ── 렌더 ──────────────────────────────────────────────────
    void render()
    {
        std::vector<Pt> path, traveled;
        double vx, vy, vyaw, vvx;
        bool has_odom, has_path;
        double steer_hw, cmd_steer, cmd_pwm;
        double cte=0, hdg_err=0;
        std::deque<double> h_cte, h_hdg, h_spd, h_str, h_pwm;
        double map_scale, map_ox, map_oy;
        int nearest_i = 0;
        std::deque<LogEntry> logs_copy;

        // debug panel data
        double d_ey, d_epsi, d_tv, d_pwm, d_steer;
        int d_nearest, d_dir;
        bool d_goal, d_has;

        {
            std::lock_guard<std::mutex> g(mu_);
            path     = path_;
            traveled = traveled_;
            vx = veh_x_; vy = veh_y_; vyaw = veh_yaw_; vvx = veh_vx_;
            has_odom = has_odom_;
            has_path = !path_.empty();
            steer_hw = steer_hw_;
            cmd_steer = cmd_steer_; cmd_pwm = cmd_pwm_;

            d_ey     = dbg_e_y_;
            d_epsi   = dbg_e_psi_;
            d_tv     = dbg_target_v_;
            d_pwm    = dbg_speed_pwm_;
            d_nearest= dbg_nearest_;
            d_dir    = dbg_direction_;
            d_steer  = dbg_steer_cmd_;
            d_goal   = dbg_goal_;
            d_has    = has_debug_;

            if (has_odom && has_path) {
                if (map_dirty_) { computeMapTransform(); map_dirty_ = false; }
                nearest_i = nearestIdx();
                const auto & ref = path_[nearest_i];
                double ex = veh_x_ - ref.x;
                double ey = veh_y_ - ref.y;
                cte     = -std::sin(ref.yaw)*ex + std::cos(ref.yaw)*ey;
                hdg_err =  normAngle(veh_yaw_ - ref.yaw) * 180.0 / M_PI;
            }
            h_cte = cte_hist_; h_hdg = hdg_hist_;
            h_spd = spd_hist_; h_str = str_hist_;
            h_pwm = pwm_hist_;
            map_scale = map_scale_; map_ox = map_ox_; map_oy = map_oy_;
            logs_copy = logs_;
        }

        cv::Mat canvas(WIN_H, WIN_W, CV_8UC3, cv::Scalar(28,28,28));

        // ① 지도
        drawMap(canvas(cv::Rect(0, 0, MAP_W, WIN_H)),
                path, traveled, vx, vy, vyaw, vvx, steer_hw,
                has_odom, has_path, nearest_i,
                map_scale, map_ox, map_oy, cte, hdg_err);

        // ② MPC 내부값 패널
        drawStatePanel(canvas(cv::Rect(MAP_W, 0, SIDE_W, STATE_H)),
                       d_ey, d_epsi, d_tv, d_pwm, d_nearest, d_dir,
                       d_steer, d_goal, d_has, cmd_pwm, cmd_steer);

        // ③ 그래프
        {
            struct G { const std::deque<double>* d; const char* lbl; double lo,hi; cv::Scalar col; }
            graphs[5] = {
                {&h_cte, "CTE e_y [m]",           -3.0,  3.0, {0,220,255}},
                {&h_hdg, "Hdg Err [deg]",         -45.0, 45.0, {255,200,0}},
                {&h_spd, "Target Speed [m/s]",     -1.5,  1.5, {0,200,100}},
                {&h_str, "MPC Steer [deg]",        -60.0, 60.0, {200,100,255}},
                {&h_pwm, "Speed PWM",             -255.0,255.0, {255,160,50}},
            };
            for (int i = 0; i < N_GRAPH; ++i) {
                int gy = STATE_H + i * GRAPH_H;
                cv::Mat row = canvas(cv::Rect(MAP_W, gy, SIDE_W, GRAPH_H));
                drawGraph(row, *graphs[i].d, graphs[i].lbl,
                          graphs[i].lo, graphs[i].hi, graphs[i].col);
                cv::line(canvas, {MAP_W, gy}, {WIN_W, gy}, {60,60,70}, 1);
            }
        }

        // ④ 로그 패널
        {
            int ly = STATE_H + GRAPH_AREA_H;
            cv::Mat lp = canvas(cv::Rect(MAP_W, ly, SIDE_W, LOG_H));
            drawLogPanel(lp, logs_copy);
            cv::line(canvas, {MAP_W, ly}, {WIN_W, ly}, {90,90,90}, 1);
        }

        // 세로 구분선
        cv::line(canvas, {MAP_W, 0}, {MAP_W, WIN_H}, {70,70,70}, 1);

        cv::imshow("MPC Debug Monitor", canvas);
        cv::waitKey(1);
    }

    // ── MPC 상태 패널 ─────────────────────────────────────────
    void drawStatePanel(cv::Mat panel,
                        double e_y, double e_psi_deg,
                        double target_v, double speed_pwm,
                        int nearest, int direction,
                        double steer_cmd, bool goal, bool has_debug,
                        double cmd_pwm_actual, double cmd_steer_actual)
    {
        const int W = panel.cols, H = panel.rows;
        panel.setTo(cv::Scalar(18, 18, 24));

        if (!has_debug) {
            cv::putText(panel, "Waiting /mpc/debug...", {10, H/2},
                        cv::FONT_HERSHEY_SIMPLEX, 0.65, {140,140,140}, 1);
            return;
        }

        // ─ 방향 표시줄 (전체 너비, 40px 높이) ─
        const bool is_rev = (direction < 0);
        cv::Scalar dir_col = is_rev ? cv::Scalar(0, 50, 220) : cv::Scalar(0, 160, 60);
        cv::rectangle(panel, {0,0}, {W, 40}, dir_col, -1);
        const char* dir_label = is_rev ? "<<  REVERSE  <<" : ">>  FORWARD  >>";
        cv::putText(panel, dir_label, {W/2 - 100, 28},
                    cv::FONT_HERSHEY_SIMPLEX, 0.85,
                    is_rev ? cv::Scalar(255,200,200) : cv::Scalar(200,255,200), 2);

        // GOAL 표시
        if (goal) {
            cv::rectangle(panel, {W-90, 5}, {W-5, 35}, {0,200,200}, -1);
            cv::putText(panel, "GOAL", {W-80, 28},
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, {0,0,0}, 2);
        }

        // ─ 수치 테이블 ─
        // Row y 기준
        const int R1 = 65, R2 = 105, R3 = 145;
        const int C1 = 8,  C2 = 160, C3 = 315;

        // 색상 헬퍼 (임계 초과시 빨간)
        auto valColor = [](double v, double warn, double err) -> cv::Scalar {
            double a = std::abs(v);
            if (a >= err)  return {0, 60, 255};
            if (a >= warn) return {0, 200, 255};
            return {200, 255, 200};
        };

        char buf[64];

        // Row1: e_y | e_psi | nearest_idx
        cv::putText(panel, "e_y [m]", {C1, R1-16}, cv::FONT_HERSHEY_SIMPLEX, 0.32, {160,160,160}, 1);
        std::snprintf(buf, sizeof(buf), "%+.3f", e_y);
        cv::putText(panel, buf, {C1, R1},
                    cv::FONT_HERSHEY_SIMPLEX, 0.75, valColor(e_y, 1.0, 2.5), 2);

        cv::putText(panel, "e_psi [deg]", {C2, R1-16}, cv::FONT_HERSHEY_SIMPLEX, 0.32, {160,160,160}, 1);
        std::snprintf(buf, sizeof(buf), "%+.1f", e_psi_deg);
        cv::putText(panel, buf, {C2, R1},
                    cv::FONT_HERSHEY_SIMPLEX, 0.75, valColor(e_psi_deg, 20.0, 45.0), 2);

        cv::putText(panel, "nearest idx", {C3, R1-16}, cv::FONT_HERSHEY_SIMPLEX, 0.32, {160,160,160}, 1);
        std::snprintf(buf, sizeof(buf), "%d", nearest);
        cv::putText(panel, buf, {C3, R1}, cv::FONT_HERSHEY_SIMPLEX, 0.75, {200,200,200}, 2);

        // Row2: target_v | speed_pwm | steer_cmd(MPC)
        cv::putText(panel, "Target v [m/s]", {C1, R2-16}, cv::FONT_HERSHEY_SIMPLEX, 0.32, {160,160,160}, 1);
        std::snprintf(buf, sizeof(buf), "%+.3f", target_v);
        cv::Scalar v_col = (target_v < -0.05) ? cv::Scalar(100,100,255) : cv::Scalar(100,255,150);
        cv::putText(panel, buf, {C1, R2}, cv::FONT_HERSHEY_SIMPLEX, 0.75, v_col, 2);

        cv::putText(panel, "Speed PWM", {C2, R2-16}, cv::FONT_HERSHEY_SIMPLEX, 0.32, {160,160,160}, 1);
        std::snprintf(buf, sizeof(buf), "%+.1f", speed_pwm);
        cv::Scalar pwm_col = (speed_pwm < -5.0) ? cv::Scalar(100,100,255)
                           : (std::abs(speed_pwm) < 20.0) ? cv::Scalar(0,200,255)
                           : cv::Scalar(100,255,150);
        cv::putText(panel, buf, {C2, R2}, cv::FONT_HERSHEY_SIMPLEX, 0.75, pwm_col, 2);

        cv::putText(panel, "MPC steer [deg]", {C3, R2-16}, cv::FONT_HERSHEY_SIMPLEX, 0.32, {160,160,160}, 1);
        std::snprintf(buf, sizeof(buf), "%+.1f", steer_cmd);
        cv::putText(panel, buf, {C3, R2},
                    cv::FONT_HERSHEY_SIMPLEX, 0.75, valColor(steer_cmd, 35.0, 50.0), 2);

        // Row3: cmd_vel 실제 발행값 (Arduino에 가는 값)
        cv::line(panel, {0, R3-26}, {W, R3-26}, {50,50,60}, 1);
        cv::putText(panel, "CMD->Arduino:", {C1, R3-12}, cv::FONT_HERSHEY_SIMPLEX, 0.34, {140,160,140}, 1);
        std::snprintf(buf, sizeof(buf), "PWM %+.1f | Steer %+.1f deg", cmd_pwm_actual, cmd_steer_actual);
        cv::Scalar cmd_col = (cmd_pwm_actual < -5.0) ? cv::Scalar(100,100,255) : cv::Scalar(160,220,160);
        cv::putText(panel, buf, {C1, R3+2}, cv::FONT_HERSHEY_SIMPLEX, 0.50, cmd_col, 1);
    }

    // ── 지도 패널 ─────────────────────────────────────────────
    void drawMap(cv::Mat panel,
                 const std::vector<Pt> & path,
                 const std::vector<Pt> & traveled,
                 double vx, double vy, double vyaw, double vvx, double steer_hw,
                 bool has_odom, bool has_path, int ni,
                 double scale, double ox, double oy,
                 double cte, double hdg_err)
    {
        panel.setTo(cv::Scalar(28,33,28));
        auto w2p = [&](double wx, double wy) -> cv::Point2i {
            return { (int)((wx-ox)*scale),
                     WIN_H - (int)((wy-oy)*scale) };
        };
        if (!has_path) {
            cv::putText(panel, "Waiting /global_path ...",
                        {20, WIN_H/2}, cv::FONT_HERSHEY_SIMPLEX, 0.7,
                        {160,160,160}, 1);
            return;
        }
        // 5m 그리드
        {
            double x1 = ox + MAP_W/scale, y1 = oy + WIN_H/scale;
            for (double gx=std::floor(ox/5)*5; gx<=x1; gx+=5)
                cv::line(panel, w2p(gx,oy), w2p(gx,y1), {45,50,45}, 1);
            for (double gy=std::floor(oy/5)*5; gy<=y1; gy+=5)
                cv::line(panel, w2p(ox,gy), w2p(x1,gy), {45,50,45}, 1);
        }
        // 글로벌 경로
        for (size_t i = 1; i < path.size(); ++i)
            cv::line(panel, w2p(path[i-1].x, path[i-1].y),
                            w2p(path[i].x,   path[i].y),
                     {0,200,0}, 2, cv::LINE_AA);
        for (size_t i = 0; i < path.size(); i += 20) {
            auto base = w2p(path[i].x, path[i].y);
            double len = 0.8 * scale;
            cv::arrowedLine(panel, base,
                {base.x + (int)(len*std::cos(path[i].yaw)),
                 base.y - (int)(len*std::sin(path[i].yaw))},
                {0,150,0}, 1, cv::LINE_AA, 0, 0.35);
        }
        // 주행 경로
        for (size_t i = 1; i < traveled.size(); ++i)
            cv::line(panel, w2p(traveled[i-1].x, traveled[i-1].y),
                            w2p(traveled[i].x,   traveled[i].y),
                     {0,140,255}, 2, cv::LINE_AA);
        // 차량
        if (has_odom) {
            auto vp = w2p(vx, vy);
            int r = std::max(6, (int)(0.45*scale));
            cv::circle(panel, vp, r, {50,50,220}, -1);
            cv::circle(panel, vp, r, {255,255,255}, 1);
            cv::arrowedLine(panel, vp,
                {vp.x+(int)(r*2.8*std::cos(vyaw)),
                 vp.y-(int)(r*2.8*std::sin(vyaw))},
                {80,80,255}, 2, cv::LINE_AA, 0, 0.3);
            if (!path.empty()) {
                auto rp = w2p(path[ni].x, path[ni].y);
                cv::line(panel, vp, rp, {0,230,230}, 1, cv::LINE_AA);
                cv::circle(panel, rp, 4, {0,230,230}, -1);
            }
            // 상태 텍스트
            char buf[160];
            std::snprintf(buf, sizeof(buf),
                "v=%.2f m/s  hw_steer=%.1fdeg  CTE=%.3fm  HE=%.1fdeg",
                vvx, steer_hw, cte, hdg_err);
            cv::rectangle(panel,{0,WIN_H-32},{MAP_W,WIN_H},{20,20,20},-1);
            cv::putText(panel, buf, {8,WIN_H-10},
                        cv::FONT_HERSHEY_SIMPLEX, 0.48, {220,220,220}, 1);
            // 진행률 바
            if (!path.empty()) {
                double prog = (path.size()>1) ? (double)ni/(path.size()-1) : 0.0;
                int bw = (int)(prog*(MAP_W-20));
                cv::rectangle(panel,{10,WIN_H-45},{MAP_W-10,WIN_H-35},{50,50,50},-1);
                cv::rectangle(panel,{10,WIN_H-45},{10+bw,WIN_H-35},{0,200,100},-1);
                char pb[48];
                std::snprintf(pb,sizeof(pb),"%.1f%%  %d/%d",
                    prog*100.0, ni, (int)path.size());
                cv::putText(panel, pb, {MAP_W/2-40,WIN_H-36},
                            cv::FONT_HERSHEY_SIMPLEX, 0.38, {200,200,200}, 1);
            }
        }
        cv::putText(panel, "MPC Debug Monitor", {10,22},
                    cv::FONT_HERSHEY_SIMPLEX, 0.60, {160,220,160}, 1);
    }

    // ── 그래프 ────────────────────────────────────────────────
    void drawGraph(cv::Mat panel,
                   const std::deque<double> & data,
                   const char * label,
                   double y_lo, double y_hi,
                   cv::Scalar color)
    {
        const int W=panel.cols, H=panel.rows;
        panel.setTo(cv::Scalar(20,20,26));
        for (int g=0; g<=2; ++g) {
            int py=(int)((double)g/2*H);
            cv::line(panel,{0,py},{W,py},{42,42,52},1);
            double val=y_hi-(double)g/2*(y_hi-y_lo);
            char buf[20]; std::snprintf(buf,sizeof(buf),"%.0f",val);
            cv::putText(panel,buf,{3,py>10?py-2:10},
                        cv::FONT_HERSHEY_SIMPLEX,0.28,{110,110,120},1);
        }
        // 0선
        if (y_lo<0 && y_hi>0) {
            int py0=(int)(y_hi/(y_hi-y_lo)*H);
            cv::line(panel,{0,py0},{W,py0},{80,80,100},1);
        }
        if (data.size()>=2) {
            int n=(int)data.size();
            for (int i=1;i<n;++i) {
                int x0=(i-1)*(W-1)/(N_HIST-1), x1=i*(W-1)/(N_HIST-1);
                double v0=std::clamp(data[i-1],y_lo,y_hi);
                double v1=std::clamp(data[i],  y_lo,y_hi);
                int p0=(int)((y_hi-v0)/(y_hi-y_lo)*H);
                int p1=(int)((y_hi-v1)/(y_hi-y_lo)*H);
                cv::line(panel,{x0,p0},{x1,p1},color,2,cv::LINE_AA);
            }
            int cx=(n-1)*(W-1)/(N_HIST-1);
            int cp=(int)((y_hi-std::clamp(data.back(),y_lo,y_hi))/(y_hi-y_lo)*H);
            cv::circle(panel,{cx,cp},3,color,-1);
            char vbuf[24]; std::snprintf(vbuf,sizeof(vbuf),"%+.2f",data.back());
            cv::putText(panel,vbuf,{W-58,H-3},cv::FONT_HERSHEY_SIMPLEX,0.38,color,1);
        }
        cv::putText(panel,label,{40,H-3},
                    cv::FONT_HERSHEY_SIMPLEX,0.34,{170,170,170},1);
    }

    // ── 로그 패널 ─────────────────────────────────────────────
    void drawLogPanel(cv::Mat panel, const std::deque<LogEntry> & logs)
    {
        const int W=panel.cols, H=panel.rows;
        panel.setTo(cv::Scalar(15,15,20));
        cv::putText(panel, "Node Log (/rosout)", {6,14},
                    cv::FONT_HERSHEY_SIMPLEX, 0.40, {160,200,160}, 1);
        cv::line(panel,{0,18},{W,18},{60,60,70},1);
        const int line_h = (H - 20) / MAX_LOGS;
        int shown = 0;
        for (int i=(int)logs.size()-1; i>=0 && shown<MAX_LOGS; --i) {
            const auto & e = logs[i];
            cv::Scalar col;
            const char * prefix;
            if      (e.level >= 50) { col={0,50,255};    prefix="[FAT]"; }
            else if (e.level >= 40) { col={0,80,255};    prefix="[ERR]"; }
            else if (e.level >= 30) { col={0,200,255};   prefix="[WRN]"; }
            else                    { col={160,160,160}; prefix="[INF]"; }
            std::string line = std::string(prefix)+" ["+e.name+"] "+e.msg;
            if ((int)line.size() > 74) line = line.substr(0,71)+"...";
            int y = H - shown*line_h - 4;
            cv::putText(panel, line, {5, y},
                        cv::FONT_HERSHEY_SIMPLEX, 0.32, col, 1);
            ++shown;
        }
    }

    // ── 내부 헬퍼 ─────────────────────────────────────────────
    void computeMapTransform()
    {
        if (path_.empty()) return;
        double xmin=path_[0].x, xmax=path_[0].x;
        double ymin=path_[0].y, ymax=path_[0].y;
        for (const auto & p : path_) {
            xmin=std::min(xmin,p.x); xmax=std::max(xmax,p.x);
            ymin=std::min(ymin,p.y); ymax=std::max(ymax,p.y);
        }
        double margin=std::max({xmax-xmin,ymax-ymin,10.0})*0.18;
        xmin-=margin; xmax+=margin; ymin-=margin; ymax+=margin;
        double sx=MAP_W/(xmax-xmin), sy=WIN_H/(ymax-ymin);
        map_scale_=std::min(sx,sy);
        map_ox_=(xmin+xmax)*0.5 - MAP_W*0.5/map_scale_;
        map_oy_=(ymin+ymax)*0.5 - WIN_H*0.5/map_scale_;
    }

    int nearestIdx() const
    {
        int best=0; double best_d2=std::numeric_limits<double>::max();
        for (int i=0;i<(int)path_.size();++i) {
            double d2=(veh_x_-path_[i].x)*(veh_x_-path_[i].x)
                     +(veh_y_-path_[i].y)*(veh_y_-path_[i].y);
            if (d2<best_d2){best_d2=d2;best=i;}
        }
        return best;
    }

    static void pushH(std::deque<double> & h, double v)
    {
        h.push_back(v);
        while ((int)h.size()>N_HIST) h.pop_front();
    }

    // ── 멤버 ─────────────────────────────────────────────────
    std::mutex mu_;

    std::vector<Pt> path_, traveled_;
    bool   map_dirty_{true};
    double map_scale_{1.0}, map_ox_{0.0}, map_oy_{0.0};

    double veh_x_{0},veh_y_{0},veh_yaw_{0},veh_vx_{0};
    bool   has_odom_{false};
    double steer_hw_{0}, cmd_steer_{0}, cmd_pwm_{0};

    // /mpc/debug 값
    double dbg_e_y_{0}, dbg_e_psi_{0}, dbg_target_v_{0};
    double dbg_speed_pwm_{0}, dbg_steer_cmd_{0};
    int    dbg_nearest_{0}, dbg_direction_{1};
    bool   dbg_goal_{false}, has_debug_{false};

    std::deque<double>   cte_hist_, hdg_hist_, spd_hist_, str_hist_, pwm_hist_;
    std::deque<LogEntry> logs_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr               path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr           odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr         cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr            steer_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr  debug_sub_;
    rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr          rosout_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MpcMonitorNode>());
    rclcpp::shutdown();
    cv::destroyAllWindows();
    return 0;
}
