// mpc_monitor_node.cpp
// MPC 실시간 모니터링 GUI (OpenCV)
//
// 레이아웃:
//   왼쪽(MAP_W×WIN_H)        : 경로 지도 + 차량 위치 + 오차선
//   오른쪽 상단(GRAPH_W×560) : 4개 그래프 (횡방향 오차 / 헤딩 오차 / 속도 / 조향각)
//   오른쪽 하단(GRAPH_W×160) : MPC 로그 패널 (/rosout)

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

// ────────────────────────── 헬퍼 ──────────────────────────────
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
    uint8_t     level;   // 10=DEBUG 20=INFO 30=WARN 40=ERROR 50=FATAL
    std::string name;    // 노드명
    std::string msg;
};

// ───────────────────────── 노드 ───────────────────────────────
class MpcMonitorNode : public rclcpp::Node
{
public:
    static constexpr int WIN_W    = 1280;
    static constexpr int WIN_H    = 720;
    static constexpr int MAP_W    = 800;
    static constexpr int GRAPH_W  = WIN_W - MAP_W;
    static constexpr int LOG_H    = 165;          // 로그 패널 높이
    static constexpr int GRAPHS_H = WIN_H - LOG_H;
    static constexpr int N_GRAPH  = 4;
    static constexpr int GRAPH_H  = GRAPHS_H / N_GRAPH;
    static constexpr int N_HIST   = 400;
    static constexpr int MAX_LOGS = 10;           // 화면에 보여줄 최대 줄 수

    MpcMonitorNode() : Node("mpc_monitor_node")
    {
        // /global_path
        auto qos_tl = rclcpp::QoS(1).reliable().transient_local();
        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/global_path", qos_tl,
            [this](nav_msgs::msg::Path::SharedPtr m){ pathCb(m); });

        // /odometry/filtered
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 20,
            [this](nav_msgs::msg::Odometry::SharedPtr m){ odomCb(m); });

        // /cmd_vel
        cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 20,
            [this](geometry_msgs::msg::Twist::SharedPtr m){
                std::lock_guard<std::mutex> g(mu_);
                cmd_steer_ = m->angular.z;
                cmd_pwm_   = m->linear.x;
            });

        // /steering_angle
        steer_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/steering_angle", 20,
            [this](std_msgs::msg::Float64::SharedPtr m){
                std::lock_guard<std::mutex> g(mu_);
                steer_deg_ = m->data;
            });

        // /rosout  →  MPC 로그 수집
        rosout_sub_ = create_subscription<rcl_interfaces::msg::Log>(
            "/rosout", rclcpp::QoS(50),
            [this](rcl_interfaces::msg::Log::SharedPtr m){ logCb(m); });

        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            [this](){ render(); });

        cv::namedWindow("MPC Monitor", cv::WINDOW_NORMAL);
        cv::resizeWindow("MPC Monitor", WIN_W, WIN_H);
        RCLCPP_INFO(get_logger(), "MPC Monitor 시작");
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
        veh_vx_   =  m->twist.twist.linear.x * cy + m->twist.twist.linear.y * sy;
        has_odom_ = true;
        if (traveled_.empty() ||
            std::hypot(veh_x_ - traveled_.back().x,
                       veh_y_ - traveled_.back().y) > 0.1) {
            traveled_.push_back({veh_x_, veh_y_, veh_yaw_});
            if (traveled_.size() > 20000) traveled_.erase(traveled_.begin());
        }
    }

    void logCb(const rcl_interfaces::msg::Log::SharedPtr m)
    {
        // DEBUG(10) 제외, INFO 이상만 표시
        if (m->level < 20) return;

        std::lock_guard<std::mutex> g(mu_);
        LogEntry e;
        e.level = m->level;
        e.name  = m->name;
        e.msg   = m->msg;
        logs_.push_back(std::move(e));
        while ((int)logs_.size() > MAX_LOGS * 3) logs_.pop_front();
    }

    // ── 렌더 ──────────────────────────────────────────────────
    void render()
    {
        // 데이터 복사
        std::vector<Pt> path, traveled;
        double vx, vy, vyaw, vvx;
        bool has_odom, has_path;
        double steer, cmd_steer, cmd_pwm;
        double cte=0, hdg_err=0;
        std::deque<double> h_cte, h_hdg, h_spd, h_str;
        double map_scale, map_ox, map_oy;
        int nearest_i = 0;
        std::deque<LogEntry> logs_copy;

        {
            std::lock_guard<std::mutex> g(mu_);
            path     = path_;
            traveled = traveled_;
            vx = veh_x_; vy = veh_y_; vyaw = veh_yaw_; vvx = veh_vx_;
            has_odom = has_odom_;
            has_path = !path_.empty();
            steer    = steer_deg_;
            cmd_steer = cmd_steer_; cmd_pwm = cmd_pwm_;

            if (has_odom && has_path) {
                if (map_dirty_) { computeMapTransform(); map_dirty_ = false; }
                nearest_i = nearestIdx();
                const auto & ref = path_[nearest_i];
                double ex = veh_x_ - ref.x;
                double ey = veh_y_ - ref.y;
                cte     = -std::sin(ref.yaw)*ex + std::cos(ref.yaw)*ey;
                hdg_err =  normAngle(veh_yaw_ - ref.yaw) * 180.0 / M_PI;
                pushH(cte_hist_, cte);
                pushH(hdg_hist_, hdg_err);
                pushH(spd_hist_, std::abs(vvx));
                pushH(str_hist_, steer);
            }
            h_cte = cte_hist_; h_hdg = hdg_hist_;
            h_spd = spd_hist_; h_str = str_hist_;
            map_scale = map_scale_; map_ox = map_ox_; map_oy = map_oy_;
            logs_copy = logs_;
        }

        cv::Mat canvas(WIN_H, WIN_W, CV_8UC3, cv::Scalar(28,28,28));

        // 지도
        drawMap(canvas(cv::Rect(0, 0, MAP_W, WIN_H)),
                path, traveled, vx, vy, vyaw, vvx, steer,
                has_odom, has_path, nearest_i,
                map_scale, map_ox, map_oy, cte, hdg_err);

        // 그래프 (상단)
        {
            struct G { const std::deque<double>* d; const char* lbl; double lo,hi; cv::Scalar col; }
            graphs[4] = {
                {&h_cte, "Lateral Error [m]",   -2.0,  2.0, {0,220,255}},
                {&h_hdg, "Heading Error [deg]", -30.0, 30.0, {255,200,0}},
                {&h_spd, "Speed [m/s]",          0.0,  2.0, {0,200,100}},
                {&h_str, "Steering [deg]",      -60.0, 60.0, {200,100,255}},
            };
            for (int i = 0; i < 4; ++i) {
                cv::Mat row = canvas(cv::Rect(MAP_W, i*GRAPH_H, GRAPH_W, GRAPH_H));
                drawGraph(row, *graphs[i].d, graphs[i].lbl,
                          graphs[i].lo, graphs[i].hi, graphs[i].col);
                if (i > 0)
                    cv::line(canvas,
                             {MAP_W, i*GRAPH_H}, {WIN_W, i*GRAPH_H},
                             {60,60,70}, 1);
            }
        }

        // 로그 패널 (하단)
        {
            cv::Mat lp = canvas(cv::Rect(MAP_W, GRAPHS_H, GRAPH_W, LOG_H));
            drawLogPanel(lp, logs_copy);
        }

        // 구분선
        cv::line(canvas, {MAP_W, 0},       {MAP_W, WIN_H},  {70,70,70}, 1);
        cv::line(canvas, {MAP_W, GRAPHS_H},{WIN_W, GRAPHS_H},{90,90,90}, 1);

        cv::imshow("MPC Monitor", canvas);
        cv::waitKey(1);
    }

    // ── 지도 패널 ─────────────────────────────────────────────
    void drawMap(cv::Mat panel,
                 const std::vector<Pt> & path,
                 const std::vector<Pt> & traveled,
                 double vx, double vy, double vyaw, double vvx, double steer,
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
            cv::putText(panel, "Waiting for /global_path ...",
                        {20, WIN_H/2}, cv::FONT_HERSHEY_SIMPLEX, 0.7,
                        {160,160,160}, 1);
            return;
        }
        // 5m 그리드
        {
            const double g5 = 5.0;
            double x1 = ox + MAP_W/scale, y1 = oy + WIN_H/scale;
            for (double gx=std::floor(ox/g5)*g5; gx<=x1; gx+=g5)
                cv::line(panel, w2p(gx,oy), w2p(gx,y1), {45,50,45}, 1);
            for (double gy=std::floor(oy/g5)*g5; gy<=y1; gy+=g5)
                cv::line(panel, w2p(ox,gy), w2p(x1,gy), {45,50,45}, 1);
        }
        // 글로벌 경로 (초록)
        for (size_t i = 1; i < path.size(); ++i)
            cv::line(panel, w2p(path[i-1].x,path[i-1].y),
                            w2p(path[i].x,  path[i].y),
                     {0,200,0}, 2, cv::LINE_AA);
        // 방향 화살표
        for (size_t i = 0; i < path.size(); i += 20) {
            auto base = w2p(path[i].x, path[i].y);
            double len = 0.8 * scale;
            cv::arrowedLine(panel, base,
                {base.x+(int)(len*std::cos(path[i].yaw)),
                 base.y-(int)(len*std::sin(path[i].yaw))},
                {0,150,0}, 1, cv::LINE_AA, 0, 0.35);
        }
        // 주행 경로 (주황)
        for (size_t i = 1; i < traveled.size(); ++i)
            cv::line(panel, w2p(traveled[i-1].x,traveled[i-1].y),
                            w2p(traveled[i].x,  traveled[i].y),
                     {0,140,255}, 2, cv::LINE_AA);
        // 차량
        if (has_odom) {
            auto vp = w2p(vx, vy);
            int r = std::max(6, (int)(0.45*scale));
            cv::circle(panel, vp, r, {50,50,220}, -1);
            cv::circle(panel, vp, r, {255,255,255}, 1);
            double alen = r*2.8;
            cv::arrowedLine(panel, vp,
                {vp.x+(int)(alen*std::cos(vyaw)),
                 vp.y-(int)(alen*std::sin(vyaw))},
                {80,80,255}, 2, cv::LINE_AA, 0, 0.3);
            // 오차선
            if (!path.empty()) {
                auto rp = w2p(path[ni].x, path[ni].y);
                cv::line(panel, vp, rp, {0,230,230}, 1, cv::LINE_AA);
                cv::circle(panel, rp, 4, {0,230,230}, -1);
            }
            // 상태 텍스트
            char buf[160];
            std::snprintf(buf, sizeof(buf),
                "v=%.2f m/s  steer=%.1f deg  CTE=%.3f m  HE=%.1f deg",
                std::abs(vvx), steer, cte, hdg_err);
            cv::rectangle(panel,{0,WIN_H-32},{MAP_W,WIN_H},{20,20,20},-1);
            cv::putText(panel, buf, {8,WIN_H-10},
                        cv::FONT_HERSHEY_SIMPLEX, 0.50, {220,220,220}, 1);
            // 진행률 바
            if (!path.empty()) {
                double prog=(path.size()>1)?(double)ni/(path.size()-1):0.0;
                int bw=(int)(prog*(MAP_W-20));
                cv::rectangle(panel,{10,WIN_H-45},{MAP_W-10,WIN_H-35},{50,50,50},-1);
                cv::rectangle(panel,{10,WIN_H-45},{10+bw,WIN_H-35},{0,200,100},-1);
                char pb[48];
                std::snprintf(pb,sizeof(pb),"%.1f%%  %d/%d",
                    prog*100.0, ni,(int)path.size());
                cv::putText(panel, pb,{MAP_W/2-40,WIN_H-36},
                            cv::FONT_HERSHEY_SIMPLEX, 0.38, {200,200,200}, 1);
            }
        }
        cv::putText(panel, "MPC Path Monitor", {10,22},
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
        for (int g=0; g<=4; ++g) {
            int py=(int)((double)g/4*H);
            cv::line(panel,{0,py},{W,py},{42,42,52},1);
            double val=y_hi-(double)g/4*(y_hi-y_lo);
            char buf[20]; std::snprintf(buf,sizeof(buf),"%.1f",val);
            cv::putText(panel,buf,{3,py>12?py-2:12},
                        cv::FONT_HERSHEY_SIMPLEX,0.30,{110,110,120},1);
        }
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
            cv::circle(panel,{cx,cp},4,color,-1);
            char vbuf[24]; std::snprintf(vbuf,sizeof(vbuf),"%.2f",data.back());
            cv::putText(panel,vbuf,{W-52,14},cv::FONT_HERSHEY_SIMPLEX,0.42,color,1);
        }
        cv::putText(panel,label,{W/2-55,H-5},
                    cv::FONT_HERSHEY_SIMPLEX,0.38,{170,170,170},1);
    }

    // ── 로그 패널 ─────────────────────────────────────────────
    void drawLogPanel(cv::Mat panel, const std::deque<LogEntry> & logs)
    {
        const int W=panel.cols, H=panel.rows;
        panel.setTo(cv::Scalar(15,15,20));

        // 제목
        cv::putText(panel, "MPC Log", {6,14},
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, {160,200,160}, 1);
        cv::line(panel,{0,18},{W,18},{60,60,70},1);

        // 최신 MAX_LOGS 줄 (아래에서 위 순서로 역순 출력)
        const int line_h = (H - 20) / MAX_LOGS;
        int shown = 0;
        for (int i = (int)logs.size()-1; i >= 0 && shown < MAX_LOGS; --i) {
            const auto & e = logs[i];
            cv::Scalar col;
            const char * prefix;
            if      (e.level >= 50) { col={0,50,255};   prefix="[FATAL]"; }
            else if (e.level >= 40) { col={0,80,255};   prefix="[ERROR]"; }
            else if (e.level >= 30) { col={0,200,255};  prefix="[WARN] "; }
            else                    { col={180,180,180}; prefix="[INFO] "; }

            // 노드명 + 메시지 (길면 자름)
            std::string line = std::string(prefix) + " [" + e.name + "] " + e.msg;
            if ((int)line.size() > 72) line = line.substr(0,69) + "...";

            int y = H - shown * line_h - 6;
            cv::putText(panel, line, {6, y},
                        cv::FONT_HERSHEY_SIMPLEX, 0.35, col, 1);
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
    double steer_deg_{0}, cmd_steer_{0}, cmd_pwm_{0};

    std::deque<double>   cte_hist_, hdg_hist_, spd_hist_, str_hist_;
    std::deque<LogEntry> logs_;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr       path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr   odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr    steer_sub_;
    rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr  rosout_sub_;
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
