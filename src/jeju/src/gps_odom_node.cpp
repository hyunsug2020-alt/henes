// ============================================================
// gps_odom_node.cpp
//
// ESKF 없이 GPS 직접 → /odometry/filtered 발행
//
// 구독:
//   /gnss_left/fix          (sensor_msgs/NavSatFix)
//   /gnss_left/fix_velocity (geometry_msgs/TwistWithCovarianceStamped, ENU)
//
// 발행:
//   /odometry/filtered      (nav_msgs/Odometry)
//   /utm_origin             (geometry_msgs/Point, transient_local)
//   /gps_odom/heading_deg   (std_msgs/Float64)
// ============================================================
#include <cmath>
#include <fstream>
#include <iomanip>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64.hpp>

// ── UTM 변환 (eskf_node.cpp 와 동일 수식) ──────────────────
namespace utm {
static constexpr double kA  = 6378137.0;
static constexpr double kF  = 1.0 / 298.257223563;
static constexpr double kK0 = 0.9996;
static constexpr double kE2 = 2*kF - kF*kF;

inline std::pair<double,double> forward(double lat_deg, double lon_deg, int zone)
{
    double lat = lat_deg * M_PI / 180.0;
    double lon = lon_deg * M_PI / 180.0;
    double lon0 = ((zone-1)*6 - 180 + 3) * M_PI / 180.0;
    double e2 = kE2;
    double N  = kA / std::sqrt(1 - e2*std::sin(lat)*std::sin(lat));
    double T  = std::tan(lat)*std::tan(lat);
    double C  = e2/(1-e2)*std::cos(lat)*std::cos(lat);
    double A  = std::cos(lat)*(lon - lon0);
    double M  = kA*((1 - e2/4 - 3*e2*e2/64 - 5*e2*e2*e2/256)*lat
                   -(3*e2/8 + 3*e2*e2/32 + 45*e2*e2*e2/1024)*std::sin(2*lat)
                   +(15*e2*e2/256 + 45*e2*e2*e2/1024)*std::sin(4*lat)
                   -(35*e2*e2*e2/3072)*std::sin(6*lat));
    double x = kK0*N*(A + (1-T+C)*A*A*A/6
                        + (5-18*T+T*T+72*C-58*e2/(1-e2))*A*A*A*A*A/120) + 500000.0;
    double y = kK0*(M + N*std::tan(lat)*(A*A/2
                        + (5-T+9*C+4*C*C)*A*A*A*A/24
                        + (61-58*T+T*T+600*C-330*e2/(1-e2))*A*A*A*A*A*A/720));
    if (lat_deg < 0.0) y += 10000000.0;
    return {x, y};
}
}  // namespace utm

// ──────────────────────────────────────────────────────────────
class GpsOdomNode : public rclcpp::Node
{
public:
    GpsOdomNode() : Node("gps_odom_node")
    {
        declare_parameter("utm_zone",          52);
        declare_parameter("odom_frame",        std::string("odom"));
        declare_parameter("base_frame",        std::string("base_footprint"));
        declare_parameter("gps_fix_topic",     std::string("/gnss_left/fix"));
        declare_parameter("gps_vel_topic",     std::string("/gnss_left/fix_velocity"));
        declare_parameter("heading_min_m",     0.05);  // RTK 위치 기반 헤딩 갱신 최소 이동 거리 [m]
        declare_parameter("max_speed_mps",     3.0);   // 발행 속도 상한 [m/s] (GPS 스파이크 방지)
        declare_parameter("origin_file",       std::string("/tmp/gps_odom_origin.txt"));

        utm_zone_      = get_parameter("utm_zone").as_int();
        odom_frame_    = get_parameter("odom_frame").as_string();
        base_frame_    = get_parameter("base_frame").as_string();
        hdg_min_m_     = get_parameter("heading_min_m").as_double();
        max_speed_mps_ = get_parameter("max_speed_mps").as_double();
        origin_file_   = get_parameter("origin_file").as_string();

        auto sq = rclcpp::SensorDataQoS();
        auto tl = rclcpp::QoS(1).reliable().transient_local();

        fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            get_parameter("gps_fix_topic").as_string(), sq,
            std::bind(&GpsOdomNode::fixCb, this, std::placeholders::_1));

        vel_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
            get_parameter("gps_vel_topic").as_string(), sq,
            std::bind(&GpsOdomNode::velCb, this, std::placeholders::_1));

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odometry/filtered", 20);
        orig_pub_ = create_publisher<geometry_msgs::msg::Point>("/utm_origin", tl);
        hdg_pub_  = create_publisher<std_msgs::msg::Float64>("/gps_odom/heading_deg", 10);

        // ── 저장된 UTM 원점 로드 (재시작 시 좌표 연속성 유지) ──
        loadOrigin();

        RCLCPP_INFO(get_logger(),
            "GPS Odom 노드 시작 (UTM zone=%d, hdg_min_m=%.3f m, origin_file=%s)",
            utm_zone_, hdg_min_m_, origin_file_.c_str());
    }

private:
    // ── GPS 위치 ───────────────────────────────────────────────
    void fixCb(const sensor_msgs::msg::NavSatFix::SharedPtr m)
    {
        if (std::isnan(m->latitude) || std::isnan(m->longitude)) return;
        if (m->status.status < 0) return;  // NO_FIX

        auto [xg, yg] = utm::forward(m->latitude, m->longitude, utm_zone_);

        if (!got_origin_) {
            ox_ = xg;  oy_ = yg;
            got_origin_ = true;
            saveOrigin();
            geometry_msgs::msg::Point pt;
            pt.x = xg;  pt.y = yg;
            orig_pub_->publish(pt);
            RCLCPP_INFO(get_logger(), "UTM 원점 설정 (%.3f, %.3f) → %s 저장",
                        xg, yg, origin_file_.c_str());
        }

        pos_x_  = xg - ox_;
        pos_y_  = yg - oy_;
        stamp_  = m->header.stamp;

        // 수평 정확도 → covariance
        if (m->position_covariance_type > 0 && m->position_covariance[0] > 0) {
            pos_cov_ = m->position_covariance[0];
        }

        // ── RTK 위치 기반 헤딩 계산 ───────────────────────────
        // velocity heading 대신 연속 GPS 위치 차이 사용
        // RTK 정확도(2cm)로 5cm 이동만 해도 신뢰 가능
        if (got_prev_pos_) {
            const double dx   = pos_x_ - prev_pos_x_;
            const double dy   = pos_y_ - prev_pos_y_;
            const double dist = std::hypot(dx, dy);
            if (dist >= hdg_min_m_) {
                yaw_ = std::atan2(dy, dx);
                std_msgs::msg::Float64 hd;
                hd.data = yaw_ * 180.0 / M_PI;
                hdg_pub_->publish(hd);
            }
        }
        prev_pos_x_ = pos_x_;
        prev_pos_y_ = pos_y_;
        got_prev_pos_ = true;

        publishOdom();
    }

    // ── GPS 속도 (ENU: x=East, y=North) ─────────────────────
    // 헤딩은 fixCb의 RTK 위치 차이로 계산 (더 정확)
    // 여기서는 odom twist 계산용 속도 성분만 저장
    void velCb(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr m)
    {
        vx_enu_ = m->twist.twist.linear.x;   // East
        vy_enu_ = m->twist.twist.linear.y;   // North
        speed_  = std::hypot(vx_enu_, vy_enu_);
        has_vel_ = true;
    }

    // ── 발행 ─────────────────────────────────────────────────
    void publishOdom()
    {
        if (!got_origin_) return;

        nav_msgs::msg::Odometry odom;
        odom.header.stamp    = stamp_;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id  = base_frame_;

        // 위치 (ENU)
        odom.pose.pose.position.x = pos_x_;
        odom.pose.pose.position.y = pos_y_;
        odom.pose.pose.position.z = 0.0;

        // 자세 (GPS 헤딩 → quaternion)
        odom.pose.pose.orientation.x = 0.0;
        odom.pose.pose.orientation.y = 0.0;
        odom.pose.pose.orientation.z = std::sin(yaw_ * 0.5);
        odom.pose.pose.orientation.w = std::cos(yaw_ * 0.5);

        // pose covariance (대각: x, y, yaw)
        odom.pose.covariance[0]  = pos_cov_;
        odom.pose.covariance[7]  = pos_cov_;
        odom.pose.covariance[35] = 0.05;

        // 속도: ENU → body frame 변환 후 상한 클램핑 (GPS 스파이크 방지)
        if (has_vel_) {
            const double cy = std::cos(yaw_);
            const double sy = std::sin(yaw_);
            double vx =  vx_enu_ * cy + vy_enu_ * sy;  // 전진
            double vy = -vx_enu_ * sy + vy_enu_ * cy;  // 횡방향
            // 속도 크기 제한
            const double spd = std::hypot(vx, vy);
            if (spd > max_speed_mps_) {
                const double scale = max_speed_mps_ / spd;
                vx *= scale;  vy *= scale;
            }
            odom.twist.twist.linear.x = vx;
            odom.twist.twist.linear.y = vy;
        }

        odom_pub_->publish(odom);
    }

    // ── 구독 ─────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr vel_sub_;

    // ── 발행 ─────────────────────────────────────────────────
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr orig_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr hdg_pub_;

    // ── UTM origin 파일 저장/로드 ────────────────────────────
    void loadOrigin()
    {
        std::ifstream f(origin_file_);
        if (!f.is_open()) return;
        double ox, oy;
        if (f >> ox >> oy) {
            ox_ = ox;  oy_ = oy;
            got_origin_ = true;
            geometry_msgs::msg::Point pt;
            pt.x = ox_;  pt.y = oy_;
            orig_pub_->publish(pt);
            RCLCPP_INFO(get_logger(), "UTM 원점 로드 (%.3f, %.3f) ← %s",
                        ox_, oy_, origin_file_.c_str());
        }
    }

    void saveOrigin()
    {
        std::ofstream f(origin_file_);
        if (f.is_open()) {
            f << std::fixed << std::setprecision(6) << ox_ << " " << oy_ << "\n";
        }
    }

    // ── 상태 ─────────────────────────────────────────────────
    int         utm_zone_;
    std::string odom_frame_, base_frame_;
    double      hdg_min_m_;     // RTK 위치 기반 헤딩 갱신 최소 거리 [m]
    double      max_speed_mps_; // 발행 속도 상한 [m/s]
    std::string origin_file_;

    bool   got_origin_{false};
    double ox_{0.0}, oy_{0.0};

    double pos_x_{0.0}, pos_y_{0.0};
    double pos_cov_{1.0};
    rclcpp::Time stamp_{0, 0, RCL_ROS_TIME};

    double vx_enu_{0.0}, vy_enu_{0.0}, speed_{0.0};
    double yaw_{0.0};
    bool   has_vel_{false};

    // RTK 위치 기반 헤딩 계산용
    double prev_pos_x_{0.0}, prev_pos_y_{0.0};
    bool   got_prev_pos_{false};
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsOdomNode>());
    rclcpp::shutdown();
    return 0;
}
