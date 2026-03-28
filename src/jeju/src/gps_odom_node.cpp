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
        declare_parameter("heading_min_mps",   0.15);   // 헤딩 갱신 최소 GPS 속도

        utm_zone_    = get_parameter("utm_zone").as_int();
        odom_frame_  = get_parameter("odom_frame").as_string();
        base_frame_  = get_parameter("base_frame").as_string();
        hdg_min_     = get_parameter("heading_min_mps").as_double();

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

        RCLCPP_INFO(get_logger(),
            "GPS Odom 노드 시작 (UTM zone=%d, hdg_min=%.2f m/s)", utm_zone_, hdg_min_);
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
            geometry_msgs::msg::Point pt;
            pt.x = xg;  pt.y = yg;
            orig_pub_->publish(pt);
            RCLCPP_INFO(get_logger(), "UTM 원점 설정 (%.3f, %.3f)", xg, yg);
        }

        pos_x_  = xg - ox_;
        pos_y_  = yg - oy_;
        stamp_  = m->header.stamp;

        // 수평 정확도 → covariance
        if (m->position_covariance_type > 0 && m->position_covariance[0] > 0) {
            pos_cov_ = m->position_covariance[0];
        }

        publishOdom();
    }

    // ── GPS 속도 (ENU: x=East, y=North) ─────────────────────
    void velCb(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr m)
    {
        vx_enu_ = m->twist.twist.linear.x;   // East
        vy_enu_ = m->twist.twist.linear.y;   // North
        speed_  = std::hypot(vx_enu_, vy_enu_);

        if (speed_ >= hdg_min_) {
            yaw_ = std::atan2(vy_enu_, vx_enu_);  // ENU 기준 yaw

            std_msgs::msg::Float64 hd;
            hd.data = yaw_ * 180.0 / M_PI;
            hdg_pub_->publish(hd);
        }
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

        // 속도 (ENU world frame — MPC tracker가 body-frame으로 변환)
        if (has_vel_) {
            odom.twist.twist.linear.x = vx_enu_;
            odom.twist.twist.linear.y = vy_enu_;
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

    // ── 상태 ─────────────────────────────────────────────────
    int         utm_zone_;
    std::string odom_frame_, base_frame_;
    double      hdg_min_;

    bool   got_origin_{false};
    double ox_{0.0}, oy_{0.0};

    double pos_x_{0.0}, pos_y_{0.0};
    double pos_cov_{1.0};
    rclcpp::Time stamp_{0, 0, RCL_ROS_TIME};

    double vx_enu_{0.0}, vy_enu_{0.0}, speed_{0.0};
    double yaw_{0.0};
    bool   has_vel_{false};
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsOdomNode>());
    rclcpp::shutdown();
    return 0;
}
