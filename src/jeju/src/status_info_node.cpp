// -*- coding: utf-8 -*-
/*
 * status_info_node.cpp
 * GPS + IMU → 로컬 오도메트리 + 헤딩 발행
 * Python status_info_node.py 의 C++ 변환
 *
 * 구독:
 *   NavSatFix      (GPS 위치)
 *   TwistWithCovarianceStamped (GPS 속도)
 *   Imu            (자세 / pitch for slope)
 *   Float64        /dual_f9p/heading
 *   Bool           /dual_f9p/heading_valid
 *
 * 발행:
 *   Odometry       /odometry/filtered
 *   Float64        /ublox_gps/heading  [deg]
 *   Float64        /slope_factor
 *   Float64        /gps/quality
 *   Point          /utm_origin
 */

#include <cmath>
#include <string>
#include <tuple>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <rclcpp/qos.hpp>

// ─────────────────────────────────────────────
// 간이 UTM 투영 (WGS84)
// ─────────────────────────────────────────────
namespace utm {
    static constexpr double kA  = 6378137.0;
    static constexpr double kF  = 1.0 / 298.257223563;
    static constexpr double kK0 = 0.9996;
    static constexpr double kE2 = 2*kF - kF*kF;

    inline std::pair<double,double> forward(double lat_deg, double lon_deg, int zone)
    {
        double lat = lat_deg * M_PI / 180.0;
        double lon = lon_deg * M_PI / 180.0;
        double lon0 = ((zone - 1) * 6 - 180 + 3) * M_PI / 180.0;

        double N = kA / std::sqrt(1 - kE2 * std::sin(lat)*std::sin(lat));
        double T = std::tan(lat)*std::tan(lat);
        double C = kE2 / (1 - kE2) * std::cos(lat)*std::cos(lat);
        double A = std::cos(lat) * (lon - lon0);

        double e2 = kE2;
        double M  = kA * ((1 - e2/4 - 3*e2*e2/64 - 5*e2*e2*e2/256)*lat
                         -(3*e2/8 + 3*e2*e2/32 + 45*e2*e2*e2/1024)*std::sin(2*lat)
                         +(15*e2*e2/256 + 45*e2*e2*e2/1024)*std::sin(4*lat)
                         -(35*e2*e2*e2/3072)*std::sin(6*lat));

        double x = kK0 * N * (A + (1-T+C)*A*A*A/6
                              + (5-18*T+T*T+72*C-58*e2/(1-e2))*A*A*A*A*A/120)
                  + 500000.0;
        double y = kK0 * (M + N*std::tan(lat)*(A*A/2
                          + (5-T+9*C+4*C*C)*A*A*A*A/24
                          + (61-58*T+T*T+600*C-330*e2/(1-e2))*A*A*A*A*A*A/720));
        if (lat_deg < 0) y += 10000000.0;
        return {x, y};
    }
}

// ─────────────────────────────────────────────
// 각도 유틸
// ─────────────────────────────────────────────
static inline double wrapAngle(double a)
{
    a = std::fmod(a + M_PI, 2.0*M_PI);
    return (a < 0.0) ? a + M_PI : a - M_PI;
}

static inline double interpAngle(double cur, double tgt, double alpha)
{
    alpha = std::max(0.0, std::min(1.0, alpha));
    return wrapAngle(cur + alpha * wrapAngle(tgt - cur));
}

static std::tuple<double,double,double> quatToEuler(double x, double y, double z, double w)
{
    double roll  = std::atan2(2*(w*x+y*z), 1-2*(x*x+y*y));
    double sinp  = 2*(w*y-z*x);
    double pitch = std::asin(std::max(-1.0, std::min(1.0, sinp)));
    double yaw   = std::atan2(2*(w*z+x*y), 1-2*(y*y+z*z));
    return {roll, pitch, yaw};
}

static std::tuple<double,double,double,double> eulerToQuat(double roll, double pitch, double yaw)
{
    double cy = std::cos(yaw*0.5), sy = std::sin(yaw*0.5);
    double cp = std::cos(pitch*0.5), sp = std::sin(pitch*0.5);
    double cr = std::cos(roll*0.5),  sr = std::sin(roll*0.5);
    return {
        cr*cp*cy + sr*sp*sy,   // w
        sr*cp*cy - cr*sp*sy,   // x
        cr*sp*cy + sr*cp*sy,   // y
        cr*cp*sy - sr*sp*cy    // z
    };
}

// ─────────────────────────────────────────────
// 노드
// ─────────────────────────────────────────────
class StatusInfoNode : public rclcpp::Node
{
public:
    StatusInfoNode() : Node("status_info_node")
    {
        declare_parameter("utm_zone",                          52);
        declare_parameter("odom_frame",                        "odom");
        declare_parameter("base_frame",                        "base_footprint");
        declare_parameter("gps_fix_topic",                     "/gnss_left/fix");
        declare_parameter("gps_vel_topic",                     "/ublox_gps/fix_velocity");
        declare_parameter("imu_topic",                         "/handsfree/imu");
        declare_parameter("dual_heading_topic",                "/dual_f9p/heading");
        declare_parameter("dual_heading_valid_topic",          "/dual_f9p/heading_valid");
        declare_parameter("gps_heading_speed_threshold_mps",   0.7);
        declare_parameter("gps_heading_speed_hysteresis_mps",  0.2);
        declare_parameter("gps_heading_alpha",                 0.35);
        declare_parameter("imu_bias_alpha",                    0.1);
        declare_parameter("dual_heading_alpha",                1.0);
        declare_parameter("dual_heading_timeout_sec",          0.5);
        declare_parameter("yaw_offset_deg",                    0.0);
        declare_parameter("slope_filter_alpha",                0.2);
        declare_parameter("pitch_threshold",                   0.05);
        declare_parameter("max_pitch",                         0.35);
        declare_parameter("max_uphill_factor",                 4.5);
        declare_parameter("max_downhill_factor",               0.8);

        utm_zone_         = get_parameter("utm_zone").as_int();
        odom_frame_       = get_parameter("odom_frame").as_string();
        base_frame_       = get_parameter("base_frame").as_string();
        spd_thresh_       = get_parameter("gps_heading_speed_threshold_mps").as_double();
        spd_hyst_         = get_parameter("gps_heading_speed_hysteresis_mps").as_double();
        gps_hdg_alpha_    = get_parameter("gps_heading_alpha").as_double();
        imu_bias_alpha_   = get_parameter("imu_bias_alpha").as_double();
        dual_hdg_alpha_   = get_parameter("dual_heading_alpha").as_double();
        dual_timeout_sec_ = get_parameter("dual_heading_timeout_sec").as_double();
        yaw_offset_rad_   = get_parameter("yaw_offset_deg").as_double() * M_PI / 180.0;
        slope_alpha_      = get_parameter("slope_filter_alpha").as_double();
        pitch_thresh_     = get_parameter("pitch_threshold").as_double();
        max_pitch_        = get_parameter("max_pitch").as_double();
        max_uphill_       = get_parameter("max_uphill_factor").as_double();
        max_downhill_     = get_parameter("max_downhill_factor").as_double();

        auto sensor_qos = rclcpp::SensorDataQoS();
        auto reliable_transient = rclcpp::QoS(1)
            .reliable().transient_local();

        // ── 구독 ──
        fix_sub_  = create_subscription<sensor_msgs::msg::NavSatFix>(
            get_parameter("gps_fix_topic").as_string(), sensor_qos,
            std::bind(&StatusInfoNode::fixCb, this, std::placeholders::_1));
        vel_sub_  = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
            get_parameter("gps_vel_topic").as_string(), sensor_qos,
            std::bind(&StatusInfoNode::velCb, this, std::placeholders::_1));
        imu_sub_  = create_subscription<sensor_msgs::msg::Imu>(
            get_parameter("imu_topic").as_string(), sensor_qos,
            std::bind(&StatusInfoNode::imuCb, this, std::placeholders::_1));
        dhdg_sub_ = create_subscription<std_msgs::msg::Float64>(
            get_parameter("dual_heading_topic").as_string(), 10,
            std::bind(&StatusInfoNode::dualHdgCb, this, std::placeholders::_1));
        dval_sub_ = create_subscription<std_msgs::msg::Bool>(
            get_parameter("dual_heading_valid_topic").as_string(), 10,
            std::bind(&StatusInfoNode::dualValidCb, this, std::placeholders::_1));

        // ── 발행 ──
        odom_pub_    = create_publisher<nav_msgs::msg::Odometry>("/odometry/filtered", 10);
        hdg_pub_     = create_publisher<std_msgs::msg::Float64>("/ublox_gps/heading", 10);
        slope_pub_   = create_publisher<std_msgs::msg::Float64>("/slope_factor", 10);
        quality_pub_ = create_publisher<std_msgs::msg::Float64>("/gps/quality", 10);
        origin_pub_  = create_publisher<geometry_msgs::msg::Point>("/utm_origin", reliable_transient);

        timer_ = create_timer(this, get_clock(),
            rclcpp::Duration::from_seconds(0.02),
            std::bind(&StatusInfoNode::publishState, this));

        RCLCPP_INFO(get_logger(), "status_info_node 시작 (UTM zone=%d)", utm_zone_);
    }

private:
    // ── 콜백 ─────────────────────────────────────────

    void fixCb(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        if (std::isnan(msg->latitude) || std::isnan(msg->longitude)) return;

        auto [x, y] = utm::forward(msg->latitude, msg->longitude, utm_zone_);

        if (!origin_set_) {
            origin_x_   = x;
            origin_y_   = y;
            origin_set_ = true;
            geometry_msgs::msg::Point pt;
            pt.x = x; pt.y = y;
            origin_pub_->publish(pt);
            RCLCPP_INFO(get_logger(), "UTM 원점 설정: (%.3f, %.3f)", x, y);
        }
        local_x_ = x - origin_x_;
        local_y_ = y - origin_y_;
        has_gps_ = true;
        last_gps_time_ = now().seconds();

        std_msgs::msg::Float64 q;
        q.data = estimateGpsQuality(*msg);
        quality_pub_->publish(q);
    }

    void velCb(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
    {
        vel_x_    = msg->twist.twist.linear.x;
        vel_y_    = msg->twist.twist.linear.y;
        gps_spd_  = std::hypot(vel_x_, vel_y_);
        last_vel_time_ = now().seconds();

        if (!gpsCourseReliable()) { has_gps_course_ = false; return; }
        double gps_hdg = std::atan2(vel_y_, vel_x_);
        has_gps_course_ = true;
        updateImuBias(gps_hdg);
        if (!dualReliable())
            updateHeading(gps_hdg, gps_hdg_alpha_, "gps_velocity");
    }

    void imuCb(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        auto [roll, pitch, raw_yaw] = quatToEuler(
            msg->orientation.x, msg->orientation.y,
            msg->orientation.z, msg->orientation.w);
        roll_     = roll;
        pitch_    = pitch;
        imu_yaw_  = wrapAngle(raw_yaw + yaw_offset_rad_);
        imu_yaw_rate_ = msg->angular_velocity.z;
        has_imu_  = true;
        last_imu_time_ = now().seconds();

        if (dualReliable())
            updateImuBias(dual_hdg_rad_);
        else if (!gpsCourseReliable() && has_imu_bias_)
            updateHeading(getImuAlignedHeading(), 1.0, "imu_fallback");

        // slope factor
        double raw_factor = 1.0;
        if (std::fabs(pitch_) > pitch_thresh_) {
            double norm = std::min(std::fabs(pitch_) / max_pitch_, 1.0);
            if (pitch_ < 0.0)
                raw_factor = 1.0 + norm * (max_uphill_ - 1.0);
            else
                raw_factor = 1.0 - norm * (1.0 - max_downhill_);
        }
        slope_factor_ = slope_factor_ * (1.0 - slope_alpha_) + raw_factor * slope_alpha_;
    }

    void dualHdgCb(const std_msgs::msg::Float64::SharedPtr msg)
    {
        dual_hdg_rad_ = wrapAngle(msg->data * M_PI / 180.0);
        has_dual_     = true;
        last_dual_time_ = now().seconds();
        updateImuBias(dual_hdg_rad_);
        if (dualReliable())
            updateHeading(dual_hdg_rad_, dual_hdg_alpha_, "dual_f9p");
    }

    void dualValidCb(const std_msgs::msg::Bool::SharedPtr msg)
    {
        dual_valid_      = msg->data;
        seen_dual_valid_ = true;
        last_dual_valid_time_ = now().seconds();
    }

    // ── 발행 ─────────────────────────────────────────

    void publishState()
    {
        if (!has_gps_ || !has_heading_) return;

        rclcpp::Time cur = now();

        nav_msgs::msg::Odometry odom;
        odom.header.stamp    = cur;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id  = base_frame_;
        odom.pose.pose.position.x = local_x_;
        odom.pose.pose.position.y = local_y_;

        auto [qw, qx, qy, qz] = eulerToQuat(0.0, 0.0, heading_rad_);
        odom.pose.pose.orientation.w = qw;
        odom.pose.pose.orientation.x = qx;
        odom.pose.pose.orientation.y = qy;
        odom.pose.pose.orientation.z = qz;
        odom.twist.twist.linear.x  = vel_x_;
        odom.twist.twist.linear.y  = vel_y_;
        odom.twist.twist.angular.z = imu_yaw_rate_;
        odom_pub_->publish(odom);

        std_msgs::msg::Float64 hd, sl;
        hd.data = heading_rad_ * 180.0 / M_PI;
        sl.data = slope_factor_;
        hdg_pub_->publish(hd);
        slope_pub_->publish(sl);

        double now_sec = cur.seconds();
        if (now_sec - last_gps_time_ > 1.5)
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "GPS timeout > 1.5s");
        if (now_sec - last_vel_time_ > 1.5)
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "GPS vel timeout > 1.5s");
        if (has_gps_ && !has_imu_ && now_sec - last_imu_time_ > 1.5)
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "IMU timeout > 1.5s");
    }

    // ── 내부 헬퍼 ─────────────────────────────────────

    bool gpsCourseReliable()
    {
        double thresh = has_gps_course_
            ? std::max(0.0, spd_thresh_ - spd_hyst_)
            : spd_thresh_;
        return gps_spd_ >= thresh;
    }

    bool dualReliable()
    {
        if (!has_dual_) return false;
        double now_sec = now().seconds();
        if (now_sec - last_dual_time_ > dual_timeout_sec_) return false;
        if (seen_dual_valid_) {
            if (now_sec - last_dual_valid_time_ > dual_timeout_sec_) return false;
            if (!dual_valid_) return false;
        }
        return true;
    }

    double getImuAlignedHeading()
    {
        if (has_imu_bias_) return wrapAngle(imu_yaw_ + imu_bias_rad_);
        return imu_yaw_;
    }

    void updateImuBias(double ref_hdg)
    {
        if (!has_imu_) return;
        double target = wrapAngle(ref_hdg - imu_yaw_);
        if (!has_imu_bias_) {
            imu_bias_rad_ = target;
            has_imu_bias_ = true;
            return;
        }
        imu_bias_rad_ = interpAngle(imu_bias_rad_, target, imu_bias_alpha_);
    }

    void updateHeading(double target, double alpha, const std::string &src)
    {
        target = wrapAngle(target);
        if (!has_heading_) {
            heading_rad_ = target;
            has_heading_ = true;
        } else {
            heading_rad_ = interpAngle(heading_rad_, target, alpha);
        }
        if (src != heading_src_) {
            heading_src_ = src;
            RCLCPP_INFO(get_logger(), "Heading source -> %s (%.1f deg)",
                        src.c_str(), heading_rad_ * 180.0 / M_PI);
        }
    }

    static double estimateGpsQuality(const sensor_msgs::msg::NavSatFix &msg)
    {
        if (msg.position_covariance.empty()) return 0.5;
        double var = msg.position_covariance[0];
        if (var <= 0.0)  return 0.7;
        if (var < 0.05)  return 0.95;
        if (var < 0.2)   return 0.8;
        if (var < 1.0)   return 0.65;
        return 0.4;
    }

    // ── 멤버 ───────────────────────────────────────────
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr  hdg_pub_, slope_pub_, quality_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr origin_pub_;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr dhdg_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dval_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    int    utm_zone_;
    std::string odom_frame_, base_frame_, heading_src_;

    double spd_thresh_, spd_hyst_, gps_hdg_alpha_, imu_bias_alpha_;
    double dual_hdg_alpha_, dual_timeout_sec_, yaw_offset_rad_;
    double slope_alpha_, pitch_thresh_, max_pitch_, max_uphill_, max_downhill_;

    bool   origin_set_ = false;
    double origin_x_ = 0.0, origin_y_ = 0.0;
    double local_x_  = 0.0, local_y_  = 0.0;
    double vel_x_    = 0.0, vel_y_    = 0.0, gps_spd_ = 0.0;
    double roll_     = 0.0, pitch_    = 0.0, imu_yaw_ = 0.0, imu_yaw_rate_ = 0.0;
    double dual_hdg_rad_ = 0.0;
    double heading_rad_  = 0.0;
    double imu_bias_rad_ = 0.0;
    double slope_factor_ = 1.0;

    bool has_gps_      = false, has_imu_     = false;
    bool has_heading_  = false, has_gps_course_ = false;
    bool has_dual_     = false, dual_valid_   = false, seen_dual_valid_ = false;
    bool has_imu_bias_ = false;

    double last_gps_time_        = 0.0, last_vel_time_       = 0.0;
    double last_imu_time_        = 0.0, last_dual_time_       = 0.0;
    double last_dual_valid_time_ = 0.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StatusInfoNode>());
    rclcpp::shutdown();
    return 0;
}
