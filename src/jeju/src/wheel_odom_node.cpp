// -*- coding: utf-8 -*-
/*
 * wheel_odom_node.cpp
 * 엔코더 카운트 → 휠 오도메트리
 * IMU 헤딩 융합 옵션 포함
 */

#include <cmath>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>

class WheelOdomNode : public rclcpp::Node
{
public:
    WheelOdomNode() : Node("wheel_odom_node")
    {
        declare_parameter("left_encoder_topic",  "/encoder1");
        declare_parameter("right_encoder_topic", "/encoder2");
        declare_parameter("odom_topic",          "/wheel/odom");
        declare_parameter("frame_id",            "odom");
        declare_parameter("child_frame_id",      "base_footprint");
        declare_parameter("counts_per_rev",      4096.0);
        declare_parameter("wheel_radius_m",      0.13);
        declare_parameter("wheel_base_m",        0.70);
        declare_parameter("publish_rate_hz",     30.0);
        declare_parameter("use_imu_heading",     true);
        declare_parameter("imu_heading_topic",   "/ublox_gps/heading");
        declare_parameter("left_sign",           1.0);
        declare_parameter("right_sign",          1.0);

        std::string left_topic  = get_parameter("left_encoder_topic").as_string();
        std::string right_topic = get_parameter("right_encoder_topic").as_string();
        std::string odom_topic  = get_parameter("odom_topic").as_string();
        frame_id_       = get_parameter("frame_id").as_string();
        child_frame_id_ = get_parameter("child_frame_id").as_string();
        counts_per_rev_ = get_parameter("counts_per_rev").as_double();
        wheel_radius_   = get_parameter("wheel_radius_m").as_double();
        wheel_base_     = get_parameter("wheel_base_m").as_double();
        use_imu_        = get_parameter("use_imu_heading").as_bool();
        left_sign_      = get_parameter("left_sign").as_double();
        right_sign_     = get_parameter("right_sign").as_double();

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic, 20);

        left_sub_  = create_subscription<std_msgs::msg::Int32>(
            left_topic, 20,
            std::bind(&WheelOdomNode::leftCb, this, std::placeholders::_1));
        right_sub_ = create_subscription<std_msgs::msg::Int32>(
            right_topic, 20,
            std::bind(&WheelOdomNode::rightCb, this, std::placeholders::_1));

        if (use_imu_) {
            std::string imu_hdg_topic = get_parameter("imu_heading_topic").as_string();
            imu_sub_ = create_subscription<std_msgs::msg::Float64>(
                imu_hdg_topic, 20,
                std::bind(&WheelOdomNode::imuHdgCb, this, std::placeholders::_1));
        }

        double hz = get_parameter("publish_rate_hz").as_double();
        timer_ = create_timer(
            this,
            get_clock(),
            rclcpp::Duration::from_seconds(1.0 / std::max(hz, 1.0)),
            std::bind(&WheelOdomNode::step, this));

        last_time_ = now();

        RCLCPP_INFO(get_logger(),
            "wheel_odom: %s %s -> %s (cpr=%.0f r=%.3f base=%.3f)",
            left_topic.c_str(), right_topic.c_str(), odom_topic.c_str(),
            counts_per_rev_, wheel_radius_, wheel_base_);
    }

private:
    void leftCb(const std_msgs::msg::Int32::SharedPtr msg)
    {
        left_count_ = msg->data;
        has_left_   = true;
    }

    void rightCb(const std_msgs::msg::Int32::SharedPtr msg)
    {
        right_count_ = msg->data;
        has_right_   = true;
    }

    void imuHdgCb(const std_msgs::msg::Float64::SharedPtr msg)
    {
        imu_heading_rad_ = msg->data * M_PI / 180.0;
        has_imu_         = true;
    }

    void step()
    {
        if (!has_left_ || !has_right_) return;

        if (!prev_init_) {
            prev_left_  = left_count_;
            prev_right_ = right_count_;
            last_time_  = now();
            prev_init_  = true;
            return;
        }

        rclcpp::Time cur = now();
        double dt = (cur - last_time_).seconds();
        if (dt <= 1e-6) return;

        double dl = (left_count_  - prev_left_)  * left_sign_;
        double dr = (right_count_ - prev_right_) * right_sign_;
        prev_left_  = left_count_;
        prev_right_ = right_count_;
        last_time_  = cur;

        double mpc = (2.0 * M_PI * wheel_radius_) / std::max(counts_per_rev_, 1.0);
        double dl_m = dl * mpc;
        double dr_m = dr * mpc;
        double ds     = 0.5 * (dl_m + dr_m);
        double dtheta = (dr_m - dl_m) / std::max(wheel_base_, 1e-6);

        if (use_imu_ && has_imu_)
            yaw_ = imu_heading_rad_;
        else
            yaw_ += dtheta;

        x_ += ds * std::cos(yaw_);
        y_ += ds * std::sin(yaw_);

        // 쿼터니언 (yaw only)
        double hy = 0.5 * yaw_;
        double qw = std::cos(hy), qz = std::sin(hy);

        nav_msgs::msg::Odometry odom;
        odom.header.stamp         = cur;
        odom.header.frame_id      = frame_id_;
        odom.child_frame_id       = child_frame_id_;
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.orientation.w = qw;
        odom.pose.pose.orientation.z = qz;
        odom.twist.twist.linear.x  = ds / dt;
        odom.twist.twist.angular.z = dtheta / dt;

        odom_pub_->publish(odom);
    }

    // ── 멤버 ──────────────────────────────────────
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr left_sub_, right_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr imu_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string frame_id_, child_frame_id_;
    double counts_per_rev_, wheel_radius_, wheel_base_;
    double left_sign_, right_sign_;
    bool   use_imu_ = true;

    int32_t left_count_ = 0, right_count_ = 0;
    int32_t prev_left_  = 0, prev_right_  = 0;
    bool    has_left_   = false, has_right_ = false, prev_init_ = false;

    double imu_heading_rad_ = 0.0;
    bool   has_imu_         = false;

    double x_ = 0.0, y_ = 0.0, yaw_ = 0.0;
    rclcpp::Time last_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelOdomNode>());
    rclcpp::shutdown();
    return 0;
}
