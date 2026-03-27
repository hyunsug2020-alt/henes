// -*- coding: utf-8 -*-
/*
 * gnss_velocity_bridge_node.cpp
 * UBX NAV-VELNED → geometry_msgs/TwistWithCovarianceStamped 변환
 */

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <ublox_ubx_msgs/msg/ubx_nav_vel_ned.hpp>

class GnssVelocityBridgeNode : public rclcpp::Node
{
public:
    GnssVelocityBridgeNode() : Node("gnss_velocity_bridge_node")
    {
        declare_parameter("input_topic",  "/gnss_left/ubx_nav_vel_ned");
        declare_parameter("output_topic", "/ublox_gps/fix_velocity");
        declare_parameter("frame_id",     "odom");

        std::string in_topic  = get_parameter("input_topic").as_string();
        std::string out_topic = get_parameter("output_topic").as_string();
        frame_id_ = get_parameter("frame_id").as_string();

        pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(out_topic, 20);
        sub_ = create_subscription<ublox_ubx_msgs::msg::UBXNavVelNED>(
            in_topic, 20,
            std::bind(&GnssVelocityBridgeNode::velCb, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "gnss_velocity_bridge: %s -> %s",
                    in_topic.c_str(), out_topic.c_str());
    }

private:
    void velCb(const ublox_ubx_msgs::msg::UBXNavVelNED::SharedPtr msg)
    {
        geometry_msgs::msg::TwistWithCovarianceStamped out;
        out.header          = msg->header;
        out.header.frame_id = frame_id_;

        // UBX NAV-VELNED: North/East/Down [cm/s] → ENU [m/s]
        // x=East, y=North (atan2(vy,vx) == 진행방향 ENU 기준)
        out.twist.twist.linear.x =  static_cast<double>(msg->vel_e) * 0.01;
        out.twist.twist.linear.y =  static_cast<double>(msg->vel_n) * 0.01;
        out.twist.twist.linear.z = -static_cast<double>(msg->vel_d) * 0.01;

        double speed_var = std::pow(static_cast<double>(msg->s_acc) * 0.01, 2.0);
        out.twist.covariance[0]  = speed_var;
        out.twist.covariance[7]  = speed_var;
        out.twist.covariance[14] = speed_var;

        double heading_sigma     = static_cast<double>(msg->c_acc) * 1e-5 * M_PI / 180.0;
        out.twist.covariance[35] = heading_sigma * heading_sigma;

        pub_->publish(out);
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_;
    rclcpp::Subscription<ublox_ubx_msgs::msg::UBXNavVelNED>::SharedPtr sub_;
    std::string frame_id_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GnssVelocityBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
