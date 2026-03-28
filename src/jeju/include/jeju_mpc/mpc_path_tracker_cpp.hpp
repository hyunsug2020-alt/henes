#pragma once

#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "jeju_mpc/mpc_controller_cpp.hpp"

namespace jeju_mpc
{

class MPCPathTrackerCpp : public rclcpp::Node
{
public:
    MPCPathTrackerCpp();

private:
    void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg);
    void pathCb(const nav_msgs::msg::Path::SharedPtr msg);
    void teleopModeCb(const std_msgs::msg::Bool::SharedPtr msg);
    void obstacleStopCb(const std_msgs::msg::Bool::SharedPtr msg);
    void slopeFactorCb(const std_msgs::msg::Float64::SharedPtr msg);
    void steeringAngleCb(const std_msgs::msg::Float64::SharedPtr msg);
    void controlLoop();

    static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q);
    static std::vector<RefPoint> pathFromMsg(const nav_msgs::msg::Path & msg);
    int  findNearestIndex(const VehicleState & state, int prev_idx) const;
    bool isGoalReached(const VehicleState & state) const;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr                cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr         debug_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr    odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr        path_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr        teleop_mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr        obstacle_stop_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr     slope_factor_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr     steering_angle_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr gps_vel_sub_;
    rclcpp::TimerBase::SharedPtr                                timer_;

    MPCControllerCpp    controller_;
    MPCControllerParams controller_params_;
    VehicleState        state_;
    std::vector<RefPoint> path_;

    bool   has_odom_             {false};
    bool   has_path_             {false};
    bool   teleop_mode_          {false};
    bool   obstacle_stop_        {false};
    bool   goal_reached_latched_ {false};

    double control_rate_hz_      {20.0};
    double forward_speed_kmh_    { 8.0};
    double slope_factor_         { 1.0};
    double goal_tolerance_m_     { 1.0};
    double wheelbase_m_          { 1.04};
    double current_steer_deg_    { 0.0};
    double vehicle_width_m_      { 0.70};
    double vehicle_length_m_     { 1.30};

    // 전진/후진 방향 프로파일
    std::vector<int> direction_profile_;
    int  current_direction_      {1};
    int  nearest_idx_prev_       {0};

    double speed_integral_       {0.0};
    double speed_kp_             {30.0};
    double speed_ki_             { 2.0};
    double speed_integral_max_   {15.0};
    int    nearest_search_window_{80};
    double cusp_cos_threshold_   {-0.98};

    // GPS 속도 기반 헤딩 (ESKF 대체)
    double gps_yaw_              {0.0};
    double gps_speed_mps_        {0.0};
    bool   has_gps_yaw_          {false};

    MPCControlOutput prev_output_;
    bool             has_prev_output_ {false};
};

}  // namespace jeju_mpc
