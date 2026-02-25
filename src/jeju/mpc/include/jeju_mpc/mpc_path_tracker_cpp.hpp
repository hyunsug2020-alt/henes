#pragma once

#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

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
  void controlLoop();

  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q);
  static std::vector<RefPoint> pathFromMsg(const nav_msgs::msg::Path & msg);
  int findNearestIndex(const VehicleState & state) const;
  bool isGoalReached(const VehicleState & state) const;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr teleop_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr slope_factor_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  MPCControllerCpp controller_;
  MPCControllerParams controller_params_;

  VehicleState state_;
  std::vector<RefPoint> path_;

  bool has_odom_ {false};
  bool has_path_ {false};
  bool teleop_mode_ {true};
  bool obstacle_stop_ {false};
  bool goal_reached_latched_ {false};

  double control_rate_hz_ {20.0};
  double forward_speed_kmh_ {8.0};
  double slope_factor_ {1.0};
  double goal_tolerance_m_ {1.0};
};

}  // namespace jeju_mpc

