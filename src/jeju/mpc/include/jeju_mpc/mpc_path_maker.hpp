#pragma once

#include <fstream>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace jeju_mpc
{

class MpcPathMakerNode : public rclcpp::Node
{
public:
  MpcPathMakerNode();
  ~MpcPathMakerNode() override;

private:
  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg);
  void originCb(const geometry_msgs::msg::Point::SharedPtr msg);
  void qualityCb(const std_msgs::msg::Float64::SharedPtr msg);
  void gpsVelCb(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);
  void publishPath();
  void recordStep();

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr origin_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr quality_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr gps_vel_sub_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  rclcpp::TimerBase::SharedPtr rec_timer_;

  nav_msgs::msg::Path global_path_;
  nav_msgs::msg::Odometry latest_odom_;
  bool have_odom_ {false};
  bool origin_set_ {false};

  double origin_x_ {0.0};
  double origin_y_ {0.0};
  double last_local_x_ {0.0};
  double last_local_y_ {0.0};
  double gps_quality_ {1.0};
  double min_record_distance_ {0.2};
  double vel_x_ {0.0};
  double vel_y_ {0.0};

  int quality_counter_ {0};
  int history_size_ {5};
  std::vector<std::pair<double, double>> history_;

  std::ofstream path_file_;
};

}  // namespace jeju_mpc

