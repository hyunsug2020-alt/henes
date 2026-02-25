#include "jeju_mpc/mpc_path_maker.hpp"

#include <cmath>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace jeju_mpc
{

MpcPathMakerNode::MpcPathMakerNode()
: Node("mpc_path_maker_node")
{
  this->declare_parameter("output_dir", std::string("~/henes_ws_ros2/paths"));
  this->declare_parameter("path_name", std::string("mpc_recorded_path"));
  this->declare_parameter("timestamped_filename", true);
  this->declare_parameter("publish_rate_hz", 1.0);
  this->declare_parameter("record_rate_hz", 30.0);
  this->declare_parameter("min_record_distance_default", 0.2);
  this->declare_parameter("odom_topic", std::string("/odometry/filtered"));

  const auto output_dir_raw = this->get_parameter("output_dir").as_string();
  const auto path_name = this->get_parameter("path_name").as_string();
  const auto timestamped = this->get_parameter("timestamped_filename").as_bool();
  const double publish_rate_hz = this->get_parameter("publish_rate_hz").as_double();
  const double record_rate_hz = this->get_parameter("record_rate_hz").as_double();
  min_record_distance_ = this->get_parameter("min_record_distance_default").as_double();
  const auto odom_topic = this->get_parameter("odom_topic").as_string();

  std::string output_dir = output_dir_raw;
  if (!output_dir.empty() && output_dir[0] == '~') {
    const char * home = std::getenv("HOME");
    if (home != nullptr) {
      output_dir = std::string(home) + output_dir.substr(1);
    }
  }

  std::filesystem::create_directories(output_dir);

  std::string filename = path_name + "_filtered.txt";
  if (timestamped) {
    const auto t = std::time(nullptr);
    std::tm tm {};
    localtime_r(&t, &tm);
    std::ostringstream oss;
    oss << path_name << "_" << std::put_time(&tm, "%Y%m%d_%H%M%S") << "_filtered.txt";
    filename = oss.str();
  }

  const auto full_path = std::filesystem::path(output_dir) / filename;
  path_file_.open(full_path);
  if (!path_file_.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Cannot open path file: %s", full_path.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "Path output: %s", full_path.c_str());
  }

  global_path_.header.frame_id = "map";

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/global_path", 10);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, 20, std::bind(&MpcPathMakerNode::odomCb, this, std::placeholders::_1));

  origin_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
    "/utm_origin", 10, std::bind(&MpcPathMakerNode::originCb, this, std::placeholders::_1));

  quality_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/gps/quality", 10, std::bind(&MpcPathMakerNode::qualityCb, this, std::placeholders::_1));

  gps_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "/ublox_gps/fix_velocity", 10, std::bind(&MpcPathMakerNode::gpsVelCb, this, std::placeholders::_1));

  auto pub_period = std::chrono::duration<double>(1.0 / std::max(0.1, publish_rate_hz));
  auto rec_period = std::chrono::duration<double>(1.0 / std::max(1.0, record_rate_hz));

  pub_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(pub_period),
    std::bind(&MpcPathMakerNode::publishPath, this));

  rec_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(rec_period),
    std::bind(&MpcPathMakerNode::recordStep, this));

  RCLCPP_INFO(this->get_logger(), "MPC path maker started. Odom topic: %s", odom_topic.c_str());
}

MpcPathMakerNode::~MpcPathMakerNode()
{
  if (path_file_.is_open()) {
    path_file_.close();
  }
}

void MpcPathMakerNode::odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  latest_odom_ = *msg;
  have_odom_ = true;
}

void MpcPathMakerNode::originCb(const geometry_msgs::msg::Point::SharedPtr msg)
{
  origin_x_ = msg->x;
  origin_y_ = msg->y;
  origin_set_ = true;
}

void MpcPathMakerNode::qualityCb(const std_msgs::msg::Float64::SharedPtr msg)
{
  gps_quality_ = msg->data;
  if (gps_quality_ > 0.8) {
    min_record_distance_ = 0.1;
  } else if (gps_quality_ > 0.6) {
    min_record_distance_ = 0.15;
  } else if (gps_quality_ > 0.4) {
    min_record_distance_ = 0.25;
  } else {
    min_record_distance_ = 0.5;
  }
}

void MpcPathMakerNode::gpsVelCb(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
  vel_x_ = msg->twist.twist.linear.x;
  vel_y_ = msg->twist.twist.linear.y;
}

void MpcPathMakerNode::publishPath()
{
  if (global_path_.poses.empty()) {
    return;
  }
  global_path_.header.stamp = this->get_clock()->now();
  path_pub_->publish(global_path_);
}

void MpcPathMakerNode::recordStep()
{
  if (!have_odom_ || !origin_set_) {
    return;
  }

  double local_x = latest_odom_.pose.pose.position.x;
  double local_y = latest_odom_.pose.pose.position.y;
  const double local_z = latest_odom_.pose.pose.position.z;

  history_.push_back({local_x, local_y});
  if (static_cast<int>(history_.size()) > history_size_) {
    history_.erase(history_.begin());
  }

  if (history_.size() >= 3) {
    const double x = 0.2 * history_[history_.size() - 3].first +
      0.3 * history_[history_.size() - 2].first +
      0.5 * history_[history_.size() - 1].first;
    const double y = 0.2 * history_[history_.size() - 3].second +
      0.3 * history_[history_.size() - 2].second +
      0.5 * history_[history_.size() - 1].second;
    local_x = x;
    local_y = y;
  }

  const double distance = std::hypot(local_x - last_local_x_, local_y - last_local_y_);
  if (distance < min_record_distance_) {
    return;
  }

  if (gps_quality_ < 0.4) {
    quality_counter_ = 0;
    return;
  }

  quality_counter_ += 1;
  const int required_count = (gps_quality_ > 0.8) ? 1 : 2;
  if (quality_counter_ < required_count) {
    return;
  }

  const double speed = std::hypot(vel_x_, vel_y_);
  const double min_speed = (gps_quality_ > 0.8) ? 0.3 : 0.5;
  if (speed < min_speed) {
    return;
  }

  const double global_x = local_x + origin_x_;
  const double global_y = local_y + origin_y_;

  if (path_file_.is_open()) {
    path_file_ << global_x << "\t" << global_y << "\t" << local_z << "\n";
    path_file_.flush();
  }

  last_local_x_ = local_x;
  last_local_y_ = local_y;

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = this->get_clock()->now();
  pose.pose.position.x = local_x;
  pose.pose.position.y = local_y;
  pose.pose.position.z = local_z;
  pose.pose.orientation = latest_odom_.pose.pose.orientation;
  global_path_.poses.push_back(pose);
}

}  // namespace jeju_mpc

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<jeju_mpc::MpcPathMakerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

