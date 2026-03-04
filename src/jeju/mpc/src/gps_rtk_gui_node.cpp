#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64.hpp>
#include <ublox_ubx_msgs/msg/ubx_nav_pvt.hpp>

namespace jeju_mpc
{

class GpsRtkGuiNode : public rclcpp::Node
{
public:
  GpsRtkGuiNode()
  : Node("gps_rtk_gui_node")
  {
    this->declare_parameter("fix_topic", "/ublox_gps/fix");
    this->declare_parameter("quality_topic", "/gps/quality");
    this->declare_parameter("pvt_topic", "/ubx_nav_pvt");
    this->declare_parameter("window_name", "HENES GPS/RTK Monitor");
    this->declare_parameter("refresh_hz", 10.0);

    fix_topic_ = this->get_parameter("fix_topic").as_string();
    quality_topic_ = this->get_parameter("quality_topic").as_string();
    pvt_topic_ = this->get_parameter("pvt_topic").as_string();
    window_name_ = this->get_parameter("window_name").as_string();
    const double hz = this->get_parameter("refresh_hz").as_double();

    const auto sensor_qos = rclcpp::SensorDataQoS();
    fix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      fix_topic_, sensor_qos, std::bind(&GpsRtkGuiNode::fixCb, this, std::placeholders::_1));

    quality_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      quality_topic_, sensor_qos, std::bind(&GpsRtkGuiNode::qualityCb, this, std::placeholders::_1));

    pvt_sub_ = this->create_subscription<ublox_ubx_msgs::msg::UBXNavPVT>(
      pvt_topic_, sensor_qos, std::bind(&GpsRtkGuiNode::pvtCb, this, std::placeholders::_1));

    cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
    cv::resizeWindow(window_name_, 920, 520);

    auto period = std::chrono::duration<double>(1.0 / std::max(1.0, hz));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&GpsRtkGuiNode::drawFrame, this));

    RCLCPP_INFO(
      this->get_logger(), "GPS/RTK GUI started: fix=%s quality=%s pvt=%s",
      fix_topic_.c_str(), quality_topic_.c_str(), pvt_topic_.c_str());
  }

  ~GpsRtkGuiNode() override
  {
    cv::destroyAllWindows();
  }

private:
  static std::string navSatStatusText(int8_t status)
  {
    if (status >= 2) {
      return "RTK/GBAS FIX";
    }
    if (status == 1) {
      return "SBAS FIX";
    }
    if (status == 0) {
      return "GPS FIX";
    }
    return "NO FIX";
  }

  static cv::Scalar navSatStatusColor(int8_t status)
  {
    if (status >= 2) {
      return cv::Scalar(80, 220, 80);
    }
    if (status == 1) {
      return cv::Scalar(80, 180, 255);
    }
    if (status == 0) {
      return cv::Scalar(0, 220, 255);
    }
    return cv::Scalar(80, 80, 255);
  }

  static std::string carrierSolutionText(uint8_t status)
  {
    if (status == 2) {
      return "RTK FIXED";
    }
    if (status == 1) {
      return "RTK FLOAT";
    }
    return "NO CARRIER SOLUTION";
  }

  static cv::Scalar carrierSolutionColor(uint8_t status)
  {
    if (status == 2) {
      return cv::Scalar(80, 220, 80);
    }
    if (status == 1) {
      return cv::Scalar(80, 180, 255);
    }
    return cv::Scalar(80, 80, 255);
  }

  void fixCb(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    last_fix_ = *msg;
    has_fix_ = true;

    if (msg->position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
      const double vx = std::max(0.0, static_cast<double>(msg->position_covariance[0]));
      const double vy = std::max(0.0, static_cast<double>(msg->position_covariance[4]));
      // Horizontal 1-sigma RMS precision (meters).
      horiz_acc_sigma_m_ = std::sqrt((vx + vy) * 0.5);
    }
  }

  void qualityCb(const std_msgs::msg::Float64::SharedPtr msg)
  {
    gps_quality_ = msg->data;
  }

  void pvtCb(const ublox_ubx_msgs::msg::UBXNavPVT::SharedPtr msg)
  {
    carr_soln_status_ = msg->carr_soln.status;
    has_pvt_ = true;
  }

  void drawBar(cv::Mat & img, const std::string & label, double value, int y, const cv::Scalar & color)
  {
    cv::putText(img, label, {50, y}, cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(230, 230, 230), 1, cv::LINE_AA);
    cv::rectangle(img, {250, y - 20}, {800, y + 4}, cv::Scalar(70, 70, 70), cv::FILLED);
    const int width = static_cast<int>(std::clamp(value, 0.0, 1.0) * 550.0);
    cv::rectangle(img, {250, y - 20}, {250 + width, y + 4}, color, cv::FILLED);
  }

  void drawFrame()
  {
    cv::Mat img(520, 920, CV_8UC3, cv::Scalar(28, 28, 28));

    cv::putText(img, "HENES GPS / RTK Accuracy Monitor", {35, 45}, cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

    if (!has_fix_) {
      cv::putText(img, "Waiting for " + fix_topic_ + " ...", {40, 110}, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(120, 120, 255), 2, cv::LINE_AA);
      cv::imshow(window_name_, img);
      cv::waitKey(1);
      return;
    }

    const auto navsat_status = last_fix_.status.status;
    const std::string navsat_text = navSatStatusText(navsat_status);
    const cv::Scalar navsat_color = navSatStatusColor(navsat_status);
    const std::string rtk_text = carrierSolutionText(carr_soln_status_);
    const cv::Scalar rtk_color = carrierSolutionColor(carr_soln_status_);

    cv::rectangle(img, {40, 80}, {880, 145}, cv::Scalar(45, 45, 45), cv::FILLED);
    cv::putText(img, "RTK Status: " + rtk_text, {60, 115}, cv::FONT_HERSHEY_SIMPLEX, 0.95, rtk_color, 2, cv::LINE_AA);
    cv::putText(img, "NavSat Status: " + navsat_text, {60, 138}, cv::FONT_HERSHEY_SIMPLEX, 0.58, navsat_color, 1, cv::LINE_AA);

    char buf[256];
    std::snprintf(buf, sizeof(buf), "Latitude: %.8f", last_fix_.latitude);
    cv::putText(img, buf, {50, 190}, cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);
    std::snprintf(buf, sizeof(buf), "Longitude: %.8f", last_fix_.longitude);
    cv::putText(img, buf, {50, 220}, cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);

    std::snprintf(buf, sizeof(buf), "RTK precision: %.3f m", horiz_acc_sigma_m_);
    cv::putText(img, buf, {50, 265}, cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);

    std::snprintf(buf, sizeof(buf), "GPS quality score: %.3f", gps_quality_);
    cv::putText(img, buf, {50, 295}, cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);

    cv::rectangle(img, {620, 180}, {870, 305}, cv::Scalar(45, 45, 45), cv::FILLED);
    cv::putText(img, "Precision [m]", {645, 215}, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(185, 185, 185), 1, cv::LINE_AA);
    std::snprintf(buf, sizeof(buf), "%.3f", horiz_acc_sigma_m_);
    cv::putText(img, buf, {655, 275}, cv::FONT_HERSHEY_DUPLEX, 1.7, cv::Scalar(120, 240, 120), 2, cv::LINE_AA);

    const double acc_score = 1.0 - std::clamp(horiz_acc_sigma_m_ / 2.0, 0.0, 1.0);
    drawBar(img, "Accuracy (better when higher)", acc_score, 360, cv::Scalar(100, 220, 120));
    drawBar(img, "RTK quality score", gps_quality_, 420, cv::Scalar(80, 180, 255));

    const std::string pvt_state = has_pvt_ ? "OK" : "WAIT";
    cv::putText(
      img, "Topics: " + fix_topic_ + " | " + quality_topic_ + " | " + pvt_topic_ + " (" + pvt_state + ")",
      {40, 490}, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(160, 160, 160), 1, cv::LINE_AA);

    cv::imshow(window_name_, img);
    cv::waitKey(1);
  }

  std::string fix_topic_;
  std::string quality_topic_;
  std::string pvt_topic_;
  std::string window_name_;

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr quality_sub_;
  rclcpp::Subscription<ublox_ubx_msgs::msg::UBXNavPVT>::SharedPtr pvt_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool has_fix_ {false};
  bool has_pvt_ {false};
  sensor_msgs::msg::NavSatFix last_fix_;
  uint8_t carr_soln_status_ {0};
  double gps_quality_ {0.0};
  double horiz_acc_sigma_m_ {0.0};
};

}  // namespace jeju_mpc

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<jeju_mpc::GpsRtkGuiNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
