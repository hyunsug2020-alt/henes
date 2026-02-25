#include <cmath>
#include <cstdio>
#include <string>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64.hpp>

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
    this->declare_parameter("window_name", "HENES GPS/RTK Monitor");
    this->declare_parameter("refresh_hz", 10.0);

    fix_topic_ = this->get_parameter("fix_topic").as_string();
    quality_topic_ = this->get_parameter("quality_topic").as_string();
    window_name_ = this->get_parameter("window_name").as_string();
    const double hz = this->get_parameter("refresh_hz").as_double();

    fix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      fix_topic_, 20, std::bind(&GpsRtkGuiNode::fixCb, this, std::placeholders::_1));

    quality_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      quality_topic_, 20, std::bind(&GpsRtkGuiNode::qualityCb, this, std::placeholders::_1));

    cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
    cv::resizeWindow(window_name_, 920, 520);

    auto period = std::chrono::duration<double>(1.0 / std::max(1.0, hz));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&GpsRtkGuiNode::drawFrame, this));

    RCLCPP_INFO(this->get_logger(), "GPS/RTK GUI started: fix=%s quality=%s", fix_topic_.c_str(), quality_topic_.c_str());
  }

  ~GpsRtkGuiNode() override
  {
    cv::destroyAllWindows();
  }

private:
  static std::string rtkStatusText(int8_t status)
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

  static cv::Scalar statusColor(int8_t status)
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

  void fixCb(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    last_fix_ = *msg;
    has_fix_ = true;

    if (msg->position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
      const double vx = std::max(0.0, static_cast<double>(msg->position_covariance[0]));
      const double vy = std::max(0.0, static_cast<double>(msg->position_covariance[4]));
      horiz_acc_sigma_m_ = std::sqrt(vx + vy);
    }
  }

  void qualityCb(const std_msgs::msg::Float64::SharedPtr msg)
  {
    gps_quality_ = msg->data;
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
      cv::putText(img, "Waiting for /ublox_gps/fix ...", {40, 110}, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(120, 120, 255), 2, cv::LINE_AA);
      cv::imshow(window_name_, img);
      cv::waitKey(1);
      return;
    }

    const auto status = last_fix_.status.status;
    const std::string status_text = rtkStatusText(status);
    const cv::Scalar st_color = statusColor(status);

    cv::rectangle(img, {40, 80}, {880, 145}, cv::Scalar(45, 45, 45), cv::FILLED);
    cv::putText(img, "RTK Status: " + status_text, {60, 122}, cv::FONT_HERSHEY_SIMPLEX, 1.0, st_color, 2, cv::LINE_AA);

    char buf[256];
    std::snprintf(buf, sizeof(buf), "Latitude: %.8f", last_fix_.latitude);
    cv::putText(img, buf, {50, 190}, cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);
    std::snprintf(buf, sizeof(buf), "Longitude: %.8f", last_fix_.longitude);
    cv::putText(img, buf, {50, 220}, cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);

    std::snprintf(buf, sizeof(buf), "Horizontal sigma (sqrt(cov_x + cov_y)): %.3f m", horiz_acc_sigma_m_);
    cv::putText(img, buf, {50, 265}, cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);

    std::snprintf(buf, sizeof(buf), "GPS quality score: %.3f", gps_quality_);
    cv::putText(img, buf, {50, 295}, cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);

    const double acc_score = 1.0 - std::clamp(horiz_acc_sigma_m_ / 2.0, 0.0, 1.0);
    drawBar(img, "Accuracy (better when higher)", acc_score, 360, cv::Scalar(100, 220, 120));
    drawBar(img, "RTK quality score", gps_quality_, 420, cv::Scalar(80, 180, 255));

    cv::putText(img, "Topics: " + fix_topic_ + " | " + quality_topic_, {40, 490}, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(160, 160, 160), 1, cv::LINE_AA);

    cv::imshow(window_name_, img);
    cv::waitKey(1);
  }

  std::string fix_topic_;
  std::string quality_topic_;
  std::string window_name_;

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr quality_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool has_fix_ {false};
  sensor_msgs::msg::NavSatFix last_fix_;
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

