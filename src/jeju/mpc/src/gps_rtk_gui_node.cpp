#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <std_msgs/msg/float64.hpp>
#include <ublox_ubx_msgs/msg/ubx_nav_pvt.hpp>
#include <ublox_ubx_msgs/msg/ubx_nav_rel_pos_ned.hpp>

namespace jeju_mpc
{

enum class RtkState
{
  OFF = 0,
  FLOAT = 1,
  FIXED = 2,
};

class GpsRtkGuiNode : public rclcpp::Node
{
public:
  GpsRtkGuiNode()
  : Node("gps_rtk_gui_node")
  {
    this->declare_parameter("fix_topic", "/ublox_gps/fix");
    this->declare_parameter("quality_topic", "/gps/quality");
    this->declare_parameter("pvt_topic", "/ubx_nav_pvt");
    this->declare_parameter("relpos_topic", "/ubx_nav_rel_pos_ned");
    this->declare_parameter("window_name", "HENES RTK Lock Monitor");
    this->declare_parameter("refresh_hz", 10.0);
    this->declare_parameter("topic_timeout_sec", 1.5);
    this->declare_parameter("log_to_console", true);

    fix_topic_ = this->get_parameter("fix_topic").as_string();
    quality_topic_ = this->get_parameter("quality_topic").as_string();
    pvt_topic_ = this->get_parameter("pvt_topic").as_string();
    relpos_topic_ = this->get_parameter("relpos_topic").as_string();
    window_name_ = this->get_parameter("window_name").as_string();
    const double hz = this->get_parameter("refresh_hz").as_double();
    topic_timeout_sec_ = this->get_parameter("topic_timeout_sec").as_double();
    log_to_console_ = this->get_parameter("log_to_console").as_bool();

    const auto sensor_qos = rclcpp::SensorDataQoS();
    fix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      fix_topic_, sensor_qos, std::bind(&GpsRtkGuiNode::fixCb, this, std::placeholders::_1));

    quality_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      quality_topic_, sensor_qos, std::bind(&GpsRtkGuiNode::qualityCb, this, std::placeholders::_1));

    auto ubx_qos = rclcpp::QoS(
      rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    ubx_qos.keep_last(10);
    ubx_qos.reliable();
    ubx_qos.transient_local();

    pvt_sub_ = this->create_subscription<ublox_ubx_msgs::msg::UBXNavPVT>(
      pvt_topic_, ubx_qos, std::bind(&GpsRtkGuiNode::pvtCb, this, std::placeholders::_1));

    relpos_sub_ = this->create_subscription<ublox_ubx_msgs::msg::UBXNavRelPosNED>(
      relpos_topic_, ubx_qos, std::bind(&GpsRtkGuiNode::relPosCb, this, std::placeholders::_1));

    cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
    cv::resizeWindow(window_name_, 1080, 640);

    auto period = std::chrono::duration<double>(1.0 / std::max(1.0, hz));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&GpsRtkGuiNode::drawFrame, this));

    RCLCPP_INFO(
      this->get_logger(), "RTK GUI started: fix=%s quality=%s pvt=%s relpos=%s",
      fix_topic_.c_str(), quality_topic_.c_str(), pvt_topic_.c_str(), relpos_topic_.c_str());
  }

  ~GpsRtkGuiNode() override
  {
    cv::destroyAllWindows();
  }

private:
  static double clamp01(double value)
  {
    return std::clamp(value, 0.0, 1.0);
  }

  static std::string navSatStatusText(int8_t status)
  {
    if (status >= sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX) {
      return "GBAS FIX";
    }
    if (status == sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX) {
      return "SBAS FIX";
    }
    if (status == sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
      return "GPS FIX";
    }
    return "NO FIX";
  }

  static cv::Scalar navSatStatusColor(int8_t status)
  {
    if (status >= sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX) {
      return cv::Scalar(70, 220, 120);
    }
    if (status == sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX) {
      return cv::Scalar(50, 190, 255);
    }
    if (status == sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
      return cv::Scalar(0, 220, 255);
    }
    return cv::Scalar(70, 70, 255);
  }

  static std::string stateText(RtkState state)
  {
    switch (state) {
      case RtkState::FIXED:
        return "FIXED";
      case RtkState::FLOAT:
        return "FLOAT";
      case RtkState::OFF:
      default:
        return "OFF";
    }
  }

  static cv::Scalar stateColor(RtkState state)
  {
    switch (state) {
      case RtkState::FIXED:
        return cv::Scalar(70, 210, 110);
      case RtkState::FLOAT:
        return cv::Scalar(40, 190, 255);
      case RtkState::OFF:
      default:
        return cv::Scalar(60, 70, 255);
    }
  }

  static std::string stateSourceText(bool pvt_fresh)
  {
    return pvt_fresh ? "UBX" : "INFERRED";
  }

  static std::string carrierModeText(uint8_t status, bool diff_soln, bool pvt_fresh)
  {
    if (pvt_fresh) {
      if (status == 2) {
        return "FIXED";
      }
      if (status == 1) {
        return "FLOAT";
      }
      if (diff_soln) {
        return "DGNSS";
      }
      return "NONE";
    }
    if (diff_soln) {
      return "INFERRED+DGNSS";
    }
    return "INFERRED";
  }

  static cv::Scalar percentColor(double percent)
  {
    if (percent >= 85.0) {
      return cv::Scalar(70, 210, 110);
    }
    if (percent >= 60.0) {
      return cv::Scalar(40, 190, 255);
    }
    return cv::Scalar(60, 70, 255);
  }

  bool isFresh(const rclcpp::Time & stamp) const
  {
    if (stamp.nanoseconds() == 0) {
      return false;
    }
    return (this->now() - stamp).seconds() <= topic_timeout_sec_;
  }

  double navStatusScore() const
  {
    if (!has_fix_ || !isFresh(last_fix_time_)) {
      return 0.0;
    }
    if (last_fix_.status.status >= sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX) {
      return 1.0;
    }
    if (last_fix_.status.status == sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX) {
      return 0.75;
    }
    if (last_fix_.status.status == sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
      return 0.45;
    }
    return 0.0;
  }

  double fallbackQualityScore() const
  {
    const double acc_m = horizontalAccuracyMeters();
    if (acc_m > 0.0) {
      if (acc_m <= 0.03) {
        return 0.99;
      }
      if (acc_m <= 0.08) {
        return 0.93;
      }
      if (acc_m <= 0.15) {
        return 0.85;
      }
      if (acc_m <= 0.30) {
        return 0.72;
      }
      if (acc_m <= 0.60) {
        return 0.55;
      }
      if (acc_m <= 1.50) {
        return 0.35;
      }
      return 0.15;
    }
    return navStatusScore();
  }

  double qualityScore(bool * using_external_topic = nullptr) const
  {
    const bool external_ok = has_quality_ && isFresh(last_quality_time_);
    if (using_external_topic != nullptr) {
      *using_external_topic = external_ok;
    }
    if (external_ok) {
      return clamp01(gps_quality_);
    }
    return fallbackQualityScore();
  }

  double horizontalAccuracyMeters() const
  {
    if (has_relpos_ && isFresh(last_relpos_time_) && relpos_acc_length_m_ > 0.0) {
      return relpos_acc_length_m_;
    }
    if (has_pvt_ && isFresh(last_pvt_time_) && pvt_h_acc_m_ > 0.0) {
      return pvt_h_acc_m_;
    }
    return horiz_acc_sigma_m_;
  }

  double accuracyScore() const
  {
    const double acc_m = horizontalAccuracyMeters();
    if (acc_m <= 0.0) {
      return 0.0;
    }
    if (acc_m <= 0.03) {
      return 1.0;
    }
    if (acc_m <= 0.08) {
      return 0.95;
    }
    if (acc_m <= 0.15) {
      return 0.88;
    }
    if (acc_m <= 0.30) {
      return 0.76;
    }
    if (acc_m <= 0.60) {
      return 0.54;
    }
    if (acc_m <= 1.00) {
      return 0.35;
    }
    return 0.12;
  }

  double pvtCarrierScore(bool pvt_fresh) const
  {
    if (!pvt_fresh) {
      return -1.0;
    }
    if (carr_soln_status_ == 2) {
      return 1.0;
    }
    if (carr_soln_status_ == 1) {
      return 0.78;
    }
    if (pvt_diff_soln_) {
      return 0.42;
    }
    if (pvt_gnss_fix_ok_) {
      return 0.18;
    }
    return 0.0;
  }

  double matchScore()
  {
    const bool fix_fresh = has_fix_ && isFresh(last_fix_time_);
    const bool pvt_fresh = has_pvt_ && isFresh(last_pvt_time_);
    bool using_external_quality = false;
    const double quality = qualityScore(&using_external_quality);
    const double acc = accuracyScore();
    const double nav = navStatusScore();

    double weighted_sum = 0.0;
    double total_weight = 0.0;

    const double pvt_score = pvtCarrierScore(pvt_fresh);
    if (pvt_score >= 0.0) {
      weighted_sum += pvt_score * 0.50;
      total_weight += 0.50;
    }

    if (using_external_quality || fix_fresh) {
      weighted_sum += quality * 0.30;
      total_weight += 0.30;
    }

    if (horizontalAccuracyMeters() > 0.0) {
      weighted_sum += acc * 0.15;
      total_weight += 0.15;
    }

    if (fix_fresh) {
      weighted_sum += nav * 0.05;
      total_weight += 0.05;
    }

    double score = (total_weight > 0.0) ? (weighted_sum / total_weight) : 0.0;
    if (pvt_fresh && carr_soln_status_ == 2) {
      score = std::max(score, 0.96);
    } else if (pvt_fresh && carr_soln_status_ == 1) {
      score = std::max(score, 0.74);
    }
    return clamp01(score);
  }

  RtkState classifyState(double match_score) const
  {
    const bool relpos_fresh = has_relpos_ && isFresh(last_relpos_time_);
    const bool pvt_fresh = has_pvt_ && isFresh(last_pvt_time_);
    if (relpos_fresh) {
      if (relpos_carr_soln_status_ == 2 && relpos_valid_) {
        return RtkState::FIXED;
      }
      if (relpos_carr_soln_status_ == 1 && relpos_valid_) {
        return RtkState::FLOAT;
      }
    }
    if (pvt_fresh) {
      if (carr_soln_status_ == 2) {
        return RtkState::FIXED;
      }
      if (carr_soln_status_ == 1) {
        return RtkState::FLOAT;
      }
      return RtkState::OFF;
    }
    if (match_score >= 0.94 && horizontalAccuracyMeters() <= 0.10) {
      return RtkState::FLOAT;
    }
    return RtkState::OFF;
  }

  void drawProgressBar(
    cv::Mat & img, const cv::Rect & rect, double value, const cv::Scalar & fill_color) const
  {
    cv::rectangle(img, rect, cv::Scalar(52, 52, 52), cv::FILLED);
    const int width = static_cast<int>(rect.width * clamp01(value));
    cv::rectangle(img, {rect.x, rect.y}, {rect.x + width, rect.y + rect.height}, fill_color, cv::FILLED);
    cv::rectangle(img, rect, cv::Scalar(90, 90, 90), 1);
  }

  void fixCb(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    last_fix_ = *msg;
    last_fix_time_ = this->now();
    has_fix_ = true;

    if (msg->position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
      const double vx = std::max(0.0, static_cast<double>(msg->position_covariance[0]));
      const double vy = std::max(0.0, static_cast<double>(msg->position_covariance[4]));
      horiz_acc_sigma_m_ = std::sqrt((vx + vy) * 0.5);
    }
  }

  void qualityCb(const std_msgs::msg::Float64::SharedPtr msg)
  {
    gps_quality_ = msg->data;
    last_quality_time_ = this->now();
    has_quality_ = true;
  }

  void pvtCb(const ublox_ubx_msgs::msg::UBXNavPVT::SharedPtr msg)
  {
    carr_soln_status_ = msg->carr_soln.status;
    pvt_gnss_fix_ok_ = msg->gnss_fix_ok;
    pvt_diff_soln_ = msg->diff_soln;
    pvt_num_sv_ = msg->num_sv;
    pvt_h_acc_m_ = static_cast<double>(msg->h_acc) * 1e-3;
    last_pvt_time_ = this->now();
    has_pvt_ = true;
  }

  void relPosCb(const ublox_ubx_msgs::msg::UBXNavRelPosNED::SharedPtr msg)
  {
    relpos_carr_soln_status_ = msg->carr_soln.status;
    relpos_diff_soln_ = msg->diff_soln;
    relpos_valid_ = msg->rel_pos_valid;
    relpos_acc_length_m_ = static_cast<double>(msg->acc_length) * 1e-4;
    last_relpos_time_ = this->now();
    has_relpos_ = true;
  }

  void drawFrame()
  {
    cv::Mat img(640, 1080, CV_8UC3, cv::Scalar(22, 22, 24));
    cv::rectangle(img, {0, 0}, {1080, 86}, cv::Scalar(34, 36, 40), cv::FILLED);
    cv::putText(
      img, "HENES RTK LOCK MONITOR", {34, 54}, cv::FONT_HERSHEY_DUPLEX, 1.15,
      cv::Scalar(250, 250, 250), 2, cv::LINE_AA);

    const bool fix_fresh = has_fix_ && isFresh(last_fix_time_);
    const bool pvt_fresh = has_pvt_ && isFresh(last_pvt_time_);
    const bool relpos_fresh = has_relpos_ && isFresh(last_relpos_time_);
    bool using_external_quality = false;
    const double quality_score = qualityScore(&using_external_quality);
    const double match_score = matchScore();
    const double match_percent = std::round(match_score * 100.0);
    const RtkState rtk_state = classifyState(match_score);
    const double acc_m = horizontalAccuracyMeters();

    if (!fix_fresh) {
      if (log_to_console_) {
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "RTK=OFF MATCH=0%% MODE=NO_FIX");
      }
      cv::putText(
        img, "WAITING FOR GPS FIX", {80, 180}, cv::FONT_HERSHEY_DUPLEX, 1.4,
        cv::Scalar(80, 90, 255), 3, cv::LINE_AA);
      cv::putText(
        img, ("Topic: " + fix_topic_).c_str(), {84, 235}, cv::FONT_HERSHEY_SIMPLEX, 0.85,
        cv::Scalar(200, 200, 210), 2, cv::LINE_AA);
      cv::putText(
        img, "The window updates after the first NavSatFix message arrives.", {84, 280},
        cv::FONT_HERSHEY_SIMPLEX, 0.72, cv::Scalar(170, 170, 180), 1, cv::LINE_AA);
      cv::imshow(window_name_, img);
      cv::waitKey(1);
      return;
    }

    const uint8_t active_carr_soln = relpos_fresh ? relpos_carr_soln_status_ : carr_soln_status_;
    const bool active_diff_soln = relpos_fresh ? relpos_diff_soln_ : pvt_diff_soln_;
    const std::string mode_text = carrierModeText(active_carr_soln, active_diff_soln, relpos_fresh || pvt_fresh);
    const std::string state_text = stateText(rtk_state);
    const cv::Scalar state_color = stateColor(rtk_state);
    const cv::Scalar percent_color = percentColor(match_percent);
    const cv::Scalar nav_color = navSatStatusColor(last_fix_.status.status);

    if (log_to_console_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "RTK=%s MATCH=%.0f%% MODE=%s SRC=%s PREC=%.3fm",
        state_text.c_str(), match_percent, mode_text.c_str(),
        stateSourceText(relpos_fresh || pvt_fresh).c_str(), acc_m);
    }

    cv::Rect state_card(36, 108, 480, 214);
    cv::Rect percent_card(554, 108, 490, 214);
    cv::rectangle(img, state_card, cv::Scalar(42, 44, 48), cv::FILLED);
    cv::rectangle(img, percent_card, cv::Scalar(42, 44, 48), cv::FILLED);

    cv::putText(img, "RTK STATE", {64, 154}, cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(190, 190, 198), 2, cv::LINE_AA);
    cv::putText(
      img, state_text.c_str(), {64, 255}, cv::FONT_HERSHEY_DUPLEX, 2.35, state_color, 4,
      cv::LINE_AA);
    cv::putText(
      img, ("MODE: " + mode_text).c_str(), {68, 286}, cv::FONT_HERSHEY_SIMPLEX, 0.82,
      cv::Scalar(220, 220, 220), 2, cv::LINE_AA);
    cv::putText(
      img, ("SRC: " + stateSourceText(relpos_fresh || pvt_fresh)).c_str(), {68, 320}, cv::FONT_HERSHEY_SIMPLEX, 0.72,
      cv::Scalar(220, 220, 220), 2, cv::LINE_AA);

    cv::putText(
      img, "MATCH", {586, 154}, cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(190, 190, 198), 2,
      cv::LINE_AA);
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%.0f%%", match_percent);
    cv::putText(img, buf, {586, 255}, cv::FONT_HERSHEY_DUPLEX, 2.30, percent_color, 4, cv::LINE_AA);
    drawProgressBar(img, {586, 275, 390, 26}, match_score, percent_color);

    cv::Rect detail_card(36, 350, 1008, 220);
    cv::rectangle(img, detail_card, cv::Scalar(35, 36, 40), cv::FILLED);

    const char * quality_source = using_external_quality ? "TOPIC" : "INTERNAL";
    const char * pvt_state = pvt_fresh ? "OK" : "WAIT";
    const char * relpos_state = relpos_fresh ? "OK" : "WAIT";
    const char * correction_state = active_diff_soln ? "ON" : "OFF";
    const uint8_t num_sv = pvt_fresh ? pvt_num_sv_ : 0U;

    std::snprintf(buf, sizeof(buf), "NavSat: %s", navSatStatusText(last_fix_.status.status).c_str());
    cv::putText(img, buf, {64, 398}, cv::FONT_HERSHEY_SIMPLEX, 0.78, nav_color, 2, cv::LINE_AA);

    std::snprintf(buf, sizeof(buf), "Precision: %s", acc_m > 0.0 ? "" : "N/A");
    cv::putText(img, buf, {64, 440}, cv::FONT_HERSHEY_SIMPLEX, 0.74, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);
    if (acc_m > 0.0) {
      std::snprintf(buf, sizeof(buf), "%.3f m", acc_m);
      cv::putText(img, buf, {196, 440}, cv::FONT_HERSHEY_SIMPLEX, 0.88, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
    }

    std::snprintf(buf, sizeof(buf), "Quality Source: %s", quality_source);
    cv::putText(img, buf, {64, 482}, cv::FONT_HERSHEY_SIMPLEX, 0.74, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);

    std::snprintf(buf, sizeof(buf), "Quality Score: %.0f%%", std::round(quality_score * 100.0));
    cv::putText(img, buf, {64, 524}, cv::FONT_HERSHEY_SIMPLEX, 0.74, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);

    std::snprintf(buf, sizeof(buf), "Corrections: %s", correction_state);
    cv::putText(img, buf, {560, 398}, cv::FONT_HERSHEY_SIMPLEX, 0.78, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);

    std::snprintf(buf, sizeof(buf), "PVT Topic: %s", pvt_state);
    cv::putText(img, buf, {560, 440}, cv::FONT_HERSHEY_SIMPLEX, 0.74, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);

    std::snprintf(buf, sizeof(buf), "RELPOS Topic: %s", relpos_state);
    cv::putText(img, buf, {560, 482}, cv::FONT_HERSHEY_SIMPLEX, 0.74, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);

    std::snprintf(buf, sizeof(buf), "Satellites: %u", static_cast<unsigned int>(num_sv));
    cv::putText(img, buf, {560, 524}, cv::FONT_HERSHEY_SIMPLEX, 0.66, cv::Scalar(210, 210, 210), 1, cv::LINE_AA);

    std::snprintf(buf, sizeof(buf), "Lat/Lon: %.7f / %.7f", last_fix_.latitude, last_fix_.longitude);
    cv::putText(img, buf, {560, 558}, cv::FONT_HERSHEY_SIMPLEX, 0.66, cv::Scalar(210, 210, 210), 1, cv::LINE_AA);

    cv::putText(
      img, ("Topics: " + fix_topic_ + " | " + quality_topic_ + " | " + pvt_topic_ + " | " + relpos_topic_).c_str(),
      {38, 611}, cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(145, 145, 152), 1, cv::LINE_AA);

    cv::imshow(window_name_, img);
    cv::waitKey(1);
  }

  std::string fix_topic_;
  std::string quality_topic_;
  std::string pvt_topic_;
  std::string relpos_topic_;
  std::string window_name_;

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr quality_sub_;
  rclcpp::Subscription<ublox_ubx_msgs::msg::UBXNavPVT>::SharedPtr pvt_sub_;
  rclcpp::Subscription<ublox_ubx_msgs::msg::UBXNavRelPosNED>::SharedPtr relpos_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool has_fix_ {false};
  bool has_quality_ {false};
  bool has_pvt_ {false};
  bool has_relpos_ {false};
  sensor_msgs::msg::NavSatFix last_fix_;
  rclcpp::Time last_fix_time_ {0, 0, RCL_ROS_TIME};
  rclcpp::Time last_quality_time_ {0, 0, RCL_ROS_TIME};
  rclcpp::Time last_pvt_time_ {0, 0, RCL_ROS_TIME};
  rclcpp::Time last_relpos_time_ {0, 0, RCL_ROS_TIME};
  uint8_t carr_soln_status_ {0};
  uint8_t relpos_carr_soln_status_ {0};
  bool pvt_gnss_fix_ok_ {false};
  bool pvt_diff_soln_ {false};
  bool relpos_diff_soln_ {false};
  bool relpos_valid_ {false};
  uint8_t pvt_num_sv_ {0};
  double gps_quality_ {0.0};
  double horiz_acc_sigma_m_ {0.0};
  double pvt_h_acc_m_ {0.0};
  double relpos_acc_length_m_ {0.0};
  double topic_timeout_sec_ {1.5};
  bool log_to_console_ {true};
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
