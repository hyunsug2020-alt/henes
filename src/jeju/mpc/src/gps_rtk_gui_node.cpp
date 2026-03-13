#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <ublox_ubx_msgs/msg/ubx_nav_pvt.hpp>
#include <ublox_ubx_msgs/msg/ubx_nav_rel_pos_ned.hpp>

namespace jeju_mpc
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr int kWindowWidth = 1500;
constexpr int kWindowHeight = 860;
const cv::Scalar kBg(18, 20, 24);
const cv::Scalar kPanel(34, 37, 42);
const cv::Scalar kPanelAlt(28, 31, 36);
const cv::Scalar kBorder(72, 78, 88);
const cv::Scalar kText(240, 242, 245);
const cv::Scalar kMuted(158, 164, 172);
const cv::Scalar kDanger(72, 92, 245);
const cv::Scalar kWarn(60, 190, 255);
const cv::Scalar kGood(88, 215, 124);
const cv::Scalar kGps1(30, 180, 255);
const cv::Scalar kGps2(210, 190, 60);

double wrap360(double deg)
{
  while (deg < 0.0) {
    deg += 360.0;
  }
  while (deg >= 360.0) {
    deg -= 360.0;
  }
  return deg;
}

std::string bearingText(double deg)
{
  static const char * kDirs[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  const int index = static_cast<int>(std::round(wrap360(deg) / 45.0)) % 8;
  return kDirs[index];
}

std::string navStatusText(int8_t status)
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

cv::Scalar navStatusColor(int8_t status)
{
  if (status >= sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX) {
    return kGood;
  }
  if (status == sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX) {
    return kWarn;
  }
  if (status == sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
    return cv::Scalar(40, 215, 255);
  }
  return kDanger;
}

}  // namespace

struct ReceiverView
{
  std::string label;
  std::string fix_topic;
  std::string pvt_topic;
  std::string relpos_topic;
  bool configured {false};

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub;
  rclcpp::Subscription<ublox_ubx_msgs::msg::UBXNavPVT>::SharedPtr pvt_sub;
  rclcpp::Subscription<ublox_ubx_msgs::msg::UBXNavRelPosNED>::SharedPtr relpos_sub;

  bool has_fix {false};
  bool has_pvt {false};
  bool has_relpos {false};
  sensor_msgs::msg::NavSatFix last_fix;
  rclcpp::Time last_fix_time {0, 0, RCL_ROS_TIME};
  rclcpp::Time last_pvt_time {0, 0, RCL_ROS_TIME};
  rclcpp::Time last_relpos_time {0, 0, RCL_ROS_TIME};

  uint8_t carr_soln_status {0};
  uint8_t relpos_carr_soln_status {0};
  bool gnss_fix_ok {false};
  bool diff_soln {false};
  bool relpos_diff_soln {false};
  bool relpos_valid {false};
  bool relpos_heading_valid {false};
  uint8_t num_sv {0};
  double horiz_acc_sigma_m {0.0};
  double pvt_h_acc_m {0.0};
  double relpos_length_m {0.0};
  double relpos_acc_length_m {0.0};
  double relpos_heading_deg {0.0};
  double relpos_acc_heading_deg {0.0};
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

    this->declare_parameter("gps1_label", "GPS1");
    this->declare_parameter("gps1_fix_topic", "");
    this->declare_parameter("gps1_pvt_topic", "");
    this->declare_parameter("gps1_relpos_topic", "");
    this->declare_parameter("gps2_label", "GPS2");
    this->declare_parameter("gps2_fix_topic", "");
    this->declare_parameter("gps2_pvt_topic", "");
    this->declare_parameter("gps2_relpos_topic", "");

    this->declare_parameter("heading_topic", "/dual_f9p/heading");
    this->declare_parameter("heading_valid_topic", "/dual_f9p/heading_valid");
    this->declare_parameter("heading_accuracy_topic", "/dual_f9p/heading_accuracy_deg");
    this->declare_parameter("baseline_topic", "/dual_f9p/baseline_length_m");
    this->declare_parameter("window_name", "HENES Dual GPS Heading Monitor");
    this->declare_parameter("refresh_hz", 10.0);
    this->declare_parameter("topic_timeout_sec", 1.5);
    this->declare_parameter("log_to_console", true);

    const std::string legacy_fix_topic = this->get_parameter("fix_topic").as_string();
    const std::string legacy_pvt_topic = this->get_parameter("pvt_topic").as_string();
    const std::string legacy_relpos_topic = this->get_parameter("relpos_topic").as_string();

    gps1_.label = this->get_parameter("gps1_label").as_string();
    gps1_.fix_topic = this->get_parameter("gps1_fix_topic").as_string();
    gps1_.pvt_topic = this->get_parameter("gps1_pvt_topic").as_string();
    gps1_.relpos_topic = this->get_parameter("gps1_relpos_topic").as_string();
    if (gps1_.fix_topic.empty()) {
      gps1_.fix_topic = legacy_fix_topic;
    }
    if (gps1_.pvt_topic.empty()) {
      gps1_.pvt_topic = legacy_pvt_topic;
    }
    if (gps1_.relpos_topic.empty()) {
      gps1_.relpos_topic = legacy_relpos_topic;
    }

    gps2_.label = this->get_parameter("gps2_label").as_string();
    gps2_.fix_topic = this->get_parameter("gps2_fix_topic").as_string();
    gps2_.pvt_topic = this->get_parameter("gps2_pvt_topic").as_string();
    gps2_.relpos_topic = this->get_parameter("gps2_relpos_topic").as_string();

    heading_topic_ = this->get_parameter("heading_topic").as_string();
    heading_valid_topic_ = this->get_parameter("heading_valid_topic").as_string();
    heading_accuracy_topic_ = this->get_parameter("heading_accuracy_topic").as_string();
    baseline_topic_ = this->get_parameter("baseline_topic").as_string();
    window_name_ = this->get_parameter("window_name").as_string();
    topic_timeout_sec_ = this->get_parameter("topic_timeout_sec").as_double();
    log_to_console_ = this->get_parameter("log_to_console").as_bool();
    const double hz = std::max(1.0, this->get_parameter("refresh_hz").as_double());

    setupReceiver(&gps1_);
    setupReceiver(&gps2_);

    const auto sensor_qos = rclcpp::SensorDataQoS();
    auto latched_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    latched_qos.keep_last(10);
    latched_qos.reliable();
    latched_qos.transient_local();

    heading_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      heading_topic_, sensor_qos,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        heading_deg_ = wrap360(msg->data);
        last_heading_time_ = this->now();
        has_heading_ = true;
      });

    heading_valid_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      heading_valid_topic_, sensor_qos,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        heading_valid_ = msg->data;
        last_heading_valid_time_ = this->now();
        has_heading_valid_ = true;
      });

    heading_acc_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      heading_accuracy_topic_, latched_qos,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        heading_acc_deg_ = msg->data;
        last_heading_acc_time_ = this->now();
        has_heading_acc_ = true;
      });

    baseline_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      baseline_topic_, latched_qos,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        baseline_m_ = msg->data;
        last_baseline_time_ = this->now();
        has_baseline_ = true;
      });

    cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
    cv::resizeWindow(window_name_, kWindowWidth, kWindowHeight);

    const auto period = std::chrono::duration<double>(1.0 / hz);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&GpsRtkGuiNode::drawFrame, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Dual GPS GUI started | gps1=%s gps2=%s heading=%s",
      gps1_.fix_topic.c_str(), gps2_.fix_topic.c_str(), heading_topic_.c_str());
  }

  ~GpsRtkGuiNode() override
  {
    cv::destroyAllWindows();
  }

private:
  void setupReceiver(ReceiverView * receiver)
  {
    if (receiver == nullptr) {
      return;
    }
    if (receiver->fix_topic.empty() && receiver->pvt_topic.empty() && receiver->relpos_topic.empty()) {
      receiver->configured = false;
      return;
    }

    receiver->configured = true;
    const auto sensor_qos = rclcpp::SensorDataQoS();
    auto ubx_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    ubx_qos.keep_last(10);
    ubx_qos.reliable();
    ubx_qos.transient_local();

    if (!receiver->fix_topic.empty()) {
      receiver->fix_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        receiver->fix_topic, sensor_qos,
        [this, receiver](const sensor_msgs::msg::NavSatFix::SharedPtr msg) { fixCb(receiver, msg); });
    }

    if (!receiver->pvt_topic.empty()) {
      receiver->pvt_sub = this->create_subscription<ublox_ubx_msgs::msg::UBXNavPVT>(
        receiver->pvt_topic, ubx_qos,
        [this, receiver](const ublox_ubx_msgs::msg::UBXNavPVT::SharedPtr msg) { pvtCb(receiver, msg); });
    }

    if (!receiver->relpos_topic.empty()) {
      receiver->relpos_sub = this->create_subscription<ublox_ubx_msgs::msg::UBXNavRelPosNED>(
        receiver->relpos_topic, ubx_qos,
        [this, receiver](const ublox_ubx_msgs::msg::UBXNavRelPosNED::SharedPtr msg) {
          relPosCb(receiver, msg);
        });
    }
  }

  void fixCb(ReceiverView * receiver, const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    receiver->last_fix = *msg;
    receiver->last_fix_time = this->now();
    receiver->has_fix = true;

    if (msg->position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
      const double vx = std::max(0.0, static_cast<double>(msg->position_covariance[0]));
      const double vy = std::max(0.0, static_cast<double>(msg->position_covariance[4]));
      receiver->horiz_acc_sigma_m = std::sqrt((vx + vy) * 0.5);
    }
  }

  void pvtCb(ReceiverView * receiver, const ublox_ubx_msgs::msg::UBXNavPVT::SharedPtr msg)
  {
    receiver->carr_soln_status = msg->carr_soln.status;
    receiver->gnss_fix_ok = msg->gnss_fix_ok;
    receiver->diff_soln = msg->diff_soln;
    receiver->num_sv = msg->num_sv;
    receiver->pvt_h_acc_m = static_cast<double>(msg->h_acc) * 1e-3;
    receiver->last_pvt_time = this->now();
    receiver->has_pvt = true;
  }

  void relPosCb(ReceiverView * receiver, const ublox_ubx_msgs::msg::UBXNavRelPosNED::SharedPtr msg)
  {
    receiver->relpos_carr_soln_status = msg->carr_soln.status;
    receiver->relpos_diff_soln = msg->diff_soln;
    receiver->relpos_valid = msg->rel_pos_valid;
    receiver->relpos_heading_valid = msg->rel_pos_heading_valid;
    receiver->relpos_length_m = static_cast<double>(msg->rel_pos_length) * 1e-2 +
      static_cast<double>(msg->rel_pos_hp_length) * 1e-4;
    receiver->relpos_acc_length_m = static_cast<double>(msg->acc_length) * 1e-4;
    receiver->relpos_heading_deg = static_cast<double>(msg->rel_pos_heading) * 1e-5;
    receiver->relpos_acc_heading_deg = static_cast<double>(msg->acc_heading) * 1e-5;
    receiver->last_relpos_time = this->now();
    receiver->has_relpos = true;
  }

  bool isFresh(const rclcpp::Time & stamp) const
  {
    if (stamp.nanoseconds() == 0) {
      return false;
    }
    return (this->now() - stamp).seconds() <= topic_timeout_sec_;
  }

  double ageSeconds(const rclcpp::Time & stamp) const
  {
    if (stamp.nanoseconds() == 0) {
      return std::numeric_limits<double>::infinity();
    }
    return std::max(0.0, (this->now() - stamp).seconds());
  }

  bool online(const ReceiverView & receiver) const
  {
    return (receiver.has_fix && isFresh(receiver.last_fix_time)) ||
      (receiver.has_pvt && isFresh(receiver.last_pvt_time)) ||
      (receiver.has_relpos && isFresh(receiver.last_relpos_time));
  }

  double accuracyMeters(const ReceiverView & receiver) const
  {
    if (receiver.has_relpos && isFresh(receiver.last_relpos_time) && receiver.relpos_acc_length_m > 0.0) {
      return receiver.relpos_acc_length_m;
    }
    if (receiver.has_pvt && isFresh(receiver.last_pvt_time) && receiver.pvt_h_acc_m > 0.0) {
      return receiver.pvt_h_acc_m;
    }
    return receiver.horiz_acc_sigma_m;
  }

  uint8_t carrierStatus(const ReceiverView & receiver) const
  {
    if (receiver.has_relpos && isFresh(receiver.last_relpos_time)) {
      return receiver.relpos_carr_soln_status;
    }
    return receiver.carr_soln_status;
  }

  bool diffSoln(const ReceiverView & receiver) const
  {
    if (receiver.has_relpos && isFresh(receiver.last_relpos_time)) {
      return receiver.relpos_diff_soln;
    }
    return receiver.diff_soln;
  }

  std::string stateText(const ReceiverView & receiver) const
  {
    if (!receiver.configured) {
      return "OFF";
    }
    if (!online(receiver)) {
      return "STALE";
    }

    const uint8_t carrier = carrierStatus(receiver);
    if (carrier == 2) {
      return "FIXED";
    }
    if (carrier == 1) {
      return "FLOAT";
    }
    if (diffSoln(receiver)) {
      return "DGNSS";
    }
    if (receiver.has_fix && isFresh(receiver.last_fix_time)) {
      return navStatusText(receiver.last_fix.status.status);
    }
    return "ONLINE";
  }

  cv::Scalar stateColor(const ReceiverView & receiver) const
  {
    if (!receiver.configured || !online(receiver)) {
      return kDanger;
    }
    const uint8_t carrier = carrierStatus(receiver);
    if (carrier == 2) {
      return kGood;
    }
    if (carrier == 1) {
      return kWarn;
    }
    if (receiver.has_fix && isFresh(receiver.last_fix_time)) {
      return navStatusColor(receiver.last_fix.status.status);
    }
    return cv::Scalar(130, 160, 220);
  }

  std::string carrierText(const ReceiverView & receiver) const
  {
    const uint8_t carrier = carrierStatus(receiver);
    if (carrier == 2) {
      return "FIXED";
    }
    if (carrier == 1) {
      return "FLOAT";
    }
    return "NONE";
  }

  cv::Scalar carrierColor(uint8_t carrier) const
  {
    if (carrier >= 2) {
      return kGood;
    }
    if (carrier == 1) {
      return kWarn;
    }
    return cv::Scalar(96, 104, 116);
  }

  std::string boolText(bool value) const
  {
    return value ? "ON" : "OFF";
  }

  std::string ageText(const rclcpp::Time & stamp) const
  {
    if (stamp.nanoseconds() == 0) {
      return "--";
    }
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.1fs", ageSeconds(stamp));
    return buf;
  }

  bool headingFresh() const
  {
    return has_heading_ && isFresh(last_heading_time_);
  }

  bool headingValidFresh() const
  {
    return has_heading_valid_ && isFresh(last_heading_valid_time_);
  }

  bool headingIsUsable() const
  {
    return headingFresh() && headingValidFresh() && heading_valid_;
  }

  std::string headingWaitReason() const
  {
    if (!gps1_.configured) {
      return "WAIT_GPS1";
    }
    if (!(gps1_.has_relpos && isFresh(gps1_.last_relpos_time))) {
      return "WAIT_RELPOS";
    }
    if (!gps1_.relpos_diff_soln) {
      return "WAIT_RTCM";
    }
    if (gps1_.relpos_carr_soln_status == 0) {
      return "WAIT_CARRIER";
    }
    if (gps1_.relpos_carr_soln_status == 1) {
      return "FLOAT";
    }
    if (!gps1_.relpos_valid) {
      return "WAIT_RELPOS_VALID";
    }
    if (!gps1_.relpos_heading_valid) {
      return "WAIT_HEADING_VALID";
    }
    if (!headingFresh()) {
      return "WAIT_HEADING_TOPIC";
    }
    if (!headingValidFresh()) {
      return "WAIT_VALID_TOPIC";
    }
    if (!heading_valid_) {
      return "WAIT_HEADING_NODE";
    }
    return "READY";
  }

  void drawPill(
    cv::Mat & img, const cv::Point & origin, const std::string & text, const cv::Scalar & color,
    double font_scale = 0.68) const
  {
    int baseline = 0;
    const cv::Size size = cv::getTextSize(text, cv::FONT_HERSHEY_DUPLEX, font_scale, 1, &baseline);
    const cv::Rect box(origin.x, origin.y - size.height - 12, size.width + 26, size.height + 16);
    cv::rectangle(img, box, color, cv::FILLED);
    cv::rectangle(img, box, cv::Scalar(255, 255, 255), 1);
    cv::putText(
      img, text, {box.x + 13, box.y + box.height - 7}, cv::FONT_HERSHEY_DUPLEX, font_scale,
      cv::Scalar(12, 14, 18), 1, cv::LINE_AA);
  }

  void drawCarrierStageBar(
    cv::Mat & img, const cv::Rect & rect, uint8_t carrier, bool enabled) const
  {
    static const char * labels[] = {"NO", "FLT", "FIX"};
    const int gap = 6;
    const int seg_width = (rect.width - gap * 2) / 3;
    const int seg_height = rect.height;
    const uint8_t clamped = std::min<uint8_t>(carrier, 2);

    for (int idx = 0; idx < 3; ++idx) {
      const int x = rect.x + idx * (seg_width + gap);
      const cv::Rect seg(x, rect.y, seg_width, seg_height);
      const bool active = enabled && static_cast<int>(clamped) == idx;
      const cv::Scalar fill = active ? carrierColor(clamped) : cv::Scalar(58, 64, 72);
      const cv::Scalar border = active ? cv::Scalar(255, 255, 255) : cv::Scalar(92, 100, 112);
      const cv::Scalar text = active ? cv::Scalar(12, 14, 18) : kMuted;

      cv::rectangle(img, seg, fill, cv::FILLED);
      cv::rectangle(img, seg, border, 1);
      cv::putText(
        img, labels[idx], {seg.x + 8, seg.y + seg.height - 7},
        cv::FONT_HERSHEY_DUPLEX, 0.42, text, 1, cv::LINE_AA);
    }
  }

  void drawReceiverCard(
    cv::Mat & img, const cv::Rect & rect, const ReceiverView & receiver, const cv::Scalar & accent) const
  {
    cv::rectangle(img, rect, kPanel, cv::FILLED);
    cv::rectangle(img, rect, kBorder, 2);
    cv::rectangle(img, {rect.x, rect.y, rect.width, 8}, accent, cv::FILLED);

    cv::putText(
      img, receiver.label, {rect.x + 22, rect.y + 48}, cv::FONT_HERSHEY_DUPLEX, 1.0, kText, 2,
      cv::LINE_AA);
    drawPill(img, {rect.x + rect.width - 140, rect.y + 54}, stateText(receiver), stateColor(receiver));

    if (!receiver.configured) {
      cv::putText(
        img, "NOT CONFIGURED", {rect.x + 34, rect.y + 140}, cv::FONT_HERSHEY_DUPLEX, 1.1, kDanger, 2,
        cv::LINE_AA);
      cv::putText(
        img, "Set gps2_* parameters to enable the second receiver card.", {rect.x + 34, rect.y + 190},
        cv::FONT_HERSHEY_SIMPLEX, 0.72, kMuted, 1, cv::LINE_AA);
      return;
    }

    const double acc_m = accuracyMeters(receiver);
    const bool fix_fresh = receiver.has_fix && isFresh(receiver.last_fix_time);
    const bool pvt_fresh = receiver.has_pvt && isFresh(receiver.last_pvt_time);
    const bool relpos_fresh = receiver.has_relpos && isFresh(receiver.last_relpos_time);
    char buf[256];

    int y = rect.y + 108;
    const int left_x = rect.x + 26;
    const int right_x = rect.x + rect.width / 2 + 10;
    const int dy = 46;

    std::snprintf(buf, sizeof(buf), "Link: %s", online(receiver) ? "ONLINE" : "STALE");
    cv::putText(img, buf, {left_x, y}, cv::FONT_HERSHEY_SIMPLEX, 0.72, kText, 1, cv::LINE_AA);
    std::snprintf(buf, sizeof(buf), "RTK: %s", carrierText(receiver).c_str());
    cv::putText(img, buf, {left_x, y + dy}, cv::FONT_HERSHEY_SIMPLEX, 0.72, kText, 1, cv::LINE_AA);
    drawCarrierStageBar(
      img, {left_x + 96, y + dy - 18, 144, 24},
      carrierStatus(receiver), online(receiver));
    std::snprintf(buf, sizeof(buf), "RTCM: %s", boolText(diffSoln(receiver)).c_str());
    cv::putText(img, buf, {left_x, y + dy * 2}, cv::FONT_HERSHEY_SIMPLEX, 0.72, kText, 1, cv::LINE_AA);
    std::snprintf(buf, sizeof(buf), "RelPos: %s", receiver.relpos_valid ? "VALID" : "INVALID");
    cv::putText(img, buf, {left_x, y + dy * 3}, cv::FONT_HERSHEY_SIMPLEX, 0.72, kText, 1, cv::LINE_AA);

    std::snprintf(
      buf, sizeof(buf), "Fix: %s",
      fix_fresh ? navStatusText(receiver.last_fix.status.status).c_str() : "WAIT");
    cv::putText(img, buf, {right_x, y}, cv::FONT_HERSHEY_SIMPLEX, 0.72, kText, 1, cv::LINE_AA);
    std::snprintf(buf, sizeof(buf), "SV: %u", static_cast<unsigned int>(pvt_fresh ? receiver.num_sv : 0U));
    cv::putText(img, buf, {right_x, y + dy}, cv::FONT_HERSHEY_SIMPLEX, 0.72, kText, 1, cv::LINE_AA);
    std::snprintf(buf, sizeof(buf), "H Acc: %s", acc_m > 0.0 ? "" : "N/A");
    cv::putText(img, buf, {right_x, y + dy * 2}, cv::FONT_HERSHEY_SIMPLEX, 0.72, kText, 1, cv::LINE_AA);
    if (acc_m > 0.0) {
      std::snprintf(buf, sizeof(buf), "%.3f m", acc_m);
      cv::putText(img, buf, {right_x + 82, y + dy * 2}, cv::FONT_HERSHEY_SIMPLEX, 0.72, accent, 2, cv::LINE_AA);
    }
    std::snprintf(
      buf, sizeof(buf), "Age F/P/R: %s / %s / %s",
      ageText(receiver.last_fix_time).c_str(), ageText(receiver.last_pvt_time).c_str(),
      ageText(receiver.last_relpos_time).c_str());
    cv::putText(img, buf, {right_x, y + dy * 3}, cv::FONT_HERSHEY_SIMPLEX, 0.62, kMuted, 1, cv::LINE_AA);

    cv::rectangle(img, {rect.x + 22, rect.y + 308, rect.width - 44, 1}, cv::Scalar(62, 68, 76), cv::FILLED);

    if (relpos_fresh) {
      std::snprintf(
        buf, sizeof(buf), "Baseline %.3f m | Heading %.2f deg | Acc %.2f deg",
        receiver.relpos_length_m, receiver.relpos_heading_deg, receiver.relpos_acc_heading_deg);
      cv::putText(img, buf, {rect.x + 24, rect.y + 346}, cv::FONT_HERSHEY_SIMPLEX, 0.63, kText, 1, cv::LINE_AA);
    } else {
      cv::putText(
        img, "Relative-position stream is not live.", {rect.x + 24, rect.y + 346},
        cv::FONT_HERSHEY_SIMPLEX, 0.63, kMuted, 1, cv::LINE_AA);
    }

    if (fix_fresh) {
      std::snprintf(
        buf, sizeof(buf), "Lat/Lon %.7f / %.7f",
        receiver.last_fix.latitude, receiver.last_fix.longitude);
      cv::putText(img, buf, {rect.x + 24, rect.y + 392}, cv::FONT_HERSHEY_SIMPLEX, 0.63, kText, 1, cv::LINE_AA);
    } else {
      cv::putText(
        img, "Lat/Lon waiting for NavSatFix.", {rect.x + 24, rect.y + 392},
        cv::FONT_HERSHEY_SIMPLEX, 0.63, kMuted, 1, cv::LINE_AA);
    }

    std::snprintf(
      buf, sizeof(buf), "Topics: %s | %s | %s",
      receiver.fix_topic.c_str(),
      receiver.pvt_topic.empty() ? "-" : receiver.pvt_topic.c_str(),
      receiver.relpos_topic.empty() ? "-" : receiver.relpos_topic.c_str());
    cv::putText(img, buf, {rect.x + 24, rect.y + 432}, cv::FONT_HERSHEY_SIMPLEX, 0.52, kMuted, 1, cv::LINE_AA);
  }

  void drawCompass(cv::Mat & img, const cv::Point & center, int radius) const
  {
    cv::circle(img, center, radius, cv::Scalar(86, 92, 100), 2, cv::LINE_AA);
    cv::circle(img, center, radius - 28, cv::Scalar(64, 70, 78), 1, cv::LINE_AA);

    for (int deg = 0; deg < 360; deg += 15) {
      const double rad = (deg - 90.0) * kPi / 180.0;
      const int outer = radius;
      const int inner = (deg % 45 == 0) ? radius - 20 : radius - 10;
      const cv::Point p1(
        center.x + static_cast<int>(std::cos(rad) * inner),
        center.y + static_cast<int>(std::sin(rad) * inner));
      const cv::Point p2(
        center.x + static_cast<int>(std::cos(rad) * outer),
        center.y + static_cast<int>(std::sin(rad) * outer));
      cv::line(img, p1, p2, cv::Scalar(118, 124, 132), 1, cv::LINE_AA);
    }

    cv::putText(img, "N", {center.x - 12, center.y - radius - 12}, cv::FONT_HERSHEY_DUPLEX, 0.9, kText, 2, cv::LINE_AA);
    cv::putText(img, "E", {center.x + radius + 14, center.y + 8}, cv::FONT_HERSHEY_DUPLEX, 0.9, kText, 2, cv::LINE_AA);
    cv::putText(img, "S", {center.x - 10, center.y + radius + 32}, cv::FONT_HERSHEY_DUPLEX, 0.9, kText, 2, cv::LINE_AA);
    cv::putText(img, "W", {center.x - radius - 34, center.y + 8}, cv::FONT_HERSHEY_DUPLEX, 0.9, kText, 2, cv::LINE_AA);

    if (!headingIsUsable()) {
      cv::putText(
        img, "WAIT", {center.x - 52, center.y + 12}, cv::FONT_HERSHEY_DUPLEX, 1.2, kDanger, 2,
        cv::LINE_AA);
      return;
    }

    const double rad = (heading_deg_ - 90.0) * kPi / 180.0;
    const cv::Point tip(
      center.x + static_cast<int>(std::cos(rad) * (radius - 22)),
      center.y + static_cast<int>(std::sin(rad) * (radius - 22)));
    const cv::Point tail(
      center.x - static_cast<int>(std::cos(rad) * (radius - 60)),
      center.y - static_cast<int>(std::sin(rad) * (radius - 60)));

    cv::arrowedLine(img, tail, tip, cv::Scalar(250, 250, 250), 5, cv::LINE_AA, 0, 0.22);
    cv::circle(img, center, 10, cv::Scalar(250, 250, 250), cv::FILLED, cv::LINE_AA);
  }

  void drawVehicleCue(cv::Mat & img, const cv::Rect & rect) const
  {
    const cv::Point center(rect.x + rect.width / 2, rect.y + rect.height / 2);
    const cv::Rect car(center.x - 68, center.y - 34, 136, 68);
    cv::rectangle(img, car, cv::Scalar(64, 68, 76), cv::FILLED);
    cv::rectangle(img, car, cv::Scalar(188, 194, 202), 2);
    cv::rectangle(img, {car.x + 16, car.y + 8, car.width - 32, 16}, cv::Scalar(92, 100, 114), cv::FILLED);
    cv::arrowedLine(
      img, {center.x, car.y + car.height / 2}, {center.x, car.y - 34}, cv::Scalar(220, 225, 230), 3,
      cv::LINE_AA, 0, 0.25);

    const cv::Point gps1(car.x + 12, center.y);
    const cv::Point gps2(car.x + car.width - 12, center.y);
    cv::circle(img, gps1, 8, kGps1, cv::FILLED, cv::LINE_AA);
    cv::circle(img, gps2, 8, kGps2, cv::FILLED, cv::LINE_AA);
    cv::putText(img, "1", {gps1.x - 6, gps1.y - 16}, cv::FONT_HERSHEY_DUPLEX, 0.55, kGps1, 2, cv::LINE_AA);
    cv::putText(img, "2", {gps2.x - 6, gps2.y - 16}, cv::FONT_HERSHEY_DUPLEX, 0.55, kGps2, 2, cv::LINE_AA);
    cv::putText(
      img, "LEFT=GPS1   RIGHT=GPS2", {rect.x + 16, rect.y + rect.height - 6},
      cv::FONT_HERSHEY_SIMPLEX, 0.6, kMuted, 1, cv::LINE_AA);
  }

  void drawHeadingPanel(cv::Mat & img, const cv::Rect & rect) const
  {
    cv::rectangle(img, rect, kPanelAlt, cv::FILLED);
    cv::rectangle(img, rect, kBorder, 2);
    cv::putText(
      img, "DUAL GPS HEADING", {rect.x + 26, rect.y + 46}, cv::FONT_HERSHEY_DUPLEX, 1.0, kText, 2,
      cv::LINE_AA);

    const bool valid = headingIsUsable();
    drawPill(
      img, {rect.x + rect.width - 160, rect.y + 52},
      valid ? "VALID" : headingWaitReason(), valid ? kGood : kDanger, valid ? 0.72 : 0.52);

    char buf[256];
    if (valid) {
      std::snprintf(buf, sizeof(buf), "%.2f deg", heading_deg_);
    } else if (headingFresh()) {
      std::snprintf(buf, sizeof(buf), "%.2f deg", heading_deg_);
    } else {
      std::snprintf(buf, sizeof(buf), "--.-- deg");
    }
    cv::putText(
      img, buf, {rect.x + 34, rect.y + 106}, cv::FONT_HERSHEY_DUPLEX, 1.25, valid ? kText : kMuted, 2,
      cv::LINE_AA);
    cv::putText(
      img, valid ? bearingText(heading_deg_) : "WAIT",
      {rect.x + 36, rect.y + 144}, cv::FONT_HERSHEY_SIMPLEX, 0.78, valid ? kWarn : kMuted, 2, cv::LINE_AA);

    const cv::Point center(rect.x + rect.width / 2, rect.y + 318);
    drawCompass(img, center, 170);

    cv::Rect metric_box(rect.x + 34, rect.y + 530, rect.width - 68, 118);
    cv::rectangle(img, metric_box, cv::Scalar(40, 43, 50), cv::FILLED);
    cv::rectangle(img, metric_box, cv::Scalar(80, 86, 94), 1);

    std::snprintf(
      buf, sizeof(buf), "Heading Accuracy: %s",
      has_heading_acc_ && isFresh(last_heading_acc_time_) ? "" : "N/A");
    cv::putText(img, buf, {metric_box.x + 18, metric_box.y + 34}, cv::FONT_HERSHEY_SIMPLEX, 0.68, kText, 1, cv::LINE_AA);
    if (has_heading_acc_ && isFresh(last_heading_acc_time_)) {
      std::snprintf(buf, sizeof(buf), "%.2f deg", heading_acc_deg_);
      cv::putText(img, buf, {metric_box.x + 214, metric_box.y + 34}, cv::FONT_HERSHEY_SIMPLEX, 0.74, kWarn, 2, cv::LINE_AA);
    }

    std::snprintf(
      buf, sizeof(buf), "Baseline: %s",
      has_baseline_ && isFresh(last_baseline_time_) ? "" : "N/A");
    cv::putText(img, buf, {metric_box.x + 18, metric_box.y + 72}, cv::FONT_HERSHEY_SIMPLEX, 0.68, kText, 1, cv::LINE_AA);
    if (has_baseline_ && isFresh(last_baseline_time_)) {
      std::snprintf(buf, sizeof(buf), "%.3f m", baseline_m_);
      cv::putText(img, buf, {metric_box.x + 118, metric_box.y + 72}, cv::FONT_HERSHEY_SIMPLEX, 0.74, kGood, 2, cv::LINE_AA);
    }

    std::snprintf(
      buf, sizeof(buf), "Rover RELPOS: carr=%u diff=%s rel=%s head=%s",
      static_cast<unsigned int>(gps1_.relpos_carr_soln_status),
      boolText(gps1_.relpos_diff_soln).c_str(),
      gps1_.relpos_valid ? "OK" : "NO",
      gps1_.relpos_heading_valid ? "OK" : "NO");
    cv::putText(img, buf, {metric_box.x + 18, metric_box.y + 106}, cv::FONT_HERSHEY_SIMPLEX, 0.58, kMuted, 1, cv::LINE_AA);

    drawVehicleCue(img, {rect.x + 92, rect.y + 666, rect.width - 184, 104});
  }

  void drawFrame()
  {
    cv::Mat img(kWindowHeight, kWindowWidth, CV_8UC3, kBg);
    cv::rectangle(img, {0, 0, kWindowWidth, 82}, cv::Scalar(26, 29, 34), cv::FILLED);
    cv::putText(
      img, "HENES Dual GPS / Heading Monitor", {36, 52}, cv::FONT_HERSHEY_DUPLEX, 1.2, kText, 2,
      cv::LINE_AA);
    cv::putText(
      img, "GPS1=Left/Rover  |  GPS2=Right/Base  |  Heading uses /dual_f9p/* topics",
      {38, 76}, cv::FONT_HERSHEY_SIMPLEX, 0.58, kMuted, 1, cv::LINE_AA);

    drawReceiverCard(img, {34, 106, 430, 458}, gps1_, kGps1);
    drawHeadingPanel(img, {498, 106, 504, 700});
    drawReceiverCard(img, {1036, 106, 430, 458}, gps2_, kGps2);

    cv::Rect footer(34, 598, 430, 208);
    cv::rectangle(img, footer, kPanel, cv::FILLED);
    cv::rectangle(img, footer, kBorder, 2);
    cv::putText(img, "CHECKLIST", {footer.x + 22, footer.y + 40}, cv::FONT_HERSHEY_DUPLEX, 0.86, kText, 2, cv::LINE_AA);
    cv::putText(
      img, "1. GPS1/GPS2 cards must be ONLINE", {footer.x + 24, footer.y + 84}, cv::FONT_HERSHEY_SIMPLEX, 0.66,
      kText, 1, cv::LINE_AA);
    cv::putText(
      img, "2. GPS1 RTK should step NONE -> FLOAT -> FIXED", {footer.x + 24, footer.y + 118},
      cv::FONT_HERSHEY_SIMPLEX, 0.66, kText, 1, cv::LINE_AA);
    cv::putText(
      img, "3. Heading panel must switch from WAIT_* to VALID", {footer.x + 24, footer.y + 152},
      cv::FONT_HERSHEY_SIMPLEX, 0.66, kText, 1, cv::LINE_AA);
    cv::putText(
      img, "4. If WAIT_RTCM persists, check GPS1<->GPS2 moving-base link", {footer.x + 24, footer.y + 186},
      cv::FONT_HERSHEY_SIMPLEX, 0.62, kWarn, 1, cv::LINE_AA);

    cv::Rect footer_right(1036, 598, 430, 208);
    cv::rectangle(img, footer_right, kPanel, cv::FILLED);
    cv::rectangle(img, footer_right, kBorder, 2);
    cv::putText(
      img, "TOPICS", {footer_right.x + 22, footer_right.y + 40}, cv::FONT_HERSHEY_DUPLEX, 0.86, kText, 2,
      cv::LINE_AA);
    cv::putText(
      img, heading_topic_, {footer_right.x + 24, footer_right.y + 82},
      cv::FONT_HERSHEY_SIMPLEX, 0.62, kText, 1, cv::LINE_AA);
    cv::putText(
      img, heading_valid_topic_, {footer_right.x + 24, footer_right.y + 116},
      cv::FONT_HERSHEY_SIMPLEX, 0.62, kText, 1, cv::LINE_AA);
    cv::putText(
      img, heading_accuracy_topic_, {footer_right.x + 24, footer_right.y + 150},
      cv::FONT_HERSHEY_SIMPLEX, 0.62, kText, 1, cv::LINE_AA);
    cv::putText(
      img, baseline_topic_, {footer_right.x + 24, footer_right.y + 184},
      cv::FONT_HERSHEY_SIMPLEX, 0.62, kText, 1, cv::LINE_AA);

    if (log_to_console_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "gps1=%s gps2=%s heading=%s reason=%s",
        stateText(gps1_).c_str(), stateText(gps2_).c_str(),
        headingIsUsable() ? "VALID" : "INVALID", headingWaitReason().c_str());
    }

    cv::imshow(window_name_, img);
    cv::waitKey(1);
  }

  ReceiverView gps1_;
  ReceiverView gps2_;

  std::string heading_topic_;
  std::string heading_valid_topic_;
  std::string heading_accuracy_topic_;
  std::string baseline_topic_;
  std::string window_name_;

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr heading_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr heading_valid_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr heading_acc_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr baseline_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool has_heading_ {false};
  bool has_heading_valid_ {false};
  bool has_heading_acc_ {false};
  bool has_baseline_ {false};
  bool heading_valid_ {false};
  double heading_deg_ {0.0};
  double heading_acc_deg_ {0.0};
  double baseline_m_ {0.0};
  rclcpp::Time last_heading_time_ {0, 0, RCL_ROS_TIME};
  rclcpp::Time last_heading_valid_time_ {0, 0, RCL_ROS_TIME};
  rclcpp::Time last_heading_acc_time_ {0, 0, RCL_ROS_TIME};
  rclcpp::Time last_baseline_time_ {0, 0, RCL_ROS_TIME};

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
