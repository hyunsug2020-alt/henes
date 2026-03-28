#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <ublox_ubx_msgs/msg/ubx_nav_pvt.hpp>
#include <ublox_ubx_msgs/msg/ubx_nav_rel_pos_ned.hpp>

namespace jeju_mpc
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr int kWindowWidth  = 1500;
constexpr int kWindowHeight = 1080;
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

double wrap360(double deg)
{
  while (deg < 0.0)   { deg += 360.0; }
  while (deg >= 360.0){ deg -= 360.0; }
  return deg;
}

std::string bearingText(double deg)
{
  static const char * kDirs[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  return kDirs[static_cast<int>(std::round(wrap360(deg) / 45.0)) % 8];
}

std::string navStatusText(int8_t status)
{
  if (status >= sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX) { return "RTK FIX"; }
  if (status == sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX) { return "SBAS FIX"; }
  if (status == sensor_msgs::msg::NavSatStatus::STATUS_FIX)      { return "GPS FIX"; }
  return "NO FIX";
}

cv::Scalar navStatusColor(int8_t status)
{
  if (status >= sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX) { return kGood; }
  if (status == sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX) { return kWarn; }
  if (status == sensor_msgs::msg::NavSatStatus::STATUS_FIX)      { return cv::Scalar(40, 215, 255); }
  return kDanger;
}

}  // namespace

struct ReceiverView
{
  std::string label;
  std::string fix_topic;
  std::string pvt_topic;
  bool configured{false};

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr    fix_sub;
  rclcpp::Subscription<ublox_ubx_msgs::msg::UBXNavPVT>::SharedPtr pvt_sub;

  bool has_fix{false};
  bool has_pvt{false};
  sensor_msgs::msg::NavSatFix last_fix;
  rclcpp::Time last_fix_time{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_pvt_time{0, 0, RCL_ROS_TIME};

  uint8_t carr_soln_status{0};
  bool    gnss_fix_ok{false};
  bool    diff_soln{false};
  uint8_t num_sv{0};
  double  pvt_h_acc_m{0.0};
  double  horiz_acc_sigma_m{0.0};
};

class GpsRtkGuiNode : public rclcpp::Node
{
public:
  GpsRtkGuiNode()
  : Node("gps_rtk_gui_node")
  {
    declare_parameter("gps1_label",     "GPS1");
    declare_parameter("gps1_fix_topic", "/gnss_left/fix");
    declare_parameter("gps1_pvt_topic", "");
    declare_parameter("heading_topic",  "/eskf/heading_deg");
    declare_parameter("cov_topic",      "/eskf/covariance_trace");
    declare_parameter("imu_topic",      "/handsfree/imu");
    declare_parameter("gps_vel_topic",  "/gnss_left/fix_velocity");
    declare_parameter("window_name",    "HENES GPS1 + ESKF Monitor");
    declare_parameter("refresh_hz",     15.0);
    declare_parameter("topic_timeout_sec", 1.5);
    declare_parameter("log_to_console", true);

    gps1_.label     = get_parameter("gps1_label").as_string();
    gps1_.fix_topic = get_parameter("gps1_fix_topic").as_string();
    gps1_.pvt_topic = get_parameter("gps1_pvt_topic").as_string();
    heading_topic_  = get_parameter("heading_topic").as_string();
    cov_topic_      = get_parameter("cov_topic").as_string();
    window_name_    = get_parameter("window_name").as_string();
    topic_timeout_  = get_parameter("topic_timeout_sec").as_double();
    log_to_console_ = get_parameter("log_to_console").as_bool();
    const double hz = std::max(1.0, get_parameter("refresh_hz").as_double());

    setupReceiver(&gps1_);

    const auto sensor_qos = rclcpp::SensorDataQoS();

    heading_sub_ = create_subscription<std_msgs::msg::Float64>(
      heading_topic_, sensor_qos,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        heading_deg_      = wrap360(msg->data);
        last_heading_time_ = now();
        has_heading_       = true;
      });

    cov_sub_ = create_subscription<std_msgs::msg::Float64>(
      cov_topic_, sensor_qos,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        cov_trace_      = msg->data;
        last_cov_time_  = now();
        has_cov_        = true;
      });

    const std::string gps_vel_topic = get_parameter("gps_vel_topic").as_string();
    if (!gps_vel_topic.empty()) {
      gps_vel_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
        gps_vel_topic, rclcpp::SensorDataQoS(),
        [this](const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
          const double vx = msg->twist.twist.linear.x;
          const double vy = msg->twist.twist.linear.y;
          gps_speed_mps_ = std::hypot(vx, vy);
          if (gps_speed_mps_ > 0.3) {
            gps_heading_deg_ = std::atan2(vy, vx) * 180.0 / kPi;
            last_gps_vel_time_ = now();
            has_gps_vel_ = true;
          }
        });
    }

    const std::string imu_topic = get_parameter("imu_topic").as_string();
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        const double qx = msg->orientation.x, qy = msg->orientation.y;
        const double qz = msg->orientation.z, qw = msg->orientation.w;
        imu_roll_  = std::atan2(2*(qw*qx+qy*qz), 1-2*(qx*qx+qy*qy)) * 180.0/kPi;
        imu_pitch_ = std::asin(std::max(-1.0, std::min(1.0, 2*(qw*qy-qz*qx)))) * 180.0/kPi;
        imu_yaw_   = std::atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz)) * 180.0/kPi;
        imu_gx_ = msg->angular_velocity.x    * 180.0/kPi;
        imu_gy_ = msg->angular_velocity.y    * 180.0/kPi;
        imu_gz_ = msg->angular_velocity.z    * 180.0/kPi;
        imu_ax_ = msg->linear_acceleration.x;
        imu_ay_ = msg->linear_acceleration.y;
        imu_az_ = msg->linear_acceleration.z;
        last_imu_time_ = now();
        has_imu_       = true;
      });

    cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
    cv::resizeWindow(window_name_, kWindowWidth, kWindowHeight);

    const auto period = std::chrono::duration<double>(1.0 / hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&GpsRtkGuiNode::drawFrame, this));

    RCLCPP_INFO(get_logger(), "GPS1 GUI started | fix=%s heading=%s",
      gps1_.fix_topic.c_str(), heading_topic_.c_str());
  }

  ~GpsRtkGuiNode() override { cv::destroyAllWindows(); }

private:
  void setupReceiver(ReceiverView * r)
  {
    if (!r || (r->fix_topic.empty() && r->pvt_topic.empty())) {
      r->configured = false;
      return;
    }
    r->configured = true;
    const auto sensor_qos = rclcpp::SensorDataQoS();
    auto ubx_qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    ubx_qos.keep_last(10).reliable().transient_local();

    if (!r->fix_topic.empty()) {
      r->fix_sub = create_subscription<sensor_msgs::msg::NavSatFix>(
        r->fix_topic, sensor_qos,
        [this, r](const sensor_msgs::msg::NavSatFix::SharedPtr msg) { fixCb(r, msg); });
    }
    if (!r->pvt_topic.empty()) {
      r->pvt_sub = create_subscription<ublox_ubx_msgs::msg::UBXNavPVT>(
        r->pvt_topic, ubx_qos,
        [this, r](const ublox_ubx_msgs::msg::UBXNavPVT::SharedPtr msg) { pvtCb(r, msg); });
    }
  }

  void fixCb(ReceiverView * r, const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    r->last_fix      = *msg;
    r->last_fix_time = now();
    r->has_fix       = true;
    if (msg->position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
      const double vx = std::max(0.0, (double)msg->position_covariance[0]);
      const double vy = std::max(0.0, (double)msg->position_covariance[4]);
      r->horiz_acc_sigma_m = std::sqrt((vx + vy) * 0.5);
    }
  }

  void pvtCb(ReceiverView * r, const ublox_ubx_msgs::msg::UBXNavPVT::SharedPtr msg)
  {
    r->carr_soln_status = msg->carr_soln.status;
    r->gnss_fix_ok      = msg->gnss_fix_ok;
    r->diff_soln        = msg->diff_soln;
    r->num_sv           = msg->num_sv;
    r->pvt_h_acc_m      = (double)msg->h_acc * 1e-3;
    r->last_pvt_time    = now();
    r->has_pvt          = true;
  }

  bool isFresh(const rclcpp::Time & t) const
  {
    return t.nanoseconds() != 0 && (now() - t).seconds() <= topic_timeout_;
  }

  double ageSeconds(const rclcpp::Time & t) const
  {
    return t.nanoseconds() == 0 ? std::numeric_limits<double>::infinity()
                                : std::max(0.0, (now() - t).seconds());
  }

  bool online(const ReceiverView & r) const
  {
    return (r.has_fix && isFresh(r.last_fix_time)) ||
           (r.has_pvt && isFresh(r.last_pvt_time));
  }

  std::string stateText(const ReceiverView & r) const
  {
    if (!r.configured)     { return "OFF"; }
    if (!online(r))        { return "STALE"; }
    if (r.carr_soln_status == 2)                          { return "FIXED"; }
    if (r.carr_soln_status == 1)                          { return "FLOAT"; }
    if (r.diff_soln)                                      { return "DGNSS"; }
    if (r.has_fix && isFresh(r.last_fix_time))            { return navStatusText(r.last_fix.status.status); }
    return "ONLINE";
  }

  cv::Scalar stateColor(const ReceiverView & r) const
  {
    if (!r.configured || !online(r)) { return kDanger; }
    if (r.carr_soln_status == 2)     { return kGood;   }
    if (r.carr_soln_status == 1)     { return kWarn;   }
    if (r.has_fix && isFresh(r.last_fix_time)) { return navStatusColor(r.last_fix.status.status); }
    return cv::Scalar(130, 160, 220);
  }

  std::string carrierText(const ReceiverView & r) const
  {
    if (r.carr_soln_status == 2) { return "FIXED"; }
    if (r.carr_soln_status == 1) { return "FLOAT"; }
    return "NONE";
  }

  cv::Scalar carrierColor(uint8_t c) const
  {
    if (c >= 2) { return kGood; }
    if (c == 1) { return kWarn; }
    return cv::Scalar(96, 104, 116);
  }

  std::string ageText(const rclcpp::Time & t) const
  {
    if (t.nanoseconds() == 0) { return "--"; }
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.1fs", ageSeconds(t));
    return buf;
  }

  void drawPill(cv::Mat & img, const cv::Point & origin,
                const std::string & text, const cv::Scalar & color,
                double font_scale = 0.68) const
  {
    int baseline = 0;
    const cv::Size sz = cv::getTextSize(text, cv::FONT_HERSHEY_DUPLEX, font_scale, 1, &baseline);
    const cv::Rect box(origin.x, origin.y - sz.height - 12, sz.width + 26, sz.height + 16);
    cv::rectangle(img, box, color, cv::FILLED);
    cv::rectangle(img, box, cv::Scalar(255, 255, 255), 1);
    cv::putText(img, text, {box.x + 13, box.y + box.height - 7},
      cv::FONT_HERSHEY_DUPLEX, font_scale, cv::Scalar(12, 14, 18), 1, cv::LINE_AA);
  }

  void drawCarrierBar(cv::Mat & img, const cv::Rect & rect, uint8_t carrier, bool enabled) const
  {
    static const char * labels[] = {"NO", "FLT", "FIX"};
    const int gap = 6, seg_w = (rect.width - gap * 2) / 3;
    const uint8_t clamped = std::min<uint8_t>(carrier, 2);
    for (int i = 0; i < 3; ++i) {
      const cv::Rect seg(rect.x + i * (seg_w + gap), rect.y, seg_w, rect.height);
      const bool active = enabled && (int)clamped == i;
      cv::rectangle(img, seg, active ? carrierColor(clamped) : cv::Scalar(58, 64, 72), cv::FILLED);
      cv::rectangle(img, seg, active ? cv::Scalar(255,255,255) : cv::Scalar(92, 100, 112), 1);
      cv::putText(img, labels[i], {seg.x + 8, seg.y + seg.height - 7},
        cv::FONT_HERSHEY_DUPLEX, 0.42,
        active ? cv::Scalar(12, 14, 18) : kMuted, 1, cv::LINE_AA);
    }
  }

  void drawGps1Card(cv::Mat & img, const cv::Rect & rect) const
  {
    cv::rectangle(img, rect, kPanel, cv::FILLED);
    cv::rectangle(img, rect, kBorder, 2);
    cv::rectangle(img, {rect.x, rect.y, rect.width, 8}, kGps1, cv::FILLED);

    cv::putText(img, gps1_.label, {rect.x + 22, rect.y + 48},
      cv::FONT_HERSHEY_DUPLEX, 1.0, kText, 2, cv::LINE_AA);
    drawPill(img, {rect.x + rect.width - 150, rect.y + 54},
      stateText(gps1_), stateColor(gps1_));

    const bool fix_fresh = gps1_.has_fix && isFresh(gps1_.last_fix_time);
    const bool pvt_fresh = gps1_.has_pvt && isFresh(gps1_.last_pvt_time);
    const double acc_m   = pvt_fresh && gps1_.pvt_h_acc_m > 0.0 ? gps1_.pvt_h_acc_m
                                                                  : gps1_.horiz_acc_sigma_m;
    char buf[256];

    const int lx = rect.x + 26, rx = rect.x + rect.width / 2 + 10;
    int y = rect.y + 108;
    const int dy = 48;

    std::snprintf(buf, sizeof(buf), "Link: %s", online(gps1_) ? "ONLINE" : "STALE");
    cv::putText(img, buf, {lx, y}, cv::FONT_HERSHEY_SIMPLEX, 0.72, kText, 1, cv::LINE_AA);

    std::snprintf(buf, sizeof(buf), "RTK: %s", carrierText(gps1_).c_str());
    cv::putText(img, buf, {lx, y + dy}, cv::FONT_HERSHEY_SIMPLEX, 0.72, kText, 1, cv::LINE_AA);
    drawCarrierBar(img, {lx + 96, y + dy - 18, 180, 24}, gps1_.carr_soln_status, online(gps1_));

    std::snprintf(buf, sizeof(buf), "RTCM: %s", gps1_.diff_soln ? "ON" : "OFF");
    cv::putText(img, buf, {lx, y + dy * 2}, cv::FONT_HERSHEY_SIMPLEX, 0.72, kText, 1, cv::LINE_AA);

    std::snprintf(buf, sizeof(buf), "Fix: %s",
      fix_fresh ? navStatusText(gps1_.last_fix.status.status).c_str() : "WAIT");
    cv::putText(img, buf, {rx, y}, cv::FONT_HERSHEY_SIMPLEX, 0.72, kText, 1, cv::LINE_AA);

    std::snprintf(buf, sizeof(buf), "SV: %u", pvt_fresh ? (unsigned)gps1_.num_sv : 0u);
    cv::putText(img, buf, {rx, y + dy}, cv::FONT_HERSHEY_SIMPLEX, 0.72, kText, 1, cv::LINE_AA);

    if (acc_m > 0.0) {
      std::snprintf(buf, sizeof(buf), "H Acc: %.3f m", acc_m);
    } else {
      std::snprintf(buf, sizeof(buf), "H Acc: N/A");
    }
    cv::putText(img, buf, {rx, y + dy * 2}, cv::FONT_HERSHEY_SIMPLEX, 0.72,
      acc_m > 0.0 ? kGps1 : kMuted, acc_m > 0.0 ? 2 : 1, cv::LINE_AA);

    std::snprintf(buf, sizeof(buf), "Age Fix/PVT: %s / %s",
      ageText(gps1_.last_fix_time).c_str(), ageText(gps1_.last_pvt_time).c_str());
    cv::putText(img, buf, {lx, y + dy * 3 + 10}, cv::FONT_HERSHEY_SIMPLEX, 0.62, kMuted, 1, cv::LINE_AA);

    cv::rectangle(img, {rect.x + 22, rect.y + 320, rect.width - 44, 1},
      cv::Scalar(62, 68, 76), cv::FILLED);

    if (fix_fresh) {
      std::snprintf(buf, sizeof(buf), "Lat  %.8f", gps1_.last_fix.latitude);
      cv::putText(img, buf, {rect.x + 24, rect.y + 358}, cv::FONT_HERSHEY_SIMPLEX, 0.68, kText, 1, cv::LINE_AA);
      std::snprintf(buf, sizeof(buf), "Lon  %.8f", gps1_.last_fix.longitude);
      cv::putText(img, buf, {rect.x + 24, rect.y + 400}, cv::FONT_HERSHEY_SIMPLEX, 0.68, kText, 1, cv::LINE_AA);
      std::snprintf(buf, sizeof(buf), "Alt  %.2f m", gps1_.last_fix.altitude);
      cv::putText(img, buf, {rect.x + 24, rect.y + 442}, cv::FONT_HERSHEY_SIMPLEX, 0.68, kText, 1, cv::LINE_AA);
    } else {
      cv::putText(img, "Waiting for NavSatFix...", {rect.x + 24, rect.y + 380},
        cv::FONT_HERSHEY_SIMPLEX, 0.68, kMuted, 1, cv::LINE_AA);
    }

    std::snprintf(buf, sizeof(buf), "Topic: %s", gps1_.fix_topic.c_str());
    cv::putText(img, buf, {rect.x + 24, rect.y + rect.height - 16},
      cv::FONT_HERSHEY_SIMPLEX, 0.52, kMuted, 1, cv::LINE_AA);
  }

  void drawCompass(cv::Mat & img, const cv::Point & center, int radius) const
  {
    cv::circle(img, center, radius,      cv::Scalar(86, 92, 100), 2, cv::LINE_AA);
    cv::circle(img, center, radius - 28, cv::Scalar(64, 70, 78),  1, cv::LINE_AA);

    for (int deg = 0; deg < 360; deg += 15) {
      const double rad = (deg - 90.0) * kPi / 180.0;
      const int outer = radius, inner = (deg % 45 == 0) ? radius - 20 : radius - 10;
      cv::line(img,
        {center.x + (int)(std::cos(rad) * inner), center.y + (int)(std::sin(rad) * inner)},
        {center.x + (int)(std::cos(rad) * outer), center.y + (int)(std::sin(rad) * outer)},
        cv::Scalar(118, 124, 132), 1, cv::LINE_AA);
    }
    cv::putText(img, "N", {center.x - 12, center.y - radius - 12}, cv::FONT_HERSHEY_DUPLEX, 0.9, kText, 2, cv::LINE_AA);
    cv::putText(img, "E", {center.x + radius + 14, center.y + 8},  cv::FONT_HERSHEY_DUPLEX, 0.9, kText, 2, cv::LINE_AA);
    cv::putText(img, "S", {center.x - 10, center.y + radius + 32}, cv::FONT_HERSHEY_DUPLEX, 0.9, kText, 2, cv::LINE_AA);
    cv::putText(img, "W", {center.x - radius - 34, center.y + 8},  cv::FONT_HERSHEY_DUPLEX, 0.9, kText, 2, cv::LINE_AA);

    const bool fresh = has_heading_ && isFresh(last_heading_time_);
    if (!fresh) {
      cv::putText(img, "WAIT", {center.x - 52, center.y + 12},
        cv::FONT_HERSHEY_DUPLEX, 1.2, kDanger, 2, cv::LINE_AA);
      return;
    }
    // ENU → 나침반 화면 변환: rad = -enu_deg * PI/180
    // (ENU 0°=East→오른쪽, 90°=North→위, CCW→CW 화면 보정)
    // GPS 헤딩 바늘 (노란색) - 이동 중에만 표시
    const bool gps_fresh = has_gps_vel_ && isFresh(last_gps_vel_time_);
    if (gps_fresh) {
      const double gps_rad = -gps_heading_deg_ * kPi / 180.0;
      const cv::Point gtip( center.x + (int)(std::cos(gps_rad) * (radius - 18)),
                            center.y + (int)(std::sin(gps_rad) * (radius - 18)));
      const cv::Point gtail(center.x - (int)(std::cos(gps_rad) * (radius - 64)),
                            center.y - (int)(std::sin(gps_rad) * (radius - 64)));
      cv::arrowedLine(img, gtail, gtip, cv::Scalar(0, 220, 255), 3, cv::LINE_AA, 0, 0.20);
    }
    // ESKF 헤딩 바늘 (흰색)
    const double rad = -heading_deg_ * kPi / 180.0;
    const cv::Point tip( center.x + (int)(std::cos(rad) * (radius - 22)),
                         center.y + (int)(std::sin(rad) * (radius - 22)));
    const cv::Point tail(center.x - (int)(std::cos(rad) * (radius - 60)),
                         center.y - (int)(std::sin(rad) * (radius - 60)));
    cv::arrowedLine(img, tail, tip, cv::Scalar(250, 250, 250), 5, cv::LINE_AA, 0, 0.22);
    cv::circle(img, center, 10, cv::Scalar(250, 250, 250), cv::FILLED, cv::LINE_AA);
    // 범례
    cv::putText(img, "W:ESKF  Y:GPS", {center.x - radius, center.y + radius + 20},
      cv::FONT_HERSHEY_SIMPLEX, 0.52, kMuted, 1, cv::LINE_AA);
  }

  void drawEskfPanel(cv::Mat & img, const cv::Rect & rect) const
  {
    cv::rectangle(img, rect, kPanelAlt, cv::FILLED);
    cv::rectangle(img, rect, kBorder, 2);
    cv::putText(img, "ESKF HEADING", {rect.x + 26, rect.y + 46},
      cv::FONT_HERSHEY_DUPLEX, 1.0, kText, 2, cv::LINE_AA);

    const bool fresh = has_heading_ && isFresh(last_heading_time_);
    drawPill(img, {rect.x + rect.width - 160, rect.y + 52},
      fresh ? "ACTIVE" : "WAIT", fresh ? kGood : kDanger);

    char buf[256];
    if (fresh) {
      std::snprintf(buf, sizeof(buf), "%.2f deg", heading_deg_);
    } else {
      std::snprintf(buf, sizeof(buf), "--.-- deg");
    }
    cv::putText(img, buf, {rect.x + 34, rect.y + 106},
      cv::FONT_HERSHEY_DUPLEX, 1.25, fresh ? kText : kMuted, 2, cv::LINE_AA);
    cv::putText(img, fresh ? bearingText(heading_deg_) : "WAIT",
      {rect.x + 36, rect.y + 144},
      cv::FONT_HERSHEY_SIMPLEX, 0.78, fresh ? kWarn : kMuted, 2, cv::LINE_AA);

    // GPS 속도 기반 헤딩 비교
    {
      const bool gps_fresh = has_gps_vel_ && isFresh(last_gps_vel_time_);
      cv::Rect cmp(rect.x + 34, rect.y + 162, rect.width - 68, 108);
      cv::rectangle(img, cmp, cv::Scalar(40, 43, 50), cv::FILLED);
      cv::rectangle(img, cmp, cv::Scalar(80, 86, 94), 1);
      cv::putText(img, "GPS HDG (이동중 기준)", {cmp.x + 12, cmp.y + 28},
        cv::FONT_HERSHEY_SIMPLEX, 0.60, kMuted, 1, cv::LINE_AA);
      if (gps_fresh && fresh) {
        double diff = gps_heading_deg_ - heading_deg_;
        while (diff >  180.0) diff -= 360.0;
        while (diff < -180.0) diff += 360.0;
        std::snprintf(buf, sizeof(buf), "GPS  %+7.1f deg  (spd %.2f m/s)",
          gps_heading_deg_, gps_speed_mps_);
        cv::putText(img, buf, {cmp.x + 12, cmp.y + 60},
          cv::FONT_HERSHEY_SIMPLEX, 0.66, cv::Scalar(0, 220, 255), 1, cv::LINE_AA);
        std::snprintf(buf, sizeof(buf), "ESKF %+7.1f deg  diff=%+.1f deg",
          heading_deg_, diff);
        cv::putText(img, buf, {cmp.x + 12, cmp.y + 92},
          cv::FONT_HERSHEY_SIMPLEX, 0.66,
          std::abs(diff) < 15.0 ? kGood : kDanger, 2, cv::LINE_AA);
      } else if (!gps_fresh) {
        cv::putText(img, "0.3m/s 이상 이동하면 GPS 헤딩 표시", {cmp.x + 12, cmp.y + 68},
          cv::FONT_HERSHEY_SIMPLEX, 0.60, kMuted, 1, cv::LINE_AA);
      }
    }

    const cv::Point center(rect.x + rect.width / 2, rect.y + 380);
    drawCompass(img, center, 170);

    // covariance trace box
    cv::Rect cov_box(rect.x + 34, rect.y + 530, rect.width - 68, 76);
    cv::rectangle(img, cov_box, cv::Scalar(40, 43, 50), cv::FILLED);
    cv::rectangle(img, cov_box, cv::Scalar(80, 86, 94), 1);
    const bool cov_fresh = has_cov_ && isFresh(last_cov_time_);
    std::snprintf(buf, sizeof(buf), "Filter Cov Trace: %s",
      cov_fresh ? "" : "N/A");
    cv::putText(img, buf, {cov_box.x + 18, cov_box.y + 34},
      cv::FONT_HERSHEY_SIMPLEX, 0.68, kText, 1, cv::LINE_AA);
    if (cov_fresh) {
      std::snprintf(buf, sizeof(buf), "%.4f", cov_trace_);
      cv::putText(img, buf, {cov_box.x + 220, cov_box.y + 34},
        cv::FONT_HERSHEY_SIMPLEX, 0.74, cov_trace_ < 10.0 ? kGood : kWarn, 2, cv::LINE_AA);
    }
    cv::putText(img, heading_topic_, {cov_box.x + 18, cov_box.y + 64},
      cv::FONT_HERSHEY_SIMPLEX, 0.52, kMuted, 1, cv::LINE_AA);

    // checklist
    cv::Rect cl(rect.x + 34, rect.y + 628, rect.width - 68, 200);
    cv::rectangle(img, cl, kPanel, cv::FILLED);
    cv::rectangle(img, cl, kBorder, 1);
    cv::putText(img, "CHECKLIST", {cl.x + 18, cl.y + 36},
      cv::FONT_HERSHEY_DUPLEX, 0.82, kText, 2, cv::LINE_AA);
    cv::putText(img, "1. GPS1 must be ONLINE",
      {cl.x + 18, cl.y + 76},  cv::FONT_HERSHEY_SIMPLEX, 0.64, kText, 1, cv::LINE_AA);
    cv::putText(img, "2. GPS1 RTK: NONE -> FLOAT -> FIXED",
      {cl.x + 18, cl.y + 112}, cv::FONT_HERSHEY_SIMPLEX, 0.64, kText, 1, cv::LINE_AA);
    cv::putText(img, "3. ESKF heading must switch from WAIT to ACTIVE",
      {cl.x + 18, cl.y + 148}, cv::FONT_HERSHEY_SIMPLEX, 0.64, kText, 1, cv::LINE_AA);
    cv::putText(img, "4. If RTCM=OFF, check NTRIP node and /dev/henes_gps",
      {cl.x + 18, cl.y + 180}, cv::FONT_HERSHEY_SIMPLEX, 0.60, kWarn, 1, cv::LINE_AA);
  }

  void drawImuPanel(cv::Mat & img, const cv::Rect & rect) const
  {
    const bool fresh = has_imu_ && (now() - last_imu_time_).seconds() <= topic_timeout_;
    cv::rectangle(img, rect, kPanel, cv::FILLED);
    cv::rectangle(img, rect, kBorder, 2);
    cv::rectangle(img, {rect.x, rect.y, rect.width, 8}, cv::Scalar(80, 180, 230), cv::FILLED);
    cv::putText(img, "IMU", {rect.x + 22, rect.y + 46},
      cv::FONT_HERSHEY_DUPLEX, 1.0, kText, 2, cv::LINE_AA);
    drawPill(img, {rect.x + rect.width - 140, rect.y + 52},
      fresh ? "ONLINE" : "STALE", fresh ? kGood : kDanger);

    if (!fresh) {
      cv::putText(img, "Waiting for IMU data...", {rect.x + 30, rect.y + 110},
        cv::FONT_HERSHEY_SIMPLEX, 0.72, kMuted, 1, cv::LINE_AA);
      return;
    }

    char buf[128];
    const int c1 = rect.x + 26, c2 = rect.x + 280, c3 = rect.x + 540;
    const int y0 = rect.y + 80, dy = 46;

    cv::putText(img, "ATTITUDE",    {c1, y0}, cv::FONT_HERSHEY_SIMPLEX, 0.60, kMuted, 1, cv::LINE_AA);
    std::snprintf(buf, sizeof(buf), "Roll  %+7.2f deg", imu_roll_);
    cv::putText(img, buf, {c1, y0+dy},   cv::FONT_HERSHEY_SIMPLEX, 0.72, kText, 1, cv::LINE_AA);
    std::snprintf(buf, sizeof(buf), "Pitch %+7.2f deg", imu_pitch_);
    cv::putText(img, buf, {c1, y0+dy*2}, cv::FONT_HERSHEY_SIMPLEX, 0.72, kText, 1, cv::LINE_AA);
    std::snprintf(buf, sizeof(buf), "Yaw   %+7.2f deg", imu_yaw_);
    cv::putText(img, buf, {c1, y0+dy*3}, cv::FONT_HERSHEY_SIMPLEX, 0.72, kText, 1, cv::LINE_AA);

    cv::putText(img, "GYRO (deg/s)", {c2, y0}, cv::FONT_HERSHEY_SIMPLEX, 0.60, kMuted, 1, cv::LINE_AA);
    std::snprintf(buf, sizeof(buf), "X %+7.2f", imu_gx_);
    cv::putText(img, buf, {c2, y0+dy},   cv::FONT_HERSHEY_SIMPLEX, 0.72, kText, 1, cv::LINE_AA);
    std::snprintf(buf, sizeof(buf), "Y %+7.2f", imu_gy_);
    cv::putText(img, buf, {c2, y0+dy*2}, cv::FONT_HERSHEY_SIMPLEX, 0.72, kText, 1, cv::LINE_AA);
    std::snprintf(buf, sizeof(buf), "Z %+7.2f", imu_gz_);
    cv::putText(img, buf, {c2, y0+dy*3}, cv::FONT_HERSHEY_SIMPLEX, 0.72, kText, 1, cv::LINE_AA);

    cv::putText(img, "ACCEL (m/s²)", {c3, y0}, cv::FONT_HERSHEY_SIMPLEX, 0.60, kMuted, 1, cv::LINE_AA);
    std::snprintf(buf, sizeof(buf), "X %+7.3f", imu_ax_);
    cv::putText(img, buf, {c3, y0+dy},   cv::FONT_HERSHEY_SIMPLEX, 0.72, kText, 1, cv::LINE_AA);
    std::snprintf(buf, sizeof(buf), "Y %+7.3f", imu_ay_);
    cv::putText(img, buf, {c3, y0+dy*2}, cv::FONT_HERSHEY_SIMPLEX, 0.72, kText, 1, cv::LINE_AA);
    std::snprintf(buf, sizeof(buf), "Z %+7.3f", imu_az_);
    cv::putText(img, buf, {c3, y0+dy*3}, cv::FONT_HERSHEY_SIMPLEX, 0.72, kText, 1, cv::LINE_AA);
  }

  void drawFrame()
  {
    cv::Mat img(kWindowHeight, kWindowWidth, CV_8UC3, kBg);

    // header
    cv::rectangle(img, {0, 0, kWindowWidth, 82}, cv::Scalar(26, 29, 34), cv::FILLED);
    cv::putText(img, "HENES GPS1 + ESKF Monitor",
      {36, 52}, cv::FONT_HERSHEY_DUPLEX, 1.2, kText, 2, cv::LINE_AA);
    cv::putText(img, "GPS1 RTK  |  ESKF: position + heading  |  IMU",
      {38, 76}, cv::FONT_HERSHEY_SIMPLEX, 0.58, kMuted, 1, cv::LINE_AA);

    // GPS1 card (left)
    drawGps1Card(img, {34, 106, 700, 692});

    // ESKF heading panel (right)
    drawEskfPanel(img, {768, 106, 698, 848});

    // IMU panel (bottom)
    drawImuPanel(img, {34, 822, 700, 234});

    if (log_to_console_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "gps1=%s  heading=%.1f deg  cov=%.4f",
        stateText(gps1_).c_str(),
        has_heading_ ? heading_deg_ : 0.0,
        has_cov_ ? cov_trace_ : -1.0);
    }

    cv::imshow(window_name_, img);
    cv::waitKey(1);
  }

  // GPS1
  ReceiverView gps1_;

  // ESKF
  std::string heading_topic_;
  std::string cov_topic_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr heading_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr cov_sub_;
  bool   has_heading_{false};
  double heading_deg_{0.0};
  rclcpp::Time last_heading_time_{0, 0, RCL_ROS_TIME};
  bool   has_cov_{false};
  double cov_trace_{0.0};
  rclcpp::Time last_cov_time_{0, 0, RCL_ROS_TIME};

  // GPS velocity heading
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr gps_vel_sub_;
  bool   has_gps_vel_{false};
  double gps_heading_deg_{0.0};
  double gps_speed_mps_{0.0};
  rclcpp::Time last_gps_vel_time_{0, 0, RCL_ROS_TIME};

  // IMU
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  bool   has_imu_{false};
  double imu_roll_{0.0}, imu_pitch_{0.0}, imu_yaw_{0.0};
  double imu_gx_{0.0}, imu_gy_{0.0}, imu_gz_{0.0};
  double imu_ax_{0.0}, imu_ay_{0.0}, imu_az_{0.0};
  rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};

  rclcpp::TimerBase::SharedPtr timer_;
  std::string window_name_;
  double  topic_timeout_{1.5};
  bool    log_to_console_{true};
};

}  // namespace jeju_mpc

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<jeju_mpc::GpsRtkGuiNode>());
  rclcpp::shutdown();
  return 0;
}
