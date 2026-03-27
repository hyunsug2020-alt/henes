// ============================================================
// gnss_navsat_bridge_node.cpp
//
// UBXNavPVT → sensor_msgs/NavSatFix 변환 노드
// ublox_dgnss_node 가 NavSatFix를 발행하지 않으므로
// 이 노드가 ubx_nav_pvt 를 받아 fix 로 재발행함
//
// 파라미터:
//   input_topic  (default: /gnss_left/ubx_nav_pvt)
//   output_topic (default: /gnss_left/fix)
// ============================================================
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <ublox_ubx_msgs/msg/ubx_nav_pvt.hpp>

class GnssNavSatBridge : public rclcpp::Node
{
public:
    GnssNavSatBridge()
    : Node("gnss_navsat_bridge_node")
    {
        this->declare_parameter("input_topic",  "/gnss_left/ubx_nav_pvt");
        this->declare_parameter("output_topic", "/gnss_left/fix");

        const auto in  = this->get_parameter("input_topic").as_string();
        const auto out = this->get_parameter("output_topic").as_string();

        pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(out, 10);

        sub_ = this->create_subscription<ublox_ubx_msgs::msg::UBXNavPVT>(
            in, 10,
            [this](const ublox_ubx_msgs::msg::UBXNavPVT::SharedPtr msg) {
                convert(msg);
            });

        RCLCPP_INFO(this->get_logger(),
            "NavSat bridge: %s → %s", in.c_str(), out.c_str());
    }

private:
    void convert(const ublox_ubx_msgs::msg::UBXNavPVT::SharedPtr msg)
    {
        sensor_msgs::msg::NavSatFix fix;
        fix.header = msg->header;

        // ── NavSatStatus ─────────────────────────────────────
        auto & st = fix.status;
        if (!msg->gnss_fix_ok) {
            st.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        } else if (msg->carr_soln.status >= 1) {
            // RTK float 또는 RTK fixed → GBAS (differential)
            st.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
        } else if (msg->diff_soln) {
            st.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
        } else {
            st.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        }
        st.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS |
                     sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO |
                     sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS |
                     sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS;

        // ── 위치 ─────────────────────────────────────────────
        fix.latitude  = msg->lat  * 1e-7;   // deg * 1e-7 → deg
        fix.longitude = msg->lon  * 1e-7;
        fix.altitude  = msg->height * 1e-3;  // mm → m

        // ── 공분산 (수평 / 수직) ──────────────────────────────
        const double h_m  = msg->h_acc * 1e-3;  // mm → m
        const double v_m  = msg->v_acc * 1e-3;
        const double h_var = h_m * h_m;
        const double v_var = v_m * v_m;
        fix.position_covariance[0] = h_var;  // East
        fix.position_covariance[4] = h_var;  // North
        fix.position_covariance[8] = v_var;  // Up
        fix.position_covariance_type =
            sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

        pub_->publish(fix);
    }

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_;
    rclcpp::Subscription<ublox_ubx_msgs::msg::UBXNavPVT>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GnssNavSatBridge>());
    rclcpp::shutdown();
    return 0;
}
