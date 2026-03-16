#ifndef COMMUNICATION_MANAGER__COMMUNICATION_BASE_HPP_
#define COMMUNICATION_MANAGER__COMMUNICATION_BASE_HPP_

#include <chrono>
#include <iomanip>
#include <sstream>
#include <utility>
#include <array>

#include <rclcpp/rclcpp.hpp>
#include <domain_bridge/domain_bridge.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::chrono_literals;
using real = double;

class CommunicationBase
{
public:
    virtual ~CommunicationBase() = default;

    virtual void activate() = 0;
    virtual void deactivate() = 0;
    bool is_active() const { return active_; }

    void vec_topics_init()
    {
        RCLCPP_INFO(node_->get_logger(), "Preparing explicit SIM<->CAV/HV bridges...");
        rclcpp::sleep_for(std::chrono::seconds(1));
    }

    void pose_communication()
    {
        const std::array<int, 4> cav_ids{{1, 2, 3, 4}};
        const std::array<int, 2> hv_ids{{19, 20}};

        for (int cav_id : cav_ids) {
            std::ostringstream id_ss;
            id_ss << std::setw(2) << std::setfill('0') << cav_id;
            const std::string id = id_ss.str();

            domain_bridge::DomainBridgeOptions bridge_opts;
            bridge_opts.name("sim_cav_" + id);
            auto db_node = std::make_shared<domain_bridge::DomainBridge>(bridge_opts);

            domain_bridge::TopicBridgeOptions pose_remap;
            pose_remap.remap_name("/cav" + id + "/ego_pose");
            db_node->bridge_topic(
                "/sim/cav" + id + "/pose",
                "geometry_msgs/msg/PoseStamped",
                100, cav_id, pose_remap);

            domain_bridge::TopicBridgeOptions accel_remap;
            accel_remap.remap_name("/sim/cav" + id + "/accel");
            db_node->bridge_topic(
                "/cav" + id + "/accel_cmd",
                "geometry_msgs/msg/Accel",
                cav_id, 100, accel_remap);

            // Legacy aliases in SIM domain (100) for existing SIM/RViz consumers.
            const int rviz_slot = cav_id - 1;

            domain_bridge::TopicBridgeOptions ego_pose_legacy_remap;
            ego_pose_legacy_remap.remap_name("/CAV_" + id);
            db_node->bridge_topic(
                "/cav" + id + "/ego_pose",
                "geometry_msgs/msg/PoseStamped",
                cav_id, 100, ego_pose_legacy_remap);

            domain_bridge::TopicBridgeOptions global_path_legacy_remap;
            global_path_legacy_remap.remap_name("/viz/slot" + std::to_string(rviz_slot) + "/global_path");
            db_node->bridge_topic(
                "/cav" + id + "/global_path",
                "nav_msgs/msg/Path",
                cav_id, 100, global_path_legacy_remap);

            domain_bridge::TopicBridgeOptions local_path_legacy_remap;
            local_path_legacy_remap.remap_name("/viz/slot" + std::to_string(rviz_slot) + "/local_path");
            db_node->bridge_topic(
                "/cav" + id + "/local_path",
                "nav_msgs/msg/Path",
                cav_id, 100, local_path_legacy_remap);

            domain_bridge::TopicBridgeOptions marker_legacy_remap;
            marker_legacy_remap.remap_name("/viz/slot" + std::to_string(rviz_slot) + "/car_marker");
            db_node->bridge_topic(
                "/cav" + id + "/car_marker",
                "visualization_msgs/msg/Marker",
                cav_id, 100, marker_legacy_remap);

            domain_bridge::TopicBridgeOptions marker_array_legacy_remap;
            marker_array_legacy_remap.remap_name("/viz/slot" + std::to_string(rviz_slot) + "/car_marker_array");
            db_node->bridge_topic(
                "/cav" + id + "/car_marker_array",
                "visualization_msgs/msg/MarkerArray",
                cav_id, 100, marker_array_legacy_remap);

            domain_bridge::TopicBridgeOptions mpc_pred_legacy_remap;
            mpc_pred_legacy_remap.remap_name("/mpc_pred_cav" + id);
            db_node->bridge_topic(
                "/cav" + id + "/mpc_predicted_path",
                "nav_msgs/msg/Path",
                cav_id, 100, mpc_pred_legacy_remap);

            domain_bridge::TopicBridgeOptions accel_cmd_legacy_remap;
            accel_cmd_legacy_remap.remap_name("/CAV_" + id + "_accel");
            db_node->bridge_topic(
                "/cav" + id + "/accel_cmd",
                "geometry_msgs/msg/Accel",
                cav_id, 100, accel_cmd_legacy_remap);

            domain_bridge::TopicBridgeOptions accel_raw_legacy_remap;
            accel_raw_legacy_remap.remap_name("/CAV_" + id + "_accel_raw");
            db_node->bridge_topic(
                "/cav" + id + "/accel_raw",
                "geometry_msgs/msg/Accel",
                cav_id, 100, accel_raw_legacy_remap);

            active_bridges[{100, cav_id}] = db_node;
            db_node->add_to_executor(executor);
        }

        for (int hv_id : hv_ids) {
            std::ostringstream id_ss;
            id_ss << std::setw(2) << std::setfill('0') << hv_id;
            const std::string id = id_ss.str();

            domain_bridge::DomainBridgeOptions bridge_opts;
            bridge_opts.name("sim_hv_" + id);
            auto db_node = std::make_shared<domain_bridge::DomainBridge>(bridge_opts);

            domain_bridge::TopicBridgeOptions pose_remap;
            pose_remap.remap_name("/hv" + id + "/pose");
            db_node->bridge_topic(
                "/sim/hv" + id + "/pose",
                "geometry_msgs/msg/PoseStamped",
                100, hv_id, pose_remap);

            active_bridges[{100, hv_id}] = db_node;
            db_node->add_to_executor(executor);
        }
    }

protected:
    bool active_ = false;
    rclcpp::Node::SharedPtr node_;

    std::map<std::pair<int, int>, std::shared_ptr<domain_bridge::DomainBridge>> active_bridges;
    rclcpp::executors::SingleThreadedExecutor executor;
};

#endif  // COMMUNICATION_MANAGER__COMMUNICATION_BASE_HPP_
