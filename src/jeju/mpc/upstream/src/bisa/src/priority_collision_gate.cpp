#include <rclcpp/rclcpp.hpp>

#include "bisa/msg/zone_reservation.hpp"
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <optional>

namespace bisa {

class PriorityCollisionGate : public rclcpp::Node {
 public:
  PriorityCollisionGate() : Node("priority_collision_gate") {
    this->declare_parameter("cav_id", 1);
    this->declare_parameter("active_cav_ids", std::vector<int64_t>{1, 2, 3, 4});
    this->declare_parameter("overlap_distance", 0.58);
    this->declare_parameter("release_overlap_distance", 0.62);
    this->declare_parameter("stop_lookahead_index", 6);
    this->declare_parameter("hard_stop_lookahead_index", 9);
    this->declare_parameter("slow_ratio", 0.35);
    this->declare_parameter("escape_force_slow_index", 5);
    this->declare_parameter("eta_min_speed", 0.15);
    this->declare_parameter("zone_clearance_sec", 0.25);
    this->declare_parameter("reservation_ttl_sec", 0.60);
    this->declare_parameter("reservation_overlap_margin_sec", 0.02);
    this->declare_parameter("heading_conflict_min_deg", 12.0);
    this->declare_parameter("near_pose_stop_distance", 0.36);
    this->declare_parameter("near_pose_release_distance", 0.46);
    this->declare_parameter("near_pose_stationary_speed_mps", 0.02);
    this->declare_parameter("near_pose_stationary_ignore_sec", 1.2);
    this->declare_parameter("occupied_pose_stop_distance", 0.52);
    this->declare_parameter("geometric_stop_pose_distance", 1.10);
    this->declare_parameter("immediate_pose_stop_distance", 0.78);
    this->declare_parameter("immediate_pose_release_distance", 0.92);
    this->declare_parameter("immediate_overlap_distance", 0.55);
    this->declare_parameter("emergency_pose_stop_distance", 0.70);
    this->declare_parameter("emergency_pose_release_distance", 0.85);
    this->declare_parameter("same_priority_ids", std::vector<int64_t>{1, 2, 3, 4});
    this->declare_parameter("same_priority_stop_distance", 1.20);
    this->declare_parameter("same_priority_release_distance", 1.40);
    this->declare_parameter("same_priority_eta_margin_sec", 0.20);
    this->declare_parameter("preentry_stop_min_hit_index", 3);
    this->declare_parameter("same_priority_latch_hit_index", 18);
    this->declare_parameter("same_priority_latch_release_hit_index", 28);
    this->declare_parameter("same_priority_latch_timeout_sec", 1.5);
    this->declare_parameter("emergency_hard_stop_distance", 0.38);
    this->declare_parameter("ttc_mode_enabled", true);
    this->declare_parameter("ttc_threshold_sec", 3.0);
    this->declare_parameter("ttc_tie_margin_sec", 0.15);
    this->declare_parameter("ttc_overlap_distance", 0.35);
    this->declare_parameter("ttc_conflict_distance", 0.35);
    this->declare_parameter("ttc_max_hit_index", 24);
    this->declare_parameter("ttc_physical_guard_distance", 0.80);
    this->declare_parameter("ttc_emergency_stop_distance", 0.45);
    this->declare_parameter("ttc_absolute_emergency_stop_distance", 0.40);
    this->declare_parameter("ttc_startup_hold_enabled", true);
    this->declare_parameter("ttc_startup_hold_sec", 1.0);
    this->declare_parameter("ttc_min_speed", 0.05);
    this->declare_parameter("ttc_default_other_speed", 0.35);
    this->declare_parameter("yield_speed_ratio", 0.55);
    this->declare_parameter("pass_speed_boost", 1.15);
    this->declare_parameter("max_velocity_cap", 0.70);
    this->declare_parameter("offline_speed_cap_enabled", true);
    this->declare_parameter("offline_speed_cap_timeout_sec", 1.0);
    this->declare_parameter("rear_same_direction_ignore_distance", 0.22);
    this->declare_parameter("rear_same_direction_heading_deg", 40.0);
    this->declare_parameter("rear_follow_slow_distance", 0.90);
    this->declare_parameter("rear_follow_slow_release_distance", 1.10);
    this->declare_parameter("rear_follow_stop_distance", 0.55);
    this->declare_parameter("rear_follow_stop_release_distance", 0.72);
    this->declare_parameter("rear_follow_lateral_distance", 0.28);

    cav_id_ = this->get_parameter("cav_id").as_int();
    overlap_distance_ = this->get_parameter("overlap_distance").as_double();
    release_overlap_distance_ = this->get_parameter("release_overlap_distance").as_double();
    stop_lookahead_index_ = this->get_parameter("stop_lookahead_index").as_int();
    hard_stop_lookahead_index_ = this->get_parameter("hard_stop_lookahead_index").as_int();
    slow_ratio_ = this->get_parameter("slow_ratio").as_double();
    slow_ratio_ = std::clamp(slow_ratio_, 0.0, 1.0);
    escape_force_slow_index_ = this->get_parameter("escape_force_slow_index").as_int();
    eta_min_speed_ = this->get_parameter("eta_min_speed").as_double();
    zone_clearance_sec_ = this->get_parameter("zone_clearance_sec").as_double();
    reservation_ttl_sec_ = this->get_parameter("reservation_ttl_sec").as_double();
    reservation_overlap_margin_sec_ = this->get_parameter("reservation_overlap_margin_sec").as_double();
    heading_conflict_min_deg_ = this->get_parameter("heading_conflict_min_deg").as_double();
    near_pose_stop_distance_ = this->get_parameter("near_pose_stop_distance").as_double();
    near_pose_release_distance_ = this->get_parameter("near_pose_release_distance").as_double();
    near_pose_stationary_speed_mps_ =
        this->get_parameter("near_pose_stationary_speed_mps").as_double();
    near_pose_stationary_ignore_sec_ =
        this->get_parameter("near_pose_stationary_ignore_sec").as_double();
    occupied_pose_stop_distance_ = this->get_parameter("occupied_pose_stop_distance").as_double();
    geometric_stop_pose_distance_ = this->get_parameter("geometric_stop_pose_distance").as_double();
    immediate_pose_stop_distance_ = this->get_parameter("immediate_pose_stop_distance").as_double();
    immediate_pose_release_distance_ = this->get_parameter("immediate_pose_release_distance").as_double();
    immediate_overlap_distance_ = this->get_parameter("immediate_overlap_distance").as_double();
    emergency_pose_stop_distance_ = this->get_parameter("emergency_pose_stop_distance").as_double();
    emergency_pose_release_distance_ = this->get_parameter("emergency_pose_release_distance").as_double();
    const auto same_priority_ids_param =
        this->get_parameter("same_priority_ids").as_integer_array();
    same_priority_ids_.clear();
    for (const auto id_raw : same_priority_ids_param) {
      same_priority_ids_.push_back(static_cast<int>(id_raw));
    }
    same_priority_stop_distance_ = this->get_parameter("same_priority_stop_distance").as_double();
    same_priority_release_distance_ =
        this->get_parameter("same_priority_release_distance").as_double();
    same_priority_eta_margin_sec_ =
        std::max(0.01, this->get_parameter("same_priority_eta_margin_sec").as_double());
    preentry_stop_min_hit_index_ =
        std::max<int>(0, static_cast<int>(this->get_parameter("preentry_stop_min_hit_index").as_int()));
    same_priority_latch_hit_index_ =
        std::max(0, static_cast<int>(this->get_parameter("same_priority_latch_hit_index").as_int()));
    same_priority_latch_release_hit_index_ = std::max(
        same_priority_latch_hit_index_,
        static_cast<int>(this->get_parameter("same_priority_latch_release_hit_index").as_int()));
    same_priority_latch_timeout_sec_ =
        std::max(0.1, this->get_parameter("same_priority_latch_timeout_sec").as_double());
    emergency_hard_stop_distance_ = this->get_parameter("emergency_hard_stop_distance").as_double();
    ttc_mode_enabled_ = this->get_parameter("ttc_mode_enabled").as_bool();
    ttc_threshold_sec_ = this->get_parameter("ttc_threshold_sec").as_double();
    ttc_tie_margin_sec_ = this->get_parameter("ttc_tie_margin_sec").as_double();
    ttc_overlap_distance_ = this->get_parameter("ttc_overlap_distance").as_double();
    ttc_conflict_distance_ = this->get_parameter("ttc_conflict_distance").as_double();
    ttc_max_hit_index_ = this->get_parameter("ttc_max_hit_index").as_int();
    ttc_physical_guard_distance_ = this->get_parameter("ttc_physical_guard_distance").as_double();
    ttc_emergency_stop_distance_ = this->get_parameter("ttc_emergency_stop_distance").as_double();
    ttc_absolute_emergency_stop_distance_ =
        this->get_parameter("ttc_absolute_emergency_stop_distance").as_double();
    ttc_startup_hold_enabled_ = this->get_parameter("ttc_startup_hold_enabled").as_bool();
    ttc_startup_hold_sec_ = this->get_parameter("ttc_startup_hold_sec").as_double();
    ttc_min_speed_ = this->get_parameter("ttc_min_speed").as_double();
    ttc_default_other_speed_ = this->get_parameter("ttc_default_other_speed").as_double();
    yield_speed_ratio_ = this->get_parameter("yield_speed_ratio").as_double();
    pass_speed_boost_ = this->get_parameter("pass_speed_boost").as_double();
    max_velocity_cap_ = this->get_parameter("max_velocity_cap").as_double();
    offline_speed_cap_enabled_ = this->get_parameter("offline_speed_cap_enabled").as_bool();
    offline_speed_cap_timeout_sec_ =
        std::max(0.1, this->get_parameter("offline_speed_cap_timeout_sec").as_double());
    rear_same_direction_ignore_distance_ = std::max(
        0.05, this->get_parameter("rear_same_direction_ignore_distance").as_double());
    rear_same_direction_heading_rad_ =
        std::clamp(this->get_parameter("rear_same_direction_heading_deg").as_double(), 5.0, 90.0) *
        M_PI / 180.0;
    rear_follow_lateral_distance_ =
        std::max(0.05, this->get_parameter("rear_follow_lateral_distance").as_double());
    rear_follow_stop_distance_ =
        std::max(0.10, this->get_parameter("rear_follow_stop_distance").as_double());
    rear_follow_stop_release_distance_ = std::max(
        rear_follow_stop_distance_ + 0.05,
        this->get_parameter("rear_follow_stop_release_distance").as_double());
    rear_follow_slow_distance_ = std::max(
        rear_follow_stop_distance_,
        this->get_parameter("rear_follow_slow_distance").as_double());
    rear_follow_slow_release_distance_ = std::max(
        rear_follow_slow_distance_ + 0.05,
        this->get_parameter("rear_follow_slow_release_distance").as_double());
    yield_speed_ratio_ = std::clamp(yield_speed_ratio_, 0.0, 1.0);
    pass_speed_boost_ = std::max(pass_speed_boost_, 1.0);
    max_velocity_cap_ = std::max(max_velocity_cap_, 0.01);
    ttc_min_speed_ = std::max(ttc_min_speed_, 0.01);
    eta_min_speed_ = std::max(eta_min_speed_, ttc_min_speed_);
    ttc_default_other_speed_ = std::max(ttc_default_other_speed_, ttc_min_speed_);
    near_pose_stationary_speed_mps_ = std::max(near_pose_stationary_speed_mps_, 0.0);
    near_pose_stationary_ignore_sec_ = std::max(near_pose_stationary_ignore_sec_, 0.0);
    ttc_max_hit_index_ = std::max(ttc_max_hit_index_, 1);
    ttc_physical_guard_distance_ = std::max(ttc_physical_guard_distance_, 0.01);
    ttc_emergency_stop_distance_ =
        std::clamp(ttc_emergency_stop_distance_, 0.01, ttc_physical_guard_distance_);
    ttc_absolute_emergency_stop_distance_ =
        std::max(ttc_absolute_emergency_stop_distance_, 0.01);
    ttc_startup_hold_sec_ = std::max(ttc_startup_hold_sec_, 0.0);

    const auto active_ids = this->get_parameter("active_cav_ids").as_integer_array();
    expected_peer_count_ = std::max(0, static_cast<int>(active_ids.size()) - 1);
    node_start_time_ = this->now();

    accel_sub_ = this->create_subscription<geometry_msgs::msg::Accel>(
        "/Accel_raw", rclcpp::SensorDataQoS(),
        std::bind(&PriorityCollisionGate::accelCallback, this, std::placeholders::_1));

    planned_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/planned_path", 10,
        std::bind(&PriorityCollisionGate::plannedPathCallback, this, std::placeholders::_1));
    ego_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/Ego_pose", rclcpp::SensorDataQoS(),
        std::bind(&PriorityCollisionGate::egoPoseCallback, this, std::placeholders::_1));

    reservation_sub_ = this->create_subscription<bisa::msg::ZoneReservation>(
        "/priority_reservation", 10,
        std::bind(&PriorityCollisionGate::reservationCallback, this, std::placeholders::_1));
    offline_cap_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/offline_speed_cap", 10,
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
          if (!msg) return;
          offline_speed_cap_mps_ = std::max(0.0, msg->data);
          offline_cap_rx_time_ = this->now();
          has_offline_cap_ = true;
        });

    accel_pub_ = this->create_publisher<geometry_msgs::msg::Accel>("/Accel", 10);
    reservation_pub_ = this->create_publisher<bisa::msg::ZoneReservation>("/priority_reservation", 10);
    const std::string gate_status_topic =
        "/cav" + idTo2Digit(cav_id_) + "/priority_gate_status";
    gate_status_pub_ = this->create_publisher<std_msgs::msg::String>(gate_status_topic, 10);
    RCLCPP_INFO(this->get_logger(), "CAV%d publishes gate status: %s",
                cav_id_, gate_status_topic.c_str());

    for (const auto id_raw : active_ids) {
      const int other_id = static_cast<int>(id_raw);
      if (other_id == cav_id_) continue;

      const std::string topic =
          "/cav" + idTo2Digit(other_id) + "/local_path";

      auto sub = this->create_subscription<nav_msgs::msg::Path>(
          topic, 10,
          [this, other_id](const nav_msgs::msg::Path::SharedPtr msg) {
            other_paths_[other_id] = *msg;
          });
      other_path_subs_.push_back(sub);
      RCLCPP_INFO(this->get_logger(), "CAV%d listens path: %s", cav_id_, topic.c_str());

      const std::string pose_topic = "/CAV_" + idTo2Digit(other_id);
      auto pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          pose_topic, rclcpp::SensorDataQoS(),
          [this, other_id](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            updateOtherPose(other_id, *msg);
          });
      other_pose_subs_.push_back(pose_sub);
    }
  }

 private:
  enum class GateMode {
    kPass = 0,
    kSlow = 1,
    kStop = 2,
  };

  struct OverlapInfo {
    bool valid{false};
    int blocking_id{-1};
    size_t first_idx{std::numeric_limits<size_t>::max()};
    size_t last_idx{0};
  };

  struct TtcDecision {
    std::string role{"free"};  // free / pass / yield
    int blocking_id{-1};
    size_t my_hit_idx{std::numeric_limits<size_t>::max()};
    size_t other_hit_idx{std::numeric_limits<size_t>::max()};
    double my_ttc{-1.0};
    double other_ttc{-1.0};
    double pose_dist{-1.0};
    bool other_in_zone{false};
  };

  static std::string idTo2Digit(int id) {
    if (id < 10) return "0" + std::to_string(id);
    return std::to_string(id);
  }

  static std::string buildZoneId(int a, int b) {
    const int lo = std::min(a, b);
    const int hi = std::max(a, b);
    return "pair_" + idTo2Digit(lo) + "_" + idTo2Digit(hi);
  }

  static double poseDistance(const geometry_msgs::msg::PoseStamped &a,
                             const geometry_msgs::msg::PoseStamped &b) {
    const double dx = a.pose.position.x - b.pose.position.x;
    const double dy = a.pose.position.y - b.pose.position.y;
    return std::hypot(dx, dy);
  }

  static double wrapAngle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  static double headingAt(const nav_msgs::msg::Path &path, size_t idx) {
    if (path.poses.size() < 2) return 0.0;
    const size_t i0 = (idx == 0) ? 0 : idx - 1;
    const size_t i1 = std::min(idx + 1, path.poses.size() - 1);
    const auto &p0 = path.poses[i0].pose.position;
    const auto &p1 = path.poses[i1].pose.position;
    return std::atan2(p1.y - p0.y, p1.x - p0.x);
  }

  static double poseYawFromQuatOrPacked(const geometry_msgs::msg::Pose &pose) {
    const auto &q = pose.orientation;
    const double n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    const bool quat_component_range_ok =
        std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w) &&
        std::abs(q.x) <= 1.0 + 1e-3 && std::abs(q.y) <= 1.0 + 1e-3 &&
        std::abs(q.z) <= 1.0 + 1e-3 && std::abs(q.w) <= 1.0 + 1e-3;
    const bool use_quaternion =
        quat_component_range_ok && std::isfinite(n) && std::abs(n - 1.0) <= 0.05;

    if (!use_quaternion) {
      // Fallback for packed yaw conventions in pose.orientation.z.
      double yaw = q.z;
      if (!std::isfinite(yaw)) return std::numeric_limits<double>::quiet_NaN();
      if (std::abs(yaw) > 2.0 * M_PI + 0.5) {
        yaw = yaw * M_PI / 180.0;  // degrees -> radians
      } else if (yaw > M_PI && yaw <= 2.0 * M_PI + 0.5) {
        yaw -= 2.0 * M_PI;
      } else if (yaw < -M_PI && yaw >= -2.0 * M_PI - 0.5) {
        yaw += 2.0 * M_PI;
      }
      return wrapAngle(yaw);
    }

    const double x = q.x / n;
    const double y = q.y / n;
    const double z = q.z / n;
    const double w = q.w / n;
    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return wrapAngle(std::atan2(siny_cosp, cosy_cosp));
  }

  bool computeSameDirectionRelative(const geometry_msgs::msg::PoseStamped &other_pose,
                                    double *forward_proj_out,
                                    double *lateral_proj_out) const {
    if (!ego_pose_.has_value()) return false;
    const auto &ego = ego_pose_.value();
    const double ego_yaw = poseYawFromQuatOrPacked(ego.pose);
    const double other_yaw = poseYawFromQuatOrPacked(other_pose.pose);
    if (!std::isfinite(ego_yaw) || !std::isfinite(other_yaw)) return false;

    const double yaw_diff = std::abs(wrapAngle(ego_yaw - other_yaw));
    if (yaw_diff > rear_same_direction_heading_rad_) return false;

    const double dx = other_pose.pose.position.x - ego.pose.position.x;
    const double dy = other_pose.pose.position.y - ego.pose.position.y;
    const double forward_proj = dx * std::cos(ego_yaw) + dy * std::sin(ego_yaw);
    const double lateral_proj = -dx * std::sin(ego_yaw) + dy * std::cos(ego_yaw);
    if (forward_proj_out) *forward_proj_out = forward_proj;
    if (lateral_proj_out) *lateral_proj_out = lateral_proj;
    return true;
  }

  bool isRearSameDirectionVehicle(const geometry_msgs::msg::PoseStamped &other_pose) const {
    double forward_proj = 0.0;
    if (!computeSameDirectionRelative(other_pose, &forward_proj, nullptr)) return false;
    return forward_proj < -rear_same_direction_ignore_distance_;
  }

  bool isFrontSameDirectionVehicle(const geometry_msgs::msg::PoseStamped &other_pose) const {
    double forward_proj = 0.0;
    double lateral_proj = 0.0;
    if (!computeSameDirectionRelative(other_pose, &forward_proj, &lateral_proj)) return false;
    if (forward_proj <= rear_same_direction_ignore_distance_) return false;
    if (std::abs(lateral_proj) > rear_follow_lateral_distance_) return false;
    return true;
  }

  void plannedPathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    my_planned_path_ = *msg;
  }

  void egoPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    const auto now = this->now();
    if (ego_pose_.has_value() && ego_pose_rx_time_.nanoseconds() > 0) {
      const double dt = (now - ego_pose_rx_time_).seconds();
      if (dt > 1e-3) {
        ego_speed_estimate_ = poseDistance(*msg, ego_pose_.value()) / dt;
      }
    }
    ego_pose_rx_time_ = now;
    ego_pose_ = *msg;
  }

  void reservationCallback(const bisa::msg::ZoneReservation::SharedPtr msg) {
    const int other_id = static_cast<int>(msg->cav_id);
    if (other_id == cav_id_) return;
    higher_reservations_[other_id] = *msg;
    reservation_rx_time_[other_id] = this->now();
  }

  void updateOtherPose(int other_id, const geometry_msgs::msg::PoseStamped &pose) {
    const auto now = this->now();
    auto it_prev = other_poses_.find(other_id);
    auto it_prev_t = other_pose_rx_time_.find(other_id);
    if (it_prev != other_poses_.end() && it_prev_t != other_pose_rx_time_.end()) {
      const double dt = (now - it_prev_t->second).seconds();
      if (dt > 1e-3) {
        other_speed_estimates_[other_id] = poseDistance(pose, it_prev->second) / dt;
      }
    }
    other_poses_[other_id] = pose;
    other_pose_rx_time_[other_id] = now;
    auto it_speed = other_speed_estimates_.find(other_id);
    if (it_speed != other_speed_estimates_.end() &&
        it_speed->second <= near_pose_stationary_speed_mps_) {
      if (other_stationary_since_.find(other_id) == other_stationary_since_.end()) {
        other_stationary_since_[other_id] = now;
      }
    } else {
      other_stationary_since_.erase(other_id);
    }
  }

  static size_t closestPathIndex(const nav_msgs::msg::Path &path,
                                 const geometry_msgs::msg::PoseStamped &pose) {
    if (path.poses.empty()) return 0;
    size_t best_idx = 0;
    double best_d2 = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < path.poses.size(); ++i) {
      const double dx = path.poses[i].pose.position.x - pose.pose.position.x;
      const double dy = path.poses[i].pose.position.y - pose.pose.position.y;
      const double d2 = dx * dx + dy * dy;
      if (d2 < best_d2) {
        best_d2 = d2;
        best_idx = i;
      }
    }
    return best_idx;
  }

  static double pathDistanceBetween(const nav_msgs::msg::Path &path,
                                    size_t from_idx,
                                    size_t to_idx) {
    if (path.poses.size() < 2) return 0.0;
    const size_t from = std::min(from_idx, path.poses.size() - 1);
    const size_t to = std::min(to_idx, path.poses.size() - 1);
    if (to <= from) return 0.0;
    double sum = 0.0;
    for (size_t i = from + 1; i <= to; ++i) {
      const double dx = path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x;
      const double dy = path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y;
      sum += std::hypot(dx, dy);
    }
    return sum;
  }

  bool findConflictPairIndices(const nav_msgs::msg::Path &my_path,
                               const nav_msgs::msg::Path &other_path,
                               double threshold,
                               size_t *my_hit_idx,
                               size_t *other_hit_idx) const {
    if (my_path.poses.empty() || other_path.poses.empty()) return false;
    bool found = false;
    double best_score = std::numeric_limits<double>::infinity();
    size_t best_my = std::numeric_limits<size_t>::max();
    size_t best_other = std::numeric_limits<size_t>::max();
    const double heading_thresh_rad = heading_conflict_min_deg_ * M_PI / 180.0;

    for (size_t i = 0; i < my_path.poses.size(); ++i) {
      for (size_t j = 0; j < other_path.poses.size(); ++j) {
        const double d = poseDistance(my_path.poses[i], other_path.poses[j]);
        if (d > threshold) continue;

        // Keep only true crossing/merge conflicts.
        const double my_h = headingAt(my_path, i);
        const double ot_h = headingAt(other_path, j);
        const double dh = std::abs(wrapAngle(my_h - ot_h));
        if (dh < heading_thresh_rad) continue;

        const double score = static_cast<double>(i + j);
        if (!found || score < best_score) {
          found = true;
          best_score = score;
          best_my = i;
          best_other = j;
        }
      }
    }

    if (!found) return false;
    if (my_hit_idx) *my_hit_idx = best_my;
    if (other_hit_idx) *other_hit_idx = best_other;
    return true;
  }

  bool isPreentryStopWindow(size_t hit_idx, int max_hit_idx) const {
    const int idx = static_cast<int>(hit_idx);
    return idx >= preentry_stop_min_hit_index_ && idx <= max_hit_idx;
  }

  bool computePairEtaToConflict(int other_id,
                                size_t *my_hit_idx_out,
                                size_t *other_hit_idx_out,
                                double *my_eta_out,
                                double *other_eta_out) const {
    if (!ego_pose_.has_value()) return false;
    if (my_planned_path_.poses.empty()) return false;
    auto it_pose = other_poses_.find(other_id);
    if (it_pose == other_poses_.end()) return false;
    auto it_path = other_paths_.find(other_id);
    if (it_path == other_paths_.end()) return false;
    const auto &other_path = it_path->second;
    if (other_path.poses.empty()) return false;

    size_t my_hit_idx = std::numeric_limits<size_t>::max();
    size_t other_hit_idx = std::numeric_limits<size_t>::max();
    if (!findConflictPairIndices(my_planned_path_, other_path, overlap_distance_,
                                 &my_hit_idx, &other_hit_idx)) {
      return false;
    }

    const size_t my_now_idx = closestPathIndex(my_planned_path_, ego_pose_.value());
    const size_t other_now_idx = closestPathIndex(other_path, it_pose->second);
    const double my_dist = pathDistanceBetween(my_planned_path_, my_now_idx, my_hit_idx);
    const double other_dist = pathDistanceBetween(other_path, other_now_idx, other_hit_idx);
    const double eta_speed_floor = std::max(eta_min_speed_, ttc_min_speed_);
    const double my_eta = my_dist / std::max(estimateMySpeed(), eta_speed_floor);
    const double other_eta = other_dist / std::max(estimateOtherSpeed(other_id), eta_speed_floor);

    if (my_hit_idx_out) *my_hit_idx_out = my_hit_idx;
    if (other_hit_idx_out) *other_hit_idx_out = other_hit_idx;
    if (my_eta_out) *my_eta_out = my_eta;
    if (other_eta_out) *other_eta_out = other_eta;
    return true;
  }

  bool shouldYieldToSamePriorityByEta(int other_id) const {
    // Fallback for startup / missing peer data.
    const bool fallback = cav_id_ > other_id;
    const auto decide_now = [this, fallback](double my_eta, double other_eta) {
      if (my_eta > other_eta + same_priority_eta_margin_sec_) return true;
      if (other_eta > my_eta + same_priority_eta_margin_sec_) return false;
      return fallback;
    };

    size_t my_hit_idx = std::numeric_limits<size_t>::max();
    size_t other_hit_idx = std::numeric_limits<size_t>::max();
    double my_eta = 0.0;
    double other_eta = 0.0;
    if (!computePairEtaToConflict(other_id, &my_hit_idx, &other_hit_idx, &my_eta, &other_eta)) {
      auto it_latch = same_priority_yield_latch_.find(other_id);
      if (it_latch != same_priority_yield_latch_.end()) {
        auto it_time = same_priority_latch_time_.find(other_id);
        if (it_time != same_priority_latch_time_.end()) {
          const double age = (this->now() - it_time->second).seconds();
          if (age <= same_priority_latch_timeout_sec_) return it_latch->second;
        }
        same_priority_yield_latch_.erase(other_id);
        same_priority_latch_time_.erase(other_id);
      }
      return fallback;
    }

    auto it_latch = same_priority_yield_latch_.find(other_id);
    if (it_latch != same_priority_yield_latch_.end()) {
      const bool other_stopped = estimateOtherSpeed(other_id) < (ttc_min_speed_ * 2.0);
      if (static_cast<int>(my_hit_idx) <= same_priority_latch_release_hit_index_ || other_stopped) {
        return it_latch->second;
      }
      same_priority_yield_latch_.erase(other_id);
      same_priority_latch_time_.erase(other_id);
    }

    const bool decision = decide_now(my_eta, other_eta);
    if (static_cast<int>(my_hit_idx) <= same_priority_latch_hit_index_) {
      same_priority_yield_latch_[other_id] = decision;
      same_priority_latch_time_[other_id] = this->now();
    }
    return decision;
  }

  double estimateMySpeed() const {
    return std::max({std::abs(latest_speed_cmd_), ego_speed_estimate_, ttc_min_speed_});
  }

  double estimateOtherSpeed(int other_id) const {
    auto it = other_speed_estimates_.find(other_id);
    if (it == other_speed_estimates_.end()) return ttc_default_other_speed_;
    return std::max(it->second, ttc_min_speed_);
  }

  bool nearestOtherPose(int *other_id_out, double *dist_out) const {
    if (!ego_pose_.has_value()) return false;
    bool found = false;
    int best_id = -1;
    double best_d = std::numeric_limits<double>::infinity();
    for (const auto &kv : other_poses_) {
      const int other_id = kv.first;
      if (other_id == cav_id_) continue;
      if (isRearSameDirectionVehicle(kv.second)) continue;
      const double d = poseDistance(ego_pose_.value(), kv.second);
      if (!found || d < best_d || (std::abs(d - best_d) < 1e-6 && other_id < best_id)) {
        found = true;
        best_id = other_id;
        best_d = d;
      }
    }
    if (!found) return false;
    if (other_id_out) *other_id_out = best_id;
    if (dist_out) *dist_out = best_d;
    return true;
  }

  bool nearestMustYieldPose(int *other_id_out, double *dist_out) const {
    if (!ego_pose_.has_value()) return false;
    bool found = false;
    int best_id = -1;
    double best_d = std::numeric_limits<double>::infinity();
    for (const auto &kv : other_poses_) {
      const int other_id = kv.first;
      if (!shouldYieldTo(other_id)) continue;
      if (isRearSameDirectionVehicle(kv.second)) continue;
      const double d = poseDistance(ego_pose_.value(), kv.second);
      if (!found || d < best_d || (std::abs(d - best_d) < 1e-6 && other_id < best_id)) {
        found = true;
        best_id = other_id;
        best_d = d;
      }
    }
    if (!found) return false;
    if (other_id_out) *other_id_out = best_id;
    if (dist_out) *dist_out = best_d;
    return true;
  }

  TtcDecision evaluateTtcDecision() const {
    TtcDecision none;
    if (!ego_pose_.has_value() || my_planned_path_.poses.empty()) return none;

    const size_t my_now_idx = closestPathIndex(my_planned_path_, ego_pose_.value());
    const double my_speed = estimateMySpeed();

    bool has_yield = false;
    bool has_pass = false;
    TtcDecision best_yield;
    TtcDecision best_pass;
    double best_yield_score = std::numeric_limits<double>::infinity();
    double best_pass_score = std::numeric_limits<double>::infinity();

    for (const auto &kv : other_paths_) {
      const int other_id = kv.first;
      if (other_id == cav_id_) continue;
      auto it_pose = other_poses_.find(other_id);
      if (it_pose == other_poses_.end()) continue;

      const auto &other_path = kv.second;
      const auto &other_pose = it_pose->second;
      if (other_path.poses.empty()) continue;

      size_t my_hit_idx = std::numeric_limits<size_t>::max();
      size_t other_hit_idx = std::numeric_limits<size_t>::max();
      if (!findConflictPairIndices(my_planned_path_, other_path, ttc_overlap_distance_,
                                   &my_hit_idx, &other_hit_idx)) {
        continue;
      }
      if (static_cast<int>(my_hit_idx) > ttc_max_hit_index_ &&
          static_cast<int>(other_hit_idx) > ttc_max_hit_index_) {
        continue;
      }

      const size_t other_now_idx = closestPathIndex(other_path, other_pose);
      const double my_dist = pathDistanceBetween(my_planned_path_, my_now_idx, my_hit_idx);
      const double other_dist = pathDistanceBetween(other_path, other_now_idx, other_hit_idx);
      const double other_speed = estimateOtherSpeed(other_id);
      const double my_ttc = my_dist / std::max(my_speed, ttc_min_speed_);
      const double other_ttc = other_dist / std::max(other_speed, ttc_min_speed_);
      const double pose_dist = poseDistance(ego_pose_.value(), other_pose);

      const bool other_in_zone = other_dist <= ttc_conflict_distance_;
      const bool urgent =
          (std::min(my_ttc, other_ttc) <= ttc_threshold_sec_) ||
          (pose_dist <= immediate_pose_stop_distance_) ||
          other_in_zone;
      if (!urgent) continue;

      const bool must_yield = shouldYieldTo(other_id);
      bool should_yield = false;
      bool should_pass = false;

      // Deterministic arbitration in urgent conflict:
      // exactly one side yields (larger ID yields) to prevent reciprocal PASS.
      // TTC is used to trigger urgency, while priority keeps decisions consistent.
      if (pose_dist <= emergency_hard_stop_distance_ || other_in_zone ||
          std::min(my_ttc, other_ttc) <= ttc_threshold_sec_) {
        should_yield = must_yield;
        should_pass = !must_yield;
      } else if (my_ttc + ttc_tie_margin_sec_ < other_ttc) {
        should_pass = true;
      } else if (other_ttc + ttc_tie_margin_sec_ < my_ttc) {
        should_yield = true;
      } else {
        should_yield = must_yield;
        should_pass = !must_yield;
      }

      TtcDecision candidate;
      candidate.role = should_yield ? "yield" : "pass";
      candidate.blocking_id = other_id;
      candidate.my_hit_idx = my_hit_idx;
      candidate.other_hit_idx = other_hit_idx;
      candidate.my_ttc = my_ttc;
      candidate.other_ttc = other_ttc;
      candidate.pose_dist = pose_dist;
      candidate.other_in_zone = other_in_zone;

      const double score = std::min(my_ttc, other_ttc);
      if (should_yield) {
        if (!has_yield || score < best_yield_score) {
          has_yield = true;
          best_yield_score = score;
          best_yield = candidate;
        }
      } else if (should_pass) {
        if (!has_pass || score < best_pass_score) {
          has_pass = true;
          best_pass_score = score;
          best_pass = candidate;
        }
      }
    }

    if (has_yield) return best_yield;
    if (has_pass) return best_pass;
    return none;
  }

  bool getOverlapSpanWithThreshold(const nav_msgs::msg::Path &my_path,
                                   const nav_msgs::msg::Path &other_path,
                                   double threshold,
                                   size_t *first_hit_index,
                                   size_t *last_hit_index,
                                   bool require_heading_conflict = true) const {
    if (my_path.poses.empty() || other_path.poses.empty()) return false;

    bool has_overlap = false;
    size_t local_first_idx = std::numeric_limits<size_t>::max();
    size_t local_last_idx = 0;

    const double heading_thresh_rad = heading_conflict_min_deg_ * M_PI / 180.0;
    for (size_t i = 0; i < my_path.poses.size(); ++i) {
      for (size_t j = 0; j < other_path.poses.size(); ++j) {
        const double d = poseDistance(my_path.poses[i], other_path.poses[j]);
        if (d <= threshold) {
          if (require_heading_conflict) {
            // Exclude near-parallel co-directional flow: treat only true crossing/merge conflicts.
            const double my_h = headingAt(my_path, i);
            const double ot_h = headingAt(other_path, j);
            const double dh = std::abs(wrapAngle(my_h - ot_h));
            const bool near_conflict_core =
                static_cast<int>(i) <= (hard_stop_lookahead_index_ + 4);
            if (!near_conflict_core && dh < heading_thresh_rad) continue;
          }
          has_overlap = true;
          local_first_idx = std::min(local_first_idx, i);
          local_last_idx = std::max(local_last_idx, i);
        }
      }
    }

    if (!has_overlap) return false;
    if (first_hit_index) *first_hit_index = local_first_idx;
    if (last_hit_index) *last_hit_index = local_last_idx;
    return true;
  }

  OverlapInfo findHighestPriorityOverlap(double threshold, bool higher_only) const {
    OverlapInfo out;
    if (my_planned_path_.poses.empty()) return out;

    int selected_id = -1;
    size_t selected_first_idx = std::numeric_limits<size_t>::max();
    size_t selected_last_idx = 0;

    for (const auto &kv : other_paths_) {
      const int other_id = kv.first;
      const auto &other_path = kv.second;
      if (higher_only && other_id >= cav_id_) continue;

      size_t first_idx = std::numeric_limits<size_t>::max();
      size_t last_idx = 0;
      if (!getOverlapSpanWithThreshold(my_planned_path_, other_path, threshold, &first_idx, &last_idx)) {
        continue;
      }

      const bool should_replace =
          (selected_id == -1) || (other_id < selected_id) ||
          (other_id == selected_id && first_idx < selected_first_idx);
      if (should_replace) {
        selected_id = other_id;
        selected_first_idx = first_idx;
        selected_last_idx = last_idx;
      }
    }

    if (selected_id == -1) return out;
    out.valid = true;
    out.blocking_id = selected_id;
    out.first_idx = selected_first_idx;
    out.last_idx = std::max(selected_last_idx, selected_first_idx);
    return out;
  }

  bool getOverlapWithPeer(int other_id, double threshold,
                          size_t *first_idx_out,
                          size_t *last_idx_out = nullptr) const {
    if (other_id < 0) return false;
    auto it_path = other_paths_.find(other_id);
    if (it_path == other_paths_.end()) return false;
    size_t first_idx = std::numeric_limits<size_t>::max();
    size_t last_idx = 0;
    const bool has_overlap = getOverlapSpanWithThreshold(
        my_planned_path_, it_path->second, threshold, &first_idx, &last_idx);
    if (!has_overlap) return false;
    if (first_idx_out) *first_idx_out = first_idx;
    if (last_idx_out) *last_idx_out = last_idx;
    return true;
  }

  bool hasOccupiedConflictZoneNear(double threshold, size_t *hit_idx_out, int *other_id_out) const {
    if (!ego_pose_.has_value()) return false;
    OverlapInfo nearest;
    bool found = false;
    for (const auto &kv : other_paths_) {
      const int other_id = kv.first;
      if (other_id == cav_id_) continue;
      // Lower-priority vehicles should yield to higher-priority vehicles only.
      if (other_id >= cav_id_) continue;

      bisa::msg::ZoneReservation other_res;
      if (!hasFreshHigherReservation(other_id, &other_res)) continue;
      if (!other_res.in_zone) continue;
      auto it_pose = other_poses_.find(other_id);
      if (it_pose == other_poses_.end()) continue;
      const double pose_d = poseDistance(ego_pose_.value(), it_pose->second);
      if (pose_d > occupied_pose_stop_distance_) continue;

      size_t first_idx = std::numeric_limits<size_t>::max();
      size_t last_idx = 0;
      if (!getOverlapSpanWithThreshold(my_planned_path_, kv.second, threshold, &first_idx, &last_idx)) {
        continue;
      }

      if (!found || first_idx < nearest.first_idx) {
        found = true;
        nearest.valid = true;
        nearest.blocking_id = other_id;
        nearest.first_idx = first_idx;
      }
    }
    if (!found) return false;
    if (hit_idx_out) *hit_idx_out = nearest.first_idx;
    if (other_id_out) *other_id_out = nearest.blocking_id;
    return true;
  }

  GateMode decideGateMode(const OverlapInfo &overlap) const {
    if (!overlap.valid) return GateMode::kPass;
    if (isPreentryStopWindow(overlap.first_idx, stop_lookahead_index_)) return GateMode::kStop;
    return GateMode::kPass;
  }

  double pathDistanceUntil(size_t end_idx) const {
    if (my_planned_path_.poses.size() < 2) return 0.0;
    const size_t capped_end = std::min(end_idx, my_planned_path_.poses.size() - 1);
    double sum = 0.0;
    for (size_t i = 1; i <= capped_end; ++i) {
      sum += poseDistance(my_planned_path_.poses[i - 1], my_planned_path_.poses[i]);
    }
    return sum;
  }

  bool reservationsOverlap(double a_in, double a_out, double b_in, double b_out) const {
    const double left = std::max(a_in, b_in);
    const double right = std::min(a_out, b_out);
    return left <= (right + reservation_overlap_margin_sec_);
  }

  bool buildMyReservation(const OverlapInfo &overlap,
                          bisa::msg::ZoneReservation *msg_out,
                          double *eta_in_out,
                          double *eta_out_out) const {
    if (!overlap.valid || !msg_out || !eta_in_out || !eta_out_out) return false;
    if (my_planned_path_.poses.empty()) return false;

    const double speed = std::max(std::abs(latest_speed_cmd_), eta_min_speed_);
    const double dist_in = pathDistanceUntil(overlap.first_idx);
    const double dist_out = pathDistanceUntil(overlap.last_idx);
    const double now_sec = this->now().seconds();
    const double eta_in = now_sec + dist_in / speed;
    const double eta_out = now_sec + dist_out / speed + zone_clearance_sec_;

    msg_out->cav_id = cav_id_;
    msg_out->priority = cav_id_;
    msg_out->zone_id = buildZoneId(cav_id_, overlap.blocking_id);
    msg_out->eta_in_sec = eta_in;
    msg_out->eta_out_sec = eta_out;
    msg_out->in_zone = static_cast<int>(overlap.first_idx) <= escape_force_slow_index_;

    *eta_in_out = eta_in;
    *eta_out_out = eta_out;
    return true;
  }

  bool hasFreshHigherReservation(int other_id, bisa::msg::ZoneReservation *out) const {
    auto it_msg = higher_reservations_.find(other_id);
    auto it_rx = reservation_rx_time_.find(other_id);
    if (it_msg == higher_reservations_.end() || it_rx == reservation_rx_time_.end()) return false;
    const double age = (this->now() - it_rx->second).seconds();
    if (age > reservation_ttl_sec_) return false;
    if (out) *out = it_msg->second;
    return true;
  }

  double getActiveOfflineSpeedCap() const {
    if (!offline_speed_cap_enabled_) return std::numeric_limits<double>::infinity();
    if (!has_offline_cap_) return std::numeric_limits<double>::infinity();
    const double age = (this->now() - offline_cap_rx_time_).seconds();
    if (age > offline_speed_cap_timeout_sec_) return std::numeric_limits<double>::infinity();
    return std::max(0.0, offline_speed_cap_mps_);
  }

  bool hasNearPoseConflict(double stop_d, double release_d, int *blocking_id_out) const {
    if (!ego_pose_.has_value()) return false;
    if (my_planned_path_.poses.empty()) return false;
    const double threshold = near_pose_blocking_latched_ ? release_d : stop_d;
    bool blocking = false;
    int blocking_id = -1;

    for (const auto &kv : other_poses_) {
      const int other_id = kv.first;
      // Apply near-pose hard stop only to lower-priority vehicles.
      if (other_id >= cav_id_) continue;
      const auto &other_pose = kv.second;
      if (isRearSameDirectionVehicle(other_pose)) continue;
      auto it_path = other_paths_.find(other_id);
      if (it_path == other_paths_.end()) continue;

      // Near-pose stop must also be a near-term geometric conflict on local paths.
      // This prevents unnecessary stop when vehicles are physically close but path-separated.
      size_t first_idx = std::numeric_limits<size_t>::max();
      size_t last_idx = 0;
      const bool has_local_overlap = getOverlapSpanWithThreshold(
          my_planned_path_, it_path->second, overlap_distance_, &first_idx, &last_idx);
      if (!has_local_overlap) continue;
      if (static_cast<int>(first_idx) > (hard_stop_lookahead_index_ + 2)) continue;
      if (isOtherStationaryLongEnough(other_id)) continue;

      const double d = poseDistance(ego_pose_.value(), other_pose);
      if (d < threshold) {
        if (!blocking || other_id < blocking_id) {
          blocking = true;
          blocking_id = other_id;
        }
      }
    }

    if (blocking_id_out) *blocking_id_out = blocking_id;
    return blocking;
  }

  bool isOtherStationaryLongEnough(int other_id) const {
    auto it_speed = other_speed_estimates_.find(other_id);
    if (it_speed == other_speed_estimates_.end()) return false;
    if (it_speed->second > near_pose_stationary_speed_mps_) return false;
    auto it_since = other_stationary_since_.find(other_id);
    if (it_since == other_stationary_since_.end()) return false;
    const double stopped_age = (this->now() - it_since->second).seconds();
    return stopped_age >= near_pose_stationary_ignore_sec_;
  }

  bool hasFrontSameDirectionTooClose(double stop_d, double release_d,
                                     bool latched,
                                     int *blocking_id_out,
                                     double *closest_d_out) const {
    if (!ego_pose_.has_value()) return false;
    const double threshold = latched ? release_d : stop_d;

    bool blocking = false;
    int blocking_id = -1;
    double closest_d = std::numeric_limits<double>::infinity();
    for (const auto &kv : other_poses_) {
      const int other_id = kv.first;
      if (other_id == cav_id_) continue;
      const auto &other_pose = kv.second;
      if (!isFrontSameDirectionVehicle(other_pose)) continue;
      const double d = poseDistance(ego_pose_.value(), other_pose);
      if (d < threshold &&
          (d < closest_d || (std::abs(d - closest_d) < 1e-6 && other_id < blocking_id))) {
        blocking = true;
        blocking_id = other_id;
        closest_d = d;
      }
    }

    if (blocking_id_out) *blocking_id_out = blocking_id;
    if (closest_d_out) *closest_d_out = blocking ? closest_d : -1.0;
    return blocking;
  }

  bool hasImmediatePoseCollisionRisk(double stop_d, double release_d,
                                     int *blocking_id_out,
                                     size_t *hit_idx_out) const {
    if (!ego_pose_.has_value()) return false;
    if (my_planned_path_.poses.empty()) return false;

    const double threshold = immediate_pose_blocking_latched_ ? release_d : stop_d;
    bool blocking = false;
    int blocking_id = -1;
    size_t blocking_hit_idx = std::numeric_limits<size_t>::max();
    double nearest_d = std::numeric_limits<double>::infinity();

    for (const auto &kv : other_poses_) {
      const int other_id = kv.first;
      if (!shouldYieldTo(other_id)) continue;
      const auto &other_pose = kv.second;
      if (isRearSameDirectionVehicle(other_pose)) continue;
      auto it_path = other_paths_.find(other_id);
      if (it_path == other_paths_.end()) continue;

      // Immediate stop only for the vehicle that should yield by deterministic rule.
      // This prevents reciprocal STOP deadlocks while keeping near-term safety margin.
      size_t first_idx = std::numeric_limits<size_t>::max();
      size_t last_idx = 0;
      const bool has_local_overlap = getOverlapSpanWithThreshold(
          my_planned_path_, it_path->second, immediate_overlap_distance_, &first_idx, &last_idx, false);
      if (!has_local_overlap) continue;
      // Geo pre-entry stop handles the near conflict window first.
      // Avoid geo/imm oscillation by skipping immediate-stop in the same lookahead band.
      if (static_cast<int>(first_idx) <= stop_lookahead_index_) continue;
      if (static_cast<int>(first_idx) > (hard_stop_lookahead_index_ + 2)) continue;

      const double d = poseDistance(ego_pose_.value(), other_pose);
      if (d < threshold) {
        if (!blocking || d < nearest_d ||
            (std::abs(d - nearest_d) < 1e-6 && other_id < blocking_id)) {
          blocking = true;
          blocking_id = other_id;
          blocking_hit_idx = first_idx;
          nearest_d = d;
        }
      }
    }

    if (blocking_id_out) *blocking_id_out = blocking_id;
    if (hit_idx_out) *hit_idx_out = blocking_hit_idx;
    return blocking;
  }

  bool isSamePriorityPeer(int other_id) const {
    if (other_id == cav_id_) return false;
    const auto has_id = [this](int id) {
      return std::find(same_priority_ids_.begin(), same_priority_ids_.end(), id) !=
             same_priority_ids_.end();
    };
    return has_id(cav_id_) && has_id(other_id);
  }

  bool shouldYieldTo(int other_id) const {
    if (other_id == cav_id_) return false;
    // Same-priority peers: ETA/TTC arbitration with deterministic fallback.
    if (isSamePriorityPeer(other_id)) return shouldYieldToSamePriorityByEta(other_id);
    // Different priorities: larger ID yields.
    return cav_id_ > other_id;
  }

  bool hasSamePriorityTieBreakStop(double stop_d, double release_d,
                                   int *blocking_id_out,
                                   double *closest_d_out) const {
    if (!ego_pose_.has_value()) return false;
    if (my_planned_path_.poses.empty()) return false;
    if (same_priority_ids_.empty()) return false;

    const double threshold =
        same_priority_blocking_latched_ ? release_d : stop_d;
    bool blocking = false;
    int blocking_id = -1;
    double best_d = std::numeric_limits<double>::infinity();
    double best_my_eta = std::numeric_limits<double>::infinity();

    for (const auto &kv : other_poses_) {
      const int other_id = kv.first;
      if (!isSamePriorityPeer(other_id)) continue;
      if (!shouldYieldTo(other_id)) continue;
      const auto &other_pose = kv.second;
      if (isRearSameDirectionVehicle(other_pose)) continue;

      size_t my_hit_idx = std::numeric_limits<size_t>::max();
      size_t other_hit_idx = std::numeric_limits<size_t>::max();
      double my_eta = 0.0;
      double other_eta = 0.0;
      if (!computePairEtaToConflict(other_id, &my_hit_idx, &other_hit_idx, &my_eta, &other_eta)) {
        continue;
      }
      // Prevent stopping in the conflict core: tie-stop only before entry.
      if (!isPreentryStopWindow(my_hit_idx, stop_lookahead_index_)) continue;

      const double d = poseDistance(ego_pose_.value(), other_pose);
      if (d < threshold &&
          (my_eta < best_my_eta ||
           (std::abs(my_eta - best_my_eta) < 1e-6 &&
            (d < best_d || (std::abs(d - best_d) < 1e-6 && other_id < blocking_id))))) {
        blocking = true;
        blocking_id = other_id;
        best_d = d;
        best_my_eta = my_eta;
      }
    }

    if (blocking_id_out) *blocking_id_out = blocking_id;
    if (closest_d_out) *closest_d_out = blocking ? best_d : -1.0;
    return blocking;
  }

  bool hasEmergencyPoseTooClose(double stop_d, double release_d,
                                int *blocking_id_out,
                                double *closest_d_out) const {
    if (!ego_pose_.has_value()) return false;

    const double threshold = emergency_pose_blocking_latched_ ? release_d : stop_d;
    bool blocking = false;
    int blocking_id = -1;
    double best_d = std::numeric_limits<double>::infinity();

    for (const auto &kv : other_poses_) {
      const int other_id = kv.first;
      if (other_id == cav_id_) continue;
      const auto &other_pose = kv.second;
      if (isRearSameDirectionVehicle(other_pose)) continue;

      const double d = poseDistance(ego_pose_.value(), other_pose);
      const bool must_yield = shouldYieldTo(other_id);
      // For same-priority peers, deterministic yielding already decides PASS/YIELD.
      // Avoid reciprocal emergency STOP deadlocks on the higher-priority side.
      if (!must_yield && isSamePriorityPeer(other_id)) continue;
      if (!must_yield && d >= emergency_hard_stop_distance_) continue;
      const double trigger = must_yield ? threshold : emergency_hard_stop_distance_;
      if (d < trigger &&
          (d < best_d || (std::abs(d - best_d) < 1e-6 && other_id < blocking_id))) {
        blocking = true;
        blocking_id = other_id;
        best_d = d;
      }
    }

    if (blocking_id_out) *blocking_id_out = blocking_id;
    if (closest_d_out) *closest_d_out = blocking ? best_d : -1.0;
    return blocking;
  }

  bool isBlockingPoseClose(int blocking_id) const {
    if (blocking_id < 0) return false;
    if (!ego_pose_.has_value()) return false;
    auto it_pose = other_poses_.find(blocking_id);
    if (it_pose == other_poses_.end()) return false;
    const double d = poseDistance(ego_pose_.value(), it_pose->second);
    return d <= geometric_stop_pose_distance_;
  }

  void accelCallback(const geometry_msgs::msg::Accel::SharedPtr msg) {
    latest_speed_cmd_ = std::abs(msg->linear.x);

    const bool has_ego_pose = ego_pose_.has_value();
    const bool has_my_path = !my_planned_path_.poses.empty();
    const size_t other_pose_count = other_poses_.size();
    const size_t other_path_count = other_paths_.size();
    if (!has_ego_pose || !has_my_path || other_pose_count == 0 || other_path_count == 0) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "CAV%d gate input not ready: ego=%d my_path_pts=%zu other_poses=%zu other_paths=%zu",
          cav_id_, has_ego_pose ? 1 : 0, my_planned_path_.poses.size(),
          other_pose_count, other_path_count);
    }

    if (ttc_mode_enabled_) {
      const bool peers_ready =
          has_ego_pose && has_my_path &&
          static_cast<int>(other_pose_count) >= expected_peer_count_ &&
          static_cast<int>(other_path_count) >= expected_peer_count_;
      const double since_start = (this->now() - node_start_time_).seconds();
      const bool startup_hold = ttc_startup_hold_enabled_ &&
                                (!peers_ready || since_start < ttc_startup_hold_sec_);

      const TtcDecision ttc_decision = evaluateTtcDecision();
      int nearest_any_id = -1;
      double nearest_any_d = -1.0;
      const bool has_nearest_any = nearestOtherPose(&nearest_any_id, &nearest_any_d);
      int nearest_yield_id = -1;
      double nearest_yield_d = -1.0;
      const bool has_nearest_yield = nearestMustYieldPose(&nearest_yield_id, &nearest_yield_d);
      const bool absolute_emergency_stop =
          has_nearest_any && nearest_any_d <= ttc_absolute_emergency_stop_distance_;
      const bool near_emergency_stop =
          has_nearest_yield && nearest_yield_d <= ttc_emergency_stop_distance_;
      const bool near_yield_slow =
          has_nearest_yield && nearest_yield_d <= ttc_physical_guard_distance_;

      GateMode mode = GateMode::kPass;
      std::string ttc_role = ttc_decision.role;
      int status_block = ttc_decision.blocking_id;
      int status_hit = (ttc_decision.my_hit_idx == std::numeric_limits<size_t>::max())
                           ? -1
                           : static_cast<int>(ttc_decision.my_hit_idx);
      bool near_stop_flag = false;
      bool imm_stop_flag = false;

      if (startup_hold) {
        mode = GateMode::kStop;
        ttc_role = "hold";
        status_block = -1;
        status_hit = -1;
      } else if (absolute_emergency_stop) {
        mode = GateMode::kStop;
        ttc_role = "abs_emergency";
        status_block = nearest_any_id;
        status_hit = -1;
        near_stop_flag = true;
        imm_stop_flag = true;
      } else if (near_emergency_stop) {
        mode = GateMode::kStop;
        ttc_role = "near_emergency";
        status_block = nearest_yield_id;
        status_hit = -1;
        near_stop_flag = true;
        imm_stop_flag = true;
      } else if (near_yield_slow) {
        mode = GateMode::kSlow;
        ttc_role = "near_yield";
        status_block = nearest_yield_id;
        status_hit = -1;
        near_stop_flag = true;
      } else if (ttc_decision.role == "yield") {
        mode = GateMode::kSlow;
      } else if (ttc_decision.role == "pass") {
        mode = GateMode::kPass;
      }

      geometry_msgs::msg::Accel gated = *msg;
      const double raw_v = std::max(0.0, msg->linear.x);
      double target_v = std::min(raw_v, max_velocity_cap_);
      if (mode == GateMode::kStop) {
        target_v = 0.0;
      } else if (mode == GateMode::kSlow) {
        target_v = std::max(ttc_min_speed_, raw_v * yield_speed_ratio_);
      } else if (ttc_decision.role == "pass") {
        target_v = std::min(raw_v * pass_speed_boost_, max_velocity_cap_);
      }

      const double offline_cap = getActiveOfflineSpeedCap();
      const bool offline_cap_active = std::isfinite(offline_cap);
      const bool offline_hold = offline_cap_active && offline_cap <= 1e-3;
      if (offline_cap_active) {
        target_v = std::min(target_v, offline_cap);
      }
      if (offline_hold) {
        mode = GateMode::kStop;
        ttc_role = "offline_plan";
        status_block = -1;
        status_hit = -1;
      }

      current_mode_ = mode;
      const char *mode_str = (current_mode_ == GateMode::kPass) ? "PASS"
                             : (current_mode_ == GateMode::kSlow) ? "SLOW"
                                                                   : "STOP";

      if (current_mode_ == GateMode::kStop) {
        target_v = 0.0;
        gated.angular.z = 0.0;
      }
      gated.linear.x = target_v;
      accel_pub_->publish(gated);

      std_msgs::msg::String gate_status_msg;
      gate_status_msg.data =
          "mode=" + std::string(mode_str) +
          " block=" + std::to_string(status_block) +
          " hit=" + std::to_string(status_hit) +
          " geo=0 res=0 occ=0 near=" + std::string(near_stop_flag ? "1" : "0") +
          " near_block=" + std::to_string(near_stop_flag ? status_block : -1) +
          " imm=" + std::string(imm_stop_flag ? "1" : "0") +
          " imm_block=" + std::to_string(imm_stop_flag ? status_block : -1) +
          " imm_hit=-1"
          " tie=0 tie_block=-1 tie_d=-1 ems=0 ems_block=-1 ems_d=-1"
          " ego=" + std::string(has_ego_pose ? "1" : "0") +
          " my_path_n=" + std::to_string(my_planned_path_.poses.size()) +
          " other_pose_n=" + std::to_string(other_pose_count) +
          " other_path_n=" + std::to_string(other_path_count) +
          " ttc_role=" + ttc_role +
          " ttc_my=" + std::to_string(ttc_decision.my_ttc) +
          " ttc_other=" + std::to_string(ttc_decision.other_ttc) +
          " ttc_d=" + std::to_string(ttc_decision.pose_dist) +
          " ttc_in_zone=" + std::string(ttc_decision.other_in_zone ? "1" : "0") +
          " ttc_boot=" + std::string(startup_hold ? "1" : "0") +
          " ttc_boot_ready=" + std::string(peers_ready ? "1" : "0") +
          " offline_cap=" + std::to_string(offline_cap_active ? offline_cap : -1.0) +
          " offline_hold=" + std::string(offline_hold ? "1" : "0");
      gate_status_pub_->publish(gate_status_msg);

      if (current_mode_ != last_logged_mode_) {
        RCLCPP_WARN(this->get_logger(),
                    "CAV%d gate=%s role=%s block=%d hit=%d my_ttc=%.3f other_ttc=%.3f d=%.3f in_zone=%s "
                    "near_any_id=%d near_any_d=%.3f near_yield_id=%d near_yield_d=%.3f boot=%s ready=%s v_raw=%.3f v_out=%.3f",
                    cav_id_, mode_str, ttc_role.c_str(), status_block, status_hit,
                    ttc_decision.my_ttc, ttc_decision.other_ttc, ttc_decision.pose_dist,
                    ttc_decision.other_in_zone ? "true" : "false",
                    nearest_any_id, nearest_any_d, nearest_yield_id, nearest_yield_d,
                    startup_hold ? "true" : "false",
                    peers_ready ? "true" : "false", raw_v, target_v);
        last_logged_mode_ = current_mode_;
      }
      return;
    }

    OverlapInfo overlap_now = findHighestPriorityOverlap(overlap_distance_, true);
    OverlapInfo overlap_release = findHighestPriorityOverlap(release_overlap_distance_, true);

    if (geometric_preentry_latched_) {
      size_t release_first_idx = std::numeric_limits<size_t>::max();
      const bool still_overlap = getOverlapWithPeer(
          geometric_preentry_blocking_id_, release_overlap_distance_, &release_first_idx);
      // Keep geo pre-entry STOP independent from same-priority ETA/tie oscillation.
      // Once latched, release only when the overlap is no longer in pre-entry window.
      if (still_overlap && isPreentryStopWindow(release_first_idx, stop_lookahead_index_)) {
        geometric_preentry_hit_idx_ = release_first_idx;
      } else {
        geometric_preentry_latched_ = false;
        geometric_preentry_blocking_id_ = -1;
        geometric_preentry_hit_idx_ = std::numeric_limits<size_t>::max();
      }
    }
    if (!geometric_preentry_latched_ && overlap_now.valid &&
        shouldYieldTo(overlap_now.blocking_id) &&
        isPreentryStopWindow(overlap_now.first_idx, stop_lookahead_index_)) {
      geometric_preentry_latched_ = true;
      geometric_preentry_blocking_id_ = overlap_now.blocking_id;
      geometric_preentry_hit_idx_ = overlap_now.first_idx;
    }

    OverlapInfo overlap_for_mode = overlap_now;
    // Base proactive stop from geometric lookahead to keep collision safety margin.
    GateMode mode = decideGateMode(overlap_for_mode);
    const auto should_hold_geometric_stop = [this](int blocking_id) {
      if (blocking_id < 0) return false;
      auto it_pose = other_poses_.find(blocking_id);
      if (it_pose == other_poses_.end()) return false;
      // Ignore trailing same-direction vehicles for geometric pre-stop.
      if (isRearSameDirectionVehicle(it_pose->second)) return false;
      // Keep proactive pre-entry STOP whenever deterministic arbitration says we must yield.
      // This prevents late imm/ems STOP near the conflict core.
      if (shouldYieldTo(blocking_id)) return true;
      // Same-priority PASS side should not be held by geometric close-distance only.
      if (isSamePriorityPeer(blocking_id)) return false;
      return isBlockingPoseClose(blocking_id);
    };
    if (mode == GateMode::kStop && overlap_for_mode.valid &&
        !should_hold_geometric_stop(overlap_for_mode.blocking_id)) {
      mode = GateMode::kPass;
    }
    if (current_mode_ != GateMode::kPass && mode == GateMode::kPass) {
      overlap_for_mode = overlap_release;
      mode = decideGateMode(overlap_for_mode);
      if (mode == GateMode::kStop && overlap_for_mode.valid &&
          !should_hold_geometric_stop(overlap_for_mode.blocking_id)) {
        mode = GateMode::kPass;
      }
    }
    if (geometric_preentry_latched_) {
      mode = GateMode::kStop;
    }
    bool geometric_pre_stop = (mode == GateMode::kStop);

    bisa::msg::ZoneReservation my_reservation;
    my_reservation.cav_id = cav_id_;
    my_reservation.priority = cav_id_;
    my_reservation.zone_id = "";
    my_reservation.eta_in_sec = -1.0;
    my_reservation.eta_out_sec = -1.0;
    my_reservation.in_zone = false;

    bool has_my_reservation = false;
    double my_eta_in = -1.0;
    double my_eta_out = -1.0;
    if (buildMyReservation(overlap_now, &my_reservation, &my_eta_in, &my_eta_out)) {
      has_my_reservation = true;
    }
    reservation_pub_->publish(my_reservation);

    bool reservation_preentry_stop = false;
    if (has_my_reservation && overlap_now.valid) {
      bisa::msg::ZoneReservation higher_res;
      if (hasFreshHigherReservation(overlap_now.blocking_id, &higher_res) &&
          higher_res.zone_id == my_reservation.zone_id &&
          higher_res.eta_in_sec >= 0.0 && higher_res.eta_out_sec >= 0.0) {
        reservation_preentry_stop =
            reservationsOverlap(my_eta_in, my_eta_out, higher_res.eta_in_sec, higher_res.eta_out_sec);
      }
    }

    if (reservation_preentry_stop) {
      mode = GateMode::kStop;
    }

    // Safety override: if a higher-priority vehicle is already in-zone on a conflicting
    // path nearby, stop to avoid a physical crash.
    size_t occupied_hit_idx = std::numeric_limits<size_t>::max();
    int occupied_other_id = -1;
    bool occupied_zone_stop = false;
    if (hasOccupiedConflictZoneNear(overlap_distance_, &occupied_hit_idx, &occupied_other_id) &&
        isPreentryStopWindow(occupied_hit_idx, stop_lookahead_index_)) {
      mode = GateMode::kStop;
      occupied_zone_stop = true;
    }

    int near_blocking_id = -1;
    near_pose_blocking_latched_ =
        hasNearPoseConflict(near_pose_stop_distance_, near_pose_release_distance_, &near_blocking_id);
    bool near_pose_stop = false;
    if (near_pose_blocking_latched_) {
      mode = GateMode::kStop;
      near_pose_stop = true;
    }

    int rear_slow_blocking_id = -1;
    double rear_slow_closest_d = -1.0;
    rear_follow_slow_latched_ = hasFrontSameDirectionTooClose(
        rear_follow_slow_distance_, rear_follow_slow_release_distance_,
        rear_follow_slow_latched_, &rear_slow_blocking_id, &rear_slow_closest_d);
    int rear_stop_blocking_id = -1;
    double rear_stop_closest_d = -1.0;
    rear_follow_stop_latched_ = hasFrontSameDirectionTooClose(
        rear_follow_stop_distance_, rear_follow_stop_release_distance_,
        rear_follow_stop_latched_, &rear_stop_blocking_id, &rear_stop_closest_d);
    bool rear_follow_slow = false;
    bool rear_follow_stop = false;
    int rear_follow_blocking_id = -1;
    double rear_follow_closest_d = -1.0;
    if (rear_follow_stop_latched_) {
      mode = GateMode::kStop;
      rear_follow_stop = true;
      rear_follow_blocking_id = rear_stop_blocking_id;
      rear_follow_closest_d = rear_stop_closest_d;
    } else if (rear_follow_slow_latched_ && mode != GateMode::kStop) {
      mode = GateMode::kSlow;
      rear_follow_slow = true;
      rear_follow_blocking_id = rear_slow_blocking_id;
      rear_follow_closest_d = rear_slow_closest_d;
    }

    int immediate_blocking_id = -1;
    size_t immediate_hit_idx = std::numeric_limits<size_t>::max();
    immediate_pose_blocking_latched_ = hasImmediatePoseCollisionRisk(
        immediate_pose_stop_distance_, immediate_pose_release_distance_,
        &immediate_blocking_id, &immediate_hit_idx);
    bool immediate_pose_stop = false;
    if (immediate_pose_blocking_latched_) {
      mode = GateMode::kStop;
      immediate_pose_stop = true;
    }

    int same_priority_blocking_id = -1;
    double same_priority_closest_d = -1.0;
    same_priority_blocking_latched_ = hasSamePriorityTieBreakStop(
        same_priority_stop_distance_, same_priority_release_distance_,
        &same_priority_blocking_id, &same_priority_closest_d);
    bool same_priority_tie_stop = false;
    if (same_priority_blocking_latched_) {
      mode = GateMode::kStop;
      same_priority_tie_stop = true;
    }

    int emergency_blocking_id = -1;
    double emergency_closest_d = -1.0;
    emergency_pose_blocking_latched_ = hasEmergencyPoseTooClose(
        emergency_pose_stop_distance_, emergency_pose_release_distance_,
        &emergency_blocking_id, &emergency_closest_d);
    bool emergency_pose_stop = false;
    if (emergency_pose_blocking_latched_) {
      mode = GateMode::kStop;
      emergency_pose_stop = true;
    }

    current_mode_ = mode;

    geometry_msgs::msg::Accel gated = *msg;
    if (current_mode_ == GateMode::kStop) {
      gated.linear.x = 0.0;
      gated.angular.z = 0.0;
    }
    const double offline_cap = getActiveOfflineSpeedCap();
    const bool offline_cap_active = std::isfinite(offline_cap);
    const bool offline_hold = offline_cap_active && offline_cap <= 1e-3;
    if (offline_cap_active) {
      gated.linear.x = std::min(gated.linear.x, offline_cap);
    }
    if (offline_hold) {
      gated.linear.x = 0.0;
      gated.angular.z = 0.0;
      current_mode_ = GateMode::kStop;
    }
    const char *mode_str = (current_mode_ == GateMode::kPass) ? "PASS"
                           : (current_mode_ == GateMode::kSlow) ? "SLOW"
                                                                 : "STOP";

    accel_pub_->publish(gated);

    int status_block = overlap_for_mode.valid ? overlap_for_mode.blocking_id : -1;
    int status_hit = overlap_for_mode.valid ? static_cast<int>(overlap_for_mode.first_idx) : -1;
    if (geometric_preentry_latched_) {
      status_block = geometric_preentry_blocking_id_;
      status_hit = geometric_preentry_hit_idx_ == std::numeric_limits<size_t>::max()
                       ? -1
                       : static_cast<int>(geometric_preentry_hit_idx_);
    }
    if (occupied_zone_stop) {
      status_block = occupied_other_id;
      status_hit = static_cast<int>(occupied_hit_idx);
    }
    if (near_pose_stop && near_blocking_id >= 0) {
      status_block = near_blocking_id;
    }
    if ((rear_follow_stop || rear_follow_slow) && rear_follow_blocking_id >= 0) {
      status_block = rear_follow_blocking_id;
      status_hit = -1;
    }
    if (immediate_pose_stop && immediate_blocking_id >= 0) {
      status_block = immediate_blocking_id;
      status_hit = (immediate_hit_idx == std::numeric_limits<size_t>::max())
                       ? -1
                       : static_cast<int>(immediate_hit_idx);
    }
    if (emergency_pose_stop && emergency_blocking_id >= 0) {
      status_block = emergency_blocking_id;
      status_hit = -1;
    }
    if (same_priority_tie_stop && same_priority_blocking_id >= 0) {
      status_block = same_priority_blocking_id;
      status_hit = -1;
    }

    std_msgs::msg::String gate_status_msg;
    gate_status_msg.data =
        "mode=" + std::string(mode_str) +
        " block=" + std::to_string(status_block) +
        " hit=" + std::to_string(status_hit) +
        " geo=" + std::string(geometric_pre_stop ? "1" : "0") +
        " res=" + std::string(reservation_preentry_stop ? "1" : "0") +
        " occ=" + std::string(occupied_zone_stop ? "1" : "0") +
        " near=" + std::string(near_pose_stop ? "1" : "0") +
        " near_block=" + std::to_string(near_blocking_id) +
        " imm=" + std::string(immediate_pose_stop ? "1" : "0") +
        " imm_block=" + std::to_string(immediate_blocking_id) +
        " imm_hit=" + std::to_string(
            immediate_hit_idx == std::numeric_limits<size_t>::max()
                ? -1
                : static_cast<int>(immediate_hit_idx)) +
        " ems=" + std::string(emergency_pose_stop ? "1" : "0") +
        " ems_block=" + std::to_string(emergency_blocking_id) +
        " ems_d=" + std::to_string(emergency_closest_d) +
        " rear_slow=" + std::string(rear_follow_slow ? "1" : "0") +
        " rear_stop=" + std::string(rear_follow_stop ? "1" : "0") +
        " rear_block=" + std::to_string(rear_follow_blocking_id) +
        " rear_d=" + std::to_string(rear_follow_closest_d) +
        " tie=" + std::string(same_priority_tie_stop ? "1" : "0") +
        " tie_block=" + std::to_string(same_priority_blocking_id) +
        " tie_d=" + std::to_string(same_priority_closest_d) +
        " offline_cap=" + std::to_string(offline_cap_active ? offline_cap : -1.0) +
        " offline_hold=" + std::string(offline_hold ? "1" : "0") +
        " ego=" + std::string(has_ego_pose ? "1" : "0") +
        " my_path_n=" + std::to_string(my_planned_path_.poses.size()) +
        " other_pose_n=" + std::to_string(other_pose_count) +
        " other_path_n=" + std::to_string(other_path_count);
    gate_status_pub_->publish(gate_status_msg);

    if (current_mode_ == GateMode::kStop) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 300,
                           "CAV%d gate=%s block=%d hit=%d geo_stop=%s res_hold=%s occ_stop=%s "
                           "near_stop=%s near_block=%d imm_stop=%s imm_block=%d imm_hit=%d "
                           "tie_stop=%s tie_block=%d tie_d=%.3f "
                           "ems_stop=%s ems_block=%d ems_d=%.3f "
                           "rear_slow=%s rear_stop=%s rear_block=%d rear_d=%.3f",
                           cav_id_, mode_str,
                           status_block,
                           status_hit,
                           geometric_pre_stop ? "true" : "false",
                           reservation_preentry_stop ? "true" : "false",
                           occupied_zone_stop ? "true" : "false",
                           near_pose_stop ? "true" : "false",
                           near_blocking_id,
                           immediate_pose_stop ? "true" : "false",
                           immediate_blocking_id,
                           immediate_hit_idx == std::numeric_limits<size_t>::max()
                               ? -1
                               : static_cast<int>(immediate_hit_idx),
                           same_priority_tie_stop ? "true" : "false",
                           same_priority_blocking_id,
                           same_priority_closest_d,
                           emergency_pose_stop ? "true" : "false",
                           emergency_blocking_id,
                           emergency_closest_d,
                           rear_follow_slow ? "true" : "false",
                           rear_follow_stop ? "true" : "false",
                           rear_follow_blocking_id,
                           rear_follow_closest_d);
    } else if (current_mode_ != last_logged_mode_) {
      RCLCPP_WARN(this->get_logger(),
                  "CAV%d gate=%s block=%d hit=%d geo_stop=%s res_hold=%s occ_stop=%s "
                  "near_stop=%s near_block=%d imm_stop=%s imm_block=%d imm_hit=%d "
                  "tie_stop=%s tie_block=%d tie_d=%.3f "
                  "ems_stop=%s ems_block=%d ems_d=%.3f "
                  "rear_slow=%s rear_stop=%s rear_block=%d rear_d=%.3f",
                  cav_id_, mode_str,
                  status_block,
                  status_hit,
                  geometric_pre_stop ? "true" : "false",
                  reservation_preentry_stop ? "true" : "false",
                  occupied_zone_stop ? "true" : "false",
                  near_pose_stop ? "true" : "false",
                  near_blocking_id,
                  immediate_pose_stop ? "true" : "false",
                  immediate_blocking_id,
                  immediate_hit_idx == std::numeric_limits<size_t>::max()
                      ? -1
                      : static_cast<int>(immediate_hit_idx),
                  same_priority_tie_stop ? "true" : "false",
                  same_priority_blocking_id,
                  same_priority_closest_d,
                  emergency_pose_stop ? "true" : "false",
                  emergency_blocking_id,
                  emergency_closest_d,
                  rear_follow_slow ? "true" : "false",
                  rear_follow_stop ? "true" : "false",
                  rear_follow_blocking_id,
                  rear_follow_closest_d);
      last_logged_mode_ = current_mode_;
    }
    if (current_mode_ == GateMode::kStop) {
      last_logged_mode_ = current_mode_;
    }
  }

  int cav_id_{1};
  double overlap_distance_{0.58};
  double release_overlap_distance_{0.62};
  int stop_lookahead_index_{6};
  int hard_stop_lookahead_index_{9};
  double slow_ratio_{0.35};
  int escape_force_slow_index_{5};
  double eta_min_speed_{0.15};
  double zone_clearance_sec_{0.25};
  double reservation_ttl_sec_{0.6};
  double reservation_overlap_margin_sec_{0.02};
  double heading_conflict_min_deg_{12.0};
  double near_pose_stop_distance_{0.36};
  double near_pose_release_distance_{0.46};
  double near_pose_stationary_speed_mps_{0.02};
  double near_pose_stationary_ignore_sec_{1.2};
  double occupied_pose_stop_distance_{0.52};
  double geometric_stop_pose_distance_{1.10};
  double immediate_pose_stop_distance_{0.78};
  double immediate_pose_release_distance_{0.92};
  double immediate_overlap_distance_{0.55};
  double emergency_pose_stop_distance_{0.70};
  double emergency_pose_release_distance_{0.85};
  std::vector<int> same_priority_ids_;
  double same_priority_stop_distance_{1.20};
  double same_priority_release_distance_{1.40};
  double same_priority_eta_margin_sec_{0.20};
  int preentry_stop_min_hit_index_{3};
  int same_priority_latch_hit_index_{18};
  int same_priority_latch_release_hit_index_{28};
  double same_priority_latch_timeout_sec_{1.5};
  double emergency_hard_stop_distance_{0.38};
  bool ttc_mode_enabled_{true};
  double ttc_threshold_sec_{3.0};
  double ttc_tie_margin_sec_{0.15};
  double ttc_overlap_distance_{0.35};
  double ttc_conflict_distance_{0.35};
  int ttc_max_hit_index_{24};
  double ttc_physical_guard_distance_{0.80};
  double ttc_emergency_stop_distance_{0.45};
  double ttc_absolute_emergency_stop_distance_{0.40};
  bool ttc_startup_hold_enabled_{true};
  double ttc_startup_hold_sec_{1.0};
  double ttc_min_speed_{0.05};
  double ttc_default_other_speed_{0.35};
  double yield_speed_ratio_{0.55};
  double pass_speed_boost_{1.15};
  double max_velocity_cap_{0.70};
  bool offline_speed_cap_enabled_{true};
  double offline_speed_cap_timeout_sec_{1.0};
  double rear_same_direction_ignore_distance_{0.22};
  double rear_same_direction_heading_rad_{40.0 * M_PI / 180.0};
  double rear_follow_slow_distance_{0.90};
  double rear_follow_slow_release_distance_{1.10};
  double rear_follow_stop_distance_{0.55};
  double rear_follow_stop_release_distance_{0.72};
  double rear_follow_lateral_distance_{0.28};
  double offline_speed_cap_mps_{0.0};
  bool has_offline_cap_{false};
  double latest_speed_cmd_{0.15};
  double ego_speed_estimate_{0.0};
  bool near_pose_blocking_latched_{false};
  bool rear_follow_slow_latched_{false};
  bool rear_follow_stop_latched_{false};
  bool geometric_preentry_latched_{false};
  int geometric_preentry_blocking_id_{-1};
  size_t geometric_preentry_hit_idx_{std::numeric_limits<size_t>::max()};
  bool immediate_pose_blocking_latched_{false};
  bool same_priority_blocking_latched_{false};
  bool emergency_pose_blocking_latched_{false};
  int expected_peer_count_{0};

  GateMode current_mode_{GateMode::kPass};
  GateMode last_logged_mode_{GateMode::kPass};
  nav_msgs::msg::Path my_planned_path_;
  std::optional<geometry_msgs::msg::PoseStamped> ego_pose_;
  std::unordered_map<int, nav_msgs::msg::Path> other_paths_;
  std::unordered_map<int, geometry_msgs::msg::PoseStamped> other_poses_;
  std::unordered_map<int, double> other_speed_estimates_;
  std::unordered_map<int, rclcpp::Time> other_stationary_since_;
  mutable std::unordered_map<int, bool> same_priority_yield_latch_;
  mutable std::unordered_map<int, rclcpp::Time> same_priority_latch_time_;
  std::unordered_map<int, bisa::msg::ZoneReservation> higher_reservations_;
  std::unordered_map<int, rclcpp::Time> reservation_rx_time_;
  std::unordered_map<int, rclcpp::Time> other_pose_rx_time_;
  rclcpp::Time offline_cap_rx_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time ego_pose_rx_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time node_start_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<geometry_msgs::msg::Accel>::SharedPtr accel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr planned_path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ego_pose_sub_;
  rclcpp::Subscription<bisa::msg::ZoneReservation>::SharedPtr reservation_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr offline_cap_sub_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr> other_path_subs_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> other_pose_subs_;
  rclcpp::Publisher<geometry_msgs::msg::Accel>::SharedPtr accel_pub_;
  rclcpp::Publisher<bisa::msg::ZoneReservation>::SharedPtr reservation_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gate_status_pub_;
};

}  // namespace bisa

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bisa::PriorityCollisionGate>());
  rclcpp::shutdown();
  return 0;
}
