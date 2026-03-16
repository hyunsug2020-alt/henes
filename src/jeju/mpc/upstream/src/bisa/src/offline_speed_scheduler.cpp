#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace bisa {

namespace {

std::string idTo2Digit(int id) {
  if (id < 10) return "0" + std::to_string(id);
  return std::to_string(id);
}

double wrapAngle(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

double poseDistance(const geometry_msgs::msg::PoseStamped& a,
                    const geometry_msgs::msg::PoseStamped& b) {
  const double dx = a.pose.position.x - b.pose.position.x;
  const double dy = a.pose.position.y - b.pose.position.y;
  return std::hypot(dx, dy);
}

double headingAt(const nav_msgs::msg::Path& path, size_t idx) {
  if (path.poses.size() < 2) return 0.0;
  const size_t i0 = (idx == 0) ? 0 : idx - 1;
  const size_t i1 = std::min(idx + 1, path.poses.size() - 1);
  const auto& p0 = path.poses[i0].pose.position;
  const auto& p1 = path.poses[i1].pose.position;
  return std::atan2(p1.y - p0.y, p1.x - p0.x);
}

std::vector<double> cumulativeArcLengths(const nav_msgs::msg::Path& path) {
  if (path.poses.empty()) return {0.0};
  std::vector<double> out(path.poses.size(), 0.0);
  for (size_t i = 1; i < path.poses.size(); ++i) {
    out[i] = out[i - 1] + poseDistance(path.poses[i - 1], path.poses[i]);
  }
  return out;
}

}  // namespace

class OfflineSpeedScheduler : public rclcpp::Node {
 public:
  OfflineSpeedScheduler() : Node("offline_speed_scheduler") {
    this->declare_parameter("cav_ids", std::vector<int64_t>{1, 2, 3, 4});
    this->declare_parameter("default_speed_mps", 0.55);
    this->declare_parameter("nominal_speed_entries", std::vector<std::string>{});
    this->declare_parameter("conflict_distance", 0.45);
    this->declare_parameter("heading_conflict_min_deg", 12.0);
    this->declare_parameter("zone_merge_distance", 0.55);
    this->declare_parameter("zone_half_window_m", 0.35);
    this->declare_parameter("safety_margin_sec", 0.90);
    this->declare_parameter("max_start_delay_sec", 25.0);
    this->declare_parameter("publish_hz", 10.0);
    this->declare_parameter("plan_timeout_sec", 8.0);

    const auto cav_ids_param = this->get_parameter("cav_ids").as_integer_array();
    for (const auto id_raw : cav_ids_param) cav_ids_.push_back(static_cast<int>(id_raw));

    default_speed_mps_ = std::max(0.05, this->get_parameter("default_speed_mps").as_double());
    conflict_distance_ = std::max(0.05, this->get_parameter("conflict_distance").as_double());
    heading_conflict_min_rad_ =
        std::max(0.0, this->get_parameter("heading_conflict_min_deg").as_double()) * M_PI / 180.0;
    zone_merge_distance_ = std::max(0.05, this->get_parameter("zone_merge_distance").as_double());
    zone_half_window_m_ = std::max(0.05, this->get_parameter("zone_half_window_m").as_double());
    safety_margin_sec_ = std::max(0.0, this->get_parameter("safety_margin_sec").as_double());
    max_start_delay_sec_ = std::max(0.0, this->get_parameter("max_start_delay_sec").as_double());
    publish_hz_ = std::max(1.0, this->get_parameter("publish_hz").as_double());
    plan_timeout_sec_ = std::max(1.0, this->get_parameter("plan_timeout_sec").as_double());

    for (const int cav_id : cav_ids_) {
      nominal_speed_map_[cav_id] = default_speed_mps_;
      start_delay_map_[cav_id] = 0.0;
    }
    parseNominalSpeedEntries(this->get_parameter("nominal_speed_entries").as_string_array());

    auto qos_path = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
    auto qos_cap = rclcpp::QoS(rclcpp::KeepLast(20)).reliable();
    auto qos_status = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
    for (const int cav_id : cav_ids_) {
      const std::string path_topic = "/cav" + idTo2Digit(cav_id) + "/global_path";
      auto sub = this->create_subscription<nav_msgs::msg::Path>(
          path_topic, qos_path,
          [this, cav_id](const nav_msgs::msg::Path::SharedPtr msg) {
            if (!msg || msg->poses.empty()) return;
            paths_[cav_id] = *msg;
          });
      path_subs_.push_back(sub);

      const std::string cap_topic = "/cav" + idTo2Digit(cav_id) + "/offline_speed_cap";
      cap_pubs_[cav_id] = this->create_publisher<std_msgs::msg::Float64>(cap_topic, qos_cap);
    }
    status_pub_ =
        this->create_publisher<std_msgs::msg::String>("/offline_scheduler/status", qos_status);

    node_start_time_ = this->now();
    plan_start_time_ = this->now();
    publishStatus(planner_state_);

    const auto publish_period =
        std::chrono::duration<double>(1.0 / std::max(1.0, publish_hz_));
    publish_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(publish_period),
        std::bind(&OfflineSpeedScheduler::publishCaps, this));
    status_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500), std::bind(&OfflineSpeedScheduler::publishStatusTick, this));
    plan_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200), std::bind(&OfflineSpeedScheduler::maybePlanOnce, this));

    RCLCPP_INFO(this->get_logger(),
                "offline scheduler initialized cav_count=%zu default_speed=%.2f",
                cav_ids_.size(), default_speed_mps_);
  }

 private:
  struct ConflictSample {
    int cav_a{0};
    size_t idx_a{0};
    int cav_b{0};
    size_t idx_b{0};
    double mx{0.0};
    double my{0.0};
  };

  using ZoneIntervals = std::map<int, std::pair<double, double>>;  // cav_id -> [s_enter, s_exit]

  void parseNominalSpeedEntries(const std::vector<std::string>& entries) {
    for (const auto& raw : entries) {
      const auto pos = raw.find(':');
      if (pos == std::string::npos) continue;
      try {
        const int cav_id = std::stoi(raw.substr(0, pos));
        const double speed = std::max(0.05, std::stod(raw.substr(pos + 1)));
        if (nominal_speed_map_.count(cav_id) > 0) nominal_speed_map_[cav_id] = speed;
      } catch (...) {
      }
    }
  }

  void maybePlanOnce() {
    if (plan_ready_) return;

    std::vector<int> missing;
    for (const int cav_id : cav_ids_) {
      if (paths_.count(cav_id) == 0) missing.push_back(cav_id);
    }

    const double elapsed = (this->now() - node_start_time_).seconds();
    if (!missing.empty() && elapsed < plan_timeout_sec_) {
      planner_state_ = "waiting_paths";
      return;
    }

    if (!missing.empty()) {
      planner_state_ = "timeout_partial";
      std::ostringstream oss;
      for (size_t i = 0; i < missing.size(); ++i) {
        if (i) oss << ",";
        oss << missing[i];
      }
      RCLCPP_WARN(this->get_logger(),
                  "planner timeout: missing global paths for [%s], running partial schedule",
                  oss.str().c_str());
    } else {
      planner_state_ = "planning";
    }

    std::vector<int> available_ids;
    for (const int cav_id : cav_ids_) {
      if (paths_.count(cav_id) > 0) available_ids.push_back(cav_id);
    }

    if (available_ids.size() < 2) {
      plan_ready_ = true;
      plan_start_time_ = this->now();
      planner_state_ = "ready_no_conflict";
      publishStatus(planner_state_);
      return;
    }

    const auto delays = computeStartDelays(available_ids);
    for (const int cav_id : cav_ids_) {
      auto it = delays.find(cav_id);
      start_delay_map_[cav_id] = (it != delays.end()) ? std::max(0.0, it->second) : 0.0;
    }

    plan_ready_ = true;
    plan_start_time_ = this->now();
    planner_state_ = "ready";
    publishStatus(planner_state_);

    std::ostringstream oss;
    for (size_t i = 0; i < cav_ids_.size(); ++i) {
      const int cav_id = cav_ids_[i];
      if (i) oss << ", ";
      oss << "CAV" << idTo2Digit(cav_id) << "=" << start_delay_map_[cav_id];
    }
    RCLCPP_INFO(this->get_logger(), "offline start delays: %s", oss.str().c_str());
  }

  std::map<int, double> computeStartDelays(const std::vector<int>& available_ids) {
    std::map<int, std::vector<double>> arc_map;
    for (const int cav_id : available_ids) {
      arc_map[cav_id] = cumulativeArcLengths(paths_.at(cav_id));
    }

    const auto conflicts = extractConflictSamples(available_ids);
    const auto zones = buildZones(conflicts, arc_map);

    std::map<int, double> delays;
    for (const int cav_id : available_ids) delays[cav_id] = 0.0;
    if (zones.empty()) return delays;

    for (int iter = 0; iter < 24; ++iter) {
      bool changed = false;
      for (const auto& zone : zones) {
        std::vector<int> participants;
        participants.reserve(zone.size());
        for (const auto& kv : zone) participants.push_back(kv.first);
        std::sort(participants.begin(), participants.end());

        for (size_t i = 0; i < participants.size(); ++i) {
          for (size_t j = i + 1; j < participants.size(); ++j) {
            const int a = participants[i];
            const int b = participants[j];
            const auto a_it = zone.find(a);
            const auto b_it = zone.find(b);
            if (a_it == zone.end() || b_it == zone.end()) continue;

            const double a_enter = a_it->second.first;
            const double a_exit = a_it->second.second;
            const double b_enter = b_it->second.first;
            const double b_exit = b_it->second.second;
            const double va = nominalSpeed(a);
            const double vb = nominalSpeed(b);

            const double a_in = delays[a] + a_enter / va;
            const double a_out = delays[a] + a_exit / va;
            const double b_in = delays[b] + b_enter / vb;
            const double b_out = delays[b] + b_exit / vb;

            const double left = std::max(a_in, b_in);
            const double right = std::min(a_out, b_out);
            if (left > right + safety_margin_sec_) continue;

            // Deterministic priority: larger ID yields.
            const int yield_id = std::max(a, b);
            const int pass_id = std::min(a, b);
            const double y_enter = zone.at(yield_id).first;
            const double pass_out = delays[pass_id] + zone.at(pass_id).second / nominalSpeed(pass_id);
            double required = pass_out + safety_margin_sec_ - y_enter / nominalSpeed(yield_id);
            required = std::clamp(required, 0.0, max_start_delay_sec_);
            if (required > delays[yield_id] + 1e-4) {
              delays[yield_id] = required;
              changed = true;
            }
          }
        }
      }
      if (!changed) break;
    }
    return delays;
  }

  std::vector<ConflictSample> extractConflictSamples(const std::vector<int>& available_ids) const {
    std::vector<ConflictSample> out;
    std::vector<int> ids = available_ids;
    std::sort(ids.begin(), ids.end());
    for (size_t i = 0; i < ids.size(); ++i) {
      for (size_t j = i + 1; j < ids.size(); ++j) {
        const int a = ids[i];
        const int b = ids[j];
        const auto& path_a = paths_.at(a);
        const auto& path_b = paths_.at(b);

        for (size_t ia = 0; ia < path_a.poses.size(); ++ia) {
          const double ha = headingAt(path_a, ia);
          for (size_t ib = 0; ib < path_b.poses.size(); ++ib) {
            const double d = poseDistance(path_a.poses[ia], path_b.poses[ib]);
            if (d > conflict_distance_) continue;
            const double hb = headingAt(path_b, ib);
            if (std::abs(wrapAngle(ha - hb)) < heading_conflict_min_rad_) continue;

            const double mx =
                0.5 * (path_a.poses[ia].pose.position.x + path_b.poses[ib].pose.position.x);
            const double my =
                0.5 * (path_a.poses[ia].pose.position.y + path_b.poses[ib].pose.position.y);
            out.push_back(ConflictSample{a, ia, b, ib, mx, my});
          }
        }
      }
    }
    return out;
  }

  std::vector<ZoneIntervals> buildZones(
      const std::vector<ConflictSample>& conflicts,
      const std::map<int, std::vector<double>>& arc_map) const {
    struct Cluster {
      double mx{0.0};
      double my{0.0};
      std::vector<ConflictSample> samples;
    };

    std::vector<ZoneIntervals> zones;
    if (conflicts.empty()) return zones;

    std::vector<Cluster> clusters;
    for (const auto& s : conflicts) {
      bool placed = false;
      for (auto& c : clusters) {
        if (std::hypot(s.mx - c.mx, s.my - c.my) <= zone_merge_distance_) {
          c.samples.push_back(s);
          const double n = static_cast<double>(c.samples.size());
          c.mx = ((n - 1.0) * c.mx + s.mx) / n;
          c.my = ((n - 1.0) * c.my + s.my) / n;
          placed = true;
          break;
        }
      }
      if (!placed) {
        Cluster c;
        c.mx = s.mx;
        c.my = s.my;
        c.samples.push_back(s);
        clusters.push_back(std::move(c));
      }
    }

    for (const auto& c : clusters) {
      std::map<int, std::vector<size_t>> indices_by_cav;
      for (const auto& s : c.samples) {
        indices_by_cav[s.cav_a].push_back(s.idx_a);
        indices_by_cav[s.cav_b].push_back(s.idx_b);
      }
      if (indices_by_cav.size() < 2) continue;

      ZoneIntervals zone;
      for (auto& kv : indices_by_cav) {
        const int cav_id = kv.first;
        auto arc_it = arc_map.find(cav_id);
        if (arc_it == arc_map.end() || arc_it->second.empty()) continue;
        auto& idxs = kv.second;
        std::sort(idxs.begin(), idxs.end());
        size_t mid_idx = idxs[idxs.size() / 2];
        mid_idx = std::min(mid_idx, arc_it->second.size() - 1);

        const double s_mid = arc_it->second[mid_idx];
        const double s_enter = std::max(0.0, s_mid - zone_half_window_m_);
        const double s_exit =
            std::max(s_enter, std::min(arc_it->second.back(), s_mid + zone_half_window_m_));
        zone[cav_id] = std::make_pair(s_enter, s_exit);
      }
      if (zone.size() >= 2) zones.push_back(std::move(zone));
    }

    return zones;
  }

  void publishCaps() {
    const auto now = this->now();
    const double elapsed_since_start = (now - node_start_time_).seconds();
    const double elapsed_since_plan = (now - plan_start_time_).seconds();

    for (const int cav_id : cav_ids_) {
      double cap = nominalSpeed(cav_id);
      if (!plan_ready_) {
        if (elapsed_since_start < plan_timeout_sec_) cap = 0.0;
      } else {
        const double delay = start_delay_map_[cav_id];
        if (elapsed_since_plan < delay) cap = 0.0;
      }
      std_msgs::msg::Float64 msg;
      msg.data = std::max(0.0, cap);
      cap_pubs_[cav_id]->publish(msg);
    }
  }

  void publishStatus(const std::string& state) {
    std::ostringstream missing_oss;
    bool any_missing = false;
    for (const int cav_id : cav_ids_) {
      if (paths_.count(cav_id) > 0) continue;
      if (any_missing) missing_oss << ",";
      missing_oss << cav_id;
      any_missing = true;
    }

    std::ostringstream oss;
    oss << "state=" << state
        << " plan_ready=" << (plan_ready_ ? "1" : "0")
        << " missing=" << (any_missing ? missing_oss.str() : "-")
        << " delays=";
    for (size_t i = 0; i < cav_ids_.size(); ++i) {
      const int cav_id = cav_ids_[i];
      if (i) oss << ",";
      oss << cav_id << ":" << start_delay_map_[cav_id];
    }
    std_msgs::msg::String msg;
    msg.data = oss.str();
    status_pub_->publish(msg);
  }

  void publishStatusTick() { publishStatus(planner_state_); }

  double nominalSpeed(int cav_id) const {
    auto it = nominal_speed_map_.find(cav_id);
    if (it == nominal_speed_map_.end()) return default_speed_mps_;
    return std::max(0.05, it->second);
  }

  std::vector<int> cav_ids_;
  std::map<int, nav_msgs::msg::Path> paths_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr> path_subs_;
  std::map<int, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> cap_pubs_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr plan_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  std::map<int, double> nominal_speed_map_;
  std::map<int, double> start_delay_map_;

  double default_speed_mps_{0.55};
  double conflict_distance_{0.45};
  double heading_conflict_min_rad_{12.0 * M_PI / 180.0};
  double zone_merge_distance_{0.55};
  double zone_half_window_m_{0.35};
  double safety_margin_sec_{0.9};
  double max_start_delay_sec_{25.0};
  double publish_hz_{10.0};
  double plan_timeout_sec_{8.0};

  bool plan_ready_{false};
  std::string planner_state_{"boot"};
  rclcpp::Time node_start_time_;
  rclcpp::Time plan_start_time_;
};

}  // namespace bisa

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<bisa::OfflineSpeedScheduler>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
