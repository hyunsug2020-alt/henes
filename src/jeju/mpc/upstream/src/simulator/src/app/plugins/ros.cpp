
// #include <rclcpp/rclcpp.hpp>
// #include <std_msgs/msg/string.hpp>
// #include "my_custom_msgs_mc/msg/custom_message.hpp"

#include "pch.hpp"
#include "engine/engine.hpp"
#include "app/plugins.hpp"
#include <chrono>
#include <string>
#include <unordered_map>
#include <vector>

namespace ROSSystem {

    namespace {
        struct CAVStateSnapshot {
            int cav_id{0};
            real x{0.0f};
            real y{0.0f};
        };

        bool should_log_physics_off(EntityID id) {
            static std::unordered_map<EntityID, std::chrono::steady_clock::time_point> last_log;
            const auto now = std::chrono::steady_clock::now();
            const auto it = last_log.find(id);
            if (it == last_log.end() || (now - it->second) > std::chrono::seconds(1)) {
                last_log[id] = now;
                return true;
            }
            return false;
        }

        int parse_cav_id_from_topic(const std::string &topic) {
            // Expected: /sim/cavXX/accel
            const std::string key = "/sim/cav";
            const auto p = topic.find(key);
            if (p == std::string::npos) return 0;
            const size_t i = p + key.size();
            if (i + 1 >= topic.size()) return 0;
            if (!std::isdigit(static_cast<unsigned char>(topic[i])) ||
                !std::isdigit(static_cast<unsigned char>(topic[i + 1]))) {
                return 0;
            }
            return (topic[i] - '0') * 10 + (topic[i + 1] - '0');
        }

        real priority_speed_scale(
            int cav_id, real x, real y,
            const std::vector<CAVStateSnapshot> &all_cavs) {
            // Priority: CAV01 > CAV02 > CAV03 > CAV04 (smaller id is higher).
            static std::unordered_map<int, bool> yielding_state;
            constexpr real yield_start_dist = 1.0f;
            constexpr real yield_release_dist = 1.6f;
            constexpr real yield_speed_scale = 0.35f;
            // Apply yielding only around merge/roundabout area to avoid unnecessary
            // slowdowns on far straight sections.
            constexpr real conflict_center_x = 0.574f;
            constexpr real conflict_center_y = -0.291f;
            constexpr real conflict_region_radius = 2.3f;

            if (cav_id <= 1) return 1.0f;

            const real ego_dx = x - conflict_center_x;
            const real ego_dy = y - conflict_center_y;
            const real ego_d = std::sqrt(ego_dx * ego_dx + ego_dy * ego_dy);
            if (ego_d > conflict_region_radius) {
                yielding_state[cav_id] = false;
                return 1.0f;
            }

            real min_higher_dist = 1e9f;
            for (const auto &s : all_cavs) {
                if (s.cav_id <= 0 || s.cav_id >= cav_id) continue;
                const real hx = s.x - conflict_center_x;
                const real hy = s.y - conflict_center_y;
                const real hd = std::sqrt(hx * hx + hy * hy);
                if (hd > conflict_region_radius) continue;
                const real dx = x - s.x;
                const real dy = y - s.y;
                const real d = std::sqrt(dx * dx + dy * dy);
                if (d < min_higher_dist) min_higher_dist = d;
            }

            bool yielding = yielding_state[cav_id];
            if (!yielding && min_higher_dist < yield_start_dist) {
                yielding = true;
            } else if (yielding && min_higher_dist > yield_release_dist) {
                yielding = false;
            }
            yielding_state[cav_id] = yielding;

            return yielding ? yield_speed_scale : 1.0f;
        }
    } // namespace

    void init(ECS &ecs, World &world, real dt) {
    }

    void rk4_step_pose(real x, real y, real yaw, real v, real omega, real dt,
                       real &x_next, real &y_next, real &yaw_next) {
        auto f = [&](real /*X*/, real /*Y*/, real Yaw, real &dX, real &dY, real &dYaw) {
            dX = v * std::cos(Yaw);
            dY = v * std::sin(Yaw);
            dYaw = omega;
        };

        real k1x, k1y, k1yaw;
        real k2x, k2y, k2yaw;
        real k3x, k3y, k3yaw;
        real k4x, k4y, k4yaw;

        f(x, y, yaw, k1x, k1y, k1yaw);
        f(x + 0.5f * dt * k1x, y + 0.5f * dt * k1y, yaw + 0.5f * dt * k1yaw, k2x, k2y, k2yaw);
        f(x + 0.5f * dt * k2x, y + 0.5f * dt * k2y, yaw + 0.5f * dt * k2yaw, k3x, k3y, k3yaw);
        f(x + dt * k3x, y + dt * k3y, yaw + dt * k3yaw, k4x, k4y, k4yaw);

        const real w = dt / 6.0f;
        x_next = x + w * (k1x + 2.0f * k2x + 2.0f * k3x + k4x);
        y_next = y + w * (k1y + 2.0f * k2y + 2.0f * k3y + k4y);
        yaw_next = yaw + w * (k1yaw + 2.0f * k2yaw + 2.0f * k3yaw + k4yaw);
    }

    void receiveData(ECS &ecs, World &world, real dt) {
        std::vector<CAVStateSnapshot> all_cavs;
        {
            auto snap_view = ecs.write<Vehicle, State, ROSTopic, Physics>();
            snap_view.iterate([&](EntityID, Vehicle &vehicle, State &state, ROSTopic &topic, Physics &physics) {
                if (!vehicle.AV || !physics.on) return;
                const int cav_id = parse_cav_id_from_topic(topic.subscriber);
                if (cav_id <= 0) return;
                all_cavs.push_back(CAVStateSnapshot{cav_id, state.position.x, state.position.y});
            });
        }

        auto view = ecs.write<Vehicle, State, ROSTopic, ROSData, Physics>();
        view.iterate([&](EntityID id, Vehicle &vehicle, State &state, ROSTopic &topic, ROSData &data, Physics &physics) {
            if (!physics.on) {
                if (should_log_physics_off(id)) {
                    std::cout << "[DBG][ROS] physics.off -> skip control apply, id=" << id
                              << ", sub_topic=" << topic.subscriber
                              << ", pos=(" << state.position.x << ", " << state.position.y << ")\n";
                }
                return;
            }

            // newData: geometry_msgs::msg::AccelStamped
            InputData newData = world.ros->receiveTopic(topic.subscriber);
            if (newData.is_cav) {
                if (newData.received) {
                    data.linear_velocity = newData.linear_velocity;
                    data.angular_velocity = newData.angular_velocity;
                    const int cav_id = parse_cav_id_from_topic(topic.subscriber);
                    if (cav_id > 0) {
                        const real scale = priority_speed_scale(
                            cav_id, state.position.x, state.position.y, all_cavs);
                        data.linear_velocity *= scale;
                    }
                    data.updated = true;
                    ecs.markChanged<ROSData>(id);
                    newData.received = false;
                }

                real yaw_temp = glm::eulerAngles(state.rotation).z;
                rk4_step_pose(state.position.x, state.position.y, yaw_temp,
                              data.linear_velocity, data.angular_velocity, dt,
                              state.position.x, state.position.y, yaw_temp);
                state.rotation = glm::angleAxis(static_cast<float>(yaw_temp), vec3(0, 0, 1));
                ecs.markChanged<State>(id);
            } else {
                if (newData.received) {
                    state.position.x = newData.position_x;
                    state.position.y = newData.position_y;
                    state.rotation = glm::angleAxis(static_cast<float>(newData.yaw), vec3(0, 0, 1));
                    data.updated = true;
                    ecs.markChanged<ROSData>(id);
                    ecs.markChanged<State>(id);
                    newData.received = false;

                    // if (topic.subscriber.find("19") != std::string::npos) {
                    //     std::cout<<"[LOG] HV19 received pose update from ROS2: {"
                    //              << "x: " << state.position.x
                    //              << ", y: " << state.position.y
                    //              << "}\n";
                    // }
                }
            }
        });
    }

    void sendData(ECS &ecs, World &world, real dt) {
        auto view = ecs.write<Vehicle, State, Motion, ROSTopic, Physics>();
        view.iterate([&](EntityID id, Vehicle &vehicle, State &state, Motion &motion, ROSTopic &topic, Physics &physics) {
            if (!physics.on) {
                if (should_log_physics_off(id)) {
                    std::cout << "[DBG][ROS] physics.off -> skip pose publish, id=" << id
                              << ", pub_topic=" << topic.publisher
                              << ", pos=(" << state.position.x << ", " << state.position.y << ")\n";
                }
                return;
            }

            if (vehicle.AV == false)
                return;

            OutputData data;
            data.position_x = state.position.x;
            data.position_y = state.position.y;
            real yaw = glm::eulerAngles(state.rotation).z;
            data.yaw = yaw;
            world.ros->sendTopic(topic.publisher, data);
        });
    }

}; // namespace ROSSystem

class Engine;
void ROSPlusgin(Engine &engine) {
    engine.addLogicSystem(SystemType::INIT, ROSSystem::init);
    engine.addLogicSystem(SystemType::UPDATE, ROSSystem::receiveData);
    engine.addLogicSystem(SystemType::UPDATE, ROSSystem::sendData);
};
