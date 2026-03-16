#include "pch.hpp"
#include "engine/engine.hpp"
#include "app/plugins.hpp"
#include "app/functions/vehicle.hpp"

namespace VehicleSystem {

    EntityID spawn(ECS &ecs, World &world, VehicleData &data) {
        quat rotation = quat(1.0f, 0.0f, 0.0f, 0.0f);
        vec3 position = vec3(data.position, 0.0f);
        VehicleFunctions::findClosestLane(ecs, position, rotation);
        const auto &worldState = world.source.get<WorldState>();
        const bool physics_on_init = (worldState.state == WorldState::run);
        EntityID id = ecs.createEntity();
        ecs.add(id, Vehicle{.AV = data.av});
        ecs.add(id, State{.position = position,
                          .previous = position,
                          .rotation = rotation});
        ecs.add(id, StartingPosition{.position = position});
        ecs.add(id, Motion{.speed = vec3(0.0f, 0.0f, 0.0f)});
        ecs.add(id, Physics{.on = physics_on_init});
        ecs.add(id, Mesh{.name = "vehicle",
                         .color = vec3(0.3f, 0.3f, 0.3f) + vec3(!data.av * 0.7f, 0.0f, data.av * 0.7f)});
        ecs.add(id, MouseMesh{.bound = vec3(0.4f)});
        ecs.add(id, Interactive{.clickable = true,
                                .draggable = true});

        // ecs.add(id, Timer{.life = 1.0f / 20.0f});

        auto &size = world.source.get<CollisionBoxSize>();
        std::vector<vec2> points;
        points.push_back({size.LF, size.LW});
        points.push_back({-size.LR, size.LW});
        points.push_back({-size.LR, -size.LW});
        points.push_back({size.LF, -size.LW});

        ecs.add(id, CollisionMesh{.points = points});
        if (data.av) {
            auto &av = world.source.get<AVQueue>();
            av.queue.push_back(id);
        } else {
            auto &hv = world.source.get<HVQueue>();
            hv.queue.push_back(id);
        }
        auto names = world.ros->createEntity(data.av);
        ecs.add(id, ROSTopic{.publisher = names.pub,
                             .subscriber = names.sub,
                             .AV = names.is_cav});

        ecs.add(id, ROSData{.updated = false});
        auto &topic = ecs.get<ROSTopic>(id);
        if (data.av)
            std::cout << "[LOG] " << "created CAV and pub 3D pose :{" << topic.publisher << "}\n";
        else
            std::cout << "[LOG] " << "created HV moving :{" << topic.subscriber << "}\n";
        std::cout << "[DBG][SPAWN] physics.on=" << (physics_on_init ? "true" : "false")
                  << ", id=" << id << "\n";

        EntityID textID = ecs.createEntity();
        ecs.add(id, TextTag{.id = textID});
        size_t cycle = (names.id - 1) / 10;
        size_t frame = (names.id - 1) % 10;
        ecs.add(textID, State{.position = position + vec3(0.0, 0.0, 0.2)});
        ecs.add(textID, Sprite{.name = "id",
                               .size = 1.0f,
                               .cycle = cycle,
                               .frame = frame});
        ecs.add(textID, SpriteType{.fixed = false});
        return id;
    }

    void init(ECS &ecs, World &world, real dt) {
        world.source.add<AVQueue>();
        world.source.add<HVQueue>();
    }

    void update(ECS &ecs, World &world, real dt) {
        auto textView = ecs.write<TextTag, State>();
        textView.iterate([&](EntityID id, TextTag &tag, State &state) {
            auto &type = ecs.get<SpriteType>(tag.id);
            if (!type.fixed) {
                auto &tagState = ecs.get<State>(tag.id);
                tagState.position.x = state.position.x;
                tagState.position.y = state.position.y;
                ecs.markChanged<State>(tag.id);
            }
        });
    }

    void dragVehicle(ECS &ecs, World &world, real dt) {
        auto &worldState = world.source.get<WorldState>();
        if (worldState.state == worldState.setup) {
            auto &mouse = world.source.get<Mouse>();
            auto view = ecs.write<Interactive, Vehicle, StartingPosition, State>();
            view.iterate([&](EntityID id, Interactive &interact, Vehicle &vehicle, StartingPosition &start, State &state) {
                if (!vehicle.AV)
                    return;
                if (interact.clicked && id == mouse.selected) {
                    if (mouse.vecDir.z != 0.0f) {
                        real t = -mouse.origin.z / mouse.vecDir.z;
                        vec3 intersection = mouse.origin + t * mouse.vecDir;
                        vec3 clamped = glm::clamp(vec3(intersection.x, intersection.y, 0.0f),
                                                  vec3(-5.75f, -3.0f, 0.0f),
                                                  vec3(5.75f, 3.0f, 0.0f));
                        

                        VehicleFunctions::findClosestLane(ecs, clamped, state.rotation);
                        state.position = clamped;
                        start.position = clamped;

                        ecs.markChanged<State>(id);
                        ecs.markChanged<StartingPosition>(id);
                    }
                } });
        }
    }

    void checkInsideMap(ECS &ecs, World &world, real dt) {
        auto view = ecs.write<Vehicle, State, Physics>();
        view.iterate([&](EntityID id, Vehicle &vehicle, State &state, Physics &physics) {
            constexpr real x_min = -5.75f;
            constexpr real x_max = 5.75f;
            constexpr real y_min = -3.0f;
            constexpr real y_max = 3.0f;
            // Allow a small tolerance to avoid false trip from tiny integration overshoot.
            constexpr real boundary_margin = 0.05f;

            const bool out_x = (state.position.x <= x_min - boundary_margin) ||
                               (state.position.x >= x_max + boundary_margin);
            const bool out_y = (state.position.y <= y_min - boundary_margin) ||
                               (state.position.y >= y_max + boundary_margin);
            if (!(out_x || out_y)) {
                return;
            }

            if (!physics.on) {
                return;
            }

            physics.on = false;
            if (out_x) {
                std::cout << "[DBG][MAP] physics.off set by x-boundary, id=" << id
                          << ", x=" << state.position.x
                          << ", y=" << state.position.y
                          << ", margin=" << boundary_margin << "\n";
            } else {
                std::cout << "[DBG][MAP] physics.off set by y-boundary, id=" << id
                          << ", x=" << state.position.x
                          << ", y=" << state.position.y
                          << ", margin=" << boundary_margin << "\n";
            }
            ecs.markChanged<Physics>(id);
        });
    }

    void spawnVehicle(ECS &ecs, World &world, Event &baseEvent) {
        SpawnVehicleEvent &event = static_cast<SpawnVehicleEvent &>(baseEvent);
        EntityID id = 0;
        if (event.manual) {
            auto &mouse = world.source.get<Mouse>();
            real t = -mouse.origin.z / mouse.vecDir.z;
            vec3 intersection = mouse.origin + t * mouse.vecDir;
            vec3 clamped = glm::clamp(vec3(intersection.x, intersection.y, 0.0f),
                                      vec3(-5.75f, -3.0f, 0.0f),
                                      vec3(5.75f, 3.0f, 0.0f));
            VehicleData data;
            data.position = clamped;
            data.av = event.AV;
            if (data.av) {
                auto &av = world.source.get<AVQueue>();
                if (av.queue.size() >= 18)
                    return;
            } else {
                auto &hv = world.source.get<HVQueue>();
                if (hv.queue.size() >= 18)
                    return;
            }
            id = spawn(ecs, world, data);
        } else {
            id = spawn(ecs, world, event.data);
        }
    }

    void removeVehicle(ECS &ecs, World &world, Event &baseEvent) {
        RemoveVehicleEvent &event = static_cast<RemoveVehicleEvent &>(baseEvent);
        EntityID id = 0;
        bool AV = false;
        bool found = false;
        if (event.AV) {
            auto &av = world.source.get<AVQueue>();
            if (!av.queue.empty()) {
                id = av.queue.back();
                av.queue.pop_back();
                AV = true;
                found = true;
            }
        } else {
            auto &hv = world.source.get<HVQueue>();
            if (!hv.queue.empty()) {
                id = hv.queue.back();
                hv.queue.pop_back();
                found = true;
            }
        }
        if (found) {
            auto &textTag = ecs.get<TextTag>(id);
            ecs.removeEntity(textTag.id);
            auto &topic = ecs.get<ROSTopic>(id);
            if (AV)
                std ::cout << "[LOG] " << "removed CAV sub:{" << topic.publisher << "}\n";
            else
                std ::cout << "[LOG] " << "removed HV sub:{" << topic.subscriber << "}\n";
            TopicName name{topic.publisher, topic.subscriber};
            world.ros->removeEntity(name, event.AV);
            ecs.removeEntity(id);
        }
    }

}; // namespace VehicleSystem

class Engine;
void VehiclePlugin(Engine &engine) {
    engine.addLogicSystem(SystemType::INIT, VehicleSystem::init);
    engine.addLogicSystem(SystemType::UPDATE, VehicleSystem::update);
    engine.addLogicSystem(SystemType::UPDATE, VehicleSystem::checkInsideMap);
    engine.addLogicSystem(SystemType::UPDATE, VehicleSystem::dragVehicle);

    engine.addEventHandler<SpawnVehicleEvent>(VehicleSystem::spawnVehicle);
    engine.addEventHandler<RemoveVehicleEvent>(VehicleSystem::removeVehicle);
};
