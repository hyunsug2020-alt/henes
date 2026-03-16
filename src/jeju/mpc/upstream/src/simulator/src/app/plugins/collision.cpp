#include "pch.hpp"
#include "engine/engine.hpp"
#include "app/plugins.hpp"

namespace CollisionFunctions {

    std::vector<vec2> getTransformedVertices(const std::vector<vec2> &vertices, const vec2 &position, const quat &rotation) {
        std::vector<vec2> transformed;
        transformed.reserve(vertices.size());
        for (auto &v : vertices) {
            vec3 p3(v.x, v.y, 0.0f);
            vec3 rotated = rotation * p3;
            transformed.emplace_back(
                rotated.x + position.x,
                rotated.y + position.y);
        }
        return transformed;
    }

    void projectPolygon(const std::vector<vec2> &verts, const glm::vec2 &axis, float &min, float &max) {
        min = max = glm::dot(verts[0], axis);
        for (size_t i = 1; i < verts.size(); ++i) {
            float proj = glm::dot(verts[i], axis);
            if (proj < min)
                min = proj;
            if (proj > max)
                max = proj;
        }
    }

    bool checkCollisionSAT(const std::vector<vec2> &polyA, const std::vector<vec2> &polyB) {
        auto testEdges = [&](const std::vector<vec2> &verts) {
            for (size_t i = 0; i < verts.size(); ++i) {
                vec2 p1 = verts[i];
                vec2 p2 = verts[(i + 1) % verts.size()];
                vec2 edge = p2 - p1;
                vec2 axis(-edge.y, edge.x);
                axis = glm::normalize(axis);

                float minA, maxA, minB, maxB;
                projectPolygon(polyA, axis, minA, maxA);
                projectPolygon(polyB, axis, minB, maxB);

                if (maxA < minB || maxB < minA)
                    return false;
            }
            return true;
        };
        return testEdges(polyA) && testEdges(polyB);
    }

} // namespace CollisionFunctions

namespace CollisionSystem {

    void initVehicleSpecs(ECS &ecs, World &world, real dt) {
        world.source.add<CollisionBoxSize>();

        std::string configPath = PATH_CONFIG + std::string("config.ini");
        CSimpleIniA ini;
        ini.SetUnicode();
        SI_Error rc = ini.LoadFile(configPath.c_str());
        if (rc < 0) {
            throw std::runtime_error("Failed to load config file");
        }
        auto &size = world.source.get<CollisionBoxSize>();
        size.LF = std::stof(ini.GetValue("collision", "lengthF", "0.17"));
        size.LR = std::stof(ini.GetValue("collision", "lengthR", "0.16"));
        size.LW = std::stof(ini.GetValue("collision", "lengthW", "0.075"));
    }

    void updateCollisions(ECS &ecs, World &world, real dt) {
        auto outer = ecs.write<State, CollisionMesh, Physics, ROSTopic>();
        auto inner = ecs.write<State, CollisionMesh, Physics, ROSTopic>();
        outer.iterate([&](EntityID idA, State &stateA, CollisionMesh &meshA, Physics &physicsA, ROSTopic &topicA) {
            inner.iterate([&](EntityID idB, State &stateB, CollisionMesh &meshB, Physics &physicsB, ROSTopic &topicB) {
                if (idA >= idB)
                    return;
                if (!physicsA.on && !physicsB.on)
                    return;

                auto vertsA = CollisionFunctions::getTransformedVertices(meshA.points, stateA.position, stateA.rotation);
                auto vertsB = CollisionFunctions::getTransformedVertices(meshB.points, stateB.position, stateB.rotation);
                if (CollisionFunctions::checkCollisionSAT(vertsA, vertsB)) {
                    std::string nameA = topicA.AV ? topicA.publisher : topicA.subscriber;
                    std::string nameB = topicB.AV ? topicB.publisher : topicB.subscriber;

                    std::cout << "[LOG] Collision between: [" << nameA << "] and [" << nameB << "]\n";

                    // 충돌한 두 차량만 멈춤 (전체 차량 동결 방지)
                    physicsA.on = false;
                    physicsB.on = false;
                    std::cout << "[DBG][COLLISION] physics.off: [" << nameA << "] and [" << nameB << "]\n";
                }
            });
        });
    };

} // namespace CollisionSystem

void CollisionPlugin(Engine &engine) {
    engine.addLogicSystem(SystemType::INIT, CollisionSystem::initVehicleSpecs);
    engine.addLogicSystem(SystemType::UPDATE, CollisionSystem::updateCollisions);
};
