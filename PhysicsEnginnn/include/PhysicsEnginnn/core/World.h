#pragma once

#include <cstddef>
#include <variant>
#include <vector>

#include "PhysicsEnginnn/collision/Shapes.h"
#include "PhysicsEnginnn/dynamics/RigidBody.h"
#include "PhysicsEnginnn/forces/ForceGenerator.h"
#include "PhysicsEnginnn/forces/ForceRegistry.h"

namespace PhysicsEnginnn::core
{
    class World
    {
    public:
        struct BodyEntry
        {
            dynamics::RigidBody body{};
            std::variant<collision::CircleShape, collision::PolygonShape> shape;

            math::Transform GetTransform() const;
        };

        /**
         * @param fixedTimeStep Fixed simulation step in seconds.
         */
        explicit World(float fixedTimeStep = 1.0f / 60.0f);

        /**
         * @param desc Body descriptor.
         * @param circle Circle collision shape.
         * @return Index of the created body entry.
         */
        std::size_t CreateCircleBody(const dynamics::RigidBodyDesc& desc, const collision::CircleShape& circle);
        /**
         * @param desc Body descriptor.
         * @param polygon Polygon collision shape.
         * @return Index of the created body entry.
         */
        std::size_t CreatePolygonBody(const dynamics::RigidBodyDesc& desc, const collision::PolygonShape& polygon);

        /**
         * @param gravityValue New gravity vector.
         */
        void SetGravity(const math::Vec2& gravityValue);
        /**
         * @param dt Frame delta time in seconds.
         */
        void Step(float dt);

        /** @return Read-only list of world bodies and shapes. */
        const std::vector<BodyEntry>& GetBodies() const;

    private:
        void StepFixed(float dt);
        void RebuildForceRegistry();

        float m_fixedTimeStep;
        float m_accumulator{ 0.0f };
        math::Vec2 m_gravity{ 0.0f, 9.81f };
        std::vector<BodyEntry> m_bodies;
        forces::GravityForceGenerator m_gravityGenerator{ m_gravity };
        forces::ForceRegistry m_forceRegistry;
    };
}
