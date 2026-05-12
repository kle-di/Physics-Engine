#pragma once

#include <cstddef>
#include <variant>
#include <vector>

#include "Physics-Engine/broadphase/UniformGridBroadPhase.h"
#include "Physics-Engine/ccd/Ccd.h"
#include "Physics-Engine/collision/Collision.h"
#include "Physics-Engine/collision/Shapes.h"
#include "Physics-Engine/dynamics/CollisionResolver.h"
#include "Physics-Engine/dynamics/RigidBody.h"
#include "Physics-Engine/forces/ForceGenerator.h"
#include "Physics-Engine/forces/ForceRegistry.h"

namespace PhysicsEngine::core
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

        struct ContactEntry
        {
            std::size_t bodyA{ 0 };
            std::size_t bodyB{ 0 };
            collision::ContactManifold manifold{};
        };

        /**
         * @param fixedTimeStep Fixed simulation step in seconds.
         */
        explicit World(float fixedTimeStep = 1.0f / 120.0f);

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
        /**
         * @param bodyIndex Body to drag.
         * @param localGrabPoint Body-local grab point.
         * @param targetWorldPoint Desired world-space drag target.
         * @param dt Frame delta time in seconds.
         * @param stiffness Drag spring stiffness.
         * @param damping Drag damping.
         * @param maxImpulse Maximum impulse applied per step.
         */
        void ApplyMouseDrag(
            std::size_t bodyIndex,
            const math::Vec2& localGrabPoint,
            const math::Vec2& targetWorldPoint,
            float dt,
            float stiffness,
            float damping,
            float maxImpulse);

        /** @return Read-only list of world bodies and shapes. */
        const std::vector<BodyEntry>& GetBodies() const;
        /** @return Read-only list of contacts from the latest fixed step. */
        const std::vector<ContactEntry>& GetContacts() const;

    private:
        void StepFixed(float dt);
        void RebuildForceRegistry();

        float m_fixedTimeStep;
        float m_accumulator{ 0.0f };
        math::Vec2 m_gravity{ 0.0f, 9.81f };
        std::vector<BodyEntry> m_bodies;
        std::vector<ContactEntry> m_contacts;
        forces::GravityForceGenerator m_gravityGenerator{ m_gravity };
        forces::ForceRegistry m_forceRegistry;
        dynamics::CollisionResolver m_collisionResolver{};
        broadphase::UniformGridBroadPhase m_broadPhase{ 1.0f };
        ccd::CcdSettings m_ccdSettings{};
    };
}
