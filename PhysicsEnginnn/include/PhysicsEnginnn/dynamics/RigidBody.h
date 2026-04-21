#pragma once

#include "PhysicsEnginnn/math/Vec2.h"

namespace PhysicsEnginnn::dynamics
{
    struct RigidBodyDesc
    {
        math::Vec2 position{};
        float rotation{ 0.0f };

        math::Vec2 linearVelocity{};
        float angularVelocity{ 0.0f };

        float mass{ 1.0f };
        float inertia{ 1.0f };

        float restitution{ 0.2f };
        float friction{ 0.5f };
    };

    class RigidBody
    {
    public:
        /**
         * @param desc Initial body properties.
         */
        explicit RigidBody(const RigidBodyDesc& desc = {});

        /**
         * @param mass New mass value.
         */
        void SetMass(float mass);

        /**
         * @param inertia New inertia value.
         */
        void SetInertia(float inertia);

        /**
         * @param force Force applied at center of mass.
         */
        void ApplyForce(const math::Vec2& force);

        /**
         * @param impulse Instantaneous impulse.
         * @param contactVector Offset from center of mass to contact point.
         */
        void ApplyImpulse(const math::Vec2& impulse, const math::Vec2& contactVector);

        /**
         * @param dt Time step in seconds.
         */
        void IntegrateForces(float dt);

        /**
         * @param dt Time step in seconds.
         */
        void IntegrateVelocity(float dt);
        void ClearAccumulators();

        /** @return True when body is static (non-dynamic). */
        bool IsStatic() const;

        math::Vec2 position{};
        float rotation{ 0.0f };

        math::Vec2 linearVelocity{};
        float angularVelocity{ 0.0f };

        math::Vec2 forceAccumulator{};
        float torqueAccumulator{ 0.0f };

        float mass{ 1.0f };
        float inverseMass{ 1.0f };

        float inertia{ 1.0f };
        float inverseInertia{ 1.0f };

        float restitution{ 0.2f };
        float friction{ 0.5f };
    };
}
