#pragma once

#include "Physics-Engine/dynamics/RigidBody.h"
#include "Physics-Engine/math/Vec2.h"

namespace PhysicsEngine::forces
{
    class ForceGenerator
    {
    public:
        virtual ~ForceGenerator() = default;
        /**
         * @param body Body to update.
         * @param dt Time step in seconds.
         */
        virtual void UpdateForce(dynamics::RigidBody& body, float dt) const = 0;
    };

    class GravityForceGenerator final : public ForceGenerator
    {
    public:
        /**
         * @param gravity Gravity vector.
         */
        explicit GravityForceGenerator(const math::Vec2& gravity = { 0.0f, 9.81f })
            : m_gravity(gravity)
        {
        }

        /**
         * @param gravity New gravity vector.
         */
        void SetGravity(const math::Vec2& gravity)
        {
            m_gravity = gravity;
        }

        /** @return Current gravity vector. */
        [[nodiscard]] const math::Vec2& GetGravity() const
        {
            return m_gravity;
        }

        void UpdateForce(dynamics::RigidBody& body, float) const override
        {
            if (body.IsStatic())
            {
                return;
            }

            body.ApplyForce(m_gravity * body.mass);
        }

    private:
        math::Vec2 m_gravity;
    };
}
