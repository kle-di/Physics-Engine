#include "Physics-Engine/dynamics/RigidBody.h"

#include <cmath>

namespace PhysicsEngine::dynamics
{
    namespace
    {
        constexpr float kMinDynamicValue = 1.0e-6f;
        constexpr float kMaxLinearSpeed = 24.0f;
        constexpr float kMaxAngularSpeed = 30.0f;
    }

    RigidBody::RigidBody(const RigidBodyDesc& desc)
        : position(desc.position)
        , rotation(desc.rotation)
        , linearVelocity(desc.linearVelocity)
        , angularVelocity(desc.angularVelocity)
        , restitution(desc.restitution)
        , friction(desc.friction)
    {
        SetMass(desc.mass);
        SetInertia(desc.inertia);
    }

    void RigidBody::SetMass(float massValue)
    {
        mass = massValue;
        inverseMass = massValue > kMinDynamicValue ? 1.0f / massValue : 0.0f;
    }

    void RigidBody::SetInertia(float inertiaValue)
    {
        inertia = inertiaValue;
        inverseInertia = inertiaValue > kMinDynamicValue ? 1.0f / inertiaValue : 0.0f;
    }

    void RigidBody::ApplyForce(const math::Vec2& force)
    {
        if (IsStatic())
        {
            return;
        }

        forceAccumulator += force;
    }

    void RigidBody::ApplyImpulse(const math::Vec2& impulse, const math::Vec2& contactVector)
    {
        if (IsStatic())
        {
            return;
        }

        linearVelocity += impulse * inverseMass;
        angularVelocity += inverseInertia * math::Vec2::Cross(contactVector, impulse);
    }

    void RigidBody::IntegrateForces(float dt)
    {
        if (IsStatic())
        {
            return;
        }

        linearVelocity += forceAccumulator * (inverseMass * dt);
        angularVelocity += torqueAccumulator * inverseInertia * dt;
    }

    void RigidBody::IntegrateVelocity(float dt)
    {
        if (IsStatic())
        {
            return;
        }

        const float linearSpeedSq = linearVelocity.LengthSquared();
        const float maxLinearSpeedSq = kMaxLinearSpeed * kMaxLinearSpeed;
        if (linearSpeedSq > maxLinearSpeedSq)
        {
            const float linearSpeed = std::sqrt(linearSpeedSq);
            linearVelocity *= (kMaxLinearSpeed / linearSpeed);
        }

        if (std::abs(angularVelocity) > kMaxAngularSpeed)
        {
            angularVelocity = angularVelocity > 0.0f ? kMaxAngularSpeed : -kMaxAngularSpeed;
        }

        position += linearVelocity * dt;
        rotation += angularVelocity * dt;

        ClearAccumulators();
    }

    void RigidBody::ClearAccumulators()
    {
        forceAccumulator = {};
        torqueAccumulator = 0.0f;
    }

    bool RigidBody::IsStatic() const
    {
        return inverseMass == 0.0f;
    }
}
