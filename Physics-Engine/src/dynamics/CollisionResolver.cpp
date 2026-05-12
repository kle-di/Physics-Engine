#include "Physics-Engine/dynamics/CollisionResolver.h"

#include <algorithm>
#include <cmath>

namespace PhysicsEngine::dynamics
{
    namespace
    {
        constexpr float kEpsilon = 1.0e-6f;

        [[nodiscard]] math::Vec2 VelocityAtContact(const RigidBody& body, const math::Vec2& contactVector)
        {
            return body.linearVelocity + math::Vec2::Cross(body.angularVelocity, contactVector);
        }
    }

    CollisionResolver::CollisionResolver(const CollisionResolverConfig& config)
        : m_config(config)
    {
    }

    const CollisionResolverConfig& CollisionResolver::GetConfig() const
    {
        return m_config;
    }

    void CollisionResolver::ResolveVelocity(
        RigidBody& bodyA,
        RigidBody& bodyB,
        const collision::ContactManifold& manifold) const
    {
        if (manifold.pointCount <= 0 || (bodyA.IsStatic() && bodyB.IsStatic()))
        {
            return;
        }

        bodyA.SetAwake(true);
        bodyB.SetAwake(true);

        const float restitution = std::min(bodyA.restitution, bodyB.restitution);
        const float friction = std::sqrt(std::max(0.0f, bodyA.friction * bodyB.friction));

        for (int pointIndex = 0; pointIndex < manifold.pointCount; ++pointIndex)
        {
            const math::Vec2 contactPoint = manifold.points[pointIndex];
            const math::Vec2 ra = contactPoint - bodyA.position;
            const math::Vec2 rb = contactPoint - bodyB.position;

            math::Vec2 relativeVelocity = VelocityAtContact(bodyB, rb) - VelocityAtContact(bodyA, ra);
            const float normalVelocity = math::Vec2::Dot(relativeVelocity, manifold.normal);
            if (normalVelocity > 0.0f)
            {
                continue;
            }

            const float raCrossN = math::Vec2::Cross(ra, manifold.normal);
            const float rbCrossN = math::Vec2::Cross(rb, manifold.normal);
            const float normalMass = bodyA.inverseMass
                + bodyB.inverseMass
                + raCrossN * raCrossN * bodyA.inverseInertia
                + rbCrossN * rbCrossN * bodyB.inverseInertia;

            if (normalMass <= kEpsilon)
            {
                continue;
            }

            float effectiveRestitution = restitution;
            if (std::abs(normalVelocity) < m_config.restitutionVelocityThreshold)
            {
                effectiveRestitution = 0.0f;
            }

            float normalImpulseMagnitude = -(1.0f + effectiveRestitution) * normalVelocity;
            normalImpulseMagnitude /= normalMass;
            normalImpulseMagnitude /= static_cast<float>(manifold.pointCount);

            const math::Vec2 normalImpulse = manifold.normal * normalImpulseMagnitude;
            bodyA.ApplyImpulse(normalImpulse * -1.0f, ra);
            bodyB.ApplyImpulse(normalImpulse, rb);

            relativeVelocity = VelocityAtContact(bodyB, rb) - VelocityAtContact(bodyA, ra);
            const math::Vec2 tangentRaw = relativeVelocity - manifold.normal * math::Vec2::Dot(relativeVelocity, manifold.normal);
            if (tangentRaw.LengthSquared() <= kEpsilon)
            {
                continue;
            }

            const math::Vec2 tangent = tangentRaw.Normalized();
            const float raCrossT = math::Vec2::Cross(ra, tangent);
            const float rbCrossT = math::Vec2::Cross(rb, tangent);
            const float tangentMass = bodyA.inverseMass
                + bodyB.inverseMass
                + raCrossT * raCrossT * bodyA.inverseInertia
                + rbCrossT * rbCrossT * bodyB.inverseInertia;

            if (tangentMass <= kEpsilon)
            {
                continue;
            }

            float tangentImpulseMagnitude = -math::Vec2::Dot(relativeVelocity, tangent);
            tangentImpulseMagnitude /= tangentMass;
            tangentImpulseMagnitude /= static_cast<float>(manifold.pointCount);

            const float maxFriction = friction * normalImpulseMagnitude;
            tangentImpulseMagnitude = std::clamp(tangentImpulseMagnitude, -maxFriction, maxFriction);

            const math::Vec2 tangentImpulse = tangent * tangentImpulseMagnitude;
            bodyA.ApplyImpulse(tangentImpulse * -1.0f, ra);
            bodyB.ApplyImpulse(tangentImpulse, rb);
        }
    }

    void CollisionResolver::CorrectPosition(
        RigidBody& bodyA,
        RigidBody& bodyB,
        const collision::ContactManifold& manifold) const
    {
        if (manifold.penetration <= 0.0f)
        {
            return;
        }

        const float inverseMassSum = bodyA.inverseMass + bodyB.inverseMass;
        if (inverseMassSum <= kEpsilon)
        {
            return;
        }

        const float correctionMagnitude = std::max(manifold.penetration - m_config.positionSlop, 0.0f)
            * m_config.positionCorrectionPercent
            / inverseMassSum;

        const math::Vec2 correction = manifold.normal * correctionMagnitude;
        bodyA.position -= correction * bodyA.inverseMass;
        bodyB.position += correction * bodyB.inverseMass;
    }
}
