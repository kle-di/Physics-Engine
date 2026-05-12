#pragma once

#include "Physics-Engine/collision/Collision.h"
#include "Physics-Engine/dynamics/RigidBody.h"

namespace PhysicsEngine::dynamics
{
    struct CollisionResolverConfig
    {
        int velocityIterations{ 8 };
        int positionIterations{ 3 };
        float restitutionVelocityThreshold{ 1.0f };
        float positionCorrectionPercent{ 0.2f };
        float positionSlop{ 0.01f };
    };

    class CollisionResolver
    {
    public:
        explicit CollisionResolver(const CollisionResolverConfig& config = {});

        [[nodiscard]] const CollisionResolverConfig& GetConfig() const;

        void ResolveVelocity(
            RigidBody& bodyA,
            RigidBody& bodyB,
            const collision::ContactManifold& manifold) const;

        void CorrectPosition(
            RigidBody& bodyA,
            RigidBody& bodyB,
            const collision::ContactManifold& manifold) const;

    private:
        CollisionResolverConfig m_config;
    };
}
