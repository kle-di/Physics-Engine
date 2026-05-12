#pragma once

#include "Physics-Engine/collision/Collision.h"
#include "Physics-Engine/dynamics/RigidBody.h"

namespace PhysicsEngine::ccd
{
    struct CcdSettings
    {
        bool enabled{ true };
        float bulletSpeedThreshold{ 4.0f };
        float minTravelDistance{ 0.02f };
        float toiEpsilon{ 1.0e-4f };
    };

    struct ToiResult
    {
        bool hit{ false };
        float toi{ 1.0f };
        collision::ContactManifold manifold{};
    };

    [[nodiscard]] bool IsBulletBody(const dynamics::RigidBody& body, float dt, const CcdSettings& settings);

    [[nodiscard]] collision::Aabb ComputeSweptAabb(
        const collision::CollisionShape& shape,
        const math::Transform& start,
        const math::Transform& end);

    [[nodiscard]] ToiResult ComputeToi(
        const collision::CollisionShape& shapeA,
        const math::Transform& startA,
        const math::Transform& endA,
        const collision::CollisionShape& shapeB,
        const math::Transform& startB,
        const math::Transform& endB,
        float dt,
        const CcdSettings& settings);
}
