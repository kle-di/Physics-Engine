#pragma once

#include <array>
#include <cstddef>
#include <variant>

#include "Physics-Engine/collision/Shapes.h"

namespace PhysicsEngine::collision
{
    struct Aabb
    {
        math::Vec2 min{};
        math::Vec2 max{};

        [[nodiscard]] bool Overlaps(const Aabb& other) const;
        [[nodiscard]] math::Vec2 ClosestPoint(const math::Vec2& point) const;
    };

    struct ContactManifold
    {
        math::Vec2 normal{};
        float penetration{ 0.0f };
        math::Vec2 point{};
        std::array<math::Vec2, 2> points{};
        int pointCount{ 0 };
    };

    using CollisionShape = std::variant<CircleShape, PolygonShape>;

    [[nodiscard]] Aabb ComputeAabb(const CircleShape& circle, const math::Transform& transform);
    [[nodiscard]] Aabb ComputeAabb(const PolygonShape& polygon, const math::Transform& transform);

    [[nodiscard]] bool CollideAabbVsAabb(const Aabb& a, const Aabb& b, ContactManifold& outManifold);

    [[nodiscard]] bool CollideCircleVsCircle(
        const CircleShape& a,
        const math::Transform& transformA,
        const CircleShape& b,
        const math::Transform& transformB,
        ContactManifold& outManifold);

    [[nodiscard]] bool CollideCircleVsAabb(
        const CircleShape& circle,
        const math::Transform& circleTransform,
        const Aabb& box,
        ContactManifold& outManifold);

    [[nodiscard]] bool CollideCircleVsPolygon(
        const CircleShape& circle,
        const math::Transform& circleTransform,
        const PolygonShape& polygon,
        const math::Transform& polygonTransform,
        ContactManifold& outManifold);

    [[nodiscard]] bool CollidePolygonVsPolygon(
        const PolygonShape& a,
        const math::Transform& transformA,
        const PolygonShape& b,
        const math::Transform& transformB,
        ContactManifold& outManifold);

    [[nodiscard]] bool GenerateContact(
        const CollisionShape& shapeA,
        const math::Transform& transformA,
        const CollisionShape& shapeB,
        const math::Transform& transformB,
        ContactManifold& outManifold);
}
