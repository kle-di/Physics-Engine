#include "Physics-Engine/ccd/Ccd.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>

namespace PhysicsEngine::ccd
{
    namespace
    {
        [[nodiscard]] math::Transform LerpTransform(const math::Transform& start, const math::Transform& end, float t)
        {
            return math::Transform{
                start.position + (end.position - start.position) * t,
                start.rotation + (end.rotation - start.rotation) * t
            };
        }

        [[nodiscard]] math::Vec2 AabbCenter(const collision::Aabb& aabb)
        {
            return (aabb.min + aabb.max) * 0.5f;
        }

        [[nodiscard]] ToiResult ComputeCircleCircleToi(
            const collision::CircleShape& circleA,
            const math::Transform& startA,
            const math::Transform& endA,
            const collision::CircleShape& circleB,
            const math::Transform& startB,
            const math::Transform& endB,
            const CcdSettings& settings)
        {
            const math::Vec2 d0 = startA.position - startB.position;
            const math::Vec2 vRel = (endA.position - startA.position) - (endB.position - startB.position);
            const float radius = circleA.radius + circleB.radius;

            const float a = math::Vec2::Dot(vRel, vRel);
            const float b = 2.0f * math::Vec2::Dot(d0, vRel);
            const float c = math::Vec2::Dot(d0, d0) - radius * radius;

            if (c <= 0.0f)
            {
                ToiResult result{};
                result.hit = collision::CollideCircleVsCircle(circleA, startA, circleB, startB, result.manifold);
                result.toi = 0.0f;
                return result;
            }

            if (a <= settings.toiEpsilon)
            {
                return {};
            }

            const float discriminant = b * b - 4.0f * a * c;
            if (discriminant < 0.0f)
            {
                return {};
            }

            const float sqrtDiscriminant = std::sqrt(discriminant);
            const float inv2A = 0.5f / a;
            const float t0 = (-b - sqrtDiscriminant) * inv2A;
            const float t1 = (-b + sqrtDiscriminant) * inv2A;

            float toi = std::numeric_limits<float>::max();
            if (t0 >= 0.0f && t0 <= 1.0f)
            {
                toi = t0;
            }
            else if (t1 >= 0.0f && t1 <= 1.0f)
            {
                toi = t1;
            }

            if (toi == std::numeric_limits<float>::max())
            {
                return {};
            }

            const math::Transform toiA = LerpTransform(startA, endA, toi);
            const math::Transform toiB = LerpTransform(startB, endB, toi);

            ToiResult result{};
            if (!collision::CollideCircleVsCircle(circleA, toiA, circleB, toiB, result.manifold))
            {
                return {};
            }

            result.hit = true;
            result.toi = toi;
            return result;
        }

        [[nodiscard]] ToiResult ComputeCircleAabbToi(
            const collision::CircleShape& circle,
            const math::Transform& startCircle,
            const math::Transform& endCircle,
            const collision::Aabb& startAabb,
            const collision::Aabb& endAabb,
            const CcdSettings& settings)
        {
            const math::Vec2 circleStart = startCircle.position;
            const math::Vec2 circleDelta = endCircle.position - startCircle.position;
            const math::Vec2 boxDelta = AabbCenter(endAabb) - AabbCenter(startAabb);
            const math::Vec2 relativeDelta = circleDelta - boxDelta;

            const collision::Aabb expanded{
                startAabb.min - math::Vec2{ circle.radius, circle.radius },
                startAabb.max + math::Vec2{ circle.radius, circle.radius }
            };

            float tEnter = 0.0f;
            float tExit = 1.0f;
            math::Vec2 normal{};

            auto solveAxis = [&](float origin, float delta, float minBound, float maxBound, math::Vec2 normalNeg, math::Vec2 normalPos) -> bool
            {
                if (std::abs(delta) <= settings.toiEpsilon)
                {
                    return origin >= minBound && origin <= maxBound;
                }

                float t1 = (minBound - origin) / delta;
                float t2 = (maxBound - origin) / delta;
                math::Vec2 enterNormal = normalNeg;

                if (t1 > t2)
                {
                    std::swap(t1, t2);
                    enterNormal = normalPos;
                }

                if (t1 > tEnter)
                {
                    tEnter = t1;
                    normal = enterNormal;
                }

                tExit = std::min(tExit, t2);
                return tEnter <= tExit;
            };

            if (!solveAxis(circleStart.x, relativeDelta.x, expanded.min.x, expanded.max.x, { -1.0f, 0.0f }, { 1.0f, 0.0f }))
            {
                return {};
            }

            if (!solveAxis(circleStart.y, relativeDelta.y, expanded.min.y, expanded.max.y, { 0.0f, -1.0f }, { 0.0f, 1.0f }))
            {
                return {};
            }

            if (tExit < 0.0f || tEnter > 1.0f)
            {
                return {};
            }

            const float toi = std::clamp(tEnter, 0.0f, 1.0f);
            const math::Transform toiCircle = LerpTransform(startCircle, endCircle, toi);
            const collision::Aabb toiAabb{
                startAabb.min + boxDelta * toi,
                startAabb.max + boxDelta * toi
            };

            ToiResult result{};
            if (!collision::CollideCircleVsAabb(circle, toiCircle, toiAabb, result.manifold))
            {
                return {};
            }

            result.manifold.normal = normal.LengthSquared() > settings.toiEpsilon ? normal : result.manifold.normal;
            result.hit = true;
            result.toi = toi;
            return result;
        }
    }

    bool IsBulletBody(const dynamics::RigidBody& body, float dt, const CcdSettings& settings)
    {
        if (!settings.enabled || body.IsStatic() || !body.IsAwake())
        {
            return false;
        }

        const float speed = body.linearVelocity.Length();
        return speed >= settings.bulletSpeedThreshold && speed * dt >= settings.minTravelDistance;
    }

    collision::Aabb ComputeSweptAabb(
        const collision::CollisionShape& shape,
        const math::Transform& start,
        const math::Transform& end)
    {
        const collision::Aabb startAabb = std::visit(
            [&](const auto& typedShape)
            {
                return collision::ComputeAabb(typedShape, start);
            },
            shape);

        const collision::Aabb endAabb = std::visit(
            [&](const auto& typedShape)
            {
                return collision::ComputeAabb(typedShape, end);
            },
            shape);

        return collision::Aabb{
            math::Vec2{
                std::min(startAabb.min.x, endAabb.min.x),
                std::min(startAabb.min.y, endAabb.min.y)
            },
            math::Vec2{
                std::max(startAabb.max.x, endAabb.max.x),
                std::max(startAabb.max.y, endAabb.max.y)
            }
        };
    }

    ToiResult ComputeToi(
        const collision::CollisionShape& shapeA,
        const math::Transform& startA,
        const math::Transform& endA,
        const collision::CollisionShape& shapeB,
        const math::Transform& startB,
        const math::Transform& endB,
        float,
        const CcdSettings& settings)
    {
        return std::visit(
            [&](const auto& a, const auto& b) -> ToiResult
            {
                using A = std::decay_t<decltype(a)>;
                using B = std::decay_t<decltype(b)>;

                if constexpr (std::is_same_v<A, collision::CircleShape> && std::is_same_v<B, collision::CircleShape>)
                {
                    return ComputeCircleCircleToi(a, startA, endA, b, startB, endB, settings);
                }
                else if constexpr (std::is_same_v<A, collision::CircleShape> && std::is_same_v<B, collision::PolygonShape>)
                {
                    const collision::Aabb startAabb = collision::ComputeAabb(b, startB);
                    const collision::Aabb endAabb = collision::ComputeAabb(b, endB);
                    return ComputeCircleAabbToi(a, startA, endA, startAabb, endAabb, settings);
                }
                else if constexpr (std::is_same_v<A, collision::PolygonShape> && std::is_same_v<B, collision::CircleShape>)
                {
                    ToiResult result = ComputeCircleAabbToi(
                        b,
                        startB,
                        endB,
                        collision::ComputeAabb(a, startA),
                        collision::ComputeAabb(a, endA),
                        settings);

                    if (result.hit)
                    {
                        result.manifold.normal *= -1.0f;
                    }

                    return result;
                }
                else
                {
                    return {};
                }
            },
            shapeA,
            shapeB);
    }
}
