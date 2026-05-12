#include "Physics-Engine/collision/Collision.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>

namespace PhysicsEngine::collision
{
    namespace
    {
        constexpr float kEpsilon = 1.0e-6f;

        struct Projection
        {
            float min;
            float max;
        };

        struct SatResult
        {
            float overlap{ std::numeric_limits<float>::max() };
            math::Vec2 axis{};
            std::size_t edgeIndex{ 0 };
        };

        [[nodiscard]] Projection ProjectVertices(const std::vector<math::Vec2>& vertices, const math::Vec2& axis)
        {
            const float first = math::Vec2::Dot(vertices.front(), axis);
            Projection projection{ first, first };

            for (std::size_t i = 1; i < vertices.size(); ++i)
            {
                const float value = math::Vec2::Dot(vertices[i], axis);
                projection.min = std::min(projection.min, value);
                projection.max = std::max(projection.max, value);
            }

            return projection;
        }

        [[nodiscard]] Projection ProjectCircle(const math::Vec2& center, float radius, const math::Vec2& axis)
        {
            const float c = math::Vec2::Dot(center, axis);
            return Projection{ c - radius, c + radius };
        }

        [[nodiscard]] float ComputeOverlap(const Projection& a, const Projection& b)
        {
            return std::min(a.max, b.max) - std::max(a.min, b.min);
        }

        [[nodiscard]] math::Vec2 ComputePolygonCenter(const std::vector<math::Vec2>& vertices)
        {
            math::Vec2 center{};
            for (const auto& vertex : vertices)
            {
                center += vertex;
            }

            return center / static_cast<float>(vertices.size());
        }

        [[nodiscard]] int FindIncidentEdgeIndex(const std::vector<math::Vec2>& normals, const math::Vec2& referenceNormal)
        {
            int bestIndex = 0;
            float bestDot = std::numeric_limits<float>::max();

            for (std::size_t i = 0; i < normals.size(); ++i)
            {
                const float dot = math::Vec2::Dot(normals[i], referenceNormal);
                if (dot < bestDot)
                {
                    bestDot = dot;
                    bestIndex = static_cast<int>(i);
                }
            }

            return bestIndex;
        }

        [[nodiscard]] int ClipSegmentToLine(
            std::array<math::Vec2, 2>& outPoints,
            const math::Vec2& pointA,
            const math::Vec2& pointB,
            const math::Vec2& normal,
            float offset)
        {
            const float distanceA = math::Vec2::Dot(normal, pointA) - offset;
            const float distanceB = math::Vec2::Dot(normal, pointB) - offset;

            int pointCount = 0;

            if (distanceA <= 0.0f)
            {
                outPoints[pointCount++] = pointA;
            }

            if (distanceB <= 0.0f)
            {
                outPoints[pointCount++] = pointB;
            }

            if (distanceA * distanceB < 0.0f)
            {
                const float t = distanceA / (distanceA - distanceB);
                outPoints[pointCount++] = pointA + (pointB - pointA) * t;
            }

            return pointCount;
        }

        [[nodiscard]] math::Vec2 ClosestPointOnSegment(const math::Vec2& point, const math::Vec2& a, const math::Vec2& b)
        {
            const math::Vec2 ab = b - a;
            const float abLenSq = ab.LengthSquared();
            if (abLenSq <= kEpsilon)
            {
                return a;
            }

            const float t = std::clamp(math::Vec2::Dot(point - a, ab) / abLenSq, 0.0f, 1.0f);
            return a + ab * t;
        }

        [[nodiscard]] math::Vec2 ClosestPointOnPolygonEdges(const math::Vec2& point, const std::vector<math::Vec2>& vertices)
        {
            math::Vec2 best = vertices.front();
            float bestDistanceSq = std::numeric_limits<float>::max();

            for (std::size_t i = 0; i < vertices.size(); ++i)
            {
                const math::Vec2& a = vertices[i];
                const math::Vec2& b = vertices[(i + 1) % vertices.size()];
                const math::Vec2 candidate = ClosestPointOnSegment(point, a, b);
                const float distanceSq = (point - candidate).LengthSquared();
                if (distanceSq < bestDistanceSq)
                {
                    bestDistanceSq = distanceSq;
                    best = candidate;
                }
            }

            return best;
        }

        [[nodiscard]] bool FindBestSatAxis(
            const std::vector<math::Vec2>& axes,
            const std::vector<math::Vec2>& verticesA,
            const std::vector<math::Vec2>& verticesB,
            SatResult& outResult)
        {
            for (std::size_t axisIndex = 0; axisIndex < axes.size(); ++axisIndex)
            {
                const math::Vec2& axisRaw = axes[axisIndex];
                const math::Vec2 axis = axisRaw.Normalized();
                if (axis.LengthSquared() <= kEpsilon)
                {
                    continue;
                }

                const float overlap = ComputeOverlap(ProjectVertices(verticesA, axis), ProjectVertices(verticesB, axis));
                if (overlap <= 0.0f)
                {
                    return false;
                }

                if (overlap < outResult.overlap)
                {
                    outResult.overlap = overlap;
                    outResult.axis = axis;
                    outResult.edgeIndex = axisIndex;
                }
            }

            return true;
        }

        void StoreContactPoint(ContactManifold& manifold, const math::Vec2& point)
        {
            manifold.point = point;
            manifold.points[0] = point;
            manifold.points[1] = point;
            manifold.pointCount = 1;
        }

        void StoreContactPoints(ContactManifold& manifold, const math::Vec2& pointA, const math::Vec2& pointB)
        {
            manifold.point = (pointA + pointB) * 0.5f;
            manifold.points[0] = pointA;
            manifold.points[1] = pointB;
            manifold.pointCount = 2;
        }
    }

    bool Aabb::Overlaps(const Aabb& other) const
    {
        return !(max.x < other.min.x
            || other.max.x < min.x
            || max.y < other.min.y
            || other.max.y < min.y);
    }

    math::Vec2 Aabb::ClosestPoint(const math::Vec2& point) const
    {
        return math::Vec2{
            std::clamp(point.x, min.x, max.x),
            std::clamp(point.y, min.y, max.y)
        };
    }

    Aabb ComputeAabb(const CircleShape& circle, const math::Transform& transform)
    {
        const math::Vec2 extent{ circle.radius, circle.radius };
        return Aabb{ transform.position - extent, transform.position + extent };
    }

    Aabb ComputeAabb(const PolygonShape& polygon, const math::Transform& transform)
    {
        const std::vector<math::Vec2> vertices = polygon.GetWorldVertices(transform);

        math::Vec2 min = vertices.front();
        math::Vec2 max = vertices.front();

        for (const auto& vertex : vertices)
        {
            min.x = std::min(min.x, vertex.x);
            min.y = std::min(min.y, vertex.y);
            max.x = std::max(max.x, vertex.x);
            max.y = std::max(max.y, vertex.y);
        }

        return Aabb{ min, max };
    }

    bool CollideAabbVsAabb(const Aabb& a, const Aabb& b, ContactManifold& outManifold)
    {
        outManifold = {};

        if (!a.Overlaps(b))
        {
            return false;
        }

        const float overlapX = std::min(a.max.x, b.max.x) - std::max(a.min.x, b.min.x);
        const float overlapY = std::min(a.max.y, b.max.y) - std::max(a.min.y, b.min.y);

        const math::Vec2 centerA = (a.min + a.max) * 0.5f;
        const math::Vec2 centerB = (b.min + b.max) * 0.5f;

        if (overlapX < overlapY)
        {
            outManifold.normal = centerB.x >= centerA.x ? math::Vec2{ 1.0f, 0.0f } : math::Vec2{ -1.0f, 0.0f };
            outManifold.penetration = overlapX;
        }
        else
        {
            outManifold.normal = centerB.y >= centerA.y ? math::Vec2{ 0.0f, 1.0f } : math::Vec2{ 0.0f, -1.0f };
            outManifold.penetration = overlapY;
        }

        StoreContactPoint(
            outManifold,
            math::Vec2{
                std::clamp((centerA.x + centerB.x) * 0.5f, std::max(a.min.x, b.min.x), std::min(a.max.x, b.max.x)),
                std::clamp((centerA.y + centerB.y) * 0.5f, std::max(a.min.y, b.min.y), std::min(a.max.y, b.max.y))
            });

        return true;
    }

    bool CollideCircleVsCircle(
        const CircleShape& a,
        const math::Transform& transformA,
        const CircleShape& b,
        const math::Transform& transformB,
        ContactManifold& outManifold)
    {
        outManifold = {};

        const math::Vec2 delta = transformB.position - transformA.position;
        const float distanceSq = delta.LengthSquared();
        const float radiusSum = a.radius + b.radius;

        if (distanceSq >= radiusSum * radiusSum)
        {
            return false;
        }

        float distance = 0.0f;
        math::Vec2 normal{ 1.0f, 0.0f };

        if (distanceSq > kEpsilon)
        {
            distance = std::sqrt(distanceSq);
            normal = delta / distance;
        }

        outManifold.normal = normal;
        outManifold.penetration = radiusSum - distance;
        StoreContactPoint(outManifold, transformA.position + normal * (a.radius - outManifold.penetration * 0.5f));
        return true;
    }

    bool CollideCircleVsAabb(
        const CircleShape& circle,
        const math::Transform& circleTransform,
        const Aabb& box,
        ContactManifold& outManifold)
    {
        outManifold = {};

        const math::Vec2 closestPoint = box.ClosestPoint(circleTransform.position);
        const math::Vec2 delta = closestPoint - circleTransform.position;
        const float distanceSq = delta.LengthSquared();

        if (distanceSq > circle.radius * circle.radius)
        {
            return false;
        }

        math::Vec2 normal{ 0.0f, 1.0f };
        float distance = 0.0f;
        math::Vec2 point = closestPoint;

        if (distanceSq > kEpsilon)
        {
            distance = std::sqrt(distanceSq);
            normal = delta / distance;
        }
        else
        {
            const float toLeft = std::abs(circleTransform.position.x - box.min.x);
            const float toRight = std::abs(box.max.x - circleTransform.position.x);
            const float toBottom = std::abs(circleTransform.position.y - box.min.y);
            const float toTop = std::abs(box.max.y - circleTransform.position.y);

            const float minDistance = std::min(std::min(toLeft, toRight), std::min(toBottom, toTop));
            if (minDistance == toLeft)
            {
                normal = math::Vec2{ -1.0f, 0.0f };
                point = math::Vec2{ box.min.x, circleTransform.position.y };
            }
            else if (minDistance == toRight)
            {
                normal = math::Vec2{ 1.0f, 0.0f };
                point = math::Vec2{ box.max.x, circleTransform.position.y };
            }
            else if (minDistance == toBottom)
            {
                normal = math::Vec2{ 0.0f, -1.0f };
                point = math::Vec2{ circleTransform.position.x, box.min.y };
            }
            else
            {
                normal = math::Vec2{ 0.0f, 1.0f };
                point = math::Vec2{ circleTransform.position.x, box.max.y };
            }

            distance = -minDistance;
        }

        outManifold.normal = normal;
        outManifold.penetration = circle.radius - distance;
        StoreContactPoint(outManifold, point);
        return true;
    }

    bool CollideCircleVsPolygon(
        const CircleShape& circle,
        const math::Transform& circleTransform,
        const PolygonShape& polygon,
        const math::Transform& polygonTransform,
        ContactManifold& outManifold)
    {
        outManifold = {};

        const std::vector<math::Vec2> polygonVertices = polygon.GetWorldVertices(polygonTransform);
        std::vector<math::Vec2> axes = polygon.GetWorldNormals(polygonTransform.rotation);

        auto closestVertexIt = polygonVertices.begin();
        float closestVertexDistanceSq = (circleTransform.position - *closestVertexIt).LengthSquared();
        for (auto it = polygonVertices.begin() + 1; it != polygonVertices.end(); ++it)
        {
            const float distanceSq = (circleTransform.position - *it).LengthSquared();
            if (distanceSq < closestVertexDistanceSq)
            {
                closestVertexDistanceSq = distanceSq;
                closestVertexIt = it;
            }
        }

        const math::Vec2 circleToVertex = *closestVertexIt - circleTransform.position;
        if (circleToVertex.LengthSquared() > kEpsilon)
        {
            axes.push_back(circleToVertex.Normalized());
        }

        float minOverlap = std::numeric_limits<float>::max();
        math::Vec2 bestAxis{ 1.0f, 0.0f };

        for (const auto& axisRaw : axes)
        {
            const math::Vec2 axis = axisRaw.Normalized();
            if (axis.LengthSquared() <= kEpsilon)
            {
                continue;
            }

            const float overlap = ComputeOverlap(ProjectCircle(circleTransform.position, circle.radius, axis), ProjectVertices(polygonVertices, axis));
            if (overlap <= 0.0f)
            {
                return false;
            }

            if (overlap < minOverlap)
            {
                minOverlap = overlap;
                bestAxis = axis;
            }
        }

        const math::Vec2 polygonCenter = ComputePolygonCenter(polygonVertices);
        if (math::Vec2::Dot(bestAxis, polygonCenter - circleTransform.position) < 0.0f)
        {
            bestAxis *= -1.0f;
        }

        outManifold.normal = bestAxis;
        outManifold.penetration = minOverlap;

        const math::Vec2 polygonPoint = ClosestPointOnPolygonEdges(circleTransform.position, polygonVertices);
        const math::Vec2 circlePoint = circleTransform.position + bestAxis * circle.radius;
        StoreContactPoint(outManifold, (polygonPoint + circlePoint) * 0.5f);
        return true;
    }

    bool CollidePolygonVsPolygon(
        const PolygonShape& a,
        const math::Transform& transformA,
        const PolygonShape& b,
        const math::Transform& transformB,
        ContactManifold& outManifold)
    {
        outManifold = {};

        const std::vector<math::Vec2> verticesA = a.GetWorldVertices(transformA);
        const std::vector<math::Vec2> verticesB = b.GetWorldVertices(transformB);
        const std::vector<math::Vec2> axesA = a.GetWorldNormals(transformA.rotation);
        const std::vector<math::Vec2> axesB = b.GetWorldNormals(transformB.rotation);

        SatResult satA{};
        SatResult satB{};

        if (!FindBestSatAxis(axesA, verticesA, verticesB, satA)
            || !FindBestSatAxis(axesB, verticesA, verticesB, satB))
        {
            return false;
        }

        const math::Vec2 centerA = ComputePolygonCenter(verticesA);
        const math::Vec2 centerB = ComputePolygonCenter(verticesB);

        const bool referenceIsA = satA.overlap <= satB.overlap;
        const std::vector<math::Vec2>& referenceVertices = referenceIsA ? verticesA : verticesB;
        const std::vector<math::Vec2>& incidentVertices = referenceIsA ? verticesB : verticesA;
        const std::vector<math::Vec2>& incidentNormals = referenceIsA ? axesB : axesA;
        const SatResult& referenceSat = referenceIsA ? satA : satB;

        math::Vec2 clipNormal = referenceSat.axis;
        const math::Vec2 referenceToIncident = referenceIsA ? (centerB - centerA) : (centerA - centerB);
        if (math::Vec2::Dot(clipNormal, referenceToIncident) < 0.0f)
        {
            clipNormal *= -1.0f;
        }

        outManifold.normal = referenceIsA ? clipNormal : clipNormal * -1.0f;
        outManifold.penetration = referenceSat.overlap;

        const int referenceEdgeIndex = static_cast<int>(referenceSat.edgeIndex);
        const math::Vec2 refV1 = referenceVertices[static_cast<std::size_t>(referenceEdgeIndex)];
        const math::Vec2 refV2 = referenceVertices[static_cast<std::size_t>((referenceEdgeIndex + 1) % referenceVertices.size())];
        const math::Vec2 referenceTangent = (refV2 - refV1).Normalized();
        const math::Vec2 referencePlaneNormal = clipNormal;

        const int incidentEdgeIndex = FindIncidentEdgeIndex(incidentNormals, referencePlaneNormal);
        const math::Vec2 incV1 = incidentVertices[static_cast<std::size_t>(incidentEdgeIndex)];
        const math::Vec2 incV2 = incidentVertices[static_cast<std::size_t>((incidentEdgeIndex + 1) % incidentVertices.size())];

        std::array<math::Vec2, 2> clipPointsA{};
        std::array<math::Vec2, 2> clipPointsB{};
        int clipCount = ClipSegmentToLine(
            clipPointsA,
            incV1,
            incV2,
            referenceTangent * -1.0f,
            -math::Vec2::Dot(referenceTangent, refV1));

        if (clipCount < 2)
        {
            return false;
        }

        clipCount = ClipSegmentToLine(
            clipPointsB,
            clipPointsA[0],
            clipPointsA[1],
            referenceTangent,
            math::Vec2::Dot(referenceTangent, refV2));

        if (clipCount <= 0)
        {
            return false;
        }

        const float referencePlaneOffset = math::Vec2::Dot(referencePlaneNormal, refV1);
        std::array<math::Vec2, 2> contactPoints{};
        int contactCount = 0;

        for (int i = 0; i < clipCount; ++i)
        {
            const float separation = math::Vec2::Dot(referencePlaneNormal, clipPointsB[static_cast<std::size_t>(i)]) - referencePlaneOffset;
            if (separation <= kEpsilon)
            {
                contactPoints[static_cast<std::size_t>(contactCount)] =
                    clipPointsB[static_cast<std::size_t>(i)] - referencePlaneNormal * separation;
                ++contactCount;
                outManifold.penetration = std::max(outManifold.penetration, -separation);
            }
        }

        if (contactCount == 0)
        {
            return false;
        }

        if (contactCount == 1)
        {
            StoreContactPoint(outManifold, contactPoints[0]);
        }
        else
        {
            StoreContactPoints(outManifold, contactPoints[0], contactPoints[1]);
        }
        return true;
    }

    bool GenerateContact(
        const CollisionShape& shapeA,
        const math::Transform& transformA,
        const CollisionShape& shapeB,
        const math::Transform& transformB,
        ContactManifold& outManifold)
    {
        return std::visit(
            [&](const auto& a, const auto& b)
            {
                using A = std::decay_t<decltype(a)>;
                using B = std::decay_t<decltype(b)>;

                if constexpr (std::is_same_v<A, CircleShape> && std::is_same_v<B, CircleShape>)
                {
                    return CollideCircleVsCircle(a, transformA, b, transformB, outManifold);
                }
                else if constexpr (std::is_same_v<A, CircleShape> && std::is_same_v<B, PolygonShape>)
                {
                    return CollideCircleVsPolygon(a, transformA, b, transformB, outManifold);
                }
                else if constexpr (std::is_same_v<A, PolygonShape> && std::is_same_v<B, CircleShape>)
                {
                    const bool hit = CollideCircleVsPolygon(b, transformB, a, transformA, outManifold);
                    if (hit)
                    {
                        outManifold.normal *= -1.0f;
                    }

                    return hit;
                }
                else
                {
                    return CollidePolygonVsPolygon(a, transformA, b, transformB, outManifold);
                }
            },
            shapeA,
            shapeB);
    }
}
