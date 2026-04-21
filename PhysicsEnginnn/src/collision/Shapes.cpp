#include "PhysicsEnginnn/collision/Shapes.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace PhysicsEnginnn::collision
{
    namespace
    {
        constexpr float kMinShapeValue = 1.0e-6f;

        float SignedArea(const std::vector<math::Vec2>& vertices)
        {
            float areaTwice = 0.0f;
            const std::size_t count = vertices.size();
            for (std::size_t i = 0; i < count; ++i)
            {
                const math::Vec2& a = vertices[i];
                const math::Vec2& b = vertices[(i + 1) % count];
                areaTwice += math::Vec2::Cross(a, b);
            }

            return areaTwice * 0.5f;
        }
    }

    CircleShape::CircleShape(float r)
        : radius(std::max(r, kMinShapeValue))
    {
    }

    float CircleShape::ComputeInertia(float mass) const
    {
        return 0.5f * mass * radius * radius;
    }

    PolygonShape::PolygonShape(std::vector<math::Vec2> points)
    {
        SetVertices(std::move(points));
    }

    PolygonShape PolygonShape::CreateBox(float halfWidth, float halfHeight)
    {
        const float hx = std::max(halfWidth, kMinShapeValue);
        const float hy = std::max(halfHeight, kMinShapeValue);

        return PolygonShape({
            { -hx, -hy },
            { hx, -hy },
            { hx, hy },
            { -hx, hy }
        });
    }

    void PolygonShape::SetVertices(std::vector<math::Vec2> points)
    {
        if (points.size() < 3)
        {
            throw std::invalid_argument("PolygonShape requires at least 3 vertices");
        }

        vertices = std::move(points);
        EnsureCounterClockwise();
        ComputeEdgeNormals();
    }

    float PolygonShape::ComputeInertia(float mass) const
    {
        float minX = vertices[0].x;
        float maxX = vertices[0].x;
        float minY = vertices[0].y;
        float maxY = vertices[0].y;

        for (const auto& v : vertices)
        {
            minX = std::min(minX, v.x);
            maxX = std::max(maxX, v.x);
            minY = std::min(minY, v.y);
            maxY = std::max(maxY, v.y);
        }

        const float width = maxX - minX;
        const float height = maxY - minY;
        return mass * (width * width + height * height) / 12.0f;
    }

    std::vector<math::Vec2> PolygonShape::GetWorldVertices(const math::Transform& transform) const
    {
        std::vector<math::Vec2> world;
        world.reserve(vertices.size());

        for (const auto& localVertex : vertices)
        {
            world.push_back(transform.Apply(localVertex));
        }

        return world;
    }

    std::vector<math::Vec2> PolygonShape::GetWorldNormals(float rotation) const
    {
        std::vector<math::Vec2> worldNormals;
        worldNormals.reserve(normals.size());

        for (const auto& localNormal : normals)
        {
            worldNormals.push_back(math::Rotate(localNormal, rotation));
        }

        return worldNormals;
    }

    void PolygonShape::EnsureCounterClockwise()
    {
        if (SignedArea(vertices) < 0.0f)
        {
            std::reverse(vertices.begin(), vertices.end());
        }
    }

    void PolygonShape::ComputeEdgeNormals()
    {
        normals.clear();
        normals.reserve(vertices.size());

        const std::size_t count = vertices.size();
        for (std::size_t i = 0; i < count; ++i)
        {
            const math::Vec2& a = vertices[i];
            const math::Vec2& b = vertices[(i + 1) % count];
            const math::Vec2 edge = b - a;
            const math::Vec2 outward = math::Vec2{ edge.y, -edge.x }.Normalized();
            normals.push_back(outward);
        }
    }
}
