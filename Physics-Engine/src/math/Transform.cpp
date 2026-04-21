#include "Physics-Engine/math/Transform.h"

#include <cmath>

namespace PhysicsEngine::math
{
    Vec2 Rotate(const Vec2& v, float radians)
    {
        const float c = std::cos(radians);
        const float s = std::sin(radians);

        return Vec2{
            v.x * c - v.y * s,
            v.x * s + v.y * c
        };
    }

    Vec2 Transform::Apply(const Vec2& point) const
    {
        return position + Rotate(point, rotation);
    }

    Vec2 Transform::InverseApply(const Vec2& point) const
    {
        return Rotate(point - position, -rotation);
    }
}
