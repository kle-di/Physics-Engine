#include "Physics-Engine/math/Vec2.h"

#include <cmath>

namespace PhysicsEngine::math
{
    float Vec2::LengthSquared() const
    {
        return x * x + y * y;
    }

    float Vec2::Length() const
    {
        return std::sqrt(LengthSquared());
    }

    Vec2 Vec2::Normalized() const
    {
        const float len = Length();
        if (len <= 1.0e-6f)
        {
            return Vec2{};
        }

        return *this / len;
    }

    float Vec2::Dot(const Vec2& a, const Vec2& b)
    {
        return a.x * b.x + a.y * b.y;
    }

    float Vec2::Cross(const Vec2& a, const Vec2& b)
    {
        return a.x * b.y - a.y * b.x;
    }

    Vec2 Vec2::Cross(const Vec2& v, float s)
    {
        return Vec2{ s * v.y, -s * v.x };
    }

    Vec2 Vec2::Cross(float s, const Vec2& v)
    {
        return Vec2{ -s * v.y, s * v.x };
    }
}
