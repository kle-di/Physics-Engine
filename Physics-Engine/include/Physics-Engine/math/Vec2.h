#pragma once

namespace PhysicsEngine::math
{
    struct Vec2
    {
        float x{ 0.0f };
        float y{ 0.0f };

        constexpr Vec2() = default;
        constexpr Vec2(float xValue, float yValue) : x(xValue), y(yValue) {}

        constexpr Vec2 operator+(const Vec2& rhs) const;
        constexpr Vec2 operator-(const Vec2& rhs) const;
        constexpr Vec2 operator*(float scalar) const;
        constexpr Vec2 operator/(float scalar) const;

        constexpr Vec2& operator+=(const Vec2& rhs);
        constexpr Vec2& operator-=(const Vec2& rhs);
        constexpr Vec2& operator*=(float scalar);
        constexpr Vec2& operator/=(float scalar);

        /** @return Squared vector magnitude. */
        [[nodiscard]] float LengthSquared() const;
        /** @return Vector magnitude. */
        [[nodiscard]] float Length() const;
        /** @return Unit-length vector (or zero vector when length is near zero). */
        [[nodiscard]] Vec2 Normalized() const;

        /**
         * @param a First vector.
         * @param b Second vector.
         * @return Dot product of `a` and `b`.
         */
        static float Dot(const Vec2& a, const Vec2& b);
        /**
         * @param a First vector.
         * @param b Second vector.
         * @return 2D scalar cross product of `a` and `b`.
         */
        static float Cross(const Vec2& a, const Vec2& b);
        /**
         * @param v Input vector.
         * @param s Scalar.
         * @return Perpendicular vector representing `v x s` in 2D form.
         */
        static Vec2 Cross(const Vec2& v, float s);
        /**
         * @param s Scalar.
         * @param v Input vector.
         * @return Perpendicular vector representing `s x v` in 2D form.
         */
        static Vec2 Cross(float s, const Vec2& v);
    };

    constexpr Vec2 operator*(float scalar, const Vec2& v)
    {
        return v * scalar;
    }

    constexpr Vec2 Vec2::operator+(const Vec2& rhs) const
    {
        return Vec2{ x + rhs.x, y + rhs.y };
    }

    constexpr Vec2 Vec2::operator-(const Vec2& rhs) const
    {
        return Vec2{ x - rhs.x, y - rhs.y };
    }

    constexpr Vec2 Vec2::operator*(float scalar) const
    {
        return Vec2{ x * scalar, y * scalar };
    }

    constexpr Vec2 Vec2::operator/(float scalar) const
    {
        return Vec2{ x / scalar, y / scalar };
    }

    constexpr Vec2& Vec2::operator+=(const Vec2& rhs)
    {
        x += rhs.x;
        y += rhs.y;
        return *this;
    }

    constexpr Vec2& Vec2::operator-=(const Vec2& rhs)
    {
        x -= rhs.x;
        y -= rhs.y;
        return *this;
    }

    constexpr Vec2& Vec2::operator*=(float scalar)
    {
        x *= scalar;
        y *= scalar;
        return *this;
    }

    constexpr Vec2& Vec2::operator/=(float scalar)
    {
        x /= scalar;
        y /= scalar;
        return *this;
    }
}
