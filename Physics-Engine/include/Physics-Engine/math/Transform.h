#pragma once

#include "Physics-Engine/math/Vec2.h"

namespace PhysicsEngine::math
{
    /**
     * @param v Vector to rotate.
     * @param radians Rotation angle in radians.
     * @return Rotated vector.
     */
    [[nodiscard]] Vec2 Rotate(const Vec2& v, float radians);

    struct Transform
    {
        Vec2 position{};
        float rotation{ 0.0f };

        /**
         * @param point Local-space point.
         * @return World-space point after applying position and rotation.
         */
        [[nodiscard]] Vec2 Apply(const Vec2& point) const;
        /**
         * @param point World-space point.
         * @return Local-space point after applying inverse transform.
         */
        [[nodiscard]] Vec2 InverseApply(const Vec2& point) const;
    };
}
