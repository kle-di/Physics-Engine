#pragma once

#include <vector>

#include "PhysicsEnginnn/math/Transform.h"

namespace PhysicsEnginnn::collision
{
    enum class ShapeType
    {
        Circle,
        Polygon
    };

    struct CircleShape
    {
        float radius{ 0.5f };

        /**
         * @param r Circle radius.
         */
        explicit CircleShape(float r = 0.5f);
        /**
         * @param mass Shape mass.
         * @return Moment of inertia for a solid circle.
         */
        float ComputeInertia(float mass) const;
    };

    struct PolygonShape
    {
        std::vector<math::Vec2> vertices;
        std::vector<math::Vec2> normals;

        PolygonShape() = default;
        /**
         * @param points Polygon vertices in local space.
         */
        explicit PolygonShape(std::vector<math::Vec2> points);

        /**
         * @param halfWidth Half-width of the box.
         * @param halfHeight Half-height of the box.
         * @return Polygon shape representing an axis-aligned box in local space.
         */
        static PolygonShape CreateBox(float halfWidth, float halfHeight);

        /**
         * @param points Polygon vertices in local space.
         */
        void SetVertices(std::vector<math::Vec2> points);
        /**
         * @param mass Shape mass.
         * @return Approximate moment of inertia.
         */
        float ComputeInertia(float mass) const;
        /**
         * @param transform Transform from local to world space.
         * @return Vertices transformed to world space.
         */
        std::vector<math::Vec2> GetWorldVertices(const math::Transform& transform) const;
        /**
         * @param rotation Rotation in radians.
         * @return Edge normals rotated to world orientation.
         */
        std::vector<math::Vec2> GetWorldNormals(float rotation) const;

    private:
        void EnsureCounterClockwise();
        void ComputeEdgeNormals();
    };
}
