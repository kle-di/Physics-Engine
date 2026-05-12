#pragma once

#include <cstddef>
#include <vector>

#include "Physics-Engine/collision/Collision.h"

namespace PhysicsEngine::broadphase
{
    struct BroadPhasePair
    {
        std::size_t bodyA{ 0 };
        std::size_t bodyB{ 0 };
    };

    class UniformGridBroadPhase
    {
    public:
        explicit UniformGridBroadPhase(float cellSize = 1.0f);

        [[nodiscard]] const std::vector<BroadPhasePair>& BuildPairs(const std::vector<collision::Aabb>& aabbs);

    private:
        float m_cellSize;
        std::vector<BroadPhasePair> m_pairs;
    };
}
