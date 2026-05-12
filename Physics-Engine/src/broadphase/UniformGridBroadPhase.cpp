#include "Physics-Engine/broadphase/UniformGridBroadPhase.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>

namespace PhysicsEngine::broadphase
{
    namespace
    {
        [[nodiscard]] std::int64_t CellKey(int x, int y)
        {
            return (static_cast<std::int64_t>(x) << 32) ^ static_cast<std::uint32_t>(y);
        }

        [[nodiscard]] std::uint64_t PairKey(std::size_t a, std::size_t b)
        {
            const std::size_t minValue = std::min(a, b);
            const std::size_t maxValue = std::max(a, b);
            return (static_cast<std::uint64_t>(minValue) << 32) ^ static_cast<std::uint64_t>(maxValue);
        }
    }

    UniformGridBroadPhase::UniformGridBroadPhase(float cellSize)
        : m_cellSize(std::max(cellSize, 0.1f))
    {
    }

    const std::vector<BroadPhasePair>& UniformGridBroadPhase::BuildPairs(const std::vector<collision::Aabb>& aabbs)
    {
        m_pairs.clear();

        std::unordered_map<std::int64_t, std::vector<std::size_t>> grid;
        grid.reserve(aabbs.size() * 2);

        for (std::size_t bodyIndex = 0; bodyIndex < aabbs.size(); ++bodyIndex)
        {
            const collision::Aabb& aabb = aabbs[bodyIndex];
            const int minX = static_cast<int>(std::floor(aabb.min.x / m_cellSize));
            const int maxX = static_cast<int>(std::floor(aabb.max.x / m_cellSize));
            const int minY = static_cast<int>(std::floor(aabb.min.y / m_cellSize));
            const int maxY = static_cast<int>(std::floor(aabb.max.y / m_cellSize));

            for (int x = minX; x <= maxX; ++x)
            {
                for (int y = minY; y <= maxY; ++y)
                {
                    grid[CellKey(x, y)].push_back(bodyIndex);
                }
            }
        }

        std::unordered_set<std::uint64_t> seenPairs;
        seenPairs.reserve(aabbs.size() * 4);

        for (const auto& [_, members] : grid)
        {
            if (members.size() < 2)
            {
                continue;
            }

            for (std::size_t i = 0; i < members.size(); ++i)
            {
                for (std::size_t j = i + 1; j < members.size(); ++j)
                {
                    const std::size_t bodyA = members[i];
                    const std::size_t bodyB = members[j];
                    const std::uint64_t key = PairKey(bodyA, bodyB);
                    if (!seenPairs.insert(key).second)
                    {
                        continue;
                    }

                    if (!aabbs[bodyA].Overlaps(aabbs[bodyB]))
                    {
                        continue;
                    }

                    m_pairs.push_back(BroadPhasePair{ bodyA, bodyB });
                }
            }
        }

        return m_pairs;
    }
}
