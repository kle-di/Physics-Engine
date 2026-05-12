#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include "Physics-Engine/core/World.h"

union SDL_Event;

namespace PhysicsEngine::app
{
    class MouseInteractionController
    {
    public:
        MouseInteractionController(int windowWidth, int windowHeight, float pixelsPerUnit);

        void HandleEvent(const SDL_Event& event, core::World& world);
        void Update(core::World& world, float dt);

    private:
        struct GrabState
        {
            bool active{ false };
            std::size_t bodyIndex{ 0 };
            math::Vec2 localGrabPoint{};
            math::Vec2 targetWorldPoint{};
        };

        [[nodiscard]] math::Vec2 ScreenToWorld(int screenX, int screenY) const;
        [[nodiscard]] std::optional<std::size_t> PickBodyAtPoint(const core::World& world, const math::Vec2& worldPoint) const;
        [[nodiscard]] static bool PointInConvexPolygon(const std::vector<math::Vec2>& vertices, const math::Vec2& point);

        int m_windowWidth;
        int m_windowHeight;
        float m_pixelsPerUnit;
        float m_dragStiffness{ 180.0f };
        float m_dragDamping{ 25.0f };
        float m_maxDragImpulse{ 18.0f };
        GrabState m_grabState{};
    };
}
