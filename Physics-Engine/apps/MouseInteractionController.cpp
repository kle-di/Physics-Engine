#include "MouseInteractionController.h"

#include <algorithm>

#if PHYSICSENGINE_HAS_SDL2
#include <SDL.h>
#endif

namespace PhysicsEngine::app
{
    MouseInteractionController::MouseInteractionController(int windowWidth, int windowHeight, float pixelsPerUnit)
        : m_windowWidth(windowWidth)
        , m_windowHeight(windowHeight)
        , m_pixelsPerUnit(pixelsPerUnit)
    {
    }

    #if PHYSICSENGINE_HAS_SDL2
    void MouseInteractionController::HandleEvent(const SDL_Event& event, core::World& world)
    {
        if (event.type == SDL_MOUSEBUTTONDOWN && event.button.button == SDL_BUTTON_LEFT)
        {
            const math::Vec2 worldPoint = ScreenToWorld(event.button.x, event.button.y);
            if (const auto pickedBody = PickBodyAtPoint(world, worldPoint))
            {
                const auto& bodyEntry = world.GetBodies()[*pickedBody];
                m_grabState.active = true;
                m_grabState.bodyIndex = *pickedBody;
                m_grabState.localGrabPoint = bodyEntry.GetTransform().InverseApply(worldPoint);
                m_grabState.targetWorldPoint = worldPoint;
            }
        }
        else if (event.type == SDL_MOUSEMOTION && m_grabState.active)
        {
            m_grabState.targetWorldPoint = ScreenToWorld(event.motion.x, event.motion.y);
        }
        else if (event.type == SDL_MOUSEBUTTONUP && event.button.button == SDL_BUTTON_LEFT)
        {
            m_grabState.active = false;
        }
    }

    void MouseInteractionController::Update(core::World& world, float dt)
    {
        if (!m_grabState.active)
        {
            return;
        }

        world.ApplyMouseDrag(
            m_grabState.bodyIndex,
            m_grabState.localGrabPoint,
            m_grabState.targetWorldPoint,
            dt,
            m_dragStiffness,
            m_dragDamping,
            m_maxDragImpulse);
    }
    #else
    void MouseInteractionController::HandleEvent(const SDL_Event&, core::World&)
    {
    }

    void MouseInteractionController::Update(core::World&, float)
    {
    }
    #endif

    math::Vec2 MouseInteractionController::ScreenToWorld(int screenX, int screenY) const
    {
        return math::Vec2{
            (static_cast<float>(screenX) - static_cast<float>(m_windowWidth) * 0.5f) / m_pixelsPerUnit,
            (static_cast<float>(screenY) - static_cast<float>(m_windowHeight) * 0.5f) / m_pixelsPerUnit
        };
    }

    std::optional<std::size_t> MouseInteractionController::PickBodyAtPoint(
        const core::World& world,
        const math::Vec2& worldPoint) const
    {
        const auto& bodies = world.GetBodies();
        for (std::size_t i = bodies.size(); i > 0; --i)
        {
            const std::size_t bodyIndex = i - 1;
            const auto& entry = bodies[bodyIndex];
            if (entry.body.IsStatic())
            {
                continue;
            }

            const auto transform = entry.GetTransform();
            if (const auto* circle = std::get_if<collision::CircleShape>(&entry.shape))
            {
                if ((worldPoint - transform.position).LengthSquared() <= circle->radius * circle->radius)
                {
                    return bodyIndex;
                }

                continue;
            }

            const auto& polygon = std::get<collision::PolygonShape>(entry.shape);
            if (PointInConvexPolygon(polygon.GetWorldVertices(transform), worldPoint))
            {
                return bodyIndex;
            }
        }

        return std::nullopt;
    }

    bool MouseInteractionController::PointInConvexPolygon(
        const std::vector<math::Vec2>& vertices,
        const math::Vec2& point)
    {
        if (vertices.size() < 3)
        {
            return false;
        }

        constexpr float kEpsilon = 1.0e-5f;
        for (std::size_t i = 0; i < vertices.size(); ++i)
        {
            const math::Vec2& a = vertices[i];
            const math::Vec2& b = vertices[(i + 1) % vertices.size()];
            const math::Vec2 edge = b - a;
            const math::Vec2 toPoint = point - a;
            if (math::Vec2::Cross(edge, toPoint) < -kEpsilon)
            {
                return false;
            }
        }

        return true;
    }
}
