#include "PhysicsEnginnn/rendering/DebugRenderer.h"

#include <cmath>

#include <SDL.h>

namespace PhysicsEnginnn::rendering
{
    DebugRenderer::DebugRenderer(int width, int height, float pixelsPerUnit)
        : m_width(width)
        , m_height(height)
        , m_pixelsPerUnit(pixelsPerUnit)
    {
        if (SDL_Init(SDL_INIT_VIDEO) != 0)
        {
            return;
        }

        m_window = SDL_CreateWindow(
            "PhysicsEnginnn Debug Renderer",
            SDL_WINDOWPOS_CENTERED,
            SDL_WINDOWPOS_CENTERED,
            m_width,
            m_height,
            SDL_WINDOW_SHOWN);

        if (m_window == nullptr)
        {
            SDL_Quit();
            return;
        }

        m_renderer = SDL_CreateRenderer(m_window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
        if (m_renderer == nullptr)
        {
            SDL_DestroyWindow(m_window);
            m_window = nullptr;
            SDL_Quit();
            return;
        }

        m_sdlReady = true;
    }

    DebugRenderer::~DebugRenderer()
    {
        if (m_renderer != nullptr)
        {
            SDL_DestroyRenderer(m_renderer);
            m_renderer = nullptr;
        }

        if (m_window != nullptr)
        {
            SDL_DestroyWindow(m_window);
            m_window = nullptr;
        }

        if (m_sdlReady)
        {
            SDL_Quit();
        }
    }

    bool DebugRenderer::IsValid() const
    {
        return m_sdlReady && m_window != nullptr && m_renderer != nullptr;
    }

    bool DebugRenderer::PumpEvents()
    {
        SDL_Event event{};
        while (SDL_PollEvent(&event) != 0)
        {
            if (event.type == SDL_QUIT)
            {
                return false;
            }
        }

        return true;
    }

    void DebugRenderer::Clear()
    {
        SDL_SetRenderDrawColor(m_renderer, 20, 20, 26, 255);
        SDL_RenderClear(m_renderer);
    }

    void DebugRenderer::DrawWorld(const core::World& world)
    {
        SDL_SetRenderDrawColor(m_renderer, 230, 230, 230, 255);

        for (const auto& entry : world.GetBodies())
        {
            const math::Transform transform = entry.GetTransform();

            if (const auto* circle = std::get_if<collision::CircleShape>(&entry.shape))
            {
                DrawCircle(transform.position, circle->radius);
                continue;
            }

            const auto& polygon = std::get<collision::PolygonShape>(entry.shape);
            DrawPolygon(polygon.GetWorldVertices(transform));
        }
    }

    void DebugRenderer::Present()
    {
        SDL_RenderPresent(m_renderer);
    }

    void DebugRenderer::DrawCircle(const math::Vec2& center, float radius)
    {
        constexpr int segments = 24;
        const float step = 2.0f * 3.1415926535f / static_cast<float>(segments);

        math::Vec2 prev = ToScreen(center + math::Vec2{ radius, 0.0f });
        for (int i = 1; i <= segments; ++i)
        {
            const float angle = static_cast<float>(i) * step;
            const math::Vec2 point = center + math::Vec2{ std::cos(angle) * radius, std::sin(angle) * radius };
            const math::Vec2 current = ToScreen(point);

            SDL_RenderDrawLine(
                m_renderer,
                static_cast<int>(prev.x),
                static_cast<int>(prev.y),
                static_cast<int>(current.x),
                static_cast<int>(current.y));

            prev = current;
        }
    }

    void DebugRenderer::DrawPolygon(const std::vector<math::Vec2>& vertices)
    {
        if (vertices.size() < 2)
        {
            return;
        }

        for (std::size_t i = 0; i < vertices.size(); ++i)
        {
            const math::Vec2 a = ToScreen(vertices[i]);
            const math::Vec2 b = ToScreen(vertices[(i + 1) % vertices.size()]);
            SDL_RenderDrawLine(
                m_renderer,
                static_cast<int>(a.x),
                static_cast<int>(a.y),
                static_cast<int>(b.x),
                static_cast<int>(b.y));
        }
    }

    math::Vec2 DebugRenderer::ToScreen(const math::Vec2& worldPoint) const
    {
        return math::Vec2{
            worldPoint.x * m_pixelsPerUnit + static_cast<float>(m_width) * 0.5f,
            worldPoint.y * m_pixelsPerUnit + static_cast<float>(m_height) * 0.5f
        };
    }
}
