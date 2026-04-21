#pragma once

#include "PhysicsEnginnn/core/World.h"

struct SDL_Renderer;
struct SDL_Window;

namespace PhysicsEnginnn::rendering
{
    class DebugRenderer
    {
    public:
        /**
         * @param width Window width in pixels.
         * @param height Window height in pixels.
         * @param pixelsPerUnit World-to-screen scale.
         */
        DebugRenderer(int width, int height, float pixelsPerUnit);
        ~DebugRenderer();

        DebugRenderer(const DebugRenderer&) = delete;
        DebugRenderer& operator=(const DebugRenderer&) = delete;

        /** @return True if SDL objects were initialized successfully. */
        [[nodiscard]] bool IsValid() const;
        /** @return False when quit event is requested; otherwise true. */
        [[nodiscard]] bool PumpEvents();

        void Clear();
        /**
         * @param world World state to draw.
         */
        void DrawWorld(const core::World& world);
        void Present();

    private:
        void DrawCircle(const math::Vec2& center, float radius);
        void DrawPolygon(const std::vector<math::Vec2>& vertices);
        [[nodiscard]] math::Vec2 ToScreen(const math::Vec2& worldPoint) const;

        SDL_Window* m_window{ nullptr };
        SDL_Renderer* m_renderer{ nullptr };
        int m_width{ 0 };
        int m_height{ 0 };
        float m_pixelsPerUnit{ 100.0f };
        bool m_sdlReady{ false };
    };
}
