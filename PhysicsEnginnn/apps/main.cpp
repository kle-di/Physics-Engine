#include <algorithm>
#include <chrono>
#include <iostream>

#include "PhysicsEnginnn/collision/Shapes.h"
#include "PhysicsEnginnn/core/World.h"

#if PHYSICSENGINNN_HAS_SDL2
#include "PhysicsEnginnn/rendering/DebugRenderer.h"
#endif

int main()
{
    PhysicsEnginnn::core::World world(1.0f / 60.0f);
    world.SetGravity({ 0.0f, 9.81f });

    PhysicsEnginnn::dynamics::RigidBodyDesc floorDesc{};
    floorDesc.position = { 0.0f, 3.71f };
    floorDesc.mass = 0.0f;
    floorDesc.inertia = 0.0f;
    world.CreatePolygonBody(floorDesc, PhysicsEnginnn::collision::PolygonShape::CreateBox(6.7f, 0.08f));

    PhysicsEnginnn::dynamics::RigidBodyDesc ceilingDesc{};
    ceilingDesc.position = { 0.0f, -3.71f };
    ceilingDesc.mass = 0.0f;
    ceilingDesc.inertia = 0.0f;
    world.CreatePolygonBody(ceilingDesc, PhysicsEnginnn::collision::PolygonShape::CreateBox(6.7f, 0.08f));

    PhysicsEnginnn::dynamics::RigidBodyDesc leftWallDesc{};
    leftWallDesc.position = { -6.62f, 0.0f };
    leftWallDesc.mass = 0.0f;
    leftWallDesc.inertia = 0.0f;
    world.CreatePolygonBody(leftWallDesc, PhysicsEnginnn::collision::PolygonShape::CreateBox(0.08f, 4.0f));

    PhysicsEnginnn::dynamics::RigidBodyDesc rightWallDesc{};
    rightWallDesc.position = { 6.62f, 0.0f };
    rightWallDesc.mass = 0.0f;
    rightWallDesc.inertia = 0.0f;
    world.CreatePolygonBody(rightWallDesc, PhysicsEnginnn::collision::PolygonShape::CreateBox(0.08f, 4.0f));

    auto spawnBox = [&](const PhysicsEnginnn::math::Vec2& position,
                        const PhysicsEnginnn::math::Vec2& velocity,
                        float halfWidth,
                        float halfHeight,
                        float mass,
                        float restitution,
                        float friction,
                        float angularVelocity)
    {
        PhysicsEnginnn::dynamics::RigidBodyDesc desc{};
        desc.position = position;
        desc.linearVelocity = { velocity.x, velocity.y };
        desc.angularVelocity = angularVelocity;
        desc.mass = mass;
        desc.restitution = restitution;
        desc.friction = friction;
        world.CreatePolygonBody(desc, PhysicsEnginnn::collision::PolygonShape::CreateBox(halfWidth, halfHeight));
    };

    auto spawnCircle = [&](const PhysicsEnginnn::math::Vec2& position,
                           const PhysicsEnginnn::math::Vec2& velocity,
                           float radius,
                           float mass,
                           float restitution,
                           float friction)
    {
        PhysicsEnginnn::dynamics::RigidBodyDesc desc{};
        desc.position = position;
        desc.linearVelocity = {velocity.x, velocity.y };
        desc.mass = mass;
        desc.restitution = restitution;
        desc.friction = friction;
        world.CreateCircleBody(desc, PhysicsEnginnn::collision::CircleShape(radius));
    };

    spawnBox({ -2.0f, -2.8f }, { 0.0f, 0.0f }, 0.65f, 0.45f, 1.2f, 0.15f, 0.7f, 1.0f);
    spawnBox({ 2.1f, -2.2f }, { 0.0f, 0.1f }, 0.55f, 0.55f, 1.0f, 0.10f, 0.7f, -0.2f);

    spawnCircle({ -3.1f, -2.4f }, { 0.0f, -0.05f }, 0.35f, 0.8f, 0.55f, 0.35f);
    spawnCircle({ 3.2f, -1.9f }, { 0.0f, -0.15f }, 0.42f, 1.1f, 0.60f, 0.30f);

    // Box stack stability test.
    spawnBox({ -5.2f, 3.2f }, { 0.0f, 0.0f }, 0.35f, 0.35f, 1.0f, 0.05f, 0.8f, 0.0f);
    spawnBox({ -5.2f, 2.45f }, { 0.0f, 0.0f }, 0.35f, 0.35f, 1.0f, 0.05f, 0.8f, 0.0f);
    spawnBox({ -5.2f, 1.7f }, { 0.0f, 0.0f }, 0.35f, 0.35f, 1.0f, 0.05f, 0.8f, 0.0f);
    spawnBox({ -5.2f, 0.95f }, { 0.0f, 0.0f }, 0.35f, 0.35f, 1.0f, 0.05f, 0.8f, 0.0f);

    // Mixed mass-ratio and friction test.
    spawnBox({ -0.2f, 2.8f }, { 0.0f, 0.0f }, 0.45f, 0.45f, 4.0f, 0.03f, 0.9f, 0.0f);
    spawnCircle({ -0.2f, 1.85f }, { 0.0f, 0.0f }, 0.22f, 0.3f, 0.35f, 0.55f);
    spawnCircle({ 0.45f, 1.65f }, { 0.0f, 0.0f }, 0.18f, 0.2f, 0.35f, 0.55f);

    // Sliding / spin interaction near right side.
    spawnBox({ 4.3f, 2.2f }, { 0.0f, 0.0f }, 0.6f, 0.20f, 0.9f, 0.02f, 0.95f, 0.6f);
    spawnBox({ 4.3f, 1.55f }, { 0.0f, 0.0f }, 0.6f, 0.20f, 0.9f, 0.02f, 0.95f, -0.4f);

#if PHYSICSENGINNN_HAS_SDL2
    PhysicsEnginnn::rendering::DebugRenderer renderer(1280, 720, 95.0f);
    if (!renderer.IsValid())
    {
        std::cerr << "SDL2 renderer init failed.\n";
        return 1;
    }

    using clock = std::chrono::steady_clock;
    auto previous = clock::now();
    constexpr float maxFrameTime = 1.0f / 30.0f;

    bool running = true;
    while (running)
    {
        running = renderer.PumpEvents();

        const auto now = clock::now();
        const std::chrono::duration<float> frameTime = now - previous;
        previous = now;

        world.Step(std::min(frameTime.count(), maxFrameTime));
        renderer.Clear();
        renderer.DrawWorld(world);
        renderer.Present();
    }
#else
    std::cerr << "SDL2 support is disabled. Rebuild with SDL2 to use DebugRenderer.\n";
#endif
    return 0;
}
