#include "PhysicsEnginnn/core/World.h"

namespace PhysicsEnginnn::core
{
    math::Transform World::BodyEntry::GetTransform() const
    {
        return math::Transform{ body.position, body.rotation };
    }

    World::World(float fixedTimeStep)
        : m_fixedTimeStep(fixedTimeStep)
    {
    }

    std::size_t World::CreateCircleBody(const dynamics::RigidBodyDesc& desc, const collision::CircleShape& circle)
    {
        BodyEntry entry{};
        entry.body = dynamics::RigidBody(desc);
        entry.shape = circle;

        if (!entry.body.IsStatic())
        {
            entry.body.SetInertia(circle.ComputeInertia(entry.body.mass));
        }

        m_bodies.push_back(std::move(entry));
        RebuildForceRegistry();
        return m_bodies.size() - 1;
    }

    std::size_t World::CreatePolygonBody(const dynamics::RigidBodyDesc& desc, const collision::PolygonShape& polygon)
    {
        BodyEntry entry{};
        entry.body = dynamics::RigidBody(desc);
        entry.shape = polygon;

        if (!entry.body.IsStatic())
        {
            entry.body.SetInertia(polygon.ComputeInertia(entry.body.mass));
        }

        m_bodies.push_back(std::move(entry));
        RebuildForceRegistry();
        return m_bodies.size() - 1;
    }

    void World::SetGravity(const math::Vec2& gravityValue)
    {
        m_gravity = gravityValue;
        m_gravityGenerator.SetGravity(gravityValue);
    }

    void World::Step(float dt)
    {
        if (dt <= 0.0f)
        {
            return;
        }

        m_accumulator += dt;
        while (m_accumulator >= m_fixedTimeStep)
        {
            StepFixed(m_fixedTimeStep);
            m_accumulator -= m_fixedTimeStep;
        }
    }

    const std::vector<World::BodyEntry>& World::GetBodies() const
    {
        return m_bodies;
    }

    void World::StepFixed(float dt)
    {
        m_forceRegistry.UpdateForces(dt);

        for (auto& entry : m_bodies)
        {
            entry.body.IntegrateForces(dt);
            entry.body.IntegrateVelocity(dt);
        }
    }

    void World::RebuildForceRegistry()
    {
        m_forceRegistry.Clear();

        for (auto& entry : m_bodies)
        {
            if (!entry.body.IsStatic())
            {
                m_forceRegistry.Add(&entry.body, &m_gravityGenerator);
            }
        }
    }
}
