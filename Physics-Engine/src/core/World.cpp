#include "Physics-Engine/core/World.h"

#include <algorithm>

namespace PhysicsEngine::core
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

    void World::ApplyMouseDrag(
        std::size_t bodyIndex,
        const math::Vec2& localGrabPoint,
        const math::Vec2& targetWorldPoint,
        float dt,
        float stiffness,
        float damping,
        float maxImpulse)
    {
        if (bodyIndex >= m_bodies.size())
        {
            return;
        }

        auto& bodyEntry = m_bodies[bodyIndex];
        auto& body = bodyEntry.body;
        if (body.IsStatic())
        {
            return;
        }

        const math::Transform transform = bodyEntry.GetTransform();
        const math::Vec2 grabWorldPoint = transform.Apply(localGrabPoint);
        const math::Vec2 contactVector = grabWorldPoint - body.position;
        const math::Vec2 pointVelocity = body.linearVelocity + math::Vec2::Cross(body.angularVelocity, contactVector);
        const math::Vec2 error = targetWorldPoint - grabWorldPoint;

        math::Vec2 impulse = (error * stiffness - pointVelocity * damping) * dt;
        const float maxImpulseSq = maxImpulse * maxImpulse;
        const float impulseSq = impulse.LengthSquared();
        if (impulseSq > maxImpulseSq && impulseSq > 0.0f)
        {
            impulse = impulse.Normalized() * maxImpulse;
        }

        body.ApplyImpulse(impulse, contactVector);
    }

    const std::vector<World::BodyEntry>& World::GetBodies() const
    {
        return m_bodies;
    }

    const std::vector<World::ContactEntry>& World::GetContacts() const
    {
        return m_contacts;
    }

    void World::StepFixed(float dt)
    {
        m_forceRegistry.UpdateForces(dt);

        for (auto& entry : m_bodies)
        {
            entry.body.IntegrateForces(dt);
        }

        std::vector<math::Transform> currentTransforms;
        currentTransforms.reserve(m_bodies.size());
        std::vector<math::Transform> predictedTransforms;
        predictedTransforms.reserve(m_bodies.size());

        for (const auto& entry : m_bodies)
        {
            currentTransforms.push_back(entry.GetTransform());
            predictedTransforms.push_back(math::Transform{
                entry.body.position + entry.body.linearVelocity * dt,
                entry.body.rotation + entry.body.angularVelocity * dt
            });
        }

        std::vector<collision::Aabb> sweptAabbs;
        sweptAabbs.reserve(m_bodies.size());
        for (std::size_t i = 0; i < m_bodies.size(); ++i)
        {
            sweptAabbs.push_back(ccd::ComputeSweptAabb(m_bodies[i].shape, currentTransforms[i], predictedTransforms[i]));
        }

        m_contacts.clear();
        const auto& candidatePairs = m_broadPhase.BuildPairs(sweptAabbs);
        for (const auto& pair : candidatePairs)
        {
            auto& a = m_bodies[pair.bodyA];
            auto& b = m_bodies[pair.bodyB];

            const bool aInactive = a.body.IsStatic() || !a.body.IsAwake();
            const bool bInactive = b.body.IsStatic() || !b.body.IsAwake();
            if (aInactive && bInactive)
            {
                continue;
            }

            collision::ContactManifold manifold{};
            bool hasContact = collision::GenerateContact(
                a.shape,
                currentTransforms[pair.bodyA],
                b.shape,
                currentTransforms[pair.bodyB],
                manifold);

            if (!hasContact)
            {
                const bool aBullet = ccd::IsBulletBody(a.body, dt, m_ccdSettings);
                const bool bBullet = ccd::IsBulletBody(b.body, dt, m_ccdSettings);
                if (aBullet || bBullet)
                {
                    const ccd::ToiResult toiResult = ccd::ComputeToi(
                        a.shape,
                        currentTransforms[pair.bodyA],
                        predictedTransforms[pair.bodyA],
                        b.shape,
                        currentTransforms[pair.bodyB],
                        predictedTransforms[pair.bodyB],
                        dt,
                        m_ccdSettings);

                    if (toiResult.hit)
                    {
                        manifold = toiResult.manifold;
                        hasContact = true;
                    }
                }
            }

            if (hasContact)
            {
                m_contacts.push_back(ContactEntry{ pair.bodyA, pair.bodyB, manifold });
            }
        }

        const dynamics::CollisionResolverConfig& resolverConfig = m_collisionResolver.GetConfig();
        for (int iteration = 0; iteration < resolverConfig.velocityIterations; ++iteration)
        {
            for (const auto& contact : m_contacts)
            {
                auto& bodyA = m_bodies[contact.bodyA].body;
                auto& bodyB = m_bodies[contact.bodyB].body;
                m_collisionResolver.ResolveVelocity(bodyA, bodyB, contact.manifold);
            }
        }

        for (auto& entry : m_bodies)
        {
            entry.body.IntegrateVelocity(dt);
        }

        for (int iteration = 0; iteration < resolverConfig.positionIterations; ++iteration)
        {
            for (const auto& contact : m_contacts)
            {
                auto& bodyA = m_bodies[contact.bodyA].body;
                auto& bodyB = m_bodies[contact.bodyB].body;
                m_collisionResolver.CorrectPosition(bodyA, bodyB, contact.manifold);
            }
        }

        constexpr float kSleepLinearThreshold = 0.05f;
        constexpr float kSleepAngularThreshold = 0.05f;
        constexpr float kSleepDelaySeconds = 0.35f;
        for (auto& entry : m_bodies)
        {
            entry.body.UpdateSleepState(dt, kSleepLinearThreshold, kSleepAngularThreshold, kSleepDelaySeconds);
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
