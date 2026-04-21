#include "Physics-Engine/forces/ForceRegistry.h"

#include <algorithm>

namespace PhysicsEngine::forces
{
    void ForceRegistry::Add(dynamics::RigidBody* body, const ForceGenerator* generator)
    {
        if (body == nullptr || generator == nullptr)
        {
            return;
        }

        m_registrations.push_back(Registration{ body, generator });
    }

    void ForceRegistry::Remove(dynamics::RigidBody* body, const ForceGenerator* generator)
    {
        m_registrations.erase(
            std::remove_if(
                m_registrations.begin(),
                m_registrations.end(),
                [&](const Registration& registration)
                {
                    return registration.body == body && registration.generator == generator;
                }),
            m_registrations.end());
    }

    void ForceRegistry::Clear()
    {
        m_registrations.clear();
    }

    void ForceRegistry::UpdateForces(float dt) const
    {
        for (const auto& registration : m_registrations)
        {
            registration.generator->UpdateForce(*registration.body, dt);
        }
    }
}
