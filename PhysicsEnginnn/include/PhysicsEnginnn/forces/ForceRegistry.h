#pragma once

#include <vector>

#include "PhysicsEnginnn/dynamics/RigidBody.h"
#include "PhysicsEnginnn/forces/ForceGenerator.h"

namespace PhysicsEnginnn::forces
{
    class ForceRegistry
    {
    public:
        /**
         * @param body Body to receive force updates.
         * @param generator Force generator to apply.
         */
        void Add(dynamics::RigidBody* body, const ForceGenerator* generator);

        /**
         * @param body Body registration to remove.
         * @param generator Generator registration to remove.
         */
        void Remove(dynamics::RigidBody* body, const ForceGenerator* generator);

        void Clear();

        /**
         * @param dt Time step in seconds.
         */
        void UpdateForces(float dt) const;

    private:
        struct Registration
        {
            dynamics::RigidBody* body{ nullptr };
            const ForceGenerator* generator{ nullptr };
        };

        std::vector<Registration> m_registrations;
    };
}
