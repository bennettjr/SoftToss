#pragma once
#include "vec3.hpp"
#include <cstdint>

namespace SoftToss
{

    struct Collider
    {
        std::uint8_t type; // collider type index for restitution and friction lookup
        Vec3 point;        // point of contact on collider surface
        Vec3 velocity;     // velocity at center of mass
        Vec3 omega;        // angular velocity at center of mass
        Vec3 leverArm;     // vector from collider point to ball center at contact
        float invMass;     // inverse mass (0 for immovable objects)
        float invI_0;      // inverse moment of inertia about axis perpendicular to contact plane (0 for infinite inertia)
        float invI_z;      // inverse moment of inertia about axis parallel to contact plane (0 for infinite inertia)
    };

} // namespace SoftToss
