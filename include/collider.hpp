#pragma once
#include "vec3.hpp"

namespace SoftToss
{
    enum class ColliderType
    {
        Dirt,
        Grass,
        // Add more types as needed
    };

    struct Collider
    {
        ColliderType type;
        Vec3 point;
    };

} // namespace SoftToss
