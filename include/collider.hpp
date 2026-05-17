#pragma once
#include "vec3.hpp"

namespace SoftToss
{

    enum class ColliderType : int
    {
        Dirt = 0,
        Grass = 1,
        Wall = 2,
        Wood = 3,
        Aluminum = 4,
        Count
    };

    struct Collider
    {
        ColliderType type;
        Vec3 point;
    };

} // namespace SoftToss
