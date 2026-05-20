#pragma once
#include "vec3.hpp"

namespace SoftToss
{
    struct Twist
    {
        Vec3 velocity;
        Vec3 omega;
    };

    struct Wrench
    {
        Vec3 force;
        Vec3 torque;
    };
}
