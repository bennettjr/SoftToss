#pragma once
#include "vec3.hpp"
#include <string>
#include <unordered_map>

namespace SoftToss
{

    struct BallState
    {
        Vec3 position;
        Vec3 velocity;
        Vec3 omega;
        Vec3 acceleration;
    };

    struct BallSpec
    {
        float mass;   // slugs
        float radius; // ft
        float I;      // slug*ft^2

        float c_d0; // zero-lift drag coefficient
        float c_d;  // drag coefficient linear term for spin
        float c_l0; // zero-spin lift coefficient
        float c_l1; // lift coefficient linear term for spin
        float c_l2; // lift coefficient quadratic term for spin
    };

} // namespace SoftToss
