#pragma once
#include "vec3.hpp"

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

        float e_n[static_cast<int>(ColliderType::Count)]; // restitution coefficients for different collider types
        float e_t[static_cast<int>(ColliderType::Count)]; // tangential restitution coefficients for different collider types
        float mu[static_cast<int>(ColliderType::Count)];  // friction coefficients for different collider types

        // float c_d0 = 0.3008f; // zero-lift drag coefficient
        // float c_l0 = 0.583f; // zero-spin lift coefficient
        // float c_l1 = 2.333f; // lift coefficient linear term for spin
        // float c_l2 = 1.120f; // lift coefficient quadratic term for spin
    };

} // namespace SoftToss
