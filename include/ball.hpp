#pragma once
#include "vec3.hpp"
#include <vector>

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

        std::vector<float> e_n;  // restitution coefficients for different collider types
        std::vector<float> e_t;  // tangential restitution coefficients for different collider types
        std::vector<float> mu_s; // static friction coefficients for different collider types
        std::vector<float> mu_k; // kinetic friction coefficients for different collider types

        // float c_d0 = 0.3008f; // zero-lift drag coefficient
        // float c_l0 = 0.583f; // zero-spin lift coefficient
        // float c_l1 = 2.333f; // lift coefficient linear term for spin
        // float c_l2 = 1.120f; // lift coefficient quadratic term for spin
    };

} // namespace SoftToss
