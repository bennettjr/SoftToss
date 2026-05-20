#pragma once
#include "vec3.hpp"
#include <array>

namespace SoftToss
{

    struct DragModel
    {
        float cd_laminar;
        float cd_turbulent;

        float Re_crit;
        float Re_width;

        float k_spin;
        float S_crit;
    };

    struct LiftModel
    {
        float c_l0;
        float c_l1;
        float c_l2;

        float Re_rev;
        float Re_width;

        float c_l_rev;
        float S_rev;
    };

    struct CorModel
    {
        std::array<float, 32> e_n; // restitution coefficients for different collider types
        std::array<float, 32> k;   // velocity decay rate s/ft
        float beta;                // temperature coefficient 1 / deg F

        std::array<float, 32> e_t; // tangential restitution coefficients for different collider types
    };

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

        std::array<float, 32> e_n;  // restitution coefficients for different collider types
        std::array<float, 32> e_t;  // tangential restitution coefficients for different collider types
        std::array<float, 32> mu_s; // static friction coefficients for different collider types
        std::array<float, 32> mu_k; // kinetic friction coefficients for different collider types
        std::array<float, 32> c_rr; // rolling resistance coefficients for different collider types

        DragModel dragModel;
        LiftModel liftModel;
        CorModel corModel;
    };

} // namespace SoftToss
