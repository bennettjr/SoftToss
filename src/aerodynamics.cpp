#include "aerodynamics.hpp"
#include <cmath>

namespace SoftToss
{

    Vec3 dragForce(const Vec3 &v_rel, const float &w_act, const float &kappa)
    {
        float c_d0 = 0.3008;                                    // drag coefficient for a non-spinning ball at high speed
        const float c_d = c_d0 + (2.92e-5f * (w_act * 9.549f)); // drag coefficient considering velocity and spin
        Vec3 F_drag = -kappa * c_d * v_rel.mag() * v_rel;       // drag force slug*ft/s^2
        return F_drag;
    }

    Vec3 magnusForce(const BallSpec &spec, const BallState &state, const Vec3 &v_rel, const float &w_act, const float &kappa)
    {
        const float s = (v_rel.mag2() > 1e-6) ? (spec.radius * w_act) / v_rel.mag() : 0.0f; // spin factor
        const float c_l0 = 0.583;                                                           // lift coefficient constants
        const float c_l1 = 2.333;
        const float c_l2 = 1.120;
        const float c_l = c_l2 * s / (c_l0 + c_l1 * s);                                                           // lift coefficient
        Vec3 F_mag = (w_act > 1e-6) ? (kappa * c_l * (v_rel.mag() / w_act) * cross(state.omega, v_rel)) : Vec3(); // Magnus force slug*ft/s^2
        return F_mag;
    }

    Vec3 gravityForce(const BallSpec &spec)
    {
        constexpr auto g = -32.174f; // ft/s^2
        return Vec3{0.0f, 0.0f, spec.mass * g};
    }

    Vec3 sswForce()
    {
        return Vec3();
    }

    Vec3 spindownTorque(const BallSpec &spec, const BallState &state, Vec3 &F)
    {
        const auto k = 0.02f;                                                                                          // Torque Parameter (Check this value)
        Vec3 torque = (state.omega.mag2() > 1e-6f) ? k * spec.radius * F.mag() * (-state.omega.normalized()) : Vec3(); // torque slug*ft^2/s^2
        return torque;
    };

}; // namespace SoftToss
