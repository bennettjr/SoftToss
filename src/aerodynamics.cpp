#include "aerodynamics.hpp"
#include <numbers> // for std::numbers::pi_v
#include <cmath>   // for std::tanh, std::exp, std::sqrt

namespace SoftToss
{

    Wrench aeroWrench(const BallSpec &spec, const BallState &state, const Environment &env)
    {
        constexpr float degToRad = std::numbers::pi_v<float> / 180.0f;

        constexpr float z_0 = 0.1f;                                                                      // roughness length for wind profile (ft)
        constexpr float z_ref = 10.0f;                                                                   // reference height for wind speed (ft)
        const float windScale = std::log(std::max(state.position.z, z_0) / z_0) / std::log(z_ref / z_0); // wind speed scaling factor
        const Vec3 windVel = {env.windSpeed * windScale * std::cos(env.windDir * degToRad), env.windSpeed * windScale * std::sin(env.windDir * degToRad), 0.0f};

        const float kappa = 0.5f * env.rho * (std::numbers::pi_v<float> * spec.radius * spec.radius); // constant for force calculations

        const Vec3 v_rel = state.velocity - windVel; // relative velocity ft/s
        const float v_rel_mag = v_rel.mag();
        const float w_parallel = dot(state.omega, v_rel) / v_rel_mag; // spin component parallel to velocity
        const float w_act = (v_rel_mag > 1e-6)                        // active spin rad/s
                                ? std::sqrt(std::max(0.0f, state.omega.mag2() - w_parallel * w_parallel))
                                : 0.0f;

        const float S = (v_rel_mag > 1e-6f) // spin parameter
                            ? (spec.radius * w_act) / v_rel_mag
                            : 0.0f;

        const float Re = (env.mu > 1e-9f) // Reynolds number
                             ? (env.rho * v_rel_mag * 2.0f * spec.radius) / env.mu
                             : 0.0f;

        Vec3 F_drag = dragForce(spec, v_rel, w_act, S, Re, kappa);
        Vec3 F_mag = magnusForce(spec, state, v_rel, w_act, S, Re, kappa);
        Vec3 F_grav = gravityForce(spec, env.gravity);
        Vec3 F_ssw = sswForce();
        Vec3 T_sd = spindownTorque(spec, state, F_mag);

        return Wrench{F_drag + F_mag + F_grav + F_ssw, T_sd};
    }

    Vec3 dragForce(const BallSpec &spec, const Vec3 &v_rel, const float w_act, const float S, const float Re, const float kappa) // drag force slug*ft/s^2
    {
        if (v_rel.mag2() < 1e-6f)
            return Vec3();

        const float cd_base = spec.dragModel.cd_turbulent + (spec.dragModel.cd_laminar - spec.dragModel.cd_turbulent) * 0.5f * (1.0f + std::tanh((spec.dragModel.Re_crit - Re) / spec.dragModel.Re_width)); // base drag coefficient

        const float dS = S - spec.dragModel.S_crit;
        const float cd_spin = spec.dragModel.k_spin * std::sqrt(dS * dS + 1e-6f);

        const float c_d = cd_base + cd_spin;              // total drag coefficient
        Vec3 F_drag = -kappa * c_d * v_rel.mag() * v_rel; // drag force slug*ft/s^2
        return F_drag;
    }

    Vec3 magnusForce(const BallSpec &spec, const BallState &state, const Vec3 &v_rel, const float w_act, const float S, const float Re, const float kappa) // Magnus force slug*ft/s^2
    {
        if (w_act < 1e-6f)
            return Vec3();

        // Normal Magnus: Saturation form (Nathan curve)
        const float cl_normal = (spec.liftModel.c_l2 * S) / (spec.liftModel.c_l0 + spec.liftModel.c_l1 * S);

        // Reverse Magnus: Negative lift at low S, Re-gated to the drag crisis region
        const float re_gate = 0.5f * (1.0f + std::tanh((spec.liftModel.Re_rev - Re) / spec.liftModel.Re_width));
        const float cl_rev = -spec.liftModel.c_l_rev * S * std::exp(-S / spec.liftModel.S_rev) * re_gate;

        const float c_l = cl_normal + cl_rev;

        // Magnus force direction: perpendicular to both spin axis and relative velocity
        Vec3 F_mag = kappa * c_l * (v_rel.mag() / w_act) * cross(state.omega, v_rel);
        return F_mag;
    }

    Vec3 gravityForce(const BallSpec &spec, const float gravity) // gravitational force slug*ft/s^2
    {
        return Vec3{0.0f, 0.0f, spec.mass * gravity};
    }

    Vec3 sswForce() // shifted seam wake force slug*ft/s^2
    {
        return Vec3();
    }

    Vec3 spindownTorque(const BallSpec &spec, const BallState &state, const Vec3 &F) // spindown torque slug*ft^2/s^2
    {
        constexpr auto k = 0.02f;                                                                                      // Torque Parameter (Check this value)
        Vec3 torque = (state.omega.mag2() > 1e-6f) ? k * spec.radius * F.mag() * (-state.omega.normalized()) : Vec3(); // torque slug*ft^2/s^2
        return torque;
    }

} // namespace SoftToss
