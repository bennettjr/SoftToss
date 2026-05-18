#include "solver.hpp"
#include <numbers>

namespace SoftToss
{

    [[nodiscard]] BallState updateState(const BallSpec &spec, const BallState &state, const Environment &env, float dt, std::optional<Collider> collider)
    {
        constexpr float degToRad = std::numbers::pi_v<float> / 180.0f;
        const Vec3 windVel = (state.position.z >= env.windHeight) // wind velocity ft/s
                                 ? Vec3{env.windSpeed * std::cos(env.windDir * degToRad), env.windSpeed * std::sin(env.windDir * degToRad), 0.0}
                                 : Vec3();
        const Vec3 v_rel = state.velocity - windVel;       // relative velocity ft/s
        const float w_act = (state.velocity.mag2() > 1e-6) // active spin rad/s
                                ? std::sqrt(state.omega.mag2() - std::pow(dot(state.omega, state.velocity) / state.velocity.mag(), 2))
                                : 0.0f;
        const float kappa = 0.5f * env.rho * (std::numbers::pi_v<float> * spec.radius * spec.radius); // constant for force calculations

        const Vec3 F_drag = dragForce(v_rel, w_act, kappa);               // drag force slug*ft/s^2
        const Vec3 F_mag = magnusForce(spec, state, v_rel, w_act, kappa); // Magnus force slug*ft/s^2
        const Vec3 F_grav = gravityForce(spec, env);                      // gravitational force slug*ft/s^2
        const Vec3 F_ssw = sswForce();                                    // shifted seam wake force slug*ft/s^2
        const Vec3 T_sd = spindownTorque(spec, state, F_mag);             // spindown torque slug*ft^2/s^2

        Vec3 F_total = F_drag + F_mag + F_grav + F_ssw; // total force slug*ft/s^2
        Vec3 T_total = T_sd;                            // total torque slug*ft^2/s^2

        if (collider)
        {
            const Vec3 F_norm = normalForce(state, *collider, F_total);
            const Vec3 F_fric = frictionForce(spec, state, *collider, F_total);
            const Vec3 T_fric = frictionTorque(spec, state, *collider, F_fric, F_norm);

            F_total += F_norm + F_fric;
            T_total += T_fric;
        }

        BallState newState = state; // copy current state to modify

        Vec3 alpha = (spec.I > 1e-6f) ? T_total / spec.I : Vec3(); // angular acceleration rad/s^2

        // Update acceleration
        newState.acceleration = F_total / spec.mass; // ft/s^2

        // Update velocity
        newState.velocity = newState.velocity + newState.acceleration * dt;

        // Update position
        newState.position = newState.position + newState.velocity * dt;

        // Update omega
        newState.omega = newState.omega + alpha * dt;

        if (F_total.mag2() < 1e-6f && state.velocity.mag2() < 1e-1f)
        {
            newState.velocity = Vec3();
            newState.omega = Vec3();
            newState.acceleration = Vec3();
        }
        return newState;
    }

} // namespace SoftToss
