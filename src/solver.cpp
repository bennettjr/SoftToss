#include "solver.hpp"
#include <numbers>

namespace SoftToss
{

    [[nodiscard]] BallState updateState(const BallSpec &spec, const BallState &state, const Environment &env, float dt, Integrator integrator, std::optional<Collider> collider)
    {

        constexpr float degToRad = std::numbers::pi_v<float> / 180.0f;
        const Vec3 windVel = (state.position.z >= env.windHeight) // wind velocity ft/s
                                 ? Vec3{env.windSpeed * std::cos(env.windDir * degToRad), env.windSpeed * std::sin(env.windDir * degToRad), 0.0f}
                                 : Vec3();

        const float kappa = 0.5f * env.rho * (std::numbers::pi_v<float> * spec.radius * spec.radius); // constant for force calculations

        struct StateDerivative
        {
            Vec3 velocity;
            Vec3 acceleration;
            Vec3 alpha;
        };

        auto deriveState = [&](const BallState &st) -> StateDerivative
        {
            const Vec3 v_rel = st.velocity - windVel; // relative velocity ft/s
            const float w_act = (v_rel.mag2() > 1e-6) // active spin rad/s
                                    ? std::sqrt(std::max(0.0f, st.omega.mag2() - (dot(st.omega, v_rel) / v_rel.mag()) * (dot(st.omega, v_rel) / v_rel.mag())))
                                    : 0.0f;

            const Vec3 F_drag = dragForce(v_rel, w_act, kappa);            // drag force slug*ft/s^2
            const Vec3 F_mag = magnusForce(spec, st, v_rel, w_act, kappa); // Magnus force slug*ft/s^2
            const Vec3 F_grav = gravityForce(spec, env);                   // gravitational force slug*ft/s^2
            const Vec3 F_ssw = sswForce();                                 // shifted seam wake force slug*ft/s^2
            const Vec3 T_sd = spindownTorque(spec, st, F_mag);             // spindown torque slug*ft^2/s^2

            Vec3 F_total = F_drag + F_mag + F_grav + F_ssw; // total force slug*ft/s^2
            Vec3 T_total = T_sd;                            // total torque slug*ft^2/s^2

            if (collider)
            {
                const Vec3 F_norm = normalForce(st, *collider, F_total);
                const Vec3 F_fric = frictionForce(spec, st, *collider, F_total);
                const Vec3 T_fric = frictionTorque(spec, st, *collider, F_fric, F_norm);

                F_total += F_norm + F_fric;
                T_total += T_fric;
            }

            return StateDerivative{st.velocity, F_total / spec.mass, (spec.I > 1e-6f) ? T_total / spec.I : Vec3()};
        };

        auto integrateEuler = [&](const BallState &oldState) -> BallState // symplectic Euler integration
        {
            const StateDerivative derivative = deriveState(oldState);

            BallState newState = oldState; // copy current state to modify
            newState.acceleration = derivative.acceleration;
            newState.velocity = oldState.velocity + derivative.acceleration * dt;
            newState.position = oldState.position + newState.velocity * dt;
            newState.omega = oldState.omega + derivative.alpha * dt;
            return newState;
        };

        auto integrateRK4 = [&](const BallState &oldState) -> BallState // Runge-Kutta 4th order integration
        {
            auto applyDerivative = [&](const BallState &st, const StateDerivative &d, float t) -> BallState
            {
                return {st.position + d.velocity * t, st.velocity + d.acceleration * t, st.omega + d.alpha * t, d.acceleration};
            };

            const StateDerivative k1 = deriveState(oldState);                                 // derivative at start of step
            const StateDerivative k2 = deriveState(applyDerivative(oldState, k1, dt * 0.5f)); // derivative at midpoint using k1
            const StateDerivative k3 = deriveState(applyDerivative(oldState, k2, dt * 0.5f)); // derivative at midpoint using k2 (more accurate)
            const StateDerivative k4 = deriveState(applyDerivative(oldState, k3, dt));        // derivative at end of step using k3

            const Vec3 dx = (k1.velocity + 2.0f * k2.velocity + 2.0f * k3.velocity + k4.velocity) * (dt / 6.0f);
            const Vec3 dv = (k1.acceleration + 2.0f * k2.acceleration + 2.0f * k3.acceleration + k4.acceleration) * (dt / 6.0f);
            const Vec3 dw = (k1.alpha + 2.0f * k2.alpha + 2.0f * k3.alpha + k4.alpha) * (dt / 6.0f);

            BallState newState = oldState;
            newState.position = oldState.position + dx;
            newState.velocity = oldState.velocity + dv;
            newState.omega = oldState.omega + dw;
            newState.acceleration = k1.acceleration;
            return newState;
        };

        switch (integrator)
        {
        case Integrator::Euler:
            return integrateEuler(state);
        case Integrator::RK4:
            return integrateRK4(state);
        default:
            return integrateEuler(state);
        }
    }

} // namespace SoftToss
