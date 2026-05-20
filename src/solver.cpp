#include "solver.hpp"

namespace SoftToss
{

    [[nodiscard]] BallState updateState(const BallSpec &spec, const BallState &state, const Environment &env, float dt, Integrator integrator, std::optional<Collider> collider)
    {

        struct StateDerivative
        {
            Vec3 velocity;
            Vec3 acceleration;
            Vec3 alpha;
        };

        auto deriveState = [&](const BallState &st) -> StateDerivative
        {
            const Wrench aero = aeroWrench(spec, st, env);
            Vec3 F_total = aero.force;
            Vec3 T_total = aero.torque;

            if (collider)
            {
                const Wrench coll = contactWrench(spec, st, *collider, F_total);
                F_total += coll.force;
                T_total += coll.torque;
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

    [[nodiscard]] BallState collisionState(const BallSpec &spec, const BallState &state, const Collider &collider)
    {
        const Vec3 n_hat = (state.position - collider.point).normalized();
        const Vec3 J = collisionImpulse(spec, state, collider);
        BallState newState = state;
        newState.velocity = newState.velocity + J / spec.mass;
        newState.omega = newState.omega + cross(-spec.radius * n_hat, J) / spec.I;

        return newState;
    }

} // namespace SoftToss
