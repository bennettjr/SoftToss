#include "collision.hpp"
#include <cmath>

namespace SoftToss
{

    Vec3 normalForce(const BallState &state, const Collider &collider, Vec3 &F)
    {
        Vec3 n_hat = (state.position - collider.point).normalized(); // normal at contact point from surface to ball
        float F_n = -dot(F, n_hat);
        if (F_n < 0.0f)
            F_n = 0.0f; // no tensile normal force
        return F_n * n_hat;
    };

    Vec3 frictionForce(const BallSpec &spec, const BallState &state, const Collider &collider, Vec3 &F)
    {
        Vec3 n_hat = (state.position - collider.point).normalized();                                                   // normal vector from ground up to ball center
        Vec3 v_slip = (state.velocity - dot(state.velocity, n_hat) * n_hat) - spec.radius * cross(state.omega, n_hat); // tangential slip velocity at contact point

        const float N = normalForce(state, collider, F).mag();     // magnitude of normal force
        const float mu = spec.mu[static_cast<int>(collider.type)]; // coefficient of friction

        if (v_slip.mag2() > 1e-6f)
        {
            Vec3 F_f = -mu * N * v_slip.normalized(); // kinetic friction force
            return F_f;
        }
        else
        {
            Vec3 F_req = F + N * n_hat; // required friction force to prevent slip
            if (F_req.mag() < mu * N)
            {
                return -F_req; // static friction force
            }
            else
            {
                return -mu * N * F_req.normalized(); // kinetic friction force
            }
        }
    };

    Vec3 frictionTorque(const BallSpec &spec, const BallState &state, const Collider &collider, Vec3 &F_fric, Vec3 &F_norm)
    {
        Vec3 n_hat = (state.position - collider.point).normalized(); // normal vector from ground up to ball center
        Vec3 v_slip = (state.velocity - dot(state.velocity, n_hat) * n_hat) - spec.radius * cross(state.omega, n_hat);

        if (v_slip.mag2() > 1e-6f)
        {
            Vec3 r = -spec.radius * n_hat;  // vector from ball center to contact point
            Vec3 torque = cross(r, F_fric); // friction torque slug*ft^2/s^2
            return torque;
        }
        else
        {
            const float c_rr = 0.1f;
            Vec3 torque = (state.omega.mag2() > 1e-6f) ? -c_rr * F_norm.mag() * spec.radius * state.omega.normalized() : Vec3();
            return torque;
        }
    };

    BallState collision(const BallSpec &spec, const BallState &state, const Collider &collider)
    {
        const float e_n = spec.e_n[static_cast<int>(collider.type)]; // normal coefficient of restitution
        const float e_t = spec.e_t[static_cast<int>(collider.type)]; // tangential coefficient of restitution
        const float mu = spec.mu[static_cast<int>(collider.type)];   // coefficient of friction

        Vec3 n_hat = (state.position - collider.point).normalized(); // normal vector from ground up to ball center

        Vec3 J_n = -spec.mass * (1 + e_n) * dot(state.velocity, n_hat) * n_hat; // normal impulse
        const float J_fmax = mu * J_n.mag();                                    // maximum frictional impulse

        Vec3 v_slip = (state.velocity - dot(state.velocity, n_hat) * n_hat) - spec.radius * cross(state.omega, n_hat);
        Vec3 J_slide = -J_fmax * v_slip.normalized();

        Vec3 J_grip = -(2.0 / 7.0) * spec.mass * (1 + e_t) * v_slip;

        Vec3 J_t = (J_grip.mag() <= J_fmax) ? J_grip : J_slide; // tangential impulse

        Vec3 J = J_n + J_t; // total impulse

        BallState newState = state; // copy current state to modify

        newState.velocity = newState.velocity + J / spec.mass;                     // update ball velocity
        newState.omega = newState.omega + cross(-spec.radius * n_hat, J) / spec.I; // update ball angular velocity
        return newState;
    };

}; // namespace SoftToss
