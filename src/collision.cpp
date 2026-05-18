#include "collision.hpp"

namespace SoftToss
{

    Vec3 normalForce(const BallState &state, const Collider &collider, const Vec3 &F)
    {
        Vec3 n_hat = (state.position - collider.point).normalized(); // normal at contact point from surface to ball
        float F_n = -dot(F, n_hat);
        if (F_n < 0.0f)
            F_n = 0.0f; // no tensile normal force
        return F_n * n_hat;
    };

    Vec3 frictionForce(const BallSpec &spec, const BallState &state, const Collider &collider, const Vec3 &F)
    {
        Vec3 n_hat = (state.position - collider.point).normalized();                                                   // normal vector from ground up to ball center
        Vec3 v_slip = (state.velocity - dot(state.velocity, n_hat) * n_hat) - spec.radius * cross(state.omega, n_hat); // tangential slip velocity at contact point

        const float N = normalForce(state, collider, F).mag(); // magnitude of normal force
        const float mu_s = spec.mu_s[collider.type];           // static coefficient of friction
        const float mu_k = spec.mu_k[collider.type];           // kinetic coefficient of friction

        if (v_slip.mag2() > 1e-6f)
        {
            Vec3 F_f = -mu_k * N * v_slip.normalized(); // kinetic friction force
            return F_f;
        }
        else
        {
            Vec3 F_req = F + N * n_hat; // required friction force to prevent slip
            if (F_req.mag() < mu_s * N)
            {
                return -F_req; // static friction force
            }
            else
            {
                return -mu_k * N * F_req.normalized(); // kinetic friction force
            }
        }
    };

    Vec3 frictionTorque(const BallSpec &spec, const BallState &state, const Collider &collider, const Vec3 &F_fric, const Vec3 &F_norm)
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
        const float e_n = spec.e_n[collider.type];   // normal coefficient of restitution
        const float e_t = spec.e_t[collider.type];   // tangential coefficient of restitution
        const float mu_s = spec.mu_s[collider.type]; // static coefficient of friction
        const float mu_k = spec.mu_k[collider.type]; // kinetic coefficient of friction

        const Vec3 n_hat = (state.position - collider.point).normalized();                            // normal vector from surface to ball center at contact
        const Vec3 v_contact_ball = state.velocity - spec.radius * cross(state.omega, n_hat);         // velocity at ball contact point
        const Vec3 v_contact_collider = collider.velocity + cross(collider.omega, collider.leverArm); // velocity at collider contact point
        const Vec3 v_rel = v_contact_ball - v_contact_collider;                                       // relative velocity at contact point

        const Vec3 v_slip = v_rel - dot(v_rel, n_hat) * n_hat; // tangential slip velocity at contact point
        const float v_slip_mag = v_slip.mag();                 // magnitude of slip velocity for determining friction direction

        const Vec3 refAxis = (std::abs(n_hat.y) > 0.99f) // reference axis for defining tangential direction (avoid near-parallel case)
                                 ? Vec3(1.0f, 0.0f, 0.0f)
                                 : Vec3(0.0f, 1.0f, 0.0f);
        const Vec3 t_hat = (v_slip_mag > 1e-6f) // tangential direction along slip velocity if significant, otherwise use reference axis
                               ? (v_slip / v_slip_mag)
                               : cross(n_hat, refAxis).normalized();
        const Vec3 b_hat = cross(n_hat, t_hat).normalized(); // binormal direction perpendicular to both normal and tangential directions

        const float ball_invMass = 1.0f / spec.mass;                  // inverse mass of the ball
        const float ball_invI = (spec.radius * spec.radius) / spec.I; // inverse moment of inertia for rotation about axis through contact point

        const float invEffMass_n = ball_invMass + collider.invMass + cross(collider.leverArm, n_hat).mag2() * collider.invI_0;             // effective inverse mass for normal direction
        const float invEffMass_t = ball_invMass + ball_invI + collider.invMass + cross(collider.leverArm, t_hat).mag2() * collider.invI_0; //  inverse effective mass for tangential direction
        const float invEffMass_b = ball_invMass + ball_invI + collider.invMass + cross(collider.leverArm, b_hat).mag2() * collider.invI_z; //  inverse effective mass for binormal direction

        const float j_n_mag = std::max(0.0f, -(1.0f + e_n) * dot(v_rel, n_hat) / invEffMass_n); // magnitude of normal impulse (clamped to prevent tensile force)
        const Vec3 J_n = j_n_mag * n_hat;                                                       // normal impulse vector

        const Vec3 J_grip_t = (-(1.0f + e_t) * dot(v_rel, t_hat) / invEffMass_t) * t_hat; // tangential component of grip impulse
        const Vec3 J_grip_b = (-(1.0f + e_t) * dot(v_rel, b_hat) / invEffMass_b) * b_hat; // binormal component of grip impulse
        const Vec3 J_grip = J_grip_t + J_grip_b;                                          // total grip impulse vector

        const Vec3 J_t = (J_grip.mag() <= mu_s * j_n_mag) ? J_grip : -(mu_k * j_n_mag) * t_hat; // applied tangential impulse vector (either grip or sliding friction)

        const Vec3 J = J_n + J_t; // total impulse vector
        BallState newState = state;
        newState.velocity = newState.velocity + J / spec.mass;
        newState.omega = newState.omega + cross(-spec.radius * n_hat, J) / spec.I;

        return newState;
    };

}; // namespace SoftToss
