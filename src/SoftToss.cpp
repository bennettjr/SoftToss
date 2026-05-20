#include "SoftToss.h"

#include "solver.hpp"

#include <cstddef>
#include <optional>
#include <cstring>

namespace SoftToss
{
    namespace
    {
        inline Vec3 toVec3(const ST_Vec3 &v) noexcept { return {v.x, v.y, v.z}; }
        inline ST_Vec3 fromVec3(const Vec3 &v) noexcept { return {v.x, v.y, v.z}; }

        inline BallState toBallState(const ST_BallState &s) noexcept
        {
            return {toVec3(s.position), toVec3(s.velocity), toVec3(s.omega), toVec3(s.acceleration)};
        }

        inline ST_BallState fromBallState(const BallState &s) noexcept
        {
            return {fromVec3(s.position), fromVec3(s.velocity), fromVec3(s.omega), fromVec3(s.acceleration)};
        }

        inline BallSpec toBallSpec(const ST_BallSpec &s) noexcept
        {
            BallSpec out;
            out.mass = s.mass;
            out.radius = s.radius;
            out.I = s.I;

            // copy contiguous float arrays
            std::memcpy(out.e_n.data(), s.e_n, sizeof(float) * ST_COLLIDER_COUNT);
            std::memcpy(out.e_t.data(), s.e_t, sizeof(float) * ST_COLLIDER_COUNT);
            std::memcpy(out.mu_s.data(), s.mu_s, sizeof(float) * ST_COLLIDER_COUNT);
            std::memcpy(out.mu_k.data(), s.mu_k, sizeof(float) * ST_COLLIDER_COUNT);
            std::memcpy(out.c_rr.data(), s.c_rr, sizeof(float) * ST_COLLIDER_COUNT);

            // copy nested models
            out.dragModel = {s.dragModel.cd_laminar, s.dragModel.cd_turbulent, s.dragModel.Re_crit, s.dragModel.Re_width, s.dragModel.k_spin, s.dragModel.S_crit};
            out.liftModel = {s.liftModel.c_l0, s.liftModel.c_l1, s.liftModel.c_l2, s.liftModel.Re_rev, s.liftModel.Re_width, s.liftModel.c_l_rev, s.liftModel.S_rev};

            // corModel: copy arrays
            std::memcpy(out.corModel.e_n.data(), s.corModel.e_n, sizeof(float) * ST_COLLIDER_COUNT);
            std::memcpy(out.corModel.k.data(), s.corModel.k, sizeof(float) * ST_COLLIDER_COUNT);
            out.corModel.beta = s.corModel.beta;
            std::memcpy(out.corModel.e_t.data(), s.corModel.e_t, sizeof(float) * ST_COLLIDER_COUNT);

            return out;
        }

        inline Environment toEnvironment(const ST_Environment &e) noexcept
        {
            Environment out;
            out.gravity = e.gravity;
            out.temp = e.temp;
            out.elev = e.elev;
            out.humid = e.humid;
            out.pres = e.pres;
            out.mu = e.mu;
            out.rho = e.rho;
            out.windSpeed = e.windSpeed;
            out.windDir = e.windDir;
            out.windHeight = e.windHeight;
            return out;
        }

        inline Collider toCollider(const ST_Collider &c) noexcept
        {
            Collider out;
            out.type = c.type;
            out.point = toVec3(c.point);
            out.velocity = toVec3(c.velocity);
            out.omega = toVec3(c.omega);
            out.leverArm = toVec3(c.leverArm);
            out.invMass = c.invMass;
            out.invI_0 = c.invI_0;
            out.invI_z = c.invI_z;
            return out;
        }

        inline Integrator toIntegrator(ST_Integrator i) noexcept { return (i == ST_Integrator_RK4) ? Integrator::RK4 : Integrator::Euler; }
    } // namespace
} // namespace SoftToss

extern "C"
{
    void SoftToss_updateState(const ST_BallSpec *spec, const ST_BallState *state, const ST_Environment *env, float dt, ST_Integrator integrator, const ST_Collider *collider, ST_BallState *out)
    {
        if (!spec || !state || !env || !out)
        {
            return;
        }

        const SoftToss::BallSpec cppSpec = SoftToss::toBallSpec(*spec);
        const SoftToss::BallState cppState = SoftToss::toBallState(*state);
        const SoftToss::Environment cppEnv = SoftToss::toEnvironment(*env);

        std::optional<SoftToss::Collider> cppCollider;
        if (collider)
        {
            cppCollider = SoftToss::toCollider(*collider);
        }

        const SoftToss::BallState result = SoftToss::updateState(cppSpec, cppState, cppEnv, dt, SoftToss::toIntegrator(integrator), cppCollider);
        *out = SoftToss::fromBallState(result);
    }
}