#include "SoftToss.h"

#include "solver.hpp"

#include <cstddef>
#include <optional>

namespace SoftToss
{
    namespace
    {
        Vec3 toVec3(const ST_Vec3 &value)
        {
            return {value.x, value.y, value.z};
        }

        ST_Vec3 fromVec3(const Vec3 &value)
        {
            return {value.x, value.y, value.z};
        }

        BallState toBallState(const ST_BallState &value)
        {
            BallState state;
            state.position = toVec3(value.position);
            state.velocity = toVec3(value.velocity);
            state.omega = toVec3(value.omega);
            state.acceleration = toVec3(value.acceleration);
            return state;
        }

        ST_BallState fromBallState(const BallState &value)
        {
            ST_BallState state;
            state.position = fromVec3(value.position);
            state.velocity = fromVec3(value.velocity);
            state.omega = fromVec3(value.omega);
            state.acceleration = fromVec3(value.acceleration);
            return state;
        }

        BallSpec toBallSpec(const ST_BallSpec &value)
        {
            BallSpec spec;
            spec.mass = value.mass;
            spec.radius = value.radius;
            spec.I = value.I;

            for (std::size_t index = 0; index < ST_COLLIDER_COUNT; ++index)
            {
                spec.e_n[index] = value.e_n[index];
                spec.e_t[index] = value.e_t[index];
                spec.mu_s[index] = value.mu_s[index];
                spec.mu_k[index] = value.mu_k[index];
            }

            return spec;
        }

        Environment toEnvironment(const ST_Environment &value)
        {
            Environment environment;
            environment.gravity = value.gravity;
            environment.temp = value.temp;
            environment.elev = value.elev;
            environment.humid = value.humid;
            environment.pres = value.pres;
            environment.rho = value.rho;
            environment.windSpeed = value.windSpeed;
            environment.windDir = value.windDir;
            environment.windHeight = value.windHeight;
            return environment;
        }

        Collider toCollider(const ST_Collider &value)
        {
            Collider collider;
            collider.type = value.type;
            collider.point = toVec3(value.point);
            collider.velocity = toVec3(value.velocity);
            collider.omega = toVec3(value.omega);
            collider.leverArm = toVec3(value.leverArm);
            collider.invMass = value.invMass;
            collider.invI_0 = value.invI_0;
            collider.invI_z = value.invI_z;
            return collider;
        }

        Integrator toIntegrator(ST_Integrator value)
        {
            switch (value)
            {
            case ST_Integrator_RK4:
                return Integrator::RK4;
            case ST_Integrator_Euler:
            default:
                return Integrator::Euler;
            }
        }
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