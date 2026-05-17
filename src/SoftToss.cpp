#include "SoftToss.h"

#include <optional>

#include "ball.hpp"
#include "environment.hpp"
#include "collider.hpp"

namespace SoftToss
{
    // Forward-declare the solver implementation (solver.cpp defines this).
    [[nodiscard]] BallState updateState(const BallSpec &spec, const BallState &state, const Environment &env, float dt, std::optional<Collider> collider = std::nullopt);

} // namespace SoftToss

extern "C"
{

    void SoftToss_updateState(const ST_BallSpec *spec, const ST_BallState *state, const ST_Environment *env, float dt, const ST_Collider *collider, ST_BallState *out)
    {
        using namespace SoftToss;
        if (!spec || !state || !env || !out)
            return;

        BallSpec cpp_spec;
        cpp_spec.mass = spec->mass;
        cpp_spec.radius = spec->radius;
        cpp_spec.I = spec->I;
        // cpp_spec.c_d0 = spec->c_d0;
        // cpp_spec.c_d = spec->c_d;
        // cpp_spec.c_l0 = spec->c_l0;
        // cpp_spec.c_l1 = spec->c_l1;
        // cpp_spec.c_l2 = spec->c_l2;

        BallState cpp_state;
        cpp_state.position = {state->position.x, state->position.y, state->position.z};
        cpp_state.velocity = {state->velocity.x, state->velocity.y, state->velocity.z};
        cpp_state.omega = {state->omega.x, state->omega.y, state->omega.z};
        cpp_state.acceleration = {state->acceleration.x, state->acceleration.y, state->acceleration.z};

        Environment cpp_env;
        cpp_env.temp = env->temp;
        cpp_env.elev = env->elev;
        cpp_env.humid = env->humid;
        cpp_env.pres = env->pres;
        cpp_env.rho = env->rho;
        cpp_env.windSpeed = env->windSpeed;
        cpp_env.windDir = env->windDir;
        cpp_env.windHeight = env->windHeight;

        std::optional<Collider> optColl;
        if (collider)
        {
            Collider c;
            c.type = static_cast<ColliderType>(collider->type);
            c.point = {collider->point.x, collider->point.y, collider->point.z};
            optColl = c;
        }

        BallState res = updateState(cpp_spec, cpp_state, cpp_env, dt, optColl);

        out->position.x = res.position.x;
        out->position.y = res.position.y;
        out->position.z = res.position.z;

        out->velocity.x = res.velocity.x;
        out->velocity.y = res.velocity.y;
        out->velocity.z = res.velocity.z;

        out->omega.x = res.omega.x;
        out->omega.y = res.omega.y;
        out->omega.z = res.omega.z;

        out->acceleration.x = res.acceleration.x;
        out->acceleration.y = res.acceleration.y;
        out->acceleration.z = res.acceleration.z;
    }

} // extern "C"

// static_assert(static_cast<int>(SurfaceType::Count) == 4,
//     "ST_BallSpec surface arrays need updating to match SurfaceType::Count");