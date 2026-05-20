#pragma once
#include "vec3.hpp"
#include "screw.hpp"
#include "ball.hpp"
#include "environment.hpp"
#include "collider.hpp"
#include <optional>

namespace SoftToss
{

    Wrench aeroWrench(const BallSpec &spec, const BallState &state, const Environment &env);
    Vec3 dragForce(const BallSpec &spec, const Vec3 &v_rel, const float w_act, const float S, const float Re, const float kappa);
    Vec3 magnusForce(const BallSpec &spec, const BallState &state, const Vec3 &v_rel, const float w_act, const float S, const float Re, const float kappa);
    Vec3 gravityForce(const BallSpec &spec, const float gravity);
    Vec3 sswForce();
    Vec3 spindownTorque(const BallSpec &spec, const BallState &state, const Vec3 &F);

} // namespace SoftToss
