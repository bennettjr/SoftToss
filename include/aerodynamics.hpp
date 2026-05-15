#pragma once
#include "vec3.hpp"
#include "ball.hpp"
#include "environment.hpp"
#include <optional>

namespace SoftToss
{

    Vec3 dragForce(const Vec3 &v_rel, const float &w_act, const float &kappa);
    Vec3 magnusForce(const BallSpec &spec, const BallState &state, const Vec3 &v_rel, const float &w_act, const float &kappa);
    Vec3 gravityForce();
    Vec3 sswForce();

} // namespace SoftToss
