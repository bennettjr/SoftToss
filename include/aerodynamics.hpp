#pragma once
#include "vec3.hpp"
#include "ball.hpp"
#include "environment.hpp"
#include "collider.hpp"
#include <optional>

namespace SoftToss
{

    Vec3 dragForce(const Vec3 &v_rel, const float &w_act, const float &kappa);
    Vec3 magnusForce(const BallSpec &spec, const BallState &state, const Vec3 &v_rel, const float &w_act, const float &kappa);
    Vec3 gravityForce(const BallSpec &spec);
    Vec3 sswForce();
    Vec3 spindownTorque(const BallSpec &spec, const BallState &state, Vec3 &F);

} // namespace SoftToss
