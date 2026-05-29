#pragma once
#include "vec3.hpp"
#include "screw.hpp"
#include "ball.hpp"
#include "collider.hpp"
#include "environment.hpp"

namespace SoftToss
{
    Wrench contactWrench(const BallSpec &spec, const BallState &state, const Collider &collider, const Vec3 &F);
    Vec3 normalForce(const BallState &state, const Collider &collider, const Vec3 &F);
    Vec3 frictionForce(const BallSpec &spec, const BallState &state, const Collider &collider, const Vec3 &F);
    Vec3 frictionTorque(const BallSpec &spec, const BallState &state, const Collider &collider, const Vec3 &F_fric, const Vec3 &F);
    Vec3 collisionImpulse(const BallSpec &spec, const BallState &state, const Collider &collider, const Environment &env);

} // namespace SoftToss
