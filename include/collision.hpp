#pragma once
#include "vec3.hpp"
#include "ball.hpp"
#include "collider.hpp"

namespace SoftToss
{

    Vec3 normalForce(const BallState &state, const Collider &collider, Vec3 &F);
    Vec3 frictionForce(const BallSpec &spec, const BallState &state, const Collider &collider, Vec3 &F);
    Vec3 spindownTorque(const BallSpec &spec, const BallState &state, const Collider &collider, Vec3 &F);
    Vec3 frictionTorque(const BallSpec &spec, const BallState &state, const Collider &collider, Vec3 &F_fric, Vec3 &F_norm);
    BallState collision(const BallSpec &spec, const BallState &state, const Collider &collider);

} // namespace SoftToss
