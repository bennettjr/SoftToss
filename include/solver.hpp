#pragma once
#include "aerodynamics.hpp"
#include "collision.hpp"
#include <optional>

namespace SoftToss
{

    enum class Integrator
    {
        Euler,
        RK4
    };

    [[nodiscard]] BallState updateState(const BallSpec &spec, const BallState &state, const Environment &env, float dt, Integrator integrator, std::optional<Collider> contact = std::nullopt);

} // namespace SoftToss
