#pragma once
#include "aerodynamics.hpp"
#include "collision.hpp"

namespace SoftToss
{

    [[nodiscard]] BallState updateState(const BallSpec &spec, const BallState &state, const Environment &env, float dt, std::optional<Contact> contact = std::nullopt);

} // namespace SoftToss
