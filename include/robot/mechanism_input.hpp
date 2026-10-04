#pragma once
#include "robot/mechanism.hpp"
#include "pros/error.h"
#include <cstddef>
#include <limits>

namespace robot::mechanism {
template<class Encoder>
void readForwardPodInput(Input& in, Encoder* encoder) {
    in.forwardPodDegrees = std::numeric_limits<double>::quiet_NaN();
    if (!encoder) return;
    const auto position = encoder->get_position();
    if (position != PROS_ERR) in.forwardPodDegrees = position / 100.0;
}

// Shared with host tests: keep missing-device detection separate from a small
// disagreement between otherwise valid lift encoders.
template<class Encoder, class Arm, class Lift>
Input readMechanismInput(Encoder& encoder, Arm& arm, Lift& lift,
                         std::uint32_t now, std::size_t liftCount) {
    Input in;
    in.now = now;
    const auto invalid = [&](const char* reason) {
        if (in.valid) in.problem = reason;
        in.valid = false;
    };
    const auto position = encoder.get_position();
    const auto reversed = encoder.get_reversed();
    in.arm = position / (100.0 * cfg::encoderDegreesPerArmDegree);
    if (position == PROS_ERR || reversed == PROS_ERR) invalid("Rotation braccio assente");
    else if (reversed != static_cast<int>(cfg::encoderPort < 0)) invalid("Rotation: verso cambiato");
    in.armCurrent = arm.get_current_draw();
    in.armVelocity = arm.get_actual_velocity();
    if (!std::isfinite(in.armCurrent) || in.armCurrent == PROS_ERR || in.armCurrent < 0
        || !std::isfinite(in.armVelocity) || std::abs(in.armVelocity) > 1000)
        invalid("Motore braccio non disponibile");
    double minCurrent = std::numeric_limits<double>::infinity(), maxCurrent = 0, maxVelocity = 0;
    double firstLift = 0;
    if (!liftCount) invalid("Motori lift non configurati");
    for (std::size_t i = 0; i < liftCount; ++i) {
        const auto index = static_cast<std::uint8_t>(i);
        const double pos = lift.get_position(index) * cfg::liftDirection;
        const double current = lift.get_current_draw(index);
        const double velocity = lift.get_actual_velocity(index);
        if (!std::isfinite(pos) || !std::isfinite(current) || current == PROS_ERR || current < 0
            || !std::isfinite(velocity) || std::abs(velocity) > 1000)
            invalid(i == 0 ? "Lift 1 non disponibile" : "Lift 2 non disponibile");
        if (i == 0) firstLift = pos;
        // Offsets/backlash under load are advisory, not a lost sensor. Actual
        // missing readings still pause both motors; homing uses both currents.
        if (std::isfinite(pos) && std::abs(pos - firstLift) > 0.25)
            in.warning = "Lift: encoder diversi";
        in.lift += pos;
        minCurrent = std::min(minCurrent, current);
        maxCurrent = std::max(maxCurrent, current);
        maxVelocity = std::max(maxVelocity, std::abs(velocity));
    }
    if (liftCount) in.lift /= liftCount;
    in.liftCurrent = minCurrent; in.liftPeakCurrent = maxCurrent; in.liftVelocity = maxVelocity;
    return in;
}
}
