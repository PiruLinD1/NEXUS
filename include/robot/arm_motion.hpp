#pragma once
#include "robot/mechanism_config.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace robot::mechanism {
// Encoder-based fast travel with a short distance-shaped deceleration, followed
// by passive braking and then motor HOLD at low speed. No P/I/D terms.
class ArmMotion {
public:
    struct Command {
        int power = 0;
        bool hold = true, arrived = false, brake = false, exhausted = false;
    };
    void reset() { *this = ArmMotion{}; }
    Command update(std::uint32_t now, double position, double target,
                   double tolerance = config::mechanism::armToleranceDeg) {
        namespace cfg = config::mechanism;
        if (!std::isfinite(position) || !std::isfinite(target)) { reset(); return {}; }
        if (!initialized || target != destination) {
            reset(); initialized = true; destination = target;
            previousPosition = position; lastUpdate = now;
            direction = target >= position ? 1 : -1;
        }
        const double dt = std::clamp((now - lastUpdate) * 0.001, 0.001, 0.05);
        const double measuredSpeed = (position - previousPosition) / dt;
        speed += dt / (cfg::armMotionSpeedFilterSeconds + dt) * (measuredSpeed - speed);
        previousPosition = position; lastUpdate = now;
        const double error = target - position;
        const bool close = std::abs(error) <= tolerance;
        const bool slow = std::abs(speed) <= cfg::armArrivalSpeedDegPerSecond;
        if (arrived) return {0, true, true};
        if (exhausted) return {0, true, false, false, true};
        if (braking) {
            // Do not ask the motor's position controller to catch a fast arm:
            // dissipate momentum first, then latch HOLD without switching it
            // on/off in response to every noisy speed sample.
            if (!holding) {
                if (!slow && now - brakeSince < cfg::armBrakeMaxMs) return {0, false, false, true};
                holding = true; holdSince = now;
            }
            if (close && slow) {
                if (!settling) { settling = true; settleSince = now; }
                arrived = now - settleSince >= cfg::armArrivalMs;
            } else settling = false;
            // Let HOLD settle before any correction. Only two attempts are
            // allowed, with a lower starting effort on the last: never hunt indefinitely across
            // the target or claim arrival when still outside its tolerance.
            if (!close && slow && now - holdSince >= cfg::armBrakePauseMs) {
                if (corrections == cfg::armMaxCorrections) {
                    exhausted = true;
                    return {0, true, false, false, true};
                }
                // Use the observed stopping travel for the next slow approach.
                // A fixed early cutoff otherwise repeats the same shortfall on
                // a light arm, while too little lead overshoots a loaded arm.
                if (std::abs(brakeSpeed) > cfg::armArrivalSpeedDegPerSecond)
                    correctionBrakeSeconds = std::clamp(std::abs(position - brakePosition)
                        / std::abs(brakeSpeed), 0.005, 0.15);
                ++corrections; braking = holding = struggling = false;
                direction = error >= 0 ? 1 : -1;
            }
            return {0, true, arrived};
        }
        const double remaining = error * direction;
        const double brakeLead = std::max(0.0, speed * direction)
            * (corrections ? correctionBrakeSeconds : cfg::armBrakeLookaheadSeconds);
        if ((close && slow) || remaining <= cfg::armBrakeMarginDeg + brakeLead) {
            braking = true; brakeSince = now; command = 0; settling = struggling = false;
            brakePosition = position; brakeSpeed = speed;
            holding = slow; holdSince = now;
            return {0, holding, false, !holding};
        }
        // sqrt(distance) gives a compact deceleration segment. Minimum effort
        // keeps the last degrees moving under load, even with static friction.
        const double fraction = std::clamp((remaining - tolerance) / cfg::armSlowdownDeg, 0.0, 1.0);
        double desired = corrections ? (corrections == cfg::armMaxCorrections
                ? cfg::armFinalCorrectionPower : cfg::armApproachPower)
            : std::max(double(cfg::armApproachPower), cfg::armMoveMaxPower * std::sqrt(fraction));
        // Fixed low correction power can be less than gravity/static friction.
        // If actual forward speed stays low after the command ramp, increase
        // effort at a bounded rate. Remember it for this direction/request so
        // a subsequent correction does not drop below the same load again.
        // No accumulated position error; predictive braking still takes priority.
        double& loadFloor = loadPower[direction > 0 ? 0 : 1];
        desired = std::max(desired, loadFloor);
        if (!close && speed * direction < cfg::armLoadAssistSpeedDegPerSecond
                && command >= desired - 1 && command < cfg::armMoveMaxPower) {
            if (!struggling) { struggling = true; struggleSince = now; }
            if (now - struggleSince >= cfg::armLoadAssistDelayMs) {
                loadFloor = std::min(double(cfg::armMoveMaxPower),
                    std::max(loadFloor, command) + cfg::armLoadAssistPowerPerSecond * dt);
                desired = std::max(desired, loadFloor);
            }
        } else struggling = false;
        const double rate = desired < command ? cfg::armDecelerationPowerPerSecond : cfg::armAccelerationPowerPerSecond;
        command += std::clamp(desired - command, -rate * dt, rate * dt);
        return {direction * static_cast<int>(command), false, false};
    }
private:
    bool initialized = false, braking = false, holding = false, settling = false, arrived = false, exhausted = false;
    bool struggling = false;
    unsigned corrections = 0;
    int direction = 1;
    double previousPosition = 0, destination = 0, speed = 0, command = 0;
    double brakePosition = 0, brakeSpeed = 0;
    double loadPower[2]{};
    double correctionBrakeSeconds = config::mechanism::armBrakeLookaheadSeconds;
    std::uint32_t lastUpdate = 0, brakeSince = 0, holdSince = 0, settleSince = 0;
    std::uint32_t struggleSince = 0;
};
}
