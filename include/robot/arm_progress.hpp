#pragma once
#include "robot/mechanism_config.hpp"
#include <cmath>
#include <cstdint>

namespace robot::mechanism {
// Separates an actual jam from the short motion opposing a new braking command.
class ArmProgress {
public:
    enum class Result { ok, blocked, opposite };
    void reset() { tracking = false; direction = 0; }
    Result update(std::uint32_t now, double position, double error, int command,
                  std::uint32_t blockedMs = config::mechanism::armBlockedMs) {
        namespace cfg = config::mechanism;
        if (std::abs(error) <= cfg::armToleranceDeg) { reset(); return Result::ok; }
        if (!tracking || std::abs(position - anchor) >= cfg::armProgressDeg) {
            tracking = true; anchor = position; progressSince = now;
        } else if (now - progressSince >= blockedMs) return Result::blocked;

        // Ignore small settling corrections and restart the observation whenever
        // torque reverses. Target-error sign alone does not describe motor torque.
        const int requested = std::abs(command) >= cfg::armDirectionMinPower
            && std::abs(error) > cfg::armDirectionCheckErrorDeg ? (command > 0 ? 1 : -1) : 0;
        if (!requested || requested != direction) {
            direction = requested; directionSince = now; directionAnchor = position;
            return Result::ok;
        }
        if (now - directionSince < cfg::armDirectionGraceMs) {
            directionAnchor = position;
            return Result::ok;
        }
        const double travel = (position - directionAnchor) * direction;
        if (travel < -cfg::armWrongDirectionDeg) return Result::opposite;
        if (travel >= cfg::armProgressDeg) directionAnchor = position;
        return Result::ok;
    }
private:
    bool tracking = false;
    int direction = 0;
    double anchor = 0, directionAnchor = 0;
    std::uint32_t progressSince = 0, directionSince = 0;
};
}
