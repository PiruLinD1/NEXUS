#pragma once

#include "nexus/types.hpp"
#include <array>
#include <limits>

namespace nexus::calibration {
// These values are part of the characterization CSV format.
enum class CharacterizationStage : unsigned {
    idle = 0, settling = 1, powered = 2, coast = 3, complete = 4, aborted = 5
};
enum class CharacterizationFailure : unsigned {
    none = 0, notPermitted = 1, invalidClock = 2, invalidConfiguration = 3,
    invalidSensors = 4, staleSensors = 5, schedulerGap = 6, sensorJump = 7,
    initialMotion = 8, translationLimit = 9, rotationLimit = 10,
    rotationTooFast = 11, stalled = 12, timeout = 13, coastTimeout = 14,
    wrongDirection = 15, invalidTrial = 16, loggingUnavailable = 17
};
struct CharacterizationTrial {
    const char* name;
    Voltage voltage;
    double poweredSeconds;
    double straightLeadSeconds = 0;
};
inline constexpr unsigned characterizationTrialCount = 18;
inline constexpr std::array<CharacterizationTrial, characterizationTrialCount> characterizationTrials{{
    {"Avanti 3 V", {3, 3}, .40}, {"Indietro 3 V", {-3, -3}, .40},
    {"Giro CW 3 V", {3, -3}, .40}, {"Giro CCW 3 V", {-3, 3}, .40},
    {"Curva sinistra 3 V", {.75, 3}, .40, .20}, {"Curva destra 3 V", {3, .75}, .40, .20},
    {"Avanti 4.5 V", {4.5, 4.5}, .45}, {"Indietro 4.5 V", {-4.5, -4.5}, .45},
    {"Giro CW 4.5 V", {4.5, -4.5}, .45}, {"Giro CCW 4.5 V", {-4.5, 4.5}, .45},
    {"Curva sinistra 4.5 V", {1.125, 4.5}, .45, .20}, {"Curva destra 4.5 V", {4.5, 1.125}, .45, .20},
    {"Avanti 6 V", {6, 6}, .50}, {"Indietro 6 V", {-6, -6}, .50},
    {"Giro CW 6 V", {6, -6}, .50}, {"Giro CCW 6 V", {-6, 6}, .50},
    {"Curva sinistra 6 V", {1.5, 6}, .50, .20}, {"Curva destra 6 V", {6, 1.5}, .50, .20}
}};
inline const CharacterizationTrial* characterizationTrial(unsigned id) {
    return id < characterizationTrialCount ? &characterizationTrials[id] : nullptr;
}
inline const char* characterizationStageName(CharacterizationStage stage) {
    switch (stage) {
    case CharacterizationStage::idle: return "idle";
    case CharacterizationStage::settling: return "settling";
    case CharacterizationStage::powered: return "powered";
    case CharacterizationStage::coast: return "coast";
    case CharacterizationStage::complete: return "complete";
    case CharacterizationStage::aborted: return "aborted";
    }
    return "unknown";
}
inline const char* characterizationFailureName(CharacterizationFailure failure) {
    switch (failure) {
    case CharacterizationFailure::none: return "OK";
    case CharacterizationFailure::notPermitted: return "released/disabled";
    case CharacterizationFailure::invalidClock: return "invalid clock";
    case CharacterizationFailure::invalidConfiguration: return "invalid pod/IMU calibration";
    case CharacterizationFailure::invalidSensors: return "pod/IMU unavailable";
    case CharacterizationFailure::staleSensors: return "stale sensors";
    case CharacterizationFailure::schedulerGap: return "scheduler gap";
    case CharacterizationFailure::sensorJump: return "pod jump";
    case CharacterizationFailure::initialMotion: return "robot moving at start";
    case CharacterizationFailure::translationLimit: return "travel over 1.2 m";
    case CharacterizationFailure::rotationLimit: return "total rotation over 360 deg";
    case CharacterizationFailure::rotationTooFast: return "rotation over 720 deg/s";
    case CharacterizationFailure::stalled: return "no trial motion";
    case CharacterizationFailure::timeout: return "trial timeout";
    case CharacterizationFailure::coastTimeout: return "coast did not settle";
    case CharacterizationFailure::wrongDirection: return "wrong motion direction";
    case CharacterizationFailure::invalidTrial: return "invalid trial";
    case CharacterizationFailure::loggingUnavailable: return "characterization log unavailable";
    }
    return "unknown";
}
struct CharacterizationOutput {
    Voltage voltage{};
    CharacterizationStage stage = CharacterizationStage::idle;
    CharacterizationFailure failure = CharacterizationFailure::none;
    double elapsed = 0, stageElapsed = 0;
    double distance = 0, angle = 0, speed = 0, omega = 0;
    unsigned trialId = 0;
};

// This runner issues one bounded voltage pulse. The hardware adapter must
// apply genuine COAST at zero voltage and independently revoke motor ownership
// on disable. Sensor-derived limits cannot guarantee a physical field boundary.
class CharacterizationRoutine {
public:
    static constexpr double maximumSampleAge = .10;
    static constexpr double maximumDuration = 8.0;
    static constexpr double maximumDistance = 1.2;
    static constexpr double maximumAngle = 2 * pi;
    static constexpr double maximumOmega = 4 * pi;
    static constexpr double maximumCoastDuration = 3.0;
    bool start(const SensorSample& sample, double now, unsigned trialId, const EstimatorConfig& config) {
        if (active()) return false;
        *this = CharacterizationRoutine{};
        output_.trialId = trialId;
        config_ = config;
        started_ = lastNow_ = stageStarted_ = quietSince_ = now;
        output_.stage = CharacterizationStage::settling;
        if (!characterizationTrial(trialId)) { abort(CharacterizationFailure::invalidTrial); return false; }
        if (!validConfig()) { abort(CharacterizationFailure::invalidConfiguration); return false; }
        const auto failure = validate(sample, now);
        if (failure != CharacterizationFailure::none) { abort(failure); return false; }
        if (sample.gyroRateValid && std::abs(sample.gyroRate * config_.gyroScale) > quietOmega) {
            abort(CharacterizationFailure::initialMotion); return false;
        }
        previous_ = sample;
        quiet_ = true;
        return true;
    }
    bool active() const {
        return output_.stage == CharacterizationStage::settling
            || output_.stage == CharacterizationStage::powered || output_.stage == CharacterizationStage::coast;
    }
    CharacterizationOutput output() const { return output_; }
    CharacterizationOutput abort(CharacterizationFailure failure = CharacterizationFailure::notPermitted) {
        if ((output_.stage != CharacterizationStage::complete && output_.stage != CharacterizationStage::aborted)
            || (output_.stage == CharacterizationStage::complete && failure == CharacterizationFailure::loggingUnavailable)) {
            output_.stage = CharacterizationStage::aborted;
            output_.failure = failure;
        }
        output_.voltage = {};
        return output_;
    }
    CharacterizationOutput update(const SensorSample& sample, double now, bool permit) {
        if (!active()) return output_;
        if (!permit) return abort(CharacterizationFailure::notPermitted);
        if (!std::isfinite(now) || now < lastNow_ - 1e-9 || sample.timestamp < previous_.timestamp - 1e-9)
            return abort(CharacterizationFailure::invalidClock);
        if (now - lastNow_ > maximumSampleAge + 1e-9) return abort(CharacterizationFailure::schedulerGap);
        const auto failure = validate(sample, now);
        if (failure != CharacterizationFailure::none) return abort(failure);
        lastNow_ = now;
        output_.elapsed = now - started_;
        output_.stageElapsed = now - stageStarted_;
        if (output_.elapsed >= maximumDuration) return abort(CharacterizationFailure::timeout);
        const double dt = sample.timestamp - previous_.timestamp;
        bool fresh = dt > 1e-9;
        if (fresh) {
            if (dt > maximumSampleAge + 1e-9) return abort(CharacterizationFailure::staleSensors);
            const double yaw = (sample.gyro - previous_.gyro) * config_.gyroScale;
            output_.omega = yaw / dt;
            if (std::abs(output_.omega) > maximumOmega + 1e-9
                || (sample.gyroRateValid && std::abs(sample.gyroRate * config_.gyroScale) > maximumOmega))
                return abort(CharacterizationFailure::rotationTooFast);
            // Translation of the robot center, with both known pod lever arms.
            const double forward = (sample.forward - previous_.forward) * config_.forwardScale + config_.forwardOffset * yaw;
            const double lateral = (sample.lateral - previous_.lateral) * config_.lateralScale - config_.lateralOffset * yaw;
            const double distance = std::hypot(forward, lateral);
            if (distance > 4.0 * dt + .002) return abort(CharacterizationFailure::sensorJump);
            output_.distance += distance;
            output_.angle += yaw;
            absoluteAngle_ += std::abs(yaw);
            output_.speed = distance / dt;
            if (output_.distance > maximumDistance) return abort(CharacterizationFailure::translationLimit);
            if (absoluteAngle_ > maximumAngle) return abort(CharacterizationFailure::rotationLimit);
            const bool still = output_.speed <= quietSpeed && std::abs(output_.omega) <= quietOmega
                && (!sample.gyroRateValid || std::abs(sample.gyroRate * config_.gyroScale) <= quietOmega);
            if (output_.stage == CharacterizationStage::settling && !still)
                return abort(CharacterizationFailure::initialMotion);
            if (!still) quiet_ = false;
            else if (!quiet_) { quiet_ = true; quietSince_ = now; }
            if (output_.stage == CharacterizationStage::powered) {
                const unsigned kind = output_.trialId % 6;
                forwardProgress_ += forward * (kind == 1 ? -1 : 1);
                const int turnSign = kind == 2 || kind == 5 ? 1 : -1;
                const bool turning = kind >= 2 && output_.voltage.left != output_.voltage.right;
                if (turning) turnProgress_ += yaw * turnSign;
                if ((kind != 2 && kind != 3 && forwardProgress_ < -.03)
                    || (turning && turnProgress_ < -radians(5)))
                    return abort(CharacterizationFailure::wrongDirection);
            }
            previous_ = sample;
        }
        if (output_.stage == CharacterizationStage::settling && fresh && quiet_ && now - quietSince_ >= .5 - 1e-9)
            enter(CharacterizationStage::powered, now);
        else if (output_.stage == CharacterizationStage::powered
                 && now - stageStarted_ >= characterizationTrial(output_.trialId)->poweredSeconds - 1e-9) {
            const unsigned kind = output_.trialId % 6;
            if ((kind == 2 || kind == 3) ? turnProgress_ < radians(3) : forwardProgress_ < .01)
                return abort(CharacterizationFailure::stalled);
            enter(CharacterizationStage::coast, now);
        } else if (output_.stage == CharacterizationStage::coast) {
            if (fresh && quiet_ && now - quietSince_ >= .3 - 1e-9)
                enter(CharacterizationStage::complete, now);
            else if (now - stageStarted_ >= maximumCoastDuration)
                return abort(CharacterizationFailure::coastTimeout);
        }
        output_.voltage = {};
        if (output_.stage == CharacterizationStage::powered) {
            const auto& trial = *characterizationTrial(output_.trialId);
            output_.voltage = trial.voltage;
            // Enter a sharp curve while already moving: first accelerate both
            // sides equally, then step the inside side down without a stop.
            if (now - stageStarted_ < trial.straightLeadSeconds - 1e-9) {
                const double outer = std::max(trial.voltage.left, trial.voltage.right);
                output_.voltage = {outer, outer};
            }
        }
        return output_;
    }
private:
    static constexpr double quietSpeed = .03, quietOmega = radians(2);
    CharacterizationOutput output_{};
    EstimatorConfig config_{};
    SensorSample previous_{};
    double started_ = 0, lastNow_ = 0, stageStarted_ = 0, quietSince_ = 0;
    double absoluteAngle_ = 0, forwardProgress_ = 0, turnProgress_ = 0;
    bool quiet_ = false;
    void enter(CharacterizationStage stage, double now) {
        output_.stage = stage; output_.stageElapsed = 0; stageStarted_ = now;
        quiet_ = false;
    }
    bool validConfig() const {
        return std::isfinite(config_.forwardScale) && config_.forwardScale > 0 && config_.forwardScale <= 10
            && std::isfinite(config_.lateralScale) && config_.lateralScale > 0 && config_.lateralScale <= 10
            && std::isfinite(config_.gyroScale) && config_.gyroScale > 0 && config_.gyroScale <= 10
            && std::isfinite(config_.forwardOffset) && std::abs(config_.forwardOffset) <= 1
            && std::isfinite(config_.lateralOffset) && std::abs(config_.lateralOffset) <= 1;
    }
    CharacterizationFailure validate(const SensorSample& sample, double now) const {
        if (!std::isfinite(now) || now < 0 || !std::isfinite(sample.timestamp) || sample.timestamp < 0
            || sample.timestamp > now + .002) return CharacterizationFailure::invalidClock;
        if (now - sample.timestamp > maximumSampleAge + 1e-9) return CharacterizationFailure::staleSensors;
        if (!sample.forwardValid || !sample.lateralValid || !sample.gyroValid
            || !std::isfinite(sample.forward) || !std::isfinite(sample.lateral) || !std::isfinite(sample.gyro)
            || (sample.gyroRateValid && !std::isfinite(sample.gyroRate)))
            return CharacterizationFailure::invalidSensors;
        return CharacterizationFailure::none;
    }
};
} // namespace nexus::calibration
