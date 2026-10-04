#pragma once

#include "nexus/types.hpp"

namespace nexus {

enum class CalibrationStage {
    idle, settlingCW, clockwise, settlingCCW, counterclockwise, finalSettle, complete, aborted
};

enum class CalibrationFailure {
    none, notPermitted, invalidClock, invalidConfiguration, invalidGyro, invalidDrive,
    staleSensors, encoderJump, rotationTooFast, wrongDirection, translation, stalled, timeout
};

struct CalibrationRoutineOutput {
    Voltage voltage{};
    CalibrationStage stage = CalibrationStage::idle;
    CalibrationFailure failure = CalibrationFailure::none;
    double clockwiseRadians = 0;
    double counterclockwiseRadians = 0;
};

inline const char* calibrationFailureName(CalibrationFailure failure) {
    switch (failure) {
    case CalibrationFailure::none: return "OK";
    case CalibrationFailure::notPermitted: return "released/disabled";
    case CalibrationFailure::invalidClock: return "invalid clock";
    case CalibrationFailure::invalidConfiguration: return "invalid gyro calibration";
    case CalibrationFailure::invalidGyro: return "IMU unavailable";
    case CalibrationFailure::invalidDrive: return "drive encoder unavailable";
    case CalibrationFailure::staleSensors: return "stale sensors";
    case CalibrationFailure::encoderJump: return "drive encoder jump";
    case CalibrationFailure::rotationTooFast: return "rotation too fast";
    case CalibrationFailure::wrongDirection: return "wrong rotation direction";
    case CalibrationFailure::translation: return "translation over 200 mm";
    case CalibrationFailure::stalled: return "no rotation";
    case CalibrationFailure::timeout: return "calibration timeout";
    }
    return "unknown";
}

inline const char* calibrationStageName(CalibrationStage stage) {
    switch (stage) {
    case CalibrationStage::idle: return "idle";
    case CalibrationStage::settlingCW: return "settling before CW";
    case CalibrationStage::clockwise: return "turn clockwise";
    case CalibrationStage::settlingCCW: return "settling before CCW";
    case CalibrationStage::counterclockwise: return "turn counterclockwise";
    case CalibrationStage::finalSettle: return "final settling";
    case CalibrationStage::complete: return "complete";
    case CalibrationStage::aborted: return "aborted";
    }
    return "unknown";
}

// Pure, allocation-free supervised motion generator. The owner must call
// update frequently, apply every returned voltage (including zero), and enforce
// its own output watchdog. `permit` includes held deadman, controller presence,
// and the permitted competition mode. No call to update can restart a run.
// Sensor distances are metres; timestamps are monotonic seconds on one clock.
class GeometryRoutine {
public:
    static constexpr double turnVoltage = 6.0;
    static constexpr double targetAngularSpeed = radians(120);
    static constexpr double maximumAngularSpeed = radians(360);
    static constexpr double rampTime = 0.30;
    static constexpr double settleTime = 0.70;
    static constexpr double turnAngle = 2 * pi;
    static constexpr double maximumDuration = 45.0;
    static constexpr double maximumSampleAge = 0.10;

    // Explicit starts/restarts are permitted only after the UI has required
    // release and a fresh hold. Starting alone never commands a motor.
    bool start(const SensorSample& sample, double now, double gyroScale = 1, double gyroBias = 0) {
        output_ = {};
        output_.stage = CalibrationStage::settlingCW;
        previous_ = sample;
        startTime_ = stageTime_ = previousNow_ = now;
        stillSince_ = now;
        centerTravel_ = 0;
        maximumProgress_ = progressCheckpoint_ = 0;
        progressTime_ = now;
        regulatedVoltage_ = filteredAngularSpeed_ = 0;
        gyroScale_ = gyroScale;
        gyroBias_ = gyroBias;
        if (!std::isfinite(gyroScale) || gyroScale <= 0 || !std::isfinite(gyroBias)
            || std::abs(gyroBias) > radians(3)) {
            abort(CalibrationFailure::invalidConfiguration);
            return false;
        }
        const auto failure = validate(sample, now);
        if (failure != CalibrationFailure::none) {
            abort(failure);
            return false;
        }
        return true;
    }

    CalibrationRoutineOutput update(const SensorSample& sample, double now, bool permit) {
        if (output_.stage == CalibrationStage::idle || output_.stage == CalibrationStage::complete
            || output_.stage == CalibrationStage::aborted) return output_;
        if (!permit) return abort(CalibrationFailure::notPermitted);
        if (!std::isfinite(now) || now < previousNow_ - 1e-9)
            return abort(CalibrationFailure::invalidClock);
        previousNow_ = now;
        const auto failure = validate(sample, now);
        if (failure != CalibrationFailure::none) return abort(failure);
        if (now - startTime_ >= maximumDuration) return abort(CalibrationFailure::timeout);
        if (sample.timestamp < previous_.timestamp - 1e-9)
            return abort(CalibrationFailure::invalidClock);
        // Duplicate snapshots do not advance ramping, settling, or progress.
        // Their last acquisition timestamp still enforces the freshness limit.
        if (sample.timestamp <= previous_.timestamp) return output_;

        const double dt = sample.timestamp - previous_.timestamp;
        if (dt > maximumSampleAge + 1e-9) return abort(CalibrationFailure::staleSensors);
        const double angle = (sample.gyro - previous_.gyro) * gyroScale_ - gyroBias_ * dt;
        const double left = sample.left - previous_.left;
        const double right = sample.right - previous_.right;
        previous_ = sample;
        if (!std::isfinite(angle) || std::abs(angle) > maximumAngularSpeed * dt + 1e-9)
            return abort(CalibrationFailure::rotationTooFast);
        if (!std::isfinite(left) || !std::isfinite(right)
            || std::max<double>(std::abs(left), std::abs(right)) > 3.0 * dt + 0.01)
            return abort(CalibrationFailure::encoderJump);
        centerTravel_ += std::abs(0.5 * (left + right));
        if (centerTravel_ > 0.20) return abort(CalibrationFailure::translation);

        const bool turningCW = output_.stage == CalibrationStage::clockwise;
        const bool turningCCW = output_.stage == CalibrationStage::counterclockwise;
        if (turningCW || output_.stage == CalibrationStage::settlingCCW)
            output_.clockwiseRadians += angle;
        if (turningCCW || output_.stage == CalibrationStage::finalSettle)
            output_.counterclockwiseRadians -= angle;
        if (turningCW || turningCCW) {
            // Keep startup torque available, then reduce voltage as the measured
            // speed approaches the target. Fixed 6 V can spin a free robot too fast.
            const double direction = turningCW ? 1.0 : -1.0;
            filteredAngularSpeed_ += dt / (0.04 + dt)
                * (direction * angle / dt - filteredAngularSpeed_);
            const double progress = turningCW ? output_.clockwiseRadians : output_.counterclockwiseRadians;
            maximumProgress_ = std::max(maximumProgress_, progress);
            if (maximumProgress_ - progress > radians(8)) return abort(CalibrationFailure::wrongDirection);
            if (progress - progressCheckpoint_ >= radians(8)) {
                progressCheckpoint_ = progress;
                progressTime_ = now;
            }
            if (now - progressTime_ > 4.0) return abort(CalibrationFailure::stalled);
            if (progress >= turnAngle) {
                enter(turningCW ? CalibrationStage::settlingCCW : CalibrationStage::finalSettle, now);
                return output_;
            }
            const double correction = 8.0 * (targetAngularSpeed - filteredAngularSpeed_) * dt;
            regulatedVoltage_ = std::clamp(regulatedVoltage_ +
                std::clamp(correction, -turnVoltage * dt / rampTime, turnVoltage * dt / rampTime),
                0.0, turnVoltage);
            const double voltage = regulatedVoltage_;
            output_.voltage = turningCW ? Voltage{voltage, -voltage} : Voltage{-voltage, voltage};
            return output_;
        }

        output_.voltage = {};
        const bool settled = std::abs(angle) <= radians(5) * dt
            && std::max<double>(std::abs(left), std::abs(right)) <= 0.04 * dt;
        if (!settled) stillSince_ = -1;
        else if (stillSince_ < 0) stillSince_ = now;
        if (stillSince_ >= 0 && now - stillSince_ >= settleTime) {
            if (output_.stage == CalibrationStage::settlingCW) enter(CalibrationStage::clockwise, now);
            else if (output_.stage == CalibrationStage::settlingCCW) enter(CalibrationStage::counterclockwise, now);
            else enter(CalibrationStage::complete, now);
        }
        return output_;
    }

    CalibrationRoutineOutput output() const { return output_; }

private:
    SensorSample previous_{};
    CalibrationRoutineOutput output_{};
    double startTime_ = 0, stageTime_ = 0, previousNow_ = 0, stillSince_ = -1;
    double centerTravel_ = 0, gyroScale_ = 1, gyroBias_ = 0;
    double maximumProgress_ = 0, progressCheckpoint_ = 0, progressTime_ = 0;
    double regulatedVoltage_ = 0, filteredAngularSpeed_ = 0;

    CalibrationFailure validate(const SensorSample& sample, double now) const {
        if (!std::isfinite(now) || !std::isfinite(sample.timestamp) || sample.timestamp > now + 0.005)
            return CalibrationFailure::invalidClock;
        if (now - sample.timestamp > maximumSampleAge + 1e-9)
            return CalibrationFailure::staleSensors;
        if (!sample.gyroValid || !std::isfinite(sample.gyro)) return CalibrationFailure::invalidGyro;
        if (!sample.leftValid || !sample.rightValid || !std::isfinite(sample.left) || !std::isfinite(sample.right))
            return CalibrationFailure::invalidDrive;
        return CalibrationFailure::none;
    }

    void enter(CalibrationStage stage, double now) {
        output_.stage = stage;
        output_.voltage = {};
        stageTime_ = now;
        stillSince_ = -1;
        if (stage == CalibrationStage::clockwise || stage == CalibrationStage::counterclockwise) {
            regulatedVoltage_ = filteredAngularSpeed_ = 0;
            maximumProgress_ = progressCheckpoint_ = 0;
            progressTime_ = now;
        }
    }

    CalibrationRoutineOutput abort(CalibrationFailure failure) {
        output_.voltage = {};
        output_.stage = CalibrationStage::aborted;
        output_.failure = failure;
        return output_;
    }
};

} // namespace nexus
