#pragma once

#include "nexus/types.hpp"

namespace nexus::calibration {

struct PodOffsetResult {
    unsigned checkpoints = 0;
    double forwardOffset = 0, lateralOffset = 0;
    double forwardDifference = 0, lateralDifference = 0;
    std::array<std::array<double, 2>, 2> individual{};
    std::array<double, 2> angles{};
    bool consistent = false;
};

// Fit offsets from two returns of one fixed chassis point to the same XY mark,
// at DIFFERENT headings. Integrate all motion, including manual recentering.
// No drive encoders, assumed stationary pivot, or current offset is used.
class PodOffsetFit {
public:
    void reset(double forwardScale, double lateralScale, double gyroScale) {
        *this = PodOffsetFit{};
        forwardScale_ = forwardScale; lateralScale_ = lateralScale; gyroScale_ = gyroScale;
    }
    bool add(const SensorSample& sample) {
        if (!sample.forwardValid || !sample.lateralValid || !sample.gyroValid ||
            !std::isfinite(sample.forward) || !std::isfinite(sample.lateral) ||
            !std::isfinite(sample.gyro) || !std::isfinite(sample.timestamp) ||
            !std::isfinite(forwardScale_) || forwardScale_ <= 0 ||
            !std::isfinite(lateralScale_) || lateralScale_ <= 0 ||
            !std::isfinite(gyroScale_) || gyroScale_ <= 0) return false;
        if (!initialized_) { first_ = previous_ = sample; initialized_ = true; return true; }
        if (sample.timestamp < previous_.timestamp) return false;
        if (sample.timestamp == previous_.timestamp) return true;
        const double angle = (sample.gyro - previous_.gyro) * gyroScale_;
        const double midpoint = heading() + angle / 2;
        const double half = angle / 2;
        const double chord = std::abs(half) < 1e-8 ? 1 - half * half / 6 : std::sin(half) / half;
        const double forward = (sample.forward - previous_.forward) * forwardScale_;
        const double lateral = (sample.lateral - previous_.lateral) * lateralScale_;
        rawX_ += chord * (lateral * std::cos(midpoint) + forward * std::sin(midpoint));
        rawY_ += chord * (forward * std::cos(midpoint) - lateral * std::sin(midpoint));
        previous_ = sample;
        return std::isfinite(rawX_) && std::isfinite(rawY_) && std::isfinite(heading());
    }
    bool checkpoint() {
        if (!initialized_ || result_.checkpoints == 2) return false;
        const double angle = heading(), a = 1 - std::cos(angle), s = std::sin(angle);
        const double weight = a * a + s * s;
        // A full-turn closure has zero offset sensitivity. Do not manufacture a
        // calibration from roundoff or tiny heading errors near 0 / 360 degrees.
        if (weight < square(2 * std::sin(radians(30) / 2))) return false;
        const double bx = -a * rawX_ - s * rawY_;
        const double by = s * rawX_ - a * rawY_;
        const unsigned point = result_.checkpoints++;
        result_.individual[point] = {bx / weight, by / weight};
        result_.angles[point] = angle;
        sumX_ += bx; sumY_ += by; sumWeight_ += weight;
        result_.forwardOffset = sumX_ / sumWeight_;
        result_.lateralOffset = sumY_ / sumWeight_;
        if (result_.checkpoints == 2) {
            result_.forwardDifference = std::abs(result_.individual[0][0] - result_.individual[1][0]);
            result_.lateralDifference = std::abs(result_.individual[0][1] - result_.individual[1][1]);
            // Advisory consistency, not a motion abort or certified accuracy.
            result_.consistent = result_.forwardDifference <= .005 && result_.lateralDifference <= .005 &&
                std::abs(result_.forwardOffset) <= .5 && std::abs(result_.lateralOffset) <= .5;
        }
        return true;
    }
    double heading() const { return (previous_.gyro - first_.gyro) * gyroScale_; }
    double forwardTravel() const { return (previous_.forward - first_.forward) * forwardScale_; }
    double lateralTravel() const { return (previous_.lateral - first_.lateral) * lateralScale_; }
    const PodOffsetResult& result() const { return result_; }
private:
    bool initialized_ = false;
    double forwardScale_ = 1, lateralScale_ = 1, gyroScale_ = 1;
    double rawX_ = 0, rawY_ = 0, sumX_ = 0, sumY_ = 0, sumWeight_ = 0;
    SensorSample first_{}, previous_{};
    PodOffsetResult result_{};
};

enum class PodOffsetStage { idle, turning, settling, align, ready, complete, aborted };
enum class PodOffsetFailure { none, released, sensors, clock, timeout, reference };
inline const char* podOffsetFailureName(PodOffsetFailure failure) {
    switch (failure) {
    case PodOffsetFailure::none: return "OK";
    case PodOffsetFailure::released: return "R1 rilasciato: prova annullata";
    case PodOffsetFailure::sensors: return "Pod/IMU assenti o dati vecchi";
    case PodOffsetFailure::clock: return "Tempo sensori non valido";
    case PodOffsetFailure::timeout: return "Rotazione oltre 20s: motori fermi";
    case PodOffsetFailure::reference: return "Heading troppo vicino alla partenza";
    }
    return "Errore";
}
struct PodOffsetOutput {
    Voltage voltage{};
    PodOffsetStage stage = PodOffsetStage::idle;
    PodOffsetFailure failure = PodOffsetFailure::none;
    unsigned leg = 0; // +90, -90, then home. Relative to initial IMU rotation.
    double heading = 0, target = 0, speed = 0;
};

class PodOffsetRoutine {
public:
    static constexpr double maxVoltage = 4.0;
    static constexpr double targetSpeed = radians(90);
    static constexpr double homeSpeed = radians(45);
    static constexpr double sampleMaxAge = .10;
    static constexpr double turnTimeout = 20;
    bool start(const SensorSample& sample, double now, const EstimatorConfig& config) {
        *this = PodOffsetRoutine{};
        fit_.reset(config.forwardScale, config.lateralScale, config.gyroScale);
        if (!config.nativeImuHeading || !valid(sample, now) || !fit_.add(sample)) {
            abort(PodOffsetFailure::sensors); return false;
        }
        lastNow_ = sampleTime_ = rateTime_ = now;
        sampleTime_ = sample.timestamp;
        beginTurn(now);
        return true;
    }
    PodOffsetOutput update(const SensorSample& sample, double now, bool permit, bool confirm = false) {
        if (output_.stage == PodOffsetStage::idle || output_.stage == PodOffsetStage::complete ||
            output_.stage == PodOffsetStage::aborted) return output_;
        const bool powered = output_.stage == PodOffsetStage::turning || output_.stage == PodOffsetStage::settling;
        if (powered && !permit) return abort(PodOffsetFailure::released);
        if (!std::isfinite(now) || now < lastNow_ || sample.timestamp < sampleTime_)
            return abort(PodOffsetFailure::clock);
        if (!valid(sample, now) || sample.timestamp - sampleTime_ > sampleMaxAge + 1e-9 || !fit_.add(sample))
            return abort(PodOffsetFailure::sensors);
        const double dt = std::clamp(now - lastNow_, 0.0, sampleMaxAge);
        lastNow_ = now; sampleTime_ = sample.timestamp;
        output_.heading = fit_.heading();
        // Rate across a window spanning multiple PROS cache refreshes. A held
        // 1 ms sample followed by a 10 ms jump is not a speed violation.
        if (now - rateTime_ >= .04 - 1e-9) {
            const double interval = now - rateTime_;
            output_.speed += -std::expm1(-interval / .06) *
                ((output_.heading - rateHeading_) / interval - output_.speed);
            rateTime_ = now; rateHeading_ = output_.heading;
        }
        if (powered && now - turnStarted_ > turnTimeout) return abort(PodOffsetFailure::timeout);
        if (output_.stage == PodOffsetStage::turning) {
            const double error = output_.target - output_.heading;
            const double direction = turnDirection_;
            // Coast near the target. The calibration uses the ACTUAL angle,
            // so the two reference turns need not settle at exactly +/-90.
            const double lead = std::max(radians(.5), std::max(0.0, direction * output_.speed) * coastTime_);
            if (direction * error <= lead) {
                output_.voltage = {}; voltage_ = 0;
                output_.stage = PodOffsetStage::settling; settleStarted_ = now;
                coastHeading_ = output_.heading;
                coastSpeed_ = output_.speed;
            } else {
                const double speedLimit = output_.leg == 2 ? homeSpeed : targetSpeed;
                const double desiredSpeed = std::min(speedLimit, std::sqrt(2 * radians(150) * std::abs(error)));
                const double speedError = desiredSpeed - direction * output_.speed;
                integral_ = std::clamp(integral_ + speedError * dt, -1.0, 1.0);
                const double requested = std::clamp(1.8 + .5 * desiredSpeed + .6 * speedError + .5 * integral_,
                                                    0.0, maxVoltage);
                voltage_ += std::clamp(requested - voltage_, -8 * dt, 8 * dt);
                output_.voltage = {direction * voltage_, -direction * voltage_};
            }
        } else if (output_.stage == PodOffsetStage::settling) {
            output_.voltage = {};
            if (now - settleStarted_ >= .7) {
                // Use the observed coast after each stop to anticipate the next
                // one; this is only motor control, never part of the offset fit.
                if (std::abs(coastSpeed_) > radians(10))
                    coastTime_ = std::clamp((output_.heading - coastHeading_) / coastSpeed_, .03, .35);
                if (output_.leg < 2) output_.stage = PodOffsetStage::align;
                else if (std::abs(output_.heading) <= radians(2) || homeCorrections_ >= 4)
                    output_.stage = PodOffsetStage::complete;
                else {
                    ++homeCorrections_;
                    // Retain the overall home deadline across small corrections.
                    const double originalStart = turnStarted_;
                    beginTurn(now);
                    turnStarted_ = originalStart;
                }
            }
        } else if (output_.stage == PodOffsetStage::align) {
            output_.voltage = {};
            if (confirm && !permit) {
                if (!fit_.checkpoint()) return abort(PodOffsetFailure::reference);
                ++output_.leg;
                output_.target = targets_[output_.leg];
                output_.stage = PodOffsetStage::ready;
            }
        } else if (output_.stage == PodOffsetStage::ready) {
            output_.voltage = {};
            if (permit) beginTurn(now);
        }
        return output_;
    }
    const PodOffsetOutput& output() const { return output_; }
    const PodOffsetFit& fit() const { return fit_; }
private:
    static constexpr std::array<double, 3> targets_{pi / 2, -pi / 2, 0};
    PodOffsetFit fit_;
    PodOffsetOutput output_;
    double lastNow_ = 0, sampleTime_ = 0, rateTime_ = 0, rateHeading_ = 0;
    double turnStarted_ = 0, settleStarted_ = 0, turnDirection_ = 1, voltage_ = 0, integral_ = 0;
    double coastTime_ = .12, coastHeading_ = 0, coastSpeed_ = 0;
    unsigned homeCorrections_ = 0;
    static bool valid(const SensorSample& sample, double now) {
        return std::isfinite(now) && std::isfinite(sample.timestamp) &&
            sample.timestamp <= now + .005 && now - sample.timestamp <= sampleMaxAge &&
            sample.forwardValid && sample.lateralValid && sample.gyroValid &&
            std::isfinite(sample.forward) && std::isfinite(sample.lateral) && std::isfinite(sample.gyro);
    }
    void beginTurn(double now) {
        output_.target = targets_[output_.leg];
        output_.stage = PodOffsetStage::turning; output_.voltage = {};
        turnDirection_ = output_.target >= output_.heading ? 1 : -1;
        turnStarted_ = now; voltage_ = integral_ = 0;
    }
    PodOffsetOutput abort(PodOffsetFailure reason) {
        output_.voltage = {}; output_.stage = PodOffsetStage::aborted; output_.failure = reason;
        return output_;
    }
};
} // namespace nexus::calibration
