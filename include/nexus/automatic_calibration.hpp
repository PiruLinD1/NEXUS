#pragma once

#include "nexus/calibration.hpp"
#include <array>

namespace nexus { namespace calibration {

// All quantities in this portable layer are SI. These collectors never command
// a motor and never allocate. Call from one task, with fresh sensor snapshots.
struct StationaryResult {
    FitQuality quality{};
    double gyroBias = 0;         // Calibrated rad/s, valid for this power cycle.
    double gyroBiasVariance = 0; // (rad/s)^2, includes a correlation/noise floor.
    double gyroIncrementStd = 0; // Observed 10 ms increment noise, rad.
    double duration = 0;
};
class StationaryCalibration {
public:
    static constexpr std::size_t capacity = 640;
    void reset(double gyroScale = 1.0);
    bool add(const SensorSample& sample);
    StationaryResult finish() const;
private:
    std::array<double, capacity> times_{}, angles_{};
    SensorSample first_{}, previous_{};
    std::size_t count_ = 0;
    double gyroScale_ = 1;
    FitFailure failure_ = FitFailure::none;
};

struct RotationCalibrationConfig {
    double forwardScale = 1.0, lateralScale = 1.0, gyroScale = 1.0;
    double gyroBias = 0; // Calibrated rad/s, e.g. stationary result above.
    bool useForwardPod = true, useLateralPod = true;
};
struct AutomaticGeometryResult {
    bool accepted = false;
    FitFailure failure = FitFailure::insufficientData;
    ScalarFit forwardOffset{}, lateralOffset{}, trackWidth{};
    double clockwiseAngle = 0, counterclockwiseAngle = 0; // Both nonnegative.
    std::size_t samples = 0, outliers = 0, rejectedFrames = 0;
    // Merge only the accepted geometry bits; preserve independent scale/dynamics
    // calibrations. This does NOT calibrate absolute wheel or gyro scale.
    bool applyTo(CalibrationProfile& profile) const;
};
class RotationCalibration {
public:
    static constexpr std::size_t capacity = 128;
    void reset(RotationCalibrationConfig config = {});
    bool add(const SensorSample& sample);
    AutomaticGeometryResult finish() const;
private:
    struct Window { double angle = 0, forward = 0, lateral = 0, drive = 0; };
    std::array<Window, capacity> windows_{};
    RotationCalibrationConfig config_{};
    SensorSample previous_{};
    Window pending_{};
    double pendingCenter_ = 0;
    std::size_t count_ = 0, rejectedFrames_ = 0;
    bool initialized_ = false;
    FitFailure failure_ = FitFailure::none;
};

}} // namespace nexus::calibration
