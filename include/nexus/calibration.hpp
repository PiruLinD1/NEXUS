#pragma once

#include "nexus/types.hpp"
#include <string>
#include <vector>

namespace nexus { namespace calibration {

// Offline tools: none of these routines drives a motor or runs on the RT loop.
enum class FitFailure { none, insufficientData, insufficientExcitation,
                        illConditioned, implausible, excessiveResidual,
                        invalidData, moving, inconsistentDirections, capacityExceeded };
const char* fitFailureName(FitFailure failure);
struct FitQuality {
    bool accepted = false;
    FitFailure failure = FitFailure::insufficientData;
    std::size_t samples = 0, outliers = 0;
    double rms = 0, inlierRms = 0, maxResidual = 0;
    double condition = 0; // Condition number of the normalized information matrix.
};
struct WheelSample {
    double voltage = 0;      // Measured applied terminal voltage (V), signed.
    double velocity = 0;     // Wheel ground velocity (m/s), signed.
    double acceleration = 0; // Synchronized derivative (m/s^2), signed.
    bool valid = true;
};
struct FitLimits {
    std::size_t minSamples = 30;
    double maxVoltage = 13.0, minSpeed = 0.04;
    double maxCondition = 10000, maxInlierRms = 0.8;
    double maxOutlierFraction = 0.25;
};
struct WheelFit {
    double kS = 0, kV = 0, kA = 0;
    FitQuality quality{};
};
WheelFit fitWheel(const std::vector<WheelSample>& samples,
                  const FitLimits& limits = FitLimits{});

struct ReferenceSample {
    double measured = 0;  // Raw travel or distance (m); travel may be signed.
    double reference = 0; // Independently measured physical travel/distance (m).
};
struct ScalarFit { double value = 0; FitQuality quality{}; };
ScalarFit fitTravelScale(const std::vector<ReferenceSample>& samples);

struct DistanceFit {
    // corrected distance = scale * raw distance + offset; lengths are metres.
    double scale = 1.0, offset = 0;
    double measuredMin = 0, measuredMax = 0; // Accepted raw-data domain, not extrapolation limits proven safe.
    FitQuality quality{};
};
// V5 distance affine calibration: raw/reference in 20..2000 mm, at least 12
// inliers spanning 500 mm, with >=3 in each third of the measured interval.
// Residuals describe this dataset only; they are not a runtime sensor sigma.
DistanceFit fitDistance(const std::vector<ReferenceSample>& samples);

struct RotationSample {
    double referenceAngle = 0; // Independently measured clockwise rotation (rad).
    double gyroAngle = 0;      // Raw continuous gyro increment (rad).
    double forwardTravel = 0, lateralTravel = 0; // Raw pod travel (m).
    double leftTravel = 0, rightTravel = 0;       // Calibrated wheel travel (m).
    bool gyroValid = true, forwardValid = true, lateralValid = true, driveValid = true;
};
struct RotationFit {
    ScalarFit gyroScale{}, forwardOffset{}, lateralOffset{}, trackWidth{};
};
// Pod geometry requires pure turns. Finite drive travel detects arcs and
// corrects small forward-center drift; without it, verify the pure turn externally.
RotationFit fitRotation(const std::vector<RotationSample>& samples,
                        double forwardScale = 1.0, double lateralScale = 1.0);

enum CalibrationField : std::uint32_t {
    forwardScaleField = 1u, lateralScaleField = 2u, gyroScaleField = 4u,
    forwardOffsetField = 8u, lateralOffsetField = 16u, trackWidthField = 32u,
    leftDynamicsField = 64u, rightDynamicsField = 128u
};
struct CalibrationProfile {
    std::uint64_t hardwareFingerprint = 0; // Change when hardware/season changes.
    std::uint32_t calibratedMask = 0;     // Defaults are explicitly uncalibrated.
    double forwardScale = 1.0, lateralScale = 1.0, gyroScale = 1.0;
    double forwardOffset = 17.4752 * millimeter, lateralOffset = 54.991 * millimeter;
    double trackWidth = 283.972 * millimeter;
    double kSLeft = 0.6, kVLeft = 6.0, kALeft = 0.65;
    double kSRight = 0.6, kVRight = 6.0, kARight = 0.65;
};
bool validProfile(const CalibrationProfile& profile);
// Apply only fields whose calibrated bits are set. Invalid profiles change nothing.
bool applyProfile(const CalibrationProfile& profile, EstimatorConfig& estimator,
                  DynamicsConfig& dynamics);
enum class StorageStatus { ok, uncalibrated, invalidProfile, wrongHardware,
                           unsupportedVersion, checksumMismatch, ioError };
// Portable, versioned, little-endian binary file with CRC32. Main + .bak recovery.
StorageStatus saveProfile(const std::string& path, const CalibrationProfile& profile);
StorageStatus loadProfile(const std::string& path, std::uint64_t expectedHardwareFingerprint,
                          CalibrationProfile& profile, bool* recoveredBackup = nullptr);

}} // namespace nexus::calibration
