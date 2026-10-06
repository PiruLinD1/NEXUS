#pragma once
#include "nexus/calibration.hpp"

namespace nexus { struct Diagnostics; }

namespace robot::detail {
// Internal bridge; normal robot setup remains in robot-config.hpp and main.cpp.
struct CalibrationContext {
    nexus::EstimatorConfig estimator{};
    nexus::calibration::CalibrationProfile profile{};
    bool imuCalibrationFailed = false;
};
struct CalibrationApplyResult {
    bool applied = false;
    nexus::calibration::StorageStatus storage = nexus::calibration::StorageStatus::uncalibrated;
};
CalibrationContext calibrationContext();
CalibrationApplyResult applyCalibrationProfile(const nexus::calibration::CalibrationProfile& profile, bool persist);
void startCalibrationService();
void publishCalibrationInput(std::uint32_t buttons, bool connected);
void abortCalibration();
bool calibrationScreenActive();
// Bounded, nonblocking USB recording. False means data loss: abort the trial.
bool publishCharacterization(std::uint32_t run, std::uint32_t trial,
    std::uint32_t stage, std::uint32_t failure, const nexus::Diagnostics& data);
void setCharacterizationTelemetry(bool enabled);
} // namespace robot::detail
