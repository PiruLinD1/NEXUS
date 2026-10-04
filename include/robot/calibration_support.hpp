#pragma once
#include "nexus/calibration.hpp"

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
} // namespace robot::detail
