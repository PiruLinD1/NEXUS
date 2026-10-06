#pragma once
#include <cstdint>

namespace robot::config::mechanism {
inline constexpr std::int8_t encoderPort = 19; // V5 Rotation; preMatch learns encoder polarity.
inline constexpr std::uint8_t opticalPort = 0; // Optical sensor disabled; object count is manual.
// Signs mapping logical commands to motors: positive means away from arm zero / lift up.
inline constexpr int armDirection = 1, liftDirection = -1;
inline constexpr double encoderDegreesPerArmDegree = 7.0; // 7 encoder turns per arm turn.
inline constexpr double armClearanceDeg = 144.0; // Physical arm angle, independent of the encoder gearing.
inline constexpr double armClearanceEncoderDegrees = armClearanceDeg * encoderDegreesPerArmDegree;
inline constexpr double armReturnClearanceToleranceDeg = 5.0; // R2 return only: accept 139..149 arm degrees.
inline constexpr double liftLowerRotations = 3.25; // Low pin pickup position, measured down from the calibrated top.
inline constexpr double pickupLiftRotations = 0.18; // Clearance rise after closing the clamp.
inline constexpr double returnLiftRotations = 3.0; // Distance below the calibrated top before the front arm endstop.
inline constexpr double driverLiftStepRotations = 1.38;
// Extra travel below the normal low position, only with the arm at endstop 2.
inline constexpr double liftEndstopExtraLowerRotations = 0.68;
inline constexpr double armPickupDeg = 150, armReleaseDeg = 90;
inline constexpr int armReleasePower = 80; // Fast open-loop R2 move, stopped by the Rotation.
inline constexpr double armReleaseToleranceDeg = 5;
inline constexpr std::uint32_t sensorRecoveryMs = 200, driverBlockedMs = 1500;
inline constexpr int armMoveMaxPower = 127, armApproachPower = 30;
inline constexpr double armAccelerationPowerPerSecond = 1600, armDecelerationPowerPerSecond = 5000;
inline constexpr double armSlowdownDeg = 24, armBrakeLookaheadSeconds = 0.07;
inline constexpr double armBrakeMarginDeg = 0.5;
inline constexpr double armMotionSpeedFilterSeconds = 0.02, armArrivalSpeedDegPerSecond = 5;
inline constexpr std::uint32_t armArrivalMs = 40, armBrakePauseMs = 60;
inline constexpr std::uint32_t armBrakeMaxMs = 200; // Restore load support even if gravity prevents a passive stop.
inline constexpr unsigned armMaxCorrections = 2;
inline constexpr int armFinalCorrectionPower = 20;
// Raise effort only when a powered approach is too slow under load, without PID.
inline constexpr double armLoadAssistSpeedDegPerSecond = 20;
inline constexpr double armLoadAssistPowerPerSecond = 400;
inline constexpr std::uint32_t armLoadAssistDelayMs = 60;
inline constexpr double armEndstopContactDeg = 4;
inline constexpr int armEndstopContactPower = 40; // Final contact after the fast, profiled approach.
inline constexpr double liftKp = 220.0; // Downward response; avoid losing torque near the lower target.
inline constexpr double liftUpKp = 200.0;
inline constexpr int liftUpMinPower = 70;
inline constexpr int liftMaxPower = 127, liftUpMaxPower = 127, liftHomePower = 80;
inline constexpr double liftProgressRotations = 0.03;
inline constexpr std::uint32_t liftBlockedMs = 800;
inline constexpr double liftBlockedCurrentMa = 900, liftBlockedVelocityRpm = 5;
inline constexpr double liftUpOverloadCurrentMa = 2200;
inline constexpr std::uint32_t liftUpOverloadMs = 150;
inline constexpr std::uint32_t liftEndstopContactMs = 150;
inline constexpr int armHomePower = 127; // Initial endstop discovery and return to zero.
inline constexpr std::uint32_t armDirectionGraceMs = 350;
inline constexpr int armDirectionMinPower = 12;
inline constexpr double armDirectionCheckErrorDeg = 6;
inline constexpr int armBackoffPower = 25; // Constant power for both calibration backoffs.
inline constexpr double stallCurrentMa = 1200, stallVelocityRpm = 5;
// Independent arm contact threshold; also requires low velocity and a stationary encoder.
inline constexpr double armHomeCurrentMa = 400, armStillToleranceDeg = 0.4;
inline constexpr std::uint32_t armHomeStillMs = 400, armBlockedMs = 800;
// Known endstops during driver macros need a shorter contact confirmation.
// Initial/repeated preMatch keeps the original 350ms grace and 400ms stillness.
inline constexpr std::uint32_t armDriverHomeGraceMs = 100, armDriverHomeStillMs = 100;
inline constexpr double armProgressDeg = 0.5, armWrongDirectionDeg = 3;
inline constexpr std::uint32_t stallMs = 250, homeGraceMs = 350;
inline constexpr std::uint32_t motionTimeoutMs = 7000, settleMs = 150;
inline constexpr double armToleranceDeg = 2, liftToleranceRot = 0.025;
inline constexpr double armBackoffDeg = 12, liftBackoffRot = 0.15;
inline constexpr double repeatArmToleranceDeg = 4, repeatLiftToleranceRot = 0.05;
// Required physical travel must cover every macro target.
inline constexpr double minimumArmTravelDeg = armClearanceDeg > armPickupDeg ? armClearanceDeg : armPickupDeg;
inline constexpr int proximityEnter = 150, proximityExit = 90;
inline constexpr double minimumSaturation = 0.35;
inline constexpr double redMax = 25, redMin = 335, yellowMin = 35, yellowMax = 80;
inline constexpr double blueMin = 180, blueMax = 260;
inline constexpr std::uint32_t objectDebounceMs = 60, classifyMs = 120;
inline constexpr std::uint32_t intakeReverseHoldMs = 400; // X: hold before reversing; release resumes forward.
inline constexpr std::uint32_t pickupClampPauseMs = 300; // R1: close clamp before moving lift/arm.
inline constexpr std::uint32_t releaseClampPauseMs = 200; // R2: wait after the arm stops before opening.
inline constexpr std::uint32_t captureMs = 1000, reverseMs = 1500, releasePauseMs = 250;
inline constexpr double releaseReturnPodDegrees = 180; // Half a forward/backward pod turn since R2 before automatic return.
static_assert(releaseReturnPodDegrees > 0);
static_assert(liftLowerRotations > pickupLiftRotations && pickupLiftRotations > 0);
static_assert(driverLiftStepRotations > 0);
static_assert(returnLiftRotations > 0 && returnLiftRotations < liftLowerRotations);
static_assert(liftEndstopExtraLowerRotations > 0 && liftEndstopExtraLowerRotations < driverLiftStepRotations);
static_assert(encoderDegreesPerArmDegree > 0 && proximityExit < proximityEnter);
static_assert(armBackoffPower > 0 && armBackoffPower <= 127);
static_assert(armApproachPower > 0 && armApproachPower <= armMoveMaxPower && armMoveMaxPower <= 127);
static_assert(armSlowdownDeg > 0 && armBrakeLookaheadSeconds >= 0);
static_assert(armBrakeMarginDeg >= 0 && armBrakeMarginDeg < armToleranceDeg);
static_assert(armFinalCorrectionPower > 0 && armFinalCorrectionPower <= armApproachPower);
static_assert(armLoadAssistSpeedDegPerSecond > armArrivalSpeedDegPerSecond && armLoadAssistPowerPerSecond > 0);
static_assert(armReleasePower > 0 && armReleasePower <= 127);
static_assert(armHomePower > 0 && armHomePower <= 127);
static_assert(armEndstopContactDeg > armToleranceDeg && armEndstopContactPower > 0 && armEndstopContactPower <= 127);
}
