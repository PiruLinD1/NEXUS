#pragma once
#include <array>
#include <cstdint>
#include <initializer_list>

// EDIT HERE for each new robot. Lengths in mm; wheel diameters in inches; angles in degrees.
namespace robot::config {
inline constexpr std::uint64_t robotRevision = 2026092001ULL;
inline constexpr std::initializer_list<std::int8_t> leftPorts = {5, -4, 6};
inline constexpr std::initializer_list<std::int8_t> rightPorts = {-7, 9, -10};
// Index 1 is port 4/9: fixed 200 RPM 5.5 W motors (USB type readback), with
// separate external gearing. Keep all six
// motors in the drive groups, but use only the four 600 RPM encoders for odometry.
inline constexpr std::uint8_t leftOdometryMask = 0b101, rightOdometryMask = 0b101;
// Diagnostic trial: use only the two tracking pods and native IMU for odometry.
// Keep the physical encoder masks above for the existing calibration fingerprint.
inline constexpr bool useDriveEncodersForOdometry = false;
inline constexpr std::initializer_list<std::int8_t> liftPorts = {13, -14};
inline constexpr std::initializer_list<std::int8_t> intakePorts = {-15};
inline constexpr std::int8_t braccioPort = 20;
inline constexpr std::uint8_t clampPort = 'D';
inline constexpr bool clampClosedLevel = true;

// Use 0 to disable an optional sensor. Negative pod port reverses its direction.
inline constexpr std::uint8_t imuPort = 1;
inline constexpr std::int8_t forwardPodPort = 2;
inline constexpr std::int8_t lateralPodPort = -3;
inline constexpr std::uint32_t odometryPeriodMs = 1; // Software polling: nominal 1000 Hz; PROS cache refreshes at 10 ms.
inline constexpr double trackWidth = 287.0; // mm
inline constexpr double driveWheelDiameter = 3.25; // inches
inline constexpr double motorRPM = 600, wheelRPM = 360; // Blue + external gearing.
inline constexpr double forwardWheelDiameter = 2.0, lateralWheelDiameter = 2.0; // inches
// Measured from the geometric drive centre by the user, 2026-10-04.
// Use these physical references; the automatic offset trial is not repeatable yet.
inline constexpr double forwardPodX = 20.0; // mm; right of centre is positive.
inline constexpr double lateralPodY = -70.0; // mm; in front of centre is positive.
// IMU angle scale: true rotation = scale * IMU rotation. Measure it with N full
// turns against a physical reference: scale = (360 * N) / IMU reading.
// 1.0 = uncorrected. A 1% error gives ~70 mm closure error on the main auton.
// Measured 2026-10-04, 10 turns each way: clockwise 3614.4 deg, counter-
// clockwise 3616.0 deg. Mean 3615.2 -> 3600 / 3615.2. The 0.04% difference
// between directions is within the manual alignment error: one scale suffices.
inline constexpr double imuScale = 0.99400; //0.995796;

struct WallSensor {
    std::uint8_t port;
    double x, y, heading, latencyMs; // mm, mm, degrees, ms
    double distanceScale = 1, distanceOffset = 0; // corrected mm = scale * raw mm + offset.
    double minimumDistance = 20, maximumDistance = 2000; // Raw mm covered by calibration.
    double distanceStd = -1, distanceRelativeStd = -1; // mm/fraction; -1 inherits conservative defaults.
};
// At most two physical Distance sensors; use perpendicular beams (e.g. front/right).
// Port 0 means absent. Unused library slots preserve the existing hardware fingerprint.
inline constexpr std::array<WallSensor, 4> wallSensors{{
    {0, 0, 0, 0, 0}, {0, 0, 0, 90, 0},
    {0, 0, 0, 180, 0}, {0, 0, 0, -90, 0}
}};
static_assert([] {
    unsigned count = 0;
    for (const auto& sensor : wallSensors) if (sensor.port) ++count;
    return count <= 2;
}(), "This robot supports at most two physical Distance sensors");
inline constexpr double fieldMinX = -1828.8, fieldMaxX = 1828.8, fieldMinY = -1828.8, fieldMaxY = 1828.8;

// Initial physical model: replace with measured values from calibration.
inline constexpr double kVLeft = 0.006, kVRight = 0.006; // V/(mm/s)
inline constexpr double kALeft = 0.00065, kARight = 0.00065; // V/(mm/s^2)
inline constexpr double kSLeft = 0.6, kSRight = 0.6; // V
inline constexpr double maxSpeed = 1650; // mm/s
inline constexpr double maxAcceleration = 3500; // mm/s^2, longitudinal.
// All-omni drivetrain: conservative initial cornering limit, to measure on the
// field. It limits v*omega; it is not an odometry correction or a measured grip.
inline constexpr double maxLateralAcceleration = 1000; // mm/s^2
inline constexpr double commandLatencyMs = 10; // Measure on the robot before changing.
inline constexpr const char* calibrationPath = "/usd/nexus.cfg";
inline constexpr bool logTelemetry = false; // CSV to microSD, 20 Hz, separate low-priority task.
// Diagnostic USB stream of every acquired odometry frame. Disable after the audit.
inline constexpr bool logOdometryUsb = true;

static_assert(trackWidth > 0 && driveWheelDiameter > 0 && wheelRPM > 0 && motorRPM > 0);
static_assert(forwardWheelDiameter > 0 && lateralWheelDiameter > 0);
static_assert(odometryPeriodMs >= 1 && odometryPeriodMs <= 50);
static_assert(fieldMinX < fieldMaxX && fieldMinY < fieldMaxY);
static_assert(leftPorts.size() > 0 && leftPorts.size() <= 8 && rightPorts.size() > 0 && rightPorts.size() <= 8);
static_assert(clampPort >= 'A' && clampPort <= 'H');
} // namespace robot::config
