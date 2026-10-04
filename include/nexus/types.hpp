#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <algorithm>

namespace nexus {
// Core: metres, seconds, radians. X right, Y forward, heading clockwise from +Y.
constexpr double pi = 3.14159265358979323846;
constexpr double millimeter = 0.001; // Convert user-facing millimetres to internal metres.
constexpr double inch = 25.4 * millimeter; // Wheel diameters only.
constexpr double radians(double degrees) { return degrees * pi / 180.0; }
constexpr double degrees(double angle) { return angle * 180.0 / pi; }
inline double wrap(double angle) { return std::remainder(angle, 2.0 * pi); }
inline double square(double x) { return x * x; }
struct Pose { double x = 0, y = 0, theta = 0; };
struct DriveState {
    Pose pose{};
    double v = 0, omega = 0;
    // Body-frame velocity to the right, m/s, observed by the lateral pod.
    // Keep last so existing {pose, forwardVelocity, angularVelocity} remains valid.
    double lateralV = 0;
};
struct Voltage { double left = 0, right = 0; }; // Volts, not millivolts.
struct Field { double minX = -1.8288, maxX = 1.8288, minY = -1.8288, maxY = 1.8288; };
constexpr std::size_t maxRanges = 4;
struct RangeMount { double x = 0, y = 0, heading = 0; }; // Body X right, Y forward.
struct RangeCalibration {
    double scale = 1, offset = 0; // corrected metres = scale * raw metres + offset.
    double minimum = 0.02, maximum = 2.0; // Validated raw-distance interval, metres.
};
struct RangeNoise {
    // Negative values inherit the estimator-wide conservative noise model.
    double standardDeviation = -1, relativeStd = -1;
};
struct RangeReading {
    double distance = 0;
    double timestamp = 0; // Monotonic seconds, acquisition time (latency corrected).
    int confidence = 0;
    bool valid = false;
    bool confidenceAvailable = true; // V5 hardware does not report confidence at raw distance <=200 mm.
};
struct SensorSample {
    double timestamp = 0;
    double forward = 0, lateral = 0; // Cumulative tracking wheel travel.
    double left = 0, right = 0; // Cumulative drivetrain wheel travel.
    double gyro = 0; // Continuous clockwise IMU rotation, not wrapped heading.
    double gyroRate = 0; // Clockwise rad/s.
    bool forwardValid = false, lateralValid = false;
    bool leftValid = false, rightValid = false, gyroValid = false;
    bool gyroRateValid = false;
    // Native IMU diagnostics only; not integrated into the position estimate.
    std::array<double, 3> accelerationG{}, rawGyroDps{};
    bool accelerationValid = false, rawGyroValid = false;
    std::array<RangeReading, maxRanges> ranges{};
};
enum class Health { healthy, degraded, lost };
enum class HeadingSource { unavailable, imu, driveEncoders };
// Cumulative world-frame translation since reset, in metres. These terms
// explain odometry; they are not independent measurements of the true pose.
struct OdometryTravel {
    double forwardX = 0, forwardY = 0;
    double lateralX = 0, lateralY = 0;
    double offsetX = 0, offsetY = 0;
    // Integrated odometric rotation since reset, unwrapped, in radians.
    // Extrema retain transients even after heading returns to its start.
    // Absolute range corrections do not contribute to these three values.
    double headingChange = 0, minimumHeadingChange = 0, maximumHeadingChange = 0;
};
struct QuietDiagnostics {
    // Last reasoned restart of the quiet detector, retained through later quiet.
    // Bits: 1 F span, 2 lateral span, 4 left span, 8 right span, 16 gyro rate,
    // 32 interval >stationaryMaxInterval, 64 gyro/yaw evidence unavailable, 128 wheel mask
    // changed, 256 rejected increment. More than one reason may apply.
    std::uint16_t lastBreakMask = 0;
    // Counts updates restarting quiet for an actual reason, not SI->NO edges.
    // Initialization/reconfigure baselines are excluded. Saturates at uint32 max.
    std::uint32_t breaks = 0;
    // Pre-restart scaled cumulative ranges, metres, F/lateral/left/right.
    // Strict zero-tolerance mode records that update's absolute increments.
    std::array<double, 4> wheelSpans{};
    double gyroRate = 0; // Last restart's angle difference / interval, rad/s.
    double sampleInterval = 0, timestamp = 0; // Last restart, seconds.
    double dwell = 0; // Current uninterrupted qualifying interval, seconds.
    double encoderSpan = 0; // Current configured quiet-window span, metres.
};
struct Estimate {
    DriveState state{};
    std::array<std::array<double, 3>, 3> covariance{}; // x,y,heading marginal.
    double gyroBias = 0; // Effective local correction; zero with native IMU heading.
    double gyroBiasStd = 0; // Estimated bias std, rad/s; zero when local bias is disabled.
    double confidence = 0;
    double slip = 0;
    double timestamp = 0;
    std::array<double, maxRanges> rangeResidual{};
    std::uint32_t acceptedRanges = 0, rejectedRanges = 0;
    // Count channels rejected by cumulative-increment plausibility checks;
    // missing/nonfinite sensor readings and reconnect baselines are not counted.
    std::uint32_t rejectedIncrements = 0;
    // Bits: 1 forward pod, 2 lateral pod, 4 left drive, 8 right drive, 16 IMU.
    std::uint8_t rejectedIncrementMask = 0; // This update only.
    std::uint8_t lastRejectedIncrementMask = 0; // Latest rejection, retained until reset.
    // Updates without a usable increment: forward pod, lateral pod, IMU.
    // Includes invalid input, rejected jumps and reconnection baselines;
    // counts updates, not fault events, including intentionally absent sensors.
    // Initial/reconfigure baseline and whole acquisition blackouts are excluded.
    // Preserved on reconfigure; cleared by reset; never counted again in replay.
    std::array<std::uint32_t, 3> unusableIncrements{};
    bool stationary = false; // Independent wheel evidence passed the stationary dwell.
    QuietDiagnostics quiet{}; // Reset clears; reconfigure preserves history and clears dwell.
    OdometryTravel odometry{};
    Health health = Health::lost;
    HeadingSource headingSource = HeadingSource::unavailable;
};
struct EstimatorConfig {
    double trackWidth = 283.972 * millimeter;
    double forwardOffset = 17.4752 * millimeter; // Pod position X; forward = reading + X*dtheta.
    double lateralOffset = 54.991 * millimeter; // Pod position Y; lateral = reading - Y*dtheta.
    double gyroScale = 1.0;
    // Integrate the sensor's scaled IMU angle directly, including during quiet.
    // Disables local bias learning/correction and stationary wheel-yaw replacement;
    // setGyroBias returns false. Drive-encoder heading fallback remains available.
    // Entering this policy discards the old bias; leaving starts a fresh bias prior.
    bool nativeImuHeading = false;
    double forwardScale = 1.0, lateralScale = 1.0;
    double encoderStd = 0.0015, gyroStd = radians(0.12);
    double maxSpeed = 3.0, maxOmega = 18.0;
    // Extra elapsed time allowed for a previously unchanged cumulative sensor
    // when checking its next increment. Zero requires synchronous samples.
    // Set from a known device/cache refresh bound; never infer unlimited age at rest.
    double maxSensorHold = 0;
    // Maximum cumulative wheel-counter span during a quiet window, metres.
    // Zero requires unchanged counters; values are capped at 0.1 mm.
    // This is not a per-sample deadband:
    // measured translation remains integrated even inside the quiet band.
    double stationaryEncoderSpan = 0;
    // Longest interval that can certify quiet, seconds, capped at 100 ms.
    // Allow scheduling jitter above the configured acquisition period.
    double stationaryMaxInterval = 0.05;
    double rangeStd = 0.025, rangeGate = 9.0;
    double rangeRelativeStd = 0.05; // Conservative raw V5 distance error; calibrate before reducing.
    double maxRangeCorrection = 0.10;
    Field field{};
    std::array<RangeMount, maxRanges> mounts{};
    std::size_t rangeCount = 0;
    std::array<RangeNoise, maxRanges> rangeNoise{};
};
struct DynamicsConfig {
    double trackWidth = 283.972 * millimeter;
    double kVLeft = 6.0, kVRight = 6.0; // V per (m/s).
    double kALeft = 0.65, kARight = 0.65; // V per (m/s^2).
    double kSLeft = 0.6, kSRight = 0.6; // Static friction V.
    double maxAcceleration = 3.5, maxLateralAcceleration = 3.0;
    double maxSpeed = 1.65, maxOmega = 8.0;
    double maxVoltage = 12.0;
    double commandLatency = 0.01;
};
struct MotionOptions {
    bool reverse = false;
    double maxSpeed = 1.4; // Internal m/s; public MoveOptions uses mm/s.
    double positionTolerance = 0.015;
    double headingTolerance = radians(1.5);
    double settleTime = 0.18;
    double timeout = 5.0;
};
struct Target { Pose pose{}; bool constrainHeading = false; bool turnOnly = false; };
struct SolverStats {
    unsigned iterations = 0;
    double initialCost = 0, finalCost = 0;
    double computeMs = 0;
    bool converged = false, valid = false, budgetExceeded = false;
};
} // namespace nexus
