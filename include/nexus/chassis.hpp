#pragma once

#include "nexus/types.hpp"
#include "nexus/autonomous_units.hpp"
#include "nexus/estimator.hpp"
#include "nexus/controller.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "pros/distance.hpp"
#include "pros/rtos.hpp"
#include "pros/apix.h"
#include <atomic>
#include <functional>
#include <memory>

namespace nexus {
struct Hardware {
    pros::MotorGroup& left;
    pros::MotorGroup& right;
    pros::Imu* imu = nullptr;
    pros::Rotation* forwardPod = nullptr;
    pros::Rotation* lateralPod = nullptr;
    std::array<pros::Distance*, maxRanges> distances{};
};
struct HardwareConfig {
    double driveWheelDiameter = 3.25; // Inches; converted only when reading encoders.
    double driveWheelPerMotor = 450.0 / 600.0;
    double forwardWheelDiameter = 2.125; // Inches.
    double lateralWheelDiameter = 2.125; // Inches.
    std::size_t leftMotorCount = 3, rightMotorCount = 3;
    bool forwardPodReversed = false, lateralPodReversed = false;
    pros::MotorGearset driveGearset = pros::MotorGearset::blue;
    double gyroSign = 1.0; // +1 or -1; scale calibration belongs in EstimatorConfig.
    bool useRawGyroRate = false; // Enable only after checking mounted raw Z sign against rotation.
    std::array<double, maxRanges> rangeLatency{}; // seconds, measured per sensor.
    double solverBudgetMs = 12.0;
    std::array<RangeCalibration, maxRanges> rangeCalibration{};
    bool captureOdometryTrace = false;
    // Optional diagnostic packet clock: 0 forward pod, 1 lateral pod, 2 IMU.
    // Must return promptly; never used to time or correct the estimator.
    std::uint32_t (*tracePacketClock)(std::size_t source) = nullptr;
    // Bit i selects motor-group index i for odometry, not for actuation.
    // Selected encoders must share driveGearset and driveWheelPerMotor.
    std::uint8_t leftOdometryMask = 0xff, rightOdometryMask = 0xff;
    // An all-omni robot cannot observe sideways travel from drive encoders.
    bool requireLateralPodForMotion = false;
    std::uint32_t odometryPeriodMs = 10; // Software polling/update period, clamped to 1..50 ms.
};

struct ArcadeCurves {
    // Cubic blend: 0 = linear, 1 = cubic (softer near center).
    double forward = 0.55, turn = 0.55;
};
enum class MotionStatus { idle, running, settled, timedOut, cancelled, sensorFault, solverFault, invalidRequest };
enum class CalibrationSensors { driveEncoders, trackingPods, characterization };
const char* statusName(MotionStatus status);
struct MotionResult {
    MotionStatus status = MotionStatus::idle;
    explicit operator bool() const { return status == MotionStatus::settled; }
};
struct EstimatorTiming {
    std::uint32_t periodMs = 10;
    double longIntervalLimitMs = 50;
    // Start-to-start interval, including publication and sleep/scheduling.
    double intervalMs = 0, worstIntervalMs = 0;
    std::uint32_t longIntervals = 0;
    // Latched together for the last interval over longIntervalLimitMs in either clock.
    double lastAt = 0, lastIntervalMs = 0;
    std::uint32_t lastRtosMs = 0;
    double lastWorkMs = 0, lastPublishMs = 0, lastWaitMs = 0;
    // Maximum micros() bracket around either millis() reading for this pair.
    // A large bracket prevents attributing disagreement to the clocks alone.
    double lastClockReadMs = 0;
};
struct Diagnostics {
    // Raw mathematical snapshot in SI. Brain display and CSV convert lengths to mm.
    Estimate estimate{};
    SensorSample sensors{};
    Voltage command{};
    SolverStats solver{};
    MotionStatus status = MotionStatus::idle;
    double estimatorMs = 0, worstEstimatorMs = 0, worstSolverMs = 0;
    EstimatorTiming timing{};
    std::uint32_t estimatorOverruns = 0, controllerOverruns = 0;
    double peakAccelerationG = 0, peakGyroRateDps = 0;
    std::uint32_t accelerationNearLimit = 0, gyroNearLimit = 0;
    std::uint32_t generation = 0;
    bool motionActive = false, calibrationActive = false;
    Target target{};
    DriveState reference{};
    double controlTimestamp = 0;
    std::uint32_t controlGeneration = 0;
};

struct OdometryTrace {
    // Sequence counts committed samples, including records dropped by a full
    // queue. Epoch changes on pose/configuration reset; neither resets sequence.
    std::uint32_t sequence = 0, dropped = 0, epoch = 0;
    double timestamp = 0; // Software acquisition start, seconds.
    std::array<double, 5> counters{}; // Forward, lateral, left, right metres; gyro radians.
    Pose pose{};
    double gyroBias = 0;
    std::array<double, 3> rawGyroDps{};
    // Bits 0..4: F/L/left/right/gyro; bit 5: native gyro XYZ valid.
    std::uint8_t validMask = 0, rejectedMask = 0;
    HeadingSource headingSource = HeadingSource::unavailable;
    bool stationary = false;
    // Vendor packet-clock values around each primary getter, F/L/IMU.
    // Zero also means unavailable. Different clocks from timestamp above;
    // these are observations, not assertions of atomic/physical acquisition.
    std::array<std::uint32_t, 3> packetBeforeMs{}, packetAfterMs{};
};

// Own this object for the lifetime of the robot. No copying or task teardown races.
class Chassis {
public:
    Chassis(Hardware hardware, HardwareConfig hardwareConfig,
            EstimatorConfig estimatorConfig, DynamicsConfig dynamicsConfig);
    Chassis(const Chassis&) = delete;
    Chassis& operator=(const Chassis&) = delete;
    void begin(); // Starts estimation and motion workers; never starts a motion.
    void setPose(double x, double y, double heading);
    RobotPose getPose();
    Diagnostics diagnostics();
    // Optional, one consumer. Starting is idempotent and never clears records.
    bool startOdometryTrace();
    bool popOdometryTrace(OdometryTrace& trace); // Nonblocking FIFO read.
    std::uint32_t droppedOdometryTrace(); // Saturating count, including trailing losses.
    MotionResult moveToPoint(double x, double y, MoveOptions options = {});
    MotionResult moveToPose(double x, double y, double heading, MoveOptions options = {});
    // One continuous route through 2..8 points; stop only at its final pose.
    MotionResult moveThrough(std::initializer_list<Waypoint> points, double heading, MoveOptions options = {});
    MotionResult turnToHeading(double heading, MoveOptions options = {});
    std::uint32_t startMoveToPoint(double x, double y, MoveOptions options = {});
    std::uint32_t startMoveToPose(double x, double y, double heading, MoveOptions options = {});
    std::uint32_t startMoveThrough(std::initializer_list<Waypoint> points, double heading, MoveOptions options = {});
    std::uint32_t startTurnToHeading(double heading, MoveOptions options = {});
    MotionResult waitUntilDone(std::uint32_t handle);
    void cancel();
    void arcade(int throttle, int turn, ArcadeCurves curves = {}); // -127..127, cancels autonomous control.
    bool busy();
    std::uint32_t cancellationEpoch();
    // Supervised practice only; commands expire after 100 ms without renewal.
    std::uint32_t beginCalibration(CalibrationSensors sensors = CalibrationSensors::driveEncoders);
    bool calibrationVoltage(std::uint32_t token, double leftVolts, double rightVolts);
    bool endCalibration(std::uint32_t token);
    bool stopPractice(); // Conditional UI stop; cannot cancel a later competition motion.
    bool applyConfiguration(EstimatorConfig estimatorConfig, DynamicsConfig dynamicsConfig);
    bool setGyroBias(double bias, double variance);
private:
    friend class Sequence;
    enum class RequestKind { pose, move, cancel, arcade, practiceStop, calibrationBegin, calibrationVoltage, calibrationEnd, configure, gyroBias };
    struct Request {
        RequestKind kind = RequestKind::cancel;
        std::uint32_t id = 0, generation = 0, cancellationToken = 0, calibrationToken = 0;
        bool hasCancellationToken = false, autonomous = false, connected = false;
        CalibrationSensors calibrationSensors = CalibrationSensors::driveEncoders;
        double createdAt = 0;
        Target target{};
        MoveOptions options{};
        int throttle = 0, turn = 0;
        ArcadeCurves arcadeCurves{};
        double left = 0, right = 0, bias = 0, variance = 0;
        EstimatorConfig estimatorConfig{};
        DynamicsConfig dynamicsConfig{};
    };
    struct Response { std::uint32_t id = 0, value = 0; bool accepted = false; };
    static constexpr std::size_t responseSlots = 32;
    Hardware hw_;
    HardwareConfig hwConfig_;
    Estimator estimator_;
    Controller controller_;
    pros::Mutex mutex_;
    std::unique_ptr<pros::Task> estimatorTask_, controllerTask_, requestTask_;
    pros::c::queue_t requestQueue_ = nullptr, snapshotQueue_ = nullptr;
    pros::c::queue_t odometryTraceQueue_ = nullptr;
    std::atomic<bool> odometryTraceActive_{false};
    std::atomic<std::uint32_t> odometryTraceDropped_{0};
    std::uint32_t odometryTraceSequence_ = 0; // Estimator worker only; wraps modulo 2^32.
    std::array<pros::c::queue_t, responseSlots> responses_{};
    Diagnostics diagnostics_{};
    Target target_{};
    MotionOptions options_{};
    std::atomic<bool> begun_{false};
    bool requested_ = false, autonomousMode_ = false, connectedMode_ = false;
    bool manual_ = false, calibrating_ = false, configurationPending_ = false;
    // Characterization zero commands coast; revocation uses passive BRAKE.
    bool characterizationMode_ = false, characterizationBraking_ = false;
    CalibrationSensors calibrationSensors_ = CalibrationSensors::driveEncoders;
    DynamicsConfig pendingDynamics_{};
    std::atomic<std::uint32_t> requestCounter_{0}, commandGeneration_{0}, cancellationEpoch_{0};
    std::uint32_t generation_ = 0;
    std::uint32_t sensorEpoch_ = 0;
    double requestedAt_ = 0;
    double lastControlAt_ = 0;
    double lastManualAt_ = 0, calibrationAt_ = 0;
    double lastRangePoll_ = -1;
    double previousMotorSampleAt_ = -1;
    std::array<std::array<double, 8>, 2> previousMotorPosition_{};
    // Each encoder is aligned to the side's current cumulative travel when
    // establishing a new baseline; medians operate on totals, never increments.
    std::array<std::array<double, 8>, 2> alignedMotorTravel_{};
    std::array<std::array<bool, 8>, 2> previousMotorValid_{};
    std::array<double, 2> motorTravel_{};
    std::array<bool, 2> podConfigured_{};
    bool imuConfigured_ = false;
    SensorSample readSensors(OdometryTrace* trace = nullptr);
    void estimatorLoop();
    void controllerLoop();
    void requestLoop();
    void publish(); // Internal worker lock held; queue copies are deletion-safe for readers.
    Response send(Request request, bool wait = true);
    void abandon(const Request& request);
    Request request(RequestKind kind, bool invalidateMotion, bool cancelSequence = false);
    bool practiceAllowed() const;
    bool freshCalibrationSensors(double now, CalibrationSensors sensors) const;
    bool stopped() const;
    std::uint32_t submit(Target target, MoveOptions options);
    void brake(); // Requires mutex while tasks are running.
    void restoreDriveCoast(); // Requires mutex; new accepted motion only.
};

// Blocking sequence: a timeout ends only its motion. Faults/cancellation skip later steps.
class Sequence {
public:
    explicit Sequence(Chassis& chassis);
    Sequence& setPose(double x, double y, double heading);
    Sequence& moveToPoint(double x, double y, MoveOptions options = {});
    // Optional caller-task actuator update; false revokes this motion and later steps.
    Sequence& moveToPose(double x, double y, double heading, MoveOptions options = {},
                         const std::function<bool()>& update = {});
    Sequence& moveThrough(std::initializer_list<Waypoint> points, double heading, MoveOptions options = {},
                          const std::function<bool()>& update = {});
    Sequence& turnToHeading(double heading, MoveOptions options = {});
    Sequence& wait(std::uint32_t milliseconds);
    Sequence& abort(MotionStatus status = MotionStatus::cancelled);
    template <typename Action> Sequence& action(Action action) {
        if (canContinue()) action();
        return *this;
    }
    MotionResult result() const { return result_; }
private:
    Chassis& chassis_;
    std::uint32_t cancellationEpoch_, generation_;
    MotionResult result_{MotionStatus::settled};
    bool modeAutonomous_ = false, modeConnected_ = false;
    bool canContinue();
    void awaitMotion(std::uint32_t handle, const std::function<bool()>& update);
    std::uint32_t submit(Target target, MoveOptions options, bool poseOnly = false);
};
} // namespace nexus
