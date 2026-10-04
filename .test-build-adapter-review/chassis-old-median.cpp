#include "nexus/chassis.hpp"
#include "nexus/calibration_routine.hpp"
#include "nexus/imu_status.hpp"
#include "pros/misc.hpp"
#include "pros/error.h"
#include <limits>

namespace nexus {
static_assert(pros::E_IMU_STATUS_CALIBRATING == 1 && pros::E_IMU_STATUS_ERROR == 0xff);
namespace {
struct Lock {
    pros::Mutex& mutex;
    explicit Lock(pros::Mutex& value) : mutex(value) { mutex.take(); }
    ~Lock() { mutex.give(); }
};
double nowSeconds() { return static_cast<double>(pros::micros()) * 1e-6; }
bool finite(double value) { return std::isfinite(value) && value != PROS_ERR_F; }
double curve(int value, double cubicBlend) {
    if (std::abs(value) < 4) return 0;
    const double normalized = std::clamp(value / 127.0, -1.0, 1.0);
    cubicBlend = std::isfinite(cubicBlend) ? std::clamp(cubicBlend, 0.0, 1.0) : 0.55;
    return (1.0 - cubicBlend) * normalized + cubicBlend * normalized * normalized * normalized;
}
}

const char* statusName(MotionStatus status) {
    switch (status) {
    case MotionStatus::idle: return "idle";
    case MotionStatus::running: return "running";
    case MotionStatus::settled: return "settled";
    case MotionStatus::timedOut: return "timeout";
    case MotionStatus::cancelled: return "cancelled";
    case MotionStatus::sensorFault: return "sensor fault";
    case MotionStatus::solverFault: return "solver fault";
    case MotionStatus::invalidRequest: return "invalid request";
    }
    return "unknown";
}

Chassis::Chassis(Hardware hardware, HardwareConfig hardwareConfig,
                 EstimatorConfig estimatorConfig, DynamicsConfig dynamicsConfig)
    : hw_(hardware), hwConfig_(hardwareConfig), estimator_(estimatorConfig), controller_(dynamicsConfig) {}

void Chassis::begin() {
    if (begun_.load()) return;
    requestQueue_ = pros::c::queue_create(16, sizeof(Request));
    snapshotQueue_ = pros::c::queue_create(1, sizeof(Diagnostics));
    for (auto& queue : responses_) queue = pros::c::queue_create(1, sizeof(Response));
    if (!requestQueue_ || !snapshotQueue_ ||
        std::any_of(responses_.begin(), responses_.end(), [](auto queue) { return !queue; })) return;
    hw_.left.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
    hw_.right.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
    hw_.left.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    hw_.right.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    // PROS still copies its shared sensor buffer every 10 ms. The documented
    // 5 ms device rate reduces the age of each value within that buffer.
    if (hw_.imu) hw_.imu->set_data_rate(5);
    if (hw_.forwardPod) hw_.forwardPod->set_data_rate(5);
    if (hw_.lateralPod) hw_.lateralPod->set_data_rate(5);
    controller_.setClock(nowSeconds);
    controller_.setBudgetMs(hwConfig_.solverBudgetMs);
    begun_.store(true);
    publish();
    // Competition callbacks may be killed by PROS at any instruction. Only
    // persistent workers hold mutex_; public methods exchange copied queues.
    requestTask_ = std::make_unique<pros::Task>([this] { requestLoop(); },
        TASK_PRIORITY_DEFAULT + 2, 16384, "nexus-requests");
    // Estimation has higher priority than optimization. No solver runs under mutex.
    estimatorTask_ = std::make_unique<pros::Task>([this] { estimatorLoop(); },
        TASK_PRIORITY_DEFAULT + 2, 16384, "nexus-estimator");
    controllerTask_ = std::make_unique<pros::Task>([this] { controllerLoop(); },
        TASK_PRIORITY_DEFAULT + 1, 32768, "nexus-nmpc");
}

SensorSample Chassis::readSensors() {
    SensorSample sample;
    sample.timestamp = nowSeconds();
    const auto readPod = [this](pros::Rotation* pod, std::size_t index, bool reversed,
                               double diameter, double& value) {
        if (!pod || !finite(diameter) || diameter <= 0) return false;
        const auto raw = pod->get_position();
        const auto actualReversed = pod->get_reversed();
        if (raw == PROS_ERR || actualReversed == PROS_ERR) {
            podConfigured_[index] = false;
            return false;
        }
        if (!podConfigured_[index] || actualReversed != static_cast<int>(reversed)) {
            // A sensor absent at construction never received its direction.
            // A reboot can also reset direction entirely between two polls.
            // Rebind before acquiring a fresh baseline in either case.
            podConfigured_[index] = pod->set_reversed(reversed) == PROS_SUCCESS &&
                                    pod->set_data_rate(5) == PROS_SUCCESS;
            return false;
        }
        value = static_cast<double>(raw) / 36000.0 * pi * diameter * inch;
        return finite(value);
    };
    sample.forwardValid = readPod(hw_.forwardPod, 0, hwConfig_.forwardPodReversed,
                                  hwConfig_.forwardWheelDiameter, sample.forward);
    sample.lateralValid = readPod(hw_.lateralPod, 1, hwConfig_.lateralPodReversed,
                                  hwConfig_.lateralWheelDiameter, sample.lateral);
    // Capture the three primary odometry counters together, before motor and
    // diagnostic reads. This narrows software skew without pretending that
    // separate PROS devices expose synchronized acquisition timestamps.
    const bool imuReady = hw_.imu &&
        detail::imuStatusReady(static_cast<std::uint32_t>(hw_.imu->get_status()));
    const bool validGyroSign = hwConfig_.gyroSign == 1.0 || hwConfig_.gyroSign == -1.0;
    if (imuReady) {
        if (!imuConfigured_) imuConfigured_ = hw_.imu->set_data_rate(5) == PROS_SUCCESS;
        const double rotation = hw_.imu->get_rotation();
        sample.gyroValid = validGyroSign && finite(rotation);
        if (sample.gyroValid) sample.gyro = radians(rotation) * hwConfig_.gyroSign;
    } else imuConfigured_ = false;
    // Keep the reset margin, but allow real travel across a delayed acquisition.
    // The fastest supported cartridge is 600 RPM (3600 degrees/second).
    // Longer blackouts are independently rebaselined by the estimator.
    const double motorInterval = previousMotorSampleAt_ < 0 ? .01 : sample.timestamp - previousMotorSampleAt_;
    previousMotorSampleAt_ = sample.timestamp;
    const double motorDeltaLimit = 180.0 + 3600.0 * std::clamp(motorInterval - .01, 0.0, .99);
    // Median aligned cumulative positions. Summing medians of per-tick changes
    // can invent travel when asynchronous/noisy encoders each return to start.
    const auto readSide = [this, motorDeltaLimit](pros::MotorGroup& motors, std::size_t side, std::size_t count, double& value) {
        if (!finite(hwConfig_.driveWheelDiameter) || hwConfig_.driveWheelDiameter <= 0 ||
            !finite(hwConfig_.driveWheelPerMotor) || hwConfig_.driveWheelPerMotor <= 0) return false;
        std::array<double, 8> positions{};
        std::array<bool, 8> rebaselined{};
        const double travelPerDegree = pi * hwConfig_.driveWheelDiameter * inch * hwConfig_.driveWheelPerMotor / 360.0;
        std::size_t valid = 0;
        for (std::size_t i = 0; i < std::min(count, positions.size()); ++i) {
            const double raw = motors.get_position(static_cast<std::uint8_t>(i));
            bool validNow = finite(raw);
            if (validNow && !previousMotorValid_[side][i]) {
                const auto index = static_cast<std::uint8_t>(i);
                previousMotorValid_[side][i] = motors.set_gearing(hwConfig_.driveGearset, index) == PROS_SUCCESS &&
                    motors.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES, index) == PROS_SUCCESS &&
                    motors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST, index) == PROS_SUCCESS;
                // Read only after degrees were restored, never compare old units.
                previousMotorPosition_[side][i] = motors.get_position(index);
                previousMotorValid_[side][i] = previousMotorValid_[side][i] && finite(previousMotorPosition_[side][i]);
                rebaselined[i] = previousMotorValid_[side][i];
                continue;
            }
            if (validNow && previousMotorValid_[side][i]) {
                const double change = raw - previousMotorPosition_[side][i];
                // Reconnected/reset encoders establish a new baseline, never a jump.
                if (finite(change) && std::abs(change) <= motorDeltaLimit) positions[valid++] = change;
                // A rejected counter cannot become the next baseline: a later,
                // slower acquisition could otherwise accept its rebound.
                else validNow = false;
            }
            previousMotorValid_[side][i] = validNow;
            if (validNow) previousMotorPosition_[side][i] = raw;
        }
        if (valid == 0) {
            for (std::size_t i = 0; i < rebaselined.size(); ++i)
                if (rebaselined[i]) alignedMotorTravel_[side][i] = motorTravel_[side];
            return false;
        }
        for (std::size_t i = 1; i < valid; ++i) {
            const double key = positions[i];
            std::size_t j = i;
            while (j > 0 && positions[j - 1] > key) { positions[j] = positions[j - 1]; --j; }
            positions[j] = key;
        }
        const double median = valid % 2 ? positions[valid / 2]
            : (positions[valid / 2 - 1] + positions[valid / 2]) * 0.5;
        // A recovering encoder joins at the CURRENT aggregate, after the
        // surviving encoders have contributed this frame's real travel.
        for (std::size_t i = 0; i < rebaselined.size(); ++i)
            if (rebaselined[i]) alignedMotorTravel_[side][i] = median;
        // Changing the valid set can move the median within the surviving
        // encoder disagreement, but cannot accumulate bounded encoder jitter.
        value = motorTravel_[side] += median * travelPerDegree;
        return true;
    };
    sample.leftValid = readSide(hw_.left, 0, hwConfig_.leftMotorCount, sample.left);
    sample.rightValid = readSide(hw_.right, 1, hwConfig_.rightMotorCount, sample.right);
    if (imuReady) {
        const auto rate = hw_.imu->get_gyro_rate();
        const auto acceleration = hw_.imu->get_accel();
        sample.rawGyroDps = {rate.x, rate.y, rate.z};
        sample.accelerationG = {acceleration.x, acceleration.y, acceleration.z};
        sample.rawGyroValid = finite(rate.x) && finite(rate.y) && finite(rate.z);
        sample.accelerationValid = finite(acceleration.x) && finite(acceleration.y) && finite(acceleration.z);
        sample.gyroRateValid = validGyroSign && hwConfig_.useRawGyroRate && finite(rate.z);
        if (sample.gyroRateValid) sample.gyroRate = radians(rate.z) * hwConfig_.gyroSign;
    }
    // PROS exposes no acquisition timestamp. Decimate to avoid overcounting cached readings.
    const bool pollRanges = sample.timestamp - lastRangePoll_ >= 0.05;
    if (pollRanges) lastRangePoll_ = sample.timestamp;
    for (std::size_t i = 0; pollRanges && i < maxRanges; ++i) {
        if (!hw_.distances[i]) continue;
        auto& range = sample.ranges[i];
        const auto distance = hw_.distances[i]->get();
        range.confidence = hw_.distances[i]->get_confidence();
        range.confidenceAvailable = distance > 200;
        range.timestamp = sample.timestamp - hwConfig_.rangeLatency[i];
        const auto& fit = hwConfig_.rangeCalibration[i];
        const double raw = distance * 0.001;
        const double corrected = raw * fit.scale + fit.offset;
        range.valid = distance != PROS_ERR && distance >= 20 && distance <= 2000
                      && range.confidence != PROS_ERR && range.confidence >= 0 && range.confidence <= 63
                      && finite(hwConfig_.rangeLatency[i]) && hwConfig_.rangeLatency[i] >= 0
                      && finite(fit.scale) && fit.scale >= .8 && fit.scale <= 1.2
                      && finite(fit.offset) && std::abs(fit.offset) <= .1
                      && finite(fit.minimum) && finite(fit.maximum) && fit.minimum >= .02
                      && fit.maximum <= 2 && fit.minimum < fit.maximum
                      && raw >= fit.minimum - 1e-9 && raw <= fit.maximum + 1e-9
                      && finite(corrected) && corrected >= .02 - 1e-9 && corrected <= 2 + 1e-9;
        if (range.valid) range.distance = std::clamp(corrected, .02, 2.0);
    }
    return sample;
}

void Chassis::estimatorLoop() {
    std::uint32_t wake = pros::millis();
    for (;;) {
        const double started = nowSeconds();
        std::uint32_t epoch;
        { Lock lock(mutex_); epoch = sensorEpoch_; }
        const SensorSample sample = readSensors();
        {
            Lock lock(mutex_);
            if (epoch == sensorEpoch_) {
                diagnostics_.estimate = estimator_.update(sample);
                diagnostics_.sensors = sample;
                if (sample.accelerationValid) {
                    double peak = 0;
                    for (double axis : sample.accelerationG) peak = std::max<double>(peak, std::abs(axis));
                    diagnostics_.peakAccelerationG = std::max(diagnostics_.peakAccelerationG, peak);
                    if (peak >= 3.8) ++diagnostics_.accelerationNearLimit;
                }
                if (sample.rawGyroValid) {
                    double peak = 0;
                    for (double axis : sample.rawGyroDps) peak = std::max<double>(peak, std::abs(axis));
                    diagnostics_.peakGyroRateDps = std::max(diagnostics_.peakGyroRateDps, peak);
                    if (peak >= 950) ++diagnostics_.gyroNearLimit;
                }
            }
            const bool superseded = generation_ != commandGeneration_.load();
            const bool changedMode = pros::competition::is_disabled() ||
                static_cast<bool>(pros::competition::is_autonomous()) != autonomousMode_ ||
                static_cast<bool>(pros::competition::is_connected()) != connectedMode_;
            if ((requested_ || manual_) && (superseded || changedMode)) {
                requested_ = manual_ = false;
                diagnostics_.status = MotionStatus::cancelled;
                brake();
            }
            if (manual_ && sample.timestamp - lastManualAt_ > .10) { manual_ = false; brake(); }
            if (calibrating_ && (superseded || !practiceAllowed() ||
                sample.timestamp - calibrationAt_ > .10 || !freshCalibrationSensors(sample.timestamp))) {
                calibrating_ = false;
                diagnostics_.status = MotionStatus::cancelled;
                brake();
            }
            // Independent of optimizer progress: estimation task can revoke stale output.
            if (requested_ && sample.timestamp - lastControlAt_ > 0.10) {
                requested_ = false;
                diagnostics_.status = MotionStatus::solverFault;
                brake();
            }
            diagnostics_.estimatorMs = (nowSeconds() - started) * 1000.0;
            diagnostics_.worstEstimatorMs = std::max(diagnostics_.worstEstimatorMs, diagnostics_.estimatorMs);
            if (diagnostics_.estimatorMs > 10.0) ++diagnostics_.estimatorOverruns;
            publish();
        }
        if (pros::millis() - wake >= 10) wake = pros::millis();
        pros::Task::delay_until(&wake, 10);
    }
}

void Chassis::brake() { hw_.left.brake(); hw_.right.brake(); diagnostics_.command = {}; }

void Chassis::publish() {
    diagnostics_.generation = generation_;
    diagnostics_.motionActive = requested_;
    diagnostics_.calibrationActive = calibrating_;
    pros::c::queue_reset(snapshotQueue_);
    pros::c::queue_append(snapshotQueue_, &diagnostics_, 0);
}

bool Chassis::practiceAllowed() const {
    return !pros::competition::is_connected() && !pros::competition::is_autonomous() &&
           !pros::competition::is_disabled();
}
bool Chassis::freshCalibrationSensors(double now) const {
    const auto& sensors = diagnostics_.sensors;
    return sensors.gyroValid && sensors.leftValid && sensors.rightValid &&
           now - sensors.timestamp >= 0 && now - sensors.timestamp <= .10;
}
bool Chassis::stopped() const {
    return !requested_ && !manual_ && !calibrating_ &&
           std::abs(diagnostics_.command.left) < .001 && std::abs(diagnostics_.command.right) < .001 &&
           std::abs(diagnostics_.estimate.state.v) < .025 &&
           std::abs(diagnostics_.estimate.state.omega) < radians(5);
}

void Chassis::controllerLoop() {
    std::uint32_t wake = pros::millis(), solverGeneration = 0;
    double previousTime = nowSeconds();
    for (;;) {
        const double started = nowSeconds();
        const double dt = std::clamp(started - previousTime, 0.005, 0.10);
        previousTime = started;
        Estimate estimate;
        Target target;
        MotionOptions options;
        std::uint32_t generation;
        bool active, wasAutonomous, wasConnected;
        bool reconfigure;
        DynamicsConfig dynamics;
        double requestedAt;
        {
            Lock lock(mutex_);
            estimate = diagnostics_.estimate;
            target = target_; options = options_; generation = generation_;
            active = requested_; requestedAt = requestedAt_;
            wasAutonomous = autonomousMode_; wasConnected = connectedMode_;
            reconfigure = configurationPending_;
            if (reconfigure) { dynamics = pendingDynamics_; configurationPending_ = false; }
        }
        if (reconfigure) { controller_.reconfigure(dynamics); solverGeneration = 0; }
        if (active) {
            MotionStatus outcome = MotionStatus::running;
            Voltage voltage{};
            if (pros::competition::is_disabled()
                || pros::competition::is_autonomous() != wasAutonomous
                || pros::competition::is_connected() != wasConnected) outcome = MotionStatus::cancelled;
            else if (estimate.health == Health::lost && started - requestedAt < 0.05) {
                // A setPose immediately followed by move is valid: acquire fresh baselines.
                pros::Task::delay_until(&wake, 20);
                continue;
            }
            else if (estimate.health == Health::lost || started - estimate.timestamp > 0.10)
                outcome = MotionStatus::sensorFault;
            else if (started - requestedAt >= options.timeout) outcome = MotionStatus::timedOut;
            else {
                if (generation != solverGeneration) {
                    controller_.reset();
                    controller_.start(estimate.state, target, options);
                    solverGeneration = generation;
                }
                const double battery = pros::battery::get_voltage() * 0.001;
                if (!finite(battery) || battery < 7.0 || battery > 15.0) outcome = MotionStatus::solverFault;
                else {
                    voltage = controller_.update(estimate, dt, battery,
                        std::max(0.0, started - estimate.timestamp));
                    const auto stats = controller_.stats();
                    if (!stats.valid || !finite(voltage.left) || !finite(voltage.right))
                        outcome = MotionStatus::solverFault;
                    else if (controller_.settled(estimate, dt)) outcome = MotionStatus::settled;
                }
            }
            // Fence late optimizer output: cancel/manual drive/new request always wins.
            Lock lock(mutex_);
            diagnostics_.solver = controller_.stats();
            diagnostics_.worstSolverMs = std::max(diagnostics_.worstSolverMs, diagnostics_.solver.computeMs);
            if ((nowSeconds() - started) > 0.02) ++diagnostics_.controllerOverruns;
            if (generation == generation_ && generation == commandGeneration_.load() && requested_) {
                if (pros::competition::is_disabled()
                    || pros::competition::is_autonomous() != wasAutonomous
                    || pros::competition::is_connected() != wasConnected) outcome = MotionStatus::cancelled;
                else if (diagnostics_.estimate.health == Health::lost
                    || nowSeconds() - diagnostics_.estimate.timestamp > 0.10) outcome = MotionStatus::sensorFault;
                diagnostics_.status = outcome;
                if (outcome == MotionStatus::running) {
                    diagnostics_.command = voltage;
                    lastControlAt_ = nowSeconds();
                    hw_.left.move_voltage(static_cast<int>(std::clamp(voltage.left, -12.0, 12.0) * 1000));
                    hw_.right.move_voltage(static_cast<int>(std::clamp(voltage.right, -12.0, 12.0) * 1000));
                } else { requested_ = false; brake(); }
            }
            publish();
        }
        if (pros::millis() - wake >= 20) wake = pros::millis();
        pros::Task::delay_until(&wake, 20);
    }
}

Chassis::Request Chassis::request(RequestKind kind, bool invalidateMotion, bool cancelSequence) {
    Request value;
    value.kind = kind;
    value.id = requestCounter_.fetch_add(1) + 1;
    value.createdAt = nowSeconds();
    value.autonomous = pros::competition::is_autonomous();
    value.connected = pros::competition::is_connected();
    if (cancelSequence) cancellationEpoch_.fetch_add(1);
    value.generation = invalidateMotion ? commandGeneration_.fetch_add(1) + 1 : commandGeneration_.load();
    return value;
}

void Chassis::abandon(const Request& value) {
    const bool ownsControl = value.kind == RequestKind::pose || value.kind == RequestKind::move ||
        value.kind == RequestKind::cancel || value.kind == RequestKind::arcade ||
        (value.kind == RequestKind::calibrationVoltage && value.calibrationToken == value.generation);
    auto expected = value.generation;
    if (ownsControl && commandGeneration_.compare_exchange_strong(expected, value.generation + 1))
        cancellationEpoch_.fetch_add(1);
}

Chassis::Response Chassis::send(Request value, bool wait) {
    if (!begun_.load() || !pros::c::queue_append(requestQueue_, &value, 0)) {
        abandon(value);
        return {};
    }
    if (!wait) return {value.id, value.generation, true};
    const auto started = pros::millis();
    Response response;
    while (pros::millis() - started < 250) {
        if (pros::c::queue_peek(responses_[value.id % responseSlots], &response, 0)) {
            if (response.id == value.id) return response;
            // A sufficiently newer ticket has reused this bounded mailbox.
            if (static_cast<std::int32_t>(response.id - value.id) > 0) { abandon(value); return {}; }
        }
        pros::delay(1);
    }
    // A delayed motion must not appear after its caller gave up waiting.
    abandon(value);
    return {};
}

void Chassis::requestLoop() {
    Request value;
    for (;;) {
        if (!pros::c::queue_recv(requestQueue_, &value, 10)) continue;
        Response result{value.id, value.generation, false};
        {
            Lock lock(mutex_);
            const double now = nowSeconds();
            const bool current = value.generation == commandGeneration_.load();
            const bool modeMatches = value.autonomous == static_cast<bool>(pros::competition::is_autonomous()) &&
                                     value.connected == static_cast<bool>(pros::competition::is_connected());
            const bool fresh = now - value.createdAt <= .10;
            const bool sequenceMatches = !value.hasCancellationToken ||
                (modeMatches && !pros::competition::is_disabled() &&
                 value.cancellationToken == cancellationEpoch_.load());
            if (!current || !fresh || !sequenceMatches) result.value = 0;
            if (current && fresh && sequenceMatches) {
                switch (value.kind) {
                case RequestKind::pose:
                    if (!modeMatches) break;
                    requested_ = manual_ = calibrating_ = false;
                    generation_ = value.generation;
                    ++sensorEpoch_; brake();
                    estimator_.reset(value.target.pose, now);
                    diagnostics_.estimate = estimator_.estimate();
                    diagnostics_.status = MotionStatus::idle;
                    result.accepted = true;
                    break;
                case RequestKind::cancel:
                    requested_ = manual_ = calibrating_ = false;
                    generation_ = value.generation;
                    diagnostics_.status = MotionStatus::cancelled;
                    brake(); result.accepted = true;
                    break;
                case RequestKind::arcade: {
                    requested_ = calibrating_ = false;
                    generation_ = value.generation;
                    autonomousMode_ = value.autonomous; connectedMode_ = value.connected;
                    manual_ = modeMatches && !pros::competition::is_disabled();
                    diagnostics_.status = MotionStatus::idle;
                    if (!manual_) { brake(); break; }
                    const double forward = curve(value.throttle, value.arcadeCurves.forward);
                    const double rotation = curve(value.turn, value.arcadeCurves.turn) * .75;
                    const double scale = std::max<double>(1.0, std::abs(forward) + std::abs(rotation));
                    hw_.left.move(static_cast<int>((forward + rotation) / scale * 127));
                    hw_.right.move(static_cast<int>((forward - rotation) / scale * 127));
                    diagnostics_.command = {(forward + rotation) / scale * 12, (forward - rotation) / scale * 12};
                    lastManualAt_ = now;
                    result.accepted = true;
                    break;
                }
                case RequestKind::move: {
                    requested_ = manual_ = calibrating_ = false;
                    generation_ = value.generation; brake();
                    const auto& target = value.target;
                    const auto& options = value.options;
                    const bool valid = finite(target.pose.x) && finite(target.pose.y) && finite(target.pose.theta) &&
                        finite(options.maxSpeed) && options.maxSpeed > 0 && options.timeout > 0 && options.timeout <= 120000 &&
                        finite(options.positionTolerance) && options.positionTolerance > 0 &&
                        finite(options.headingTolerance) && options.headingTolerance > 0 &&
                        options.settleTime > 0 && options.settleTime < options.timeout;
                    if (!valid) { diagnostics_.status = MotionStatus::invalidRequest; break; }
                    if (!modeMatches || pros::competition::is_disabled() ||
                        (value.hasCancellationToken && value.cancellationToken != cancellationEpoch_.load())) {
                        diagnostics_.status = MotionStatus::cancelled; break;
                    }
                    target_ = target; options_ = optionsToCore(options);
                    autonomousMode_ = value.autonomous; connectedMode_ = value.connected;
                    requestedAt_ = lastControlAt_ = now; requested_ = true;
                    diagnostics_.status = MotionStatus::running;
                    result.accepted = true;
                    break;
                }
                case RequestKind::practiceStop:
                case RequestKind::calibrationBegin: {
                    if (!modeMatches || !practiceAllowed() ||
                        (value.kind == RequestKind::calibrationBegin && !freshCalibrationSensors(now))) break;
                    auto expected = value.generation;
                    if (!commandGeneration_.compare_exchange_strong(expected, value.generation + 1)) break;
                    cancellationEpoch_.fetch_add(1);
                    requested_ = manual_ = false;
                    calibrating_ = value.kind == RequestKind::calibrationBegin;
                    generation_ = value.generation + 1;
                    result.value = generation_;
                    calibrationAt_ = now;
                    brake();
                    diagnostics_.status = MotionStatus::idle;
                    result.accepted = true;
                    break;
                }
                case RequestKind::calibrationVoltage:
                    if (!calibrating_ || value.calibrationToken != generation_) break;
                    if (practiceAllowed() &&
                        freshCalibrationSensors(now) && now - calibrationAt_ <= .10 &&
                        finite(value.left) && finite(value.right)) {
                        calibrationAt_ = now;
                        diagnostics_.command = {
                            std::clamp(value.left, -GeometryRoutine::turnVoltage, GeometryRoutine::turnVoltage),
                            std::clamp(value.right, -GeometryRoutine::turnVoltage, GeometryRoutine::turnVoltage)};
                        hw_.left.move_voltage(static_cast<int>(diagnostics_.command.left * 1000));
                        hw_.right.move_voltage(static_cast<int>(diagnostics_.command.right * 1000));
                        result.accepted = true;
                    } else { calibrating_ = false; brake(); }
                    break;
                case RequestKind::calibrationEnd:
                    if (calibrating_ && value.calibrationToken == generation_) {
                        calibrating_ = false; brake(); result.accepted = true;
                    }
                    break;
                case RequestKind::configure:
                    if (stopped()) {
                        estimator_.reconfigure(value.estimatorConfig);
                        diagnostics_.estimate = estimator_.estimate();
                        ++sensorEpoch_;
                        pendingDynamics_ = value.dynamicsConfig; configurationPending_ = true;
                        result.accepted = true;
                    }
                    break;
                case RequestKind::gyroBias:
                    if (stopped()) {
                        result.accepted = estimator_.setGyroBias(value.bias, value.variance);
                        diagnostics_.estimate = estimator_.estimate();
                    }
                    break;
                }
            }
            publish();
        }
        // Queue storage belongs to Chassis, never to a possibly deleted caller.
        auto queue = responses_[value.id % responseSlots];
        pros::c::queue_reset(queue);
        pros::c::queue_append(queue, &result, 0);
    }
}

void Chassis::setPose(double x, double y, double heading) {
    if (!finite(x) || !finite(y) || !finite(heading)) { cancel(); return; }
    auto value = request(RequestKind::pose, true);
    value.target.pose = poseToCore(x, y, heading);
    if (!send(value).accepted) abandon(value);
}
RobotPose Chassis::getPose() { return poseToMillimeters(diagnostics().estimate.state.pose); }
Diagnostics Chassis::diagnostics() {
    Diagnostics snapshot;
    if (begun_.load()) pros::c::queue_peek(snapshotQueue_, &snapshot, 20);
    return snapshot;
}
bool Chassis::busy() { return diagnostics().motionActive; }
std::uint32_t Chassis::cancellationEpoch() { return cancellationEpoch_.load(); }
void Chassis::cancel() { send(request(RequestKind::cancel, true, true)); }
void Chassis::arcade(int throttle, int turn, ArcadeCurves curves) {
    auto value = request(RequestKind::arcade, true, true);
    value.throttle = std::clamp(throttle, -127, 127); value.turn = std::clamp(turn, -127, 127);
    value.arcadeCurves = curves;
    send(value, false);
}
std::uint32_t Chassis::beginCalibration() {
    const auto result = send(request(RequestKind::calibrationBegin, false));
    return result.accepted ? result.value : 0;
}
bool Chassis::calibrationVoltage(std::uint32_t token, double left, double right) {
    auto value = request(RequestKind::calibrationVoltage, false);
    value.calibrationToken = token; value.left = left; value.right = right;
    return send(value).accepted;
}
bool Chassis::endCalibration(std::uint32_t token) {
    auto value = request(RequestKind::calibrationEnd, false);
    value.calibrationToken = token;
    return send(value).accepted;
}
bool Chassis::stopPractice() { return send(request(RequestKind::practiceStop, false)).accepted; }
bool Chassis::applyConfiguration(EstimatorConfig estimatorConfig, DynamicsConfig dynamicsConfig) {
    auto value = request(RequestKind::configure, false);
    value.estimatorConfig = estimatorConfig; value.dynamicsConfig = dynamicsConfig;
    return send(value).accepted;
}
bool Chassis::setGyroBias(double bias, double variance) {
    auto value = request(RequestKind::gyroBias, false);
    value.bias = bias; value.variance = variance;
    return send(value).accepted;
}
std::uint32_t Chassis::submit(Target target, MoveOptions options) {
    auto value = request(RequestKind::move, true);
    value.target = target; value.options = options;
    const auto result = send(value);
    return result.id ? result.value : 0;
}
std::uint32_t Chassis::startMoveToPoint(double x, double y, MoveOptions options) {
    return submit({poseToCore(x, y, 0), false, false}, options);
}
std::uint32_t Chassis::startMoveToPose(double x, double y, double heading, MoveOptions options) {
    return submit({poseToCore(x, y, heading), true, false}, options);
}
std::uint32_t Chassis::startTurnToHeading(double heading, MoveOptions options) {
    const RobotPose pose = getPose();
    return submit({poseToCore(pose.x, pose.y, heading), true, true}, options);
}
MotionResult Chassis::waitUntilDone(std::uint32_t handle) {
    if (!handle) return {MotionStatus::invalidRequest};
    for (;;) {
        if (handle != commandGeneration_.load()) return {MotionStatus::cancelled};
        const auto snapshot = diagnostics();
        if (snapshot.generation == handle && !snapshot.motionActive) return {snapshot.status};
        pros::delay(10);
    }
}
MotionResult Chassis::moveToPoint(double x, double y, MoveOptions options) {
    return waitUntilDone(startMoveToPoint(x, y, options));
}
MotionResult Chassis::moveToPose(double x, double y, double heading, MoveOptions options) {
    return waitUntilDone(startMoveToPose(x, y, heading, options));
}
MotionResult Chassis::turnToHeading(double heading, MoveOptions options) {
    return waitUntilDone(startTurnToHeading(heading, options));
}

Sequence::Sequence(Chassis& chassis)
    : chassis_(chassis), cancellationEpoch_(chassis.cancellationEpoch()),
      generation_(chassis.commandGeneration_.load()),
      modeAutonomous_(pros::competition::is_autonomous()), modeConnected_(pros::competition::is_connected()) {}

bool Sequence::canContinue() {
    if (!result_) return false;
    if (chassis_.cancellationEpoch() != cancellationEpoch_ || chassis_.commandGeneration_.load() != generation_) {
        result_ = {MotionStatus::cancelled}; return false;
    }
    if (pros::competition::is_disabled() || pros::competition::is_autonomous() != modeAutonomous_
        || pros::competition::is_connected() != modeConnected_) {
        // The workers revoke old-mode output. A stale sequence must not cancel
        // a newer command that already belongs to the new mode.
        result_ = {MotionStatus::cancelled}; return false;
    }
    return true;
}
std::uint32_t Sequence::submit(Target target, MoveOptions options, bool poseOnly) {
    if (!canContinue()) return 0;
    auto value = chassis_.request(poseOnly ? Chassis::RequestKind::pose : Chassis::RequestKind::move, false);
    value.target = target; value.options = options;
    value.autonomous = modeAutonomous_; value.connected = modeConnected_;
    value.hasCancellationToken = true; value.cancellationToken = cancellationEpoch_;
    // A cancelled/suspended callback must never acquire ownership over a newer
    // command. Keep the original mode/token and claim only our own generation.
    auto expected = generation_;
    if (!chassis_.commandGeneration_.compare_exchange_strong(expected, generation_ + 1)) {
        result_ = {MotionStatus::cancelled}; return 0;
    }
    value.generation = ++generation_;
    const auto response = chassis_.send(value);
    if (!response.id || !response.value || (poseOnly && !response.accepted)) {
        chassis_.abandon(value);
        result_ = {MotionStatus::cancelled}; return 0;
    }
    return response.value;
}
Sequence& Sequence::setPose(double x, double y, double heading) {
    if (canContinue()) {
        if (!finite(x) || !finite(y) || !finite(heading)) result_ = {MotionStatus::invalidRequest};
        else if (submit({poseToCore(x, y, heading), false, false}, {}, true)) wait(30);
    }
    return *this;
}
Sequence& Sequence::moveToPoint(double x, double y, MoveOptions options) {
    if (canContinue()) {
        const auto handle = submit({poseToCore(x, y, 0), false, false}, options);
        result_ = handle ? chassis_.waitUntilDone(handle) : MotionResult{MotionStatus::cancelled};
    }
    return *this;
}
Sequence& Sequence::moveToPose(double x, double y, double heading, MoveOptions options) {
    if (canContinue()) {
        const auto handle = submit({poseToCore(x, y, heading), true, false}, options);
        result_ = handle ? chassis_.waitUntilDone(handle) : MotionResult{MotionStatus::cancelled};
    }
    return *this;
}
Sequence& Sequence::turnToHeading(double heading, MoveOptions options) {
    if (canContinue()) {
        const auto pose = chassis_.getPose();
        const auto handle = submit({poseToCore(pose.x, pose.y, heading), true, true}, options);
        result_ = handle ? chassis_.waitUntilDone(handle) : MotionResult{MotionStatus::cancelled};
    }
    return *this;
}
Sequence& Sequence::wait(std::uint32_t milliseconds) {
    const auto start = pros::millis();
    while (canContinue() && pros::millis() - start < milliseconds) pros::delay(10);
    return *this;
}
} // namespace nexus
