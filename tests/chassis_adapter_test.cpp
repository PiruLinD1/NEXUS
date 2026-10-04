#include "nexus/chassis.hpp"
#include <cstdlib>
#include <iostream>

namespace {
std::array<std::atomic<std::uint32_t>, 3> packetClockCalls{};
std::uint32_t packetClock(std::size_t source) {
    // Deliberately changes between the two calls: these values must be
    // retained as observations, never trigger sensor retries/corrections.
    return static_cast<std::uint32_t>(source * 100000u) + ++packetClockCalls[source];
}
void require(bool condition, const char* message) {
    if (!condition) { std::cerr << "FAILED: " << message << '\n'; std::exit(1); }
}
template<class Predicate> bool eventually(Predicate predicate, unsigned timeout = 250) {
    const auto start = pros::millis();
    while (pros::millis() - start < timeout) {
        if (predicate()) return true;
        pros::delay(2);
    }
    return predicate();
}
void checkTimingStages(const nexus::EstimatorTiming& timing) {
    require(timing.lastWorkMs >= 0 && timing.lastPublishMs >= 0 && timing.lastWaitMs >= 0 &&
            timing.lastClockReadMs >= 0, "latched timing stages are nonnegative");
    require(std::abs(timing.lastWorkMs + timing.lastPublishMs + timing.lastWaitMs -
                     timing.lastIntervalMs) < .001,
            "work, publication and waiting account for the complete acquisition interval");
    require(timing.worstIntervalMs >= timing.lastIntervalMs,
            "worst acquisition interval includes the latched timing event");
}
void checkTimingPreserved(const nexus::EstimatorTiming& before, const nexus::EstimatorTiming& after) {
    require(after.longIntervals >= before.longIntervals && after.lastAt >= before.lastAt &&
            after.worstIntervalMs >= before.worstIntervalMs,
            "timing history survives later samples and pose reset");
    // An unrelated host scheduling pause can legitimately replace the event.
    // Without a newer event, all diagnostic fields must remain latched together.
    if (after.longIntervals == before.longIntervals)
        require(after.lastAt == before.lastAt && after.lastIntervalMs == before.lastIntervalMs &&
                after.lastRtosMs == before.lastRtosMs && after.lastWorkMs == before.lastWorkMs &&
                after.lastPublishMs == before.lastPublishMs && after.lastWaitMs == before.lastWaitMs &&
                after.lastClockReadMs == before.lastClockReadMs,
                "ordinary samples and setPose do not clear or mix the last timing event");
}
void checkRotationDirectionModel() {
    pros::Rotation sensor;
    sensor.position = 12345;
    require(sensor.get_position() == 12345 && sensor.set_reversed(true) == PROS_SUCCESS &&
            sensor.get_position() == 12345,
            "runtime reversal preserves a nonzero cumulative Rotation counter");
    sensor.position += 200;
    require(sensor.get_position() == 12145,
            "reversed Rotation changes only the sign of subsequent physical increments");
    require(sensor.set_reversed(true) == PROS_SUCCESS && sensor.get_position() == 12145,
            "reapplying the same Rotation direction preserves the cumulative counter");
    require(sensor.set_reversed(false) == PROS_SUCCESS && sensor.get_position() == 12145,
            "a second direction change preserves the current counter including its offset");
    sensor.position += 200;
    require(sensor.get_position() == 12345,
            "restored Rotation direction resumes positive increments without losing its offset");

    // Concurrent getters must never observe the new sign with the old offset.
    std::thread reversals([&] {
        for (int i = 0; i < 10000; ++i) sensor.set_reversed(i % 2 == 0);
    });
    bool positionPreserved = true;
    for (int i = 0; i < 10000; ++i)
        positionPreserved = (sensor.get_position() == 12345) && positionPreserved;
    reversals.join();
    require(positionPreserved && sensor.get_position() == 12345,
            "concurrent Rotation direction changes publish sign and offset together");
    sensor.simulate_reboot(321);
    require(sensor.get_position() == 321 && sensor.get_reversed() == 0 && sensor.dataRate == 10,
            "a simulated Rotation reboot resets the cumulative reference and configuration");
}
}

int main() {
    using namespace nexus;
    checkRotationDirectionModel();
    pros::MotorGroup left({1, -2, 3}), right({-4, 5, -6});
    pros::Imu imu;
    pros::Rotation forward, lateral;
    pros::Distance calibratedRange, uncalibratedRange, invalidRange, boundaryRange;
    lateral.connected = false;
    left.connected[0] = false;
    HardwareConfig hardwareConfig;
    hardwareConfig.captureOdometryTrace = true;
    hardwareConfig.requireLateralPodForMotion = true;
    hardwareConfig.tracePacketClock = packetClock;
    hardwareConfig.lateralPodReversed = true;
    hardwareConfig.rangeCalibration[0] = {1.05, .02, .2, 1.8};
    hardwareConfig.rangeCalibration[2].scale = std::numeric_limits<double>::quiet_NaN();
    hardwareConfig.rangeCalibration[3] = {1, 0, .7, 1.4};
    hardwareConfig.rangeLatency[0] = .015;
    EstimatorConfig adapterEstimatorConfig;
    adapterEstimatorConfig.stationaryEncoderSpan = .010 * millimeter;
    auto* chassis = new Chassis({left, right, &imu, &forward, &lateral,
        {&calibratedRange, &uncalibratedRange, &invalidRange, &boundaryRange}}, hardwareConfig, adapterEstimatorConfig, {});
    OdometryTrace trace;
    require(!chassis->startOdometryTrace() && !chassis->popOdometryTrace(trace),
            "optional trace cannot start or read before begin");
    chassis->begin();
    require(eventually([&] { return chassis->diagnostics().sensors.gyroValid &&
                                   chassis->diagnostics().sensors.leftValid; }), "sensor workers start");
    require(imu.dataRate == 5 && forward.dataRate == 5,
            "primary sensors request the documented freshest device refresh interval");
    require(eventually([&] {
        const auto data = chassis->diagnostics().sensors;
        return data.ranges[0].valid && data.ranges[1].valid && !data.ranges[2].valid &&
            std::abs(data.ranges[0].distance - 1.07) < 1e-10 &&
            std::abs(data.ranges[1].distance - 1.0) < 1e-10 &&
            std::abs(data.timestamp - data.ranges[0].timestamp - .015) < 1e-10;
    }), "individual distance scale/offset and latency preserve SI; invalid calibration rejected");
    calibratedRange.distance = 1900;
    require(eventually([&] {
        const auto data = chassis->diagnostics().sensors;
        return data.ranges[1].valid && !data.ranges[0].valid;
    }), "calibrated sensor cannot extrapolate beyond its measured raw-distance interval");
    calibratedRange.distance = 1000;
    calibratedRange.confidence = PROS_ERR;
    require(eventually([&] {
        const auto data = chassis->diagnostics().sensors;
        return data.ranges[1].valid && !data.ranges[0].valid;
    }), "distance calibration cannot make a failed sensor valid");
    calibratedRange.confidence = 63;
    calibratedRange.distance = 200;
    calibratedRange.confidence = 10;
    require(eventually([&] {
        const auto range = chassis->diagnostics().sensors.ranges[0];
        return range.valid && !range.confidenceAvailable && std::abs(range.distance - .230) < 1e-10;
    }), "confidence availability uses raw 200mm even when corrected distance exceeds 200mm");
    calibratedRange.distance = 201;
    require(eventually([&] {
        const auto range = chassis->diagnostics().sensors.ranges[0];
        return range.valid && range.confidenceAvailable && range.confidence == 10;
    }), "raw 201mm exposes measured weak confidence");
    calibratedRange.distance = 1000;
    calibratedRange.confidence = 63;
    uncalibratedRange.distance = 150;
    uncalibratedRange.confidence = 10;
    require(eventually([&] {
        const auto range = chassis->diagnostics().sensors.ranges[1];
        return range.valid && !range.confidenceAvailable && range.confidence == 10 &&
            std::abs(range.distance - .150) < 1e-10;
    }), "short-distance confidence is unavailable, not fabricated as high confidence");
    uncalibratedRange.confidence = PROS_ERR;
    require(eventually([&] {
        const auto data = chassis->diagnostics().sensors;
        return data.ranges[0].valid && !data.ranges[1].valid;
    }), "unavailable short-range confidence does not hide a PROS error");
    uncalibratedRange.confidence = 10;
    uncalibratedRange.distance = 250;
    require(eventually([&] {
        const auto range = chassis->diagnostics().sensors.ranges[1];
        return range.valid && range.confidenceAvailable && range.confidence == 10;
    }), "weak measured confidence beyond 200mm remains distinguishable from unavailable confidence");
    uncalibratedRange.distance = 1000;
    uncalibratedRange.confidence = 63;
    boundaryRange.distance = 700;
    require(eventually([&] {
        const auto range = chassis->diagnostics().sensors.ranges[3];
        return range.valid && std::abs(range.distance - .7) < 1e-10;
    }), "inclusive raw minimum survives SI floating-point rounding");
    boundaryRange.distance = 1400;
    require(eventually([&] {
        const auto range = chassis->diagnostics().sensors.ranges[3];
        return range.valid && std::abs(range.distance - 1.4) < 1e-10;
    }), "inclusive raw maximum survives SI floating-point rounding");
    boundaryRange.distance = 1401;
    require(eventually([&] {
        const auto data = chassis->diagnostics().sensors;
        return data.ranges[1].valid && !data.ranges[3].valid;
    }), "rounding tolerance cannot admit another raw millimeter outside calibration");
    require(!lateral.reversed && left.gearingWrites[0] == 0, "absent devices cannot receive startup settings");
    lateral.connected = true;
    left.connected[0] = true;
    require(eventually([&] { return lateral.reversed && lateral.dataRate == 5 && left.gearingWrites[0] > 0 &&
                                   left.unitWrites[0] > 0 && chassis->diagnostics().sensors.lateralValid; }),
            "late pod/motor connection reapplies direction, gear and encoder units");
    lateral.connected = false;
    require(eventually([&] { return !chassis->diagnostics().sensors.lateralValid; }), "pod dropout is visible");
    lateral.simulate_reboot(1234);
    lateral.connected = true;
    require(eventually([&] { return lateral.reversed && lateral.directionWrites >= 2; }),
            "sensor reboot restores configured reversal");
    imu.connected = false;
    require(eventually([&] { return !chassis->diagnostics().sensors.gyroValid; }), "IMU dropout is visible");
    imu.dataRate = 10;
    imu.connected = true;
    require(eventually([&] { return imu.dataRate == 5 && chassis->diagnostics().sensors.gyroValid; }),
            "IMU reconnection restores its configured refresh interval");

    // A delayed estimator tick must retain valid drive travel, even when the
    // cumulative motor increment exceeds the old fixed 180 degree cutoff.
    const auto beforeDelay = chassis->diagnostics().sensors;
    const auto gapsBeforeDelay = chassis->diagnostics().timing.longIntervals;
    pros::test::pauseEstimator = true;
    require(eventually([] { return pros::test::estimatorPaused.load(); }), "estimator pause is acknowledged");
    pros::delay(80);
    for (std::size_t i = 0; i < 3; ++i) left.position[i] = right.position[i] = 240;
    pros::test::pauseEstimator = false;
    const double delayedTravel = 240.0 / 360.0 * pi * hardwareConfig.driveWheelDiameter * inch * hardwareConfig.driveWheelPerMotor;
    require(eventually([&] {
        const auto sensors = chassis->diagnostics().sensors;
        return sensors.leftValid && sensors.rightValid &&
            std::abs(sensors.left - beforeDelay.left - delayedTravel) < 1e-10 &&
            std::abs(sensors.right - beforeDelay.right - delayedTravel) < 1e-10;
    }), "elapsed acquisition time preserves physically possible drive travel after a delayed tick");
    const auto delayedTiming = chassis->diagnostics().timing;
    require(delayedTiming.longIntervals > gapsBeforeDelay && delayedTiming.lastIntervalMs >= 80 &&
            delayedTiming.lastWaitMs >= 80,
            "an estimator pause after publication is captured as a long waiting interval");
    checkTimingStages(delayedTiming);
    require(std::abs(delayedTiming.lastIntervalMs - delayedTiming.lastRtosMs) <=
            delayedTiming.lastClockReadMs + 2,
            "both clocks describe the forced pause within their read bracket and millisecond quantization");
    require(eventually([&] { return chassis->diagnostics().sensors.timestamp > delayedTiming.lastAt + .015; }),
            "sampling resumes after the recorded pause");
    checkTimingPreserved(delayedTiming, chassis->diagnostics().timing);
    pros::test::pauseEstimator = true;
    require(eventually([] { return pros::test::estimatorPaused.load(); }), "second estimator pause is acknowledged");
    const double resetSampleAt = chassis->diagnostics().sensors.timestamp;
    for (std::size_t i = 0; i < 3; ++i) left.position[i] = right.position[i] = 10000;
    pros::test::pauseEstimator = false;
    require(eventually([&] {
        const auto afterReset = chassis->diagnostics().sensors;
        return afterReset.timestamp > resetSampleAt &&
            afterReset.leftValid && afterReset.rightValid &&
            std::abs(afterReset.left - beforeDelay.left - delayedTravel) < 1e-10 &&
            std::abs(afterReset.right - beforeDelay.right - delayedTravel) < 1e-10;
    }),
            "implausible encoder reset is rebaselined without adding drive travel");

    // Sample one tick, then pause after its publication. This exercises a
    // rejected spike followed by a slower tick where its rebound looks possible.
    const auto oneEstimatorCycle = [&] {
        pros::test::pauseAfterEstimatorCycle = true;
        pros::test::pauseEstimator = false;
        require(eventually([] { return pros::test::pauseEstimator.load() &&
                                       pros::test::estimatorPaused.load(); }),
                "one estimator sample completes before the next pause");
        return chassis->diagnostics().sensors;
    };
    pros::test::pauseEstimator = true;
    require(eventually([] { return pros::test::estimatorPaused.load(); }), "motor spike pause acknowledged");
    const auto beforeSpike = chassis->diagnostics().sensors;
    for (std::size_t i = 0; i < 3; ++i) left.position[i] = right.position[i] = 10400;
    const auto spike = oneEstimatorCycle();
    require(!spike.leftValid && !spike.rightValid, "400 degree instantaneous motor spike is rejected");
    pros::delay(100);
    for (std::size_t i = 0; i < 3; ++i) left.position[i] = right.position[i] = 10000;
    (void)oneEstimatorCycle();
    pros::test::pauseEstimator = false;
    require(eventually([&] {
        const auto sensors = chassis->diagnostics().sensors;
        return sensors.leftValid && sensors.rightValid &&
            std::abs(sensors.left - beforeSpike.left) < 1e-10 &&
            std::abs(sensors.right - beforeSpike.right) < 1e-10;
    }), "rejected motor spike cannot become travel when the next acquisition is delayed");

    // A direction reset may occur entirely between polls. A readable counter
    // with the wrong polarity must not feed an inverted cumulative delta.
    pros::test::pauseEstimator = true;
    require(eventually([] { return pros::test::estimatorPaused.load(); }), "pod direction pause acknowledged");
    lateral.position = 4000;
    (void)oneEstimatorCycle();
    (void)oneEstimatorCycle();
    const auto beforeDirectionReset = chassis->getPose();
    const auto directionWrites = lateral.directionWrites.load();
    lateral.simulate_reboot(5678);
    const auto resetDirection = oneEstimatorCycle();
    require(!resetDirection.lateralValid && lateral.reversed && lateral.dataRate == 5 &&
            lateral.directionWrites > directionWrites,
            "unexpected pod direction is restored without accepting its counter");
    (void)oneEstimatorCycle();
    (void)oneEstimatorCycle();
    const auto afterDirectionReset = chassis->getPose();
    require(std::hypot(afterDirectionReset.x - beforeDirectionReset.x,
                       afterDirectionReset.y - beforeDirectionReset.y) < .1,
            "silent pod reboot and direction restoration cannot create translation");

    // All three raw encoders remain in a 5.4 micrometre band and return to
    // precisely their own starting counts. A sum of median increments would
    // lose the three separately arriving positive edges but keep the shared
    // negative edge, inventing negative side travel on every closed cycle.
    const auto beforeJitter = chassis->diagnostics().sensors;
    const double travelPerDegree = pi * hardwareConfig.driveWheelDiameter * inch * hardwareConfig.driveWheelPerMotor / 360.0;
    for (int cycle = 0; cycle < 8; ++cycle) {
        for (std::size_t motor = 0; motor < 3; ++motor) {
            left.position[motor] = right.position[motor] = 10000.01;
            (void)oneEstimatorCycle();
        }
        for (std::size_t motor = 0; motor < 3; ++motor)
            left.position[motor] = right.position[motor] = 10000;
        const auto returned = oneEstimatorCycle();
        require(returned.leftValid && returned.rightValid &&
                std::abs(returned.left - beforeJitter.left) < 1e-10 &&
                std::abs(returned.right - beforeJitter.right) < 1e-10,
                "bounded asynchronous encoder jitter returning to start cannot accumulate side travel");
    }
    require(chassis->diagnostics().estimate.stationary,
            "bounded raw motor jitter remains quiet across the full stationary observation window");

    for (std::size_t motor = 0; motor < 3; ++motor) {
        left.position[motor] = right.position[motor] = 10030;
        (void)oneEstimatorCycle();
    }
    const auto forwardCycle = chassis->diagnostics().sensors;
    require(std::abs(forwardCycle.left - beforeJitter.left - 30 * travelPerDegree) < 1e-10 &&
            std::abs(forwardCycle.right - beforeJitter.right - 30 * travelPerDegree) < 1e-10,
            "asynchronously arriving real motion retains cumulative forward travel");
    for (std::size_t motor = 0; motor < 3; ++motor)
        left.position[motor] = right.position[motor] = 10000;
    const auto closedCycle = oneEstimatorCycle();
    require(std::abs(closedCycle.left - beforeJitter.left) < 1e-10 &&
            std::abs(closedCycle.right - beforeJitter.right) < 1e-10,
            "real forward and reverse travel closes even when motor updates arrive in different phases");

    // A motor that rejoins during motion must align to this frame's surviving
    // encoders, so losing those encoders next frame does not reveal an offset.
    left.connected[0] = false;
    left.position[1] = left.position[2] = 10010;
    (void)oneEstimatorCycle();
    left.connected[0] = true;
    left.position[0] = 50000;
    left.position[1] = left.position[2] = 10020;
    const auto rejoined = oneEstimatorCycle();
    require(std::abs(rejoined.left - beforeJitter.left - 20 * travelPerDegree) < 1e-10,
            "surviving motors retain travel while another encoder establishes its baseline");
    left.connected[1] = left.connected[2] = false;
    left.position[0] = 50010;
    const auto soleSurvivor = oneEstimatorCycle();
    require(soleSurvivor.leftValid &&
            std::abs(soleSurvivor.left - beforeJitter.left - 30 * travelPerDegree) < 1e-10,
            "reconnected motor carries the current side reference into single-encoder fallback");
    left.connected[1] = left.connected[2] = true;
    left.position[0] = 50020;
    left.position[1] = left.position[2] = 10030;
    (void)oneEstimatorCycle();
    left.position[0] = 50030;
    left.position[1] = left.position[2] = 10040;
    const auto fullyRejoined = oneEstimatorCycle();
    require(fullyRejoined.leftValid &&
            std::abs(fullyRejoined.left - beforeJitter.left - 50 * travelPerDegree) < 1e-10,
            "all recovered encoder totals remain aligned as motion continues");
    pros::test::pauseEstimator = false;

    const auto timingBeforePoseReset = chassis->diagnostics().timing;
    chassis->setPose(120, -340, 22);
    require(std::abs(chassis->getPose().x - 120) < .001, "setPose acknowledges copied request before returning");
    checkTimingPreserved(timingBeforePoseReset, chassis->diagnostics().timing);
    EstimatorConfig sensorFit;
    DynamicsConfig modelFit;
    sensorFit.trackWidth = modelFit.trackWidth = .31;
    require(chassis->applyConfiguration(sensorFit, modelFit), "stopped model update accepted");
    require(std::abs(chassis->getPose().y + 340) < .001, "calibration preserves pose prior");
    require(chassis->setGyroBias(radians(.1), square(radians(.01))), "stationary bias update accepted");
    require(!chassis->setGyroBias(100, -1), "invalid bias observation rejected");
    pros::delay(50);

    const auto token = chassis->beginCalibration();
    require(token != 0, "practice calibration lease starts with fresh sensors");
    require(chassis->calibrationVoltage(token, 8, -9), "calibration command accepted");
    require(left.voltage == 6000 && right.voltage == -6000, "calibration is clamped to six volts");
    require(!chassis->applyConfiguration(sensorFit, modelFit), "calibration cannot change model while motors leased");
    require(eventually([&] { return left.voltage == 0 && right.voltage == 0 &&
                                   !chassis->diagnostics().calibrationActive; }, 350),
            "independent estimator watchdog expires abandoned calibration lease");
    require(!chassis->calibrationVoltage(token, 1, -1), "expired calibration token cannot restart output");

    pros::test::connected = true;
    require(chassis->beginCalibration() == 0, "field connection prohibits calibration");
    pros::test::connected = false;
    const auto secondToken = chassis->beginCalibration();
    require(secondToken && chassis->calibrationVoltage(secondToken, 1, -1), "new lease starts after previous failure");
    pros::test::disabled = true;
    require(eventually([&] { return left.voltage == 0 && right.voltage == 0; }), "disable revokes calibration output");
    pros::test::disabled = false;
    require(!chassis->calibrationVoltage(secondToken, 1, -1), "re-enable does not resurrect old token");

    const auto thirdToken = chassis->beginCalibration();
    require(thirdToken != 0, "fresh calibration starts");
    require(chassis->endCalibration(thirdToken), "owner can release its calibration lease");
    pros::test::autonomous = true;
    const auto motion = chassis->startMoveToPoint(120, 800, {.maxSpeed = 400, .timeout = 2000});
    require(motion != 0 && eventually([&] { return left.voltage > 100 && right.voltage > 100; }),
            "autonomous executes after copied request");
    require(!chassis->endCalibration(thirdToken), "stale calibration exit cannot cancel autonomous");
    require(!chassis->calibrationVoltage(thirdToken, 0, 0), "stale calibration command is rejected");
    require(!chassis->stopPractice() && !chassis->beginCalibration(),
            "practice UI cannot claim or stop competition motion");
    require(chassis->busy(), "stale calibration callbacks preserve newer motion");
    chassis->cancel();
    require(chassis->waitUntilDone(motion).status == MotionStatus::cancelled,
            "cancellation fences late optimizer output");
    pros::delay(50);
    require(left.voltage == 0 && right.voltage == 0, "late optimizer cannot reapply voltage");

    pros::test::autonomous = false;
    pros::test::pauseCommands = true;
    std::atomic<bool> delayedStop{true};
    std::thread staleUi([&] { delayedStop = chassis->stopPractice(); });
    pros::delay(10);
    pros::test::autonomous = true;
    std::atomic<std::uint32_t> laterMotion{0};
    std::thread nextMode([&] { laterMotion = chassis->startMoveToPoint(120, 800, {.maxSpeed = 400}); });
    pros::delay(10);
    pros::test::pauseCommands = false;
    staleUi.join(); nextMode.join();
    require(!delayedStop && laterMotion != 0 && chassis->busy(),
            "queued practice stop cannot revoke a newer autonomous generation");
    chassis->cancel();
    pros::test::autonomous = false;

    // Mode ownership starts when a sequence is constructed, even before its
    // first movement or mechanism action.
    Sequence dormant(*chassis);
    pros::test::autonomous = true;
    const auto freshMotion = chassis->startMoveToPoint(120, 800, {.maxSpeed = 400});
    require(freshMotion != 0 && chassis->busy(), "new mode has its own active motion");
    bool staleActionRan = false;
    dormant.action([&] { staleActionRan = true; });
    require(!staleActionRan && dormant.result().status == MotionStatus::cancelled,
            "sequence cannot start its first action in a different competition mode");
    require(chassis->busy(), "old sequence does not cancel a newer mode's movement");
    chassis->cancel();
    pros::test::autonomous = false;

    Sequence superseded(*chassis);
    const auto replacement = chassis->startMoveToPoint(120, 800, {.maxSpeed = 400});
    superseded.setPose(999, 999, 0).moveToPoint(999, 999);
    require(superseded.result().status == MotionStatus::cancelled && chassis->busy() &&
            chassis->diagnostics().generation == replacement && std::abs(chassis->getPose().x - 120) < .1,
            "replaced sequence cannot reset pose or take over a newer command in the same mode");
    chassis->cancel();

    Sequence successful(*chassis);
    bool completedAction = false;
    successful.setPose(120, -340, 22).moveToPoint(120, -340).wait(10)
        .action([&] { completedAction = true; });
    require(successful.result().status == MotionStatus::settled && completedAction,
            "sequence retains ownership across its own pose, movement, wait and action");

    Sequence timeouts(*chassis);
    for (unsigned kind = 0; kind < 3; ++kind) {
        const MoveOptions shortTimeout{.maxSpeed = 400, .timeout = 120, .settleTime = 20};
        // Fake sensor counters remain fixed: each target is unreachable.
        if (kind == 0) timeouts.moveToPoint(600, 600, shortTimeout);
        else if (kind == 1) timeouts.moveToPose(600, 600, 90, shortTimeout);
        else timeouts.turnToHeading(90, shortTimeout);
        require(timeouts.result().status == MotionStatus::timedOut && !timeouts.result(),
                "timeout remains observable as an unsuccessful individual motion");
        require(!chassis->busy() && left.voltage == 0 && right.voltage == 0,
                "timed out movement stops its wheel outputs before returning");
        bool afterTimeout = false;
        const auto waitStart = pros::millis();
        timeouts.wait(30).action([&] { afterTimeout = true; });
        require(afterTimeout && pros::millis() - waitStart >= 30,
                "wait and mechanism actions execute after a movement timeout");
    }
    timeouts.moveToPoint(120, -340, {.timeout = 1000});
    require(timeouts.result().status == MotionStatus::settled,
            "the next movement acquires a fresh timeout and can settle after previous timeouts");

    Sequence timedThenCancelled(*chassis);
    timedThenCancelled.moveToPoint(600, 600, {.timeout = 120, .settleTime = 20});
    require(timedThenCancelled.result().status == MotionStatus::timedOut, "cancellation test first times out");
    chassis->cancel();
    bool cancelledAction = false;
    timedThenCancelled.action([&] { cancelledAction = true; }).moveToPoint(120, -340);
    require(!cancelledAction && timedThenCancelled.result().status == MotionStatus::cancelled && !chassis->busy(),
            "timeout recovery does not bypass an explicit cancellation");

    Sequence invalid(*chassis);
    bool invalidAction = false;
    invalid.moveToPoint(600, 600, {.timeout = 0}).action([&] { invalidAction = true; })
        .moveToPoint(120, -340);
    require(!invalidAction && invalid.result().status == MotionStatus::invalidRequest && !chassis->busy(),
            "invalid requests still block later autonomous commands");

    lateral.connected = false;
    require(eventually([&] { return !chassis->diagnostics().sensors.lateralValid; }), "lateral dropout is visible");
    Sequence lostLateral(*chassis);
    bool blindAction = false;
    lostLateral.moveToPoint(600, 600, {.timeout = 1000}).action([&] { blindAction = true; });
    require(lostLateral.result().status == MotionStatus::sensorFault && !blindAction && !chassis->busy() &&
            left.voltage == 0 && right.voltage == 0,
            "required lateral pod loss stops autonomous and cannot be bypassed as a timeout");
    lateral.connected = true;
    require(eventually([&] { return chassis->diagnostics().sensors.lateralValid; }), "lateral pod reconnects");
    pros::delay(30);

    const auto beforeModeChange = chassis->getPose();
    pros::test::pauseCommands = true;
    std::thread delayedPose([&] { chassis->setPose(777, 888, 90); });
    pros::delay(10);
    pros::test::autonomous = true;
    pros::test::pauseCommands = false;
    delayedPose.join();
    require(std::abs(chassis->getPose().x - beforeModeChange.x) < .1,
            "queued setPose from previous mode cannot reset current odometry");
    pros::test::autonomous = false;

    Sequence queuedSequence(*chassis);
    const auto beforeSequence = chassis->getPose();
    const auto enqueues = pros::test::commandEnqueues.load();
    pros::test::pauseCommands = true;
    std::thread sequencePose([&] { queuedSequence.setPose(555, 555, 0); });
    require(eventually([&] { return pros::test::commandEnqueues > enqueues; }),
            "sequence pose request is queued before changing mode");
    pros::test::autonomous = true;
    pros::test::pauseCommands = false;
    sequencePose.join();
    require(queuedSequence.result().status == MotionStatus::cancelled &&
            std::abs(chassis->getPose().x - beforeSequence.x) < .1,
            "sequence pose preserves its original mode and rejects without waiting for an unpublished handle");
    pros::test::autonomous = false;

    // An abandoned call must not later execute with a stale caller's pose.
    const auto beforeAbandon = chassis->getPose();
    pros::test::pauseCommands = true;
    std::thread abandoned([&] { chassis->setPose(999, 999, 0); });
    abandoned.join();
    pros::test::pauseCommands = false;
    pros::delay(40);
    require(std::abs(chassis->getPose().x - beforeAbandon.x) < .1,
            "expired copied request cannot mutate pose after caller gives up");

    // The callback thread leaves immediately: the queued payload must not refer
    // to its stack, and no persistent mutex may remain owned by that callback.
    std::thread callback([&] { chassis->arcade(70, 0); });
    callback.join();
    require(eventually([&] { return left.voltage == 43 * 12000 / 127 && right.voltage == 43 * 12000 / 127; }),
            "completed callback preserves the original default joystick curve in its copied request");
    pros::test::disabled = true;
    require(eventually([&] { return left.voltage == 0 && right.voltage == 0; }),
            "field transition remains responsive after callback exits");
    pros::test::disabled = false;
    chassis->cancel();

    // Check the public asynchronous API and resulting motor powers, including
    // unchanged defaults, independent axes, and normalization at full input.
    const auto checkArcade = [&](int throttle, int turn, ArcadeCurves curves,
                                 int leftPower, int rightPower, const char* message) {
        const auto previousGeneration = chassis->diagnostics().generation;
        chassis->arcade(throttle, turn, curves);
        require(eventually([&] {
            return chassis->diagnostics().generation != previousGeneration &&
                left.voltage == leftPower * 12000 / 127 &&
                right.voltage == rightPower * 12000 / 127;
        }), message);
    };
    checkArcade(64, 0, {}, 37, 37, "default forward curve retains legacy power");
    checkArcade(0, 64, {}, 28, -28, "default turn curve retains legacy power and steering scale");
    checkArcade(64, 0, {0, 1}, 64, 64, "linear forward ignores the selected turn curve");
    checkArcade(64, 0, {1, 0}, 16, 16, "cubic forward softens partial stick input");
    checkArcade(0, 64, {1, 0}, 48, -48, "linear turn ignores the selected forward curve");
    checkArcade(0, 64, {0, 1}, 12, -12, "cubic turn softens partial stick input");
    checkArcade(64, 64, {0, 1}, 76, 51, "combined input independently shapes forward and turn");
    checkArcade(64, 64, {1, 0}, 64, -31, "swapping the two curves changes both mixed outputs");
    checkArcade(-64, -64, {1, 0}, -64, 31, "negative joystick input preserves curve symmetry");
    checkArcade(127, 0, {1, 0}, 127, 127, "cubic forward preserves full stick power");
    checkArcade(-127, 0, {0, 1}, -127, -127, "linear forward preserves full reverse power");
    checkArcade(0, 127, {0, 1}, 95, -95, "cubic turn preserves full steering power");
    checkArcade(0, -127, {1, 0}, -95, 95, "linear turn preserves full reverse steering power");
    checkArcade(0, 0, {0, 1}, 0, 0, "centered sticks stop for configurable curves");
    checkArcade(3, -3, {0, 0}, 0, 0, "both sides of the joystick deadband remain stopped");
    checkArcade(4, 0, {0, 0}, 4, 4, "forward input at the existing deadband boundary is retained");
    checkArcade(0, -4, {0, 0}, -3, 3, "turn input at the existing deadband boundary is retained");
    checkArcade(127, 127, {0, 1}, 127, 18, "full forward and turn normalize to bounded motor powers");
    checkArcade(127, -127, {1, 0}, 18, 127, "opposite full steering retains the normalized mix");
    checkArcade(64, 64, {-1, 2}, 76, 51, "curve coefficients clamp to the linear and cubic endpoints");
    checkArcade(64, 64, {2, -1}, 64, -31, "both axis coefficients clamp independently");
    checkArcade(64, 64, {std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::infinity()}, 66, 9,
                "nonfinite curve coefficients fall back to the original response");
    chassis->cancel();

    std::thread producer([&] {
        for (int i = 0; i < 80; ++i) { chassis->arcade(i % 2 ? 40 : -40, 0); pros::delay(1); }
    });
    for (unsigned i = 0; i < 20; ++i) {
        (void)chassis->diagnostics(); chassis->cancel(); pros::delay(2);
    }
    producer.join(); chassis->cancel();
    require(eventually([&] { return left.voltage == 0 && right.voltage == 0; }),
            "concurrent cancellation and snapshots do not strand command processing");

    imu.acceleration[1] = -3.9;
    imu.rate[0] = 960;
    require(eventually([&] {
        const auto snapshot = chassis->diagnostics();
        return snapshot.accelerationNearLimit > 0 && snapshot.gyroNearLimit > 0;
    }), "axis-near-limit diagnostics include all mounted axes");
    imu.acceleration[1] = 0; imu.rate[0] = 0;
    pros::delay(30);
    const auto peaks = chassis->diagnostics();
    require(peaks.peakAccelerationG >= 3.9 && peaks.peakGyroRateDps >= 960,
            "transient clipping-proximity peaks remain observable");

    // Inject a known clock disagreement while the worker is between samples.
    // This does not rely on a host sleep lasting an exact wall-clock duration.
    pros::test::pauseEstimator = true;
    require(eventually([] { return pros::test::estimatorPaused.load(); }),
            "clock disagreement is injected outside the measured work and clock-read brackets");
    const auto beforeClockShift = chassis->diagnostics().timing;
    constexpr std::uint64_t clockShiftMicros = 500000;
    pros::test::highResolutionOffsetMicros += clockShiftMicros;
    (void)oneEstimatorCycle();
    const auto shiftedTiming = chassis->diagnostics().timing;
    require(shiftedTiming.longIntervals > beforeClockShift.longIntervals &&
            shiftedTiming.lastIntervalMs >= 500 && shiftedTiming.lastWaitMs >= 500,
            "a high-resolution-only clock advance remains visible in the full interval");
    checkTimingStages(shiftedTiming);
    require(std::abs(shiftedTiming.lastIntervalMs - shiftedTiming.lastRtosMs -
                     static_cast<double>(clockShiftMicros) / 1000) <= shiftedTiming.lastClockReadMs + 2,
            "diagnostics retain the independent RTOS interval instead of masking clock disagreement");
    pros::test::pauseEstimator = false;

    // Capture begins after calibration, without replacing the snapshot queue.
    // Timestamp callbacks are entirely absent until the explicit start.
    pros::test::pauseEstimator = true;
    require(eventually([] { return pros::test::estimatorPaused.load(); }), "trace start pause acknowledged");
    require(!chassis->popOdometryTrace(trace) && chassis->droppedOdometryTrace() == 0 &&
            packetClockCalls[0] == 0 && packetClockCalls[1] == 0 && packetClockCalls[2] == 0,
            "inactive trace neither reads packet clocks nor captures startup samples");
    require(chassis->startOdometryTrace(), "configured trace starts after sensor workers");
    imu.rate[0] = 1.25; imu.rate[1] = -2.5; imu.rate[2] = 3.75;
    (void)oneEstimatorCycle();
    const auto captured = chassis->diagnostics();
    require(chassis->popOdometryTrace(trace) && trace.sequence == 1 && trace.dropped == 0,
            "first trace record starts a contiguous sequence without initial losses");
    require(trace.timestamp == captured.sensors.timestamp &&
            trace.counters == std::array<double, 5>{captured.sensors.forward, captured.sensors.lateral,
                captured.sensors.left, captured.sensors.right, captured.sensors.gyro} &&
            trace.pose.x == captured.estimate.state.pose.x && trace.pose.y == captured.estimate.state.pose.y &&
            trace.pose.theta == captured.estimate.state.pose.theta && trace.gyroBias == captured.estimate.gyroBias &&
            trace.rawGyroDps == captured.sensors.rawGyroDps && trace.validMask == 63 &&
            trace.rejectedMask == captured.estimate.rejectedIncrementMask &&
            trace.headingSource == captured.estimate.headingSource && trace.stationary == captured.estimate.stationary,
            "trace preserves matching raw counters, pose, bias, flags and native gyro in documented SI units");
    for (std::size_t source = 0; source < 3; ++source)
        require(packetClockCalls[source] == 2 && trace.packetBeforeMs[source] == source * 100000u + 1 &&
                trace.packetAfterMs[source] == source * 100000u + 2,
                "each primary getter is bracketed once without retrying changed packet clocks");
    const auto originalTraceEpoch = trace.epoch;
    require(!chassis->popOdometryTrace(trace), "empty trace FIFO returns immediately");
    (void)oneEstimatorCycle();
    (void)oneEstimatorCycle();
    require(chassis->popOdometryTrace(trace) && trace.sequence == 2,
            "FIFO retains the older of two unconsumed samples");
    const auto secondTraceTimestamp = trace.timestamp;
    require(chassis->popOdometryTrace(trace) && trace.sequence == 3 && trace.timestamp > secondTraceTimestamp &&
            !chassis->popOdometryTrace(trace), "FIFO returns the next sample exactly once");
    require(chassis->startOdometryTrace(), "starting active capture is idempotent");
    chassis->setPose(10, 20, 30);
    (void)oneEstimatorCycle();
    require(chassis->popOdometryTrace(trace) && trace.sequence == 4 && trace.dropped == 0 &&
            trace.epoch == originalTraceEpoch + 1 && std::abs(trace.pose.x - .010) < 1e-12 &&
            std::abs(trace.pose.y - .020) < 1e-12 && std::abs(trace.pose.theta - radians(30)) < 1e-12,
            "pose reset changes trace epoch and pose while retaining sequence and loss history");
    const auto resetTraceEpoch = trace.epoch;

    // A stopped consumer must never stall acquisition or replace queued data.
    // Wait for the actual fixed-capacity queue to fill, rather than asserting
    // any host scheduling duration or changing the production queue capacity.
    pros::test::pauseEstimator = false;
    require(eventually([&] { return chassis->droppedOdometryTrace() > 0; }, 10000),
            "full trace queue reports losses without requiring a consumer");
    pros::test::pauseEstimator = true;
    require(eventually([] { return pros::test::estimatorPaused.load(); }), "full trace queue pause acknowledged");
    const auto beforeFullMove = chassis->diagnostics();
    const auto lossesBeforeMove = chassis->droppedOdometryTrace();
    forward.position += 1000;
    (void)oneEstimatorCycle();
    const auto afterFullMove = chassis->diagnostics();
    require(chassis->droppedOdometryTrace() == lossesBeforeMove + 1 &&
            afterFullMove.sensors.timestamp > beforeFullMove.sensors.timestamp &&
            std::hypot(afterFullMove.estimate.odometry.forwardX - beforeFullMove.estimate.odometry.forwardX,
                       afterFullMove.estimate.odometry.forwardY - beforeFullMove.estimate.odometry.forwardY) > .001,
            "estimation integrates real movement even while trace output remains full");
    std::uint32_t lastQueuedSequence = 4, queuedRecords = 0;
    double lastQueuedTimestamp = 0;
    while (chassis->popOdometryTrace(trace)) {
        require(trace.sequence == lastQueuedSequence + 1 && trace.timestamp > lastQueuedTimestamp &&
                trace.epoch == resetTraceEpoch && trace.dropped == 0 &&
                trace.counters[0] == beforeFullMove.sensors.forward,
                "overflow preserves FIFO order and copied payloads instead of overwriting oldest samples");
        lastQueuedSequence = trace.sequence;
        lastQueuedTimestamp = trace.timestamp;
        ++queuedRecords;
    }
    require(queuedRecords > 1, "trace queue buffers a sequence of unconsumed samples");
    const auto totalTraceLosses = chassis->droppedOdometryTrace();
    (void)oneEstimatorCycle();
    require(chassis->popOdometryTrace(trace) && trace.sequence == lastQueuedSequence + totalTraceLosses + 1 &&
            trace.dropped == totalTraceLosses && trace.counters[0] == afterFullMove.sensors.forward,
            "capture resumes with an exact sequence gap and cumulative dropped count");
    const auto resumedSequence = trace.sequence;
    chassis->setPose(0, 0, 0);
    (void)oneEstimatorCycle();
    require(chassis->popOdometryTrace(trace) && trace.sequence == resumedSequence + 1 &&
            trace.epoch == resetTraceEpoch + 1 && trace.dropped == totalTraceLosses &&
            chassis->droppedOdometryTrace() == totalTraceLosses,
            "later pose reset does not hide trace sequence gaps or final loss count");

    // Runtime configuration must remove a previously accepted local bias and
    // carry the native IMU policy through the request queue and trace output.
    const auto beforeNativeEpoch = trace.epoch;
    require(chassis->setGyroBias(radians(.1), square(radians(.01))) &&
            chassis->diagnostics().estimate.gyroBias > 0,
            "legacy mode has a nonzero accepted bias before switching policy");
    auto nativeConfig = sensorFit;
    nativeConfig.nativeImuHeading = true;
    nativeConfig.stationaryEncoderSpan = .010 * millimeter;
    const auto beforeNativePose = chassis->diagnostics().estimate.state.pose;
    require(chassis->applyConfiguration(nativeConfig, modelFit),
            "native heading policy crosses the stopped configuration request queue");
    const auto configuredNative = chassis->diagnostics().estimate;
    require(configuredNative.state.pose.x == beforeNativePose.x &&
            configuredNative.state.pose.y == beforeNativePose.y &&
            configuredNative.state.pose.theta == beforeNativePose.theta &&
            configuredNative.gyroBias == 0 && configuredNative.gyroBiasStd == 0,
            "native reconfiguration preserves pose and removes the effective local bias");
    require(!chassis->setGyroBias(radians(.2), square(radians(.01))),
            "native policy rejects a later manual bias through the public adapter API");
    (void)oneEstimatorCycle();
    require(chassis->popOdometryTrace(trace) && trace.epoch == beforeNativeEpoch + 1 &&
            trace.gyroBias == 0 && trace.dropped == totalTraceLosses,
            "native policy starts a new trace epoch without losing capture history");
    pros::test::pauseEstimator = false;
    require(eventually([&] { return chassis->diagnostics().estimate.stationary; }, 1000),
            "native policy still reports stationary diagnostics at rest");
    pros::test::pauseEstimator = true;
    require(eventually([] { return pros::test::estimatorPaused.load(); }),
            "native heading check waits between acquisitions");
    while (chassis->popOdometryTrace(trace)) {}
    const auto beforeNativeIncrement = chassis->diagnostics().estimate;
    imu.rotation += .005;
    (void)oneEstimatorCycle();
    const auto afterNativeIncrement = chassis->diagnostics().estimate;
    require(afterNativeIncrement.stationary && afterNativeIncrement.gyroBias == 0 &&
            afterNativeIncrement.headingSource == HeadingSource::imu &&
            std::abs(afterNativeIncrement.state.pose.theta - beforeNativeIncrement.state.pose.theta -
                     radians(.005)) < 1e-12,
            "native IMU increment changes heading even while wheels remain stationary");
    require(chassis->popOdometryTrace(trace) && trace.epoch == beforeNativeEpoch + 1 &&
            trace.stationary && trace.headingSource == HeadingSource::imu && trace.gyroBias == 0 &&
            trace.pose.theta == afterNativeIncrement.state.pose.theta &&
            trace.counters[4] == chassis->diagnostics().sensors.gyro,
            "trace records native heading and unmodified IMU counter from the same quiet sample");
    imu.rate[0] = imu.rate[1] = imu.rate[2] = 0;
    pros::test::pauseEstimator = false;

    // An optional queue allocation failure cannot suppress the main workers.
    auto* traceFailureLeft = new pros::MotorGroup({1, 2, 3});
    auto* traceFailureRight = new pros::MotorGroup({4, 5, 6});
    auto* traceFailureImu = new pros::Imu;
    HardwareConfig traceFailureConfig;
    traceFailureConfig.captureOdometryTrace = true;
    auto* traceFailure = new Chassis({*traceFailureLeft, *traceFailureRight, traceFailureImu},
                                    traceFailureConfig, {}, {});
    pros::test::failQueueCreationOfSize = sizeof(OdometryTrace);
    traceFailure->begin();
    pros::test::failQueueCreationOfSize = 0;
    require(!traceFailure->startOdometryTrace() && !traceFailure->popOdometryTrace(trace) &&
            traceFailure->droppedOdometryTrace() == 0 &&
            eventually([&] { return traceFailure->diagnostics().sensors.timestamp > 0; }),
            "unavailable optional capture leaves normal sensor workers running");

    const auto invalidGeometry = [](HardwareConfig config) {
        // Workers own these test devices until the process-wide stop below.
        auto* badLeft = new pros::MotorGroup({1, 2, 3});
        auto* badRight = new pros::MotorGroup({4, 5, 6});
        auto* badImu = new pros::Imu;
        auto* badForward = new pros::Rotation;
        auto* badLateral = new pros::Rotation;
        config.useRawGyroRate = true;
        auto* invalid = new Chassis({*badLeft, *badRight, badImu, badForward, badLateral, {}}, config, {}, {});
        invalid->begin();
        OdometryTrace disabledTrace;
        require(!invalid->startOdometryTrace() && !invalid->popOdometryTrace(disabledTrace) &&
                invalid->droppedOdometryTrace() == 0, "default configuration leaves trace disabled");
        require(eventually([&] { return invalid->diagnostics().sensors.timestamp > 0; }), "invalid-geometry workers start");
        const double firstSample = invalid->diagnostics().sensors.timestamp;
        require(eventually([&] {
            const auto snapshot = invalid->diagnostics();
            return snapshot.sensors.timestamp - firstSample >= .18 &&
                !snapshot.sensors.forwardValid && !snapshot.sensors.lateralValid &&
                !snapshot.sensors.leftValid && !snapshot.sensors.rightValid &&
                !snapshot.sensors.gyroValid && !snapshot.sensors.gyroRateValid &&
                snapshot.estimate.health == Health::lost;
        }, 500), "invalid hardware geometry cannot report healthy stationary localization");
    };
    HardwareConfig zeroGeometry;
    zeroGeometry.forwardWheelDiameter = 0;
    zeroGeometry.lateralWheelDiameter = -1;
    zeroGeometry.driveWheelDiameter = 0;
    zeroGeometry.gyroSign = 0;
    invalidGeometry(zeroGeometry);
    HardwareConfig nonfiniteGeometry;
    nonfiniteGeometry.forwardWheelDiameter = std::numeric_limits<double>::quiet_NaN();
    nonfiniteGeometry.lateralWheelDiameter = std::numeric_limits<double>::infinity();
    nonfiniteGeometry.driveWheelPerMotor = -1;
    nonfiniteGeometry.gyroSign = std::numeric_limits<double>::quiet_NaN();
    invalidGeometry(nonfiniteGeometry);

    // Mixed cartridges: excluded encoders must not leak into fallback when a
    // selected encoder disconnects (a three-way median only hides that bug).
    auto* mixedLeft = new pros::MotorGroup({5, -4, 6});
    auto* mixedRight = new pros::MotorGroup({-7, 9, -10});
    mixedLeft->connected[2] = mixedRight->connected[2] = false;
    HardwareConfig mixedConfig;
    mixedConfig.leftOdometryMask = mixedConfig.rightOdometryMask = 0b101;
    auto* mixed = new Chassis({*mixedLeft, *mixedRight}, mixedConfig, {}, {});
    mixed->begin();
    require(eventually([&] { return mixed->diagnostics().sensors.leftValid &&
                                   mixed->diagnostics().sensors.rightValid; }),
            "mixed drivetrain starts from the selected surviving blue encoders");
    mixedLeft->position[0] = mixedRight->position[0] = 90;
    mixedLeft->position[1] = mixedRight->position[1] = 30;
    const double mixedTravel = 90.0 / 360 * pi * mixedConfig.driveWheelDiameter * inch * mixedConfig.driveWheelPerMotor;
    require(eventually([&] {
        const auto s = mixed->diagnostics().sensors;
        return std::abs(s.left - mixedTravel) < 1e-10 && std::abs(s.right - mixedTravel) < 1e-10;
    }), "excluded green encoders cannot dilute single-blue fallback distance");
    require(mixedLeft->gearingWrites[1] == 0 && mixedRight->gearingWrites[1] == 0,
            "odometry does not overwrite gearing on excluded motors");
    mixedLeft->connected[0] = mixedRight->connected[0] = false;
    require(eventually([&] { return !mixed->diagnostics().sensors.leftValid &&
                                   !mixed->diagnostics().sensors.rightValid; }),
            "an excluded encoder cannot silently replace all missing selected encoders");
    mixedLeft->connected[2] = mixedRight->connected[2] = true;
    mixedLeft->position[2] = mixedRight->position[2] = 2000;
    require(eventually([&] { return mixed->diagnostics().sensors.leftValid &&
                                   mixed->diagnostics().sensors.rightValid; }),
            "selected encoder reconnect restores drive evidence");
    mixedLeft->position[2] = mixedRight->position[2] = 2090;
    require(eventually([&] {
        const auto s = mixed->diagnostics().sensors;
        return std::abs(s.left - 2*mixedTravel) < 1e-10 && std::abs(s.right - 2*mixedTravel) < 1e-10;
    }), "selected reconnect keeps the distance baseline without a mixed-cartridge jump");
    // The robot's 20 Hz acquisition must also support an immediate reset/move.
    auto* slowLeft = new pros::MotorGroup({1, 2, 3});
    auto* slowRight = new pros::MotorGroup({4, 5, 6});
    auto* slowImu = new pros::Imu;
    auto* slowForward = new pros::Rotation;
    auto* slowLateral = new pros::Rotation;
    HardwareConfig slowHardware;
    slowHardware.odometryPeriodMs = 50;
    slowHardware.captureOdometryTrace = true;
    EstimatorConfig slowEstimator;
    slowEstimator.nativeImuHeading = true;
    slowEstimator.stationaryEncoderSpan = .010 * millimeter;
    slowEstimator.stationaryMaxInterval = .075;
    auto* slow = new Chassis({*slowLeft, *slowRight, slowImu, slowForward, slowLateral, {}},
                             slowHardware, slowEstimator, {});
    slow->begin();
    require(slow->startOdometryTrace(), "20 Hz acquisition trace starts");
    double firstSlowTimestamp = 0, lastSlowTimestamp = 0;
    unsigned slowFrames = 0;
    require(eventually([&] {
        OdometryTrace record;
        while (slow->popOdometryTrace(record)) {
            if (!slowFrames) firstSlowTimestamp = record.timestamp;
            lastSlowTimestamp = record.timestamp;
            ++slowFrames;
        }
        return slowFrames >= 8;
    }, 1500), "20 Hz task publishes fresh sensor frames");
    require((lastSlowTimestamp - firstSlowTimestamp) / (slowFrames - 1) >= .045,
            "sensor timestamps confirm the task no longer acquires at 100 Hz");
    const auto slowTiming = slow->diagnostics().timing;
    require(slowTiming.periodMs == 50 && slowTiming.longIntervalLimitMs == 75,
            "timing diagnostics follow the selected acquisition cadence");
    for (unsigned repeat = 0; repeat < 3; ++repeat) {
        Sequence resetAndMove(*slow);
        resetAndMove.setPose(0, 0, 0).moveToPoint(0, 0, {.timeout = 1000, .settleTime = 20});
        require(resetAndMove.result().status == MotionStatus::settled,
                "setPose then move waits for 20 Hz baselines without a sensor or solver fault");
    }
    // Host threads do not simulate V5 real-time performance. Verify the 1 ms
    // setting is accepted and workers/reset/move operate at that setting.
    auto* fastLeft = new pros::MotorGroup({1, 2, 3});
    auto* fastRight = new pros::MotorGroup({4, 5, 6});
    auto* fastImu = new pros::Imu;
    auto* fastForward = new pros::Rotation;
    auto* fastLateral = new pros::Rotation;
    HardwareConfig fastHardware;
    fastHardware.odometryPeriodMs = 1;
    fastHardware.leftOdometryMask = fastHardware.rightOdometryMask = 0;
    EstimatorConfig fastEstimator = slowEstimator;
    fastEstimator.maxSensorHold = .010;
    fastEstimator.stationaryMaxInterval = .050;
    auto* fast = new Chassis({*fastLeft, *fastRight, fastImu, fastForward, fastLateral, {}},
                             fastHardware, fastEstimator, {});
    fast->begin();
    require(eventually([&] { return fast->diagnostics().estimate.health != Health::lost; }),
            "1 ms acquisition workers produce usable estimates");
    require(fast->diagnostics().timing.periodMs == 1,
            "1 ms polling is not silently clamped to the old 5 ms minimum");
    Sequence fastResetAndMove(*fast);
    fastResetAndMove.setPose(0, 0, 0).moveToPoint(0, 0, {.timeout = 1000, .settleTime = 20});
    require(fastResetAndMove.result().status == MotionStatus::settled,
            "reset then move works with 1 ms polling");
    require(fast->beginCalibration() == 0, "legacy drive geometry still requires drive encoders");
    const auto podLease = fast->beginCalibration(CalibrationSensors::trackingPods);
    require(podLease && fast->calibrationVoltage(podLease, 6, -6) &&
            fastLeft->voltage == 4000 && fastRight->voltage == -4000,
            "pod-only calibration works without drive encoders and caps both sides at four volts");
    require(fast->endCalibration(podLease) && fastLeft->voltage == 0 && fastRight->voltage == 0,
            "pod calibration release stops both sides");
    const auto stalePodLease = fast->beginCalibration(CalibrationSensors::trackingPods);
    require(stalePodLease && fast->calibrationVoltage(stalePodLease, 3, -3), "pod watchdog fixture starts");
    require(eventually([&] { return !fast->diagnostics().calibrationActive &&
        fastLeft->voltage == 0 && fastRight->voltage == 0; }, 300) &&
        !fast->calibrationVoltage(stalePodLease, 3, -3),
        "pod lease expires without a command renewal and cannot resume from a stale token");
    // The encoder-free diagnostic mode must not even read the drive counters.
    // Deliberately contradictory values cannot enter pose, slip or quiet.
    for (unsigned i = 0; i < 3; ++i) {
        fastLeft->position[i] = 1000 * (i + 1);
        fastRight->position[i] = -1000 * (i + 1);
    }
    const double beforeMotorChanges = fast->diagnostics().sensors.timestamp;
    require(eventually([&] { return fast->diagnostics().sensors.timestamp > beforeMotorChanges + .025; }),
            "encoder-free acquisition continues with changing drive counters");
    auto podsOnly = fast->diagnostics();
    require(!podsOnly.sensors.leftValid && !podsOnly.sensors.rightValid &&
            std::hypot(podsOnly.estimate.state.pose.x, podsOnly.estimate.state.pose.y) < 1e-10 &&
            std::abs(podsOnly.estimate.state.pose.theta) < 1e-10 && podsOnly.estimate.slip == 0 &&
            !podsOnly.estimate.stationary,
            "disabled motor encoders cannot affect position, heading, slip or certify quiet");
    fastForward->position = 2000;
    fastLateral->position = 1000;
    const double expectedY = 2000.0 / 36000 * pi * fastHardware.forwardWheelDiameter * inch;
    const double expectedX = 1000.0 / 36000 * pi * fastHardware.lateralWheelDiameter * inch;
    require(eventually([&] {
        const auto pose = fast->diagnostics().estimate.state.pose;
        return std::abs(pose.x - expectedX) < 1e-10 && std::abs(pose.y - expectedY) < 1e-10;
    }), "both tracking pods still contribute with every motor encoder disabled");
    const auto lostPodLease = fast->beginCalibration(CalibrationSensors::trackingPods);
    require(lostPodLease && fast->calibrationVoltage(lostPodLease, 3, -3), "pod lease starts before sensor loss");
    fastImu->connected = fastForward->connected = false;
    require(eventually([&] {
        const auto e = fast->diagnostics().estimate;
        return e.health == Health::lost && e.headingSource == HeadingSource::unavailable;
    }, 500), "losing pod/IMU cannot activate a hidden motor-encoder fallback");
    require(!fast->diagnostics().calibrationActive && fastLeft->voltage == 0 && fastRight->voltage == 0 &&
            !fast->calibrationVoltage(lostPodLease, 3, -3),
            "pod sensor loss revokes output and cannot restart with the old token");
    for (unsigned i = 0; i < 3; ++i)
        require(fastLeft->positionReads[i] == 0 && fastRight->positionReads[i] == 0,
                "zero odometry masks suppress all drive-position getter calls");
    pros::test::stop();
    std::cout << "Chassis adapter lifecycle, calibration lease, reconnect, timing, trace and diagnostic tests passed.\n";
}
