#include "nexus/calibration_routine.hpp"
#include "nexus/automatic_calibration.hpp"

#include <cstdio>
#include <cstdlib>
#include <limits>

namespace {
using namespace nexus;
unsigned checks = 0;
void check(bool passed, const char* message) {
    ++checks;
    if (!passed) { std::fprintf(stderr, "FAIL: %s\n", message); std::exit(1); }
}
void near(double value, double expected, double tolerance, const char* message) {
    check(std::isfinite(value) && std::abs(value - expected) <= tolerance, message);
}
void stopped(const CalibrationRoutineOutput& output) {
    near(output.voltage.left, 0, 0, "left motor stopped");
    near(output.voltage.right, 0, 0, "right motor stopped");
}
struct Fixture {
    GeometryRoutine routine;
    SensorSample sample{};
    double gyroScale = 1, gyroBias = 0;
    Fixture() {
        sample.leftValid = sample.rightValid = sample.gyroValid = true;
    }
    void start() { check(routine.start(sample, sample.timestamp, gyroScale, gyroBias), "valid start accepted"); }
    CalibrationRoutineOutput step(double omega = 0, double speed = 0, double dt = 0.01, bool permit = true) {
        sample.timestamp += dt;
        sample.gyro += (omega + gyroBias) * dt / gyroScale;
        sample.left += (speed + 0.284 * omega / 2) * dt;
        sample.right += (speed - 0.284 * omega / 2) * dt;
        return routine.update(sample, sample.timestamp, permit);
    }
    void startTurning() {
        start();
        for (int i = 0; i < 120 && routine.output().stage != CalibrationStage::clockwise; ++i) step();
        check(routine.output().stage == CalibrationStage::clockwise, "stillness starts first turn");
    }
};

void completeBothDirections() {
    Fixture f;
    f.gyroScale = 1.015;
    f.gyroBias = radians(0.18);
    f.sample.gyro = -20 * pi;
    f.start();
    stopped(f.routine.output());
    double lastLeft = 0;
    bool clockwise = false, counterclockwise = false, middleSettling = false;
    unsigned completedAt = 0;
    for (unsigned i = 0; i < 4000; ++i) {
        const auto previous = f.routine.output();
        // The unregulated plant would reach approximately 258 deg/s at 6 V.
        const double omega = 0.75 * previous.voltage.left;
        const auto output = f.step(omega);
        check(std::isfinite(output.voltage.left) && std::isfinite(output.voltage.right), "finite commands");
        check(std::abs(output.voltage.left) <= 6 && std::abs(output.voltage.right) <= 6, "commands bounded at 6 V");
        near(output.voltage.left, -output.voltage.right, 1e-12, "pure turn uses opposite wheel voltages");
        check(output.stage != CalibrationStage::aborted, "normal valid trajectory never aborts");
        if (std::abs(output.voltage.left) > std::abs(lastLeft))
            check(std::abs(output.voltage.left - lastLeft) <= 6.0 / 30 + 1e-9, "turn voltage rise is limited to 20 V/s");
        lastLeft = output.voltage.left;
        clockwise = clockwise || output.stage == CalibrationStage::clockwise;
        counterclockwise = counterclockwise || output.stage == CalibrationStage::counterclockwise;
        middleSettling = middleSettling || output.stage == CalibrationStage::settlingCCW;
        if (output.stage == CalibrationStage::complete) { completedAt = i; break; }
    }
    check(completedAt > 0 && clockwise && counterclockwise && middleSettling, "routine completes both directions and settling");
    const auto completed = f.routine.output();
    check(completed.clockwiseRadians >= 2 * pi && completed.counterclockwiseRadians >= 2 * pi,
          "each fitted direction has at least one corrected full revolution");
    check(completed.clockwiseRadians < 2 * pi + 0.1 && completed.counterclockwiseRadians < 2 * pi + 0.1,
          "continuous gyro count, non-unit scale, and bias preserve correct progress");
    stopped(completed);
    for (int i = 0; i < 300; ++i) {
        const auto output = f.step(0, 0, 0.01, i % 2 == 0);
        check(output.stage == CalibrationStage::complete, "complete state never restarts when permission changes");
        stopped(output);
    }
}

void regulatedTorqueAndSpeed() {
    // The fast drivetrain would exceed 360 deg/s at fixed 6 V. The loaded
    // drivetrain needs over 3 V to break static friction but less once moving.
    for (bool loaded : {false, true}) {
        Fixture f;
        f.sample.forwardValid = f.sample.lateralValid = true;
        nexus::calibration::RotationCalibration capture;
        capture.reset({});
        check(capture.add(f.sample), "regulated turn baseline captured");
        f.start();
        double omega = 0, peakSpeed = 0, peakVoltage = 0;
        bool complete = false;
        unsigned cruiseSamples = 0;
        for (unsigned i = 0; i < 4000; ++i) {
            const double voltage = f.routine.output().voltage.left;
            const double magnitude = std::abs(voltage);
            peakVoltage = std::max(peakVoltage, magnitude);
            const bool stuck = loaded && std::abs(omega) < 0.02 && magnitude < 3.5;
            const double kineticFriction = loaded ? 1.2 : 0;
            const double desired = stuck ? 0 : std::copysign(
                2.5 * std::max(0.0, magnitude - kineticFriction), voltage);
            omega += (desired - omega) * 0.01 / 0.10;
            peakSpeed = std::max(peakSpeed, std::abs(omega));
            const auto output = f.step(omega);
            f.sample.forward = -0.024 * f.sample.gyro;
            f.sample.lateral = 0.058 * f.sample.gyro;
            check(capture.add(f.sample), "speed-regulated sensor stream accepted by geometry fit");
            check(output.stage != CalibrationStage::aborted, "fast and high-friction drivetrains do not abort");
            if (std::abs(std::abs(omega) - GeometryRoutine::targetAngularSpeed) < radians(15)) ++cruiseSamples;
            if (output.stage == CalibrationStage::complete) { complete = true; break; }
        }
        check(complete, "regulated drivetrain completes both full turns");
        check(peakSpeed < GeometryRoutine::maximumAngularSpeed, "regulation avoids overspeed abort");
        check(cruiseSamples > 100, "both drivetrains cruise near target angular speed");
        if (loaded) check(peakVoltage > 3.5, "startup torque overcomes friction that stalled the old command");
        const auto result = capture.finish();
        check(result.accepted, "regulated turns produce accepted pod calibration");
        near(result.forwardOffset.value, 0.024, 1e-6, "regulated turn preserves forward pod offset");
        near(result.lateralOffset.value, 0.058, 1e-6, "regulated turn preserves lateral pod offset");
        near(result.trackWidth.value, 0.284, 1e-6, "regulated turn preserves track width");
        stopped(f.routine.output());
    }
}

void deadmanAndSensorFailures() {
    Fixture deadman;
    deadman.startTurning();
    deadman.step(0.1, 0, 0.05);
    const auto released = deadman.step(0.1, 0, 0.01, false);
    check(released.failure == CalibrationFailure::notPermitted, "deadman/mode/radio revoke aborts immediately");
    stopped(released);
    for (int i = 0; i < 50; ++i) {
        auto output = deadman.step();
        check(output.stage == CalibrationStage::aborted, "restored permit cannot resume an aborted routine");
        stopped(output);
    }

    Fixture duplicate;
    duplicate.startTurning();
    duplicate.step(0.1, 0, 0.05);
    const auto previous = duplicate.routine.output();
    check(previous.voltage.left > 0, "duplicate test begins with a nonzero command");
    auto output = duplicate.routine.update(duplicate.sample, duplicate.sample.timestamp + 0.02, true);
    near(output.voltage.left, previous.voltage.left, 0, "fresh duplicate snapshot retains last command");
    near(output.clockwiseRadians, previous.clockwiseRadians, 0, "duplicate snapshot cannot add progress");
    output = duplicate.routine.update(duplicate.sample, duplicate.sample.timestamp + 0.101, true);
    check(output.failure == CalibrationFailure::staleSensors, "duplicate snapshot cannot hide stale data");
    stopped(output);

    Fixture gap;
    gap.startTurning();
    output = gap.step(0.1, 0, 0.11);
    check(output.failure == CalibrationFailure::staleSensors, "fresh snapshot after an acquisition gap also aborts");
    stopped(output);

    Fixture gyro;
    gyro.startTurning();
    gyro.sample.gyroValid = false;
    output = gyro.step();
    check(output.failure == CalibrationFailure::invalidGyro, "gyro disconnect aborts immediately");
    stopped(output);
    Fixture drive;
    drive.startTurning();
    drive.sample.rightValid = false;
    output = drive.step();
    check(output.failure == CalibrationFailure::invalidDrive, "drive encoder disconnect aborts immediately");
    stopped(output);
}

void motionGuards() {
    Fixture wrong;
    wrong.startTurning();
    for (int i = 0; i < 50 && wrong.routine.output().stage != CalibrationStage::aborted; ++i) wrong.step(-1);
    check(wrong.routine.output().failure == CalibrationFailure::wrongDirection, "reverse rotation sign aborts after eight degrees");
    stopped(wrong.routine.output());

    Fixture reverses;
    reverses.startTurning();
    for (int i = 0; i < 100; ++i) reverses.step(1);
    for (int i = 0; i < 50 && reverses.routine.output().stage != CalibrationStage::aborted; ++i) reverses.step(-1);
    check(reverses.routine.output().failure == CalibrationFailure::wrongDirection,
          "wrong direction after initial correct progress is caught within eight reverse degrees");

    Fixture translation;
    translation.startTurning();
    for (int i = 0; i < 300 && translation.routine.output().stage != CalibrationStage::aborted; ++i)
        translation.step(0, 0.15);
    check(translation.routine.output().failure == CalibrationFailure::translation, "straight-driving misconfiguration aborts within 200 mm");
    stopped(translation.routine.output());

    Fixture alternating;
    alternating.startTurning();
    for (int i = 0; i < 300 && alternating.routine.output().stage != CalibrationStage::aborted; ++i)
        alternating.step(0, i % 2 == 0 ? 0.15 : -0.15);
    check(alternating.routine.output().failure == CalibrationFailure::translation,
          "back-and-forth center travel cannot evade the distance guard by cancellation");

    Fixture fast;
    fast.startTurning();
    auto output = fast.step(radians(361));
    check(output.failure == CalibrationFailure::rotationTooFast, "angular speed over 360 deg/s aborts");
    stopped(output);

    Fixture reset;
    reset.startTurning();
    reset.sample.left += 1;
    reset.sample.right -= 1;
    output = reset.step();
    check(output.failure == CalibrationFailure::encoderJump, "counter resets cannot hide behind pure-spin center travel");
    stopped(output);

    Fixture stalled;
    stalled.startTurning();
    for (int i = 0; i < 500 && stalled.routine.output().stage != CalibrationStage::aborted; ++i) stalled.step();
    check(stalled.routine.output().failure == CalibrationFailure::stalled, "powered but stationary robot stops after four seconds");
    stopped(stalled.routine.output());

    Fixture laterStall;
    laterStall.startTurning();
    for (int i = 0; i < 100; ++i) laterStall.step(1);
    for (int i = 0; i < 500 && laterStall.routine.output().stage != CalibrationStage::aborted; ++i) laterStall.step();
    check(laterStall.routine.output().failure == CalibrationFailure::stalled,
          "blocking a previously moving robot still stops within four seconds");
    stopped(laterStall.routine.output());
}

void invalidStartsAndDeadline() {
    Fixture bad;
    bad.sample.gyro = std::numeric_limits<double>::quiet_NaN();
    check(!bad.routine.start(bad.sample, 0), "nonfinite gyro rejected at start");
    stopped(bad.routine.output());
    bad.sample.gyro = 0;
    check(!bad.routine.start(bad.sample, 0, 0), "zero gyro scale rejected at start");
    check(!bad.routine.start(bad.sample, 0, 1, radians(4)), "implausible gyro bias rejected at start");
    check(!bad.routine.start(bad.sample, 0.101), "stale startup sample rejected");
    check(!bad.routine.start(bad.sample, -0.1), "sample from future rejected");
    check(bad.routine.start(bad.sample, 0), "explicit new start may restart an aborted routine");
    stopped(bad.routine.output());
    auto output = bad.routine.update(bad.sample, -0.001, true);
    check(output.failure == CalibrationFailure::invalidClock, "backwards wall clock aborts");

    Fixture timeout;
    timeout.start();
    for (int i = 0; i < 4600 && timeout.routine.output().stage != CalibrationStage::aborted; ++i)
        timeout.step(radians(6)); // Never quiet enough to finish the initial settling stage.
    check(timeout.routine.output().failure == CalibrationFailure::timeout, "45-second deadline applies even while settling");
    stopped(timeout.routine.output());

    GeometryRoutine idle;
    auto initial = idle.update(SensorSample{}, 100, true);
    check(initial.stage == CalibrationStage::idle, "permission alone never starts calibration");
    stopped(initial);
}
} // namespace

int main() {
    completeBothDirections();
    regulatedTorqueAndSpeed();
    deadmanAndSensorFailures();
    motionGuards();
    invalidStartsAndDeadline();
    std::printf("Calibration routine: %u checks passed\n", checks);
}
