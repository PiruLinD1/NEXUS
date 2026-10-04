#include "nexus/pod_offset_calibration.hpp"
#include <cstdio>
#include <cstdlib>
#include <limits>

using namespace nexus;
using namespace nexus::calibration;
namespace {
void check(bool condition, const char* message) {
    if (!condition) { std::fprintf(stderr, "FAIL: %s\n", message); std::exit(1); }
}
void near(double actual, double expected, double tolerance, const char* message) {
    if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance) {
        std::fprintf(stderr, "actual %.9f expected %.9f\n", actual, expected);
        check(false, message);
    }
}
struct Robot {
    EstimatorConfig config;
    SensorSample sample;
    Pose truePose;
    double forwardOffset = .020, lateralOffset = -.033;
    Robot() {
        config.nativeImuHeading = true;
        config.forwardScale = 1.013; config.lateralScale = .987; config.gyroScale = 1.004;
        sample.forwardValid = sample.lateralValid = sample.gyroValid = true;
        sample.forward = 12; sample.lateral = -8; sample.gyro = 17 * pi;
        // Deliberately wrong current offsets: calibration must not use them.
        config.forwardOffset = -.4; config.lateralOffset = .4;
    }
    void step(double forward, double lateral, double omega, double dt = .01) {
        const double theta = truePose.theta;
        if (std::abs(omega) > 1e-10) {
            truePose.x += forward / omega * (std::cos(theta) - std::cos(theta + omega * dt))
                        + lateral / omega * (std::sin(theta + omega * dt) - std::sin(theta));
            truePose.y += forward / omega * (std::sin(theta + omega * dt) - std::sin(theta))
                        + lateral / omega * (std::cos(theta + omega * dt) - std::cos(theta));
        } else {
            truePose.x += (forward * std::sin(theta) + lateral * std::cos(theta)) * dt;
            truePose.y += (forward * std::cos(theta) - lateral * std::sin(theta)) * dt;
        }
        truePose.theta += omega * dt;
        sample.timestamp += dt;
        sample.forward += (forward - forwardOffset * omega) * dt / config.forwardScale;
        sample.lateral += (lateral + lateralOffset * omega) * dt / config.lateralScale;
        sample.gyro += omega * dt / config.gyroScale;
    }
    template<class Observe> void recenter(Observe observe, double x = 0, double y = 0) {
        const double dx = x - truePose.x, dy = y - truePose.y;
        const double forward = dx * std::sin(truePose.theta) + dy * std::cos(truePose.theta);
        const double lateral = dx * std::cos(truePose.theta) - dy * std::sin(truePose.theta);
        for (unsigned i = 0; i < 50; ++i) { step(forward / .5, lateral / .5, 0); observe(sample); }
    }
};

void driftingRotationReferences() {
    for (double firstError : {0.0, .020}) {
        Robot robot;
        PodOffsetFit fit;
        fit.reset(robot.config.forwardScale, robot.config.lateralScale, robot.config.gyroScale);
        check(fit.add(robot.sample), "start from nonzero raw counters without motor encoders");
        for (unsigned point = 0; point < 2; ++point) {
            const double target = point == 0 ? radians(93) : radians(-88);
            const double omega = (target - robot.truePose.theta) / 2;
            for (unsigned i = 0; i < 200; ++i) {
                robot.step(.07, -.04, omega);
                check(fit.add(robot.sample), "drifting rotation is retained");
            }
            check(std::hypot(robot.truePose.x, robot.truePose.y) > .05, "fixture has substantial translation");
            robot.recenter([&](const SensorSample& sample) { check(fit.add(sample), "manual recentering is recorded"); },
                           point == 0 ? firstError : 0);
            check(fit.checkpoint(), "changed heading makes both offsets identifiable");
        }
        const auto result = fit.result();
        check(result.checkpoints == 2, "both reference measurements retained");
        if (firstError == 0) {
            near(result.forwardOffset, robot.forwardOffset, 1e-10, "forward offset recovered despite translation");
            near(result.lateralOffset, robot.lateralOffset, 1e-10, "lateral offset recovered despite translation");
            check(result.consistent, "correct independent references agree");
        } else check(!result.consistent, "inconsistent manual references are shown without claiming a reliable fit");
    }
}

void fullTurnsAreNotReferences() {
    Robot robot;
    PodOffsetFit fit;
    fit.reset(robot.config.forwardScale, robot.config.lateralScale, robot.config.gyroScale);
    check(fit.add(robot.sample), "full-turn fixture starts");
    for (unsigned i = 0; i < 4000; ++i) {
        // Ten full turns with a constant forward component close in world XY.
        robot.step(.04, 0, pi / 2);
        check(fit.add(robot.sample), "full-turn path remains a valid measured path");
    }
    near(std::hypot(robot.truePose.x, robot.truePose.y), 0, 1e-10, "ten drifting turns close at the original position");
    check(!fit.checkpoint(), "same heading after ten turns cannot identify pod offsets");
    check(std::abs(-fit.forwardTravel() / fit.heading() - robot.forwardOffset) > .020,
          "raw travel divided by angle remains wrong even after an exact XY closure");
}

void automaticTurns(double stiction, unsigned refreshFrames) {
    Robot robot;
    PodOffsetRoutine routine;
    check(routine.start(robot.sample, robot.sample.timestamp, robot.config), "pod routine starts without drive encoders");
    double omega = 0, peakSpeed = 0;
    SensorSample cache = robot.sample;
    unsigned alignments = 0;
    for (unsigned i = 1; i < 10000; ++i) {
        const auto old = routine.output();
        if (old.stage == PodOffsetStage::complete) break;
        if (old.stage == PodOffsetStage::align) {
            omega = 0;
            robot.recenter([&](const SensorSample& sample) {
                check(routine.update(sample, sample.timestamp, false).stage == PodOffsetStage::align,
                      "R1 may be released for manual recentering");
            });
            robot.step(0, 0, 0);
            const auto captured = routine.update(robot.sample, robot.sample.timestamp, false, true);
            check(captured.stage == PodOffsetStage::ready && captured.voltage.left == 0,
                  "A captures the reference but cannot start the next turn");
            ++alignments; cache = robot.sample;
            continue;
        }
        const double friction = std::abs(omega) < .01 ? stiction : .65 * stiction;
        const double desired = std::copysign(std::max(0.0, std::abs(old.voltage.left) - friction), old.voltage.left) * 2.0;
        omega += -std::expm1(-.01 / .12) * (desired - omega);
        peakSpeed = std::max(peakSpeed, std::abs(omega));
        robot.step(.02 * omega, -.01 * omega, omega);
        if (i % refreshFrames == 0) cache = robot.sample;
        cache.timestamp = robot.sample.timestamp;
        const auto output = routine.update(cache, robot.sample.timestamp, true);
        if (output.stage == PodOffsetStage::aborted)
            std::fprintf(stderr, "abort: %s, leg %u, angle %.2f\n", podOffsetFailureName(output.failure), output.leg, degrees(output.heading));
        check(output.stage != PodOffsetStage::aborted, "normal plant does not trip arbitrary speed or encoder gates");
        near(output.voltage.left, -output.voltage.right, 1e-12, "left and right turn oppositely");
        check(std::abs(output.voltage.left) <= PodOffsetRoutine::maxVoltage, "four volt output limit");
        if (std::abs(output.voltage.left) > std::abs(old.voltage.left))
            check(std::abs(output.voltage.left) - std::abs(old.voltage.left) <= .080001, "voltage ramps instead of jumping");
    }
    check(routine.output().stage == PodOffsetStage::complete && alignments == 2,
          "two signed reference turns and return home complete");
    near(degrees(routine.output().heading), 0, 4, "home returns close to the starting heading");
    const auto result = routine.fit().result();
    near(result.forwardOffset, robot.forwardOffset, .0003, "automatic sequence recovers forward offset");
    near(result.lateralOffset, robot.lateralOffset, .0003, "automatic sequence recovers lateral offset");
    check(result.consistent && peakSpeed < radians(190), "simulated speed and reference agreement are reasonable");
    std::printf("pod offsets: friction %.1f V, cache %u ms, peak %.1f deg/s, final heading %.2f deg\n",
                stiction, refreshFrames * 10, degrees(peakSpeed), degrees(routine.output().heading));
}

void stopsAndBadData() {
    for (unsigned cause = 0; cause < 6; ++cause) {
        Robot robot;
        PodOffsetRoutine routine;
        check(routine.start(robot.sample, 0, robot.config), "failure fixture starts");
        PodOffsetOutput output;
        if (cause == 0) { robot.step(0, 0, 0); output = routine.update(robot.sample, .01, false); }
        if (cause == 1) { robot.sample.forwardValid = false; output = routine.update(robot.sample, .01, true); }
        if (cause == 2) output = routine.update(robot.sample, .2, true);
        if (cause == 3) {
            robot.sample.gyro = std::numeric_limits<double>::quiet_NaN();
            output = routine.update(robot.sample, .01, true);
        }
        if (cause == 4) output = routine.update(robot.sample, -.01, true);
        if (cause == 5) {
            for (unsigned i = 0; i < 2100 && routine.output().stage != PodOffsetStage::aborted; ++i) {
                robot.step(0, 0, 0);
                output = routine.update(robot.sample, robot.sample.timestamp, true);
            }
            check(output.failure == PodOffsetFailure::timeout, "blocked rotation has a finite deadline");
        }
        check(output.stage == PodOffsetStage::aborted && output.voltage.left == 0 && output.voltage.right == 0,
              "release, sensor loss, stale time, NaN and timeout stop outputs");
        check(routine.update(robot.sample, robot.sample.timestamp, true).stage == PodOffsetStage::aborted,
              "an aborted test never resumes from R1 alone");
    }
}
}
int main() {
    driftingRotationReferences();
    fullTurnsAreNotReferences();
    for (double friction : {1.5, 2.3, 2.8}) for (unsigned refresh : {1u, 2u}) automaticTurns(friction, refresh);
    stopsAndBadData();
    std::puts("Pod offset calibration tests passed.");
}
