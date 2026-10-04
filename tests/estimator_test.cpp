#include "nexus/estimator.hpp"
#include "nexus/imu_status.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <random>

namespace {
using namespace nexus;
unsigned checks = 0;
void check(bool passed, const char* description) {
    ++checks;
    if (!passed) {
        std::fprintf(stderr, "FAIL: %s\n", description);
        std::exit(1);
    }
}
double distance(Pose a, Pose b) { return std::hypot(a.x - b.x, a.y - b.y); }
void near(double actual, double expected, double tolerance, const char* description) {
    if (std::abs(actual - expected) > tolerance) {
        std::fprintf(stderr, "actual %.12f expected %.12f tolerance %.12f\n", actual, expected, tolerance);
    }
    check(std::isfinite(actual) && std::abs(actual - expected) <= tolerance, description);
}
void covarianceValid(const Estimate& estimate) {
    // Sylvester's criterion on the symmetric 3x3 marginal catches indefinite
    // corrections, not just negative diagonal entries.
    const auto& p = estimate.covariance;
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j) {
            check(std::isfinite(p[i][j]), "finite covariance");
            near(p[i][j], p[j][i], 1e-12, "symmetric covariance");
        }
    check(p[0][0] > 0, "positive first covariance minor");
    check(p[0][0] * p[1][1] - p[0][1] * p[1][0] > 0, "positive second covariance minor");
    const double determinant = p[0][0] * (p[1][1] * p[2][2] - p[1][2] * p[2][1])
        - p[0][1] * (p[1][0] * p[2][2] - p[1][2] * p[2][0])
        + p[0][2] * (p[1][0] * p[2][1] - p[1][1] * p[2][0]);
    check(determinant > 0, "positive third covariance minor");
}
struct Simulator {
    EstimatorConfig config{};
    Pose truth{};
    SensorSample sample{};
    double gyroBias = 0;
    Simulator() {
        sample.forwardValid = sample.lateralValid = true;
        sample.leftValid = sample.rightValid = true;
        sample.gyroValid = sample.gyroRateValid = true;
    }
    SensorSample step(double speed, double omega, double lateral = 0, double dt = 0.01) {
        // Independent closed-form integration of constant body velocity.
        const double theta = truth.theta;
        if (std::abs(omega) > 1e-9) {
            truth.x += speed / omega * (std::cos(theta) - std::cos(theta + omega * dt))
                + lateral / omega * (std::sin(theta + omega * dt) - std::sin(theta));
            truth.y += speed / omega * (std::sin(theta + omega * dt) - std::sin(theta))
                + lateral / omega * (std::cos(theta + omega * dt) - std::cos(theta));
        } else {
            truth.x += (speed * std::sin(theta) + lateral * std::cos(theta)) * dt;
            truth.y += (speed * std::cos(theta) - lateral * std::sin(theta)) * dt;
        }
        truth.theta = wrap(theta + omega * dt);
        sample.timestamp += dt;
        sample.forward += (speed - config.forwardOffset * omega) * dt;
        sample.lateral += (lateral + config.lateralOffset * omega) * dt;
        sample.left += (speed + config.trackWidth * omega / 2) * dt;
        sample.right += (speed - config.trackWidth * omega / 2) * dt;
        sample.gyro += (omega + gyroBias) * dt;
        sample.gyroRate = omega + gyroBias;
        return sample;
    }
};

void imuMountingStatusAndWheelSlip() {
    // Status encodes the six physical mounting orientations in bits 1..3.
    // Previously status == 0 silently discarded the gyro in five orientations.
    for (unsigned orientation = 0; orientation < 6; ++orientation) {
        const unsigned readyStatus = orientation << 1;
        check(detail::imuStatusReady(readyStatus), "ready IMU accepted in every mounting orientation");
        check(!detail::imuStatusReady(readyStatus | 1u), "calibration bit rejects every orientation");
        Simulator sim;
        sim.sample.gyroRateValid = false; // Same rotation-only path as the robot.
        sim.sample.gyroValid = detail::imuStatusReady(readyStatus);
        Estimator estimator(sim.config);
        estimator.update(sim.sample);
        for (int i = 0; i < 200; ++i) {
            auto sample = sim.step(0.2, radians(45));
            sample.gyroValid = detail::imuStatusReady(readyStatus);
            // The real robot turns 90 deg; inaccurate/slipping drive wheels
            // suggest 126 deg. A healthy IMU must remain the heading source.
            sample.left *= 1.4;
            sample.right *= 1.4;
            const auto estimate = estimator.update(sample);
            check(estimate.headingSource == HeadingSource::imu, "heading reports the IMU actually used");
            near(wrap(estimate.state.pose.theta - sim.truth.theta), 0, 1e-9,
                 "wheel rotation error cannot replace valid gyro heading");
        }
        near(degrees(estimator.estimate().state.pose.theta), 90, 1e-9,
             "90 degree turn remains 90 degrees in all six mountings");
    }
    check(!detail::imuStatusReady(0xff), "IMU error sentinel rejected");
    check(!detail::imuStatusReady(19), "calibration flag with other status bits rejected");
    check(detail::imuStatusReady(0x12), "non-calibration flags do not reject a ready IMU");
    check(!detail::imuStatusReady(std::numeric_limits<std::uint32_t>::max()), "invalid status rejected");
}

void continuousTurnsAndConfidence() {
    Simulator sim;
    sim.sample.gyroRateValid = false;
    Estimator normal(sim.config), shifted(sim.config);
    const auto updateBoth = [&](const SensorSample& sample) {
        const auto estimate = normal.update(sample);
        auto differentCounter = sample;
        differentCounter.gyro -= 4 * pi; // Same physical motion, counter starts 720 deg lower.
        const auto other = shifted.update(differentCounter);
        near(wrap(estimate.state.pose.theta - other.state.pose.theta), 0, 1e-9,
             "whole turns in gyro counter do not change heading");
        near(estimate.confidence, other.confidence, 1e-9,
             "whole turns in gyro counter do not change confidence");
        for (std::size_t i = 0; i < 3; ++i)
            for (std::size_t j = 0; j < 3; ++j)
                near(estimate.covariance[i][j], other.covariance[i][j], 1e-9,
                     "covariance is independent of the absolute gyro counter");
        return estimate;
    };
    updateBoth(sim.sample);
    for (int i = 0; i < 300; ++i) updateBoth(sim.step(0, 0));
    for (int i = 0; i < 800; ++i) updateBoth(sim.step(0, -pi / 2));
    near(degrees(sim.sample.gyro), -720, 1e-8, "gyro continuously accumulates two full turns");
    near(degrees(normal.estimate().state.pose.theta), 0, 1e-8,
         "heading returns to zero after two turns without resetting the IMU");
    near(normal.estimate().odometry.headingChange, -4 * pi, 1e-10,
         "diagnostic heading retains complete turns hidden by the wrapped pose heading");
    near(normal.estimate().odometry.minimumHeadingChange, -4 * pi, 1e-10,
         "persistent minimum heading crosses the wrap boundary without losing a turn");
    near(normal.estimate().odometry.maximumHeadingChange, 0, 1e-12,
         "negative complete turns keep the reset origin as maximum diagnostic heading");
    near(distance(normal.estimate().state.pose, {}), 0, 1e-9,
         "correct tracking offsets keep pure rotations at the same position");
    std::printf("two-turn simulation: raw IMU %.1f deg, heading %.6f deg, confidence %.3f, position sigma %.2f mm, heading sigma %.3f deg; shifted counter gives identical confidence\n",
        degrees(sim.sample.gyro), degrees(normal.estimate().state.pose.theta), normal.estimate().confidence,
        1000 * std::sqrt(normal.estimate().covariance[0][0] + normal.estimate().covariance[1][1]),
        degrees(std::sqrt(normal.estimate().covariance[2][2])));
}

void exactOdometry() {
    Simulator sim;
    Estimator estimator(sim.config);
    estimator.update(sim.sample);
    for (int i = 0; i < 600; ++i) {
        const double speed = i < 200 ? 1.2 : (i < 400 ? -0.65 : 0.9);
        const double omega = i < 200 ? 0 : (i < 400 ? 2.2 : -3.4);
        const double lateral = i < 400 ? 0 : 0.12;
        const Estimate estimate = estimator.update(sim.step(speed, omega, lateral));
        check(distance(estimate.state.pose, sim.truth) < 1e-9, "straight/reverse/curve/push SE2 position");
        const auto& odom = estimate.odometry;
        near(odom.forwardX + odom.lateralX + odom.offsetX, sim.truth.x, 1e-9,
             "diagnostic X contributions reconstruct independent mixed-motion truth");
        near(odom.forwardY + odom.lateralY + odom.offsetY, sim.truth.y, 1e-9,
             "diagnostic Y contributions reconstruct independent mixed-motion truth");
        near(wrap(estimate.state.pose.theta - sim.truth.theta), 0, 1e-10, "large heading change");
    }
    covarianceValid(estimator.estimate());
    check(estimator.estimate().health == Health::healthy, "pods and gyro healthy without ranges");
    near(estimator.estimate().state.v, 0.9, 1e-8, "forward velocity converges");
    near(estimator.estimate().state.omega, -3.4, 1e-8, "angular velocity converges");
    near(estimator.estimate().state.lateralV, 0.12, 1e-8, "body lateral velocity survives simultaneous turning");
    near(estimator.estimate().slip, 0, 1e-9, "offset turns do not falsely detect slip");
}

void relocatedPodsClosedPath() {
    Simulator sim;
    sim.config.forwardOffset = 20 * millimeter;
    sim.config.lateralOffset = -70 * millimeter;
    sim.sample.gyroRateValid = false; // Same rotation-only path as the robot.
    Estimator exact(sim.config);
    auto wrongGyro = sim.config;
    wrongGyro.gyroScale = 1.02;
    Estimator angularScaleError(wrongGyro);
    exact.update(sim.sample);
    angularScaleError.update(sim.sample);
    // A 20-second square: alternating forward/reverse sides, clockwise-frame
    // headings chosen to close the real path, with one-second stops after turns.
    for (int side = 0; side < 4; ++side) {
        for (int i = 0; i < 500; ++i) {
            const double speed = i < 200 ? (side % 2 ? -0.5 : 0.5) : 0;
            const double omega = i >= 200 && i < 400 ? -pi / 4 : 0;
            const auto reading = sim.step(speed, omega);
            exact.update(reading);
            angularScaleError.update(reading);
        }
    }
    near(distance(sim.truth, {}), 0, 1e-9, "mixed forward/reverse square physically closes after 20 seconds");
    near(distance(exact.estimate().state.pose, {}), 0, 1e-9,
         "relocated pods at X +20 mm and Y -70 mm preserve closed-path position");
    near(wrap(exact.estimate().state.pose.theta), 0, 1e-9,
         "relocated pods preserve closed-path heading");
    const auto faulty = angularScaleError.estimate().state.pose;
    check(distance(faulty, {}) > 0.05,
          "angular scale mismatch can accumulate position error on a closed curved path");
    std::printf("20-second relocated-pod loop: ideal %.6f mm; simulated 2%% gyro scale error X %.1f Y %.1f mm, heading %.2f deg\n",
        distance(exact.estimate().state.pose, {}) / millimeter,
        faulty.x / millimeter, faulty.y / millimeter, degrees(faulty.theta));
}

void dropoutReconnect() {
    Simulator sim;
    Estimator estimator(sim.config);
    estimator.update(sim.sample);
    for (int i = 0; i < 500; ++i) {
        SensorSample sample = sim.step(0.65, 0.7);
        if (i >= 100 && i < 250) {
            sample.forwardValid = sample.lateralValid = sample.gyroValid = false;
        }
        if (i >= 250) {
            // Reconnected devices reset their own cumulative readings.
            sample.forward -= 4;
            sample.lateral += 3;
            sample.gyro += 4;
        }
        const Estimate estimate = estimator.update(sample);
        check(distance(estimate.state.pose, sim.truth) < 1e-8, "drive fallback and reconnect baseline");
        near(wrap(estimate.state.pose.theta - sim.truth.theta), 0, 1e-9, "gyro fallback preserves heading");
        if (i == 150) {
            check(estimate.health == Health::degraded, "dropout reports degraded");
            check(estimate.headingSource == HeadingSource::driveEncoders, "dropout explicitly reports encoder heading");
        }
        if (i == 300) check(estimate.headingSource == HeadingSource::imu, "reconnected gyro restores IMU source");
    }
    covarianceValid(estimator.estimate());
    const Pose before = estimator.estimate().state.pose;
    SensorSample missing;
    missing.timestamp = sim.sample.timestamp;
    for (int i = 0; i < 25; ++i) {
        missing.timestamp += 0.01;
        estimator.update(missing);
    }
    check(estimator.estimate().health == Health::lost, "all sensors missing stops trusted localization");
    check(estimator.estimate().headingSource == HeadingSource::unavailable, "missing heading source is explicit");
    near(distance(before, estimator.estimate().state.pose), 0, 1e-10, "missing sensors never invent travel");
}

void rejectedCounterCannotBecomeMotionOnReturn() {
    struct Counter {
        double SensorSample::*value;
        double spike;
        bool driveOnly;
        std::uint8_t mask;
    };
    const Counter counters[] = {
        {&SensorSample::lateral, 0.056, false, 2},
        {&SensorSample::forward, 0.050, false, 1},
        {&SensorSample::left, 0.076, true, 4},
        {&SensorSample::right, 0.076, true, 8},
        {&SensorSample::gyro, 0.220, false, 16}
    };
    for (const auto& counter : counters) {
        Simulator sim;
        Estimator estimator(sim.config);
        if (counter.driveOnly) sim.sample.forwardValid = sim.sample.gyroValid = false;
        estimator.update(sim.sample);
        auto spike = sim.step(0, 0, 0, 0.009);
        spike.*counter.value += counter.spike;
        auto estimate = estimator.update(spike);
        check(estimate.rejectedIncrements == 1 && estimate.rejectedIncrementMask == counter.mask &&
              estimate.lastRejectedIncrementMask == counter.mask, "rejected counter identifies its diagnostic source");
        near(distance(estimate.state.pose, {}), 0, 1e-12, "implausible counter spike is rejected");
        near(estimate.state.pose.theta, 0, 1e-12, "implausible gyro spike is rejected");
        // A 2 ms scheduling difference makes the same return delta fit the
        // next frame's plausibility limit. It still is not physical travel.
        estimate = estimator.update(sim.step(0, 0, 0, 0.011));
        check(estimate.rejectedIncrements == 1 && estimate.rejectedIncrementMask == 0 &&
              estimate.lastRejectedIncrementMask == counter.mask,
              "reconnect is not another rejection and the latest source remains visible");
        near(distance(estimate.state.pose, {}), 0, 1e-12,
             "return from a rejected counter spike only establishes a baseline");
        near(estimate.state.pose.theta, 0, 1e-12,
             "return from a rejected gyro spike cannot rotate the pose");
        estimator.update(sim.step(0.24, 0, 0, 0.01));
        near(estimator.estimate().state.pose.y, 0.0024, 1e-12,
             "valid motion resumes immediately after a fresh counter baseline");
        estimator.reset({});
        check(estimator.estimate().rejectedIncrements == 0 &&
              estimator.estimate().lastRejectedIncrementMask == 0, "explicit pose reset clears rejection diagnostics");
    }

    Simulator sim;
    sim.sample.lateral = 0.4;
    Estimator estimator(sim.config);
    estimator.update(sim.sample);
    for (int i = 0; i < 100; ++i) estimator.update(sim.step(0, 0));
    check(estimator.estimate().stationary, "healthy wheel dwell is exposed for physical diagnosis");
    sim.sample.lateral = 0; // Persistent hardware counter reset, rather than a transient spike.
    auto estimate = estimator.update(sim.step(0, 0));
    check(!estimate.stationary, "a rejected wheel reading cannot certify stationary bias evidence");
    estimator.update(sim.step(0, 0));
    estimate = estimator.update(sim.step(0, 0, 0.01));
    near(estimate.state.pose.x, 0.0001, 1e-12,
         "persistent counter reset rebaselines once and preserves subsequent slow lateral motion");
    near(estimate.gyroBiasStd, std::sqrt(square(estimate.gyroBiasStd)), 1e-12,
         "bias uncertainty diagnostic is finite and nonnegative");
    check(estimator.setGyroBias(0, square(radians(0.02))), "explicit calibration for diagnostic is accepted");
    near(estimator.estimate().gyroBiasStd, radians(0.02), 1e-12,
         "explicit bias calibration immediately updates its uncertainty diagnostic");
}

void slowForwardReverseWithoutAbsoluteSensors() {
    Simulator sim;
    Estimator uncalibrated(sim.config), calibrated(sim.config);
    check(calibrated.setGyroBias(0, square(radians(0.02))), "known bias calibration accepted for slow-motion regression");
    uncalibrated.update(sim.sample);
    calibrated.update(sim.sample);
    for (int i = 0; i < 1000; ++i) {
        const double dt = i == 100 ? 0.009 : i == 101 ? 0.011 : 0.01;
        auto sample = sim.step(i < 500 ? 0.24 : -0.24, 0, 0, dt);
        if (i == 100) sample.lateral += 0.056;
        uncalibrated.update(sample);
        calibrated.update(sample);
    }
    for (const auto* estimator : {&uncalibrated, &calibrated}) {
        const auto estimate = estimator->estimate();
        near(distance(estimate.state.pose, {}), 0, 1e-10,
             "1.2 m out and back at 240 mm/s rejects a transient lateral spike without a pose shift");
        near(estimate.state.pose.theta, 0, 1e-12, "ideal forward/reverse motion preserves heading");
        check(estimate.rejectedIncrements == 1 && estimate.lastRejectedIncrementMask == 2,
              "a brief lateral fault remains diagnosable after finishing the route");
        check(estimate.acceptedRanges == 0, "slow-motion regression needs no absolute distance sensors");
        covarianceValid(estimate);
    }
    const auto raw = uncalibrated.estimate(), known = calibrated.estimate();
    std::printf("1.2 m out + 1.2 m back, 5 s each, no Distance: ideal final error %.3f mm; uncertainty uncalibrated %.1f mm / %.2f deg, calibrated %.1f mm / %.2f deg; lateral rejection count %u\n",
        distance(raw.state.pose, {}) / millimeter,
        std::sqrt(raw.covariance[0][0] + raw.covariance[1][1]) / millimeter,
        degrees(std::sqrt(raw.covariance[2][2])),
        std::sqrt(known.covariance[0][0] + known.covariance[1][1]) / millimeter,
        degrees(std::sqrt(known.covariance[2][2])), raw.rejectedIncrements);
}

void boundedCachedSensorIncrements() {
    EstimatorConfig config;
    config.forwardOffset = 0.020;
    config.lateralOffset = -0.070;
    config.trackWidth = 0.287;
    config.maxSensorHold = 0.010; // Known 10 ms shared-cache refresh bound.
    EstimatorConfig synchronous = config;
    synchronous.maxSensorHold = 0;
    struct Motion { double forward, omega, lateral; };
    const Motion motions[] = {{2.7, 0, 0}, {0, 12, 0}, {2.7, 12, 0},
                              {2.7, -12, 0}, {0, -12, 2.7}};
    unsigned rejectedSources = 0;
    for (const auto& motion : motions) for (double pollPeriod : {0.01, 0.001}) {
        Simulator sim;
        sim.config = config;
        sim.sample.gyroRateValid = false;
        Estimator cached(config), strict(synchronous);
        cached.update(sim.sample); strict.update(sim.sample);
        SensorSample published = sim.sample;
        const int refreshEvery = pollPeriod < .01 ? 10 : 2;
        const int samples = pollPeriod < .01 ? 2000 : 200;
        for (int i = 1; i <= samples; ++i) {
            const auto current = sim.step(motion.forward, motion.omega, motion.lateral, pollPeriod);
            // Exercise both a repeated 100 Hz cache frame and 1000 Hz polling
            // of data that arrives every 10 ms, including fast curves.
            if (i % refreshEvery == 0) published = current;
            published.timestamp = current.timestamp;
            const auto estimate = cached.update(published);
            rejectedSources |= strict.update(published).rejectedIncrementMask;
            check(estimate.rejectedIncrements == 0, "bounded held counters preserve physically valid fast motion");
        }
        const auto estimate = cached.estimate();
        near(distance(estimate.state.pose, sim.truth), 0, 1e-10,
             "cached forward, lateral and turning increments retain complete travel");
        near(wrap(estimate.state.pose.theta - sim.truth.theta), 0, 1e-10,
             "a held gyro resumes without false fallback or lost rotation");
        covarianceValid(estimate);
    }
    check(rejectedSources == 31, "strict single-poll limits can falsely reject all five cached counter channels");

    // Isolate the actual adapter pattern: pods cached, motor counters fresh.
    Simulator sim;
    sim.config = config;
    sim.sample.gyroRateValid = false;
    Estimator cached(config), strict(synchronous);
    cached.update(sim.sample); strict.update(sim.sample);
    auto sample = sim.step(2.7, 0);
    sample.forward = 0;
    cached.update(sample); strict.update(sample);
    sample = sim.step(2.7, 0);
    cached.update(sample); strict.update(sample);
    near(cached.estimate().state.pose.y, 0.054, 1e-12,
         "54 mm cached pod packet at 2.7 m/s is retained after a held frame");
    near(strict.estimate().state.pose.y, 0.027, 1e-12,
         "strict gate reproducer loses half the travel despite valid fresh drive fallback");
    std::printf("cached forward pod at 2.7 m/s: synchronous-only gate %.1f mm, bounded 10 ms hold gate %.1f mm, actual 54.0 mm\n",
        strict.estimate().state.pose.y / millimeter, cached.estimate().state.pose.y / millimeter);

    sim = Simulator{};
    sim.config = config;
    cached.reset({});
    cached.update(sim.sample);
    for (int i = 0; i < 1000; ++i) cached.update(sim.step(0, 0));
    sample = sim.step(0, 0);
    sample.forward = sample.lateral = sample.left = sample.right = sample.gyro = 1;
    auto estimate = cached.update(sample);
    check(estimate.rejectedIncrementMask == 31,
          "long standstill cannot grow the configured hold allowance and admit a counter reset");
    near(distance(estimate.state.pose, {}), 0, 1e-12, "bounded idle gate rejects large false travel");
    near(estimate.state.pose.theta, 0, 1e-12, "bounded idle gate rejects large false heading");
    cached.update(sim.step(0, 0)); // Rebaseline after rejected counters.
    sample = sim.step(0, 0);
    sample.forward = 0.054;
    estimate = cached.update(sample);
    check(estimate.rejectedIncrementMask == 1,
          "reconnect baseline clears earlier held age before the next increment");

    config.maxSensorHold = std::numeric_limits<double>::quiet_NaN();
    near(Estimator(config).config().maxSensorHold, 0, 1e-12, "nonfinite hold allowance restores synchronous gating");
    config.maxSensorHold = 1000;
    near(Estimator(config).config().maxSensorHold, 0.1, 1e-12, "configured hold allowance has a finite upper bound");
}

void biasNoiseAndSlip() {
    Simulator sim;
    sim.gyroBias = radians(0.18);
    Estimator estimator(sim.config);
    estimator.update(sim.sample);
    for (int i = 0; i < 800; ++i) estimator.update(sim.step(0, 0));
    near(estimator.estimate().gyroBias, sim.gyroBias, radians(0.006), "stationary gyro bias learned");
    const double heading = estimator.estimate().state.pose.theta;
    for (int i = 0; i < 800; ++i) estimator.update(sim.step(0.6, 0));
    near(estimator.estimate().state.pose.theta, heading, radians(0.06), "bias compensation limits moving drift");
    const double learned = estimator.estimate().gyroBias;
    estimator.reset({1, 2, 0.5});
    near(estimator.estimate().gyroBias, learned, 1e-12, "pose reset preserves learned gyro bias");

    Simulator rotationOnly;
    rotationOnly.gyroBias = radians(0.18);
    rotationOnly.sample.gyroRateValid = false;
    Estimator differenceBias(rotationOnly.config);
    differenceBias.update(rotationOnly.sample);
    for (int i = 0; i < 800; ++i) differenceBias.update(rotationOnly.step(0, 0));
    near(differenceBias.estimate().gyroBias, rotationOnly.gyroBias, radians(0.003),
         "nonoverlapping gyro windows learn bias without raw rate");
    const double rotationHeading = differenceBias.estimate().state.pose.theta;
    for (int i = 0; i < 800; ++i) differenceBias.update(rotationOnly.step(0.6, 0));
    near(differenceBias.estimate().state.pose.theta, rotationHeading, radians(0.03),
         "gyro-window bias holds heading while moving");

    Simulator slipping;
    Estimator slipEstimator(slipping.config);
    slipEstimator.update(slipping.sample);
    double motorSlip = 0;
    for (int i = 0; i < 300; ++i) {
        SensorSample sample = slipping.step(0.5, 0.3);
        if (i >= 100 && i < 200) motorSlip += 0.016;
        sample.left += motorSlip;
        sample.right += motorSlip;
        const Estimate estimate = slipEstimator.update(sample);
        check(distance(estimate.state.pose, slipping.truth) < 1e-8, "drive slip does not override pods");
        if (i == 180) check(estimate.slip > 0.4, "sustained drivetrain slip detected");
    }

    std::mt19937 random(1127);
    std::normal_distribution<double> podNoise(0, 0.00020), gyroNoise(0, radians(0.025));
    double squaredPositionError = 0, maxPositionError = 0, maxHeadingError = 0;
    for (int trial = 0; trial < 12; ++trial) {
        Simulator noisy;
        Estimator filtered(noisy.config);
        filtered.update(noisy.sample);
        for (int i = 0; i < 1000; ++i) {
            SensorSample sample = noisy.step(0.7 * std::sin(i * 0.009), 1.2 * std::sin(i * 0.006));
            sample.forward += podNoise(random);
            sample.lateral += podNoise(random);
            sample.gyro += gyroNoise(random);
            filtered.update(sample);
        }
        const double error = distance(filtered.estimate().state.pose, noisy.truth);
        squaredPositionError += error * error;
        maxPositionError = std::max(maxPositionError, error);
        maxHeadingError = std::max(maxHeadingError, std::abs(wrap(filtered.estimate().state.pose.theta - noisy.truth.theta)));
        covarianceValid(filtered.estimate());
    }
    check(maxPositionError < 0.025, "noisy repeated odometry stays within 25 mm");
    check(maxHeadingError < radians(0.2), "noisy repeated heading stays within 0.2 degrees");
    std::printf("noisy simulation: 12 runs, position RMS %.3f mm, max %.3f mm, heading max %.4f deg\n",
                1000 * std::sqrt(squaredPositionError / 12), 1000 * maxPositionError, degrees(maxHeadingError));
}

void closedManualCyclesAndSensorTiming() {
    for (double lag : {0.0, 0.010}) {
        EstimatorConfig config;
        config.forwardOffset = .020; config.lateralOffset = -.070; config.trackWidth = .287;
        config.maxSensorHold = .010;
        Estimator estimator(config);
        estimator.setGyroBias(0, square(radians(.02)));
        constexpr double amplitude = .250, duration = 20;
        const double k = radians(3) / amplitude;
        const auto yAt = [](double t) {
            return t <= 0 || t >= duration ? 0.0 : amplitude * std::sin(2 * pi * t);
        };
        SensorSample sample;
        sample.forwardValid = sample.lateralValid = sample.leftValid = sample.rightValid = sample.gyroValid = true;
        for (int i = 0; i <= 2100; ++i) {
            const double t = i * .010, y = yAt(t), theta = k * y;
            sample.timestamp = t;
            // Independent analytic sensor integrals for true X=0, Y=y(t),
            // heading=k*y(t). All cumulative counters close after 20 cycles.
            const double centerTravel = std::sin(theta) / k;
            sample.forward = centerTravel - config.forwardOffset * theta;
            sample.lateral = (std::cos(theta) - 1) / k + config.lateralOffset * theta;
            sample.left = centerTravel + config.trackWidth * .5 * theta;
            sample.right = centerTravel - config.trackWidth * .5 * theta;
            sample.gyro = k * yAt(t - lag);
            estimator.update(sample);
        }
        const auto out = estimator.estimate();
        const auto& odom = out.odometry;
        check(out.rejectedIncrements == 0, "manual-cycle timing fixture contains only plausible increments");
        near(out.state.pose.theta, 0, 1e-10, "closed heading does not imply closed position with sensor delay");
        near(odom.forwardX + odom.lateralX + odom.offsetX, out.state.pose.x, 1e-10,
             "diagnostics account for the complete X residual after repeated reversals");
        near(odom.forwardY + odom.lateralY + odom.offsetY, out.state.pose.y, 1e-10,
             "diagnostics account for the complete Y residual after repeated reversals");
        if (lag == 0) near(distance(out.state.pose, {}), 0, 1e-8, "synchronized reciprocal manual path closes");
        else {
            // This documents an observability limit, not a calibrated hardware
            // delay or a claimed fix: no guessed lag correction is permitted.
            check(std::abs(out.state.pose.x) > .030, "small relative sensor delay can accumulate centimetres of X drift");
            near(odom.forwardX, out.state.pose.x, .001, "delayed-heading example is identifiable as forward projection drift");
        }
        std::printf("20 manual closed cycles, synthetic gyro lag %.0f ms: X %.3f Y %.3f mm; X terms forward %.3f lateral %.3f offset %.3f mm; heading %.5f deg\n",
                    lag * 1000, out.state.pose.x / millimeter, out.state.pose.y / millimeter,
                    odom.forwardX / millimeter, odom.lateralX / millimeter, odom.offsetX / millimeter,
                    degrees(out.state.pose.theta));
        estimator.reconfigure(config);
        near(estimator.estimate().odometry.forwardX, odom.forwardX, 1e-12,
             "geometry reconfiguration preserves cumulative odometry diagnostics");
        estimator.reset({1, 2, 0});
        const auto reset = estimator.estimate().odometry;
        check(reset.forwardX == 0 && reset.forwardY == 0 && reset.lateralX == 0 && reset.lateralY == 0 &&
              reset.offsetX == 0 && reset.offsetY == 0, "known pose starts a fresh diagnostic displacement history");
    }
}

void independentArcAndPollTiming() {
    // A 500 mm centre arc turning 30 degrees, followed by its exact reverse.
    // Known circle coordinates generate the reference directly; neither the
    // Simulator nor the estimator's incremental SE(2) formula generates truth.
    constexpr double length = .500, turn = pi / 6, radius = length / turn;
    const auto progress = [](int frame) {
        frame = std::clamp(frame, 0, 200);
        return (frame <= 100 ? frame : 200 - frame) / 100.0;
    };
    for (bool heldGyro : {false, true}) {
        EstimatorConfig config;
        config.forwardOffset = .020; config.lateralOffset = -.070; config.trackWidth = .287;
        config.maxSensorHold = .010;
        config.stationaryEncoderSpan = .010 * millimeter;
        Estimator regular(config), jittered(config);
        regular.setGyroBias(0, square(radians(.02)));
        jittered.setGyroBias(0, square(radians(.02)));
        double maximumTimingDifference = 0;
        for (int frame = 0; frame <= 201; ++frame) {
            const double fraction = progress(frame), theta = turn * fraction;
            const double travel = length * fraction;
            SensorSample sample;
            sample.timestamp = frame * .010;
            sample.forwardValid = sample.lateralValid = sample.leftValid = sample.rightValid = sample.gyroValid = true;
            sample.forward = travel - config.forwardOffset * theta;
            sample.lateral = config.lateralOffset * theta;
            sample.left = travel + config.trackWidth * theta / 2;
            sample.right = travel - config.trackWidth * theta / 2;
            sample.gyro = turn * progress(frame - (heldGyro ? 1 : 0));
            const auto synchronous = regular.update(sample);
            // Same cumulative increments, with a valid 6/14 ms polling pattern.
            // Bias is zero, every increment remains plausible and motion never
            // enters quiet. Timing may affect velocities/Q but must not scale pose.
            sample.timestamp -= frame % 2 ? .004 : 0;
            const auto irregular = jittered.update(sample);
            const auto& actual = synchronous.state.pose;
            const auto& compared = irregular.state.pose;
            near(compared.x, actual.x, 1e-12, "poll jitter does not rescale each arc X increment");
            near(compared.y, actual.y, 1e-12, "poll jitter does not rescale each arc Y increment");
            near(compared.theta, actual.theta, 1e-12, "poll jitter does not rescale integrated gyro angles at zero bias");
            maximumTimingDifference = std::max(maximumTimingDifference, distance(compared, actual));
            check(!synchronous.stationary && !irregular.stationary,
                  "the moving arc fixture does not exercise a quiet-heading substitution");
            if (!heldGyro) {
                near(actual.x, radius * (1 - std::cos(theta)), 1e-10,
                     "two-pod arc X agrees with independent circle geometry at every sample");
                near(actual.y, radius * std::sin(theta), 1e-10,
                     "two-pod arc Y agrees with independent circle geometry at every sample");
                near(actual.theta, theta, 1e-12, "arc heading agrees with independent geometry");
            }
        }
        const auto regularEnd = regular.estimate(), jitteredEnd = jittered.estimate();
        check(regularEnd.rejectedIncrements == 0 && jitteredEnd.rejectedIncrements == 0,
              "regular and jittered arc increments all pass the plausibility checks");
        near(regularEnd.state.pose.theta, 0, 1e-12, "heading closes after the final gyro counter catches up");
        if (!heldGyro) near(distance(regularEnd.state.pose, {}), 0, 1e-10,
                            "synchronized arc and its inverse close at the start");
        else {
            // This is an input timing-contract limitation, not a measured V5
            // latency or permission to compensate a guessed delay. All host
            // timestamps can be regular while one device returns an older frame.
            check(distance(regularEnd.state.pose, {}) > .004,
                  "one stale gyro frame per poll can prevent a closed arc from closing position");
        }
        std::printf("independent 500 mm / 30 deg arc + inverse, gyro %s: final X %.6f Y %.6f mm, heading %.8f deg; max per-sample pose difference 10 ms vs 6/14 ms %.12g mm\n",
            heldGyro ? "previous frame" : "current frame", regularEnd.state.pose.x / millimeter,
            regularEnd.state.pose.y / millimeter, degrees(regularEnd.state.pose.theta),
            maximumTimingDifference / millimeter);
    }
}

void persistentOdometryDiagnosticsWithMissingPods() {
    EstimatorConfig config;
    config.forwardOffset = .020;
    config.lateralOffset = -.070;
    config.trackWidth = .287;
    Estimator complete(config), interrupted(config);
    constexpr double excursion = .040;
    const auto sampleAt = [&](double timestamp, double heading) {
        // Independent cumulative wheel geometry for rotation about a fixed
        // centre; no estimator or Simulator integration generates these data.
        SensorSample sample;
        sample.timestamp = timestamp;
        sample.forward = -config.forwardOffset * heading;
        sample.lateral = config.lateralOffset * heading;
        sample.left = config.trackWidth * heading / 2;
        sample.right = -sample.left;
        sample.gyro = heading;
        sample.forwardValid = sample.lateralValid = true;
        sample.leftValid = sample.rightValid = sample.gyroValid = true;
        return sample;
    };
    const std::array<std::uint32_t, 3> none{};
    complete.update(sampleAt(0, 0));
    interrupted.update(sampleAt(0, 0));
    check(interrupted.estimate().unusableIncrements == none,
          "initial sensor baselines are not reported as unusable increments");

    auto missing = sampleAt(.01, -excursion);
    complete.update(missing);
    missing.forwardValid = missing.lateralValid = false;
    auto estimate = interrupted.update(missing);
    check(estimate.unusableIncrements == std::array<std::uint32_t, 3>{1, 1, 0},
          "missing pod samples are retained in diagnostics despite zero rejected increments");
    check(estimate.rejectedIncrements == 0 && estimate.headingSource == HeadingSource::imu,
          "missing pods neither count as implausible jumps nor disable a valid IMU");
    missing.gyroValid = false;
    estimate = interrupted.update(missing); // Duplicate timestamp must be ignored entirely.
    check(estimate.unusableIncrements == std::array<std::uint32_t, 3>{1, 1, 0},
          "duplicate timestamps cannot inflate persistent unavailable-channel counters");

    complete.update(sampleAt(.02, -excursion));
    estimate = interrupted.update(sampleAt(.02, -excursion));
    check(estimate.unusableIncrements == std::array<std::uint32_t, 3>{2, 2, 0},
          "the first returning pod samples count as rebaselines without usable increments");
    complete.update(sampleAt(.03, 0));
    estimate = interrupted.update(sampleAt(.03, 0));
    check(estimate.rejectedIncrements == 0 && estimate.lastRejectedIncrementMask == 0,
          "a brief dropout and recovery leave the plausibility-rejection history empty");
    check(estimate.unusableIncrements == std::array<std::uint32_t, 3>{2, 2, 0},
          "usable returning increments leave the dropout counters unchanged");
    check(complete.estimate().unusableIncrements == none,
          "the otherwise identical uninterrupted path loses no sensor increments");
    for (const auto* estimator : {&complete, &interrupted}) {
        const auto result = estimator->estimate();
        near(distance(result.state.pose, {}), 0, 1e-12,
             "pure rotation keeps the centre fixed with a drive fallback or complete pods");
        near(result.state.pose.theta, 0, 1e-12,
             "heading closes despite the intermediate pod dropout");
        near(result.odometry.headingChange, 0, 1e-12,
             "integrated diagnostic heading closes with the measured turn");
        near(result.odometry.minimumHeadingChange, -excursion, 1e-12,
             "persistent heading diagnostics retain a transient missed by endpoint readings");
        near(result.odometry.maximumHeadingChange, 0, 1e-12,
             "heading extrema include the reset origin without inventing positive rotation");
    }
    near(complete.estimate().odometry.offsetX, 0, 1e-12,
         "constant tracking offsets cancel over an uninterrupted closed heading path");
    near(complete.estimate().odometry.offsetY, 0, 1e-12,
         "uninterrupted rotation has no residual offset contribution on either axis");
    // Only the return leg uses pods. Integrating the fixed lever arms on that
    // leg predicts a nonzero offset term even though both pose and yaw close.
    const double expectedX = .020 * (std::cos(excursion) - 1) + .070 * std::sin(excursion);
    const double expectedY = .020 * std::sin(excursion) + .070 * (1 - std::cos(excursion));
    near(estimate.odometry.offsetX, expectedX, 1e-12,
         "missing pods explain an offset X residual without any rejected increment");
    near(estimate.odometry.offsetY, expectedY, 1e-12,
         "missing pods explain an offset Y residual without an erroneous final heading");

    interrupted.reconfigure(config);
    check(interrupted.estimate().unusableIncrements == estimate.unusableIncrements,
          "geometry reconfiguration preserves unavailable-channel history with odometry history");
    near(interrupted.estimate().odometry.minimumHeadingChange, -excursion, 1e-12,
         "reconfiguration preserves the earlier transient heading excursion");
    interrupted.update(sampleAt(.04, 0));
    check(interrupted.estimate().unusableIncrements == estimate.unusableIncrements,
          "the explicit reconfiguration baseline is excluded from missing-increment counts");
    auto spike = sampleAt(.05, 0);
    spike.forward = spike.lateral = 1;
    spike.gyro = 2;
    auto rejected = interrupted.update(spike);
    check(rejected.unusableIncrements == std::array<std::uint32_t, 3>{3, 3, 1}
              && rejected.rejectedIncrements == 3,
          "implausible pod and gyro jumps are included among unusable increments");
    rejected = interrupted.update(sampleAt(.06, 0));
    check(rejected.unusableIncrements == std::array<std::uint32_t, 3>{4, 4, 2}
              && rejected.rejectedIncrements == 3,
          "rebaselining after rejected jumps is distinct from another rejection");
    interrupted.reset({.1, .2, radians(170)}, .06);
    const auto reset = interrupted.estimate();
    check(reset.unusableIncrements == none && reset.rejectedIncrements == 0,
          "an explicit pose reset clears both persistent diagnostic counter families");
    check(reset.odometry.headingChange == 0 && reset.odometry.minimumHeadingChange == 0
              && reset.odometry.maximumHeadingChange == 0,
          "heading history starts at zero relative to any new absolute pose heading");
    std::printf("closed heading with unreported pod dropout: offset X %.3f Y %.3f mm; rejected %u, unusable F/L/G %u/%u/%u\n",
                expectedX / millimeter, expectedY / millimeter, estimate.rejectedIncrements,
                estimate.unusableIncrements[0], estimate.unusableIncrements[1], estimate.unusableIncrements[2]);
}

void singleEncoderFallbackAndSlowMotion() {
    Simulator sim;
    Estimator estimator(sim.config);
    sim.sample.forwardValid = sim.sample.lateralValid = sim.sample.rightValid = false;
    estimator.update(sim.sample);
    for (int i = 0; i < 400; ++i) {
        const Estimate estimate = estimator.update(sim.step(0.3, 0.7));
        check(distance(estimate.state.pose, sim.truth) < 1e-9, "one drive encoder and gyro observe centre travel");
        check(estimate.health == Health::degraded, "single encoder fallback remains degraded");
    }
    covarianceValid(estimator.estimate());
    Simulator creeping;
    Estimator creepEstimator(creeping.config);
    creepEstimator.update(creeping.sample);
    for (int i = 0; i < 1000; ++i) creepEstimator.update(creeping.step(0.003, 0));
    near(creepEstimator.estimate().state.pose.y, 0.03, 1e-9, "slow final positioning is not mistaken for stationary");

    Simulator verySlow;
    Estimator verySlowEstimator(verySlow.config);
    verySlowEstimator.update(verySlow.sample);
    for (int i = 0; i < 1000; ++i) verySlowEstimator.update(verySlow.step(0.0001, 0.0002));
    near(distance(verySlowEstimator.estimate().state.pose, verySlow.truth), 0, 1e-9,
         "sub-millimetre per second travel is not discarded by a stationary deadband");
    near(wrap(verySlowEstimator.estimate().state.pose.theta - verySlow.truth.theta), 0, 1e-9,
         "very slow real rotation is not frozen or learned as gyro bias");
    near(verySlowEstimator.estimate().gyroBias, 0, 1e-12, "real slow movement cannot train gyro bias");
}

void stationaryUncertaintyAndAsynchronousSamples() {
    Simulator sim;
    sim.gyroBias = radians(0.18);
    Estimator estimator(sim.config);
    estimator.update(sim.sample);
    for (int i = 0; i < 100; ++i) estimator.update(sim.step(0, 0));
    const auto resting = estimator.estimate();
    for (int i = 0; i < 12000; ++i) estimator.update(sim.step(0, 0));
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            near(estimator.estimate().covariance[i][j], resting.covariance[i][j], 1e-12,
                 "confirmed rest does not integrate fictitious odometry uncertainty");
    near(estimator.estimate().confidence, resting.confidence, 1e-12,
         "waiting two minutes at rest preserves confidence without resetting its prior");
    near(estimator.estimate().gyroBias, sim.gyroBias, radians(0.002), "automatic stationary bias learning continues at rest");
    covarianceValid(estimator.estimate());

    Simulator unobservedSideways;
    unobservedSideways.sample.lateralValid = false;
    Estimator missingLateral(unobservedSideways.config);
    missingLateral.update(unobservedSideways.sample);
    for (int i = 0; i < 100; ++i) missingLateral.update(unobservedSideways.step(0, 0));
    const double priorXVariance = missingLateral.estimate().covariance[0][0];
    for (int i = 0; i < 1000; ++i) missingLateral.update(unobservedSideways.step(0, 0));
    check(missingLateral.estimate().covariance[0][0] > priorXVariance + 0.008,
          "without a sideways pod rest still admits unobserved lateral displacement");

    Simulator delayed;
    Estimator asynchronous(delayed.config);
    asynchronous.update(delayed.sample);
    SensorSample held = delayed.sample;
    for (int i = 0; i < 1200; ++i) {
        const auto fresh = delayed.step(0.001, 0.004);
        if (i % 4 == 3) held = fresh; // Devices publish at 25 Hz; loop runs at 100 Hz.
        held.timestamp = fresh.timestamp;
        asynchronous.update(held);
    }
    near(distance(asynchronous.estimate().state.pose, delayed.truth), 0, 1e-9,
         "asynchronous cumulative wheel samples preserve slow travel");
    near(wrap(asynchronous.estimate().state.pose.theta - delayed.truth.theta), 0, 1e-9,
         "short sensor publication gaps do not become false stationary bias evidence");
    near(asynchronous.estimate().gyroBias, 0, 1e-12, "asynchronous slow turn does not train gyro bias");
    std::printf("stationary regression: confidence %.3f -> %.3f over 120 s with all pods; missing lateral sensing still increases uncertainty\n",
                resting.confidence, estimator.estimate().confidence);
}

void stationaryEvidenceMustObserveRotation() {
    Simulator pivoting;
    pivoting.sample.leftValid = pivoting.sample.rightValid = false;
    Estimator estimator(pivoting.config);
    estimator.update(pivoting.sample);
    constexpr double omega = 0.02;
    for (int i = 0; i < 600; ++i) {
        // Rotation about the intersection of the two pod rolling axes leaves
        // both pod counters unchanged, despite real body translation and yaw.
        estimator.update(pivoting.step(pivoting.config.forwardOffset * omega, omega,
                                      -pivoting.config.lateralOffset * omega));
    }
    near(distance(estimator.estimate().state.pose, pivoting.truth), 0, 1e-9,
         "two stationary orthogonal pods do not prove the robot is stationary");
    near(wrap(estimator.estimate().state.pose.theta - pivoting.truth.theta), 0, 1e-9,
         "unobservable pod pivot is not frozen by stationary detection");
    near(estimator.estimate().gyroBias, 0, 1e-12,
         "real pod-pivot rotation cannot be learned as gyro bias");

    Simulator colocated;
    colocated.config.forwardOffset = -colocated.config.trackWidth / 2;
    colocated.sample.lateralValid = colocated.sample.rightValid = false;
    Estimator colocatedEstimator(colocated.config);
    colocatedEstimator.update(colocated.sample);
    for (int i = 0; i < 600; ++i)
        colocatedEstimator.update(colocated.step(colocated.config.forwardOffset * omega, omega));
    near(distance(colocatedEstimator.estimate().state.pose, colocated.truth), 0, 1e-9,
         "coincident forward pod and drive encoder axes do not observe yaw");
    near(colocatedEstimator.estimate().gyroBias, 0, 1e-12,
         "redundant wheel channels cannot train gyro bias during a pivot");

    Simulator independent;
    independent.sample.lateralValid = independent.sample.rightValid = false;
    independent.gyroBias = radians(0.18);
    Estimator independentEstimator(independent.config);
    independentEstimator.update(independent.sample);
    for (int i = 0; i < 800; ++i) independentEstimator.update(independent.step(0, 0));
    near(independentEstimator.estimate().gyroBias, independent.gyroBias, radians(0.006),
         "separated parallel wheel axes still provide independent stationary yaw evidence");
}

void boundedEncoderNoiseAndSubResolutionCreep() {
    EstimatorConfig config;
    config.forwardOffset = .020; config.lateralOffset = -.070; config.trackWidth = .287;
    config.stationaryEncoderSpan = .010 * millimeter;
    // Synthetic quantization in API units; this is not a measured V5 noise or
    // hardware-resolution model (centidegrees describe the return value's units).
    constexpr double podQuantum = pi * 2 * inch / 36000;
    constexpr double motorQuantum = .002 * millimeter;
    Simulator sim;
    sim.config = config;
    sim.gyroBias = radians(.18);
    sim.sample.gyroRateValid = false;
    Estimator estimator(config);
    estimator.setGyroBias(sim.gyroBias, square(radians(.02)));
    estimator.update(sim.sample);
    std::mt19937 random(41023);
    std::uniform_int_distribution<int> jitter(-1, 1);
    Estimate resting;
    for (int i = 1; i <= 12000; ++i) {
        auto sample = sim.step(0, 0);
        sample.forward += jitter(random) * podQuantum;
        sample.lateral += jitter(random) * podQuantum;
        sample.left += jitter(random) * motorQuantum;
        sample.right += jitter(random) * motorQuantum;
        sample.gyro += jitter(random) * radians(.004);
        const auto estimate = estimator.update(sample);
        if (i == 100) resting = estimate;
        if (i > 100) {
            check(estimate.stationary, "bounded one-count wheel noise remains quiet after the dwell");
            const double headingBound = radians(.004) + 4 * motorQuantum / config.trackWidth;
            check(std::abs(estimate.state.pose.theta) < headingBound + 1e-10,
                  "continuous quiet heading stays within gyro-entry and independent wheel endpoint noise");
        }
    }
    const auto quiet = estimator.estimate();
    near(quiet.covariance[2][2], resting.covariance[2][2], 1e-12,
         "bounded wheel jitter cannot accumulate moving heading noise for two minutes");
    check(quiet.covariance[0][0] + quiet.covariance[1][1] <
          resting.covariance[0][0] + resting.covariance[1][1] + square(.001),
          "bounded wheel jitter cannot accumulate centimetres of fictitious position noise");
    check(distance(quiet.state.pose, {}) < .000030,
          "retaining quiet raw pod increments keeps stationary position within bounded measurement noise");
    near(quiet.gyroBias, sim.gyroBias, radians(.004), "quiet bias windows tolerate quantized motor and pod noise");
    covarianceValid(quiet);
    std::printf("bounded wheel jitter at rest: sigma %.2f -> %.2f mm, %.3f -> %.3f deg over 119 s; final pose %.4f %.4f mm\n",
        std::sqrt(resting.covariance[0][0] + resting.covariance[1][1]) / millimeter,
        std::sqrt(quiet.covariance[0][0] + quiet.covariance[1][1]) / millimeter,
        degrees(std::sqrt(resting.covariance[2][2])), degrees(std::sqrt(quiet.covariance[2][2])),
        quiet.state.pose.x / millimeter, quiet.state.pose.y / millimeter);

    for (double speed : {.0001, .00001, .000002}) {
        Simulator creep;
        creep.config = config;
        creep.sample.gyroRateValid = false;
        Estimator measured(config);
        measured.setGyroBias(0, square(radians(.02)));
        measured.update(creep.sample);
        for (int i = 0; i < 12000; ++i)
            measured.update(creep.step(speed, speed * 2, speed / 2));
        const auto result = measured.estimate();
        near(distance(result.state.pose, creep.truth), 0, 1e-9,
             "bounded-rest detection preserves accumulated secular translation below its resolution");
        near(wrap(result.state.pose.theta - creep.truth.theta), 0, 1e-9,
             "independent quiet wheel yaw preserves genuine very slow rotation");
        near(result.gyroBias, 0, 1e-12, "slow wheel yaw is subtracted before learning gyro bias");
        near(result.odometry.forwardX + result.odometry.lateralX + result.odometry.offsetX,
             result.state.pose.x, 1e-10, "quiet creep remains included in exact odometry diagnostics");
    }

    Simulator creeping;
    creeping.config = config;
    creeping.sample.gyroRateValid = false;
    Estimator measured(config);
    measured.setGyroBias(0, square(radians(.01)));
    measured.update(creeping.sample);
    bool wasQuiet = false;
    unsigned quietIntervals = 0;
    double biasAngleBound = 0;
    for (int i = 0; i < 12000; ++i) {
        auto sample = creeping.step(.00001, .000002, .000005);
        sample.forward = std::round(sample.forward / podQuantum) * podQuantum + jitter(random) * podQuantum;
        sample.lateral = std::round(sample.lateral / podQuantum) * podQuantum + jitter(random) * podQuantum;
        sample.left = std::round(sample.left / motorQuantum) * motorQuantum + jitter(random) * motorQuantum;
        sample.right = std::round(sample.right / motorQuantum) * motorQuantum + jitter(random) * motorQuantum;
        const double previousBias = measured.estimate().gyroBias;
        const auto estimate = measured.update(sample);
        if (estimate.stationary && !wasQuiet) ++quietIntervals;
        if (!estimate.stationary) biasAngleBound += std::abs(previousBias) * .01;
        wasQuiet = estimate.stationary;
    }
    const auto result = measured.estimate();
    const double error = distance(result.state.pose, creeping.truth);
    const double headingError = std::abs(wrap(result.state.pose.theta - creeping.truth.theta));
    // Each noisy motor position differs from truth by at most 1.5 quanta:
    // half a quantization step plus the injected one-quantum noise. A quiet
    // interval uses two differences of two positions, hence four endpoint
    // errors. This is a deterministic bound, not an independence assumption.
    const double headingBound = quietIntervals * 4 * 1.5 * motorQuantum / config.trackWidth + biasAngleBound;
    std::printf("quantized noisy 10 um/s creep over 120 s: position error %.4f mm, heading error %.6f deg (%.6f deg bound, %u quiet intervals), bias %.6f deg/s\n",
        error / millimeter, degrees(headingError), degrees(headingBound), quietIntervals, degrees(result.gyroBias));
    check(error < .000050, "quantized noisy sub-resolution creep retains accumulated travel within 50 um");
    check(quietIntervals > 0 && headingError <= headingBound + 1e-10,
          "quiet/noisy switching error respects the independently derived encoder quantization bound");
    check(std::abs(result.gyroBias) < radians(.002), "quantized wheel jitter cannot train a large spurious gyro bias");
    covarianceValid(result);

    config.forwardOffset = -config.trackWidth / 2 + .000001;
    Estimator illConditioned(config);
    Simulator nearlyColocated;
    nearlyColocated.config = config;
    nearlyColocated.sample.rightValid = false;
    nearlyColocated.sample.gyroRateValid = false;
    illConditioned.update(nearlyColocated.sample);
    for (int i = 0; i < 100; ++i) {
        auto sample = nearlyColocated.step(0, 0);
        sample.forward += (i % 2) * podQuantum;
        sample.left -= (i % 2) * motorQuantum;
        const auto estimate = illConditioned.update(sample);
        check(!estimate.stationary, "nearly coincident noisy wheel axes cannot supply a trusted quiet yaw constraint");
        near(estimate.state.pose.theta, 0, 1e-12, "ill-conditioned quiet geometry cannot manufacture heading from wheel noise");
    }
}

void fiftyMillisecondAcquisition() {
    EstimatorConfig config;
    config.nativeImuHeading = true;
    config.stationaryEncoderSpan = .010 * millimeter;
    config.stationaryMaxInterval = .075;
    Estimator estimator(config);
    SensorSample sample;
    sample.forwardValid = sample.lateralValid = sample.leftValid = sample.rightValid = sample.gyroValid = true;
    estimator.update(sample);
    for (unsigned i = 0; i < 40; ++i) {
        sample.timestamp += i % 2 ? .049 : .051;
        const auto result = estimator.update(sample);
        if (i >= 8) check(result.stationary, "50 ms acquisition tolerates scheduling jitter during quiet");
        check(result.quiet.breaks == 0, "ordinary 49/51 ms samples do not report acquisition pauses");
    }
    sample.timestamp += .080;
    auto result = estimator.update(sample);
    check(!result.stationary && result.quiet.lastBreakMask == 32,
          "a real gap above 75 ms still interrupts quiet at 20 Hz");
    for (unsigned i = 0; i < 20; ++i) {
        sample.timestamp += .050;
        sample.forward += .025;
        sample.left += .025;
        sample.right += .025;
        result = estimator.update(sample);
    }
    near(result.state.pose.y, .5, 1e-10, "20 Hz samples integrate measured distance without a dt scale error");
    check(!result.stationary && result.rejectedIncrements == 0,
          "normal movement remains valid and leaves quiet at 20 Hz");
}

void quietBreakDiagnostics() {
    EstimatorConfig config;
    config.forwardOffset = .020; config.lateralOffset = -.070; config.trackWidth = .287;
    config.stationaryEncoderSpan = .010 * millimeter;
    const auto baseSample = [] {
        SensorSample sample;
        sample.forwardValid = sample.lateralValid = sample.leftValid = sample.rightValid = sample.gyroValid = true;
        return sample;
    };
    const auto advance = [](Estimator& estimator, SensorSample& sample, unsigned count) {
        for (unsigned i = 0; i < count; ++i) { sample.timestamp += .01; estimator.update(sample); }
    };
    const auto prime = [&](Estimator& estimator, SensorSample& sample) {
        estimator.update(sample);
        advance(estimator, sample, 30);
        check(estimator.estimate().stationary && estimator.estimate().quiet.breaks == 0,
              "initial quiet baseline is not counted as a broken window");
    };
    Estimator estimator(config);
    auto sample = baseSample();
    prime(estimator, sample);
    const double podStep = 3 * pi * 2 * inch / 36000;
    sample.forward += podStep;
    advance(estimator, sample, 1);
    auto quiet = estimator.estimate().quiet;
    check(quiet.lastBreakMask == 1 && quiet.breaks == 1 && !estimator.estimate().stationary,
          "three Rotation counts expose a forward-pod quiet-window restart");
    near(quiet.wheelSpans[0], podStep, 1e-12, "quiet diagnostic retains the pre-reset pod span");
    near(quiet.encoderSpan, config.stationaryEncoderSpan, 1e-12, "diagnostic snapshots contain the configured threshold");
    near(quiet.timestamp, sample.timestamp, 1e-12, "quiet diagnostic records the actual failing sample time");
    near(quiet.dwell, 0, 1e-12, "a broken quiet window restarts its dwell");
    advance(estimator, sample, 40);
    check(estimator.estimate().stationary && estimator.estimate().quiet.lastBreakMask == 1 &&
          estimator.estimate().quiet.breaks == 1, "the cause remains visible after quiet returns");
    near(estimator.estimate().quiet.wheelSpans[0], podStep, 1e-12,
         "later unchanged samples cannot erase the measured failure span");

    // The public reported-angle accuracy is not a claim about returned sample
    // quantization. These synthetic 0.02 degree steps test cumulative bounds.
    const double motorStep = .02 / 360 * pi * 3.25 * inch * .6;
    sample.left += motorStep;
    advance(estimator, sample, 1);
    check(estimator.estimate().stationary, "one 0.02 degree motor-position step fits the cumulative quiet bound");
    sample.left += motorStep;
    advance(estimator, sample, 1);
    quiet = estimator.estimate().quiet;
    check(quiet.lastBreakMask == 4 && quiet.breaks == 2,
          "two small motor increments identify the left-wheel cumulative span as the cause");
    near(quiet.wheelSpans[2], 2 * motorStep, 1e-12, "motor span is measured across the whole quiet window");
    estimator.reconfigure(config);
    check(estimator.estimate().quiet.breaks == 2 && estimator.estimate().quiet.lastBreakMask == 4 &&
          estimator.estimate().quiet.dwell == 0, "reconfigure preserves failure history and clears only the current dwell");
    estimator.update(sample);
    advance(estimator, sample, 30);
    check(estimator.estimate().quiet.breaks == 2, "reconfigure baselines do not manufacture a mask-change failure");
    estimator.reset({});
    check(estimator.estimate().quiet.breaks == 0 && estimator.estimate().quiet.lastBreakMask == 0 &&
          estimator.estimate().quiet.wheelSpans == std::array<double, 4>{},
          "known-pose reset clears quiet failure history");
    near(estimator.estimate().quiet.encoderSpan, config.stationaryEncoderSpan, 1e-12,
         "known-pose reset retains the configured diagnostic threshold");

    for (unsigned cause = 0; cause < 7; ++cause) {
        Estimator diagnosed(config);
        auto changed = baseSample();
        prime(diagnosed, changed);
        double interval = .01;
        std::uint16_t expected = 0;
        switch (cause) {
        case 0: changed.gyro += radians(4) * interval; expected = 16; break;
        case 1: interval = .051; expected = 32; break;
        case 2: changed.lateralValid = false; expected = 128; break;
        case 3: changed.gyroValid = false; expected = 64; break;
        case 4: changed.leftValid = changed.rightValid = false; expected = 64 | 128; break;
        case 5: changed.forward = 1; expected = 256; break;
        default: interval = 1.1; expected = 32; break;
        }
        changed.timestamp += interval;
        const auto result = diagnosed.update(changed);
        check((result.quiet.lastBreakMask & expected) == expected && result.quiet.breaks == 1,
              "quiet restart distinguishes gyro, timing, availability, geometry and rejected-increment causes");
        near(result.quiet.sampleInterval, interval, 1e-12, "quiet restart captures its actual sampling interval");
        if (cause == 0) near(result.quiet.gyroRate, radians(4), 1e-12,
                             "quiet restart reports the gyro angle derivative that tripped the gate");
        const auto repeated = diagnosed.update(changed);
        check(repeated.quiet.breaks == result.quiet.breaks,
              "an ignored duplicate sensor timestamp cannot increment quiet diagnostics");
    }

    config.forwardOffset = -config.trackWidth / 2 + .000001;
    Estimator geometry(config);
    sample = baseSample();
    sample.rightValid = false;
    geometry.update(sample);
    advance(geometry, sample, 1);
    check(geometry.estimate().quiet.lastBreakMask == 64,
          "ill-conditioned yaw geometry is diagnosed without blaming gyro-rate noise");
}

void nativeImuHeadingPolicy() {
    EstimatorConfig config;
    config.forwardOffset = .020; config.lateralOffset = -.070; config.trackWidth = .287;
    config.stationaryEncoderSpan = .010 * millimeter;
    check(!config.nativeImuHeading, "the general estimator retains the existing bias policy by default");
    Simulator sim;
    sim.config = config;
    Estimator estimator(config);
    check(estimator.setGyroBias(radians(.008567), square(radians(.09))),
          "legacy bias fixture reproduces a previously applied local correction");
    estimator.update(sim.sample);
    for (int i = 0; i < 100; ++i) estimator.update(sim.step(.2, .1));
    const auto before = estimator.estimate();
    config.nativeImuHeading = true;
    estimator.reconfigure(config);
    near(distance(estimator.estimate().state.pose, before.state.pose), 0, 1e-12,
         "switching to native heading preserves the accumulated position");
    near(estimator.estimate().state.pose.theta, before.state.pose.theta, 1e-12,
         "switching to native heading preserves the accumulated heading");
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            near(estimator.estimate().covariance[i][j], before.covariance[i][j], 1e-12,
                 "native policy does not manufacture a better pose prior");
    check(estimator.estimate().gyroBias == 0 && estimator.estimate().gyroBiasStd == 0,
          "native policy reports zero effective local bias and no active bias estimate");
    check(!estimator.setGyroBias(radians(.18), square(radians(.02))),
          "native policy rejects an otherwise valid manual bias calibration");
    estimator.update(sim.sample); // Reconfigure establishes a new raw baseline.
    for (int i = 0; i < 6000; ++i) estimator.update(sim.step(0, 0));
    const auto still = estimator.estimate();
    check(still.stationary, "native heading retains the independent quiet diagnostic");
    near(distance(still.state.pose, before.state.pose), 0, 1e-12,
         "an old local bias cannot manufacture offset translation with stable native IMU readings");
    near(still.state.pose.theta, before.state.pose.theta, 1e-12,
         "an old local bias cannot manufacture heading drift with stable native IMU readings");
    near(still.covariance[2][2] - before.covariance[2][2], 6000 * square(config.gyroStd), 1e-10,
         "native quiet heading keeps IMU process noise without a dormant bias Jacobian");
    covarianceValid(still);
    estimator.reset({.3, -.2, .4});
    check(estimator.estimate().gyroBias == 0 && estimator.estimate().gyroBiasStd == 0,
          "native pose reset cannot resurrect discarded local calibration");
    config.nativeImuHeading = false;
    estimator.reconfigure(config);
    near(estimator.estimate().gyroBias, 0, 1e-12, "leaving native mode starts without a stale bias");
    near(estimator.estimate().gyroBiasStd, radians(2), 1e-12,
         "leaving native mode restores the standard uncalibrated bias prior");
    check(estimator.setGyroBias(radians(.1), square(radians(.02))),
          "explicit local bias calibration works again when the policy is enabled");

    // Wheels can certify a bounded quiet interval while the IMU still changes.
    // Native mode must preserve that change and its geometric offset correction,
    // through both legacy raw-rate and cumulative-window bias-learning paths.
    config.nativeImuHeading = true;
    config.gyroScale = 1.07;
    for (double span : {0.0, .010 * millimeter}) {
        config.stationaryEncoderSpan = span;
        Estimator quiet(config), noWheelEvidence(config);
        Simulator sensor;
        sensor.config = config;
        auto sample = sensor.sample;
        quiet.update(sample);
        auto withoutDrive = sample;
        withoutDrive.leftValid = withoutDrive.rightValid = false;
        noWheelEvidence.update(withoutDrive);
        unsigned quietFrames = 0;
        for (int i = 1; i <= 1200; ++i) {
            sample.timestamp = i * .01;
            sample.gyro = radians(.02) * sample.timestamp;
            sample.gyroRate = radians(.02);
            const auto observed = quiet.update(sample);
            withoutDrive = sample;
            withoutDrive.leftValid = withoutDrive.rightValid = false;
            const auto reference = noWheelEvidence.update(withoutDrive);
            quietFrames += observed.stationary;
            const double angle = sample.gyro * config.gyroScale;
            // Integrals of constant body travel per radian, independent of the
            // estimator's incremental sinc/cosc implementation.
            const double expectedX = -config.lateralOffset * std::sin(angle)
                + config.forwardOffset * (1 - std::cos(angle));
            const double expectedY = config.forwardOffset * std::sin(angle)
                + config.lateralOffset * (1 - std::cos(angle));
            near(observed.state.pose.theta, angle, 1e-11, "quiet native heading follows the scaled IMU delta");
            near(observed.state.pose.x, expectedX, 1e-11, "native yaw retains the exact lateral-offset correction");
            near(observed.state.pose.y, expectedY, 1e-11, "native yaw retains the exact forward-offset correction");
            near(distance(observed.state.pose, reference.state.pose), 0, 1e-12,
                 "quiet diagnostics cannot change native integration compared with absent wheel evidence");
            check(observed.gyroBias == 0 && observed.gyroBiasStd == 0,
                  "neither raw-rate nor windowed quiet evidence learns bias in native mode");
        }
        check(quietFrames > 1100, "native regression actually exercises sustained quiet");
        near(quiet.estimate().covariance[2][2] - square(radians(.5)), 1200 * square(config.gyroStd), 1e-11,
             "quiet and active native heading use the same IMU heading noise density");
    }

    config.gyroScale = 1;
    for (bool loseImu : {false, true}) {
        Simulator moving;
        moving.config = config;
        moving.sample.gyroValid = !loseImu;
        moving.sample.forwardValid = loseImu;
        moving.sample.rightValid = loseImu;
        // Either IMU + one drive encoder, or both drives without IMU.
        Estimator fallback(config);
        fallback.update(moving.sample);
        for (int i = 0; i < 600; ++i) fallback.update(moving.step(.2, .15, .01));
        near(distance(fallback.estimate().state.pose, moving.truth), 0, 1e-9,
             "native policy preserves encoder/IMU translation fallbacks and their rotation offsets");
        near(wrap(fallback.estimate().state.pose.theta - moving.truth.theta), 0, 1e-9,
             "native policy preserves drive-encoder heading fallback when IMU is unavailable");
        check(fallback.estimate().headingSource == (loseImu ? HeadingSource::driveEncoders : HeadingSource::imu),
              "native diagnostics identify the heading source actually used");
    }
}

void explicitCalibrationPreservesPrior() {
    Simulator sim;
    Estimator estimator(sim.config);
    estimator.update(sim.sample);
    for (int i = 0; i < 300; ++i) estimator.update(sim.step(0.2, 0.2));
    const auto before = estimator.estimate();
    check(!estimator.setGyroBias(std::numeric_limits<double>::quiet_NaN(), 0.01), "nonfinite calibrated bias rejected");
    check(!estimator.setGyroBias(radians(4), 0.01), "implausible calibrated bias rejected");
    check(!estimator.setGyroBias(0, 0), "zero calibrated uncertainty rejected");
    check(estimator.setGyroBias(radians(0.18), square(radians(0.02))), "verified gyro calibration accepted");
    near(distance(before.state.pose, estimator.estimate().state.pose), 0, 1e-12, "gyro calibration does not move pose");
    near(estimator.estimate().confidence, before.confidence, 1e-12, "gyro calibration does not manufacture pose confidence");
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            near(estimator.estimate().covariance[i][j], before.covariance[i][j], 1e-12,
                 "gyro calibration preserves the full pose uncertainty marginal");
    sim.gyroBias = radians(0.18);
    for (int i = 0; i < 300; ++i) estimator.update(sim.step(0.2, 0.2));
    near(wrap(estimator.estimate().state.pose.theta - sim.truth.theta), 0, 1e-9,
         "verified bias is used by subsequent moving integration");
    covarianceValid(estimator.estimate());

    const auto oldCalibration = estimator.estimate();
    sim.config.forwardOffset = -0.055;
    sim.config.lateralOffset = 0.110;
    sim.config.trackWidth = 0.310;
    estimator.reconfigure(sim.config);
    near(distance(estimator.estimate().state.pose, oldCalibration.state.pose), 0, 1e-12,
         "physical reconfiguration preserves pose");
    near(estimator.estimate().gyroBias, sim.gyroBias, 1e-12, "physical reconfiguration preserves learned bias");
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            near(estimator.estimate().covariance[i][j], oldCalibration.covariance[i][j], 1e-12,
                 "physical reconfiguration does not reset accumulated uncertainty");
    sim.sample.forward += 6;
    sim.sample.lateral -= 4;
    sim.sample.gyro += 12 * pi;
    estimator.update(sim.sample);
    near(distance(estimator.estimate().state.pose, oldCalibration.state.pose), 0, 1e-12,
         "reconfiguration rebaselines changed cumulative sensor readings");
    for (int i = 0; i < 300; ++i) estimator.update(sim.step(0.2, 0.2));
    near(distance(estimator.estimate().state.pose, sim.truth), 0, 1e-9,
         "new physical geometry applies to later increments without a pose jump");
    covarianceValid(estimator.estimate());
}

EstimatorConfig rangeConfig() {
    EstimatorConfig config;
    config.rangeCount = 2;
    config.mounts[0] = {0, 0.08, 0};
    config.mounts[1] = {0.08, 0, pi / 2};
    // Idealized, separately calibrated sensors for geometry/replay tests.
    // The raw V5 default (5% relative error) is tested independently below.
    config.rangeStd = 0.010;
    config.rangeRelativeStd = 0;
    return config;
}

void rangesAndDelay() {
    const EstimatorConfig config = rangeConfig();
    Estimator rangeEstimator(config), bareEstimator(config);
    Simulator sim;
    sim.config = config;
    rangeEstimator.update(sim.sample);
    bareEstimator.update(sim.sample);
    const auto begin = std::chrono::steady_clock::now();
    double worstMicroseconds = 0, lateralDrift = 0;
    for (int i = 0; i < 900; ++i) {
        SensorSample sample = sim.step(0.15, 0);
        sample.forward *= 1.06; // A deliberately miscalibrated tracking wheel.
        lateralDrift += 0.004 * 0.01;
        sample.lateral += lateralDrift; // Independent slow drift on the sideways pod.
        SensorSample bare = sample;
        bare.ranges = {};
        bareEstimator.update(bare);
        if (i >= 20 && i % 5 == 0) {
            const double delay = 0.085; // Also exercises interval interpolation.
            const double acquired = sample.timestamp - delay;
            const double trueY = 0.15 * acquired;
            sample.ranges[0] = {config.field.maxY - trueY - 0.08, acquired, 63, true};
            sample.ranges[1] = {config.field.maxX - 0.08, acquired, 63, true};
            for (std::size_t sensor = 0; sensor < config.rangeCount; ++sensor)
                check(sample.ranges[sensor].distance >= 0.02 && sample.ranges[sensor].distance <= 2.0,
                      "both perpendicular Distance beams stay inside their physical measurement range");
        }
        const auto tick = std::chrono::steady_clock::now();
        rangeEstimator.update(sample);
        worstMicroseconds = std::max(worstMicroseconds,
            std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - tick).count());
    }
    const double corrected = distance(rangeEstimator.estimate().state.pose, sim.truth);
    const double uncorrected = distance(bareEstimator.estimate().state.pose, sim.truth);
    check(rangeEstimator.estimate().acceptedRanges > 100, "trustworthy delayed walls accepted");
    check(corrected < 0.025 && corrected < uncorrected / 3, "delayed wall correction removes scale drift");
    const auto correctedPose = rangeEstimator.estimate().state.pose;
    const auto barePose = bareEstimator.estimate().state.pose;
    check(std::abs(correctedPose.x - sim.truth.x) < std::abs(barePose.x - sim.truth.x) / 3,
          "two perpendicular wall beams correct independent X odometry drift");
    check(std::abs(correctedPose.y - sim.truth.y) < std::abs(barePose.y - sim.truth.y) / 3,
          "two perpendicular wall beams correct independent Y odometry drift");
    covarianceValid(rangeEstimator.estimate());
    const double elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
    std::printf("wall drift simulation: no correction %.2f mm, delayed correction %.2f mm; 900 paired updates %.2f ms; worst range update %.2f us (host only)\n",
                1000 * uncorrected, 1000 * corrected, elapsed, worstMicroseconds);
    std::printf("two-Distance X/Y correction: X %.2f -> %.2f mm, Y %.2f -> %.2f mm\n",
                1000 * std::abs(barePose.x - sim.truth.x), 1000 * std::abs(correctedPose.x - sim.truth.x),
                1000 * std::abs(barePose.y - sim.truth.y), 1000 * std::abs(correctedPose.y - sim.truth.y));

    // A dynamic obstacle must not teleport a previously coherent estimate.
    const Pose before = rangeEstimator.estimate().state.pose;
    const auto acceptedBefore = rangeEstimator.estimate().acceptedRanges;
    for (int i = 0; i < 20; ++i) {
        SensorSample sample = sim.step(0, 0);
        sample.forward *= 1.06;
        sample.lateral += lateralDrift;
        sample.ranges[0] = {0.04, sample.timestamp, 63, true};
        rangeEstimator.update(sample);
    }
    check(rangeEstimator.estimate().acceptedRanges == acceptedBefore, "near obstacle rejected");
    check(distance(before, rangeEstimator.estimate().state.pose) < 1e-6, "obstacle cannot snap pose");
    SensorSample stale = sim.step(0, 0);
    stale.forward *= 1.06;
    stale.lateral += lateralDrift;
    stale.ranges[0] = {1.12, 0.1, 63, true};
    rangeEstimator.update(stale);
    near(distance(before, rangeEstimator.estimate().state.pose), 0, 1e-6, "out-of-history data ignored");

    for (int i = 0; i < 50; ++i) {
        auto sample = sim.step(0, 0);
        sample.forward *= 1.06;
        sample.lateral += lateralDrift;
        rangeEstimator.update(sample);
        if (i >= 40) {
            check(std::abs(rangeEstimator.estimate().state.v) < 0.02 &&
                  std::abs(rangeEstimator.estimate().state.omega) < radians(1),
                  "two-beam recovery limitation is tested after the robot has settled");
            check(rangeEstimator.estimate().headingSource == HeadingSource::imu &&
                  rangeEstimator.estimate().covariance[2][2] < square(radians(8)),
                  "two-beam recovery limitation is tested with a trusted IMU heading");
            sample.ranges[0] = {config.field.maxY - sim.truth.y - 0.08, sample.timestamp, 63, true};
            sample.ranges[1] = {config.field.maxX - sim.truth.x - 0.08, sample.timestamp, 63, true};
            const auto prior = rangeEstimator.estimate().state.pose;
            check(!rangeEstimator.relocalize(sample),
                  "two Distance beams support local correction but do not authorize global relocalization");
            near(distance(prior, rangeEstimator.estimate().state.pose), 0, 1e-12,
                 "insufficient two-beam global geometry leaves the existing position prior unchanged");
        }
    }
    covarianceValid(rangeEstimator.estimate());
}

void malformedAndSingleReading() {
    const EstimatorConfig config = rangeConfig();
    Estimator estimator(config);
    Simulator sim;
    sim.config = config;
    estimator.update(sim.sample);
    for (int i = 0; i < 100; ++i) estimator.update(sim.step(0.1, 0));
    const Pose before = estimator.estimate().state.pose;
    SensorSample sample = sim.step(0, 0);
    sample.ranges[0] = {config.field.maxY - 0.15, sample.timestamp, 63, true};
    estimator.update(sample);
    check(estimator.estimate().acceptedRanges == 0, "single plausible wall cannot move pose");
    near(distance(before, estimator.estimate().state.pose), 0, 1e-9, "single range is corroborated first");
    sample.timestamp -= 1;
    sample.forward += 1000;
    estimator.update(sample);
    near(distance(before, estimator.estimate().state.pose), 0, 1e-9, "out-of-order odometry ignored");
    sample.timestamp = std::numeric_limits<double>::quiet_NaN();
    estimator.update(sample);
    near(distance(before, estimator.estimate().state.pose), 0, 1e-9, "NaN timestamp ignored");
    sample = sim.step(0.1, 0);
    sample.forward = std::numeric_limits<double>::infinity();
    sample.gyro = std::numeric_limits<double>::quiet_NaN();
    estimator.update(sample);
    check(std::isfinite(estimator.estimate().state.pose.y), "nonfinite channels use valid fallback");
    covarianceValid(estimator.estimate());

    Estimator diagonal(config);
    diagonal.reset({0, 0, pi / 4});
    sim = Simulator{};
    diagonal.update(sim.sample);
    for (int i = 0; i < 20; ++i) {
        sample = sim.step(0, 0);
        sample.ranges[0] = {std::sqrt(2.0) * config.field.maxY - 0.08, sample.timestamp, 63, true};
        diagonal.update(sample);
    }
    check(diagonal.estimate().acceptedRanges == 0, "ambiguous field corner rejected");

    sample.timestamp = 1e200;
    estimator.update(sample);
    check(estimator.estimate().health == Health::lost, "long clock blackout loses trusted pose");
    covarianceValid(estimator.estimate());
}

void corroboratedRecovery() {
    EstimatorConfig config = rangeConfig();
    config.rangeStd = 0.025;
    config.rangeRelativeStd = 0.05; // Exercise recovery with conservative raw V5 defaults.
    config.rangeCount = 3;
    config.mounts[0].x = 0.12;
    config.mounts[1].y = 0.09;
    config.mounts[2] = {-0.06, -0.08, pi};
    Estimator estimator(config);
    Simulator sim;
    sim.config = config;
    sim.truth = {0.6, 0.1, 0}; // Unobserved displacement with all V5 ranges <2 m.
    estimator.update(sim.sample);
    for (int i = 0; i < 80; ++i) estimator.update(sim.step(0, 0));
    bool recovered = false;
    for (int i = 0; i < 20 && !recovered; ++i) {
        SensorSample sample = sim.step(0, 0, 0, 0.05);
        estimator.update(sample);
        sample.ranges[0] = {config.field.maxY - sim.truth.y - 0.08, sample.timestamp, 63, true};
        sample.ranges[1] = {config.field.maxX - sim.truth.x - 0.08, sample.timestamp, 63, true};
        sample.ranges[2] = {sim.truth.y - config.field.minY - 0.08, sample.timestamp, 63, true};
        for (std::size_t sensor = 0; sensor < 3; ++sensor)
            check(sample.ranges[sensor].distance >= 0.02 && sample.ranges[sensor].distance <= 2,
                  "recovery uses hardware-plausible V5 distances");
        const Pose before = estimator.estimate().state.pose;
        recovered = estimator.relocalize(sample);
        check(distance(before, estimator.estimate().state.pose) <= config.maxRangeCorrection + 1e-10,
              "recovery increments are bounded");
        if (i < 4) near(distance(before, estimator.estimate().state.pose), 0, 1e-9,
                        "recovery requires five independent frames");
    }
    check(recovered, "three-wall stationary hypothesis converges");
    near(distance(estimator.estimate().state.pose, sim.truth), 0, 1e-9, "recovery pose accuracy");
    covarianceValid(estimator.estimate());
    const auto recoveredCovariance = estimator.estimate().covariance;
    check(std::abs(recoveredCovariance[0][2]) > 1e-7 && std::abs(recoveredCovariance[1][2]) > 1e-7,
          "recovery preserves common heading correlation from offset wall beams");
    near(recoveredCovariance[0][2] / recoveredCovariance[2][2], -0.09, 1e-9,
         "shared heading uncertainty is propagated through the side beam geometry");

    Estimator obstacle(config);
    obstacle.update(SensorSample{});
    sim = Simulator{};
    for (int i = 0; i < 100; ++i) {
        SensorSample sample = sim.step(0, 0);
        obstacle.update(sample);
        sample.ranges[0] = {0.5, sample.timestamp, 63, true}; // Cannot pair with rear wall.
        sample.ranges[1] = {config.field.maxX - 0.08, sample.timestamp, 63, true};
        sample.ranges[2] = {-config.field.minY - 0.08, sample.timestamp, 63, true};
        check(!obstacle.relocalize(sample), "inconsistent wall recovery rejected");
    }
    near(distance(obstacle.estimate().state.pose, {}), 0, 1e-9, "obstacle recovery never snaps");
}

void recoveryRequiresFreshEvidenceFromEveryBeam() {
    EstimatorConfig config = rangeConfig();
    config.rangeCount = 3;
    config.mounts[2] = {0, -0.08, pi};
    Estimator estimator(config);
    Simulator sim;
    sim.config = config;
    const Pose displaced{0.06, 0.04, 0};
    estimator.update(sim.sample);
    for (int i = 0; i < 100; ++i) estimator.update(sim.step(0, 0));
    const double heldTimestamp = sim.sample.timestamp + 0.01;
    const auto addRanges = [&](SensorSample& sample, double front, double right, double rear) {
        sample.ranges[0] = {config.field.maxY - displaced.y - 0.08, front, 63, true};
        sample.ranges[1] = {config.field.maxX - displaced.x - 0.08, right, 63, true};
        sample.ranges[2] = {displaced.y - config.field.minY - 0.08, rear, 63, true};
    };
    for (int i = 0; i < 5; ++i) {
        auto sample = sim.step(0, 0);
        estimator.update(sample);
        addRanges(sample, sample.timestamp - 0.04, heldTimestamp, heldTimestamp);
        check(!estimator.relocalize(sample), "two held beams cannot corroborate five recovery frames");
        near(distance(estimator.estimate().state.pose, {}), 0, 1e-12,
             "repeated recovery evidence cannot move the pose");
    }
    bool recovered = false;
    for (int i = 0; i < 8 && !recovered; ++i) {
        auto sample = sim.step(0, 0);
        estimator.update(sample);
        // All beams are fresh, even though this frame's oldest acquisition
        // precedes the previous frame's newest acquisition.
        addRanges(sample, sample.timestamp - 0.03, sample.timestamp - 0.015, sample.timestamp);
        recovered = estimator.relocalize(sample);
    }
    check(recovered, "fresh asynchronous beams still permit corroborated recovery");
    near(distance(estimator.estimate().state.pose, displaced), 0, 1e-10,
         "fresh recovery frames restore the measured position");
    covarianceValid(estimator.estimate());
}

void acquisitionBlackoutRequiresNewPosePrior() {
    Simulator sim;
    Estimator estimator(sim.config);
    estimator.update(sim.sample);
    estimator.update(sim.step(0.5, 0.4, 0, 1.2));
    check(estimator.estimate().health == Health::lost, "long acquisition gap loses the global pose");
    for (int i = 0; i < 80; ++i) {
        const auto estimate = estimator.update(sim.step(0, 0));
        check(estimate.health == Health::lost && estimate.confidence == 0,
              "fresh sensors cannot reconstruct travel missed during an acquisition blackout");
    }
    check(estimator.setGyroBias(0, square(radians(0.02))), "post-blackout bias calibration accepted");
    estimator.reconfigure(sim.config);
    for (int i = 0; i < 30; ++i) estimator.update(sim.step(0, 0));
    check(estimator.estimate().health == Health::lost && estimator.estimate().confidence == 0,
          "bias calibration and geometry changes cannot repair a lost pose prior");
    estimator.reset(sim.truth, sim.sample.timestamp);
    estimator.update(sim.sample);
    estimator.update(sim.step(0.2, 0.1));
    check(estimator.estimate().health == Health::healthy, "a known pose restores localization after blackout");
    near(distance(estimator.estimate().state.pose, sim.truth), 0, 1e-10,
         "odometry resumes accurately from the new pose prior");
    covarianceValid(estimator.estimate());
}

void rawDistanceSystematicUncertainty() {
    EstimatorConfig config;
    config.rangeCount = 1;
    config.mounts[0] = {0, 0.08, 0};
    Simulator sim;
    sim.config = config;
    Estimator estimator(config);
    estimator.update(sim.sample);
    const double distanceToWall = config.field.maxY - 0.08;
    const double biasedRange = distanceToWall * 1.05;
    const double floor = square(config.rangeStd) + square(config.rangeRelativeStd * biasedRange);
    for (int i = 0; i < 1000; ++i) {
        SensorSample sample = sim.step(0, 0);
        if (i % 5 == 0) sample.ranges[0] = {biasedRange, sample.timestamp, 63, true};
        estimator.update(sample);
    }
    near(estimator.estimate().state.pose.y, 0, 1e-9,
         "raw far-wall data cannot override a more precise stationary odometry prior");
    check(estimator.estimate().covariance[1][1] < floor,
          "no-information wall update preserves a prior better than the sensor's systematic floor");
    // First accumulate actual odometry uncertainty by travelling back and forth
    // inside the field. The old test generated it merely by waiting at rest.
    for (int i = 0; i < 6000; ++i) estimator.update(sim.step((i / 100) % 2 == 0 ? 0.2 : -0.2, 0));
    for (int i = 0; i < 10000; ++i) {
        SensorSample sample = sim.step(0, 0);
        if (i % 5 == 0) sample.ranges[0] = {biasedRange, sample.timestamp, 63, true};
        estimator.update(sample);
    }
    check(estimator.estimate().acceptedRanges > 1000, "systematic uncertainty test accepts corroborated walls");
    const auto estimate = estimator.estimate();
    const double cosine = std::cos(estimate.state.pose.theta);
    const double predictedRange = (config.field.maxY - estimate.state.pose.y - 0.08 * cosine) / cosine;
    const double hY = -1 / cosine;
    const double hHeading = std::tan(estimate.state.pose.theta) * (0.08 + predictedRange);
    const double rayVariance = square(hY) * estimate.covariance[1][1]
        + 2 * hY * hHeading * estimate.covariance[1][2] + square(hHeading) * estimate.covariance[2][2];
    check(rayVariance >= floor / square(cosine) - 1e-8,
          "repeated biased raw ranges retain the systematic covariance floor");
    check(std::abs(estimator.estimate().state.pose.y) <= std::sqrt(estimator.estimate().covariance[1][1]),
          "systematic range bias remains covered by reported position uncertainty");
    covarianceValid(estimator.estimate());
    std::printf("raw V5 5%% biased wall: position error %.2f mm, reported Y sigma %.2f mm after 2,000 readings\n",
        1000 * std::abs(estimator.estimate().state.pose.y),
        1000 * std::sqrt(estimator.estimate().covariance[1][1]));
}

void incompleteRecoveryRemainsLostBetweenFrames() {
    EstimatorConfig config = rangeConfig();
    config.rangeCount = 3;
    config.mounts[2] = {0, -0.08, pi};
    Estimator estimator(config);
    Simulator sim;
    sim.config = config;
    sim.truth = {0.6, 0.1, 0};
    estimator.update(sim.sample);
    for (int i = 0; i < 80; ++i) estimator.update(sim.step(0, 0));
    const auto addRanges = [&](SensorSample sample) {
        sample.ranges[0] = {config.field.maxY - sim.truth.y - 0.08, sample.timestamp, 63, true};
        sample.ranges[1] = {config.field.maxX - sim.truth.x - 0.08, sample.timestamp, 63, true};
        sample.ranges[2] = {sim.truth.y - config.field.minY - 0.08, sample.timestamp, 63, true};
        return sample;
    };
    SensorSample lastFrame;
    for (int i = 0; i < 5; ++i) {
        const auto sample = sim.step(0, 0, 0, 0.05);
        estimator.update(sample);
        lastFrame = addRanges(sample);
        check(!estimator.relocalize(lastFrame), "large recovery does not converge in one bounded correction");
    }
    check(distance(estimator.estimate().state.pose, {}) > 0.09,
          "five corroborated frames begin bounded recovery");
    check(estimator.estimate().health == Health::lost, "partial recovery marks pose lost");
    auto repeated = sim.step(0, 0);
    repeated.ranges = lastFrame.ranges;
    estimator.update(repeated);
    check(estimator.estimate().health == Health::lost,
          "repeated range frame cannot restore health during incomplete recovery");
    for (int i = 0; i < 40; ++i) {
        estimator.update(sim.step(0, 0));
        check(estimator.estimate().health == Health::lost,
              "missing range frames keep incomplete recovery lost");
        near(estimator.estimate().confidence, 0, 1e-12,
             "partial recovery never publishes trusted confidence between wall observations");
    }
    check(estimator.setGyroBias(0, square(radians(0.02))), "bias calibration accepted during recovery");
    estimator.update(sim.step(0, 0));
    check(estimator.estimate().health == Health::lost,
          "bias-only calibration cannot certify an incompletely recovered position");
    Estimator suppliedPrior = estimator;
    suppliedPrior.reset(sim.truth, sim.sample.timestamp);
    suppliedPrior.update(sim.sample);
    auto next = sim.sample;
    next.timestamp += 0.01;
    suppliedPrior.update(next);
    check(suppliedPrior.estimate().health == Health::healthy,
          "an explicit pose prior clears incomplete recovery");
    estimator.reconfigure(config);
    estimator.update(sim.sample);
    estimator.update(sim.step(0, 0));
    check(estimator.estimate().health == Health::lost,
          "geometry reconfiguration preserves incomplete recovery of the pose prior");
    bool recovered = false;
    for (int i = 0; i < 20 && !recovered; ++i) {
        const auto sample = sim.step(0, 0, 0, 0.05);
        estimator.update(sample);
        recovered = estimator.relocalize(addRanges(sample));
    }
    check(recovered, "fresh corroborated wall frames finish interrupted recovery");
    near(distance(estimator.estimate().state.pose, sim.truth), 0, 1e-9,
         "interrupted recovery converges to the wall-supported position");
    estimator.update(sim.step(0, 0));
    check(estimator.estimate().health == Health::healthy,
          "completed recovery allows normal health publication again");
}

void lateralVelocityObservation() {
    Simulator sim;
    sim.config.nativeImuHeading = true;
    sim.config.forwardOffset = .020;
    sim.config.lateralOffset = -.033;
    Estimator estimator(sim.config);
    estimator.update(sim.sample);
    for (int i = 0; i < 200; ++i) estimator.update(sim.step(0, 0, .18));
    near(estimator.estimate().state.v, 0, 1e-10, "sideways sliding does not invent forward speed");
    near(estimator.estimate().state.lateralV, .18, 1e-8,
         "pure sideways sliding remains observable with stopped drive wheels");
    near(estimator.estimate().state.pose.x, .36, 1e-10,
         "sideways speed filtering does not change cumulative pose travel");
    for (int i = 0; i < 200; ++i) estimator.update(sim.step(0, 1.6));
    near(estimator.estimate().state.lateralV, 0, 1e-8,
         "pure spin subtracts lateral pod offset before estimating lateral speed");
    for (int i = 0; i < 200; ++i) estimator.update(sim.step(-.4, -1.0, -.14));
    near(estimator.estimate().state.lateralV, -.14, 1e-8,
         "negative lateral speed is retained during reverse turning");

    // Lost sensor readings cannot turn a reconnecting cumulative counter into
    // a fictitious velocity impulse. The next valid reading is only a baseline.
    for (int i = 0; i < 100; ++i) {
        auto sample = sim.step(0, 0);
        sample.lateralValid = false;
        estimator.update(sample);
    }
    const auto reconnected = estimator.update(sim.step(0, 0));
    check(std::abs(reconnected.state.lateralV) < .001,
          "missing lateral data and reconnect never invent side speed");
    for (int i = 0; i < 200; ++i) estimator.update(sim.step(0, 0, .12));
    near(estimator.estimate().state.lateralV, .12, 1e-8, "lateral speed recovers after sensor reconnect");
    estimator.reset(sim.truth, sim.sample.timestamp);
    near(estimator.estimate().state.lateralV, 0, 0, "pose reset clears lateral speed");
    estimator.update(sim.sample);
    for (int i = 0; i < 200; ++i) estimator.update(sim.step(0, 0, .12));
    estimator.reconfigure(sim.config);
    near(estimator.estimate().state.lateralV, 0, 0, "geometry reconfigure clears lateral speed");
    estimator.update(sim.sample);
    for (int i = 0; i < 200; ++i) estimator.update(sim.step(0, 0, .12));
    sim.sample.timestamp += 1.1;
    const auto blackout = estimator.update(sim.sample);
    near(blackout.state.lateralV, 0, 0, "sensor blackout clears unobservable lateral speed");
    check(blackout.health == Health::lost, "lateral speed observation does not conceal lost pose prior");
}

void velocityAccuracyWithNoiseAndMissingSamples() {
    struct Errors { double oldV = 0, newV = 0, oldOmega = 0, newOmega = 0; unsigned count = 0; };
    const auto run = [](bool dynamic, bool noise, unsigned publicationPeriod, bool dropout, bool jitter) {
        EstimatorConfig config;
        Estimator estimator(config);
        SensorSample sample;
        sample.forwardValid = sample.lateralValid = sample.leftValid = sample.rightValid = sample.gyroValid = true;
        estimator.update(sample);
        SensorSample previous = sample;
        double time = 0, legacyV = 0, legacyOmega = 0;
        std::mt19937 generator(82917);
        std::normal_distribution<double> podNoise(0, 0.0002), gyroNoise(0, radians(0.015));
        Errors errors;
        for (unsigned i = 1; i <= 1800; ++i) {
            const double dt = jitter ? (i % 3 == 0 ? 0.014 : i % 3 == 1 ? 0.006 : 0.010) : 0.010;
            time += dt;
            const double v = dynamic ? 0.8 * std::sin(2 * time) : 0.7;
            const double omega = dynamic ? 1.4 * std::sin(1.7 * time) : 0.3;
            if (i % publicationPeriod == 0) {
                // Integrate the analytic velocity functions independently of
                // both estimator and Simulator, then synthesize raw counters.
                const double travel = dynamic ? 0.4 * (1 - std::cos(2 * time)) : 0.7 * time;
                const double heading = dynamic ? 1.4 / 1.7 * (1 - std::cos(1.7 * time)) : 0.3 * time;
                sample.forward = travel - config.forwardOffset * heading + (noise ? podNoise(generator) : 0);
                sample.lateral = config.lateralOffset * heading + (noise ? podNoise(generator) : 0);
                sample.left = travel + config.trackWidth * heading / 2;
                sample.right = travel - config.trackWidth * heading / 2;
                sample.gyro = heading + (noise ? gyroNoise(generator) : 0);
            }
            sample.timestamp = time;
            sample.forwardValid = !(dropout && i >= 300 && i < 400);
            sample.gyroValid = !(dropout && i >= 600 && i < 700);
            const auto estimate = estimator.update(sample);
            const double angular = sample.gyroValid && previous.gyroValid ? sample.gyro - previous.gyro :
                (sample.left - previous.left - sample.right + previous.right) / config.trackWidth;
            const double forward = sample.forwardValid && previous.forwardValid ?
                sample.forward - previous.forward + config.forwardOffset * angular :
                (sample.left - previous.left + sample.right - previous.right) / 2;
            const double alpha = -std::expm1(-dt / 0.035);
            legacyV += alpha * (forward / dt - legacyV);
            legacyOmega += alpha * (angular / dt - legacyOmega);
            previous = sample;
            if (i > 100) {
                errors.oldV += square(legacyV - v); errors.newV += square(estimate.state.v - v);
                errors.oldOmega += square(legacyOmega - omega); errors.newOmega += square(estimate.state.omega - omega);
                ++errors.count;
            }
            check(std::isfinite(estimate.state.v) && std::isfinite(estimate.state.omega),
                  "velocity filtering stays finite with asynchronous data and sensor dropout");
        }
        errors.oldV = std::sqrt(errors.oldV / errors.count); errors.newV = std::sqrt(errors.newV / errors.count);
        errors.oldOmega = std::sqrt(errors.oldOmega / errors.count); errors.newOmega = std::sqrt(errors.newOmega / errors.count);
        return errors;
    };
    const auto moving = run(true, true, 1, false, false);
    check(moving.newV < 0.4 * moving.oldV && moving.newOmega < 0.4 * moving.oldOmega,
          "lag-compensated velocity improves noisy acceleration RMS by at least 60 percent");
    const auto fallback = run(true, true, 1, true, true);
    check(fallback.newV < 0.5 * fallback.oldV && fallback.newOmega < 0.5 * fallback.oldOmega,
          "velocity improvement survives irregular timing and pod or gyro fallback");
    const auto constant = run(false, true, 1, false, false);
    check(constant.newV < 1.10 * constant.oldV && constant.newOmega < 1.10 * constant.oldOmega,
          "lag compensation keeps constant-speed sensor-noise RMS within ten percent");
    const auto asynchronous = run(true, true, 4, false, false);
    check(asynchronous.newV < 1.10 * asynchronous.oldV && asynchronous.newOmega < 1.10 * asynchronous.oldOmega,
          "held 25 Hz sensor counters do not destabilize compensated velocity");
    std::printf("velocity comparison (old EMA -> compensated): acceleration RMS %.5f -> %.5f m/s, %.5f -> %.5f rad/s; noise-only %.5f -> %.5f m/s; 25 Hz held samples %.5f -> %.5f m/s\n",
        moving.oldV, moving.newV, moving.oldOmega, moving.newOmega, constant.oldV, constant.newV,
        asynchronous.oldV, asynchronous.newV);
}

void delayedRangesDoNotChangeVelocityFiltering() {
    EstimatorConfig config = rangeConfig();
    config.rangeCount = 1;
    config.rangeStd = 1.0; // Deliberately uninformative range factors split replay without changing the pose.
    Estimator delayed(config), bare(config);
    SensorSample sample;
    sample.forwardValid = sample.lateralValid = sample.leftValid = sample.rightValid = sample.gyroValid = true;
    delayed.update(sample); bare.update(sample);
    for (int i = 1; i <= 400; ++i) {
        sample.timestamp = i * 0.01;
        sample.forward = sample.left = sample.right = 0.2 * sample.timestamp + (1 - std::cos(3 * sample.timestamp)) / 30;
        sample.lateral = 0.05 * std::sin(2 * sample.timestamp);
        sample.ranges = {};
        const auto withoutReplay = bare.update(sample);
        if (i > 20 && i % 5 == 0) {
            const double acquired = sample.timestamp - 0.017;
            const double position = 0.2 * acquired + (1 - std::cos(3 * acquired)) / 30;
            sample.ranges[0] = {config.field.maxY - config.mounts[0].y - position, acquired, 63, true};
        }
        const auto withReplay = delayed.update(sample);
        near(withReplay.state.v, withoutReplay.state.v, 1e-11,
             "splitting odometry intervals for delayed observations preserves velocity filtering");
        near(withReplay.state.lateralV, withoutReplay.state.lateralV, 1e-11,
             "delayed replay preserves lateral velocity filtering across fractional intervals");
        near(withReplay.state.pose.x, withoutReplay.state.pose.x, 1e-11,
             "delayed replay preserves measured lateral pose travel");
        near(withReplay.state.pose.y, withoutReplay.state.pose.y, 1e-11,
             "uninformative delayed constraints preserve the position prior");
        near(withReplay.odometry.forwardY, withoutReplay.odometry.forwardY, 1e-11,
             "delayed replay does not double-count diagnostic travel");
    }
    check(delayed.estimate().acceptedRanges > 60, "velocity replay regression actually inserts delayed factors");
}

void perSensorRangeNoiseAndInvalidOverrides() {
    EstimatorConfig config = rangeConfig();
    config.rangeCount = 2;
    config.mounts[1] = config.mounts[0]; // Identical geometry isolates the noise model.
    config.rangeStd = 1.0;
    config.rangeRelativeStd = 0.25;
    config.rangeNoise[0] = {0.010, 0}; // Only this sensor has independently verified calibration.
    Estimator calibrated(config), inherited(config);
    EstimatorConfig invalid = config;
    invalid.rangeNoise[1] = {0, std::numeric_limits<double>::quiet_NaN()};
    Estimator invalidOverride(invalid);
    // This isolates the range model after a verified stationary gyro-bias
    // calibration; one wall cannot independently resolve heading/bias drift.
    const double biasVariance = square(radians(0.02));
    calibrated.setGyroBias(0, biasVariance);
    inherited.setGyroBias(0, biasVariance);
    invalidOverride.setGyroBias(0, biasVariance);
    Simulator sim;
    sim.config = config;
    calibrated.update(sim.sample); inherited.update(sim.sample); invalidOverride.update(sim.sample);
    for (int i = 1; i <= 700; ++i) {
        auto sample = sim.step(0.15, 0);
        sample.forward *= 1.06;
        if (i % 5 == 0)
            sample.ranges[0] = {config.field.maxY - config.mounts[0].y - sim.truth.y, sample.timestamp, 63, true};
        calibrated.update(sample);
        sample.ranges[1] = sample.ranges[0];
        sample.ranges[0] = {};
        const auto globalModel = inherited.update(sample);
        const auto invalidModel = invalidOverride.update(sample);
        near(distance(globalModel.state.pose, invalidModel.state.pose), 0, 1e-12,
             "invalid per-sensor noise inherits the conservative global model");
        for (std::size_t j = 0; j < 3; ++j)
            near(globalModel.covariance[j][j], invalidModel.covariance[j][j], 1e-12,
                 "invalid per-sensor noise cannot manufacture higher confidence");
    }
    const auto corrected = calibrated.estimate(), conservative = inherited.estimate();
    const double calibratedError = distance(corrected.state.pose, sim.truth);
    const double inheritedError = distance(conservative.state.pose, sim.truth);
    std::printf("per-sensor wall calibration model: verified 10 mm noise %.2f mm drift versus inherited conservative model %.2f mm (synthetic calibrated readings)\n",
                1000 * calibratedError, 1000 * inheritedError);
    check(calibratedError < 0.020 && calibratedError < inheritedError / 3,
          "one calibrated wall sensor corrects drift without changing another sensor's noise prior");
    const double cosine = std::cos(corrected.state.pose.theta);
    const double predictedRange = (config.field.maxY - corrected.state.pose.y - config.mounts[0].y * cosine) / cosine;
    const double hY = -1 / cosine;
    const double hHeading = std::tan(corrected.state.pose.theta) * (config.mounts[0].y + predictedRange);
    const double rayVariance = square(hY) * corrected.covariance[1][1] +
        2 * hY * hHeading * corrected.covariance[1][2] + square(hHeading) * corrected.covariance[2][2];
    check(rayVariance >= square(config.rangeNoise[0].standardDeviation) / square(cosine) - 1e-8,
          "per-sensor calibrated walls preserve their nonzero systematic uncertainty floor");
    covarianceValid(corrected); covarianceValid(conservative); covarianceValid(invalidOverride.estimate());
}

void closeRangesWithoutHardwareConfidence() {
    EstimatorConfig config;
    config.rangeCount = 1;
    config.mounts[0] = {0, 0.08, 0};
    Estimator close(config), weak(config), invalid(config);
    const Pose nearWall{0, config.field.maxY - config.mounts[0].y - 0.15, 0};
    close.reset(nearWall); weak.reset(nearWall); invalid.reset(nearWall);
    Simulator sim;
    close.update(sim.sample); weak.update(sim.sample); invalid.update(sim.sample);
    for (int i = 0; i < 20; ++i) {
        auto sample = sim.step(0, 0);
        sample.ranges[0] = {0.15, sample.timestamp, 10, true, false};
        close.update(sample);
        if (i < 4) check(close.estimate().acceptedRanges == 0,
                         "a close return without confidence requires five corroborating frames");
        if (i == 4) check(close.estimate().acceptedRanges == 1,
                          "the fifth coherent close return may be used conservatively");
        sample.ranges[0] = {0.30, sample.timestamp, 10, true, true};
        weak.update(sample);
        sample.ranges[0] = {0.15, sample.timestamp, -1, true, false};
        invalid.update(sample);
    }
    check(close.estimate().acceptedRanges > 10, "valid close wall readings remain usable without hardware confidence");
    check(weak.estimate().acceptedRanges == 0, "available low confidence beyond 200 mm remains rejected");
    check(invalid.estimate().acceptedRanges == 0, "missing confidence cannot hide a sensor error sentinel");
    covarianceValid(close.estimate());

    // Even a numerically high placeholder cannot authorize global recovery.
    config.field = {-0.25, 0.25, -0.25, 0.25};
    config.rangeCount = 3;
    config.mounts[1] = {0.08, 0, pi / 2};
    config.mounts[2] = {0, -0.08, pi};
    Estimator recovery(config);
    recovery.reset({0.04, 0.02, 0});
    sim = Simulator{};
    recovery.update(sim.sample);
    for (int i = 0; i < 40; ++i) recovery.update(sim.step(0, 0));
    const auto prior = recovery.estimate();
    for (int i = 0; i < 10; ++i) {
        auto sample = sim.step(0, 0, 0, 0.05);
        recovery.update(sample);
        for (std::size_t sensor = 0; sensor < 3; ++sensor)
            sample.ranges[sensor] = {0.17, sample.timestamp, 63, true, false};
        check(!recovery.relocalize(sample), "global recovery still requires actual high-confidence wall beams");
    }
    near(distance(recovery.estimate().state.pose, prior.state.pose), 0, 1e-12,
         "unavailable confidence never replaces the global pose prior");
}
}

int main() {
    imuMountingStatusAndWheelSlip();
    continuousTurnsAndConfidence();
    exactOdometry();
    relocatedPodsClosedPath();
    dropoutReconnect();
    rejectedCounterCannotBecomeMotionOnReturn();
    slowForwardReverseWithoutAbsoluteSensors();
    boundedCachedSensorIncrements();
    closedManualCyclesAndSensorTiming();
    independentArcAndPollTiming();
    persistentOdometryDiagnosticsWithMissingPods();
    biasNoiseAndSlip();
    singleEncoderFallbackAndSlowMotion();
    stationaryUncertaintyAndAsynchronousSamples();
    stationaryEvidenceMustObserveRotation();
    boundedEncoderNoiseAndSubResolutionCreep();
    fiftyMillisecondAcquisition();
    quietBreakDiagnostics();
    nativeImuHeadingPolicy();
    explicitCalibrationPreservesPrior();
    rangesAndDelay();
    malformedAndSingleReading();
    acquisitionBlackoutRequiresNewPosePrior();
    corroboratedRecovery();
    recoveryRequiresFreshEvidenceFromEveryBeam();
    rawDistanceSystematicUncertainty();
    incompleteRecoveryRemainsLostBetweenFrames();
    lateralVelocityObservation();
    velocityAccuracyWithNoiseAndMissingSamples();
    delayedRangesDoNotChangeVelocityFiltering();
    perSensorRangeNoiseAndInvalidOverrides();
    closeRangesWithoutHardwareConfidence();
    std::printf("Estimator: %u checks passed; sizeof(Estimator)=%zu bytes\n", checks, sizeof(nexus::Estimator));
    return 0;
}
