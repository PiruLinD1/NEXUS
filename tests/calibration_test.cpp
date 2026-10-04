#include "nexus/calibration.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <vector>

namespace {
using namespace nexus;
using namespace nexus::calibration;
void require(bool value, const char* message) {
    if (value) return;
    std::cerr << "FAILED: " << message << '\n'; std::exit(EXIT_FAILURE);
}
bool near(double a, double b, double tolerance) { return std::abs(a - b) <= tolerance; }
std::vector<WheelSample> wheelLog(double kS, double kV, double kA) {
    std::vector<WheelSample> samples;
    // Independently excited speed/acceleration from diverse transient segments.
    for (int direction : {-1, 1}) for (int speed = 1; speed <= 18; ++speed)
        for (int acceleration = -4; acceleration <= 4; ++acceleration) {
            const double v = direction * (0.1 + speed * 0.065);
            const double a = acceleration * 0.42;
            const double noise = 0.012 * std::sin(0.79 * samples.size());
            double u = kS * direction + kV * v + kA * a + noise;
            if (samples.size() % 31 == 0) u += 2.2;
            samples.push_back({u, v, a, true});
        }
    return samples;
}
void wheelFits() {
    const WheelFit left = fitWheel(wheelLog(0.61, 5.8, 0.72));
    const WheelFit right = fitWheel(wheelLog(0.47, 6.2, 0.84));
    require(left.quality.accepted && right.quality.accepted, "bidirectional excited log accepted");
    require(near(left.kS, 0.61, 0.01) && near(left.kV, 5.8, 0.01) && near(left.kA, 0.72, 0.01),
            "robust left fit recovers physical coefficients despite 3 percent voltage outliers");
    require(near(right.kS, 0.47, 0.01) && near(right.kV, 6.2, 0.01) && near(right.kA, 0.84, 0.01),
            "right fit preserves drivetrain asymmetry");
    require(left.quality.outliers > 0 && left.quality.inlierRms < 0.02,
            "fit reports outlier count and measured residuals");

    std::vector<WheelSample> unexcited;
    for (int i = 0; i < 60; ++i) unexcited.push_back({6, 1, 0, true});
    require(fitWheel(unexcited).quality.failure == FitFailure::insufficientExcitation,
            "constant speed cannot identify static or acceleration terms");
    std::vector<WheelSample> singular;
    for (int sign : {-1, 1}) for (int i = 0; i < 100; ++i) {
        const double v = sign * (0.1 + i * 0.009);
        singular.push_back({0.6*sign + 6*v + 0.7*2*v, v, 2*v, true});
    }
    require(fitWheel(singular).quality.failure == FitFailure::illConditioned,
            "collinear velocity and acceleration are unobservable");
    require(!fitWheel(wheelLog(-0.5, 5.8, 0.7)).quality.accepted,
            "nonphysical negative friction rejected");
    auto invalid = wheelLog(0.6, 6, 0.7);
    for (auto& sample : invalid) sample.voltage = std::numeric_limits<double>::quiet_NaN();
    require(fitWheel(invalid).quality.failure == FitFailure::insufficientData,
            "nonfinite samples cannot contaminate a model");
    auto contaminated = wheelLog(0.6, 6, 0.7);
    for (std::size_t i = 0; i < contaminated.size(); ++i)
        contaminated[i].voltage += (i % 2 == 0 ? 2.4 : -2.4);
    require(!fitWheel(contaminated).quality.accepted, "high model mismatch rejected");
    std::vector<WheelSample> oneWay;
    for (const auto& sample : wheelLog(0.6, 6, 0.7))
        if (sample.velocity > 0) oneWay.push_back(sample);
    for (int i = 0; i < 5; ++i)
        oneWay.push_back({-0.6 - 6 * (0.2 + i * 0.03) + 3, -(0.2 + i * 0.03), 0, true});
    const auto oneWayFit = fitWheel(oneWay);
    require(!oneWayFit.quality.accepted && oneWayFit.quality.failure == FitFailure::insufficientExcitation,
            "rejected reverse samples cannot satisfy bidirectional wheel excitation");
}
void geometryFits() {
    std::vector<ReferenceSample> travel;
    for (int sign : {-1, 1}) for (int i = 0; i < 10; ++i) {
        const double distance = sign * (0.5 + 0.11 * i);
        travel.push_back({distance, distance * 1.031 + 0.0005 * std::sin(i)});
    }
    travel[3].reference += 0.25;
    const auto scale = fitTravelScale(travel);
    require(scale.quality.accepted && near(scale.value, 1.031, 0.001),
            "travel scale recovered from independent physical reference");
    std::vector<RotationSample> rotations;
    for (int sign : {-1, 1}) for (int i = 0; i < 9; ++i) {
        const double angle = sign * (0.7 + 0.30 * i);
        rotations.push_back({angle, angle / 1.012, -0.029 * angle / 1.031,
                            0.054 * angle / 0.982, 0.302 * angle / 2, -0.302 * angle / 2});
    }
    const auto geometry = fitRotation(rotations, 1.031, 0.982);
    require(geometry.gyroScale.quality.accepted && near(geometry.gyroScale.value, 1.012, 1e-10),
            "gyro scale uses independent angle");
    require(geometry.forwardOffset.quality.accepted && near(geometry.forwardOffset.value, 0.029, 1e-10),
            "clockwise forward pod offset sign");
    require(geometry.lateralOffset.quality.accepted && near(geometry.lateralOffset.value, 0.054, 1e-10),
            "clockwise lateral pod offset sign");
    require(geometry.trackWidth.quality.accepted && near(geometry.trackWidth.value, 0.302, 1e-10),
            "effective track width uses left minus right");
    auto drifting = rotations;
    for (auto& sample : drifting) {
        const double center = 0.004 * sample.referenceAngle;
        sample.leftTravel += center; sample.rightTravel += center;
        sample.forwardTravel += center / 1.031;
    }
    const auto driftGeometry = fitRotation(drifting, 1.031, 0.982);
    require(driftGeometry.forwardOffset.quality.accepted && near(driftGeometry.forwardOffset.value, 0.029, 1e-10),
            "offline rotation corrects measured small center drift instead of changing forward pod offset");
    auto arcs = rotations;
    for (auto& sample : arcs) {
        const double center = 0.04 * sample.referenceAngle;
        sample.leftTravel += center; sample.rightTravel += center;
        sample.forwardTravel += center / 1.031;
    }
    const auto arcGeometry = fitRotation(arcs, 1.031, 0.982);
    require(!arcGeometry.forwardOffset.quality.accepted && !arcGeometry.lateralOffset.quality.accepted &&
            arcGeometry.forwardOffset.quality.failure == FitFailure::moving &&
            arcGeometry.lateralOffset.quality.failure == FitFailure::moving,
            "offline arcs cannot masquerade as valid pod geometry");
    require(arcGeometry.gyroScale.quality.accepted && arcGeometry.trackWidth.quality.accepted,
            "arc rejection preserves independently observable gyro scale and track width");
    auto isolatedArcs = rotations;
    isolatedArcs[2] = arcs[2]; isolatedArcs[12] = arcs[12];
    const auto isolatedGeometry = fitRotation(isolatedArcs, 1.031, 0.982);
    require(isolatedGeometry.forwardOffset.quality.accepted &&
            near(isolatedGeometry.forwardOffset.value, 0.029, 1e-10) &&
            isolatedGeometry.forwardOffset.quality.outliers == 2,
            "isolated arcs are counted and excluded from an otherwise excited pure-turn capture");
    auto noDrive = rotations;
    for (auto& sample : noDrive) sample.leftTravel = sample.rightTravel = std::numeric_limits<double>::quiet_NaN();
    const auto noDriveGeometry = fitRotation(noDrive, 1.031, 0.982);
    require(noDriveGeometry.forwardOffset.quality.accepted && noDriveGeometry.lateralOffset.quality.accepted &&
            near(noDriveGeometry.forwardOffset.value, 0.029, 1e-10) && !noDriveGeometry.trackWidth.quality.accepted,
            "externally verified pure rotations still calibrate pods when drive sensors are absent");
    for (auto& sample : rotations) sample.lateralValid = false;
    const auto missing = fitRotation(rotations);
    require(!missing.lateralOffset.quality.accepted && missing.gyroScale.quality.accepted,
            "missing pod does not prevent other geometry fits");
    travel.resize(5);
    require(!fitTravelScale(travel).quality.accepted, "short single-direction travel log rejected");
    travel.clear();
    for (int i = 0; i < 20; ++i) travel.push_back({0.5 + 0.1 * i, 1.03 * (0.5 + 0.1 * i)});
    travel.push_back({-0.5, -0.5 * 1.03 + 0.2});
    travel.push_back({-0.7, -0.7 * 1.03 + 0.2});
    const auto oneWay = fitTravelScale(travel);
    require(!oneWay.quality.accepted && oneWay.quality.failure == FitFailure::insufficientExcitation,
            "rejected reverse samples cannot satisfy bidirectional travel excitation");
}
void distanceFits() {
    std::vector<ReferenceSample> samples;
    for (int distance = 0; distance < 9; ++distance) for (int repeat = 0; repeat < 6; ++repeat) {
        const double raw = 0.10 + 0.18 * distance;
        double reference = 1.035 * raw - 0.014 + 0.0005 * std::sin(samples.size());
        if (repeat == 0 && distance % 2 == 0) reference += 0.150;
        samples.push_back({raw, reference});
    }
    samples.push_back({std::numeric_limits<double>::quiet_NaN(), 0.5});
    samples.push_back({9999 * millimeter, 0.5});
    samples.push_back({0.5, -0.1});
    const auto fit = fitDistance(samples);
    require(fit.quality.accepted && near(fit.scale, 1.035, 0.001) && near(fit.offset, -0.014, 0.001),
            "distance affine fit recovers separate scale and offset despite outliers");
    require(fit.quality.samples == 54 && fit.quality.outliers == 5 && fit.quality.inlierRms < .001,
            "distance fit excludes invalid readings and reports residuals/outliers");
    require(near(fit.measuredMin, .1, 1e-10) && near(fit.measuredMax, 1.54, 1e-10),
            "distance fit reports the raw inlier domain in metres");
    double rawError = 0, correctedError = 0;
    constexpr int holdoutCount = 15;
    for (int i = 0; i < holdoutCount; ++i) {
        // Independently located targets/noise, generated from physical truth
        // through the sensor model rather than copied from the fitted rows.
        const double truth = .23 + .083 * i;
        const double raw = (truth + .014) / 1.035 + .001 * std::cos(.91 * i + .37);
        rawError += square(raw - truth);
        correctedError += square(fit.scale * raw + fit.offset - truth);
    }
    const double rawRms = std::sqrt(rawError / holdoutCount);
    const double correctedRms = std::sqrt(correctedError / holdoutCount);
    require(rawRms > .015 && correctedRms < .0015 && correctedRms < .1 * rawRms,
            "distance correction improves unseen synthetic target distances with independent noise");
    std::cout << "Synthetic distance holdout: raw RMS=" << rawRms / millimeter
              << " mm, corrected RMS=" << correctedRms / millimeter << " mm\n";

    std::vector<ReferenceSample> repeated(30, {.8, .83});
    require(fitDistance(repeated).quality.failure == FitFailure::insufficientExcitation,
            "one wall distance cannot separate scale and offset");
    std::vector<ReferenceSample> twoDistances;
    for (int i = 0; i < 15; ++i) {
        twoDistances.push_back({.2, .2}); twoDistances.push_back({1.2, 1.2});
    }
    require(fitDistance(twoDistances).quality.failure == FitFailure::insufficientExcitation,
            "two isolated wall distances lack intermediate validation coverage");
    auto narrow = samples;
    for (auto& sample : narrow) {
        sample.measured = 0.5 + 0.01 * sample.measured;
        sample.reference = sample.measured;
    }
    require(!fitDistance(narrow).quality.accepted, "narrow distance interval cannot calibrate a useful scale");
    samples.resize(11);
    require(fitDistance(samples).quality.failure == FitFailure::insufficientData,
            "short distance dataset rejected");

    std::vector<ReferenceSample> badScale, badOffset, nonlinear;
    for (int distance = 0; distance < 9; ++distance) for (int repeat = 0; repeat < 4; ++repeat) {
        const double raw = 0.10 + 0.18 * distance;
        badScale.push_back({raw, 0.7 * raw});
        badOffset.push_back({raw, raw + .2});
        nonlinear.push_back({raw, raw + (distance % 2 ? 0.080 : -0.080)});
    }
    require(fitDistance(badScale).quality.failure == FitFailure::implausible,
            "distance scale outside hardware plausibility rejected");
    require(fitDistance(badOffset).quality.failure == FitFailure::implausible,
            "distance offset outside hardware plausibility rejected");
    require(!fitDistance(nonlinear).quality.accepted,
            "persistent nonlinear distance error cannot masquerade as an affine calibration");

    std::vector<ReferenceSample> falseCoverage;
    for (int i = 0; i < 30; ++i) {
        falseCoverage.push_back({.2, .2}); falseCoverage.push_back({1.6, 1.6});
    }
    for (int i = 0; i < 3; ++i) falseCoverage.push_back({.9, 1.1});
    const auto uncovered = fitDistance(falseCoverage);
    require(!uncovered.quality.accepted && uncovered.quality.failure == FitFailure::insufficientExcitation,
            "outliers cannot manufacture intermediate distance coverage");
}
void storage() {
    const std::string path = "nexus_calibration_test.cfg";
    std::remove(path.c_str()); std::remove((path + ".bak").c_str()); std::remove((path + ".tmp").c_str());
    CalibrationProfile p;
    EstimatorConfig estimator;
    DynamicsConfig dynamics;
    require(!applyProfile(p, estimator, dynamics), "factory values are explicitly uncalibrated");
    p.hardwareFingerprint = 0x5341575032303236ULL;
    require(saveProfile(path, p) == StorageStatus::uncalibrated, "uncalibrated profile cannot masquerade as calibrated");
    p.calibratedMask = forwardScaleField | leftDynamicsField;
    p.forwardScale = 1.024; p.kSLeft = 0.58; p.kVLeft = 5.91; p.kALeft = 0.71;
    require(applyProfile(p, estimator, dynamics), "partial valid calibration applies");
    require(estimator.forwardScale == 1.024 && dynamics.kVLeft == 5.91 && dynamics.kVRight == 6,
            "only calibrated fields applied");
    require(saveProfile(path, p) == StorageStatus::ok, "initial profile saved");
    CalibrationProfile loaded;
    require(loadProfile(path, p.hardwareFingerprint, loaded) == StorageStatus::ok && loaded.kSLeft == p.kSLeft,
            "portable format round trip");
    CalibrationProfile unchanged;
    unchanged.hardwareFingerprint = 777;
    require(loadProfile(path, 123, unchanged) == StorageStatus::wrongHardware && unchanged.hardwareFingerprint == 777,
            "wrong robot profile rejected without modifying output");
    p.forwardScale = 1.04;
    require(saveProfile(path, p) == StorageStatus::ok, "second save retains backup");
    {
        std::fstream file(path, std::ios::in | std::ios::out | std::ios::binary);
        file.seekp(30); file.put('\x7f');
    }
    bool recovered = false;
    require(loadProfile(path, p.hardwareFingerprint, loaded, &recovered) == StorageStatus::ok && recovered &&
            loaded.forwardScale == 1.024, "CRC corruption recovers last known-good backup");
    std::remove((path + ".bak").c_str());
    require(loadProfile(path, p.hardwareFingerprint, unchanged) == StorageStatus::checksumMismatch && unchanged.hardwareFingerprint == 777,
            "corrupt file cannot partially overwrite configuration");
    { std::ofstream file(path, std::ios::binary | std::ios::trunc); file << "VXMOCAL1"; }
    require(loadProfile(path, p.hardwareFingerprint, unchanged) == StorageStatus::checksumMismatch,
            "truncated file rejected");
    p.gyroScale = std::numeric_limits<double>::infinity();
    require(saveProfile(path, p) == StorageStatus::invalidProfile, "nonfinite physical configuration rejected");
    require(!applyProfile(p, estimator, dynamics) && estimator.forwardScale == 1.024,
            "invalid profile changes no live fields");
    std::remove(path.c_str()); std::remove((path + ".bak").c_str()); std::remove((path + ".tmp").c_str());
}
} // namespace
int main() {
    wheelFits(); geometryFits(); distanceFits(); storage();
    std::cout << "Calibration fits, observability, residual rejection and persistence tests passed\n";
    return EXIT_SUCCESS;
}
