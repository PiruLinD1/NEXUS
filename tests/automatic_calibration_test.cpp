#include "nexus/automatic_calibration.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

namespace {
using namespace nexus;
using namespace nexus::calibration;
void require(bool value, const char* message) {
    if (value) return;
    std::cerr << "FAILED: " << message << '\n'; std::exit(EXIT_FAILURE);
}
bool near(double a, double b, double tolerance) { return std::abs(a - b) < tolerance; }
SensorSample sample() {
    SensorSample s;
    s.timestamp = 10; s.gyro = -8 * pi;
    s.gyroValid = s.leftValid = s.rightValid = true;
    s.forwardValid = s.lateralValid = true;
    return s;
}
void stationary() {
    StationaryCalibration capture;
    capture.reset(1.012);
    for (int i = 0; i <= 400; ++i) {
        auto s = sample(); s.timestamp += 0.01 * i;
        s.gyro += (0.0025 * i * 0.01 + 0.00002 * std::sin(i * 0.73)) / 1.012;
        require(capture.add(s), "still samples accepted");
    }
    const auto result = capture.finish();
    require(result.quality.accepted && near(result.gyroBias, 0.0025, 0.000003),
            "stationary regression measures calibrated bias across continuous multi-turn angle");
    require(result.gyroIncrementStd > 0.000015 && result.gyroIncrementStd < 0.000025,
            "still residual measures increment noise rather than drift");
    require(result.gyroBiasVariance >= square(radians(0.01)),
            "bias variance has conservative correlated-reading floor");
    capture.reset(); auto s = sample();
    require(capture.add(s), "first stationary sample accepted");
    require(!capture.add(s) && capture.finish().quality.failure == FitFailure::invalidData,
            "repeated sensor timestamps cannot manufacture stationary certainty");
    capture.reset(); require(capture.add(s), "reset permits new capture");
    s.timestamp += 0.01; s.left += 0.005;
    require(!capture.add(s) && capture.finish().quality.failure == FitFailure::moving,
            "wheel displacement rejects gyro zeroing while moving");
    capture.reset(); s = sample(); capture.add(s); s.timestamp += 0.11;
    require(!capture.add(s), "large sensor gaps reject capture");
    capture.reset(); s = sample(); s.gyro = std::numeric_limits<double>::quiet_NaN();
    require(!capture.add(s), "nonfinite IMU rejected");
    capture.reset();
    for (int i = 0; i <= 400; ++i) {
        s = sample(); s.timestamp += 0.01 * i;
        s.gyro += 0.0009 * square(0.01 * i);
        require(capture.add(s), "slow changing gyro drift captured");
    }
    require(!capture.finish().quality.accepted,
            "nonstationary gyro drift cannot masquerade as constant bias");
}
struct Experiment {
    bool clockwise = true, counterclockwise = true, outliers = false;
    bool directionMismatch = false, translation = false, slightDrift = false, missingLateral = false;
    bool lateralSlip = false;
};
AutomaticGeometryResult spin(Experiment experiment = {}) {
    RotationCalibration capture;
    RotationCalibrationConfig config;
    config.forwardScale = 1.031; config.lateralScale = 0.982;
    config.gyroScale = 1.012; config.gyroBias = 0.001;
    config.useLateralPod = !experiment.missingLateral;
    capture.reset(config);
    auto s = sample(); s.lateralValid = !experiment.missingLateral;
    require(capture.add(s), "rotation baseline accepted");
    int index = 0;
    for (int direction : {1, -1}) {
        if ((direction > 0 && !experiment.clockwise) || (direction < 0 && !experiment.counterclockwise)) continue;
        for (int i = 0; i < 540; ++i, ++index) {
            const double angle = direction * (0.012 + 0.003 * std::sin(0.021 * i));
            const double width = experiment.directionMismatch && direction < 0 ? 0.350 : 0.310;
            const double center = experiment.translation ? 0.04 * std::abs(angle) :
                                  experiment.slightDrift ? 0.004 * angle : 0;
            s.timestamp += 0.01;
            s.gyro += (angle + config.gyroBias * 0.01) / config.gyroScale;
            s.left += 0.5 * width * angle + center;
            s.right -= 0.5 * width * angle - center;
            s.forward += (-0.024 * angle + center + 0.000001 * std::sin(index)) / config.forwardScale;
            s.lateral += (0.058 * angle + 0.000001 * std::cos(index)) / config.lateralScale;
            if (experiment.lateralSlip) s.lateral += 0.05 * std::sin(i * 0.055) * angle / config.lateralScale;
            if (experiment.outliers && (index == 80 || index == 580)) s.forward += 0.008;
            require(capture.add(s), "finite rotation stream captured without allocation");
        }
        // Reversal includes a still pause, as it does on the physical robot.
        for (int pause = 0; pause < 30; ++pause) {
            s.timestamp += 0.01; s.gyro += config.gyroBias * 0.01 / config.gyroScale;
            require(capture.add(s), "stopped reversal pause accepted");
        }
    }
    return capture.finish();
}
void geometry() {
    const auto clean = spin();
    require(clean.accepted, "two opposite rotations accepted");
    require(near(clean.forwardOffset.value, 0.024, 0.00001) &&
            near(clean.lateralOffset.value, 0.058, 0.00001) &&
            near(clean.trackWidth.value, 0.310, 0.00001),
            "geometry uses existing calibrated pod and gyro scales and drift correction");
    const auto noisy = spin({.outliers = true});
    require(noisy.accepted && noisy.outliers >= 2 && near(noisy.forwardOffset.value, 0.024, 0.0003),
            "isolated tracking-wheel slips are identified and robustly rejected");
    const auto arcs = spin({.translation = true});
    require(!arcs.accepted && arcs.failure == FitFailure::moving, "arcs cannot calibrate pure-turn geometry");
    const auto drift = spin({.slightDrift = true});
    require(drift.accepted && near(drift.forwardOffset.value, 0.024, 0.00001),
            "small wheel-observed center drift does not bias forward pod offset");
    require(!spin({.directionMismatch = true}).accepted,
            "different CW and CCW geometry is rejected rather than averaged");
    require(!spin({.lateralSlip = true}).accepted,
            "persistent variable pod slip cannot create an accepted geometry profile");
    const auto oneDirection = spin({.counterclockwise = false});
    require(!oneDirection.accepted && oneDirection.failure == FitFailure::insufficientExcitation,
            "one-direction fit rejected even with ample total motion");
    const auto onePod = spin({.missingLateral = true});
    require(onePod.accepted && onePod.forwardOffset.quality.accepted && !onePod.lateralOffset.quality.accepted,
            "uninstalled pod is omitted without inventing a measurement");

    CalibrationProfile profile; profile.hardwareFingerprint = 123;
    profile.calibratedMask = gyroScaleField | leftDynamicsField; profile.gyroScale = 1.012; profile.kVLeft = 5.9;
    require(clean.applyTo(profile), "valid fit merges into live profile");
    require(profile.gyroScale == 1.012 && profile.kVLeft == 5.9 &&
            profile.calibratedMask == (gyroScaleField | leftDynamicsField | forwardOffsetField | lateralOffsetField | trackWidthField),
            "automatic geometry never claims absolute scale or dynamics calibration");
    const auto mask = profile.calibratedMask;
    require(!arcs.applyTo(profile) && profile.calibratedMask == mask, "failed run cannot change a profile");
}
void invalidStreams() {
    RotationCalibration capture;
    auto s = sample(); capture.reset(); capture.add(s);
    require(!capture.add(s) && capture.finish().failure == FitFailure::invalidData,
            "duplicate rotation timestamp is rejected");
    capture.reset(); capture.add(s); s.timestamp += 0.01; s.forwardValid = false;
    require(!capture.add(s), "required pod disappearance rejects run");
    capture.reset(); s = sample(); capture.add(s); s.timestamp += 0.01; s.gyro += 2 * pi;
    require(!capture.add(s), "reset or wrapped gyro jump cannot become rotation evidence");
    capture.reset(); s = sample(); capture.add(s); s.timestamp -= 0.01;
    require(!capture.add(s), "time running backward rejected");
    capture.reset(); s = sample(); s.leftValid = false;
    require(!capture.add(s), "both drive sides required to test pure-turn assumption");
    capture.reset(); s = sample(); capture.add(s);
    bool capacityReached = false;
    for (int i = 0; i < 1100; ++i) {
        s.timestamp += 0.01; s.gyro += 0.03;
        s.left += 0.15 * 0.03; s.right -= 0.15 * 0.03;
        s.forward -= 0.02 * 0.03; s.lateral += 0.06 * 0.03;
        if (!capture.add(s)) { capacityReached = true; break; }
    }
    require(capacityReached && capture.finish().failure == FitFailure::capacityExceeded,
            "excessive capture is bounded and rejected rather than truncating or allocating");
}
} // namespace
int main() {
    stationary(); geometry(); invalidStreams();
    std::cout << "Automatic stationary bias and bidirectional geometry calibration tests passed\n";
    return EXIT_SUCCESS;
}
