#include "nexus/automatic_calibration.hpp"

#include <algorithm>
#include <cmath>

namespace nexus { namespace calibration {
namespace {
constexpr std::size_t capacity = RotationCalibration::capacity;
using Values = std::array<double, capacity>;
bool bounded(double v, double lo, double hi) {
    return std::isfinite(v) && v >= lo && v <= hi;
}
bool validBase(const SensorSample& s) {
    return s.gyroValid && s.leftValid && s.rightValid &&
        std::isfinite(s.timestamp) && std::isfinite(s.gyro) &&
        std::isfinite(s.left) && std::isfinite(s.right);
}
double median(Values values, std::size_t n) {
    if (!n) return 0;
    std::nth_element(values.begin(), values.begin() + n / 2, values.begin() + n);
    const double upper = values[n / 2];
    return n % 2 ? upper : 0.5 * (upper + *std::max_element(values.begin(), values.begin() + n / 2));
}
double robustSigma(const Values& values, std::size_t n, double floor) {
    const double center = median(values, n);
    Values deviations{};
    for (std::size_t i = 0; i < n; ++i) deviations[i] = std::abs(values[i] - center);
    return std::max(floor, 1.4826 * median(deviations, n));
}
struct Line { double slope = 0, noise = 0; };
Line line(const double* times, const double* angles, std::size_t n) {
    double mt = 0, ma = 0;
    for (std::size_t i = 0; i < n; ++i) { mt += times[i]; ma += angles[i]; }
    mt /= n; ma /= n;
    double tt = 0, ta = 0;
    for (std::size_t i = 0; i < n; ++i) {
        tt += square(times[i] - mt); ta += (times[i] - mt) * (angles[i] - ma);
    }
    Line result;
    result.slope = tt > 1e-12 ? ta / tt : 0;
    double residual = 0;
    for (std::size_t i = 0; i < n; ++i)
        residual += square(angles[i] - ma - result.slope * (times[i] - mt));
    result.noise = std::sqrt(residual / std::max<std::size_t>(1, n - 2));
    return result;
}
ScalarFit fit(const Values& x, const Values& y, std::size_t n,
              double lower, double upper, double maxRms, int direction = 0) {
    ScalarFit result;
    Values ratios{}, residuals{};
    std::size_t selected = 0;
    for (std::size_t i = 0; i < n; ++i) if (!direction || x[i] * direction > 0) {
        ratios[selected++] = y[i] / x[i];
    }
    result.quality.samples = selected;
    if (selected < 10) return result;
    result.value = median(ratios, selected);
    // Start at the median ratio rather than OLS, so a few large encoder jumps
    // cannot contaminate the residual scale of the first robust iteration.
    for (unsigned iteration = 0; iteration < 10; ++iteration) {
        std::size_t k = 0;
        for (std::size_t i = 0; i < n; ++i) if (!direction || x[i] * direction > 0)
            residuals[k++] = y[i] - result.value * x[i];
        const double delta = 1.5 * robustSigma(residuals, selected, 0.00025);
        double xx = 0, xy = 0;
        for (std::size_t i = 0; i < n; ++i) if (!direction || x[i] * direction > 0) {
            const double r = std::abs(y[i] - result.value * x[i]);
            const double weight = r > delta ? delta / r : 1.0;
            xx += weight * x[i] * x[i]; xy += weight * x[i] * y[i];
        }
        if (xx <= 1e-12) { result.quality.failure = FitFailure::illConditioned; return result; }
        result.value = xy / xx;
    }
    std::size_t k = 0;
    for (std::size_t i = 0; i < n; ++i) if (!direction || x[i] * direction > 0)
        residuals[k++] = y[i] - result.value * x[i];
    const double threshold = 4.5 * robustSigma(residuals, selected, 0.00025);
    double ss = 0, inlierSS = 0;
    for (std::size_t i = 0; i < selected; ++i) {
        ss += square(residuals[i]);
        result.quality.maxResidual = std::max<double>(result.quality.maxResidual, std::abs(residuals[i]));
        if (std::abs(residuals[i]) > threshold) ++result.quality.outliers;
        else inlierSS += square(residuals[i]);
    }
    const std::size_t inliers = selected - result.quality.outliers;
    result.quality.rms = std::sqrt(ss / selected);
    result.quality.inlierRms = std::sqrt(inlierSS / std::max<std::size_t>(1, inliers));
    result.quality.condition = 1;
    result.quality.accepted = inliers >= 10 && result.quality.inlierRms <= maxRms &&
        result.quality.outliers <= 0.15 * selected;
    result.quality.failure = result.quality.accepted ? FitFailure::none : FitFailure::excessiveResidual;
    if (!bounded(result.value, lower, upper)) {
        result.quality.accepted = false; result.quality.failure = FitFailure::implausible;
    }
    return result;
}
ScalarFit directionCheckedFit(const Values& x, const Values& y, std::size_t n,
                              double lower, double upper, double maxRms,
                              double absoluteTolerance, double relativeTolerance) {
    ScalarFit result = fit(x, y, n, lower, upper, maxRms);
    if (!result.quality.accepted) return result;
    const ScalarFit clockwise = fit(x, y, n, lower, upper, maxRms, 1);
    const ScalarFit counterclockwise = fit(x, y, n, lower, upper, maxRms, -1);
    if (!clockwise.quality.accepted || !counterclockwise.quality.accepted) {
        result.quality.accepted = false;
        result.quality.failure = !clockwise.quality.accepted ? clockwise.quality.failure : counterclockwise.quality.failure;
    } else if (std::abs(clockwise.value - counterclockwise.value) >
               absoluteTolerance + relativeTolerance * std::abs(result.value)) {
        result.quality.accepted = false;
        result.quality.failure = FitFailure::inconsistentDirections;
    }
    return result;
}
} // namespace

void StationaryCalibration::reset(double gyroScale) {
    count_ = 0; first_ = {}; previous_ = {}; gyroScale_ = gyroScale;
    failure_ = bounded(gyroScale, 0.8, 1.2) ? FitFailure::none : FitFailure::implausible;
}
bool StationaryCalibration::add(const SensorSample& s) {
    if (failure_ != FitFailure::none) return false;
    if (!validBase(s) || (s.forwardValid && !std::isfinite(s.forward)) ||
        (s.lateralValid && !std::isfinite(s.lateral))) {
        failure_ = FitFailure::invalidData; return false;
    }
    if (count_ >= capacity) { failure_ = FitFailure::capacityExceeded; return false; }
    if (!count_) first_ = s;
    else {
        const double dt = s.timestamp - previous_.timestamp;
        if (!bounded(dt, 0.001, 0.10) || s.forwardValid != first_.forwardValid ||
            s.lateralValid != first_.lateralValid) {
            failure_ = FitFailure::invalidData; return false;
        }
        if (std::abs(s.left - first_.left) > 0.0015 || std::abs(s.right - first_.right) > 0.0015 ||
            (s.forwardValid && std::abs(s.forward - first_.forward) > 0.0015) ||
            (s.lateralValid && std::abs(s.lateral - first_.lateral) > 0.0015) ||
            std::abs((s.gyro - previous_.gyro) * gyroScale_) > radians(0.4) + radians(1) * dt) {
            failure_ = FitFailure::moving; return false;
        }
    }
    times_[count_] = s.timestamp - first_.timestamp;
    angles_[count_] = (s.gyro - first_.gyro) * gyroScale_;
    ++count_; previous_ = s;
    return true;
}
StationaryResult StationaryCalibration::finish() const {
    StationaryResult result;
    result.quality.samples = count_;
    if (count_) result.duration = times_[count_ - 1];
    if (failure_ != FitFailure::none) { result.quality.failure = failure_; return result; }
    if (count_ < 100 || result.duration < 2.0) return result;
    const Line full = line(times_.data(), angles_.data(), count_);
    const std::size_t half = count_ / 2;
    const Line early = line(times_.data(), angles_.data(), half);
    const Line late = line(times_.data() + half, angles_.data() + half, count_ - half);
    result.gyroBias = full.slope;
    // Treat at most ten observations per second as independent. Cached IMU
    // samples at 100 Hz do not provide 100 independent bias measurements/s.
    const double independent = std::min(static_cast<double>(count_), result.duration * 10);
    result.gyroBiasVariance = std::max(square(radians(0.01)),
        12 * square(full.noise) / (independent * square(result.duration)) +
        0.25 * square(early.slope - late.slope));
    result.gyroIncrementStd = std::sqrt(2.0) * full.noise;
    result.quality.rms = result.quality.inlierRms = full.noise;
    result.quality.condition = 1;
    if (!bounded(full.slope, -radians(1), radians(1)) || full.noise > radians(0.08) ||
        std::abs(early.slope - late.slope) > radians(0.1)) {
        result.quality.failure = FitFailure::moving; return result;
    }
    result.quality.accepted = true; result.quality.failure = FitFailure::none;
    return result;
}

void RotationCalibration::reset(RotationCalibrationConfig config) {
    config_ = config; count_ = rejectedFrames_ = 0; initialized_ = false;
    pending_ = {}; pendingCenter_ = 0; previous_ = {};
    failure_ = bounded(config.forwardScale, 0.5, 1.5) && bounded(config.lateralScale, 0.5, 1.5) &&
        bounded(config.gyroScale, 0.8, 1.2) && bounded(config.gyroBias, -radians(1), radians(1)) ?
        FitFailure::none : FitFailure::implausible;
}
bool RotationCalibration::add(const SensorSample& s) {
    if (failure_ != FitFailure::none) return false;
    if (!validBase(s) || (config_.useForwardPod && (!s.forwardValid || !std::isfinite(s.forward))) ||
        (config_.useLateralPod && (!s.lateralValid || !std::isfinite(s.lateral)))) {
        failure_ = FitFailure::invalidData; return false;
    }
    if (!initialized_) { previous_ = s; initialized_ = true; return true; }
    const double dt = s.timestamp - previous_.timestamp;
    const double angle = (s.gyro - previous_.gyro) * config_.gyroScale - config_.gyroBias * dt;
    const double dl = s.left - previous_.left, dr = s.right - previous_.right;
    const double df = config_.useForwardPod ? (s.forward - previous_.forward) * config_.forwardScale : 0;
    const double ds = config_.useLateralPod ? (s.lateral - previous_.lateral) * config_.lateralScale : 0;
    if (!bounded(dt, 0.001, 0.10) || std::abs(angle) > 12 * dt ||
        std::abs(dl) > 4 * dt || std::abs(dr) > 4 * dt ||
        std::abs(df) > 4 * dt || std::abs(ds) > 4 * dt) {
        failure_ = FitFailure::invalidData; return false;
    }
    pending_.angle += angle; pending_.drive += dl - dr; pendingCenter_ += dl + dr;
    // Remove the small measured forward-center drift before fitting the pod
    // offset. Large arcs still fail the pure-turn check below.
    if (config_.useForwardPod) pending_.forward += df - 0.5 * (dl + dr);
    if (config_.useLateralPod) pending_.lateral += ds;
    previous_ = s;
    if (std::abs(pending_.angle) < 0.20) return true;
    // Wheel-center translation means this was an arc rather than a pure turn.
    // Isolated bad windows are discarded; repeated translation rejects the run.
    if (std::abs(pendingCenter_) > std::max<double>(0.002, 0.06 * std::abs(pending_.drive))) ++rejectedFrames_;
    else {
        if (count_ == capacity) { failure_ = FitFailure::capacityExceeded; return false; }
        windows_[count_++] = pending_;
    }
    pending_ = {}; pendingCenter_ = 0;
    return true;
}
AutomaticGeometryResult RotationCalibration::finish() const {
    AutomaticGeometryResult result;
    result.samples = count_; result.rejectedFrames = rejectedFrames_;
    if (failure_ != FitFailure::none) { result.failure = failure_; return result; }
    if (rejectedFrames_ > 0.15 * (count_ + rejectedFrames_)) {
        result.failure = FitFailure::moving; return result;
    }
    Values angles{}, forward{}, lateral{}, drive{};
    for (std::size_t i = 0; i < count_; ++i) {
        angles[i] = windows_[i].angle; forward[i] = -windows_[i].forward;
        lateral[i] = windows_[i].lateral; drive[i] = windows_[i].drive;
        if (angles[i] > 0) result.clockwiseAngle += angles[i]; else result.counterclockwiseAngle -= angles[i];
    }
    if (result.clockwiseAngle < pi || result.counterclockwiseAngle < pi) {
        result.failure = FitFailure::insufficientExcitation; return result;
    }
    result.trackWidth = directionCheckedFit(angles, drive, count_, 0.10, 1.0, 0.0025, 0.008, 0.04);
    if (config_.useForwardPod)
        result.forwardOffset = directionCheckedFit(angles, forward, count_, -1, 1, 0.0015, 0.002, 0.05);
    if (config_.useLateralPod)
        result.lateralOffset = directionCheckedFit(angles, lateral, count_, -1, 1, 0.0015, 0.002, 0.05);
    const ScalarFit* fits[] = {&result.trackWidth, &result.forwardOffset, &result.lateralOffset};
    const bool used[] = {true, config_.useForwardPod, config_.useLateralPod};
    result.outliers = rejectedFrames_;
    for (std::size_t i = 0; i < 3; ++i) if (used[i]) {
        result.outliers += fits[i]->quality.outliers;
        if (!fits[i]->quality.accepted) { result.failure = fits[i]->quality.failure; return result; }
    }
    // Conservative union upper bound, also catches different sensors slipping
    // at different times. Persistent proportional slip is not observable here.
    if (result.outliers > 0.20 * (count_ + rejectedFrames_)) {
        result.failure = FitFailure::excessiveResidual; return result;
    }
    result.accepted = true; result.failure = FitFailure::none;
    return result;
}
bool AutomaticGeometryResult::applyTo(CalibrationProfile& profile) const {
    if (!accepted || !trackWidth.quality.accepted || !validProfile(profile)) return false;
    CalibrationProfile updated = profile;
    updated.trackWidth = trackWidth.value; updated.calibratedMask |= trackWidthField;
    if (forwardOffset.quality.accepted) {
        updated.forwardOffset = forwardOffset.value; updated.calibratedMask |= forwardOffsetField;
    }
    if (lateralOffset.quality.accepted) {
        updated.lateralOffset = lateralOffset.value; updated.calibratedMask |= lateralOffsetField;
    }
    if (!validProfile(updated)) return false;
    profile = updated;
    return true;
}

}} // namespace nexus::calibration
