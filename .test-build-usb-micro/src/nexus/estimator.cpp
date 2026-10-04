#include "nexus/estimator.hpp"

#include <limits>

namespace nexus {
namespace {
constexpr double referencePeriod = 0.01;
bool finite(double value) { return std::isfinite(value); }
double positive(double value, double fallback) {
    return finite(value) && value > 0 ? value : fallback;
}
EstimatorConfig sanitizeConfig(EstimatorConfig config) {
    config.trackWidth = positive(config.trackWidth, 0.284);
    config.encoderStd = positive(config.encoderStd, 0.0015);
    config.gyroStd = positive(config.gyroStd, radians(0.12));
    config.rangeStd = positive(config.rangeStd, 0.025);
    if (!finite(config.rangeRelativeStd) || config.rangeRelativeStd < 0) config.rangeRelativeStd = 0.05;
    config.rangeGate = positive(config.rangeGate, 9.0);
    config.maxRangeCorrection = positive(config.maxRangeCorrection, 0.10);
    config.maxSpeed = positive(config.maxSpeed, 3.0);
    config.maxOmega = positive(config.maxOmega, 18.0);
    config.maxSensorHold = finite(config.maxSensorHold) ? std::clamp(config.maxSensorHold, 0.0, 0.1) : 0;
    config.stationaryEncoderSpan = finite(config.stationaryEncoderSpan)
        ? std::clamp(config.stationaryEncoderSpan, 0.0, 0.0001) : 0;
    config.rangeCount = std::min(maxRanges, config.rangeCount);
    if (!finite(config.forwardOffset)) config.forwardOffset = 0;
    if (!finite(config.lateralOffset)) config.lateralOffset = 0;
    config.forwardScale = positive(config.forwardScale, 1.0);
    config.lateralScale = positive(config.lateralScale, 1.0);
    config.gyroScale = positive(config.gyroScale, 1.0);
    for (auto& noise : config.rangeNoise) {
        if (!finite(noise.standardDeviation) || noise.standardDeviation <= 0) noise.standardDeviation = -1;
        if (!finite(noise.relativeStd) || noise.relativeStd < 0) noise.relativeStd = -1;
    }
    return config;
}
void normalizeCovariance(std::array<std::array<double, 4>, 4>& p) {
    for (std::size_t i = 0; i < 4; ++i) {
        p[i][i] = std::max(1e-12, p[i][i]);
        for (std::size_t j = i + 1; j < 4; ++j) {
            p[i][j] = p[j][i] = 0.5 * (p[i][j] + p[j][i]);
        }
    }
}
}

Estimator::Estimator(EstimatorConfig config) : config_(sanitizeConfig(config)) {
    reset({});
}

void Estimator::clearReplay() {
    historyStart_ = 0;
    historySize_ = initialized_ ? 1 : 0;
    if (initialized_) {
        node(0) = {};
        node(0).state = state_;
        node(0).timestamp = output_.timestamp;
    }
    corroboration_ = {};
    lastRangeTimestamp_.fill(-std::numeric_limits<double>::infinity());
    lastRecoveryRangeTimestamp_.fill(-std::numeric_limits<double>::infinity());
    recoveryCount_ = 0;
    recoveryTimestamp_ = -1;
}

void Estimator::reconfigure(EstimatorConfig config) {
    config = sanitizeConfig(config);
    if (config.nativeImuHeading || config_.nativeImuHeading) {
        // A disabled correction must not survive as a stale calibration when
        // the policy changes again. Preserve the complete pose marginal.
        state_.bias = 0;
        state_.p[3][3] = config.nativeImuHeading ? 1e-12 : square(radians(2));
        for (std::size_t i = 0; i < 3; ++i) state_.p[i][3] = state_.p[3][i] = 0;
    } else {
        const double biasScale = config.gyroScale / config_.gyroScale;
        state_.bias *= biasScale;
        state_.p[3][3] *= square(biasScale);
        for (std::size_t i = 0; i < 3; ++i) {
            state_.p[i][3] *= biasScale;
            state_.p[3][i] *= biasScale;
        }
    }
    config_ = config;
    initialized_ = false;
    previous_ = {};
    stationaryTime_ = missingTranslationTime_ = missingHeadingTime_ = 0;
    stationaryGyroTime_ = -1;
    stationaryBoundsValid_ = false;
    state_.v = state_.omega = state_.vSmooth = state_.omegaSmooth = 0;
    output_.state = {state_.pose, 0, 0};
    output_.gyroBias = state_.bias;
    output_.gyroBiasStd = config_.nativeImuHeading ? 0 : std::sqrt(state_.p[3][3]);
    output_.stationary = false;
    output_.quiet.dwell = 0;
    output_.quiet.encoderSpan = config_.stationaryEncoderSpan;
    output_.rejectedIncrementMask = 0;
    output_.headingSource = HeadingSource::unavailable;
    output_.health = Health::lost;
    output_.confidence = 0;
    clearReplay();
}

bool Estimator::setGyroBias(double bias, double variance) {
    if (config_.nativeImuHeading || !finite(bias) || std::abs(bias) > radians(3) || !finite(variance) || variance <= 0)
        return false;
    state_.bias = bias;
    // Preserve the pose marginal: a new bias calibration is not an absolute
    // position/heading fix, and its error is independent of the old bias prior.
    for (std::size_t i = 0; i < 3; ++i) state_.p[i][3] = state_.p[3][i] = 0;
    state_.p[3][3] = std::max(variance, 1e-12);
    output_.gyroBias = bias;
    output_.gyroBiasStd = std::sqrt(state_.p[3][3]);
    clearReplay();
    return true;
}

void Estimator::reset(Pose pose, double timestamp) {
    if (!finite(pose.x) || !finite(pose.y) || !finite(pose.theta)) pose = {};
    if (!finite(timestamp)) timestamp = 0;
    const double learnedBias = config_.nativeImuHeading ? 0 : state_.bias;
    const double learnedBiasVariance = config_.nativeImuHeading ? 1e-12 : positive(state_.p[3][3], square(radians(2)));
    state_ = {};
    state_.bias = learnedBias;
    state_.pose = {pose.x, pose.y, wrap(pose.theta)};
    state_.p[0][0] = state_.p[1][1] = square(0.01);
    state_.p[2][2] = square(radians(0.5));
    state_.p[3][3] = learnedBiasVariance;
    initialized_ = false;
    previous_ = {};
    output_ = {};
    output_.quiet.encoderSpan = config_.stationaryEncoderSpan;
    lostPrior_ = false;
    recovering_ = false;
    output_.state.pose = state_.pose;
    output_.gyroBias = state_.bias;
    output_.gyroBiasStd = config_.nativeImuHeading ? 0 : std::sqrt(state_.p[3][3]);
    output_.timestamp = timestamp;
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j) output_.covariance[i][j] = state_.p[i][j];
    stationaryTime_ = missingTranslationTime_ = missingHeadingTime_ = 0;
    stationaryGyro_ = 0;
    stationaryGyroTime_ = -1;
    stationaryBoundsValid_ = false;
    historyStart_ = historySize_ = 0;
    corroboration_ = {};
    lastRangeTimestamp_.fill(-std::numeric_limits<double>::infinity());
    lastRecoveryRangeTimestamp_.fill(-std::numeric_limits<double>::infinity());
    recoveryCandidate_ = {};
    recoveryTimestamp_ = -1;
    recoveryCount_ = 0;
}

Estimate Estimator::estimate() const { return output_; }
void Estimator::noteQuietBreak(std::uint16_t mask, const std::array<double, 4>& spans,
                              double gyroRate, double interval, double timestamp) {
    if (!mask) return;
    auto& quiet = output_.quiet;
    quiet.lastBreakMask = mask;
    if (quiet.breaks < std::numeric_limits<std::uint32_t>::max()) ++quiet.breaks;
    quiet.wheelSpans = spans;
    quiet.gyroRate = gyroRate;
    quiet.sampleInterval = interval;
    quiet.timestamp = timestamp;
}
Estimator::Node& Estimator::node(std::size_t i) {
    return history_[(historyStart_ + i) % historyCapacity];
}
const Estimator::Node& Estimator::node(std::size_t i) const {
    return history_[(historyStart_ + i) % historyCapacity];
}

void Estimator::propagate(State& state, const Step& step, double fraction) const {
    if (fraction <= 0) return;
    const double dt = step.dt * fraction;
    double angle = step.heading * fraction;
    const bool stationaryHeading = step.stationary && !config_.nativeImuHeading;
    if (step.headingSource == HeadingSource::imu && !config_.nativeImuHeading) angle -= state.bias * dt;
    if (stationaryHeading) angle = step.stationaryHeading * fraction;
    state.odometry.headingChange += angle;
    state.odometry.minimumHeadingChange = std::min(state.odometry.minimumHeadingChange, state.odometry.headingChange);
    state.odometry.maximumHeadingChange = std::max(state.odometry.maximumHeadingChange, state.odometry.headingChange);
    double forward = step.forward * fraction;
    double lateral = step.lateral * fraction;
    forward += step.forwardRotationOffset * angle;
    if (step.lateralPod) lateral -= config_.lateralOffset * angle;
    // Quiet sensor readings can still contain genuine sub-resolution creep.
    // Preserve cumulative translation instead of erasing a per-tick deadband.

    double a, b, da, db;
    if (std::abs(angle) < 1e-4) {
        const double t2 = angle * angle;
        a = 1 - t2 / 6 + t2 * t2 / 120;
        b = angle / 2 - angle * t2 / 24;
        da = -angle / 3 + angle * t2 / 30;
        db = 0.5 - t2 / 8 + t2 * t2 / 144;
    } else {
        a = std::sin(angle) / angle;
        b = (1 - std::cos(angle)) / angle;
        da = (angle * std::cos(angle) - std::sin(angle)) / square(angle);
        db = (angle * std::sin(angle) - (1 - std::cos(angle))) / square(angle);
    }
    const double c = std::cos(state.pose.theta), s = std::sin(state.pose.theta);
    const double lx = a * lateral + b * forward;
    const double ly = a * forward - b * lateral;
    const double dx = c * lx + s * ly, dy = -s * lx + c * ly;
    const double df = step.forwardRotationOffset;
    const double dl = step.lateralPod ? -config_.lateralOffset : 0;
    {
        // Exact decomposition of this SE(2) translation, including replayed
        // fractional steps. Exposes whether lateral drift comes from a pod or
        // from projecting forward travel with heading; never feeds back into pose.
        const double gx = c * b + s * a, gy = c * a - s * b;
        const double rawForward = step.forward * fraction, rawLateral = step.lateral * fraction;
        state.odometry.forwardX += gx * rawForward;
        state.odometry.forwardY += gy * rawForward;
        state.odometry.lateralX += gy * rawLateral;
        state.odometry.lateralY -= gx * rawLateral;
        state.odometry.offsetX += (gx * df + gy * dl) * angle;
        state.odometry.offsetY += (gy * df - gx * dl) * angle;
    }
    const double jx = da * lateral + a * dl + db * forward + b * df;
    const double jy = da * forward + a * df - db * lateral - b * dl;
    const double ax = c * jx + s * jy, ay = -s * jx + c * jy;
    const double dbias = step.headingSource == HeadingSource::imu && !step.stationary && !config_.nativeImuHeading ? -dt : 0;

    Covariance f{}, fp{}, next{};
    for (std::size_t i = 0; i < 4; ++i) f[i][i] = 1;
    f[0][2] = dy; f[1][2] = -dx;
    f[0][3] = ax * dbias; f[1][3] = ay * dbias; f[2][3] = dbias;
    for (std::size_t i = 0; i < 4; ++i)
        for (std::size_t j = 0; j < 4; ++j)
            for (std::size_t k = 0; k < 4; ++k) fp[i][j] += f[i][k] * state.p[k][j];
    for (std::size_t i = 0; i < 4; ++i)
        for (std::size_t j = 0; j < 4; ++j)
            for (std::size_t k = 0; k < 4; ++k) next[i][j] += fp[i][k] * f[j][k];

    // Noise densities are specified as 10 ms incremental standard deviations.
    // A bounded cumulative wheel range and an independent yaw constraint do
    // not justify accumulating the moving model's unbounded random walk.
    // Retain the pose prior and measured micro-motion; missing lateral sensing
    // still admits an unobserved sideways push during a quiet interval. Native
    // IMU heading has no wheel-yaw constraint, so its noise remains active.
    const double noiseScale = dt / referencePeriod;
    const double translationVariance = step.stationary ? 0 : square(config_.encoderStd) * noiseScale;
    const double forwardVariance = translationVariance * (step.forwardPod ? 1.0 + 12 * step.slip : 6.0 + 30 * step.slip)
        + (step.translation ? 0 : square(0.3) * dt);
    const double lateralVariance = translationVariance * (step.lateralPod ? 1.0 : 5.0)
        + (step.lateralPod ? 0 : square(0.06) * std::abs(forward) + square(0.03) * dt);
    double headingVariance = square(config_.gyroStd) * noiseScale;
    if (step.headingSource == HeadingSource::driveEncoders) headingVariance *= 8 + 40 * step.slip;
    if (step.headingSource == HeadingSource::unavailable) headingVariance += square(radians(25)) * dt;
    if (stationaryHeading) headingVariance = 0;
    const double g[3][3] = {{c * a - s * b, c * b + s * a, ax},
                            {-s * a - c * b, -s * b + c * a, ay},
                            {0, 0, 1}};
    const double variance[3] = {lateralVariance, forwardVariance, headingVariance};
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            for (std::size_t k = 0; k < 3; ++k) next[i][j] += g[i][k] * variance[k] * g[j][k];
    if (!config_.nativeImuHeading) next[3][3] += square(radians(0.015)) * dt;
    state.p = next;
    normalizeCovariance(state.p);
    state.pose.x += dx; state.pose.y += dy;
    state.pose.theta = wrap(state.pose.theta + angle);
    // Two cascaded 70 ms poles with a compensating zero remove most of the
    // 35 ms EMA's acceleration lag at nearly the same encoder-noise gain.
    // Integrate the continuous filter exactly for this constant interval
    // velocity: delayed range events may split an interval arbitrarily, and
    // must not change the velocity solely by changing the number of updates.
    constexpr double velocityPeriod = 0.070;
    const double interval = dt / velocityPeriod;
    const double decay = std::exp(-interval);
    const double alpha = -std::expm1(-interval);
    const auto filterVelocity = [&](double measurement, double& smooth, double& velocity) {
        velocity += alpha * (measurement - velocity) + (measurement - smooth) * interval * decay;
        smooth += alpha * (measurement - smooth);
    };
    filterVelocity(forward / dt, state.vSmooth, state.v);
    filterVelocity(angle / dt, state.omegaSmooth, state.omega);
}

void Estimator::observeBias(State& state, const Step& step) const {
    if (config_.nativeImuHeading || !step.observeBias) return;
    const double variance = state.p[3][3] + step.biasVariance;
    const double gain = state.p[3][3] / variance;
    // Stationary wheel evidence observes bias, not global heading/position.
    // Updating the latter via stale correlations would imply an absolute fix.
    state.bias += gain * (step.biasReading - state.bias);
    state.bias = std::clamp(state.bias, -radians(3.0), radians(3.0));
    for (std::size_t i = 0; i < 3; ++i) state.p[i][3] = state.p[3][i] = 0;
    state.p[3][3] = (1 - gain) * state.p[3][3];
    normalizeCovariance(state.p);
}

void Estimator::append(const Step& step, double timestamp) {
    if (historySize_ == historyCapacity) {
        historyStart_ = (historyStart_ + 1) % historyCapacity;
        --historySize_;
    }
    Node& entry = node(historySize_++);
    entry = {};
    entry.timestamp = timestamp;
    entry.step = step;
    propagate(state_, step, 1);
    observeBias(state_, step);
    entry.state = state_;
}

Estimator::State Estimator::stateAt(std::size_t logical, double timestamp) const {
    if (logical == 0) return node(0).state;
    const Node& entry = node(logical);
    State state = node(logical - 1).state;
    const double start = node(logical - 1).timestamp;
    double processed = start;
    for (std::size_t i = 0; i < entry.count; ++i) {
        const Event& event = entry.events[i];
        if (event.timestamp > timestamp) break;
        propagate(state, entry.step, (event.timestamp - processed) / entry.step.dt);
        correctRange(state, event);
        processed = event.timestamp;
    }
    propagate(state, entry.step, (timestamp - processed) / entry.step.dt);
    if (timestamp >= entry.timestamp) observeBias(state, entry.step);
    return state;
}

void Estimator::replayNode(std::size_t logical) {
    node(logical).state = stateAt(logical, node(logical).timestamp);
}

Estimator::Ray Estimator::raycast(const Pose& pose, std::size_t sensor) const {
    Ray result;
    const RangeMount& mount = config_.mounts[sensor];
    const double c = std::cos(pose.theta), s = std::sin(pose.theta);
    const double x = pose.x + c * mount.x + s * mount.y;
    const double y = pose.y - s * mount.x + c * mount.y;
    const Field& field = config_.field;
    if (!finite(x) || !finite(y) || x <= field.minX || x >= field.maxX ||
        y <= field.minY || y >= field.maxY) return result;
    const double ux = std::sin(pose.theta + mount.heading);
    const double uy = std::cos(pose.theta + mount.heading);
    const double tx = std::abs(ux) > 1e-8 ? ((ux > 0 ? field.maxX : field.minX) - x) / ux
                                         : std::numeric_limits<double>::infinity();
    const double ty = std::abs(uy) > 1e-8 ? ((uy > 0 ? field.maxY : field.minY) - y) / uy
                                         : std::numeric_limits<double>::infinity();
    const bool xWall = tx < ty;
    result.distance = xWall ? tx : ty;
    result.incidence = std::abs(xWall ? ux : uy);
    result.wall = xWall ? (ux > 0 ? 1 : 0) : (uy > 0 ? 3 : 2);
    if (result.incidence < 0.4 || std::abs(tx - ty) < std::max<double>(0.10, 4 * std::sqrt(rangeVariance(result.distance, sensor))))
        return result; // A beam near a corner or glancing wall is not identifiable.
    if (xWall) {
        result.h[0] = -1 / ux;
        result.h[2] = -(-s * mount.x + c * mount.y + result.distance * uy) / ux;
    } else {
        result.h[1] = -1 / uy;
        result.h[2] = -(-c * mount.x - s * mount.y - result.distance * ux) / uy;
    }
    result.valid = finite(result.distance) && result.distance > 0;
    return result;
}

double Estimator::rangeVariance(double distance, std::size_t sensor) const {
    const auto& noise = config_.rangeNoise[sensor];
    const double standardDeviation = noise.standardDeviation > 0 ? noise.standardDeviation : config_.rangeStd;
    const double relativeStd = noise.relativeStd >= 0 ? noise.relativeStd : config_.rangeRelativeStd;
    return square(standardDeviation) + square(relativeStd * distance);
}

bool Estimator::correctRange(State& state, const Event& event, double* residualOut) const {
    const Ray ray = raycast(state.pose, event.sensor);
    if (!ray.valid) return false;
    const double residual = event.distance - ray.distance;
    if (residualOut) *residualOut = residual;
    // V5 omits confidence for close returns. Keep these useful measurements
    // conservative rather than mistaking its placeholder for low confidence.
    const double confidence = event.confidenceAvailable ? event.confidence / 63.0 : 35.0 / 63.0;
    const double sensorVariance = rangeVariance(std::max(event.distance, ray.distance), event.sensor);
    const double sensorStd = std::sqrt(sensorVariance);
    const double persistentFloor = sensorVariance / square(ray.incidence);
    double r = persistentFloor * (1 + 4 * square(1 - confidence));
    std::array<double, 4> ph{};
    double predictionVariance = 0;
    for (std::size_t i = 0; i < 4; ++i)
        for (std::size_t j = 0; j < 4; ++j) ph[i] += state.p[i][j] * ray.h[j];
    for (std::size_t i = 0; i < 4; ++i) predictionVariance += ray.h[i] * ph[i];
    const double innovationVariance = predictionVariance + r;
    if (!finite(innovationVariance) || innovationVariance <= 0 ||
        square(residual) > config_.rangeGate * innovationVariance) return false;
    // Huber influence plus an obstacle mixture: unexpectedly shorter beams are
    // less credible, even when an uncertain prior passes the chi-square gate.
    const double normalized = std::abs(residual) / std::sqrt(innovationVariance);
    r *= std::max(1.0, normalized / 1.5);
    if (residual < -2 * sensorStd) r *= 4;
    const double denom = predictionVariance + r;
    std::array<double, 4> k{};
    for (std::size_t i = 0; i < 4; ++i) k[i] = ph[i] / denom;
    // Raw distance accuracy includes calibration/surface-dependent error that
    // repeated samples do not average away. Limit information along this ray
    // to a persistent floor, while preserving an already better odometry prior.
    // For gain alpha*K, posterior scalar variance is
    // p - (2*alpha-alpha^2)*p^2/(p+r); solve alpha for that floor.
    double informationScale = 1;
    if (predictionVariance <= persistentFloor) informationScale = 0;
    else if (predictionVariance * r / denom < persistentFloor) {
        const double available = std::clamp((predictionVariance - persistentFloor) * denom /
                                           square(predictionVariance), 0.0, 1.0);
        informationScale = available / (1 + std::sqrt(1 - available));
    }
    for (double& value : k) value *= informationScale;
    // A wall is not a reliable gyro-bias observation in a changing obstacle map.
    k[3] = 0;
    const double correction = std::hypot(k[0] * residual, k[1] * residual);
    const double headingCorrection = std::abs(k[2] * residual);
    double scale = 1;
    if (correction > config_.maxRangeCorrection) scale = config_.maxRangeCorrection / correction;
    if (headingCorrection > radians(2)) scale = std::min(scale, radians(2) / headingCorrection);
    for (double& value : k) value *= scale;
    state.pose.x += k[0] * residual;
    state.pose.y += k[1] * residual;
    state.pose.theta = wrap(state.pose.theta + k[2] * residual);
    // Joseph form remains PSD even with the robust/bounded, non-optimal gain.
    Covariance a{}, ap{}, next{};
    for (std::size_t i = 0; i < 4; ++i)
        for (std::size_t j = 0; j < 4; ++j) a[i][j] = (i == j ? 1.0 : 0.0) - k[i] * ray.h[j];
    for (std::size_t i = 0; i < 4; ++i)
        for (std::size_t j = 0; j < 4; ++j)
            for (std::size_t l = 0; l < 4; ++l) ap[i][j] += a[i][l] * state.p[l][j];
    for (std::size_t i = 0; i < 4; ++i)
        for (std::size_t j = 0; j < 4; ++j) {
            for (std::size_t l = 0; l < 4; ++l) next[i][j] += ap[i][l] * a[j][l];
            next[i][j] += k[i] * r * k[j];
        }
    state.p = next;
    normalizeCovariance(state.p);
    return finite(state.pose.x) && finite(state.pose.y) && finite(state.pose.theta);
}

void Estimator::processRange(std::size_t sensor, const RangeReading& reading, double now) {
    if (!reading.valid) return;
    auto reject = [&]() { ++output_.rejectedRanges; };
    if (!finite(reading.timestamp) || !finite(reading.distance) || reading.distance < 0.02 ||
        reading.confidence < 0 || reading.confidence > 63 ||
        (reading.confidenceAvailable && reading.confidence < 35) ||
        reading.timestamp > now + 1e-6 || historySize_ < 2) {
        reject(); return;
    }
    // Do not count/reuse repeated hardware readings as independent evidence.
    if (reading.timestamp <= lastRangeTimestamp_[sensor]) return;
    lastRangeTimestamp_[sensor] = reading.timestamp;
    if (reading.timestamp <= node(0).timestamp) { reject(); return; }
    std::size_t logical = 1;
    while (logical + 1 < historySize_ && node(logical).timestamp < reading.timestamp) ++logical;
    State candidate = stateAt(logical, reading.timestamp);
    const Ray ray = raycast(candidate.pose, sensor);
    const double residual = ray.valid ? reading.distance - ray.distance : 0;
    const double sensorStd = std::sqrt(rangeVariance(std::max(reading.distance, ray.distance), sensor));
    output_.rangeResidual[sensor] = residual;
    Corroboration& corroboration = corroboration_[sensor];
    if (!ray.valid) { corroboration = {}; reject(); return; }
    Event event{reading.timestamp, reading.distance, reading.confidence, reading.confidenceAvailable, sensor};
    State corrected = candidate;
    if (!correctRange(corrected, event)) { corroboration = {}; reject(); return; }
    const bool consistent = corroboration.wall == ray.wall &&
        reading.timestamp - corroboration.timestamp <= 0.25 &&
        std::abs(residual - corroboration.residual) <= std::max(0.06, 3 * sensorStd);
    corroboration.count = consistent ? std::min(corroboration.count + 1, 100u) : 1;
    corroboration.wall = ray.wall;
    corroboration.timestamp = reading.timestamp;
    corroboration.residual = residual;
    const unsigned required = !reading.confidenceAvailable || residual < -2 * sensorStd ? 5u : 3u;
    if (corroboration.count < required) { reject(); return; }
    Node& entry = node(logical);
    if (entry.count == entry.events.size()) { reject(); return; }
    // Insertion sort of at most four constraints; updates replay all later
    // already-accepted observations, rather than discarding their information.
    std::size_t position = entry.count++;
    while (position > 0 && entry.events[position - 1].timestamp > event.timestamp) {
        entry.events[position] = entry.events[position - 1];
        --position;
    }
    entry.events[position] = event;
    for (std::size_t i = logical; i < historySize_; ++i) replayNode(i);
    state_ = node(historySize_ - 1).state;
    ++output_.acceptedRanges;
}

void Estimator::publish(double timestamp, const Step& step) {
    output_.state = {state_.pose, state_.v, state_.omega};
    output_.quiet.encoderSpan = config_.stationaryEncoderSpan;
    output_.odometry = state_.odometry;
    output_.gyroBias = state_.bias;
    output_.gyroBiasStd = config_.nativeImuHeading ? 0 : std::sqrt(state_.p[3][3]);
    output_.stationary = step.stationary;
    output_.timestamp = timestamp;
    output_.headingSource = step.headingSource;
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j) output_.covariance[i][j] = state_.p[i][j];
    const double positionStd = std::sqrt(state_.p[0][0] + state_.p[1][1]);
    const double headingStd = std::sqrt(state_.p[2][2]);
    output_.health = Health::healthy;
    if (!step.forwardPod || !step.lateralPod || step.headingSource != HeadingSource::imu ||
        step.slip > 0.25 || step.dt > 0.1) output_.health = Health::degraded;
    if (lostPrior_ || recovering_ || missingTranslationTime_ > 0.15 || missingHeadingTime_ > 0.15 ||
        positionStd > 1.0 || headingStd > radians(45)) output_.health = Health::lost;
    output_.confidence = std::exp(-positionStd / 0.3 - headingStd / radians(20));
    if (output_.health == Health::degraded) output_.confidence *= 0.7;
    if (output_.health == Health::lost) output_.confidence = 0;
}

bool Estimator::relocalize(const SensorSample& sample) {
    if (!initialized_ || !finite(sample.timestamp) ||
        std::abs(sample.timestamp - output_.timestamp) > 1e-6 ||
        stationaryTime_ < 0.25 || !sample.gyroValid ||
        state_.p[2][2] > square(radians(8)) || std::abs(state_.v) > 0.02 ||
        std::abs(state_.omega) > radians(1)) return false;
    std::array<std::size_t, maxRanges> valid{};
    std::size_t count = 0;
    double oldest = sample.timestamp, newest = -1;
    for (std::size_t i = 0; i < config_.rangeCount; ++i) {
        const RangeReading& reading = sample.ranges[i];
        if (reading.valid && reading.confidenceAvailable && reading.confidence >= 50 && reading.confidence <= 63 && finite(reading.distance) &&
            finite(reading.timestamp) && reading.distance >= 0.02 &&
            reading.timestamp <= sample.timestamp && sample.timestamp - reading.timestamp <= 0.10) {
            valid[count++] = i;
            oldest = std::min(oldest, reading.timestamp);
            newest = std::max(newest, reading.timestamp);
        }
    }
    if (count < 3 || newest - oldest > 0.05 || oldest <= recoveryTimestamp_) return false;
    // Advancing only the oldest timestamp can count held readings from the
    // other sensors repeatedly. Every participating beam must be new relative
    // to its own last recovery frame; staggered acquisition remains supported.
    for (std::size_t n = 0; n < count; ++n)
        if (sample.ranges[valid[n]].timestamp <= lastRecoveryRangeTimestamp_[valid[n]]) return false;
    for (std::size_t n = 0; n < count; ++n)
        lastRecoveryRangeTimestamp_[valid[n]] = sample.ranges[valid[n]].timestamp;
    if (oldest - recoveryTimestamp_ > 0.25) recoveryCount_ = 0;
    recoveryTimestamp_ = oldest;
    std::array<double, maxRanges> candidateX{}, candidateY{};
    std::array<bool, maxRanges> hasX{}, hasY{};
    const double theta = state_.pose.theta;
    const double c = std::cos(theta), s = std::sin(theta);
    for (std::size_t n = 0; n < count; ++n) {
        const std::size_t i = valid[n];
        const RangeMount& mount = config_.mounts[i];
        const double ux = std::sin(theta + mount.heading), uy = std::cos(theta + mount.heading);
        const double distance = sample.ranges[i].distance;
        hasX[n] = std::abs(ux) > 0.4;
        hasY[n] = std::abs(uy) > 0.4;
        candidateX[n] = (ux > 0 ? config_.field.maxX : config_.field.minX)
            - c * mount.x - s * mount.y - ux * distance;
        candidateY[n] = (uy > 0 ? config_.field.maxY : config_.field.minY)
            + s * mount.x - c * mount.y - uy * distance;
    }
    // Enumerate wall-assignment hypotheses analytically at trusted heading.
    // This covers all X/Y assignments without grid quantization or particles.
    std::array<Pose, maxRanges * maxRanges> hypotheses{};
    std::array<double, maxRanges * maxRanges> scores{};
    std::size_t hypothesisCount = 0;
    for (std::size_t i = 0; i < count; ++i) {
        for (std::size_t j = 0; j < count; ++j) {
            if (i == j || !hasX[i] || !hasY[j]) continue;
            const Pose pose{candidateX[i], candidateY[j], theta};
            double score = 0;
            unsigned walls = 0;
            bool plausible = true;
            for (std::size_t n = 0; n < count; ++n) {
                const std::size_t sensor = valid[n];
                const Ray ray = raycast(pose, sensor);
                if (!ray.valid) { plausible = false; break; }
                const double residual = sample.ranges[sensor].distance - ray.distance;
                const double variance = rangeVariance(std::max(sample.ranges[sensor].distance, ray.distance), sensor) /
                    square(ray.incidence);
                if (square(residual) > 9 * variance) { plausible = false; break; }
                walls |= 1u << static_cast<unsigned>(ray.wall);
                score += square(residual) / variance;
            }
            const bool opposing = ((walls & 3u) == 3u) || ((walls & 12u) == 12u);
            if (!plausible || !opposing || !(walls & 3u) || !(walls & 12u)) continue;
            bool duplicate = false;
            for (std::size_t n = 0; n < hypothesisCount; ++n) {
                if (std::hypot(hypotheses[n].x - pose.x, hypotheses[n].y - pose.y) < 0.10) {
                    if (score < scores[n]) { hypotheses[n] = pose; scores[n] = score; }
                    duplicate = true;
                    break;
                }
            }
            if (!duplicate) {
                hypotheses[hypothesisCount] = pose;
                scores[hypothesisCount++] = score;
            }
        }
    }
    if (hypothesisCount == 0) { recoveryCount_ = 0; return false; }
    std::size_t best = 0;
    for (std::size_t i = 1; i < hypothesisCount; ++i) if (scores[i] < scores[best]) best = i;
    for (std::size_t i = 0; i < hypothesisCount; ++i) {
        if (i != best && scores[i] < scores[best] + 9) { recoveryCount_ = 0; return false; }
    }
    const Pose candidate = hypotheses[best];
    const bool consistent = std::hypot(candidate.x - recoveryCandidate_.x, candidate.y - recoveryCandidate_.y) < 0.08;
    recoveryCount_ = consistent ? std::min(recoveryCount_ + 1, 100u) : 1;
    recoveryCandidate_ = candidate;
    if (recoveryCount_ < 5) return false;
    const double dx = candidate.x - state_.pose.x, dy = candidate.y - state_.pose.y;
    const double correction = std::hypot(dx, dy);
    const double fraction = correction > config_.maxRangeCorrection ? config_.maxRangeCorrection / correction : 1;
    state_.pose.x += fraction * dx;
    state_.pose.y += fraction * dy;
    const bool converged = fraction >= 1;
    recovering_ = !converged;
    if (converged) {
        lostPrior_ = false;
        double xx = 0, xy = 0, yy = 0, xHeading = 0, yHeading = 0;
        for (std::size_t n = 0; n < count; ++n) {
            const Ray ray = raycast(candidate, valid[n]);
            const double variance = rangeVariance(std::max(sample.ranges[valid[n]].distance, ray.distance), valid[n]) / square(ray.incidence);
            xx += square(ray.h[0]) / variance;
            xy += ray.h[0] * ray.h[1] / variance;
            yy += square(ray.h[1]) / variance;
            xHeading += ray.h[0] * ray.h[2] / variance;
            yHeading += ray.h[1] * ray.h[2] / variance;
        }
        const double determinant = xx * yy - xy * xy;
        if (determinant > 1e-12) {
            // All beams share one uncertain heading. Treating its effect as
            // independent noise per beam would average it away and discard the
            // position/heading correlation required by later odometry.
            const double cx = -(yy * xHeading - xy * yHeading) / determinant;
            const double cy = -(-xy * xHeading + xx * yHeading) / determinant;
            state_.p[0][0] = yy / determinant + square(cx) * state_.p[2][2];
            state_.p[1][1] = xx / determinant + square(cy) * state_.p[2][2];
            state_.p[0][1] = state_.p[1][0] = -xy / determinant + cx * cy * state_.p[2][2];
            for (std::size_t j = 2; j < 4; ++j) {
                state_.p[0][j] = state_.p[j][0] = cx * state_.p[2][j];
                state_.p[1][j] = state_.p[j][1] = cy * state_.p[2][j];
            }
        }
        recoveryCount_ = 0;
    }
    // The explicit recovery is a new marginal prior, so older delayed factors
    // may not drag the estimator back into the discarded hypothesis.
    historyStart_ = 0;
    historySize_ = 1;
    node(0) = {};
    node(0).state = state_;
    node(0).timestamp = output_.timestamp;
    corroboration_ = {};
    output_.state.pose = state_.pose;
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j) output_.covariance[i][j] = state_.p[i][j];
    output_.health = converged ? Health::degraded : Health::lost;
    output_.confidence = converged ? 0.7 * std::exp(-std::sqrt(state_.p[0][0] + state_.p[1][1]) / 0.3
        - std::sqrt(state_.p[2][2]) / radians(20)) : 0;
    return converged;
}

Estimate Estimator::update(const SensorSample& input) {
    if (!finite(input.timestamp) || (initialized_ && input.timestamp <= previous_.timestamp)) return output_;
    output_.rejectedIncrementMask = 0;
    SensorSample sample = input;
    sample.forwardValid = sample.forwardValid && finite(sample.forward);
    sample.lateralValid = sample.lateralValid && finite(sample.lateral);
    sample.leftValid = sample.leftValid && finite(sample.left);
    sample.rightValid = sample.rightValid && finite(sample.right);
    sample.gyroValid = sample.gyroValid && finite(sample.gyro);
    sample.gyroRateValid = sample.gyroRateValid && finite(sample.gyroRate);
    if (!initialized_) {
        previous_ = sample;
        lastCounterChange_.fill(sample.timestamp);
        initialized_ = true;
        historySize_ = 1;
        node(0) = {};
        node(0).state = state_;
        node(0).timestamp = sample.timestamp;
        output_.timestamp = sample.timestamp;
        return output_;
    }
    Step step;
    step.dt = sample.timestamp - previous_.timestamp;
    if (!finite(step.dt) || step.dt > 1.0) {
        // Cumulative travel cannot reconstruct an unknown sequence of turns
        // across a long acquisition blackout. Rebaseline instead of inventing
        // one constant-curvature movement or admitting unbounded arithmetic.
        previous_ = sample;
        lastCounterChange_.fill(sample.timestamp);
        lostPrior_ = true;
        stationaryTime_ = 0;
        stationaryGyroTime_ = -1;
        stationaryBoundsValid_ = false;
        output_.quiet.dwell = 0;
        noteQuietBreak(32, {}, 0, step.dt, sample.timestamp);
        recoveryCount_ = 0;
        missingTranslationTime_ = missingHeadingTime_ = 0.16;
        state_.v = state_.omega = state_.vSmooth = state_.omegaSmooth = 0;
        state_.p[0][0] += square(0.5);
        state_.p[1][1] += square(0.5);
        state_.p[2][2] += square(radians(30));
        historyStart_ = 0;
        historySize_ = 1;
        node(0) = {};
        node(0).state = state_;
        node(0).timestamp = sample.timestamp;
        publish(sample.timestamp, Step{});
        return output_;
    }
    const double linearLimit = config_.maxSpeed * step.dt + 0.015;
    const double angularLimit = config_.maxOmega * step.dt + radians(2);
    auto delta = [&](bool& valid, bool wasValid, double current, double previous, double limit,
                     double maximumRate, bool& usable, std::size_t sensor) {
        const double change = current - previous;
        // A held cache value is still a valid cumulative baseline. Its next
        // changed reading can contain more than one polling interval of travel.
        // Grant only the configured, bounded cache age to channels actually
        // observed unchanged; a long standstill cannot admit a large reset.
        const double heldTime = std::clamp(previous_.timestamp - lastCounterChange_[sensor],
                                           0.0, config_.maxSensorHold);
        limit += maximumRate * heldTime;
        usable = valid && wasValid && finite(change) && std::abs(change) <= limit;
        if (!valid || !wasValid || !finite(change) || change != 0)
            lastCounterChange_[sensor] = sample.timestamp;
        if (valid && wasValid && !usable) {
            // A rejected cumulative reading cannot become the next frame's
            // trusted baseline. Otherwise its return to normal can be accepted
            // as real travel when scheduling jitter widens that frame's limit.
            // Treat it as a dropout: the next valid reading only rebaselines.
            valid = false;
            output_.rejectedIncrementMask |= static_cast<std::uint8_t>(1u << sensor);
            if (output_.rejectedIncrements < std::numeric_limits<std::uint32_t>::max())
                ++output_.rejectedIncrements;
        }
        return usable ? change : 0.0;
    };
    bool forward, lateral, left, right, gyro;
    const double f = delta(sample.forwardValid, previous_.forwardValid,
        sample.forward * config_.forwardScale, previous_.forward * config_.forwardScale,
        linearLimit + std::abs(config_.forwardOffset) * angularLimit,
        config_.maxSpeed + std::abs(config_.forwardOffset) * config_.maxOmega, forward, 0);
    const double l = delta(sample.lateralValid, previous_.lateralValid,
        sample.lateral * config_.lateralScale, previous_.lateral * config_.lateralScale,
        linearLimit + std::abs(config_.lateralOffset) * angularLimit,
        config_.maxSpeed + std::abs(config_.lateralOffset) * config_.maxOmega, lateral, 1);
    const double dl = delta(sample.leftValid, previous_.leftValid, sample.left, previous_.left,
        linearLimit + 0.5 * config_.trackWidth * angularLimit,
        config_.maxSpeed + 0.5 * config_.trackWidth * config_.maxOmega, left, 2);
    const double dr = delta(sample.rightValid, previous_.rightValid, sample.right, previous_.right,
        linearLimit + 0.5 * config_.trackWidth * angularLimit,
        config_.maxSpeed + 0.5 * config_.trackWidth * config_.maxOmega, right, 3);
    const double dg = delta(sample.gyroValid, previous_.gyroValid,
        sample.gyro * config_.gyroScale, previous_.gyro * config_.gyroScale,
        angularLimit, config_.maxOmega, gyro, 4);
    const bool usable[] = {forward, lateral, gyro};
    for (std::size_t i = 0; i < output_.unusableIncrements.size(); ++i) {
        auto& count = output_.unusableIncrements[i];
        if (!usable[i] && count < std::numeric_limits<std::uint32_t>::max()) ++count;
    }
    if (output_.rejectedIncrementMask)
        output_.lastRejectedIncrementMask = output_.rejectedIncrementMask;
    const bool drive = left && right;
    const double driveHeading = drive ? (dl - dr) / config_.trackWidth : 0;
    const double driveForward = drive ? (dl + dr) / 2 : 0;
    step.headingSource = gyro ? HeadingSource::imu
        : (drive ? HeadingSource::driveEncoders : HeadingSource::unavailable);
    step.heading = gyro ? dg : driveHeading;
    const double angle = step.heading - (gyro && !config_.nativeImuHeading ? state_.bias * step.dt : 0);
    step.forwardPod = forward;
    step.forwardRotationOffset = forward ? config_.forwardOffset : 0;
    step.lateralPod = lateral;
    step.translation = forward || drive;
    step.forward = forward ? f : driveForward;
    step.lateral = lateral ? l : 0;
    // With one drive encoder plus gyro, the centre travel remains observable.
    if (!step.translation && gyro && (left || right)) {
        step.forward = left ? dl : dr;
        step.forwardRotationOffset = (left ? -0.5 : 0.5) * config_.trackWidth;
        step.translation = true;
    }
    double disagreement = 0;
    if (forward && drive)
        disagreement = std::abs(f + config_.forwardOffset * angle - driveForward) /
            (0.003 + 0.18 * std::max<double>(std::abs(driveForward), std::abs(f)));
    if (gyro && drive)
        disagreement = std::max<double>(disagreement, std::abs(angle - driveHeading) /
            (radians(0.5) + 0.25 * std::max<double>(std::abs(angle), std::abs(driveHeading))));
    const double slip = std::clamp((disagreement - 1) / 3, 0.0, 1.0);
    output_.slip += -std::expm1(-step.dt / 0.08) * (slip - output_.slip);
    step.slip = output_.slip;
    // Wheel evidence must be independent of the gyro before learning its bias.
    // The optional tolerance bounds the full cumulative range in a quiet window,
    // not each increment: small persistent motion must eventually leave it.
    constexpr double stillThreshold = 1e-9;
    bool moving = false;
    auto inspectWheel = [&](bool valid, double change) {
        if (valid) moving = moving || std::abs(change) > stillThreshold;
    };
    inspectWheel(forward, f); inspectWheel(lateral, l);
    inspectWheel(left, dl); inspectWheel(right, dr);
    // Two orthogonal pods cannot observe yaw independently: rotating around
    // the intersection of their rolling axes leaves both counters unchanged.
    // Require two longitudinal wheel axes at distinct lateral positions. A
    // pod coincident with the sole available drive wheel is also redundant.
    const double halfTrack = config_.trackWidth / 2;
    // Nearly coincident axes amplify bounded encoder noise into arbitrary yaw.
    // Each wheel can span the configured tolerance: their difference must not
    // imply more than 0.1 degree across a quiet window.
    const double minimumYawSeparation = std::max(1e-6, 2 * config_.stationaryEncoderSpan / radians(.1));
    const bool quietDrive = drive && config_.trackWidth > minimumYawSeparation;
    const bool quietLeft = forward && left && std::abs(config_.forwardOffset + halfTrack) > minimumYawSeparation;
    const bool quietRight = forward && right && std::abs(config_.forwardOffset - halfTrack) > minimumYawSeparation;
    const bool wheelYawObservable = quietDrive || quietLeft || quietRight;
    double wheelHeading = 0, wheelSeparation = config_.trackWidth;
    if (quietDrive) wheelHeading = driveHeading;
    else if (quietLeft) {
        wheelSeparation = config_.forwardOffset + halfTrack;
        wheelHeading = (dl - f) / wheelSeparation;
    } else if (quietRight) {
        wheelSeparation = halfTrack - config_.forwardOffset;
        wheelHeading = (f - dr) / wheelSeparation;
    }
    const bool eligible = step.dt <= 0.05 + 1e-9 && wheelYawObservable && gyro &&
        output_.rejectedIncrementMask == 0 &&
        std::abs(dg / step.dt) < radians(3);
    const double observedGyroRate = gyro ? dg / step.dt : 0;
    std::uint16_t quietReasons = 0;
    if (step.dt > .05 + 1e-9) quietReasons |= 32;
    if (!wheelYawObservable || !gyro) quietReasons |= 64;
    if (gyro && std::abs(observedGyroRate) >= radians(3)) quietReasons |= 16;
    if (output_.rejectedIncrementMask) quietReasons |= 256;
    std::array<double, 4> observedSpans{};
    if (config_.stationaryEncoderSpan > 0) {
        const std::array<double, 4> positions{sample.forward * config_.forwardScale,
            sample.lateral * config_.lateralScale, sample.left, sample.right};
        const std::uint8_t mask = static_cast<std::uint8_t>((forward ? 1 : 0) | (lateral ? 2 : 0) |
                                                          (left ? 4 : 0) | (right ? 8 : 0));
        if (stationaryBoundsValid_) {
            if (mask != stationaryWheelMask_) quietReasons |= 128;
            for (std::size_t i = 0; i < positions.size(); ++i) if (mask & stationaryWheelMask_ & (1u << i)) {
                observedSpans[i] = std::max(stationaryMaximum_[i], positions[i]) -
                                   std::min(stationaryMinimum_[i], positions[i]);
                if (observedSpans[i] > config_.stationaryEncoderSpan + 1e-12)
                    quietReasons |= static_cast<std::uint16_t>(1u << i);
            }
        }
        bool restart = !eligible || !stationaryBoundsValid_ || mask != stationaryWheelMask_;
        if (!restart) {
            for (std::size_t i = 0; i < positions.size(); ++i) if (mask & (1u << i)) {
                stationaryMinimum_[i] = std::min(stationaryMinimum_[i], positions[i]);
                stationaryMaximum_[i] = std::max(stationaryMaximum_[i], positions[i]);
                if (stationaryMaximum_[i] - stationaryMinimum_[i] > config_.stationaryEncoderSpan + 1e-12)
                    restart = true;
            }
        }
        if (restart) {
            stationaryMinimum_ = stationaryMaximum_ = positions;
            stationaryWheelMask_ = mask;
            stationaryBoundsValid_ = eligible;
            stationaryTime_ = 0;
        } else stationaryTime_ += step.dt;
    } else {
        stationaryBoundsValid_ = false;
        stationaryTime_ = eligible && !moving ? stationaryTime_ + step.dt : 0;
        observedSpans = {forward ? std::abs(f) : 0, lateral ? std::abs(l) : 0,
                         left ? std::abs(dl) : 0, right ? std::abs(dr) : 0};
        for (std::size_t i = 0; i < observedSpans.size(); ++i)
            if (observedSpans[i] > stillThreshold) quietReasons |= static_cast<std::uint16_t>(1u << i);
    }
    noteQuietBreak(quietReasons, observedSpans, observedGyroRate, step.dt, sample.timestamp);
    output_.quiet.dwell = stationaryTime_;
    step.stationary = stationaryTime_ >= 0.25;
    // Quiet is a bounded-motion observation, not proof of exactly zero yaw.
    // The standard policy uses independent wheel yaw to preserve slow rotation
    // without integrating IMU drift at rest. Native policy retains this quiet
    // diagnostic but uses the IMU angle in propagate even during the dwell.
    step.stationaryHeading = wheelHeading;
    const double rawRate = sample.gyroRate * config_.gyroScale;
    const bool reliableRate = sample.gyroRateValid && std::abs(rawRate) < radians(3) &&
        std::abs(rawRate - dg / step.dt) < radians(1);
    const bool rateObservation = reliableRate && config_.stationaryEncoderSpan == 0;
    step.observeBias = !config_.nativeImuHeading && step.stationary && rateObservation;
    step.biasReading = rawRate;
    step.biasVariance = square(radians(0.35));
    if (config_.nativeImuHeading || !step.stationary || rateObservation) stationaryGyroTime_ = -1;
    if (!config_.nativeImuHeading && step.stationary && !rateObservation) {
        // Averaging non-overlapping gyro increments avoids pretending that
        // consecutive differences of the same quantized angle are independent.
        if (stationaryGyroTime_ < 0) {
            stationaryGyro_ = sample.gyro * config_.gyroScale;
            stationaryGyroTime_ = sample.timestamp;
            stationaryWheelYaw_ = 0;
        } else {
            stationaryWheelYaw_ += wheelHeading;
            if (sample.timestamp - stationaryGyroTime_ >= 0.5) {
                const double window = sample.timestamp - stationaryGyroTime_;
                step.biasReading = (sample.gyro * config_.gyroScale - stationaryGyro_ - stationaryWheelYaw_) / window;
                step.biasVariance = 2 * square(config_.gyroStd / window) + square(radians(0.02))
                    + 4 * square(config_.stationaryEncoderSpan / (wheelSeparation * window));
                step.observeBias = true;
                stationaryGyro_ = sample.gyro * config_.gyroScale;
                stationaryGyroTime_ = sample.timestamp;
                stationaryWheelYaw_ = 0;
            }
        }
    }
    missingTranslationTime_ = step.translation ? 0 : missingTranslationTime_ + step.dt;
    missingHeadingTime_ = step.headingSource != HeadingSource::unavailable ? 0 : missingHeadingTime_ + step.dt;
    previous_ = sample; // Every reconnect establishes a fresh cumulative baseline.
    append(step, sample.timestamp);
    for (std::size_t sensor = 0; sensor < config_.rangeCount; ++sensor)
        processRange(sensor, sample.ranges[sensor], sample.timestamp);
    publish(sample.timestamp, step);
    if (output_.health == Health::lost || state_.p[0][0] + state_.p[1][1] > square(0.20) || recoveryCount_ >= 5)
        relocalize(sample);
    return output_;
}
} // namespace nexus
