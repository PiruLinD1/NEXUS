#include "nexus/controller.hpp"

#include <limits>

namespace nexus {
namespace {
double bound(double value, double lo, double hi) { return std::max(lo, std::min(hi, value)); }
double finiteOr(double value, double fallback) { return std::isfinite(value) ? value : fallback; }
bool finite(const DriveState& s) {
    return std::isfinite(s.pose.x) && std::isfinite(s.pose.y) && std::isfinite(s.pose.theta) &&
           std::isfinite(s.v) && std::isfinite(s.omega) && std::isfinite(s.lateralV);
}
bool usable(const Estimate& estimate) {
    if (!finite(estimate.state) || !std::isfinite(estimate.slip) ||
        estimate.health == Health::lost) return false;
    for (std::size_t i = 0; i < estimate.covariance.size(); ++i) {
        if (estimate.covariance[i][i] < 0) return false;
        for (double value : estimate.covariance[i]) if (!std::isfinite(value)) return false;
    }
    return true;
}
double smoothFriction(double speed) { return std::tanh(speed / 0.07); }
}

Controller::Controller(DynamicsConfig config) { reconfigure(config); }

void Controller::reconfigure(DynamicsConfig config) {
    const DynamicsConfig defaults;
    config_ = config;
    config_.trackWidth = std::max(0.05, finiteOr(config_.trackWidth, defaults.trackWidth));
    config_.kVLeft = std::max(0.1, finiteOr(config_.kVLeft, defaults.kVLeft));
    config_.kVRight = std::max(0.1, finiteOr(config_.kVRight, defaults.kVRight));
    config_.kALeft = std::max(0.05, finiteOr(config_.kALeft, defaults.kALeft));
    config_.kARight = std::max(0.05, finiteOr(config_.kARight, defaults.kARight));
    config_.kSLeft = std::max(0.0, finiteOr(config_.kSLeft, defaults.kSLeft));
    config_.kSRight = std::max(0.0, finiteOr(config_.kSRight, defaults.kSRight));
    config_.maxAcceleration = std::max(0.1, finiteOr(config_.maxAcceleration, defaults.maxAcceleration));
    config_.maxLateralAcceleration = std::max(0.1, finiteOr(config_.maxLateralAcceleration, defaults.maxLateralAcceleration));
    config_.maxSpeed = std::max(0.05, finiteOr(config_.maxSpeed, defaults.maxSpeed));
    config_.maxOmega = std::max(0.1, finiteOr(config_.maxOmega, defaults.maxOmega));
    config_.maxVoltage = bound(finiteOr(config_.maxVoltage, defaults.maxVoltage), 0.1, 12.0);
    config_.commandLatency = bound(finiteOr(config_.commandLatency, defaults.commandLatency), 0.0, 0.15);
    reset();
}

void Controller::reset() {
    active_ = warm_ = false;
    pathReverse_ = false;
    previous_ = {};
    stats_ = {};
    pathTime_ = settledTime_ = runTime_ = lastReplan_ = 0;
    lastScale_ = 1;
    pathCount_ = 1;
    path_[0] = {};
    commandTime_ = 0;
    commandFirst_ = commandCount_ = 0;
    commandBeforeHistory_ = {};
    wheelLoad_ = {};
    lateralVelocity_ = 0;
    lateralDecay_ = 5; lateralEnergy_ = lateralDamping_ = 0;
    previousObservation_ = {};
    previousObservationTime_ = 0;
    haveObservation_ = false;
}

void Controller::rememberCommand(Voltage voltage) {
    // Preserve the final command preceding the prediction window. Older
    // entries cannot affect any future prediction with this fixed latency.
    const double firstRelevant = commandTime_ - config_.commandLatency - .1;
    while (commandCount_ > 1) {
        const std::size_t next = (commandFirst_ + 1) % commandCapacity;
        if (commandHistory_[next].issuedAt > firstRelevant) break;
        commandBeforeHistory_ = commandHistory_[commandFirst_].voltage;
        commandFirst_ = next;
        --commandCount_;
    }
    if (commandCount_ == commandCapacity) {
        // Calls faster than the supported 1ms period still have bounded memory.
        commandBeforeHistory_ = commandHistory_[commandFirst_].voltage;
        commandFirst_ = (commandFirst_ + 1) % commandCapacity;
        --commandCount_;
    }
    const std::size_t index = (commandFirst_ + commandCount_) % commandCapacity;
    commandHistory_[index] = {commandTime_, {{voltage.left, voltage.right}}};
    ++commandCount_;
}

Controller::State Controller::predictCommandHistory(State predicted, double from, double until, double voltage) const {
    Input held = commandBeforeHistory_;
    const auto advance = [&](double endTime) {
        if (endTime > from) predicted = integrate(predicted,
            {{bound(held[0], -voltage, voltage), bound(held[1], -voltage, voltage)}}, endTime - from,
            nullptr, nullptr, lateralVelocity_);
        from = endTime;
    };
    // These bounds are emission times. Shift actual times by -commandLatency
    // to replay precisely the inputs applied during the prediction interval.
    for (std::size_t i = 0; i < commandCount_; ++i) {
        const TimedCommand& command = commandHistory_[(commandFirst_ + i) % commandCapacity];
        if (command.issuedAt >= until) break;
        if (command.issuedAt > from) advance(command.issuedAt);
        held = command.voltage;
    }
    advance(until);
    return predicted;
}

Controller::State Controller::encode(const DriveState& s) const {
    return {{s.pose.x, s.pose.y, s.pose.theta,
             s.v + s.omega * config_.trackWidth / 2,
             s.v - s.omega * config_.trackWidth / 2}};
}
void Controller::observeWheelLoad(const State& observed, double observedAt, double voltage, double slip) {
    const double elapsed = observedAt - previousObservationTime_;
    if (haveObservation_ && elapsed >= .005 && elapsed <= .1 && slip < .25) {
        const State predicted = predictCommandHistory(previousObservation_, previousObservationTime_, observedAt, voltage);
        const double gain = elapsed / (.12 + elapsed);
        for (std::size_t side = 0; side < nu; ++side) {
            const double ka = side == 0 ? config_.kALeft : config_.kARight;
            const double correction = gain * ka * (predicted[3 + side] - observed[3 + side]) / elapsed;
            wheelLoad_[side] = bound(wheelLoad_[side] + bound(correction, -4 * elapsed, 4 * elapsed),
                                    -std::min(3.0, voltage * .4), std::min(3.0, voltage * .4));
        }
    }
    previousObservation_ = observed;
    previousObservationTime_ = observedAt;
    haveObservation_ = true;
}
DriveState Controller::decode(const State& x) const {
    return {{x[0], x[1], wrap(x[2])}, (x[3] + x[4]) / 2,
            (x[3] - x[4]) / config_.trackWidth};
}

// Analytic Jacobians are propagated through each midpoint integration substep.
// The bounded wheel acceleration represents available longitudinal traction;
// lateral traction is additionally constrained in reference timing and the NLP.
Controller::State Controller::integrate(State x, const Input& u, double dt,
                                        Matrix* jacA, InputJacobian* jacB, double lateralVelocity) const {
    Matrix totalA{};
    InputJacobian totalB{};
    if (jacA) for (std::size_t j = 0; j < nx; ++j) totalA[j][j] = 1;
    const unsigned steps = static_cast<unsigned>(std::max<double>(1.0, std::ceil(dt / 0.01)));
    const double h = dt / steps, width = config_.trackWidth;
    for (unsigned step = 0; step < steps; ++step) {
        double half[2], next[2], dhdx[2], dhdu[2], dndx[2], dndu[2];
        for (std::size_t side = 0; side < nu; ++side) {
            const double kv = side == 0 ? config_.kVLeft : config_.kVRight;
            const double ka = side == 0 ? config_.kALeft : config_.kARight;
            const double ks = side == 0 ? config_.kSLeft : config_.kSRight;
            auto acceleration = [&](double speed, double& dx, double& du) {
                const double friction = smoothFriction(speed);
                const double raw = (u[side] - wheelLoad_[side] - kv * speed - ks * friction) / ka;
                const bool free = std::abs(raw) < config_.maxAcceleration;
                dx = free ? -(kv + ks / 0.07 * (1 - friction * friction)) / ka : 0;
                du = free ? 1 / ka : 0;
                return bound(raw, -config_.maxAcceleration, config_.maxAcceleration);
            };
            double adx, adu;
            const double first = acceleration(x[3 + side], adx, adu);
            half[side] = x[3 + side] + h * first / 2;
            dhdx[side] = 1 + h * adx / 2;
            dhdu[side] = h * adu / 2;
            const double second = acceleration(half[side], adx, adu);
            next[side] = x[3 + side] + h * second;
            dndx[side] = 1 + h * adx * dhdx[side];
            dndu[side] = h * (adu + adx * dhdu[side]);
        }
        const double v = (half[0] + half[1]) / 2;
        const double omega = (half[0] - half[1]) / width;
        const double middleHeading = x[2] + h * omega / 2;
        const double s = std::sin(middleHeading), c = std::cos(middleHeading);
        if (jacA) {
            Matrix localA{};
            InputJacobian localB{};
            localA[0][0] = localA[1][1] = localA[2][2] = 1;
            localA[0][2] = h * (v * c - lateralVelocity * s);
            localA[1][2] = -h * (v * s + lateralVelocity * c);
            for (std::size_t side = 0; side < nu; ++side) {
                const double sign = side == 0 ? 1.0 : -1.0;
                const double dpx = h * (0.5 * s + (v * c - lateralVelocity * s) * h * sign / (2 * width));
                const double dpy = h * (0.5 * c - (v * s + lateralVelocity * c) * h * sign / (2 * width));
                localA[0][3 + side] = dpx * dhdx[side];
                localA[1][3 + side] = dpy * dhdx[side];
                localA[2][3 + side] = h * sign / width * dhdx[side];
                localA[3 + side][3 + side] = dndx[side];
                localB[0][side] = dpx * dhdu[side];
                localB[1][side] = dpy * dhdu[side];
                localB[2][side] = h * sign / width * dhdu[side];
                localB[3 + side][side] = dndu[side];
            }
            Matrix mergedA{};
            InputJacobian mergedB = localB;
            for (std::size_t i = 0; i < nx; ++i) {
                for (std::size_t j = 0; j < nx; ++j)
                    for (std::size_t k = 0; k < nx; ++k) mergedA[i][j] += localA[i][k] * totalA[k][j];
                for (std::size_t j = 0; j < nu; ++j)
                    for (std::size_t k = 0; k < nx; ++k) mergedB[i][j] += localA[i][k] * totalB[k][j];
            }
            totalA = mergedA;
            totalB = mergedB;
        }
        x[0] += h * (v * s + lateralVelocity * c);
        x[1] += h * (v * c - lateralVelocity * s);
        x[2] += h * omega;
        x[3] = next[0];
        x[4] = next[1];
    }
    if (jacA) *jacA = totalA;
    if (jacB) *jacB = totalB;
    return x;
}

DriveState Controller::predict(const DriveState& state, Voltage voltage, double dt,
                               double batteryVolts) const {
    const double limit = bound(batteryVolts, 0.0, config_.maxVoltage);
    auto prediction = decode(integrate(encode(state), {{bound(voltage.left, -limit, limit),
                          bound(voltage.right, -limit, limit)}}, bound(dt, 0.0, 1.0),
                          nullptr, nullptr, state.lateralV));
    prediction.lateralV = state.lateralV;
    return prediction;
}

void Controller::start(const DriveState& state, Target target, MotionOptions options) {
    reset();
    if (!finite(state) || !std::isfinite(target.pose.x) || !std::isfinite(target.pose.y) ||
        !std::isfinite(target.pose.theta) || !std::isfinite(options.maxSpeed) ||
        !std::isfinite(options.positionTolerance) || !std::isfinite(options.headingTolerance) ||
        !std::isfinite(options.settleTime) || !std::isfinite(options.timeout)) return;
    target_ = target;
    target_.pose.theta = wrap(target_.pose.theta);
    options_ = options;
    options_.maxSpeed = bound(options_.maxSpeed, 0.02, config_.maxSpeed);
    options_.positionTolerance = std::max(0.001, options_.positionTolerance);
    options_.headingTolerance = std::max(radians(0.1), options_.headingTolerance);
    options_.settleTime = std::max(0.0, options_.settleTime);
    pathReverse_ = options_.reverse;
    active_ = true;
    if (target_.turnOnly) { target_.pose.x = state.pose.x; target_.pose.y = state.pose.y; }
    if (active_) makePath(state);
}

void Controller::makePath(const DriveState& state, bool chooseDirection) {
    buildPath(state, pathReverse_);
    if (!chooseDirection || target_.turnOnly || exhausted()) return;

    // Compare two feasible, acceleration-limited correction paths. Preserve
    // the requested initial direction, but do not force every terminal repair
    // to use it: a target just behind us is often better reached in reverse.
    const auto duration = [&](bool reverse) {
        const double opposedSpeed = std::max(0.0, reverse ? state.v : -state.v);
        return trajectoryDuration() + opposedSpeed / (config_.maxAcceleration * .7);
    };
    double bestDuration = duration(pathReverse_);
    bool bestReverse = pathReverse_;
    double bestStaging = 0;
    buildPath(state, !pathReverse_);
    const double alternativeDuration = duration(!pathReverse_);
    // Keep the existing direction for near ties. Along with the replan
    // cooldown, this avoids switching back and forth on sensor noise.
    if (alternativeDuration + .12 < bestDuration * .9) {
        bestReverse = !pathReverse_;
        bestDuration = alternativeDuration;
    }

    // A lateral-only error has identical forward/reverse S paths, both with
    // very large heading excursions. Also compare a shallow approach to a
    // staging pose, a stopped gear change, and a straight final approach.
    // No unmeasured lateral tyre model is assumed: geometry and acceleration
    // limits determine the candidates, and actual pod feedback closes them.
    const double heading = target_.constrainHeading ? target_.pose.theta : state.pose.theta;
    const double dx = target_.pose.x - state.pose.x, dy = target_.pose.y - state.pose.y;
    const double lateral = std::abs(dx * std::cos(heading) - dy * std::sin(heading));
    const double longitudinal = dx * std::sin(heading) + dy * std::cos(heading);
    if (std::hypot(dx, dy) < .35 && lateral > options_.positionTolerance &&
        lateral > .5 * std::abs(longitudinal)) {
        const double staging = bound(2 * lateral, .06, .18);
        for (double lead : {staging, std::min(.22, staging * 1.5)}) {
            for (bool reverse : {pathReverse_, !pathReverse_}) {
                if (exhausted()) break;
                buildCorrectionPath(state, reverse, lead);
                const double candidate = duration(reverse);
                if (candidate + .12 < bestDuration * .9) {
                    bestDuration = candidate;
                    bestReverse = reverse;
                    bestStaging = lead;
                }
            }
        }
    }
    pathReverse_ = bestReverse;
    if (bestStaging > 0) buildCorrectionPath(state, bestReverse, bestStaging);
    else buildPath(state, bestReverse);
}

void Controller::buildCorrectionPath(const DriveState& state, bool reverse, double stagingDistance) {
    const Target finalTarget = target_;
    const double heading = finalTarget.constrainHeading ? finalTarget.pose.theta : state.pose.theta;
    const double direction = reverse ? -1.0 : 1.0;
    const Pose staging{finalTarget.pose.x + direction * stagingDistance * std::sin(heading),
                       finalTarget.pose.y + direction * stagingDistance * std::cos(heading), heading};
    target_ = {staging, true, false};
    buildPath(state, reverse);
    // Preserve 65 samples on the stack (about 3 KiB), never a second Controller.
    // Subsampling a piecewise-linear wheel reference preserves its acceleration
    // bound. Both segments end/start at zero wheel speed at the gear change.
    constexpr std::size_t firstCount = (pathCapacity + 1) / 2;
    constexpr std::size_t secondIntervals = pathCapacity - firstCount - 1;
    std::array<Knot, firstCount> first{};
    for (std::size_t i = 0; i < firstCount; ++i) first[i] = path_[i * 2];
    target_ = finalTarget;
    buildPath({staging, 0, 0}, !reverse);
    constexpr double stoppedDwell = .12;
    const double startSecond = first.back().time + stoppedDwell;
    // The first segment unwraps heading from the measured start, whereas the
    // second starts at the requested staging heading. Align their equivalent
    // 2*pi branches before linear interpolation across the stopped cusp.
    const double headingShift = first.back().pose.theta - path_[0].pose.theta;
    // Move from the end so every source knot survives until it has been read.
    for (std::size_t i = secondIntervals; i > 0; --i) {
        const std::size_t source = (i * (pathCapacity - 1) + secondIntervals / 2) / secondIntervals;
        path_[firstCount + i] = path_[source];
        path_[firstCount + i].time += startSecond;
        path_[firstCount + i].pose.theta += headingShift;
    }
    for (std::size_t i = 0; i < firstCount; ++i) path_[i] = first[i];
    path_[firstCount] = first.back();
    path_[firstCount].time = startSecond;
    pathCount_ = pathCapacity;
}

void Controller::buildPath(const DriveState& state, bool reverse) {
    pathTime_ = 0;
    pathCount_ = pathCapacity;
    double dx = target_.pose.x - state.pose.x, dy = target_.pose.y - state.pose.y;
    double distance = std::hypot(dx, dy);
    if (target_.turnOnly || distance < 0.0005) {
        const double change = target_.constrainHeading || target_.turnOnly ?
                              wrap(target_.pose.theta - state.pose.theta) : 0;
        // Quintic time scaling: peak s'=1.875, peak |s''|=5.774.
        const double angularLimit = std::min(config_.maxOmega,
                                      2 * options_.maxSpeed / config_.trackWidth);
        const double duration = std::max<double>(0.05, std::max<double>(1.875 * std::abs(change) / angularLimit,
                         std::sqrt(5.774 * std::abs(change) * config_.trackWidth /
                                   (2 * config_.maxAcceleration * 0.75))));
        for (std::size_t i = 0; i < pathCount_; ++i) {
            const double t = static_cast<double>(i) / (pathCount_ - 1);
            const double f = t * t * t * (10 + t * (-15 + 6 * t));
            const double df = 30 * t * t * square(1 - t);
            path_[i] = {{state.pose.x, state.pose.y, state.pose.theta + change * f},
                         0, change * df / duration, t * duration};
        }
        return;
    }
    const double direction = reverse ? -1 : 1;
    const double travelOffset = reverse ? pi : 0;
    const double startAngle = state.pose.theta + travelOffset;
    const double bearing = std::atan2(dx, dy);
    const double bearingChange = wrap(bearing - startAngle);
    // With no requested arrival heading, use the end tangent of the circular
    // arc through start and goal. A diagonal point need not countersteer back
    // to the start-to-goal bearing. Explicit pose headings are unchanged.
    const double endAngle = target_.constrainHeading ? target_.pose.theta + travelOffset :
        std::abs(bearingChange) <= pi / 2 ? startAngle + 2 * bearingChange : bearing;
    // Align before the endpoint, leaving a straight braking corridor instead
    // of demanding the last countersteer at the same instant as the stop.
    const double approach = target_.constrainHeading && distance > .35 ?
                            std::min(.12, distance * .15) : 0;
    const double curveEndX = target_.pose.x - approach * std::sin(endAngle);
    const double curveEndY = target_.pose.y - approach * std::cos(endAngle);
    dx = curveEndX - state.pose.x; dy = curveEndY - state.pose.y;
    distance = std::hypot(dx, dy);
    const std::size_t curveIntervals = approach > 0 ? 96 : pathCount_ - 1;
    // Scale the tangents with the remaining distance. A fixed minimum tangent
    // makes millimetre-scale terminal corrections form loops and near-cusps.
    const double tangent = distance * 0.85;
    const double vx0 = tangent * std::sin(startAngle), vy0 = tangent * std::cos(startAngle);
    const double vx1 = tangent * std::sin(endAngle), vy1 = tangent * std::cos(endAngle);
    // Collinear opposing endpoint tangents create a cusp in a plain Hermite
    // spline. A sixth-order transverse bow preserves endpoint position, first
    // and second derivatives while giving that degenerate case a regular path.
    const double startProjection = (vx0 * dx + vy0 * dy) / (tangent * distance);
    const double endProjection = (vx1 * dx + vy1 * dy) / (tangent * distance);
    const double startCross = (vx0 * dy - vy0 * dx) / (tangent * distance);
    const double endCross = (vx1 * dy - vy1 * dx) / (tangent * distance);
    const double bow = (std::min(startProjection, endProjection) < -0.3 &&
                        std::max<double>(std::abs(startCross), std::abs(endCross)) < 0.25) ? 0.4 * distance : 0;
    std::array<double, pathCapacity> length{}, curvature{}, velocity{}, limits{};
    double lastHeading = state.pose.theta;
    for (std::size_t i = 0; i < pathCount_; ++i) {
        const double t = std::min(1.0, static_cast<double>(i) / curveIntervals);
        auto polynomial = [t](double p0, double p1, double v0, double v1,
                              double& position, double& first, double& second) {
            const double a = 10 * (p1 - p0) - 6 * v0 - 4 * v1;
            const double b = -15 * (p1 - p0) + 8 * v0 + 7 * v1;
            const double c = 6 * (p1 - p0) - 3 * v0 - 3 * v1;
            position = p0 + v0 * t + t * t * t * (a + t * (b + t * c));
            first = v0 + t * t * (3 * a + t * (4 * b + t * 5 * c));
            second = t * (6 * a + t * (12 * b + t * 20 * c));
        };
        double x, y, xp, yp, xpp, ypp;
        polynomial(state.pose.x, curveEndX, vx0, vx1, x, xp, xpp);
        polynomial(state.pose.y, curveEndY, vy0, vy1, y, yp, ypp);
        const double bump = 64 * t * t * t * (1 - t) * (1 - t) * (1 - t);
        const double bumpFirst = 192 * t * t * square(1 - t) * (1 - 2 * t);
        const double bumpSecond = 384 * t * (1 - t) * (1 - 5 * t + 5 * t * t);
        x += bow * dy / distance * bump;
        y -= bow * dx / distance * bump;
        xp += bow * dy / distance * bumpFirst;
        yp -= bow * dx / distance * bumpFirst;
        xpp += bow * dy / distance * bumpSecond;
        ypp -= bow * dx / distance * bumpSecond;
        if (i > curveIntervals) {
            const double fraction = static_cast<double>(i - curveIntervals) / (pathCount_ - 1 - curveIntervals);
            x = curveEndX + fraction * approach * std::sin(endAngle);
            y = curveEndY + fraction * approach * std::cos(endAngle);
            xp = approach * std::sin(endAngle); yp = approach * std::cos(endAngle);
            xpp = ypp = 0;
        }
        const double derivative = std::max<double>(1e-8, std::hypot(xp, yp));
        const double heading = std::atan2(xp, yp) - travelOffset;
        lastHeading += wrap(heading - lastHeading);
        path_[i].pose = {x, y, lastHeading};
        if (i) length[i] = length[i - 1] + std::hypot(x - path_[i - 1].pose.x, y - path_[i - 1].pose.y);
        curvature[i] = (yp * xpp - xp * ypp) / (derivative * derivative * derivative);
        const double k = std::abs(curvature[i]);
        const double wheelRatio = 1 + k * config_.trackWidth / 2;
        limits[i] = std::min(options_.maxSpeed, config_.maxSpeed / wheelRatio);
        if (k > 1e-8) limits[i] = std::min<double>(limits[i], std::min<double>(config_.maxOmega / k,
                                                    std::sqrt(config_.maxLateralAcceleration * 0.75 / k)));
        velocity[i] = limits[i];
    }
    velocity[0] = std::min(limits[0], std::max(0.0, direction * state.v / lastScale_));
    velocity[pathCount_ - 1] = 0;
    // Forward/backward reachability enforces both wheel acceleration bounds,
    // including changing curvature, rather than only body acceleration.
    for (unsigned pass = 0; pass < 3; ++pass) {
        for (std::size_t i = 1; i < pathCount_; ++i) {
            const double ds = std::max(1e-7, length[i] - length[i - 1]);
            const double ratio = 1 + std::max<double>(std::abs(curvature[i]), std::abs(curvature[i - 1])) * config_.trackWidth / 2;
            const double curvatureDemand = velocity[i - 1] * velocity[i - 1] *
                     std::abs(curvature[i] - curvature[i - 1]) * config_.trackWidth / (2 * ds);
            const double acceleration = std::max(0.08, config_.maxAcceleration * 0.7 - curvatureDemand) / ratio;
            velocity[i] = std::min<double>(velocity[i], std::sqrt(square(velocity[i - 1]) + 2 * acceleration * ds));
        }
        for (std::size_t j = pathCount_ - 1; j > 0; --j) {
            const std::size_t i = j - 1;
            const double ds = std::max(1e-7, length[j] - length[i]);
            const double ratio = 1 + std::max<double>(std::abs(curvature[i]), std::abs(curvature[j])) * config_.trackWidth / 2;
            const double curvatureDemand = velocity[j] * velocity[j] *
                     std::abs(curvature[j] - curvature[i]) * config_.trackWidth / (2 * ds);
            const double acceleration = std::max(0.08, config_.maxAcceleration * 0.7 - curvatureDemand) / ratio;
            velocity[i] = std::min<double>(velocity[i], std::sqrt(square(velocity[j]) + 2 * acceleration * ds));
        }
    }
    path_[0].time = 0;
    for (std::size_t i = 0; i < pathCount_; ++i) {
        path_[i].v = direction * velocity[i];
        path_[i].omega = curvature[i] * velocity[i];
        if (i) path_[i].time = path_[i - 1].time + std::max(1e-6,
                    2 * (length[i] - length[i - 1]) / std::max(1e-9, velocity[i] + velocity[i - 1]));
    }
    // Curvature reachability above is only an approximation. In particular a
    // tight S-bend can demand more wheel acceleration than its body-speed
    // profile suggests. Verify the actual piecewise-linear wheel reference and
    // stretch its timing if needed: a time scale s reduces acceleration by s^2.
    double peakAcceleration = 0;
    for (std::size_t i = 1; i < pathCount_; ++i) {
        const double interval = path_[i].time - path_[i - 1].time;
        const double linearChange = path_[i].v - path_[i - 1].v;
        const double angularChange = (path_[i].omega - path_[i - 1].omega) * config_.trackWidth / 2;
        peakAcceleration = std::max<double>(peakAcceleration,
                                   (std::abs(linearChange) + std::abs(angularChange)) /
                                   std::max(1e-6, interval));
    }
    const double timeScale = std::max<double>(1.0, std::sqrt(peakAcceleration / config_.maxAcceleration));
    for (std::size_t i = 0; i < pathCount_; ++i) {
        path_[i].v /= timeScale;
        path_[i].omega /= timeScale;
        path_[i].time *= timeScale;
    }
}

Controller::Reference Controller::sample(double time, double scale, double voltage) const {
    Reference ref{};
    std::size_t i = 1;
    while (i + 1 < pathCount_ && path_[i].time < time) ++i;
    const Knot& a = path_[i - 1];
    const Knot& b = path_[i];
    const double interval = std::max(1e-6, b.time - a.time);
    const double f = bound((time - a.time) / interval, 0.0, 1.0);
    const auto mix = [f](double first, double second) { return first + f * (second - first); };
    DriveState s{{mix(a.pose.x, b.pose.x), mix(a.pose.y, b.pose.y), mix(a.pose.theta, b.pose.theta)},
                  mix(a.v, b.v) * scale, mix(a.omega, b.omega) * scale};
    ref.terminal = time >= path_[pathCount_ - 1].time;
    if (ref.terminal) { s.pose = path_[pathCount_ - 1].pose; s.v = s.omega = 0; }
    ref.state = encode(s);
    const double acceleration = ref.terminal ? 0 : (b.v - a.v) / interval * scale * scale;
    const double angularAcceleration = ref.terminal ? 0 : (b.omega - a.omega) / interval * scale * scale;
    ref.input[0] = bound(wheelLoad_[0] + config_.kVLeft * ref.state[3] + config_.kSLeft * smoothFriction(ref.state[3]) +
                        config_.kALeft * (acceleration + angularAcceleration * config_.trackWidth / 2), -voltage, voltage);
    ref.input[1] = bound(wheelLoad_[1] + config_.kVRight * ref.state[4] + config_.kSRight * smoothFriction(ref.state[4]) +
                        config_.kARight * (acceleration - angularAcceleration * config_.trackWidth / 2), -voltage, voltage);
    return ref;
}

DriveState Controller::reference() const {
    if (!active_) return {};
    return decode(sample(pathTime_, lastScale_, config_.maxVoltage).state);
}

double Controller::cost(const State& x, const Input& u, const Reference& ref, bool terminal,
                        double scale, Derivatives* d) const {
    if (d) *d = {};
    const double weights[nx] = {terminal ? 220.0 : 100.0 * horizonDt_,
                               terminal ? 220.0 : 100.0 * horizonDt_,
                               ref.terminal && !target_.constrainHeading && !target_.turnOnly ? 0.0 :
                               (terminal ? 18.0 : 9.0 * horizonDt_),
                               terminal ? 2.5 : 1.4 * horizonDt_,
                               terminal ? 2.5 : 1.4 * horizonDt_};
    double value = 0;
    for (std::size_t i = 0; i < nx; ++i) {
        const double error = i == 2 ? wrap(x[i] - ref.state[i]) : x[i] - ref.state[i];
        value += 0.5 * weights[i] * error * error;
        if (d) { d->x[i] = weights[i] * error; d->xx[i][i] = weights[i]; }
    }
    if (!terminal) for (std::size_t i = 0; i < nu; ++i) {
        const double error = u[i] - ref.input[i];
        const double weight = 0.012 * horizonDt_;
        value += 0.5 * weight * error * error;
        if (d) { d->u[i] = weight * error; d->uu[i] = weight; }
    }
    // Soft state inequalities use Gauss-Newton Hessians. Actuator voltage and
    // acceleration limits are hard constraints in rollout/dynamics.
    auto penalty = [&](double measurement, double limit, double weight, const State& gradient) {
        const double excess = std::abs(measurement) - limit;
        if (excess <= 0) return;
        const double sign = measurement >= 0 ? 1 : -1;
        value += 0.5 * weight * excess * excess;
        if (d) for (std::size_t i = 0; i < nx; ++i) {
            d->x[i] += weight * excess * sign * gradient[i];
            for (std::size_t j = 0; j < nx; ++j) d->xx[i][j] += weight * gradient[i] * gradient[j];
        }
    };
    const double v = (x[3] + x[4]) / 2, omega = (x[3] - x[4]) / config_.trackWidth;
    penalty(x[3], config_.maxSpeed * scale, 8.0, {{0, 0, 0, 1, 0}});
    penalty(x[4], config_.maxSpeed * scale, 8.0, {{0, 0, 0, 0, 1}});
    penalty(omega, config_.maxOmega * scale, 2.0, {{0, 0, 0, 1 / config_.trackWidth, -1 / config_.trackWidth}});
    penalty(v * omega, config_.maxLateralAcceleration * scale * scale, 12.0,
            {{0, 0, 0, omega / 2 + v / config_.trackWidth, omega / 2 - v / config_.trackWidth}});
    return value;
}

double Controller::rollout(const State& initial, const std::array<Input, horizon>& controls,
                           std::array<State, horizon + 1>& states, double scale) const {
    states[0] = initial;
    double result = 0;
    for (std::size_t k = 0; k < horizon; ++k) {
        result += cost(states[k], controls[k], refs_[k], false, scale);
        states[k + 1] = integrate(states[k], controls[k], horizonDt_, nullptr, nullptr,
                                   lateralForecast_[k]);
    }
    return result + cost(states[horizon], {{0, 0}}, refs_[horizon], true, scale);
}

bool Controller::backward(double regularization, double voltage) {
    State vx = derivatives_[horizon].x;
    Matrix vxx = derivatives_[horizon].xx;
    for (std::size_t offset = 0; offset < horizon; ++offset) {
        const std::size_t k = horizon - 1 - offset;
        State qx = derivatives_[k].x;
        Input qu = derivatives_[k].u;
        Matrix qxx = derivatives_[k].xx;
        std::array<std::array<double, nu>, nu> quu{};
        Gain qux{};
        Matrix va{};
        InputJacobian vb{};
        for (std::size_t i = 0; i < nx; ++i) {
            for (std::size_t j = 0; j < nx; ++j)
                for (std::size_t l = 0; l < nx; ++l) va[i][j] += vxx[i][l] * a_[k][l][j];
            for (std::size_t j = 0; j < nu; ++j)
                for (std::size_t l = 0; l < nx; ++l) vb[i][j] += vxx[i][l] * b_[k][l][j];
        }
        for (std::size_t i = 0; i < nx; ++i) {
            for (std::size_t l = 0; l < nx; ++l) qx[i] += a_[k][l][i] * vx[l];
            for (std::size_t j = 0; j < nx; ++j)
                for (std::size_t l = 0; l < nx; ++l) qxx[i][j] += a_[k][l][i] * va[l][j];
        }
        for (std::size_t i = 0; i < nu; ++i) {
            for (std::size_t l = 0; l < nx; ++l) qu[i] += b_[k][l][i] * vx[l];
            for (std::size_t j = 0; j < nu; ++j) {
                quu[i][j] = i == j ? derivatives_[k].uu[i] + regularization : 0;
                for (std::size_t l = 0; l < nx; ++l) quu[i][j] += b_[k][l][i] * vb[l][j];
            }
            for (std::size_t j = 0; j < nx; ++j)
                for (std::size_t l = 0; l < nx; ++l) qux[i][j] += b_[k][l][i] * va[l][j];
        }
        const double determinant = quu[0][0] * quu[1][1] - quu[0][1] * quu[1][0];
        if (!(determinant > 1e-14) || !std::isfinite(determinant)) return false;
        const double inverse[2][2] = {{quu[1][1] / determinant, -quu[0][1] / determinant},
                                      {-quu[1][0] / determinant, quu[0][0] / determinant}};
        const Input lower{{-voltage - controls_[k][0], -voltage - controls_[k][1]}};
        const Input upper{{voltage - controls_[k][0], voltage - controls_[k][1]}};
        Input bestStep{{0, 0}};
        double bestQuadratic = std::numeric_limits<double>::infinity();
        const auto consider = [&](const Input& step) {
            if (step[0] < lower[0] || step[0] > upper[0] || step[1] < lower[1] || step[1] > upper[1]) return;
            const double value = qu[0] * step[0] + qu[1] * step[1] +
                0.5 * (quu[0][0] * square(step[0]) + 2 * quu[0][1] * step[0] * step[1] + quu[1][1] * square(step[1]));
            if (value < bestQuadratic) { bestQuadratic = value; bestStep = step; }
        };
        consider({{-(inverse[0][0] * qu[0] + inverse[0][1] * qu[1]),
                    -(inverse[1][0] * qu[0] + inverse[1][1] * qu[1])}});
        // Exact two-input box QP: unconstrained minimizer or one of four edges.
        // Clipping the free coordinate also enumerates all four corners.
        for (std::size_t fixed = 0; fixed < nu; ++fixed) for (unsigned edge = 0; edge < 2; ++edge) {
            const std::size_t free = 1 - fixed;
            Input candidate{};
            candidate[fixed] = edge == 0 ? lower[fixed] : upper[fixed];
            candidate[free] = bound(-(qu[free] + quu[free][fixed] * candidate[fixed]) / quu[free][free], lower[free], upper[free]);
            consider(candidate);
        }
        feedforward_[k] = bestStep;
        const bool active[2] = {std::abs(bestStep[0] - lower[0]) < 1e-10 || std::abs(bestStep[0] - upper[0]) < 1e-10,
                                std::abs(bestStep[1] - lower[1]) < 1e-10 || std::abs(bestStep[1] - upper[1]) < 1e-10};
        for (std::size_t i = 0; i < nu; ++i) for (std::size_t j = 0; j < nx; ++j) {
            feedback_[k][i][j] = active[i] ? 0 : (active[1 - i] ? -qux[i][j] / quu[i][i] :
                            -(inverse[i][0] * qux[0][j] + inverse[i][1] * qux[1][j]));
        }
        vx = qx;
        vxx = qxx;
        for (std::size_t i = 0; i < nx; ++i) {
            for (std::size_t u = 0; u < nu; ++u) {
                vx[i] += feedback_[k][u][i] * qu[u] + qux[u][i] * feedforward_[k][u];
                for (std::size_t v = 0; v < nu; ++v)
                    vx[i] += feedback_[k][u][i] * quu[u][v] * feedforward_[k][v];
            }
            for (std::size_t j = 0; j < nx; ++j) {
                for (std::size_t u = 0; u < nu; ++u) {
                    vxx[i][j] += feedback_[k][u][i] * qux[u][j] + qux[u][i] * feedback_[k][u][j];
                    for (std::size_t v = 0; v < nu; ++v)
                        vxx[i][j] += feedback_[k][u][i] * quu[u][v] * feedback_[k][v][j];
                }
            }
        }
        for (std::size_t i = 0; i < nx; ++i) for (std::size_t j = 0; j < i; ++j)
            vxx[i][j] = vxx[j][i] = 0.5 * (vxx[i][j] + vxx[j][i]);
        if (offset % 5 == 4 && exhausted()) return false;
    }
    return true;
}

bool Controller::exhausted() {
    if (!clock_) return false;
    stats_.computeMs = (clock_() - computeStart_) * 1000;
    if (stats_.computeMs >= budgetMs_) stats_.budgetExceeded = true;
    return stats_.budgetExceeded;
}

Voltage Controller::update(const Estimate& estimate, double dt, double batteryVolts, double observationAge) {
    stats_ = {};
    computeStart_ = clock_ ? clock_() : 0;
    const bool validTime = std::isfinite(dt) && dt > 0;
    if (validTime) commandTime_ += dt;
    else {
        // An unknown elapsed interval invalidates the transport chronology.
        commandTime_ = 0;
        commandFirst_ = commandCount_ = 0;
        commandBeforeHistory_ = {};
    }
    if (!active_ || !usable(estimate) || !std::isfinite(dt) || dt <= 0 ||
        !std::isfinite(batteryVolts) || batteryVolts < 1 || !std::isfinite(observationAge) ||
        observationAge < 0 || observationAge > .1) {
        previous_ = {};
        warm_ = false;
        haveObservation_ = false;
        wheelLoad_ = {};
        lateralVelocity_ = 0;
        if (active_ && validTime) rememberCommand({});
        return {};
    }
    dt = bound(dt, 0.001, 0.1);
    runTime_ += dt;
    const double voltage = bound(batteryVolts, 0.0, config_.maxVoltage);
    const double emittedNow = commandTime_ - config_.commandLatency;
    const State observed = encode(estimate.state);
    const double observationDt = emittedNow - observationAge - previousObservationTime_;
    if (haveObservation_ && observationDt >= .005 && observationDt <= .1) {
        const double measured = estimate.state.lateralV;
        const double middle = .5 * (lateralVelocity_ + measured);
        const double previousV = .5 * (previousObservation_[3] + previousObservation_[4]);
        const double previousOmega = (previousObservation_[3] - previousObservation_[4]) / config_.trackWidth;
        const double derivative = (measured - lateralVelocity_) / observationDt +
            .5 * (previousV * previousOmega + estimate.state.v * estimate.state.omega);
        const double gain = 1 - std::exp(-observationDt / .35);
        lateralEnergy_ += gain * (middle * middle - lateralEnergy_);
        lateralDamping_ += gain * (-middle * derivative - lateralDamping_);
        if (lateralEnergy_ > square(.01)) {
            const double measuredDecay = bound(lateralDamping_ / lateralEnergy_, 0.0, 50.0);
            lateralDecay_ += observationDt / (.2 + observationDt) * (measuredDecay - lateralDecay_);
        }
    }
    lateralVelocity_ = estimate.state.lateralV;
    for (std::size_t k = 0; k < horizon; ++k)
        lateralForecast_[k] = lateralVelocity_ * std::exp(-(k + .5) * horizonDt_ * lateralDecay_);
    observeWheelLoad(observed, emittedNow - observationAge, voltage, estimate.slip);
    const State atNow = observationAge > 0 ?
        predictCommandHistory(observed, emittedNow - observationAge, emittedNow, voltage) : observed;
    const DriveState currentState = decode(atNow);
    const double positionVariance = std::max(0.0, estimate.covariance[0][0] + estimate.covariance[1][1]);
    const double headingVariance = std::max(0.0, estimate.covariance[2][2]);
    const double uncertaintyScale = 1 / (1 + 3 * std::sqrt(positionVariance) + 0.6 * std::sqrt(headingVariance));
    const double voltageScale = bound((voltage - std::max(config_.kSLeft, config_.kSRight)) /
                                     std::max(0.5, config_.maxVoltage - std::max(config_.kSLeft, config_.kSRight)), 0.15, 1.0);
    const double scale = bound(uncertaintyScale * (1 - 0.55 * bound(estimate.slip, 0.0, 1.0)) *
                               (estimate.health == Health::degraded ? 0.7 : 1.0), 0.25, 1.0) * voltageScale;
    lastScale_ = scale;
    // Re-plan after large disturbances and terminal lateral errors. A fresh
    // nonlinear trajectory supplies escape directions unavailable to a local
    // quadratic optimizer linearized at a stationary differential drive.
    const double goalError = std::hypot(currentState.pose.x - target_.pose.x, currentState.pose.y - target_.pose.y);
    // A point move does not request its spline's arrival heading. Once inside
    // the requested position tolerance, brake toward the terminal point instead
    // of finishing the timed curve/heading (which can drive back out of it).
    if (!target_.constrainHeading && !target_.turnOnly && goalError <= options_.positionTolerance)
        pathTime_ = trajectoryDuration();
    const auto current = sample(pathTime_, scale, voltage);
    const double trackingError = std::hypot(currentState.pose.x - current.state[0], currentState.pose.y - current.state[1]);
    const bool terminalCorrection = current.terminal && goalError > options_.positionTolerance &&
                                    std::hypot(currentState.v, estimate.state.lateralV) < 0.15;
    if (!target_.turnOnly && runTime_ - lastReplan_ > 0.6 &&
        (trackingError > 0.25 || terminalCorrection)) {
        makePath(currentState, terminalCorrection);
        lastReplan_ = runTime_;
        warm_ = false;
    }
    for (std::size_t k = 0; k <= horizon; ++k)
        refs_[k] = sample(pathTime_ + (config_.commandLatency + k * horizonDt_) * scale, scale, voltage);
    const double shift = bound(dt / horizonDt_, 0.0, 1.0);
    for (std::size_t k = 0; k < horizon; ++k) for (std::size_t j = 0; j < nu; ++j) {
        const double next = k + 1 < horizon ? controls_[k + 1][j] : refs_[k].input[j];
        controls_[k][j] = bound(warm_ ? controls_[k][j] * (1 - shift) + next * shift : refs_[k].input[j], -voltage, voltage);
    }
    const State initial = predictCommandHistory(atNow, emittedNow, commandTime_, voltage);
    double best = rollout(initial, controls_, states_, scale);
    stats_.initialCost = best;
    if (!std::isfinite(best)) { warm_ = false; previous_ = {}; rememberCommand({}); return {}; }
    double regularization = 0.0001;
    for (unsigned iteration = 0; iteration < 5; ++iteration) {
        if (exhausted()) break;
        bool interrupted = false;
        for (std::size_t k = 0; k < horizon; ++k) {
            integrate(states_[k], controls_[k], horizonDt_, &a_[k], &b_[k],
                      lateralForecast_[k]);
            cost(states_[k], controls_[k], refs_[k], false, scale, &derivatives_[k]);
            if (k % 5 == 4 && exhausted()) { interrupted = true; break; }
        }
        if (interrupted) break;
        cost(states_[horizon], {{0, 0}}, refs_[horizon], true, scale, &derivatives_[horizon]);
        ++stats_.iterations;
        if (!backward(regularization, voltage)) { regularization *= 10; continue; }
        bool accepted = false;
        double improvement = 0;
        for (double alpha : {1.0, 0.5, 0.25, 0.1, 0.025}) {
            if (exhausted()) break;
            trialStates_[0] = initial;
            double candidate = 0;
            for (std::size_t k = 0; k < horizon; ++k) {
                for (std::size_t j = 0; j < nu; ++j) {
                    double u = controls_[k][j] + alpha * feedforward_[k][j];
                    for (std::size_t i = 0; i < nx; ++i) {
                        const double difference = i == 2 ? wrap(trialStates_[k][i] - states_[k][i]) : trialStates_[k][i] - states_[k][i];
                        u += feedback_[k][j][i] * difference;
                    }
                    trialControls_[k][j] = bound(u, -voltage, voltage);
                }
                candidate += cost(trialStates_[k], trialControls_[k], refs_[k], false, scale);
                trialStates_[k + 1] = integrate(trialStates_[k], trialControls_[k], horizonDt_,
                                              nullptr, nullptr,
                                              lateralForecast_[k]);
                // A trial is speculative until its entire horizon is evaluated.
                // Keep the last feasible rollout if the budget expires midway.
                if (k % 5 == 4 && exhausted()) { interrupted = true; break; }
            }
            if (interrupted) break;
            candidate += cost(trialStates_[horizon], {{0, 0}}, refs_[horizon], true, scale);
            if (std::isfinite(candidate) && candidate < best) {
                improvement = best - candidate;
                best = candidate;
                controls_ = trialControls_;
                states_ = trialStates_;
                accepted = true;
                break;
            }
        }
        if (interrupted) break;
        if (accepted) {
            regularization = std::max(1e-7, regularization * 0.4);
            if (improvement < 1e-5 * (1 + best)) { stats_.converged = true; break; }
        } else {
            regularization *= 10;
            double largestStep = 0;
            for (const auto& step : feedforward_) for (double value : step)
                largestStep = std::max<double>(largestStep, std::abs(value));
            if (largestStep < 0.001) { stats_.converged = true; break; }
            if (regularization > 0.05) break;
        }
    }
    stats_.finalCost = best;
    stats_.valid = true;
    exhausted();
    previous_ = {controls_[0][0], controls_[0][1]};
    rememberCommand(previous_);
    warm_ = true;
    pathTime_ = std::min(trajectoryDuration() + horizonDt_, pathTime_ + dt * scale);
    return previous_;
}

bool Controller::settled(const Estimate& estimate, double dt) {
    if (!active_ || !usable(estimate) || !std::isfinite(dt) || dt <= 0) {
        settledTime_ = 0;
        return false;
    }
    const bool position = target_.turnOnly || std::hypot(estimate.state.pose.x - target_.pose.x,
                                      estimate.state.pose.y - target_.pose.y) <= options_.positionTolerance;
    const bool heading = (!target_.constrainHeading && !target_.turnOnly) ||
                         std::abs(wrap(estimate.state.pose.theta - target_.pose.theta)) <= options_.headingTolerance;
    const bool stopped = std::hypot(estimate.state.v, estimate.state.lateralV) < 0.035 &&
                         std::abs(estimate.state.omega) < radians(4);
    if (position && heading && stopped) settledTime_ += std::min(0.1, dt);
    else settledTime_ = 0;
    return position && heading && stopped && settledTime_ >= options_.settleTime;
}
} // namespace nexus
