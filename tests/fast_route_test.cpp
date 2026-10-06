#include "nexus/controller.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>

using namespace nexus;

namespace {
void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAILED: " << message << '\n';
        std::exit(1);
    }
}

DynamicsConfig robotDynamics() {
    DynamicsConfig dynamics;
    dynamics.trackWidth = .287;
    dynamics.maxLateralAcceleration = 1.;
    dynamics.commandLatency = .01;
    return dynamics;
}

// Independent RK4 integration at 1 ms. The plant does not call predict(),
// and transport delay is simulated by applying the previous voltage first.
DriveState plant(DriveState state, Voltage voltage, double dt, const DynamicsConfig& dynamics) {
    std::array<double, 5> x{{state.pose.x, state.pose.y, state.pose.theta,
        state.v + state.omega * dynamics.trackWidth / 2,
        state.v - state.omega * dynamics.trackWidth / 2}};
    const auto derivative = [&](const std::array<double, 5>& s) {
        const double velocity = (s[3] + s[4]) / 2;
        const auto acceleration = [&](double volts, double speed, double kv, double ka, double ks) {
            const double raw = (volts - kv * speed - ks * std::tanh(speed / .07)) / ka;
            return std::clamp(raw, -dynamics.maxAcceleration, dynamics.maxAcceleration);
        };
        return std::array<double, 5>{{velocity * std::sin(s[2]), velocity * std::cos(s[2]),
            (s[3] - s[4]) / dynamics.trackWidth,
            acceleration(voltage.left, s[3], dynamics.kVLeft, dynamics.kALeft, dynamics.kSLeft),
            acceleration(voltage.right, s[4], dynamics.kVRight, dynamics.kARight, dynamics.kSRight)}};
    };
    const unsigned steps = static_cast<unsigned>(std::ceil(dt / .001));
    const double h = dt / steps;
    for (unsigned step = 0; step < steps; ++step) {
        const auto k1 = derivative(x);
        auto stage = x;
        for (std::size_t i = 0; i < x.size(); ++i) stage[i] = x[i] + h * k1[i] / 2;
        const auto k2 = derivative(stage);
        for (std::size_t i = 0; i < x.size(); ++i) stage[i] = x[i] + h * k2[i] / 2;
        const auto k3 = derivative(stage);
        for (std::size_t i = 0; i < x.size(); ++i) stage[i] = x[i] + h * k3[i];
        const auto k4 = derivative(stage);
        for (std::size_t i = 0; i < x.size(); ++i)
            x[i] += h * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) / 6;
    }
    return {{x[0], x[1], wrap(x[2])}, (x[3] + x[4]) / 2,
        (x[3] - x[4]) / dynamics.trackWidth};
}

struct Result {
    DriveState state{};
    double seconds = 0, travel = 0, rotation = 0, backwards = 0;
    std::array<double, 7> nearest{}, crossingSpeed{};
};

Result simulate(Controller& controller, DriveState initial, const Target& target,
                const MotionOptions& options, DynamicsConfig actual, const char* name) {
    Result result;
    result.state = initial;
    result.nearest.fill(std::numeric_limits<double>::infinity());
    Estimate estimate;
    estimate.health = Health::healthy;
    Voltage previous{};
    bool settled = false;
    while (result.seconds < 12) {
        estimate.state = result.state;
        const auto command = controller.update(estimate, .02);
        require(controller.stats().valid, "route solve remains valid");
        require(std::abs(command.left) <= 12 && std::abs(command.right) <= 12,
                "route voltages remain within battery limits");
        const auto before = result.state;
        result.state = plant(result.state, previous, .01, actual);
        result.state = plant(result.state, command, .01, actual);
        previous = command;
        result.seconds += .02;
        result.travel += std::hypot(result.state.pose.x - before.pose.x, result.state.pose.y - before.pose.y);
        result.rotation += std::abs(wrap(result.state.pose.theta - before.pose.theta));
        const double requestedVelocity = options.reverse ? -result.state.v : result.state.v;
        result.backwards += std::max(0., -requestedVelocity) * .02;
        for (std::size_t i = 0; i < target.viaCount; ++i) {
            const double distance = std::hypot(result.state.pose.x - target.via[i].x,
                                               result.state.pose.y - target.via[i].y);
            if (distance < result.nearest[i]) {
                result.nearest[i] = distance;
                result.crossingSpeed[i] = requestedVelocity;
            }
        }
        estimate.state = result.state;
        if (controller.settled(estimate, .02)) { settled = true; break; }
    }
    const double error = std::hypot(result.state.pose.x - target.pose.x, result.state.pose.y - target.pose.y);
    const double headingError = std::abs(wrap(result.state.pose.theta - target.pose.theta));
    std::cout << name << " seconds=" << result.seconds << " final_mm=" << error * 1000
              << " heading_deg=" << degrees(headingError) << " travel_mm=" << result.travel * 1000
              << " reverse_mm=" << result.backwards * 1000;
    for (std::size_t i = 0; i < target.viaCount; ++i)
        std::cout << " via" << i << "_mm=" << result.nearest[i] * 1000
                  << " via" << i << "_mmps=" << result.crossingSpeed[i] * 1000;
    std::cout << '\n';
    require(settled && error <= options.positionTolerance, "route stops within requested final position");
    require(!target.constrainHeading || headingError <= options.headingTolerance,
            "route stops within requested final heading");
    require(std::abs(result.state.v) < .035 && std::abs(result.state.omega) < radians(4),
            "reaching a route endpoint requires a physical stop");
    return result;
}

Result run(DriveState initial, const Target& target, const MotionOptions& options,
           const char* name, bool mismatch = false) {
    const auto dynamics = robotDynamics();
    Controller controller(dynamics);
    // No wall clock deadline: algorithm regressions are independent of host load.
    controller.start(initial, target, options);
    auto actual = dynamics;
    if (mismatch) {
        actual.kVLeft *= 1.05; actual.kVRight *= .96;
        actual.kALeft *= 1.1; actual.kARight *= .92;
    }
    return simulate(controller, initial, target, options, actual, name);
}

void checkGuides(const Result& result, const Target& target) {
    for (std::size_t i = 0; i < target.viaCount; ++i) {
        require(result.nearest[i] < .04, "continuous route reaches every intermediate guide");
        require(result.crossingSpeed[i] > .15, "intermediate guides are traversed without stopping");
    }
    require(result.backwards < .03, "continuous route avoids unnecessary direction changes");
}

void compareMainRoute(bool reverse) {
    MotionOptions options;
    options.maxSpeed = 1.65;
    options.reverse = reverse;
    options.positionTolerance = .015;
    const double sign = reverse ? -1. : 1.;
    Target target{{sign * -.53, sign * .38, radians(-90)}, true, false};
    target.via[0] = {sign * -.3, sign * .38, 0};
    target.viaCount = 1;
    const Result continuous = run({}, target, options, reverse ? "reverse route" : "main route");
    checkGuides(continuous, target);

    Target first{{target.via[0].x, target.via[0].y, radians(-90)}, true, false};
    Target final{target.pose, true, false};
    const Result stoppedFirst = run({}, first, options, reverse ? "reverse stop 1" : "main stop 1");
    const Result stoppedFinal = run(stoppedFirst.state, final, options, reverse ? "reverse stop 2" : "main stop 2");
    const double stoppedSeconds = stoppedFirst.seconds + stoppedFinal.seconds;
    std::cout << "route speedup reverse=" << reverse << " continuous_s=" << continuous.seconds
              << " stopped_s=" << stoppedSeconds << '\n';
    require(continuous.seconds + .15 < stoppedSeconds,
            "continuous main route saves a meaningful stop and restart interval");
}

Target quarterTurn() {
    Target target{{.65, .65, radians(90)}, true, false};
    target.via[0] = {.65 * (1 - std::cos(pi / 6)), .65 * std::sin(pi / 6), 0};
    target.via[1] = {.65 * (1 - std::cos(pi / 3)), .65 * std::sin(pi / 3), 0};
    target.viaCount = 2;
    return target;
}

void checkReferenceAcceleration(const Target& target) {
    auto dynamics = robotDynamics();
    dynamics.commandLatency = 0;
    Controller controller(dynamics);
    MotionOptions options;
    options.maxSpeed = 1.65;
    options.positionTolerance = .001;
    options.headingTolerance = radians(.1);
    controller.start({}, target, options);
    Estimate estimate;
    estimate.health = Health::healthy;
    DriveState previous = controller.reference();
    double peak = 0, maximumSpeed = 0;
    const double duration = controller.trajectoryDuration();
    for (double time = 0; time <= duration + .001; time += .001) {
        estimate.state = previous;
        controller.update(estimate, .001);
        const auto next = controller.reference();
        for (double side : {-1., 1.}) {
            peak = std::max(peak, std::abs(next.v - previous.v + side * dynamics.trackWidth / 2 *
                                  (next.omega - previous.omega)) / .001);
            maximumSpeed = std::max(maximumSpeed, std::abs(next.v + side * dynamics.trackWidth / 2 * next.omega));
        }
        previous = next;
    }
    std::cout << "route reference duration_s=" << duration << " peak_wheel_acceleration=" << peak
              << " peak_wheel_speed=" << maximumSpeed << '\n';
    require(peak <= dynamics.maxAcceleration + 1e-6, "continuous joins respect both wheel acceleration limits");
    require(maximumSpeed <= dynamics.maxSpeed + 1e-6, "continuous joins respect both wheel speed limits");
}

void checkAlreadyArrivedPose() {
    const auto dynamics = robotDynamics();
    Controller controller(dynamics);
    MotionOptions options;
    options.positionTolerance = .03;
    options.settleTime = 0;
    const Target target{{.6, .6, 0}, true, false};
    controller.start({}, target, options);
    Estimate arrival;
    arrival.health = Health::healthy;
    arrival.state = {{.61, .595, 0}, .12, 0};
    require(!controller.settled(arrival, .02), "being inside pose tolerances while moving is not completion");
    const Result result = simulate(controller, arrival.state, target, options, dynamics, "early pose arrival");
    require(result.seconds < 1.25 && result.travel < .09 && result.rotation < radians(20),
            "an accepted pose brakes without restarting the unfinished approach curve");
}

void checkRepeatedCoordinates() {
    MotionOptions options;
    Target target{{0, .9, 0}, true, false};
    target.via[0] = {0, 0, 0};
    target.via[1] = {0, .3, 0};
    target.via[2] = {0, .3, 0};
    target.via[3] = {0, .6, 0};
    target.via[4] = {0, .9, 0};
    target.viaCount = 5;
    const auto result = run({}, target, options, "repeated route coordinates");
    for (std::size_t guide : {1u, 2u, 3u})
        require(result.nearest[guide] < .025 && result.crossingSpeed[guide] > .15,
                "repeated guides retain continuous passage through distinct coordinates");

    Target stationary{{0, 0, 0}, true, false};
    stationary.via[0] = stationary.via[1] = stationary.pose;
    stationary.viaCount = 2;
    const auto same = run({}, stationary, options, "stationary repeated route");
    require(same.travel < .001 && same.rotation < radians(.1),
            "a route consisting solely of one coordinate remains stationary");
}

void checkClosedRoute() {
    MotionOptions options;
    Target target{{0, 0, 0}, true, false};
    target.via[0] = {.3, .3, 0};
    target.via[1] = {.6, 0, 0};
    target.via[2] = {.3, -.3, 0};
    target.viaCount = 3;
    Controller controller(robotDynamics());
    controller.start({}, target, options);
    Estimate atStart;
    atStart.health = Health::healthy;
    require(!controller.settled(atStart, .1), "a closed route cannot complete before visiting its guides");
    const auto result = simulate(controller, {}, target, options, robotDynamics(), "closed route");
    checkGuides(result, target);
    require(result.travel > 1.2, "a closed route physically visits its intermediate coordinates");
}

void checkReplanExactlyAtGuide() {
    MotionOptions options;
    Target target{{0, 1.2, 0}, true, false};
    target.via[0] = {0, .6, 0};
    target.viaCount = 1;
    Controller controller(robotDynamics());
    controller.start({}, target, options);
    Estimate ahead;
    ahead.health = Health::healthy;
    ahead.state.pose = target.via[0];
    // The estimate is already at the still-unconsumed guide while the timed
    // reference starts behind it. Retain that observation through cooldown.
    for (unsigned i = 0; i < 8; ++i) {
        const auto command = controller.update(ahead, .1);
        const auto reference = controller.reference();
        require(controller.stats().valid && std::isfinite(command.left) && std::isfinite(command.right) &&
                std::isfinite(reference.pose.x) && std::isfinite(reference.pose.y) &&
                std::isfinite(reference.pose.theta) && std::isfinite(controller.trajectoryDuration()),
                "replanning from a coincident guide avoids a degenerate zero-length spline");
        require(!controller.settled(ahead, .1), "replanning at a guide still requires reaching the endpoint");
    }
    const auto result = simulate(controller, ahead.state, target, options, robotDynamics(), "replan at guide");
    require(result.nearest[0] < .025 && result.travel > .5,
            "a coincident guide is consumed and the remaining leg is executed");
}

void checkMissedGuideRecovery() {
    MotionOptions options;
    // A permissive final tolerance deliberately contains this lateral miss.
    // It must not bypass the narrower guide-passage check or leave an inactive
    // terminal reference forever while settled() waits for an unvisited guide.
    options.positionTolerance = .25;
    Target target{{0, .8, 0}, true, false};
    target.via[0] = {0, .4, 0};
    target.viaCount = 1;
    Controller controller(robotDynamics());
    controller.start({}, target, options);
    Estimate missed;
    missed.health = Health::healthy;
    bool replanned = false;
    for (unsigned i = 0; i < 400; ++i) {
        missed.state = controller.reference();
        missed.state.pose.x += .2;
        const auto command = controller.update(missed, .02);
        require(controller.stats().valid && std::isfinite(command.left) && std::isfinite(command.right),
                "a missed guide retains finite recovery commands");
        require(!controller.settled(missed, .02), "final tolerance cannot conceal an unvisited route guide");
        if (controller.reference().pose.x > .1) { replanned = true; break; }
    }
    require(replanned, "a terminal reference with a missed guide restarts recovery instead of waiting forever");
    const auto result = simulate(controller, missed.state, target, options, robotDynamics(), "missed guide recovery");
    require(result.nearest[0] < .08, "missed-guide recovery physically returns through the guide");
}
} // namespace

int main() {
    std::cout << std::fixed << std::setprecision(3);
    compareMainRoute(false);
    compareMainRoute(true);
    MotionOptions options;
    options.maxSpeed = 1.65;
    const auto target = quarterTurn();
    const auto curve = run({}, target, options, "quarter-turn route", true);
    checkGuides(curve, target);
    require(curve.rotation < radians(120), "smooth quarter turn avoids countersteering loops");
    checkReferenceAcceleration(target);
    checkAlreadyArrivedPose();
    checkRepeatedCoordinates();
    checkClosedRoute();
    checkReplanExactlyAtGuide();
    checkMissedGuideRecovery();
    std::cout << "Fast route simulation regressions passed; timings describe the simulated plant.\n";
}
