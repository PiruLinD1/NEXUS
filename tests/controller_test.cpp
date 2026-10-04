#include "nexus/controller.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>

using namespace nexus;
namespace {
void require(bool value, const char* message) {
    if (!value) { std::cerr << "FAILED: " << message << '\n'; std::exit(1); }
}
double clockSeconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
// Independent RK4 plant (1ms steps), not Controller::predict(). This checks the
// optimizer against a separate discretization and optional motor mismatch.
DriveState plant(DriveState state, Voltage u, double dt, DynamicsConfig c) {
    std::array<double, 5> x{{state.pose.x, state.pose.y, state.pose.theta,
                            state.v + state.omega * c.trackWidth / 2,
                            state.v - state.omega * c.trackWidth / 2}};
    const unsigned steps = static_cast<unsigned>(std::ceil(dt / 0.001));
    const double h = dt / steps;
    const auto f = [&c, u](const std::array<double, 5>& s) {
        const double v = (s[3] + s[4]) / 2;
        const auto a = [&c](double voltage, double speed, double kv, double ka, double ks) {
            return std::max(-c.maxAcceleration, std::min(c.maxAcceleration,
                     (voltage - kv * speed - ks * std::tanh(speed / 0.07)) / ka));
        };
        return std::array<double, 5>{{v * std::sin(s[2]), v * std::cos(s[2]),
                     (s[3] - s[4]) / c.trackWidth,
                     a(u.left, s[3], c.kVLeft, c.kALeft, c.kSLeft),
                     a(u.right, s[4], c.kVRight, c.kARight, c.kSRight)}};
    };
    for (unsigned step = 0; step < steps; ++step) {
        const auto k1 = f(x);
        auto stage = x;
        for (std::size_t i = 0; i < 5; ++i) stage[i] = x[i] + h * k1[i] / 2;
        const auto k2 = f(stage);
        for (std::size_t i = 0; i < 5; ++i) stage[i] = x[i] + h * k2[i] / 2;
        const auto k3 = f(stage);
        for (std::size_t i = 0; i < 5; ++i) stage[i] = x[i] + h * k3[i];
        const auto k4 = f(stage);
        for (std::size_t i = 0; i < 5; ++i) x[i] += h / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
    }
    return {{x[0], x[1], wrap(x[2])}, (x[3] + x[4]) / 2, (x[3] - x[4]) / c.trackWidth};
}
struct Result { double position, heading, seconds, rmsTracking, worstMs, meanMs; bool settled; };
Result run(const char* name, DriveState initial, Target target, MotionOptions options,
           double battery = 12, int disturbance = 0, bool mismatch = false, bool delayed = false) {
    DynamicsConfig model;
    model.commandLatency = delayed ? 0.01 : 0;
    Controller controller(model);
    controller.setClock(clockSeconds);
    controller.setBudgetMs(500); // Reproducible algorithm test, not a V5 timing claim.
    controller.start(initial, target, options);
    DynamicsConfig actual = model;
    if (mismatch) { actual.kVLeft *= 1.05; actual.kVRight *= 0.96; actual.kALeft *= 1.15; actual.kSRight *= 1.1; }
    Estimate estimate;
    estimate.health = Health::healthy;
    estimate.confidence = 1;
    estimate.state = initial;
    double elapsed = 0, squaredTracking = 0, worst = 0, total = 0;
    unsigned calls = 0, improvements = 0;
    bool done = false;
    Voltage previous{};
    for (unsigned iteration = 0; iteration < 700; ++iteration) {
        if (iteration == 55 && disturbance == 1) {
            estimate.state.pose.x += 0.14;
            estimate.state.pose.y -= 0.07;
            estimate.state.pose.theta += radians(12);
        }
        if (iteration >= 45 && iteration < 58 && disturbance == 2) {
            estimate.state.v *= 0.15;
            estimate.state.omega *= 0.15;
            estimate.slip = 0.75;
        } else estimate.slip = 0;
        const Voltage voltage = controller.update(estimate, 0.02, battery);
        const SolverStats stats = controller.stats();
        require(stats.valid, "finite valid solve");
        require(std::abs(voltage.left) <= battery + 1e-9 && std::abs(voltage.right) <= battery + 1e-9,
                "hard battery voltage limits");
        require(stats.finalCost <= stats.initialCost + 1e-9, "accepted steps never increase objective");
        improvements += stats.finalCost < stats.initialCost - 1e-6;
        worst = std::max(worst, stats.computeMs);
        total += stats.computeMs;
        ++calls;
        if (delayed) {
            estimate.state = plant(estimate.state, previous, 0.01, actual);
            estimate.state = plant(estimate.state, voltage, 0.01, actual);
        } else estimate.state = plant(estimate.state, voltage, 0.02, actual);
        previous = voltage;
        elapsed += 0.02;
        const auto ref = controller.reference();
        squaredTracking += square(estimate.state.pose.x - ref.pose.x) + square(estimate.state.pose.y - ref.pose.y);
        if (controller.settled(estimate, 0.02)) { done = true; break; }
    }
    const double position = std::hypot(estimate.state.pose.x - target.pose.x, estimate.state.pose.y - target.pose.y);
    const double heading = std::abs(wrap(estimate.state.pose.theta - target.pose.theta));
    Result result{position, heading, elapsed, std::sqrt(squaredTracking / calls), worst, total / calls, done};
    std::cout << std::left << std::setw(18) << name << " pos_mm=" << std::setw(9) << position * 1000
              << " heading_deg=" << std::setw(9) << degrees(heading) << " time_s=" << std::setw(6) << elapsed
              << " rms_mm=" << std::setw(9) << result.rmsTracking * 1000
              << " mean_ms=" << std::setw(9) << result.meanMs << " worst_ms=" << std::setw(9) << worst
              << " settled=" << done << " improves=" << improvements << '\n';
    require(improvements > 2, "nonlinear optimization actually improves rollouts");
    require(done, "motion settles within 14 simulated seconds");
    require(target.turnOnly || position <= options.positionTolerance + 1e-6, "requested final position tolerance");
    require((!target.constrainHeading && !target.turnOnly) || heading <= options.headingTolerance + 1e-6,
            "requested final heading tolerance");
    return result;
}
double fakeTime = 0;
double advancingClock() { fakeTime += 0.01; return fakeTime; }
double fineClock() { fakeTime += 0.00002; return fakeTime; }

void checkReferenceAcceleration(Target target, bool reverse) {
    DynamicsConfig dynamics;
    dynamics.commandLatency = 0;
    Controller controller(dynamics);
    controller.setClock(advancingClock);
    controller.setBudgetMs(.1); // Only the complete initial rollout is needed.
    MotionOptions options;
    options.reverse = reverse;
    controller.start({}, target, options);
    Estimate estimate;
    estimate.health = Health::healthy;
    DriveState previous = controller.reference();
    constexpr double dt = .001;
    double peak = 0, integratedHeading = previous.pose.theta, headingError = 0;
    for (double time = 0; time <= controller.trajectoryDuration() + dt; time += dt) {
        estimate.state = previous; // Follow the reference exactly; no replanning.
        controller.update(estimate, dt);
        const DriveState next = controller.reference();
        integratedHeading += (previous.omega + next.omega) * dt / 2;
        headingError = std::max(headingError, std::abs(wrap(next.pose.theta - integratedHeading)));
        for (double sign : {-1.0, 1.0}) {
            const double acceleration = (next.v - previous.v + sign * dynamics.trackWidth / 2 *
                                         (next.omega - previous.omega)) / dt;
            peak = std::max(peak, std::abs(acceleration));
        }
        previous = next;
    }
    require(peak <= dynamics.maxAcceleration + 1e-8,
            "tight curves respect both wheel accelerations in the timed reference");
    std::cout << "reference x=" << target.pose.x << " y=" << target.pose.y << " acceleration=" << peak
              << " heading_integration_error=" << headingError << '\n';
    // The 129-knot geometric quadrature is approximate, especially on a loop.
    require(headingError < .02, "reference timing agrees with its angular velocity even at submillimetre distances");
}

Result runQueuedLatency(double latency, unsigned scenario, bool jitter = true, bool mismatch = false,
                        bool precise = true, double observationAge = 0, bool compensateAge = true,
                        bool transportJitter = false) {
    DynamicsConfig model;
    model.commandLatency = latency;
    Controller controller(model);
    controller.setClock(clockSeconds);
    controller.setBudgetMs(500);
    DynamicsConfig actual = model;
    if (mismatch) { actual.kVLeft *= 1.05; actual.kVRight *= .96; actual.kALeft *= 1.15; actual.kSRight *= 1.1; }
    if (mismatch && scenario >= 3) {
        actual = model; actual.kVLeft *= .96; actual.kVRight *= 1.04; actual.kALeft *= .90; actual.kARight *= 1.12;
    }
    MotionOptions options;
    if (precise) { options.positionTolerance = .001; options.headingTolerance = radians(.1); }
    options.reverse = scenario % 3 == 2;
    const Target target = scenario < 3 ? (scenario == 1 ? Target{{0, .45, 0}, true, false} :
        (scenario == 2 ? Target{{-.8, -1.2, radians(65)}, true, false} : Target{{.7, 1.1, radians(60)}, true, false})) :
        (scenario == 4 ? Target{{.06, .65, radians(5)}, true, false} :
        (scenario == 5 ? Target{{.5, -.7, radians(-85)}, true, false} : Target{{.4, .6, radians(110)}, true, false}));
    Estimate estimate;
    estimate.health = Health::healthy;
    estimate.state.v = scenario == 1 ? 1.4 : (scenario == 4 ? 1.1 : 0);
    controller.start(estimate.state, target, options);
    DriveState truth = estimate.state;
    struct Observation { double time; DriveState state; };
    std::deque<Observation> observations{{0, truth}};
    struct Pending { double time; Voltage voltage; };
    std::deque<Pending> pending;
    Voltage applied{};
    double elapsed = 0, previousDt = .02, squaredTracking = 0, worst = 0, total = 0;
    constexpr double steps[] = {.013, .027, .016, .024, .021, .019};
    unsigned calls = 0;
    bool done = false;
    while (elapsed < 14) {
        const double dt = jitter ? steps[calls % 6] : .02;
        const double observationTime = std::max(0.0, elapsed - observationAge);
        std::size_t observed = 0;
        while (observed + 1 < observations.size() && observations[observed + 1].time <= observationTime + 1e-9) ++observed;
        estimate.state = observations[observed].state;
        const double age = std::max(0.0, elapsed - observations[observed].time);
        const Voltage command = controller.update(estimate, previousDt, 12, compensateAge ? age : 0);
        const SolverStats stats = controller.stats();
        require(stats.valid && stats.finalCost <= stats.initialCost + 1e-9,
                "queued delayed plant retains valid improving solves");
        require(std::abs(command.left) <= 12 && std::abs(command.right) <= 12, "queued delayed voltage limits");
        constexpr double delayJitter[] = {-.002, .001, .002, -.001, 0};
        pending.push_back({elapsed + latency + (transportJitter ? delayJitter[calls % 5] : 0), command});
        const double end = elapsed + dt;
        const auto advancePlant = [&](double until) {
            while (until - elapsed > 1e-9) {
                const double interval = std::min(.001, until - elapsed);
                truth = plant(truth, applied, interval, actual);
                elapsed += interval;
                observations.push_back({elapsed, truth});
                while (observations.size() > 1 && observations[1].time < elapsed - .12) observations.pop_front();
            }
            elapsed = until;
        };
        while (!pending.empty() && pending.front().time <= end) {
            advancePlant(pending.front().time);
            applied = pending.front().voltage;
            pending.pop_front();
        }
        advancePlant(end);
        previousDt = dt;
        const DriveState reference = controller.reference();
        squaredTracking += square(truth.pose.x - reference.pose.x) + square(truth.pose.y - reference.pose.y);
        worst = std::max(worst, stats.computeMs);
        total += stats.computeMs;
        ++calls;
        estimate.state = truth;
        if (controller.settled(estimate, dt)) { done = true; break; }
    }
    const double position = std::hypot(truth.pose.x - target.pose.x, truth.pose.y - target.pose.y);
    const double heading = std::abs(wrap(truth.pose.theta - target.pose.theta));
    Result result{position, heading, elapsed, std::sqrt(squaredTracking / calls), worst, total / calls, done};
    std::cout << "queue latency_ms=" << latency * 1000 << " scenario=" << scenario << " jitter=" << jitter
              << " mismatch=" << mismatch << " pos_mm=" << 1000 * position << " heading_deg=" << degrees(heading)
              << " precise=" << precise << " observation_ms=" << observationAge * 1000 << " compensated=" << compensateAge
              << " delay_jitter=" << transportJitter << " time_s=" << elapsed << " rms_mm=" << 1000 * result.rmsTracking
              << " settled=" << done << '\n';
    return result;
}
}

void checkArrivalWithFriction(unsigned scenario, bool dryFriction) {
    DynamicsConfig model;
    model.trackWidth = .287;
    model.commandLatency = .01;
    DynamicsConfig actual = model;
    actual.kVLeft *= 1.09; actual.kVRight *= .94;
    actual.kALeft *= 1.1; actual.kARight *= .85;
    actual.kSLeft = 1.45; actual.kSRight = 1.8;
    MotionOptions options;
    options.maxSpeed = 1.5;
    options.positionTolerance = .01;
    options.reverse = scenario == 2;
    const Target target = scenario == 0 ? Target{{.6, .6, 0}, true, false} :
        scenario == 1 ? Target{{-.6, .6, 0}, true, false} :
        scenario == 2 ? Target{{-.6, -.6, 0}, true, false} : Target{{.3, .7, radians(60)}, true, false};
    Controller controller(model);
    controller.setClock(clockSeconds); controller.setBudgetMs(500);
    DriveState truth;
    controller.start(truth, target, options);
    Estimate observation;
    observation.health = Health::healthy;
    observation.covariance[0][0] = observation.covariance[1][1] = .005;
    observation.covariance[2][2] = square(radians(6));
    Voltage previous{};
    double elapsed = 0, firstNear = -1, extraTravel = 0;
    bool done = false;
    const auto advance = [&](Voltage u) {
        // Independent 1 ms plant; static breakaway differs from the smooth
        // motor model optimized by the controller. Include command latency.
        for (unsigned i = 0; i < 10; ++i) {
            auto applied = u;
            auto parameters = actual;
            if (dryFriction) {
                const auto net = [](double voltage, double speed, double ks) {
                    const double friction = std::abs(speed) < .002 ?
                        std::clamp(voltage, -ks, ks) : std::copysign(ks, speed);
                    return voltage - friction;
                };
                applied.left = net(u.left, truth.v + truth.omega * actual.trackWidth / 2, actual.kSLeft);
                applied.right = net(u.right, truth.v - truth.omega * actual.trackWidth / 2, actual.kSRight);
                parameters.kSLeft = parameters.kSRight = 0;
            }
            truth = plant(truth, applied, .001, parameters);
        }
    };
    for (unsigned step = 0; step < 400; ++step) {
        if (scenario == 0 && step == 55) { truth.pose.x += .025; truth.pose.theta += radians(3); }
        observation.state = truth;
        observation.state.v += .004 * std::sin(step * 1.71);
        observation.state.omega += .008 * std::sin(step * 1.27);
        const auto command = controller.update(observation, .02);
        require(controller.stats().valid, "friction arrival keeps a valid bounded solve");
        require(std::abs(command.left) <= 12 && std::abs(command.right) <= 12, "adaptive load obeys voltage limits");
        advance(previous); advance(command); previous = command;
        elapsed += .02;
        const double error = std::hypot(truth.pose.x - target.pose.x, truth.pose.y - target.pose.y);
        if (error < .05 && firstNear < 0) firstNear = elapsed;
        if (firstNear >= 0) extraTravel += std::abs(truth.v) * .02;
        observation.state = truth;
        if (controller.settled(observation, .02)) { done = true; break; }
    }
    const double error = std::hypot(truth.pose.x - target.pose.x, truth.pose.y - target.pose.y);
    std::cout << "friction arrival " << scenario << " dry=" << dryFriction << " time=" << elapsed
              << " correction_s=" << elapsed - firstNear << " extra_mm=" << extraTravel * 1000
              << " error_mm=" << error * 1000 << " heading=" << degrees(wrap(truth.pose.theta-target.pose.theta)) << '\n';
    require(done && error <= options.positionTolerance, "unmodeled friction still reaches requested point");
    require(elapsed < 4 && elapsed - firstNear < 1.2 && extraTravel < .085,
            "first arrival settles without repeated parking curves");
}

void checkAutomaticCorrection(bool initialReverse, bool overshot, bool mismatch, bool constrainHeading) {
    DynamicsConfig model;
    model.trackWidth = .287;
    model.commandLatency = .01;
    DynamicsConfig actual = model;
    if (mismatch) {
        actual.kVLeft *= 1.09; actual.kVRight *= .94;
        actual.kALeft *= 1.1; actual.kARight *= .85;
        actual.kSLeft = 1.45; actual.kSRight = 1.8;
    }
    Controller controller(model);
    controller.setClock(clockSeconds); controller.setBudgetMs(500);
    MotionOptions options;
    options.maxSpeed = 1.5;
    options.positionTolerance = .01;
    options.reverse = initialReverse;
    // Rotating the whole experiment also covers heading wrap near +/-pi.
    const double heading = initialReverse ? radians(178) : 0;
    const double s = std::sin(heading), c = std::cos(heading);
    const double direction = initialReverse ? -1 : 1;
    const Target target{{direction * .8 * s, direction * .8 * c, heading}, constrainHeading, false};
    DriveState truth{{0, 0, heading}, 0, 0};
    Estimate estimate;
    estimate.health = Health::healthy;
    Voltage previous{};
    controller.start(truth, target, options);
    const auto step = [&](unsigned tick) {
        estimate.state = truth;
        if (mismatch) {
            estimate.state.pose.x += .001 * std::sin(tick * 1.37);
            estimate.state.pose.y += .001 * std::sin(tick * 1.11);
            estimate.state.pose.theta += radians(.15) * std::sin(tick * .71);
            estimate.state.v += .002 * std::sin(tick * 1.71);
        }
        const auto command = controller.update(estimate, .02);
        require(controller.stats().valid && std::abs(command.left) <= 12 && std::abs(command.right) <= 12,
                "automatic direction preserves valid voltage limits");
        truth = plant(truth, previous, .01, actual);
        truth = plant(truth, command, .01, actual);
        previous = command;
        estimate.state = truth;
    };
    // Complete a real initial approach, retaining its live command history
    // and learned load. Then reproduce a terminal longitudinal/lateral error.
    unsigned tick = 0;
    bool arrived = false;
    for (; tick < 500; ++tick) {
        step(tick);
        if (controller.settled(estimate, .02) && std::abs(controller.reference().v) < 1e-6) {
            arrived = true; break;
        }
    }
    require(arrived, "setup approach reaches the terminal reference");
    const double longitudinal = overshot ? .09 : -.09;
    const double lateral = initialReverse ? -.025 : .025;
    truth.pose = {target.pose.x + longitudinal * s + lateral * c,
                  target.pose.y + longitudinal * c - lateral * s, wrap(heading + radians(5))};
    truth.v = truth.omega = 0;
    const double correctionDirection = overshot ? -1 : 1;
    double elapsed = 0, correctTravel = 0, wrongTravel = 0, peakError = 0;
    int lastReferenceSign = 0;
    unsigned directionChanges = 0;
    bool done = false;
    for (unsigned correction = 0; correction < 250; ++correction) {
        step(++tick);
        elapsed += .02;
        const auto reference = controller.reference();
        if (std::abs(reference.v) > .005) {
            const int sign = reference.v > 0 ? 1 : -1;
            if (lastReferenceSign && sign != lastReferenceSign) ++directionChanges;
            lastReferenceSign = sign;
        }
        correctTravel += std::max(0.0, correctionDirection * truth.v) * .02;
        wrongTravel += std::max(0.0, -correctionDirection * truth.v) * .02;
        peakError = std::max(peakError, std::hypot(truth.pose.x - target.pose.x, truth.pose.y - target.pose.y));
        if (controller.settled(estimate, .02)) { done = true; break; }
    }
    const double error = std::hypot(truth.pose.x - target.pose.x, truth.pose.y - target.pose.y);
    std::cout << "automatic correction pose=" << constrainHeading << " initial_reverse=" << initialReverse << " overshot=" << overshot
              << " mismatch=" << mismatch << " time=" << elapsed << " error_mm=" << error * 1000
              << " correct_mm=" << correctTravel * 1000 << " wrong_mm=" << wrongTravel * 1000
              << " peak_mm=" << peakError * 1000 << " switches=" << directionChanges << '\n';
    require(done && error <= options.positionTolerance && elapsed < 3,
            "terminal correction autonomously chooses a short approach in either direction");
    require(correctTravel > .04 && wrongTravel < .015 && peakError < .115 && directionChanges <= 1,
            "correction avoids driving a loop away from the target or chattering directions");

    // An automatic repair must not change the direction of the next request.
    const Target next{{truth.pose.x + direction * .3 * std::sin(truth.pose.theta),
                       truth.pose.y + direction * .3 * std::cos(truth.pose.theta), truth.pose.theta}, true, false};
    controller.start(truth, next, options);
    controller.update(estimate, .02);
    require(direction * controller.reference().v > 0, "new request restores its explicit initial direction");
}

void checkOmniDiagonal(double gripTime, double direction) {
    DynamicsConfig model;
    model.trackWidth = .287;
    model.maxLateralAcceleration = 1.;
    Controller controller(model);
    controller.setClock(clockSeconds); controller.setBudgetMs(500);
    MotionOptions options;
    options.maxSpeed = 1.2; options.positionTolerance = .05;
    const Target target{{direction * .6, .6, 0}, false, false};
    Estimate estimate; estimate.health = Health::healthy;
    estimate.covariance[0][0] = estimate.covariance[1][1] = square(.05);
    estimate.covariance[2][2] = square(radians(10));
    controller.start({}, target, options);
    DriveState truth;
    Voltage previous{};
    double smoothV = 0, smoothOmega = 0, smoothLateral = 0;
    double rotation = 0, elapsed = 0;
    const auto filter = [](double measured, double& smooth, double& output) {
        const double interval = .02 / .07, decay = std::exp(-interval), alpha = 1 - decay;
        output += alpha * (measured - output) + (measured - smooth) * interval * decay;
        smooth += alpha * (measured - smooth);
    };
    bool done = false;
    while (elapsed < 10) {
        const auto command = controller.update(estimate, .02);
        require(controller.stats().valid, "omni diagonal retains finite prediction and control");
        // Independent plant: measured sideways momentum, unknown to the
        // controller, decays with several synthetic rolling-resistance values.
        // Include the real 10 ms command delay and compensated velocity filter.
        for (unsigned step = 0; step < 20; ++step) {
            const auto before = truth;
            truth = plant(before, step < 10 ? previous : command, .001, model);
            truth.lateralV = gripTime > 0 ? before.lateralV + .001 *
                (-before.v * before.omega - before.lateralV / gripTime) : 0;
            truth.pose.x += .001 * truth.lateralV * std::cos(before.pose.theta);
            truth.pose.y -= .001 * truth.lateralV * std::sin(before.pose.theta);
            rotation += std::abs(wrap(truth.pose.theta - before.pose.theta));
        }
        previous = command;
        estimate.state.pose = truth.pose;
        filter(truth.v, smoothV, estimate.state.v);
        filter(truth.omega, smoothOmega, estimate.state.omega);
        filter(truth.lateralV, smoothLateral, estimate.state.lateralV);
        elapsed += .02;
        if (controller.settled(estimate, .02)) { done = true; break; }
    }
    const double error = std::hypot(truth.pose.x - target.pose.x, truth.pose.y - target.pose.y);
    std::cout << "omni diagonal grip_s=" << gripTime << " direction=" << direction
              << " seconds=" << elapsed << " error_mm=" << error * 1000
              << " turn_deg=" << degrees(rotation) << '\n';
    require(done && error <= options.positionTolerance && rotation < 2 * pi,
            "600x600 omni approach settles without repeated full turns despite measured sideways drift");
}

void checkAnticipatedArrival(unsigned scenario) {
    DynamicsConfig model;
    model.trackWidth = .287;
    model.maxLateralAcceleration = 1.;
    DynamicsConfig actual = model;
    actual.kVLeft *= 1.09; actual.kVRight *= .94;
    actual.kALeft *= 1.1; actual.kARight *= .85;
    actual.kSLeft = 1.; actual.kSRight = 1.2;
    Controller controller(model);
    controller.setClock(clockSeconds); controller.setBudgetMs(500);
    MotionOptions options;
    options.maxSpeed = 1.5; options.positionTolerance = .01;
    const double direction = scenario % 2 ? -1. : 1.;
    const Target target = scenario < 8 ? Target{{direction * .6, .6, 0}, true, false} :
                                        Target{{0, .9, 0}, true, false};
    Estimate estimate;
    estimate.health = Health::healthy;
    estimate.covariance[0][0] = estimate.covariance[1][1] = square(.05);
    estimate.covariance[2][2] = square(radians(6));
    DriveState truth;
    controller.start(truth, target, options);
    Voltage previous{};
    const double grip = scenario < 4 ? 0. : scenario < 8 ? .08 : .2;
    double elapsed = 0, nearAt = -1, extra = 0, reverse = 0, rotation = 0;
    double smoothV = 0, smoothOmega = 0, smoothLateral = 0;
    bool pushed = false, done = false;
    const auto filter = [](double measured, double& smooth, double& output) {
        const double interval = .02 / .07, decay = std::exp(-interval), alpha = 1 - decay;
        output += alpha * (measured - output) + (measured - smooth) * interval * decay;
        smooth += alpha * (measured - smooth);
    };
    for (unsigned step = 0; step < 400; ++step) {
        const double remaining = std::hypot(truth.pose.x - target.pose.x, truth.pose.y - target.pose.y);
        if (!pushed && scenario % 4 >= 2 && remaining < .38) {
            truth.pose.x += direction * .045;
            truth.pose.theta += direction * radians(9);
            pushed = true;
        }
        estimate.state.pose = truth.pose;
        filter(truth.v, smoothV, estimate.state.v);
        filter(truth.omega, smoothOmega, estimate.state.omega);
        filter(truth.lateralV, smoothLateral, estimate.state.lateralV);
        const auto command = controller.update(estimate, .02);
        require(controller.stats().valid && std::abs(command.left) <= 12 && std::abs(command.right) <= 12,
                "anticipatory approach retains a finite bounded solve");
        // Independent 1ms plant: motor mismatch, 10ms transport, lateral
        // momentum and filtered feedback. The controller is not given grip.
        for (unsigned i = 0; i < 20; ++i) {
            const auto before = truth;
            truth = plant(before, i < 10 ? previous : command, .001, actual);
            truth.lateralV = grip > 0 ? before.lateralV + .001 *
                (-before.v * before.omega - before.lateralV / grip) : 0;
            truth.pose.x += .001 * truth.lateralV * std::cos(before.pose.theta);
            truth.pose.y -= .001 * truth.lateralV * std::sin(before.pose.theta);
            rotation += std::abs(wrap(truth.pose.theta - before.pose.theta));
        }
        previous = command;
        elapsed += .02;
        if (std::hypot(truth.pose.x - target.pose.x, truth.pose.y - target.pose.y) < .05 && nearAt < 0)
            nearAt = elapsed;
        if (nearAt >= 0) {
            extra += std::hypot(truth.v, truth.lateralV) * .02;
            reverse += std::max(0., -truth.v) * .02;
        }
        auto arrival = estimate;
        arrival.state = truth;
        if (controller.settled(arrival, .02)) { done = true; break; }
    }
    std::cout << "anticipated arrival " << scenario << " seconds=" << elapsed
              << " correction_s=" << elapsed - nearAt << " extra_mm=" << extra * 1000
              << " reverse_mm=" << reverse * 1000 << " turn_deg=" << degrees(rotation) << '\n';
    require(done && nearAt >= 0 && elapsed < 4 && rotation < 2 * pi,
            "approach reaches the unchanged pose tolerance without a full parking turn");
    if (scenario < 10) {
        require(elapsed - nearAt < 1.2 && extra < .085 && reverse < .005,
                "diagonal and unpushed straight approaches settle on first arrival without reversing");
    } else {
        require(elapsed - nearAt < 2 && extra < .2 && reverse < .1,
                "large late straight-line disturbance retains bounded terminal recovery");
    }
}

void checkWideCorrection(unsigned scenario) {
    DynamicsConfig model;
    model.trackWidth = .287;
    model.maxLateralAcceleration = 1.;
    DynamicsConfig actual = model;
    actual.kVLeft *= 1.05; actual.kVRight *= .96;
    actual.kSLeft = .9; actual.kSRight = 1.1;
    Controller controller(model);
    controller.setClock(clockSeconds); controller.setBudgetMs(500);
    MotionOptions options;
    options.positionTolerance = scenario < 12 ? .001 : .005;
    const double errors[] = {.003, .006, .012, .025, .050, .080};
    const double side = scenario % 2 ? -1. : 1.;
    const double error = side * errors[(scenario / 2) % 6];
    // A terminal pose residual, after the nominal path has reached its goal.
    controller.start({}, {{0, 0, 0}, true, false}, options);
    Estimate estimate;
    estimate.health = Health::healthy;
    estimate.covariance[0][0] = estimate.covariance[1][1] = square(.03);
    DriveState truth{{error, -.003, side * radians(3)}, 0, 0};
    Voltage previous{};
    double elapsed = 0, travel = 0, rotation = 0, peak = 0;
    double smoothV = 0, smoothOmega = 0, smoothLateral = 0;
    int gear = 0, switches = 0, referenceGear = 0, referenceSwitches = 0;
    bool done = false;
    const auto filter = [](double value, double& smooth, double& output) {
        const double interval = .02 / .07, decay = std::exp(-interval), alpha = 1 - decay;
        output += alpha * (value - output) + (value - smooth) * interval * decay;
        smooth += alpha * (value - smooth);
    };
    const auto countGear = [](double speed, int& previousGear, int& changes) {
        const int currentGear = speed > .02 ? 1 : speed < -.02 ? -1 : 0;
        if (currentGear) {
            if (previousGear && previousGear != currentGear) ++changes;
            previousGear = currentGear;
        }
    };
    for (unsigned step = 0; step < 300; ++step) {
        estimate.state.pose = truth.pose;
        filter(truth.v, smoothV, estimate.state.v);
        filter(truth.omega, smoothOmega, estimate.state.omega);
        filter(truth.lateralV, smoothLateral, estimate.state.lateralV);
        const auto command = controller.update(estimate, .02);
        require(controller.stats().valid && std::abs(command.left) <= 12 && std::abs(command.right) <= 12,
                "wide correction keeps a bounded valid control");
        countGear(controller.reference().v, referenceGear, referenceSwitches);
        for (unsigned i = 0; i < 20; ++i) {
            const auto before = truth;
            truth = plant(before, i < 10 ? previous : command, .001, actual);
            truth.lateralV = before.lateralV + .001 *
                (-before.v * before.omega - before.lateralV / .08);
            truth.pose.x += .001 * truth.lateralV * std::cos(before.pose.theta);
            truth.pose.y -= .001 * truth.lateralV * std::sin(before.pose.theta);
            travel += std::hypot(truth.v, truth.lateralV) * .001;
            rotation += std::abs(wrap(truth.pose.theta - before.pose.theta));
        }
        previous = command;
        elapsed += .02;
        peak = std::max(peak, std::hypot(truth.pose.x, truth.pose.y));
        countGear(truth.v, gear, switches);
        auto arrival = estimate;
        arrival.state = truth;
        if (controller.settled(arrival, .02)) { done = true; break; }
    }
    std::cout << "wide correction " << scenario << " tolerance_mm=" << options.positionTolerance * 1000
              << " seconds=" << elapsed << " switches=" << switches << " planned=" << referenceSwitches
              << " travel_mm=" << travel * 1000 << " turn_deg=" << degrees(rotation)
              << " peak_mm=" << peak * 1000 << '\n';
    require(done && elapsed < 5.5 && peak < .35 && rotation < radians(140),
            "3-80mm lateral residuals settle with bounded broad steering despite drift and filtering");
    require(switches <= 2 && referenceSwitches <= 2,
            "wide recovery avoids repeated forward-back parking oscillations");
    if (std::abs(error) >= .025)
        require(referenceSwitches == 1, "substantial lateral residual selects one planned gear change");
}

void checkLateralCorrection(double lateralError, bool initialReverse) {
    DynamicsConfig model;
    model.trackWidth = .287;
    model.commandLatency = .01;
    Controller controller(model);
    controller.setClock(clockSeconds); controller.setBudgetMs(500);
    MotionOptions options;
    options.maxSpeed = 1.5;
    options.positionTolerance = .01;
    options.reverse = initialReverse;
    const double heading = initialReverse ? radians(178) : 0;
    const double s = std::sin(heading), c = std::cos(heading);
    const double direction = initialReverse ? -1 : 1;
    const Target target{{direction * .6 * s, direction * .6 * c, heading}, true, false};
    Estimate estimate;
    estimate.health = Health::healthy;
    estimate.state.pose.theta = heading;
    controller.start(estimate.state, target, options);
    double lateralSpeed = 0;
    Voltage previous{};
    const auto step = [&]() {
        const auto command = controller.update(estimate, .02);
        require(controller.stats().valid, "lateral repair retains a valid solve");
        for (unsigned i = 0; i < 20; ++i) {
            const auto before = estimate.state;
            estimate.state = plant(before, i < 10 ? previous : command, .001, model);
            // Independent omni plant: sideways momentum persists as the body
            // turns, then decays under rolling resistance. This synthetic .05 s
            // time constant is a stress case, not a claimed robot calibration.
            lateralSpeed += .001 * (-before.v * before.omega - lateralSpeed / .05);
            estimate.state.pose.x += .001 * lateralSpeed * std::cos(before.pose.theta);
            estimate.state.pose.y -= .001 * lateralSpeed * std::sin(before.pose.theta);
            estimate.state.lateralV = lateralSpeed;
        }
        previous = command;
    };
    bool arrived = false;
    for (unsigned i = 0; i < 300; ++i) {
        step();
        if (controller.settled(estimate, .02) && std::abs(controller.reference().v) < 1e-6) {
            arrived = true; break;
        }
    }
    require(arrived, "lateral repair starts after a real approach");
    estimate.state.pose.x += lateralError * c;
    estimate.state.pose.y -= lateralError * s;
    // For the 178-degree case this crosses the IMU +/-pi branch. The first
    // correction segment and straight final segment must share one unwrapped
    // heading branch, including the stopped gear-change interval.
    if (initialReverse) estimate.state.pose.theta = wrap(heading + radians(5));
    // Still sliding sideways: the previous v/omega-only stop detector misses it.
    lateralSpeed = std::copysign(.06, lateralError);
    estimate.state.lateralV = lateralSpeed;
    double angle = 0, forward = 0, reverse = 0, elapsed = 0, peakError = 0;
    double peakReferenceAcceleration = 0, peakReferenceYawRate = 0;
    auto previousReference = controller.reference();
    bool done = false;
    for (unsigned i = 0; i < 250; ++i) {
        step(); elapsed += .02;
        const auto reference = controller.reference();
        if (i > 0) peakReferenceYawRate = std::max(peakReferenceYawRate,
            std::abs(wrap(reference.pose.theta - previousReference.pose.theta)) / .02);
        for (double side : {-1.0, 1.0}) {
            const double change = reference.v - previousReference.v + side * model.trackWidth / 2 *
                                  (reference.omega - previousReference.omega);
            peakReferenceAcceleration = std::max(peakReferenceAcceleration, std::abs(change) / .02);
        }
        previousReference = reference;
        angle += std::abs(estimate.state.omega) * .02;
        forward += std::max(0.0, estimate.state.v) * .02;
        reverse += std::max(0.0, -estimate.state.v) * .02;
        peakError = std::max(peakError, std::hypot(estimate.state.pose.x - target.pose.x,
                                               estimate.state.pose.y - target.pose.y));
        if (controller.settled(estimate, .02)) { done = true; break; }
    }
    const double error = std::hypot(estimate.state.pose.x - target.pose.x,
                                    estimate.state.pose.y - target.pose.y);
    std::cout << "lateral repair mm=" << lateralError * 1000 << " initial_reverse=" << initialReverse
              << " seconds=" << elapsed << " error_mm=" << error * 1000
              << " turn_deg=" << degrees(angle) << " forward_mm=" << forward * 1000
              << " reverse_mm=" << reverse * 1000 << '\n';
    require(done && error <= options.positionTolerance && elapsed < 3.5,
            "rolling lateral errors settle promptly at the original target");
    require(forward > .03 && reverse > .03 && angle < radians(150) && peakError < .25,
            "lateral repair uses both directions with bounded excursion instead of a tight one-gear loop");
    require(peakReferenceAcceleration <= model.maxAcceleration + 1e-7,
            "both gears and their stopped transition respect wheel reference acceleration");
    require(peakReferenceYawRate <= model.maxOmega + .01,
            "mixed correction heading stays continuous across the wrap and stopped gear change");
}

int main() {
    std::cout << std::fixed << std::setprecision(3);
    MotionOptions options;
    for (unsigned scenario = 0; scenario < 24; ++scenario) checkWideCorrection(scenario);
    for (unsigned scenario = 0; scenario < 12; ++scenario) checkAnticipatedArrival(scenario);
    for (double grip : {0., .05, .2, .5}) for (double direction : {-1., 1.})
        checkOmniDiagonal(grip, direction);
    {
        Controller drift;
        for (double heading : {0., pi / 2, pi, -pi / 2}) {
            DriveState state{{0, 0, heading}, 0, 0, .2};
            const auto predicted = drift.predict(state, {}, .1);
            require(std::abs(predicted.pose.x - .02 * std::cos(heading)) < 1e-9 &&
                    std::abs(predicted.pose.y + .02 * std::sin(heading)) < 1e-9 &&
                    predicted.lateralV == state.lateralV,
                    "prediction carries measured body-right drift into the correct world axes");
        }
    }
    {
        Controller point;
        Estimate atGoal; atGoal.health = Health::healthy;
        point.start({}, {{.6, .6, 0}, false, false});
        atGoal.state.pose = {.6, .6, radians(170)};
        const auto command = point.update(atGoal, .02);
        require(std::abs(command.left) < 1e-9 && std::abs(command.right) < 1e-9,
                "a stopped point arrival does not turn to finish the timed spline heading");
    }
    for (double lateralError : {-.05, -.02, .02, .05}) for (bool reverse : {false, true})
        checkLateralCorrection(lateralError, reverse);
    {
        Controller sliding;
        Estimate e; e.health = Health::healthy;
        MotionOptions stopOptions; stopOptions.settleTime = 0;
        sliding.start({}, {{0, 0, 0}, true, false}, stopOptions);
        e.state.lateralV = .06;
        require(!sliding.settled(e, .02), "sideways drift cannot count as stopped at the target");
        e.state.lateralV = 0;
        require(sliding.settled(e, .02), "a genuinely stopped target can settle");
        e.state.lateralV = std::numeric_limits<double>::quiet_NaN();
        const auto command = sliding.update(e, .02);
        require(command.left == 0 && command.right == 0 && !sliding.stats().valid,
                "nonfinite lateral velocity rejects the observation");
    }
    for (bool constrainHeading : {false, true}) for (bool initialReverse : {false, true})
        for (bool overshot : {false, true}) for (bool mismatch : {false, true})
            checkAutomaticCorrection(initialReverse, overshot, mismatch, constrainHeading);
    for (unsigned scenario = 0; scenario < 4; ++scenario)
        for (bool dry : {false, true}) checkArrivalWithFriction(scenario, dry);
    run("straight", {}, {{0, 1.5, 0}, false, false}, options);
    MotionOptions reverse = options;
    reverse.reverse = true;
    run("reverse", {}, {{0, -1.3, 0}, true, false}, reverse);
    run("curve pose", {}, {{0.9, 1.2, radians(90)}, true, false}, options);
    run("point off axis", {}, {{0.8, 1.1, 0}, false, false}, options);
    run("large heading", {}, {{0.8, 0.7, radians(165)}, true, false}, options);
    run("behind point", {}, {{0, -1, 0}, false, false}, options);
    run("behind pose", {}, {{0, -1, 0}, true, false}, options);
    run("lateral pose", {}, {{.4, 0, 0}, true, false}, options);
    run("rolling lateral", {{0, 0, 0}, .7, 0}, {{.4, 0, 0}, true, false}, options);
    run("reverse curve", {}, {{-.8, -1.2, radians(65)}, true, false}, reverse);
    run("short pose", {}, {{0.06, 0.1, radians(90)}, true, false}, options);
    run("fast braking", {{0, 0, 0}, 1.4, 0}, {{0, 0.45, 0}, true, false}, options);
    run("turn 170 deg", {}, {{0, 0, radians(170)}, true, true}, options);
    run("turn across wrap", {{0, 0, radians(175)}, 0, 0},
        {{0, 0, radians(-175)}, true, true}, options);
    run("pose across wrap", {{0, 0, radians(175)}, 0, 0},
        {{-.1, -1, radians(-170)}, true, false}, options);
    run("initial reverse", {{0, 0, 0}, -.7, 0}, {{.2, 1, radians(20)}, true, false}, options);
    run("turn while moving", {{0, 0, 0}, .8, 0}, {{0, 0, radians(90)}, true, true}, options);
    run("low battery", {}, {{0.7, 1.0, radians(65)}, true, false}, options, 7.5);
    run("push recovery", {}, {{0.5, 1.4, radians(30)}, true, false}, options, 12, 1);
    run("collision slip", {}, {{0.5, 1.4, radians(30)}, true, false}, options, 12, 2);
    run("motor mismatch", {}, {{0.6, 1.2, radians(45)}, true, false}, options, 12, 0, true);
    run("command latency", {}, {{0.7, 1.1, radians(60)}, true, false}, options, 12, 0, false, true);
    MotionOptions fast = options;
    fast.maxSpeed = 1.65;
    run("high speed", {}, {{1.2, 2.5, radians(55)}, true, false}, fast);
    MotionOptions precise = options;
    precise.positionTolerance = 0.001;
    precise.headingTolerance = radians(0.1);
    const auto tight = run("tight tolerance", {}, {{0.5, 1.1, radians(50)}, true, false}, precise);
    require(tight.seconds < 5, "millimetre terminal corrections avoid oversized tangent loops");
    const auto repeatedA = run("repeat A", {}, {{0.4, 1.1, radians(40)}, true, false}, options);
    const auto repeatedB = run("repeat B", {}, {{0.4, 1.1, radians(40)}, true, false}, options);
    require(std::abs(repeatedA.position - repeatedB.position) < 1e-10, "deterministic repeatability");

    checkReferenceAcceleration({{.1, 0, 0}, true, false}, false);
    checkReferenceAcceleration({{0, -1, 0}, true, false}, false);
    checkReferenceAcceleration({{-.1, 0, 0}, true, false}, true);
    checkReferenceAcceleration({{.0006, .0006, radians(90)}, true, false}, false);

    Controller certain, uncertain;
    Estimate good, poor;
    good.health = poor.health = Health::healthy;
    poor.covariance[0][0] = poor.covariance[1][1] = 0.04;
    poor.covariance[2][2] = square(radians(10));
    certain.start({}, {{0, 2, 0}, false, false});
    uncertain.start({}, {{0, 2, 0}, false, false});
    certain.update(good, 0.02);
    uncertain.update(poor, 0.02);
    require(uncertain.reference().v < certain.reference().v, "uncertainty lowers reference acceleration");

    Controller bounded;
    Estimate estimate;
    estimate.health = Health::healthy;
    bounded.start({}, {{0, 1, 0}, false, false});
    bounded.setClock(advancingClock);
    bounded.setBudgetMs(0.1);
    const Voltage fallback = bounded.update(estimate, 0.02, 8);
    require(bounded.stats().budgetExceeded && bounded.stats().valid, "deadline returns best bounded feasible rollout");
    require(std::abs(fallback.left) <= 8 && std::abs(fallback.right) <= 8, "deadline voltage limits");
    estimate.health = Health::lost;
    const Voltage lost = bounded.update(estimate, 0.02);
    require(lost.left == 0 && lost.right == 0 && !bounded.stats().valid, "lost localization stops controller");
    estimate.health = Health::healthy;
    estimate.state.pose.x = std::numeric_limits<double>::quiet_NaN();
    require(bounded.update(estimate, 0.02).left == 0, "nonfinite pose is rejected");
    estimate.state.pose.x = 0;
    estimate.covariance[0][0] = std::numeric_limits<double>::quiet_NaN();
    const Voltage invalidCovariance = bounded.update(estimate, 0.02);
    require(invalidCovariance.left == 0 && invalidCovariance.right == 0 && !bounded.stats().valid,
            "nonfinite uncertainty cannot be treated as a certain pose");
    require(!bounded.settled(estimate, .2), "invalid uncertainty cannot satisfy settling");
    estimate.covariance[0][0] = -.001;
    bounded.update(estimate, .02);
    require(!bounded.stats().valid, "negative variance is rejected");
    estimate.covariance[0][0] = 0;
    estimate.slip = std::numeric_limits<double>::quiet_NaN();
    bounded.update(estimate, .02);
    require(!bounded.stats().valid, "invalid slip estimate is rejected");

    // Expire during a speculative line-search trajectory, not before solving.
    // A partially evaluated objective must never replace the complete rollout.
    Controller interrupted;
    fakeTime = 0;
    interrupted.setClock(fineClock);
    interrupted.setBudgetMs(.25);
    interrupted.start({}, {{.8, 1, radians(70)}, true, false});
    estimate.slip = 0;
    const Voltage interruptedOutput = interrupted.update(estimate, .02, 9);
    const SolverStats interruptedStats = interrupted.stats();
    require(interruptedStats.budgetExceeded && interruptedStats.valid && interruptedStats.iterations == 1,
            "budget can interrupt the first speculative line search");
    require(interruptedStats.finalCost == interruptedStats.initialCost,
            "interrupted trial preserves complete feasible objective");
    require(std::abs(interruptedOutput.left) <= 9 && std::abs(interruptedOutput.right) <= 9,
            "interrupted trial preserves voltage feasibility");

    DynamicsConfig fitted;
    fitted.trackWidth = .34;
    fitted.kVLeft = 6.8;
    interrupted.reconfigure(fitted);
    require(interrupted.config().trackWidth == .34 && interrupted.config().kVLeft == 6.8,
            "runtime calibration updates physical controller model");
    const Voltage reconfiguredIdle = interrupted.update(estimate, .02);
    require(reconfiguredIdle.left == 0 && reconfiguredIdle.right == 0 && !interrupted.stats().valid,
            "runtime reconfiguration cancels old rollout");
    interrupted.start({}, {{.8, 1, radians(70)}, true, false});
    interrupted.update(estimate, .02);
    require(interrupted.stats().budgetExceeded && interrupted.stats().valid,
            "runtime reconfiguration preserves owner task clock and budget");

    Controller invalidOptions;
    Estimate healthy;
    healthy.health = Health::healthy;
    double MotionOptions::* optionFields[] = {&MotionOptions::maxSpeed, &MotionOptions::positionTolerance,
        &MotionOptions::headingTolerance, &MotionOptions::settleTime, &MotionOptions::timeout};
    for (const auto field : optionFields) for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
                                                               std::numeric_limits<double>::infinity()}) {
        MotionOptions invalidMotion;
        invalidMotion.*field = invalid;
        invalidOptions.start({}, {{0, 1, 0}, false, false}, invalidMotion);
        const Voltage stopped = invalidOptions.update(healthy, .02);
        require(stopped.left == 0 && stopped.right == 0 && !invalidOptions.stats().valid &&
                !invalidOptions.settled(healthy, 1), "nonfinite motion options cannot start or settle a move");
    }
    invalidOptions.start({}, {{0, 1, 0}, false, false});
    require(invalidOptions.trajectoryDuration() > 0, "active motion has a trajectory duration");
    invalidOptions.reset();
    require(invalidOptions.trajectoryDuration() == 0, "reset clears the previous trajectory duration");

    DynamicsConfig invalidDynamics;
    double DynamicsConfig::* dynamicsFields[] = {&DynamicsConfig::trackWidth, &DynamicsConfig::kVLeft,
        &DynamicsConfig::kVRight, &DynamicsConfig::kALeft, &DynamicsConfig::kARight, &DynamicsConfig::kSLeft,
        &DynamicsConfig::kSRight, &DynamicsConfig::maxAcceleration, &DynamicsConfig::maxLateralAcceleration,
        &DynamicsConfig::maxSpeed, &DynamicsConfig::maxOmega, &DynamicsConfig::maxVoltage, &DynamicsConfig::commandLatency};
    for (const auto field : dynamicsFields) invalidDynamics.*field = std::numeric_limits<double>::infinity();
    Controller sanitized(invalidDynamics);
    for (const auto field : dynamicsFields)
        require(sanitized.config().*field == DynamicsConfig{}.*field,
                "nonfinite physical parameters use finite defaults");
    sanitized.setClock(advancingClock);
    sanitized.setBudgetMs(std::numeric_limits<double>::infinity());
    sanitized.start({}, {{0, 1, 0}, false, false});
    sanitized.update(healthy, .02);
    require(sanitized.stats().valid && sanitized.stats().budgetExceeded,
            "invalid budget cannot disable the deadline or invalidate the sanitized model");

    for (double latency : {.01, .04, .08, .12}) for (unsigned scenario = 0; scenario < 3; ++scenario) {
        const Result result = runQueuedLatency(latency, scenario);
        require(result.settled && result.position <= .001 && result.heading <= radians(.1) && result.seconds < 4,
                "actual command queue settles within precise tolerances across delays and changing control periods");
    }
    // Hold-out geometries and dynamics, plus actual transport jitter not known
    // by the controller. These use the public motion tolerances, not an exact
    // model's simulated millimetre claim.
    for (double latency : {.025, .065, .105}) for (unsigned scenario = 3; scenario < 6; ++scenario) {
        const Result result = runQueuedLatency(latency, scenario, true, true, false, 0, true, true);
        require(result.settled && result.position < .008 && result.heading < radians(.7) && result.seconds < 3,
                "queued prediction remains accurate with unseen model errors and transport jitter");
    }
    for (double latency : {.01, .04}) for (double age : {.005, .01, .02}) {
        const Result delayed = runQueuedLatency(latency, 0, true, false, true, age, false);
        const Result compensated = runQueuedLatency(latency, 0, true, false, true, age, true);
        require(compensated.settled && compensated.position <= .001 && compensated.heading <= radians(.1),
                "aged observations settle within the requested pose tolerance");
        require(compensated.rmsTracking < delayed.rmsTracking * .95,
                "replaying command history improves tracking against genuinely delayed pose samples");
    }
    Controller restarted, fresh;
    DynamicsConfig longDelay;
    longDelay.commandLatency = .15;
    restarted.reconfigure(longDelay);
    fresh.reconfigure(longDelay);
    restarted.start({}, {{.6, 1, radians(65)}, true, false});
    for (unsigned i = 0; i < 300; ++i) restarted.update(healthy, .001, 12, .1);
    restarted.start({}, {{0, 1, 0}, false, false});
    fresh.start({}, {{0, 1, 0}, false, false});
    const Voltage restartVoltage = restarted.update(healthy, .02, 12, .01);
    const Voltage freshVoltage = fresh.update(healthy, .02, 12, .01);
    require(std::abs(restartVoltage.left - freshVoltage.left) < 1e-12 &&
            std::abs(restartVoltage.right - freshVoltage.right) < 1e-12,
            "new motion cannot replay commands from a previous history, including a wrapped ring buffer");
    for (double age : {-.001, .101, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
        const Voltage stopped = restarted.update(healthy, .02, 12, age);
        require(stopped.left == 0 && stopped.right == 0 && !restarted.stats().valid,
                "invalid observation age cannot move the robot");
    }
    std::cout << "Controller simulation tests passed. Host timings are not V5 deadline measurements.\n";
    return 0;
}
