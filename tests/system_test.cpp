#include "nexus/controller.hpp"
#include "nexus/estimator.hpp"
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <random>

using namespace nexus;
namespace {
void require(bool condition, const char* message) {
    if (!condition) { std::cerr << "FAILED: " << message << '\n'; std::exit(1); }
}
double clockSeconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
struct Plant {
    Pose pose{};
    double vl = 0, vr = 0, forward = 0, lateral = 0, left = 0, right = 0, gyro = 0;
    DynamicsConfig config{};
    EstimatorConfig sensors{};
    void advance(Voltage u, double dt, bool slip) {
        // Independent 1ms Euler plant: asymmetric friction/dynamics and no solver calls.
        for (unsigned j = 0; j < static_cast<unsigned>(std::round(dt / .001)); ++j) {
            const double al = std::clamp((u.left - 1.03 * config.kVLeft * vl
                - config.kSLeft * std::tanh(vl / .07)) / (1.08 * config.kALeft), -3.5, 3.5);
            const double ar = std::clamp((u.right - .97 * config.kVRight * vr
                - config.kSRight * std::tanh(vr / .07)) / config.kARight, -3.5, 3.5);
            vl += al * .001; vr += ar * .001;
            const double groundL = vl * (slip ? .65 : 1), groundR = vr * (slip ? .8 : 1);
            const double df = .5 * (groundL + groundR) * .001;
            const double dh = (groundL - groundR) / config.trackWidth * .001;
            pose.x += df * std::sin(pose.theta + dh / 2);
            pose.y += df * std::cos(pose.theta + dh / 2);
            pose.theta += dh;
            forward += df - sensors.forwardOffset * dh;
            lateral += sensors.lateralOffset * dh;
            left += vl * .001; right += vr * .001; gyro += dh;
        }
    }
};
double run(unsigned seed, bool disturbances, bool reverse, bool dropout, bool braking = false) {
    std::mt19937 generator(seed);
    std::normal_distribution<double> normal(0, 1);
    Plant plant;
    plant.config.commandLatency = 0;
    if (braking) plant.vl = plant.vr = 1.4;
    Estimator estimator(plant.sensors);
    Controller controller(plant.config);
    controller.setClock(clockSeconds);
    controller.setBudgetMs(500);
    MotionOptions options;
    options.reverse = reverse;
    const Target target = braking ? Target{{0, .45, 0}, true, false}
        : Target{{reverse ? -.3 : .65, reverse ? -.9 : 1.2, radians(reverse ? 25 : 60)}, true, false};
    SensorSample sample;
    sample.forwardValid = sample.lateralValid = sample.leftValid = sample.rightValid = sample.gyroValid = true;
    sample.gyroRateValid = true;
    estimator.update(sample);
    controller.start({{}, braking ? 1.4 : 0.0, 0}, target, options);
    Estimate estimate;
    Voltage voltage;
    double localizationRms = 0, trackingRms = 0, maxLocalization = 0, time = 0, meanSolve = 0, worstSolve = 0;
    unsigned samples = 0, calls = 0;
    bool done = false;
    for (unsigned iteration = 1; iteration <= 1400; ++iteration) {
        const bool slip = disturbances && iteration >= 90 && iteration < 120;
        plant.advance(voltage, .01, slip);
        if (disturbances && iteration == 160) {
            // Ground push measured by both pods; motor encoders do not see it.
            const double dx = .05, dy = -.02, dh = radians(4);
            plant.forward += dx * std::sin(plant.pose.theta) + dy * std::cos(plant.pose.theta)
                             - plant.sensors.forwardOffset * dh;
            plant.lateral += dx * std::cos(plant.pose.theta) - dy * std::sin(plant.pose.theta)
                             + plant.sensors.lateralOffset * dh;
            plant.pose.x += dx; plant.pose.y += dy; plant.pose.theta += dh; plant.gyro += dh;
        }
        time = iteration * .01;
        sample.timestamp = time;
        sample.forward = plant.forward + normal(generator) * .000025;
        sample.lateral = plant.lateral + normal(generator) * .000025;
        sample.left = plant.left; sample.right = plant.right;
        sample.gyro = plant.gyro + radians(.03) * time + radians(.008) * normal(generator);
        sample.gyroRate = (plant.vl - plant.vr) / plant.config.trackWidth + radians(.03);
        sample.forwardValid = !(dropout && iteration > 65 && iteration < 90);
        // No optional wall sensors exist in any closed-loop run.
        estimate = estimator.update(sample);
        require(estimate.health != Health::lost, "no-distance localization remains usable");
        const double localization = std::hypot(estimate.state.pose.x - plant.pose.x, estimate.state.pose.y - plant.pose.y);
        localizationRms += square(localization); maxLocalization = std::max(maxLocalization, localization);
        ++samples;
        if (iteration % 2 == 0) {
            voltage = controller.update(estimate, .02, 11.5);
            require(controller.stats().valid, "combined noisy estimate yields valid control");
            require(std::abs(voltage.left) <= 11.5 && std::abs(voltage.right) <= 11.5, "system voltage saturation");
            const auto stats = controller.stats();
            meanSolve += stats.computeMs; worstSolve = std::max(worstSolve, stats.computeMs); ++calls;
            const auto reference = controller.reference();
            trackingRms += square(plant.pose.x - reference.pose.x) + square(plant.pose.y - reference.pose.y);
            if (controller.settled(estimate, .02)) { done = true; break; }
        }
    }
    const double error = std::hypot(plant.pose.x - target.pose.x, plant.pose.y - target.pose.y);
    const double headingError = std::abs(wrap(plant.pose.theta - target.pose.theta));
    std::cout << "seed=" << seed << " disturbance=" << disturbances << " reverse=" << reverse << " dropout=" << dropout << " braking=" << braking
        << " final_mm=" << error * 1000 << " heading_deg=" << degrees(headingError)
        << " loc_rms_mm=" << 1000 * std::sqrt(localizationRms / samples) << " loc_max_mm=" << maxLocalization * 1000
        << " tracking_rms_mm=" << 1000 * std::sqrt(trackingRms / calls) << " seconds=" << time
        << " mean_solver_ms=" << meanSolve / calls << " worst_solver_ms=" << worstSolve << '\n';
    require(done, "closed-loop sequence settles within 14 seconds");
    require(error < .03, "absolute final ground-truth position within 30mm under simulated noise");
    require(headingError < radians(2), "absolute heading within two degrees under simulated noise");
    require(maxLocalization < .04, "bounded localization under disturbances");
    return error;
}
}
int main() {
    std::cout << std::fixed << std::setprecision(3);
    double sum = 0, sumSquare = 0;
    for (unsigned seed = 1; seed <= 8; ++seed) {
        const double error = run(seed, false, false, false);
        sum += error; sumSquare += square(error);
    }
    std::cout << "repeatability_final_error_std_mm=" << 1000 * std::sqrt(std::max(0.0, sumSquare / 8 - square(sum / 8))) << '\n';
    run(15, true, false, false);
    run(16, false, true, false);
    run(17, false, false, true);
    run(18, false, false, false, true);
    std::cout << "Coupled estimator/NMPC tests passed; physical validation still required.\n";
}
