#include "nexus/autonomous_units.hpp"
#include "nexus/controller.hpp"
#include <cmath>
#include <cstdlib>
#include <iostream>

using namespace nexus;
namespace {
void require(bool condition, const char* message) {
    if (!condition) { std::cerr << "FAILED: " << message << '\n'; std::exit(1); }
}
bool close(double a, double b) { return std::abs(a - b) < 1e-10; }
}
int main() {
    const auto pose = poseToCore(609.6, 914.4, 90);
    require(close(pose.x, .6096) && close(pose.y, .9144) && close(pose.theta, pi / 2),
            "millimetre coordinates enter the SI controller with degrees converted once");
    const auto reported = poseToMillimeters({-.3, 1.2, -pi / 2});
    require(close(reported.x, -300) && close(reported.y, 1200) && close(reported.heading, -90),
            "getPose reports mm with signed coordinates");
    const auto defaults = optionsToCore({});
    require(close(defaults.maxSpeed, 1.397) && close(defaults.positionTolerance, .01524),
            "default physical velocity and tolerance preserved after unit migration");
    const auto options = optionsToCore({.maxSpeed = 800, .timeout = 4000, .positionTolerance = 5});
    require(close(options.maxSpeed, .8) && close(options.positionTolerance, .005)
        && close(options.timeout, 4), "speed mm/s, tolerance mm, timeout ms convert independently");
    const auto route = routeToCore({{-300, 380}, {-420, 400}, {-530, 380}}, -90);
    require(route.viaCount == 2 && close(route.via[0].x, -.3) && close(route.via[0].y, .38)
        && close(route.via[1].x, -.42) && close(route.via[1].y, .4) && close(route.via[0].theta, 0)
        && close(route.pose.x, -.53) && close(route.pose.y, .38) && close(route.pose.theta, -pi / 2)
        && route.constrainHeading && !route.turnOnly,
        "route owns converted intermediate coordinates and only its final heading");
    require(routeToCore({}, 0).viaCount >= maxRoutePoints
        && routeToCore({{0, 0}}, 0).viaCount >= maxRoutePoints
        && routeToCore({{0, 0}, {1, 1}, {2, 2}, {3, 3}, {4, 4}, {5, 5}, {6, 6}, {7, 7}, {8, 8}}, 0).viaCount >= maxRoutePoints,
        "route conversion marks invalid point counts without truncating or indexing storage");
    require(routeToCore({{0, 0}, {1, 1}, {2, 2}, {3, 3}, {4, 4}, {5, 5}, {6, 6}, {7, 7}}, 0).viaCount == 7,
        "the largest route retains all seven guides and its destination");
    require(close(3.25 * inch, .08255) && close(2.125 * inch, .053975),
            "wheel diameters remain inches and convert to correct physical diameters");

    DynamicsConfig model;
    model.commandLatency = 0;
    Controller controller(model);
    const Target target{poseToCore(0, 600, 0), true, false};
    controller.start({}, target, options);
    Estimate estimate;
    estimate.health = Health::healthy;
    bool done = false;
    for (int i = 0; i < 400; ++i) {
        const auto command = controller.update(estimate, .02);
        require(controller.stats().valid, "public mm target produces a valid solve");
        estimate.state = controller.predict(estimate.state, command, .02);
        if (controller.settled(estimate, .02)) { done = true; break; }
    }
    const auto end = poseToMillimeters(estimate.state.pose);
    require(done && std::abs(end.y - 600) <= 5 && std::abs(end.x) <= 5,
            "moveToPose(0,600,0) travels 600mm, with a 5mm final tolerance");
    std::cout << "Autonomous mm units: 600mm target, final Y=" << end.y << "mm; wheel inches preserved.\n";
}
