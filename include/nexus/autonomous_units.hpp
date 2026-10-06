#pragma once
#include "nexus/types.hpp"
#include <initializer_list>

namespace nexus {
// Public autonomous interface: millimetres, degrees, milliseconds.
struct RobotPose { double x = 0, y = 0, heading = 0; };
struct Waypoint { double x = 0, y = 0; }; // Absolute millimetres.
struct MoveOptions {
    bool reverse = false;
    double maxSpeed = 1397.0; // mm/s, NOT motor power.
    std::uint32_t timeout = 5000; // ms
    double positionTolerance = 15.24; // mm
    double headingTolerance = 1.5; // degrees
    std::uint32_t settleTime = 180; // ms
};

// All public motion entry points share these conversions. Mathematical cores use SI.
inline Pose poseToCore(double x, double y, double heading) {
    return {x * millimeter, y * millimeter, radians(heading)};
}
inline RobotPose poseToMillimeters(const Pose& pose) {
    return {pose.x / millimeter, pose.y / millimeter, degrees(pose.theta)};
}
inline Target routeToCore(std::initializer_list<Waypoint> points, double heading) {
    Target target{{0, 0, radians(heading)}, true, false};
    if (points.size() < 2 || points.size() > maxRoutePoints) {
        // Keep malformed requests in the normal guarded validation path.
        target.viaCount = maxRoutePoints;
        return target;
    }
    target.viaCount = points.size() - 1;
    std::size_t index = 0;
    for (const auto& point : points) {
        if (index < target.viaCount) target.via[index] = poseToCore(point.x, point.y, 0);
        else target.pose = poseToCore(point.x, point.y, heading);
        ++index;
    }
    return target;
}
inline MotionOptions optionsToCore(const MoveOptions& options) {
    return {options.reverse, options.maxSpeed * millimeter, options.positionTolerance * millimeter,
            radians(options.headingTolerance), options.settleTime * 0.001, options.timeout * 0.001};
}
} // namespace nexus
