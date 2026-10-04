#pragma once
#include "nexus/types.hpp"

namespace nexus {
// Public autonomous interface: millimetres, degrees, milliseconds.
struct RobotPose { double x = 0, y = 0, heading = 0; };
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
inline MotionOptions optionsToCore(const MoveOptions& options) {
    return {options.reverse, options.maxSpeed * millimeter, options.positionTolerance * millimeter,
            radians(options.headingTolerance), options.settleTime * 0.001, options.timeout * 0.001};
}
} // namespace nexus
