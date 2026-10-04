#pragma once
#include "pros/rtos.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
namespace lemlib {
struct OdomSensors {
    TrackingWheel *vertical1, *vertical2, *horizontal1, *horizontal2;
    pros::Imu* imu;
    OdomSensors(TrackingWheel* v1, TrackingWheel* v2, TrackingWheel* h1, TrackingWheel* h2, pros::Imu* g)
        : vertical1(v1), vertical2(v2), horizontal1(h1), horizontal2(h2), imu(g) {}
};
struct Drivetrain { Drivetrain(void*, void*, float, float, float, float) {} };
}
