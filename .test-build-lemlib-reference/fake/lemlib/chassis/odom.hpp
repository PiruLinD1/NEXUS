#pragma once
#include "lemlib/pose.hpp"
#include "lemlib/chassis/chassis.hpp"
namespace lemlib {
void setSensors(OdomSensors, Drivetrain);
Pose getPose(bool radians = false);
void setPose(Pose, bool radians = false);
Pose getSpeed(bool radians = false);
Pose getLocalSpeed(bool radians = false);
Pose estimatePose(float, bool radians = false);
void update();
void init();
}
