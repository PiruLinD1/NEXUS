#pragma once
#include "robot/mechanism.hpp"
#include "pros/motors.h"

namespace robot::mechanism {
// Shared by the real motor and the host test double, so tests exercise the
// actual choice between a voltage command and the motor's holding controller.
template<class Motor>
void applyArmMotor(Motor& arm, const Output& output, State state, bool allowed) {
    // A confirmed endpoint stays released through idle phases and disable,
    // even if a generic pause requests HOLD. Motion clears the coast latch.
    if (output.armCoast && !output.arm && !output.armProfileActive) {
        arm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        arm.brake();
        return;
    }
    const bool passive = !allowed || (!output.armHold && (output.armProfileActive || state == State::fault));
    arm.set_brake_mode(passive ? pros::E_MOTOR_BRAKE_BRAKE : pros::E_MOTOR_BRAKE_HOLD);
    // HOLD needs a velocity stop (brake), not voltage zero (move). Keep using
    // voltage commands for transient profile zeros while the arm is still moving.
    if (!allowed || output.armHold || output.armBrake || state == State::fault) arm.brake();
    else if (output.armProfileActive || output.arm) arm.move(output.arm * cfg::armDirection);
    else arm.brake();
}
}
