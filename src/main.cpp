#include "main.h"
#include "robot.hpp"

using namespace robot;

// CURVE JOYSTICK: risposta alla posizione dello stick, da 0.0 a 1.0.
// 0.0 = lineare; 0.55 = risposta attuale; 1.0 = piu dolce vicino al centro.
constexpr nexus::ArcadeCurves joystickCurves{
    .forward = 0.55, // Avanti/indietro (LEFT_Y).
    .turn = 0.55     // Sterzo (RIGHT_X).
};

// AUTONOMOUS: mm, degrees, milliseconds.
// Test del planner e del controller: 600 mm a destra e 600 mm avanti.
// Il punto di partenza fisico diventa (0, 0), con il muso lungo +Y.
void autonomous() {
    if (!waitUntilReady(true)) return;
    Auton auton;
    auton.setPose(0, 0, 0);

    // Percorso a tappe: ogni posa impone anche il proprio heading finale.
    // Ogni timeout conclude la sua mossa; all'uscita Auton arresta i motori.
    auton.moveToPose(600, 600, 0,{.maxSpeed = 3000, .timeout = 10000,
                                .positionTolerance = 100});
    auton.moveToPose(600, 1600, 0,  {.maxSpeed = 3000, .timeout = 10000,
                                .positionTolerance = 100});
    auton.moveToPose(-600, 1600, 180, {.maxSpeed = 3000, .timeout = 10000,
                                .positionTolerance = 100});
    auton.moveToPose(-900, 1200, 180, {.maxSpeed = 3000, .timeout = 10000,
                                .positionTolerance = 50});
    auton.moveToPose(0, 0, 0, {.maxSpeed = 3000, .timeout = 18000,
                                .positionTolerance = 10});
}

// DRIVER: button assignments.
void opcontrol() {
    if (!waitUntilReady(false)) return;
    resetDriver();
    std::uint32_t wake = pros::millis();
    while (true) {
        updateController();
        const bool macroOwnsMechanisms = updateMacros();
        if (calibrationActive()) {
            pros::Task::delay_until(&wake, 10);
            continue;
        }

        arcade(LEFT_Y, RIGHT_X, joystickCurves);
        if (!macroOwnsMechanisms) {
            if (held(RIGHT) || held(Y)) manualLift(buttons(RIGHT, Y));
        }

        pros::Task::delay_until(&wake, 10);
    }
}

// Startup and competition lifecycle.
void initialize() {
    pros::lcd::initialize();
    robot::startInitialization();
}

// Before readiness, the boot-only preMatch owns the mechanisms.
void disabled() { if (robot::ready()) robot::stop(); }
void competition_initialize() {}
