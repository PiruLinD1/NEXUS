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
// Percorso di presa e rilascio con lift sincronizzato ai movimenti.
// Il punto di partenza fisico diventa (0, 0), con il muso lungo +Y.
void autonomous() {
    if (!waitUntilReady(true)) return;
    Auton auton;
    auton.setPose(0, 0, 0);

    // I punti guida si attraversano; le prese e i rilasci hanno una fermata precisa.
    // Ogni timeout conclude la sua mossa; all'uscita Auton arresta i motori.
    auton.moveToPose(-500, 380, -90, { .maxSpeed = 3000, .timeout = 10000, .positionTolerance = 50});
    auton.pinza(false);
    auton.moveToPose(0, 380, -90, {.reverse = true, .maxSpeed = 3000, .timeout = 10000, .positionTolerance = 50});
    auton.liftToBottom();
    auton.turnToHeading(-45);
    auton.moveToPose(-600, 780, -45, {.maxSpeed = 3000, .timeout = 10000, .positionTolerance = 50});
    auton.pinza(true);
    auton.moveToPose(-800, 650, 180, {.maxSpeed = 3000, .timeout = 10000, .positionTolerance = 50}, LiftMove::upStep);
    auton.pinza(false);
    auton.moveToPose(-800, 1000, 180, {.reverse = true, .maxSpeed = 3000, .timeout = 10000, .positionTolerance = 50});
    auton.turnToHeading(-135);
    auton.moveToPose(-1250, 520, -135, {.maxSpeed = 3000, .timeout = 10000, .positionTolerance = 50}, LiftMove::bottom);
    auton.pinza(true);


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
