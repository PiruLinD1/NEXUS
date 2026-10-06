#pragma once
#include "nexus/api.hpp"
#include "pros/misc.hpp"
#include <optional>

namespace robot {
extern pros::Controller controller;
bool startInitialization(); // Start once from the PROS initialize callback; returns promptly.
bool ready();
bool waitUntilReady(bool autonomousMode); // Returns false on startup failure or competition mode change.
nexus::Chassis& chassis();
void stop();
void resetDriver();
void updateController(); // Call once at the beginning of each driver loop.
bool calibrationActive(); // Skip ordinary bindings while the guided calibration owns control.
using Button = pros::controller_digital_e_t;
using Stick = pros::controller_analog_e_t;
inline constexpr Button R1 = pros::E_CONTROLLER_DIGITAL_R1, R2 = pros::E_CONTROLLER_DIGITAL_R2;
inline constexpr Button L1 = pros::E_CONTROLLER_DIGITAL_L1, L2 = pros::E_CONTROLLER_DIGITAL_L2;
inline constexpr Button X = pros::E_CONTROLLER_DIGITAL_X, Y = pros::E_CONTROLLER_DIGITAL_Y;
inline constexpr Button A = pros::E_CONTROLLER_DIGITAL_A, B = pros::E_CONTROLLER_DIGITAL_B;
inline constexpr Button UP = pros::E_CONTROLLER_DIGITAL_UP, DOWN = pros::E_CONTROLLER_DIGITAL_DOWN;
inline constexpr Button LEFT = pros::E_CONTROLLER_DIGITAL_LEFT, RIGHT = pros::E_CONTROLLER_DIGITAL_RIGHT;
inline constexpr Stick LEFT_Y = pros::E_CONTROLLER_ANALOG_LEFT_Y, LEFT_X = pros::E_CONTROLLER_ANALOG_LEFT_X;
inline constexpr Stick RIGHT_Y = pros::E_CONTROLLER_ANALOG_RIGHT_Y, RIGHT_X = pros::E_CONTROLLER_ANALOG_RIGHT_X;
bool held(Button button); // True while held.
bool pressed(Button button); // True once per press, shared by all bindings in this tick.
int buttons(Button forward, Button reverse); // +127 / -127; both or neither = 0.
void arcade(Stick forward = LEFT_Y, Stick turn = RIGHT_X, nexus::ArcadeCurves curves = {});
void intake(bool enabled);
void pinza(bool closed);
void lift(int power); // -127..127; 0 applies passive BRAKE.
void manualLift(int power); // Cancels the selected lift target while manually moving.
void braccio(int power); // -127..127; 0 holds position, or coasts after an arm endstop contact.
void toggleIntake();
void togglePinza();
bool updateMacros(); // R1/R2 macros, B/DOWN lift levels; X tap toggles, hold 400ms reverses intake.

// Same nexus::Sequence underneath; plain statements for movements and mechanisms.
// Timeout ends only its move; faults/cancellation skip later commands. Scope exit stops all motors.
enum class LiftMove { none, bottom, upStep };
class Auton {
public:
    Auton();
    ~Auton();
    Auton(const Auton&) = delete;
    Auton& operator=(const Auton&) = delete;
    Auton(Auton&&) = delete;
    Auton& operator=(Auton&&) = delete;
    void setPose(double x, double y, double heading);
    void moveToPoint(double x, double y, nexus::MoveOptions options = {});
    void moveToPose(double x, double y, double heading, nexus::MoveOptions options = {}, LiftMove liftMove = LiftMove::none);
    void moveThrough(std::initializer_list<nexus::Waypoint> points, double heading, nexus::MoveOptions options = {}, LiftMove liftMove = LiftMove::none);
    void turnToHeading(double heading, nexus::MoveOptions options = {});
    void wait(std::uint32_t milliseconds);
    void intake(bool enabled);
    void pinza(bool closed);
    void lift(int power);
    void liftToBottom(); // Wait for the calibrated lowest permitted lift position.
    void braccio(int power);
    nexus::MotionResult result() const;
private:
    std::optional<nexus::Sequence> sequence_;
    bool beginLift(LiftMove move);
    bool updateLift();
    void finishLift();
};
} // namespace robot
