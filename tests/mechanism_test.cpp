#include "robot/mechanism.hpp"
#include "robot/arm_motor.hpp"
#include "robot/mechanism_driver.hpp"
#include "robot/mechanism_input.hpp"
#include <array>
#include <cstring>
#include <cstdlib>
#include <iostream>
using namespace robot::mechanism;
constexpr double liftLow = -cfg::liftLowerRotations;
constexpr double liftPreMatch = liftLow - cfg::liftEndstopExtraLowerRotations;
constexpr double liftPickup = liftLow + cfg::pickupLiftRotations;
constexpr double liftLevel1 = liftPickup + cfg::driverLiftStepRotations;
constexpr double liftLevel2 = liftLevel1 + cfg::driverLiftStepRotations;
void require(bool ok, const char* message) {
    if (!ok) { std::cerr << message << '\n'; std::exit(1); }
}
struct ArmMotor {
    pros::motor_brake_mode_e_t mode = pros::E_MOTOR_BRAKE_HOLD;
    int power = 0;
    bool braking = false;
    void set_brake_mode(pros::motor_brake_mode_e_t value) { mode = value; }
    // Like PROS, voltage zero does not engage position holding.
    void move(int value) { power = value; braking = false; }
    void brake() { power = 0; braking = true; }
    bool holding() const { return braking && mode == pros::E_MOTOR_BRAKE_HOLD; }
    bool coasting() const { return power == 0 && mode == pros::E_MOTOR_BRAKE_COAST; }
};
struct Rig {
    Controller controller;
    Input in{.forwardPodDegrees = 0};
    bool autoDepartAfterRelease = true; // Existing full-cycle scenarios include driving away after opening.
    ArmMotor motor;
    double arm = 70, lift = -2, maxArm = 195, liftOffset = 0;
    double liftCeiling = 1e9, liftFloor = -1e9, liftSpeedScale = 1, liftExtraTop = 0;
    double forcedLiftCurrent = -1, forcedLiftPeakCurrent = -1;
    double armLoadPower = 0, armFrictionPower = 0;
    int encoderPolarity = 1;
    DriverResult driver;
    void driverAt(std::uint32_t now, bool grabPressed = false, bool releasePressed = false,
                  int liftStep = 0, bool clampTogglePressed = false,
                  bool intakeHeld = false, bool intakePressed = false) {
        in.now = now; in.arm = arm * encoderPolarity; in.lift = lift;
        driver = driverTick(controller, in, grabPressed, releasePressed, controller.output.closed,
                            liftStep, clampTogglePressed, intakeHeld, intakePressed);
        applyArmMotor(motor, controller.output, controller.state, true);
    }
    void step(bool grabPressed = false, bool releasePressed = false, int liftStep = 0,
              bool clampTogglePressed = false) {
        if (autoDepartAfterRelease && controller.state == State::releaseWait)
            in.forwardPodDegrees += 7.2; // Simulated forward pod: two turns per second.
        driverAt(in.now + 10, grabPressed, releasePressed, liftStep, clampTogglePressed);
        const auto out = controller.output;
        const double effort = motor.power * cfg::armDirection - armLoadPower;
        const double armStep = motor.holding() ? 0
            : std::copysign(std::max(0.0, std::abs(effort) - armFrictionPower), effort) * 0.02;
        const double newArm = std::clamp(arm + armStep, 0.0, maxArm);
        const double newLift = std::clamp(lift + out.lift * 0.0006 * liftSpeedScale,
            std::max(liftOffset - cfg::liftLowerRotations - cfg::liftEndstopExtraLowerRotations - 0.2, liftFloor),
            std::min(liftOffset + liftExtraTop, liftCeiling));
        in.armVelocity = (newArm - arm) / 0.01 / 6;
        in.liftVelocity = (newLift - lift) / 0.01 * 60;
        in.armCurrent = out.arm && newArm == arm ? 1600 : 100;
        in.liftCurrent = forcedLiftCurrent >= 0 ? forcedLiftCurrent : out.lift && newLift == lift ? 1600 : 100;
        in.liftPeakCurrent = forcedLiftPeakCurrent >= 0 ? forcedLiftPeakCurrent : in.liftCurrent;
        arm = newArm; lift = newLift;
    }
    void until(State target, int limit = 60000) {
        for (int i = 0; controller.state != target && i < limit; i += 10) step();
        if (controller.state != target)
            std::cerr << "State " << static_cast<int>(controller.state) << ", arm " << arm
                      << ", expected " << static_cast<int>(target) << ", lift " << lift
                      << ", target " << controller.armTargetDegrees << ": " << controller.error
                      << " / " << controller.notice << '\n';
        require(controller.state == target, "sequence did not reach expected state");
    }
    void start() {
        in.arm = arm * encoderPolarity; in.lift = lift;
        controller.start(in);
    }
    void manuallyPositionArm(double position) {
        // A real manual movement clears the last confirmed endpoint before
        // applying power; synthetic relocations must do the same.
        controller.output.armCoast = false;
        arm = position;
    }
    void preMatch() {
        start(); until(State::prepared);
        require(controller.calibrated && controller.output.closed, "preMatch must leave the clamp closed and calibrate");
        require(arm == maxArm && std::abs(lift - (liftOffset + liftPreMatch)) < 0.03, "preMatch extra-low position at endstop 2");
        require(!controller.output.armCoast && motor.holding(), "completed preMatch supports the second arm stop in HOLD");
        require(!controller.output.intake, "preMatch does not start intake at endstop 2");
    }
    void setup() {
        // Existing pickup/return scenarios start at the normal front low pose.
        // Reach it through the production R2 path after the new startup pose.
        preMatch();
        const bool depart = autoDepartAfterRelease;
        autoDepartAfterRelease = true;
        step(false, true); until(State::prepared);
        autoDepartAfterRelease = depart;
        require(std::abs(arm) < 0.1 && std::abs(lift - (liftOffset + liftLow)) < 0.03
                && !controller.output.closed, "R2 sets up the normal pickup position after preMatch");
    }
    void height(int direction, double target) {
        step(false, false, direction);
        unsigned settled = 0;
        for (unsigned i = 0; i < 1000 && settled < 25; ++i) {
            step();
            settled = std::abs(lift - target) <= cfg::liftToleranceRot && !controller.output.lift
                ? settled + 1 : 0;
        }
        if (settled < 25)
            std::cerr << "Lift " << lift << " -> " << target << ": " << controller.notice << '\n';
        require(settled == 25, "height button must reach and stop at the selected level");
    }
};
struct RotationReadout {
    std::int32_t position = 91000, reversed = cfg::encoderPort < 0;
    std::int32_t get_position() const { return position; }
    std::int32_t get_reversed() const { return reversed; }
};
struct ArmReadout {
    std::int32_t current = 200;
    double velocity = 0;
    std::int32_t get_current_draw() const { return current; }
    double get_actual_velocity() const { return velocity; }
};
struct LiftReadout {
    std::array<double, 2> positions{3.0, 3.0}, velocities{0, 0};
    std::array<std::int32_t, 2> currents{200, 200};
    double get_position(std::uint8_t i) const { return positions[i]; }
    double get_actual_velocity(std::uint8_t i) const { return velocities[i]; }
    std::int32_t get_current_draw(std::uint8_t i) const { return currents[i]; }
};
int main() {
    require(cfg::intakeReverseHoldMs == 400 && cfg::liftEndstopExtraLowerRotations == 0.68,
            "X requires 400ms to reverse and the extra-low level is 0.68 rotations");
    for (double origin : {-5.0, 0.0, 7.0}) for (int polarity : {-1, 1}) {
        Rig startup; startup.liftOffset = origin; startup.lift += origin;
        startup.encoderPolarity = polarity; startup.preMatch();
        const std::array<double, 2> references{startup.controller.liftMeasurements[0], startup.controller.liftMeasurements[1]};
        const double bottom = startup.lift;
        for (int i = 0; i < 30; ++i) startup.step();
        startup.step(false, false, -1);
        require(startup.lift == bottom && !startup.controller.output.lift
                && startup.arm == startup.maxArm && startup.motor.holding()
                && startup.controller.output.closed && !startup.controller.output.intake,
                "READY and DOWN retain the closed extra-low startup pose without intake or further descent");
        startup.height(1, origin + liftPreMatch + cfg::driverLiftStepRotations);
        startup.height(-1, origin + liftLow);
        startup.height(-1, origin + liftPreMatch);
        startup.step(true);
        startup.until(State::pickupLift);
        while (startup.controller.state == State::pickupLift) {
            startup.step();
            require(startup.arm == startup.maxArm && startup.motor.holding()
                    && startup.controller.output.closed,
                    "first R1 raises from the extra-low startup pose before moving away from endstop 2");
        }
        require(std::abs(startup.lift - (origin + liftPickup)) < 0.03,
                "startup extra travel does not alter the normal pickup clearance");
        startup.until(State::carrying);
        require(std::abs(startup.arm - cfg::armPickupDeg) <= cfg::armToleranceDeg
                && !startup.controller.endstopSelected,
                "first R1 after preMatch selects 150 degrees from endstop 2");
        require(startup.controller.liftMeasurements[0] == references[0]
                && startup.controller.liftMeasurements[1] == references[1],
                "startup pose and its height adjustments preserve both calibrated lift references");

        Rig startupRelease; startupRelease.liftOffset = origin; startupRelease.lift += origin;
        startupRelease.encoderPolarity = polarity; startupRelease.preMatch();
        startupRelease.step(false, true);
        require(startupRelease.controller.state == State::releaseWait
                && !startupRelease.controller.output.closed && startupRelease.motor.holding()
                && startupRelease.arm == startupRelease.maxArm,
                "R2 directly after preMatch opens at endstop 2 without a move to 90 degrees");
        startupRelease.until(State::returnLiftClearance);
        while (startupRelease.controller.state == State::returnLiftClearance) {
            startupRelease.step();
            require(startupRelease.arm == startupRelease.maxArm && startupRelease.motor.holding(),
                    "R2 raises the startup lift before rotating the arm");
        }
        startupRelease.until(State::prepared);
        require(startupRelease.arm == 0 && std::abs(startupRelease.lift - (origin + liftLow)) < 0.03
                && !startupRelease.controller.output.closed && startupRelease.controller.output.intake == 127,
                "R2 still returns to the normal low pickup pose and starts intake there");
    }
    for (double origin : {-5.0, 0.0, 7.0}) {
        Rig intakeRig; intakeRig.liftOffset = origin; intakeRig.lift += origin; intakeRig.setup();
        require(intakeRig.controller.output.intake == 127,
                "intake starts automatically at the calibrated low pose, independent of encoder origin");
        const auto pressedAt = intakeRig.in.now + 10;
        intakeRig.driverAt(pressedAt, false, false, 0, false, true, true);
        intakeRig.driverAt(pressedAt + 399, false, false, 0, false, true);
        require(intakeRig.controller.output.intake == 127,
                "holding X for less than 400ms preserves the forward direction");
        intakeRig.driverAt(pressedAt + 400, false, false, 0, false, true);
        require(intakeRig.controller.output.intake == -127,
                "exactly 400ms of holding X starts reverse");
        intakeRig.driverAt(pressedAt + 2400, false, false, 0, false, true);
        require(intakeRig.controller.output.intake == -127,
                "reverse continues for the entire hold without a fixed duration");
        intakeRig.driverAt(pressedAt + 2401);
        require(intakeRig.controller.output.intake == 127,
                "releasing a long X press resumes forward in the same driver tick");
        intakeRig.driverAt(pressedAt + 2500, false, false, 0, false, true, true);
        intakeRig.driverAt(pressedAt + 2599);
        require(!intakeRig.controller.output.intake, "a short X press toggles forward off on release");
        for (int i = 0; i < 50; ++i) intakeRig.step();
        require(!intakeRig.controller.output.intake,
                "remaining in the low pose does not undo a manual intake stop");
        const auto fromOff = intakeRig.in.now + 10;
        intakeRig.driverAt(fromOff, false, false, 0, false, true, true);
        intakeRig.driverAt(fromOff + 400, false, false, 0, false, true);
        require(intakeRig.controller.output.intake == -127, "long X also reverses an initially stopped intake");
        intakeRig.driverAt(fromOff + 401);
        require(intakeRig.controller.output.intake == 127, "release from reverse enables forward even if initially off");
        intakeRig.step(true);
        require(!intakeRig.controller.output.intake, "R1 still stops intake immediately before the clamp pause");
        intakeRig.until(State::carrying);
        require(!intakeRig.controller.output.intake, "intake stays stopped throughout ordinary pickup");
        const auto duringMacro = intakeRig.in.now + 10;
        intakeRig.driverAt(duringMacro, false, false, 0, false, true, true);
        intakeRig.driverAt(duringMacro + 400, false, false, 0, false, true);
        require(intakeRig.controller.output.intake == -127, "X can reverse intake while a driver macro owns mechanisms");
        intakeRig.driverAt(duringMacro + 401); intakeRig.step();
        require(intakeRig.controller.output.intake == 127, "forward after X release persists during a macro");
        intakeRig.step(false, true);
        require(!intakeRig.controller.output.intake, "a new R2 stops the manually resumed intake");
        intakeRig.until(State::returnLiftBottom);
        while (intakeRig.controller.state == State::returnLiftBottom) {
            require(!intakeRig.controller.output.intake, "intake waits for the final descent to finish");
            intakeRig.step();
        }
        require(intakeRig.controller.state == State::prepared && intakeRig.controller.output.intake == 127,
                "the complete R2 return restarts intake automatically in the arrival tick");
        intakeRig.controller.cancel(); intakeRig.step();
        require(!intakeRig.controller.output.intake, "cancel stops intake even at the low pose");
        intakeRig.controller.resume(intakeRig.in.now); intakeRig.step();
        require(intakeRig.controller.output.intake == 127, "driver resume at low pose restores automatic intake");
        const auto beforeDropout = intakeRig.in.now + 10;
        intakeRig.driverAt(beforeDropout, false, false, 0, false, true, true);
        intakeRig.driverAt(beforeDropout + 400, false, false, 0, false, true);
        intakeRig.in.valid = false;
        intakeRig.driverAt(beforeDropout + 410, false, false, 0, false, true);
        require(!intakeRig.controller.output.intake, "invalid mechanism readings stop reverse immediately");
        intakeRig.in.valid = true;
        intakeRig.driverAt(beforeDropout + 420, false, false, 0, false, true);
        require(intakeRig.controller.output.intake == 127,
                "recovery with X still held does not reuse the interrupted reverse timer");
        intakeRig.driverAt(beforeDropout + 430);
    }
    Rig simultaneousIntake; simultaneousIntake.setup();
    const auto simultaneousAt = simultaneousIntake.in.now + 10;
    simultaneousIntake.driverAt(simultaneousAt, true, false, 0, false, true, true);
    simultaneousIntake.driverAt(simultaneousAt + 299, false, false, 0, false, true);
    require(!simultaneousIntake.controller.output.intake,
            "simultaneous R1 and X cannot drive intake during the clamp pause");
    simultaneousIntake.driverAt(simultaneousAt + 300, false, false, 0, false, true);
    simultaneousIntake.driverAt(simultaneousAt + 400, false, false, 0, false, true);
    require(simultaneousIntake.controller.output.intake == -127,
            "a continuing X hold reverses after 400ms even when its press started in the R1 pause");
    simultaneousIntake.controller.suspend(); simultaneousIntake.step();
    require(!simultaneousIntake.controller.output.intake, "suspension cancels an active reverse");
    simultaneousIntake.driverAt(simultaneousIntake.in.now + 500, false, false, 0, false, true);
    require(!simultaneousIntake.controller.output.intake, "a held X after suspension cannot reuse the old timer");
    require(cfg::armPickupDeg == 150, "R1 uses the requested 150 degrees");
    require(cfg::pickupClampPauseMs >= 300, "R1 must allow at least 300ms for the clamp to close");
    require(cfg::encoderDegreesPerArmDegree == 7 && cfg::armClearanceEncoderDegrees == 1008
            && cfg::armClearanceDeg == 144, "1008 encoder degrees maps to 144 arm degrees");
    ArmMotor motor;
    Output command;
    command.armProfileActive = true;
    applyArmMotor(motor, command, State::pickupArm, true);
    require(!motor.braking && motor.power == 0, "a profile zero during acceleration remains a voltage zero");
    command.armHold = true;
    applyArmMotor(motor, command, State::pickupArm, true);
    require(motor.holding(), "settling must call brake with HOLD, not move(0)");
    applyArmMotor(motor, command, State::carrying, false);
    require(motor.braking && !motor.holding(), "disabled arm must not command active hold");
    command.armProfileActive = false; command.armCoast = true;
    for (bool allowed : {false, true}) {
        applyArmMotor(motor, command, State::carryingEndstop, allowed);
        require(motor.coasting() && !motor.holding(),
                "confirmed endstop remains COAST even with stale HOLD or disabled output");
    }
    for (bool upper : {false, true}) for (int direction : {-1, 1}) {
        Rig departure; departure.autoDepartAfterRelease = false; departure.setup();
        departure.step(true); departure.until(State::carrying);
        if (upper) { departure.step(true); departure.until(State::carryingEndstop); }
        departure.in.forwardPodDegrees = 1234; // Earlier travel does not count toward this R2.
        departure.step(false, true); departure.until(State::releaseWait);
        const double releasedArm = departure.arm, releasedLift = departure.lift;
        for (unsigned elapsed = 0; elapsed < 2 * cfg::motionTimeoutMs; elapsed += 10) {
            departure.step();
            require(departure.controller.state == State::releaseWait && departure.driver.owns
                    && !departure.controller.output.closed && !departure.controller.output.arm
                    && !departure.controller.output.lift && departure.arm == releasedArm
                    && departure.lift == releasedLift,
                    "R2 must keep the clamp open and both mechanisms still indefinitely until driving away");
        }
        require(upper ? departure.motor.coasting() : departure.motor.holding(),
                "waiting for departure preserves upper-endstop COAST or intermediate HOLD");
        for (int oscillation = 0; oscillation < 10; ++oscillation) {
            departure.in.forwardPodDegrees = 1234 + (oscillation % 2 ? -179.99 : 179.99);
            departure.step();
            require(departure.controller.state == State::releaseWait,
                    "back-and-forth travel must not accumulate into a false half-turn departure");
        }
        departure.in.forwardPodDegrees = 1234 + direction * 179.99;
        departure.step(false, true);
        require(!departure.driver.releaseAccepted && departure.controller.state == State::releaseWait,
                "repeated R2 during waiting is ignored and does not reset the departure origin");
        departure.in.forwardPodDegrees = 1234 + direction * 180;
        departure.step();
        require(departure.controller.state == State::returnArmClearance,
                "exactly half a forward or backward wheel turn after R2 authorizes return at either release pose");
        departure.until(State::prepared);
        departure.step(true); departure.until(State::carrying);
        departure.step(false, true); departure.until(State::releaseWait);
        departure.driverAt(departure.in.now + cfg::releasePauseMs);
        require(departure.controller.state == State::releaseWait,
                "each new R2 requires another half wheel turn instead of reusing previous departure");
    }
    Rig earlyDeparture; earlyDeparture.autoDepartAfterRelease = false; earlyDeparture.setup();
    earlyDeparture.step(true); earlyDeparture.step(false, true); // R2 queued during clamp/lift pause.
    earlyDeparture.in.forwardPodDegrees = -180;
    earlyDeparture.until(State::releaseWait);
    earlyDeparture.step();
    require(earlyDeparture.controller.state == State::releaseWait,
            "half a wheel turn does not bypass the minimum clamp-open pause");
    earlyDeparture.driverAt(earlyDeparture.in.now + cfg::releasePauseMs);
    require(earlyDeparture.controller.state == State::returnArmClearance,
            "queued R2 records the pod at the button press, before lift or arm movement finishes");

    Rig podDropout; podDropout.autoDepartAfterRelease = false; podDropout.setup();
    podDropout.step(true); podDropout.until(State::carrying);
    podDropout.in.forwardPodDegrees = std::numeric_limits<double>::quiet_NaN();
    podDropout.step(false, true); podDropout.until(State::releaseWait);
    podDropout.driverAt(podDropout.in.now + 20000);
    require(podDropout.controller.state == State::releaseWait && podDropout.motor.holding(),
            "an unavailable pod at R2 permits release but never automatic return");
    podDropout.in.forwardPodDegrees = 5000; podDropout.step();
    require(podDropout.controller.state == State::releaseWait,
            "first valid pod reading after R2 establishes an origin without authorizing return");
    podDropout.in.forwardPodDegrees = std::numeric_limits<double>::infinity(); podDropout.step();
    podDropout.in.forwardPodDegrees = -5000; podDropout.step();
    require(podDropout.controller.state == State::releaseWait,
            "a pod reset after a dropout cannot be mistaken for wheel travel");
    podDropout.in.forwardPodDegrees = -4820; podDropout.step();
    require(podDropout.controller.state == State::returnArmClearance,
            "a fresh half turn after pod recovery allows the waiting return");

    Rig homingCoast; homingCoast.start();
    for (State afterContact : {State::armBackoff0, State::armTop1, State::armBackoffTop}) {
        homingCoast.until(afterContact);
        require(homingCoast.controller.output.armCoast && homingCoast.motor.coasting(),
                "every accepted lower and upper preMatch contact immediately removes motor braking");
        homingCoast.step();
        require(!homingCoast.controller.output.armCoast
                && (homingCoast.controller.output.arm || homingCoast.controller.output.armProfileActive),
                "the next homing backoff or positioning request clears endpoint COAST");
    }
    for (State supported : {State::liftTop1, State::prepared}) {
        homingCoast.until(supported);
        require(!homingCoast.controller.output.armCoast && homingCoast.motor.holding(),
                "the final preMatch endstop contact stays supported during descent and READY");
    }
    for (bool upper : {false, true}) for (bool disabled : {false, true}) {
        Rig endpoint; endpoint.setup();
        if (upper) {
            endpoint.step(true); endpoint.step(true); endpoint.until(State::carryingEndstop);
        }
        for (int i = 0; i < 30; ++i) {
            endpoint.step();
            require(endpoint.controller.output.armCoast && endpoint.motor.coasting(),
                    "both confirmed arm stops remain in COAST over repeated idle updates");
        }
        if (disabled) endpoint.controller.suspend();
        else endpoint.controller.cancel();
        applyArmMotor(endpoint.motor, endpoint.controller.output, endpoint.controller.state, !disabled);
        require(endpoint.motor.coasting(), "cancelling or disabling at either arm stop preserves COAST");
        if (disabled) endpoint.controller.resume(endpoint.in.now);
        for (int i = 0; i < 30; ++i) {
            endpoint.step();
            require(endpoint.motor.coasting(), "driver reentry cannot reapply braking at a confirmed endpoint");
        }
        endpoint.step(true);
        require(endpoint.motor.coasting(), "a new clamp pause retains COAST until the arm is commanded away");
        endpoint.until(State::carrying);
        require(!endpoint.controller.output.armCoast && endpoint.motor.holding(),
                "departing either endpoint restores HOLD at the intermediate pickup target");
    }
    Rig lowLiftAtTop; lowLiftAtTop.setup();
    lowLiftAtTop.step(true); lowLiftAtTop.step(true); lowLiftAtTop.until(State::carryingEndstop);
    lowLiftAtTop.height(-1, liftLow);
    lowLiftAtTop.height(-1, liftLow - cfg::liftEndstopExtraLowerRotations);
    const double lowestDriverLift = lowLiftAtTop.lift;
    lowLiftAtTop.step(false, false, -1);
    require(!lowLiftAtTop.controller.output.lift && lowLiftAtTop.lift == lowestDriverLift,
            "another DOWN press cannot descend beyond the extra-low limit");
    lowLiftAtTop.step(false, true);
    require(lowLiftAtTop.controller.state == State::releaseWait && lowLiftAtTop.motor.coasting(),
            "release at the upper endpoint opens in place while retaining COAST");
    lowLiftAtTop.until(State::returnLiftClearance);
    while (lowLiftAtTop.controller.state == State::returnLiftClearance) {
        lowLiftAtTop.step();
        require(lowLiftAtTop.motor.coasting(), "raising the low lift keeps the arm in COAST at its upper endpoint");
    }
    lowLiftAtTop.until(State::prepared);
    require(lowLiftAtTop.motor.coasting(), "the full return ends in COAST at the lower endpoint");
    for (double origin : {-5.0, 0.0, 7.0}) {
        Rig prematch; prematch.liftOffset = origin; prematch.lift += origin;
        prematch.liftSpeedScale = 0.2; // A slow reset must finish before any arm movement.
        for (unsigned run = 0; run < 2; ++run) {
            const double initialArm = prematch.arm, initialLift = prematch.lift;
            const auto started = prematch.in.now;
            prematch.start();
            require(prematch.controller.state == State::preLiftStart && !prematch.controller.calibrated
                    && prematch.controller.output.lift > 0 && !prematch.controller.output.arm,
                    "initial and repeated preMatch start lift reset immediately");
            bool liftBackoffSeen = false;
            for (unsigned i = 0; i < 2000 && prematch.controller.state == State::preLiftStart; ++i) {
                prematch.step();
                if (prematch.controller.output.lift < 0) liftBackoffSeen = true;
                if (prematch.controller.state == State::preLiftStart)
                    require(prematch.arm == initialArm && !prematch.controller.output.arm && prematch.motor.holding(),
                            "the arm stays held throughout both lift contacts and the calibration backoff");
            }
            require(prematch.lift > initialLift && liftBackoffSeen && prematch.in.now - started > 1000,
                    "lift reset completes both contacts instead of handing off after one second");
            require(prematch.controller.state == State::armZero1 && prematch.controller.output.arm < 0
                    && !prematch.controller.output.lift && std::abs(prematch.lift - origin) < 0.03
                    && std::abs(prematch.controller.liftMeasurements[0] - origin) < 0.03
                    && std::abs(prematch.controller.liftMeasurements[1] - origin) < 0.03,
                    "arm reset begins only with the lift fully raised and both lift references verified");
            while (prematch.controller.state != State::liftTop1 && prematch.controller.state != State::fault) {
                prematch.step();
                require(!prematch.controller.output.lift && std::abs(prematch.lift - origin) < 0.03,
                        "the lift stays fully raised throughout arm reset");
            }
            prematch.until(State::prepared);
            require(prematch.controller.calibrated && std::abs(prematch.lift - (origin + liftPreMatch)) < 0.03
                    && std::abs(prematch.controller.liftMeasurements[0] - origin) < 0.03
                    && std::abs(prematch.controller.liftMeasurements[1] - origin) < 0.03
                    && prematch.arm == prematch.maxArm && prematch.controller.output.closed,
                    "sequential resets retain both lift measurements and complete the closed extra-low pose");
        }
    }
    for (bool contact : {true, false}) {
        Rig failedReset;
        failedReset.liftSpeedScale = 0;
        failedReset.forcedLiftCurrent = contact ? 1200 : 100;
        failedReset.start(); failedReset.until(State::fault);
        require(!failedReset.controller.calibrated
                && !failedReset.controller.output.arm && !failedReset.controller.output.lift
                && failedReset.arm == 70 && !failedReset.controller.encoderDirectionKnown,
                "a failed lift reset stops startup without ever moving the arm");
        require(std::strstr(failedReset.controller.error, contact ? "Lift bloccato" : "timeout"),
                "lift reset retains backoff blockage detection and per-phase deadlines");
    }
    for (std::uint32_t started : {0u, 0xfffffe00u}) {
        Controller delay;
        Input sample{.now = started, .arm = 70, .lift = -2};
        delay.start(sample);
        for (unsigned elapsed : {0u, 1u, 999u, 1000u, 2000u}) {
            sample.now = started + elapsed; delay.tick(sample);
            require(delay.state == State::preLiftStart && !delay.output.arm && delay.output.armHold
                    && delay.output.lift > 0, "elapsed time cannot start the arm before lift contact, including timer wraparound");
        }
        delay.cancel(); sample.now += 100; delay.tick(sample);
        require(!delay.output.arm && !delay.output.lift, "cancel stops lift reset and leaves the waiting arm still");
        delay.start(sample); sample.now += 999; delay.tick(sample);
        require(!delay.output.arm && delay.output.lift > 0, "restart waits for a fresh complete lift reset");
    }
    Rig slowReset; slowReset.lift = -1.9; slowReset.liftSpeedScale = 0.065; slowReset.start();
    slowReset.until(State::armZero1);
    require(slowReset.in.now > cfg::motionTimeoutMs && !slowReset.controller.output.lift
            && std::abs(slowReset.lift) < 0.03,
            "a multi-phase lift reset may exceed one motion timeout in total while each phase has its own deadline");
    slowReset.liftSpeedScale = 1;
    slowReset.until(State::prepared);
    require(slowReset.controller.calibrated, "arm reset gets a fresh timeout after a slow complete lift reset");
    for (bool raised : {false, true}) {
        Rig clampWait; clampWait.setup();
        if (raised) { clampWait.lift = liftPickup; clampWait.manuallyPositionArm(40); }
        const auto started = clampWait.in.now;
        clampWait.driverAt(started, true);
        require(clampWait.driver.owns && clampWait.controller.state == State::pickupWait
                && clampWait.controller.output.closed
                && (raised ? clampWait.motor.holding() : clampWait.motor.coasting())
                && !clampWait.controller.output.arm && !clampWait.controller.output.lift
                && !clampWait.controller.output.intake,
                "the R1 driver edge closes the clamp, retaining COAST at the lower stop and HOLD elsewhere");
        for (unsigned elapsed : {1u, 150u, 299u}) {
            clampWait.driverAt(started + elapsed, false, false, elapsed == 150 ? 1 : -1);
            require(clampWait.controller.state == State::pickupWait && clampWait.driver.owns
                    && clampWait.controller.output.closed
                    && (raised ? clampWait.motor.holding() : clampWait.motor.coasting())
                    && !clampWait.controller.output.arm && !clampWait.controller.output.lift
                    && !clampWait.controller.output.intake,
                    "no lift, arm or intake command may start before 300ms, including height requests");
        }
        clampWait.driverAt(started + cfg::pickupClampPauseMs);
        require(clampWait.controller.state == (raised ? State::pickupArm : State::pickupLift),
                "after the clamp pause the current lift height selects the arm move or pickup interlock");
        clampWait.until(State::carrying);
    }
    Rig restartedWait; restartedWait.setup();
    const auto firstPress = restartedWait.in.now;
    restartedWait.driverAt(firstPress, true);
    restartedWait.driverAt(firstPress + 299, true);
    require(restartedWait.controller.endstopSelected && restartedWait.controller.state == State::pickupWait,
            "a repeated R1 toggles destination while restarting the clamp pause");
    for (unsigned elapsed : {300u, 598u}) {
        restartedWait.driverAt(firstPress + elapsed);
        require(restartedWait.controller.state == State::pickupWait && restartedWait.motor.coasting()
                && !restartedWait.controller.output.arm && !restartedWait.controller.output.lift,
                "the earlier R1 deadline cannot start motion before 300ms after the latest press");
    }
    restartedWait.driverAt(firstPress + 299 + cfg::pickupClampPauseMs);
    require(restartedWait.controller.state == State::pickupLift,
            "a repeated R1 starts its selected destination only after the renewed pause");
    restartedWait.until(State::carryingEndstop);

    for (bool atTop : {false, true}) {
        Rig queuedAtHeight; queuedAtHeight.setup();
        queuedAtHeight.lift = liftPickup;
        queuedAtHeight.manuallyPositionArm(atTop ? queuedAtHeight.maxArm : 120);
        const auto started = queuedAtHeight.in.now;
        queuedAtHeight.driverAt(started, true);
        queuedAtHeight.driverAt(started + 50, false, true);
        require(queuedAtHeight.driver.releaseAccepted && queuedAtHeight.controller.releasePending(),
                "R2 during the clamp pause is accepted and retained at an already raised lift height");
        queuedAtHeight.driverAt(started + 299);
        require(queuedAtHeight.controller.state == State::pickupWait && queuedAtHeight.controller.output.closed
                && queuedAtHeight.motor.holding() && !queuedAtHeight.controller.output.arm
                && !queuedAtHeight.controller.output.lift && !queuedAtHeight.controller.output.intake,
                "queued R2 cannot open the clamp or start any movement before the R1 pause ends");
        queuedAtHeight.driverAt(started + cfg::pickupClampPauseMs);
        require(queuedAtHeight.controller.state == (atTop ? State::releaseWait : State::releaseArm90),
                "queued release starts immediately after the pause when the lift is already raised");
        require(queuedAtHeight.controller.output.closed == !atTop,
                "queued release preserves the usual opening position, including release at the upper endstop");
        queuedAtHeight.until(State::releaseWait);
    }
    for (bool disabled : {false, true}) {
        Rig cancelledWait; cancelledWait.setup(); cancelledWait.step(true);
        cancelledWait.step(false, true);
        if (disabled) cancelledWait.controller.suspend();
        else cancelledWait.controller.cancel();
        cancelledWait.in.now += cfg::pickupClampPauseMs + 1000;
        if (disabled) cancelledWait.controller.resume(cancelledWait.in.now);
        cancelledWait.step();
        require(!cancelledWait.driver.owns && !cancelledWait.controller.releasePending()
                && !cancelledWait.controller.output.arm && !cancelledWait.controller.output.lift
                && cancelledWait.motor.coasting(),
                "cancel or disabled cancels both the clamp wait and queued R2 without delayed movement");
        require(cancelledWait.controller.output.intake == (disabled ? 127 : 0),
                "only driver resume at the low pose restarts intake after cancellation");
        cancelledWait.step(true);
        require(cancelledWait.controller.state == State::pickupWait,
                "a fresh R1 after cancellation starts a fresh clamp pause");
    }
    Rig movingHeight; movingHeight.setup(); movingHeight.step(true); movingHeight.until(State::carrying);
    movingHeight.step(false, false, 1);
    require(movingHeight.controller.output.lift > 0, "the manual lift move is active before R1 preemption");
    movingHeight.step(true);
    const auto heightPauseStarted = movingHeight.in.now;
    const double interruptedLift = movingHeight.lift;
    require(movingHeight.controller.state == State::pickupWait && movingHeight.motor.holding()
            && !movingHeight.controller.output.arm && !movingHeight.controller.output.lift,
            "R1 stops an active manual lift move immediately while the clamp closes");
    movingHeight.driverAt(heightPauseStarted + 299, false, false, 1);
    require(!movingHeight.controller.output.lift && movingHeight.lift == interruptedLift
            && movingHeight.controller.state == State::pickupWait,
            "B cannot bypass the clamp pause after an active height move");
    movingHeight.driverAt(heightPauseStarted + cfg::pickupClampPauseMs);
    require(movingHeight.controller.state == State::pickupEndstop && movingHeight.controller.output.lift > 0,
            "the preserved manual height request can resume only after the full clamp pause");
    movingHeight.until(State::carryingEndstop);
    for (int i = 0; i < 1000; ++i) movingHeight.step();
    require(movingHeight.controller.state == State::carryingEndstop
            && std::abs(movingHeight.lift - liftLevel1) <= cfg::liftToleranceRot
            && !movingHeight.controller.output.lift,
            "the paused lift reaches its original height without extra steps, false stall or timeout");

    Rig clampToggle; clampToggle.setup();
    const double toggleArm = clampToggle.arm, toggleLift = clampToggle.lift;
    for (bool closed : {true, false}) {
        clampToggle.step(false, false, 0, true);
        require(clampToggle.driver.owns && !clampToggle.driver.releaseAccepted
                && clampToggle.controller.output.closed == closed
                && clampToggle.controller.state == State::off,
                "each DOWN edge toggles the clamp and owns its tick to override manual bindings");
        for (int i = 0; i < 100; ++i) {
            clampToggle.step();
            require(!clampToggle.driver.owns && clampToggle.controller.output.closed == closed
                    && !clampToggle.controller.output.arm && !clampToggle.controller.output.lift
                    && !clampToggle.controller.output.intake && clampToggle.motor.coasting()
                    && clampToggle.arm == toggleArm && clampToggle.lift == toggleLift,
                    "the clamp stays at the selected state without another edge or any mechanism movement");
        }
    }
    // Pneumatic state can differ from the macro's last output after manual A.
    clampToggle.controller.output.closed = false;
    clampToggle.driver = driverTick(clampToggle.controller, clampToggle.in, false, false, true, 0, true);
    require(!clampToggle.controller.output.closed,
            "DOWN toggles the real clamp state, not a stale macro output");
    for (int liftDirection : {-1, 1}) {
        const bool before = clampToggle.controller.output.closed;
        clampToggle.step(true, true, liftDirection, true);
        require(clampToggle.driver.owns && !clampToggle.driver.releaseAccepted
                && clampToggle.controller.output.closed != before
                && clampToggle.controller.state == State::off && !clampToggle.controller.releasePending()
                && !clampToggle.controller.endstopSelected && !clampToggle.controller.output.arm
                && !clampToggle.controller.output.lift && !clampToggle.controller.output.intake,
                "DOWN wins simultaneous R1, R2 and lift requests without selecting an arm destination");
    }
    clampToggle.step(true);
    require(clampToggle.controller.state == State::pickupWait && clampToggle.controller.calibrated,
            "a new R1 after DOWN still works with the existing calibration");

    Rig overrideWait; overrideWait.setup(); overrideWait.step(true); overrideWait.step(false, true);
    require(overrideWait.controller.state == State::pickupWait && overrideWait.controller.releasePending(),
            "the override regression starts during the clamp pause with R2 queued");
    overrideWait.step(false, false, 0, true);
    require(!overrideWait.controller.output.closed && !overrideWait.controller.releasePending()
            && overrideWait.controller.state == State::off && overrideWait.driver.owns,
            "DOWN opens immediately and cancels both the R1 clamp pause and queued R2");
    overrideWait.driverAt(overrideWait.in.now + 2 * cfg::motionTimeoutMs);
    require(!overrideWait.controller.output.closed && !overrideWait.controller.output.arm
            && !overrideWait.controller.output.lift && !overrideWait.controller.output.intake
            && overrideWait.controller.state == State::off && overrideWait.motor.coasting(),
            "expired R1 and R2 deadlines cannot restart a cancelled sequence");

    for (State phase : {State::releaseArm90, State::releaseClampWait, State::releaseWait, State::returnArmClearance,
                        State::returnArmZero}) {
        Rig overrideReturn; overrideReturn.setup(); overrideReturn.step(true); overrideReturn.until(State::carrying);
        overrideReturn.step(false, true); overrideReturn.until(phase);
        const bool wasClosed = overrideReturn.controller.output.closed;
        const double beforeArm = overrideReturn.arm, beforeLift = overrideReturn.lift;
        const std::array<double, 6> references{
            overrideReturn.controller.zeroMeasurements[0], overrideReturn.controller.zeroMeasurements[1],
            overrideReturn.controller.topMeasurements[0], overrideReturn.controller.topMeasurements[1],
            overrideReturn.controller.liftMeasurements[0], overrideReturn.controller.liftMeasurements[1]};
        overrideReturn.step(false, false, 0, true);
        require(overrideReturn.driver.owns && overrideReturn.controller.output.closed != wasClosed,
                "DOWN toggles immediately during each release and return phase");
        for (int i = 0; i < 1000; ++i) {
            overrideReturn.step();
            require(overrideReturn.controller.state == State::off
                    && overrideReturn.controller.output.closed != wasClosed
                    && !overrideReturn.controller.output.arm && !overrideReturn.controller.output.lift
                    && !overrideReturn.controller.output.intake && overrideReturn.motor.holding()
                    && overrideReturn.arm == beforeArm && overrideReturn.lift == beforeLift,
                    "a cancelled release or return cannot overwrite the clamp or restart motion");
        }
        require(overrideReturn.controller.calibrated
                && overrideReturn.controller.zeroMeasurements[0] == references[0]
                && overrideReturn.controller.zeroMeasurements[1] == references[1]
                && overrideReturn.controller.topMeasurements[0] == references[2]
                && overrideReturn.controller.topMeasurements[1] == references[3]
                && overrideReturn.controller.liftMeasurements[0] == references[4]
                && overrideReturn.controller.liftMeasurements[1] == references[5],
                "DOWN preserves all completed arm and lift calibration measurements");
    }

    Rig overrideHeight; overrideHeight.setup(); overrideHeight.step(true); overrideHeight.until(State::carrying);
    overrideHeight.step(false, false, 1);
    require(overrideHeight.controller.output.lift > 0, "the lift is moving before the DOWN override");
    const double overrideLift = overrideHeight.lift;
    overrideHeight.step(false, false, 0, true);
    for (int i = 0; i < 1000; ++i) {
        require(!overrideHeight.controller.output.closed && !overrideHeight.controller.output.lift
                && overrideHeight.lift == overrideLift && overrideHeight.controller.state == State::off,
                "DOWN stops an active lift request and never resumes it automatically");
        overrideHeight.step();
    }

    for (bool calibrated : {false, true}) {
        Rig invalidToggle;
        if (calibrated) {
            invalidToggle.setup(); invalidToggle.step(true); invalidToggle.until(State::pickupArm);
        }
        invalidToggle.in.valid = false;
        invalidToggle.step(false, false, 0, true);
        require(invalidToggle.driver.owns && invalidToggle.controller.output.closed == !calibrated
                && invalidToggle.controller.calibrated == calibrated
                && invalidToggle.controller.state == State::off
                && !invalidToggle.controller.output.arm && !invalidToggle.controller.output.lift,
                "DOWN works with missing sensors before calibration and during a calibrated macro");
        invalidToggle.in.valid = true;
        invalidToggle.driverAt(invalidToggle.in.now + cfg::sensorRecoveryMs + cfg::motionTimeoutMs);
        require(invalidToggle.controller.state == State::off
                && invalidToggle.controller.output.closed == !calibrated,
                "sensor recovery cannot undo the override or restart the interrupted macro");
    }
    Rig faultToggle; faultToggle.liftSpeedScale = 0; faultToggle.start(); faultToggle.until(State::fault);
    const bool faultClamp = faultToggle.controller.output.closed;
    faultToggle.in.valid = false;
    faultToggle.step(false, false, 0, true);
    require(faultToggle.driver.owns && faultToggle.controller.output.closed != faultClamp
            && !faultToggle.controller.calibrated && !faultToggle.controller.output.arm
            && !faultToggle.controller.output.lift && !faultToggle.controller.output.intake,
            "DOWN remains a clamp-only override after a failed preMatch with invalid sensors");

    Rig r; r.setup();
    // R1 is the manual grab trigger: no color, proximity, or object count is involved.
    r.controller.grab(r.in);
    require(r.controller.state == State::pickupWait && r.controller.output.closed,
            "grab closes the clamp and starts the clamp pause before the pickup sequence");
    r.until(State::pickupArm);
    require(std::abs(r.lift - liftPickup) < 0.03 && r.arm < 1, "lift rises 0.2 rotations before arm");
    r.step(); require(r.controller.output.lift == 0, "lift stops at pickup height");
    r.until(State::carrying);
    require(std::abs(r.lift - liftPickup) < 0.03 && r.controller.output.lift == 0,
            "R1 must keep the pickup lift height after the arm arrives, not climb to raw encoder zero");
    require(std::abs(r.arm - cfg::armPickupDeg) < 2 && r.controller.output.closed && !r.controller.output.intake,
            "pickup pose");
    r.step(); require(r.controller.output.armHold && r.controller.output.arm == 0,
        "settled carrying pose uses motor hold without PID vibration");
    require(r.motor.holding(), "carrying actually engages the motor's hold controller");
    r.controller.release(r.in); r.until(State::releaseWait);
    require(std::abs(r.arm - 90) <= cfg::armReleaseToleranceDeg && !r.controller.output.closed, "release opens at 90 degrees");
    r.step(); require(r.controller.output.armHold && r.controller.output.arm == 0,
        "settled release pose keeps motor hold");
    r.until(State::prepared);
    require(r.arm < 0.1 && std::abs(r.lift - liftLow) < 0.03 && !r.controller.output.closed,
            "release returns to prepared lower pose");
    Rig repeated; repeated.setup(); repeated.controller.grab(repeated.in); repeated.until(State::carrying);
    require(std::abs(repeated.arm - cfg::armPickupDeg) < 2, "second manual grab is repeatable");
    repeated.controller.cancel(); repeated.step();
    require(!repeated.controller.output.arm && !repeated.controller.output.lift
            && !repeated.controller.output.intake, "cancel stops mechanisms");
    Rig shortTravel; shortTravel.maxArm = 100; shortTravel.start();
    shortTravel.until(State::fault);
    require(!shortTravel.controller.calibrated && !shortTravel.controller.output.arm,
            "short travel cannot authorize the clearance target");
    require(shortTravel.motor.braking && !shortTravel.motor.holding(),
            "preMatch faults release homing effort with passive brake");

    Rig loaded; loaded.setup(); loaded.armLoadPower = 8;
    loaded.controller.grab(loaded.in); loaded.until(State::carrying);
    const double heldPosition = loaded.arm;
    for (unsigned i = 0; i < 2 * cfg::motionTimeoutMs; i += 10) {
        loaded.step();
        require(loaded.controller.state == State::carrying && loaded.motor.holding(),
                "loaded arm stays in carrying without a timeout or brake release");
    }
    require(loaded.arm == heldPosition, "gravity must not lower the held arm after R1");
    loaded.controller.release(loaded.in); loaded.until(State::releaseWait);
    require(loaded.motor.holding() && !loaded.controller.output.closed,
            "R2 opens the clamp with the arm held at the release pose");

    Rig friction; friction.armFrictionPower = 8; friction.setup();
    friction.step(true, false); friction.until(State::carrying);
    require(std::abs(friction.arm - cfg::armPickupDeg) <= cfg::armToleranceDeg && friction.motor.holding(),
            "R1 completes under friction instead of stalling just before the pickup target");
    friction.step(false, true); friction.until(State::releaseWait);
    require(std::abs(friction.arm - 90) <= cfg::armReleaseToleranceDeg && !friction.controller.output.closed,
            "the fast descending R2 move also overcomes friction");
    friction.until(State::prepared);

    Rig heavyFriction; heavyFriction.setup(); heavyFriction.armFrictionPower = 55;
    heavyFriction.step(true, false); heavyFriction.until(State::carrying);
    require(std::abs(heavyFriction.arm - cfg::armPickupDeg) <= cfg::armToleranceDeg && heavyFriction.motor.holding(),
            "the loaded R1 approach can exceed the old 20/30 power floor without stalling");
    heavyFriction.step(false, true); heavyFriction.until(State::prepared);
    require(heavyFriction.controller.calibrated && !heavyFriction.controller.output.closed,
            "load assistance also completes the 144-degree clearance during the full R2 return");

    Rig opposedRelease; opposedRelease.setup(); opposedRelease.step(true, false);
    opposedRelease.until(State::carrying); opposedRelease.armLoadPower = -8;
    opposedRelease.step(false, true); opposedRelease.until(State::releaseWait);
    require(std::abs(opposedRelease.arm - 90) <= cfg::armReleaseToleranceDeg && opposedRelease.motor.holding(),
            "release reaches 90 degrees even when the load opposes descent");

    Rig sensorFault; sensorFault.setup(); sensorFault.controller.grab(sensorFault.in);
    sensorFault.until(State::carrying); sensorFault.armLoadPower = 2;
    const double faultPosition = sensorFault.arm;
    sensorFault.in.valid = false;
    for (int i = 0; i < 100; ++i) {
        sensorFault.step();
        require((sensorFault.controller.state == State::carrying || sensorFault.controller.state == State::stopped)
                && sensorFault.controller.calibrated && sensorFault.motor.holding()
                && !sensorFault.controller.output.arm && !sensorFault.controller.output.lift
                && !sensorFault.controller.output.intake && sensorFault.controller.output.closed,
                "invalid input holds the load without latching a fault or losing calibration");
    }
    require(sensorFault.arm == faultPosition, "a fault must not let the load fall");
    sensorFault.in.valid = true; sensorFault.step();
    require(sensorFault.controller.state == State::stopped, "a long outage requires a new request before motion");
    sensorFault.step(false, true); sensorFault.until(State::releaseWait);
    require(sensorFault.controller.calibrated, "R2 retries after reconnection without preMatch");

    Rig blocked; blocked.setup(); blocked.controller.grab(blocked.in);
    blocked.until(State::pickupArm); blocked.maxArm = 40;
    blocked.until(State::stopped);
    require(blocked.motor.holding() && blocked.controller.calibrated && !blocked.controller.error[0],
            "real driver jams stop effort without a latched fault or lost calibration");

    Rig nearTargetJam; nearTargetJam.setup(); nearTargetJam.step(true, false);
    nearTargetJam.until(State::pickupArm); nearTargetJam.maxArm = cfg::armPickupDeg - 3;
    nearTargetJam.until(State::stopped);
    require(nearTargetJam.motor.holding() && nearTargetJam.controller.calibrated
            && nearTargetJam.controller.output.closed,
            "a real stop three degrees before target holds without opening the clamp");

    Rig hunting; hunting.setup(); hunting.step(true, false); hunting.until(State::pickupArm);
    double displaced = cfg::armPickupDeg - 30;
    for (unsigned i = 0; i < 500 && hunting.controller.state == State::pickupArm; ++i) {
        hunting.arm = displaced; hunting.step();
        if (hunting.controller.output.arm)
            displaced = cfg::armPickupDeg + (hunting.controller.output.arm > 0 ? 6 : -6);
    }
    require(hunting.controller.state == State::stopped && hunting.motor.holding()
            && hunting.controller.calibrated && !hunting.controller.error[0]
            && !hunting.controller.output.armBrake && hunting.controller.output.closed,
            "repeated overshoots stop in HOLD without a latched driver error or false arrival");
    const double stoppedArm = hunting.arm;
    // A stop cancels the previous lift target; new steps start from the actual
    // position, including its permitted settling error at the pickup height.
    const double stoppedLift = hunting.lift;
    hunting.height(1, stoppedLift + cfg::driverLiftStepRotations);
    hunting.height(-1, stoppedLift);
    require(hunting.arm == stoppedArm && hunting.controller.state == State::stopped && hunting.motor.holding(),
            "height requests still position the lift after arm corrections are exhausted, while the arm holds");
    hunting.step(true, false); hunting.until(State::carryingEndstop);
    require(hunting.controller.calibrated, "R1 retargets immediately after the correction limit without preMatch");

    for (bool useRelease : {false, true}) {
        Rig braking; braking.setup(); braking.step(true, false); braking.until(State::pickupArm);
        for (unsigned i = 0; i < 300 && !braking.controller.output.armBrake; ++i) braking.step();
        require(braking.controller.output.armBrake && braking.motor.braking
                && braking.motor.mode == pros::E_MOTOR_BRAKE_BRAKE,
                "profile braking reaches the motor as BRAKE, not voltage zero or position HOLD");
        braking.step(!useRelease, useRelease);
        require(!braking.controller.output.armBrake
                && (useRelease ? !braking.motor.braking : braking.motor.holding()),
                "R1 preempts passive braking with clamp-wait HOLD, while R2 starts release immediately");
        braking.until(useRelease ? State::releaseWait : State::carryingEndstop);
    }

    // A brief R2 press before the arm finishes settling used to disappear.
    Rig earlyRelease; earlyRelease.setup(); earlyRelease.controller.grab(earlyRelease.in);
    earlyRelease.until(State::pickupArm);
    while (earlyRelease.arm < cfg::armPickupDeg - 20 && earlyRelease.controller.state == State::pickupArm) earlyRelease.step();
    require(earlyRelease.controller.state == State::pickupArm, "arm is still approaching the pickup target");
    earlyRelease.controller.release(earlyRelease.in);
    require(earlyRelease.controller.state == State::releaseArm90,
            "R2 starts release while the arm is still approaching the pickup target");
    require(earlyRelease.controller.output.closed, "R2 must not open the clamp before reaching 90 degrees");
    earlyRelease.until(State::releaseWait);
    require(std::abs(earlyRelease.arm - 90) < cfg::armReleaseToleranceDeg && !earlyRelease.controller.output.closed,
            "early R2 opens at the release pose");
    earlyRelease.until(State::prepared);

    Rig liftingRelease; liftingRelease.setup(); liftingRelease.step(true);
    const auto liftingStarted = liftingRelease.in.now;
    liftingRelease.driverAt(liftingStarted + 50, false, true);
    require(liftingRelease.driver.releaseAccepted && liftingRelease.controller.releasePending()
            && liftingRelease.controller.state == State::pickupWait && liftingRelease.controller.output.closed,
            "R2 during the clamp pause waits for both the pause and the pickup height");
    liftingRelease.driverAt(liftingStarted + 299);
    require(liftingRelease.controller.state == State::pickupWait && liftingRelease.motor.coasting()
            && !liftingRelease.controller.output.arm && !liftingRelease.controller.output.lift,
            "queued R2 cannot shorten the minimum clamp pause at the low lift height");
    liftingRelease.driverAt(liftingStarted + cfg::pickupClampPauseMs);
    require(liftingRelease.controller.state == State::pickupLift && liftingRelease.controller.releasePending(),
            "queued R2 first raises the low lift when the clamp pause finishes");
    liftingRelease.until(State::releaseArm90);
    require(std::abs(liftingRelease.lift - liftPickup) < 0.03 && liftingRelease.arm < 1
            && liftingRelease.controller.output.closed,
            "a queued R2 press starts arm release only after the lift is raised");
    liftingRelease.until(State::releaseWait);
    // Additional presses during release must not restart the arm move or its timers.
    liftingRelease.controller.release(liftingRelease.in);
    require(liftingRelease.controller.state == State::releaseWait, "repeated R2 does not restart release");
    liftingRelease.until(State::prepared);
    liftingRelease.controller.grab(liftingRelease.in); liftingRelease.until(State::carrying);
    require(liftingRelease.controller.output.closed, "queued release is consumed once and cannot leak to the next pickup");

    Rig cancelledRelease; cancelledRelease.setup(); cancelledRelease.controller.grab(cancelledRelease.in);
    cancelledRelease.controller.release(cancelledRelease.in); cancelledRelease.controller.cancel();
    cancelledRelease.controller.grab(cancelledRelease.in);
    cancelledRelease.until(State::carryingEndstop);
    require(cancelledRelease.controller.output.closed, "R2 cancels queued release and the next R1 toggles to the endstop");
    blocked.maxArm = 195;
    blocked.step(false, true); blocked.until(State::releaseWait);
    require(blocked.motor.holding() && !blocked.controller.output.closed,
            "R2 can retry after an obstruction is removed without repeating homing");

    Rig readyRelease; readyRelease.setup();
    readyRelease.step(false, true);
    require(readyRelease.driver.owns && readyRelease.driver.releaseAccepted,
            "the driver handles R2 from prepared before enabling manual motor commands");
    require(readyRelease.controller.state == State::pickupLift && !readyRelease.controller.output.closed,
            "release from the lower pose raises the lift while preserving the initially open clamp");
    readyRelease.until(State::releaseWait);
    require(std::abs(readyRelease.arm - 90) <= cfg::armReleaseToleranceDeg && !readyRelease.controller.output.closed,
            "release from prepared opens only at 90 degrees");
    readyRelease.until(State::prepared);
    readyRelease.controller.cancel();
    readyRelease.step(false, true);
    require(readyRelease.driver.releaseAccepted, "stopping a calibrated robot must not disable R2");

    Rig manualRelease; manualRelease.setup(); manualRelease.step(true, false);
    manualRelease.until(State::carrying);
    const double measuredZero = manualRelease.controller.armPosition(0);
    manualRelease.controller.cancel();
    manualRelease.step();
    require(manualRelease.controller.state == State::off && manualRelease.controller.calibrated
            && !manualRelease.driver.owns && manualRelease.motor.holding(),
            "R2 or controller loss stops the macro and preserves calibrated manual control");
    // A changes the real clamp independently while manual bindings own it.
    manualRelease.driver = driverTick(manualRelease.controller, manualRelease.in, false, true, false);
    require(manualRelease.driver.releaseAccepted && !manualRelease.controller.output.closed,
            "A then R2 keeps the actual clamp state and accepts release after cancellation");
    manualRelease.until(State::releaseWait);
    require(std::abs(manualRelease.arm - 90) <= cfg::armReleaseToleranceDeg
            && manualRelease.controller.armPosition(0) == measuredZero,
            "release after manual control still uses the original calibrated zero");
    manualRelease.until(State::prepared);

    Rig suspended; suspended.setup(); suspended.step(true, false); suspended.until(State::carrying);
    suspended.controller.suspend(); suspended.controller.suspend();
    suspended.in.now += 20000;
    suspended.controller.resume(suspended.in.now);
    suspended.step();
    require(suspended.controller.calibrated && !suspended.driver.owns && !suspended.controller.output.arm,
            "driver reentry preserves references without resuming a previously interrupted move");
    suspended.step(false, true);
    require(suspended.driver.releaseAccepted, "R2 works after driver reentry with a held load");
    suspended.until(State::releaseWait);

    Rig noCalibration; noCalibration.controller.cancel(); noCalibration.step(false, true);
    require(!noCalibration.driver.releaseAccepted && !noCalibration.controller.calibrated,
            "a robot that never calibrated cannot run release");
    manualRelease.start(); manualRelease.controller.cancel();
    manualRelease.step(false, true);
    require(!manualRelease.driver.releaseAccepted && !manualRelease.controller.calibrated,
            "interrupting a new homing run cannot restore the old reference");
    blocked.controller.cancel(); blocked.step(false, true);
    require(blocked.driver.releaseAccepted && blocked.controller.calibrated,
            "driver stops preserve the measured references after R2 too");

    Rig driverRelease; driverRelease.setup(); driverRelease.step(true, false);
    driverRelease.until(State::carrying); driverRelease.step(false, true);
    require(driverRelease.driver.owns && driverRelease.driver.releaseAccepted
            && driverRelease.controller.state == State::releaseArm90,
            "driver R1 then R2 sends the release motion to the arm motor");
    driverRelease.until(State::releaseWait);
    require(driverRelease.motor.holding() && !driverRelease.controller.output.closed,
            "driver release engages HOLD and opens the clamp");

    for (int presses = 1; presses <= 4; ++presses) {
        Rig toggle; toggle.setup();
        for (int i = 0; i < presses; ++i) { toggle.step(true, false); toggle.step(); }
        require(toggle.controller.endstopSelected == (presses % 2 == 0),
                "every rapid R1 edge toggles the destination even while the lift is rising");
        toggle.until(presses % 2 ? State::carrying : State::carryingEndstop);
        const double expected = presses % 2 ? cfg::armPickupDeg : toggle.maxArm;
        require(std::abs(toggle.arm - expected) <= cfg::armToleranceDeg
                && (presses % 2 ? toggle.motor.holding() : toggle.motor.coasting()),
                "odd R1 presses reach the pickup target, even presses go directly to the actual second endstop");
    }
    Rig switching; switching.encoderPolarity = -1; switching.setup();
    switching.step(true, false); switching.until(State::carrying);
    switching.step(true, false);
    require(switching.controller.state == State::pickupWait && switching.motor.holding(),
            "R1 from the pickup target pauses with HOLD before the upper endstop move");
    while (switching.arm < 165) switching.step();
    switching.step(true, false);
    require(switching.controller.state == State::pickupWait && switching.motor.holding()
            && !switching.controller.output.arm && !switching.controller.output.lift,
            "R1 stops an ongoing endstop move and pauses before reversing toward the pickup target");
    switching.until(State::carrying);
    switching.step(true, false); switching.until(State::carryingEndstop);
    switching.step(true, false); switching.until(State::carrying);
    require(std::abs(switching.arm - cfg::armPickupDeg) <= cfg::armToleranceDeg,
            "pickup/endstop toggles also work with reversed Rotation polarity");

    Rig releaseAtTop; releaseAtTop.encoderPolarity = -1; releaseAtTop.setup();
    releaseAtTop.step(true, false); releaseAtTop.step(); releaseAtTop.step(true, false);
    releaseAtTop.until(State::carryingEndstop);
    const double topReleasePosition = releaseAtTop.arm;
    releaseAtTop.step(false, true);
    require(releaseAtTop.controller.state == State::releaseWait && !releaseAtTop.controller.output.closed
            && releaseAtTop.motor.coasting() && releaseAtTop.arm == topReleasePosition,
            "R2 at the second endstop opens immediately in place without the 90-degree move");
    for (int i = 0; i < 6000 && releaseAtTop.controller.state != State::prepared; ++i) {
        releaseAtTop.step();
        require(releaseAtTop.controller.state != State::releaseArm90, "endstop release skips 90 throughout the return");
    }
    require(releaseAtTop.controller.state == State::prepared && releaseAtTop.arm < 0.1
            && !releaseAtTop.controller.output.closed && std::abs(releaseAtTop.lift - liftLow) < 0.03,
            "endstop release completes the usual clearance/lift/zero return with the clamp open");
    Rig releaseOnWayToTop; releaseOnWayToTop.setup();
    releaseOnWayToTop.step(true, false); releaseOnWayToTop.step(); releaseOnWayToTop.step(true, false);
    releaseOnWayToTop.until(State::pickupEndstop);
    for (int i = 0; i < 500 && releaseOnWayToTop.arm < 160; ++i) releaseOnWayToTop.step();
    releaseOnWayToTop.step(false, true);
    require(releaseOnWayToTop.controller.state == State::releaseArm90 && releaseOnWayToTop.controller.output.closed,
            "selecting the endstop alone does not skip 90 before the arm actually gets there");
    releaseOnWayToTop.until(State::releaseWait);

    for (State phase : {State::releaseArm90, State::releaseClampWait, State::releaseWait, State::returnArmClearance,
                        State::returnArmZero, State::returnLiftBottom}) {
        Rig preempt; preempt.setup(); preempt.step(true, false); preempt.until(State::carrying);
        preempt.step(false, true); preempt.until(phase);
        const double priorLift = preempt.lift;
        preempt.step(true, false);
        const bool expectEndstop = phase != State::returnLiftBottom;
        require(preempt.controller.endstopSelected == expectEndstop && preempt.controller.output.closed
                && preempt.controller.state == State::pickupWait && (preempt.motor.holding() || preempt.motor.coasting())
                && !preempt.controller.output.arm && !preempt.controller.output.lift,
                "R1 cancels release/return immediately, closes the clamp and pauses all movement");
        preempt.until(expectEndstop ? State::carryingEndstop : State::carrying);
        require(std::abs(preempt.lift - std::max(priorLift, liftPickup)) <= cfg::liftToleranceRot,
                "R1 preemption restores pickup clearance if needed and preserves a higher lift position");
    }

    Rig fast; fast.setup(); fast.step(true, false); fast.until(State::carrying);
    const auto fastStart = fast.in.now;
    fast.step(false, true);
    require(!fast.controller.output.armProfileActive && fast.controller.output.arm == -cfg::armReleasePower,
            "R2 immediately commands fast constant power without the positioning ramp");
    fast.until(State::releaseClampWait);
    const auto fastElapsed = fast.in.now - fastStart;
    require(fastElapsed < 600 && std::abs(fast.arm - 90) <= cfg::armReleaseToleranceDeg,
            "R2 covers 150 to approximately 90 in under 600ms in the host plant");
    const auto clampPauseStarted = fast.in.now;
    const double releaseArmPosition = fast.arm, releaseLiftPosition = fast.lift;
    require(fast.controller.output.closed && fast.motor.holding(),
            "R2 keeps the clamp closed and holds the arm after reaching the release pose");
    fast.driverAt(clampPauseStarted + 199, false, true);
    require(fast.controller.state == State::releaseClampWait && fast.controller.output.closed
            && fast.motor.holding() && !fast.controller.output.arm && !fast.controller.output.lift
            && fast.arm == releaseArmPosition && fast.lift == releaseLiftPosition,
            "the clamp stays closed for 199ms without moving; repeated R2 does not restart the pause");
    fast.driverAt(clampPauseStarted + 200);
    require(fast.controller.state == State::releaseWait && !fast.controller.output.closed && fast.motor.holding(),
            "R2 opens the clamp 200ms after the arm stops");
    Rig profiled; profiled.setup(); profiled.manuallyPositionArm(cfg::armPickupDeg + 40);
    profiled.step(true, false); profiled.until(State::pickupArm);
    const auto profileStart = profiled.in.now;
    profiled.until(State::carrying);
    const auto profileElapsed = profiled.in.now - profileStart;
    require(profileElapsed < 800, "the new profile completes a 40-degree move without the old 2190ms PID tail");
    std::cout << "Simulated R2 move: " << fastElapsed << " ms; 40-degree profile: " << profileElapsed << " ms\n";
    Rig crossed; crossed.setup(); crossed.manuallyPositionArm(105);
    crossed.step(false, true); crossed.until(State::releaseArm90); crossed.step();
    crossed.arm = 82; crossed.step();
    require(crossed.controller.state == State::releaseClampWait && crossed.motor.holding(),
            "crossing the release threshold between samples stops once, without reversing to chase 90");

    Rig dropout; dropout.setup(); dropout.step(true, false); dropout.until(State::pickupArm);
    while (dropout.arm < 50) dropout.step();
    dropout.in.valid = false;
    const double pausedPosition = dropout.arm;
    for (int i = 0; i < 5; ++i) dropout.step();
    require(dropout.controller.state == State::pickupArm && dropout.motor.holding()
            && dropout.arm == pausedPosition && dropout.controller.calibrated,
            "a brief missing sample holds position without changing the request");
    dropout.in.valid = true; dropout.until(State::carrying);
    require(std::abs(dropout.arm - cfg::armPickupDeg) <= cfg::armToleranceDeg, "a brief dropout resumes the same profile target automatically");
    dropout.in.valid = false; dropout.step();
    dropout.in.valid = true; dropout.in.now += cfg::sensorRecoveryMs; dropout.step();
    require(dropout.controller.state == State::stopped && dropout.controller.calibrated,
            "a long sample gap cannot resume automatically even when the next sample is valid");
    dropout.step(true, false); dropout.until(State::carryingEndstop);

    Rig pausedToggle; pausedToggle.setup(); pausedToggle.step(true, false); pausedToggle.until(State::pickupArm);
    pausedToggle.in.valid = false; pausedToggle.step(); pausedToggle.step();
    pausedToggle.step(true, false); pausedToggle.step(); pausedToggle.in.valid = true;
    pausedToggle.until(State::carryingEndstop);
    require(pausedToggle.controller.calibrated, "retargeting during a brief dropout cannot underflow the motion timer");

    Rig simultaneous; simultaneous.setup(); simultaneous.step(true, true);
    require(!simultaneous.driver.releaseAccepted && !simultaneous.controller.endstopSelected,
            "R1 has deterministic priority over simultaneous R2");
    simultaneous.until(State::carrying);

    Rig obstructedTop; obstructedTop.setup(); obstructedTop.step(true, false); obstructedTop.step(true, false);
    obstructedTop.maxArm = 160; obstructedTop.until(State::stopped);
    require(obstructedTop.controller.calibrated && obstructedTop.motor.holding()
            && !obstructedTop.controller.output.armCoast,
            "an obstruction before the learned second endstop is a retryable stop, not a completed endstop move");

    Rig profiledTop; profiledTop.setup(); profiledTop.step(true, false); profiledTop.step(true, false);
    profiledTop.until(State::pickupEndstop);
    bool fastCruise = false, slowContact = false;
    for (int i = 0; i < 700 && profiledTop.controller.state == State::pickupEndstop; ++i) {
        const double position = profiledTop.arm;
        profiledTop.step();
        const int power = profiledTop.controller.output.arm;
        if (position < profiledTop.maxArm - 20 && power >= 100) fastCruise = true;
        if (position >= profiledTop.maxArm - 3 && power > 0 && power <= 50) slowContact = true;
    }
    require(fastCruise && slowContact && profiledTop.controller.state == State::carryingEndstop
            && profiledTop.motor.coasting() && profiledTop.arm == profiledTop.maxArm,
            "upper endstop uses fast cruise, a soft final contact and COAST after real contact");

    Rig liftBeforeArmTiming;
    liftBeforeArmTiming.start(); liftBeforeArmTiming.until(State::armZero1);
    auto& prematchTiming = liftBeforeArmTiming.controller;
    Input contact = liftBeforeArmTiming.in;
    contact.armCurrent = 800;
    contact.armVelocity = 0;
    require(prematchTiming.state == State::armZero1,
            "first arm contact timing starts after the complete lift reset");
    const auto armHomeStart = contact.now;
    for (unsigned now : {0u, 100u, 349u, 350u, 749u}) {
        contact.now = armHomeStart + now; prematchTiming.tick(contact);
        require(prematchTiming.state == State::armZero1,
                "preMatch still requires the original 350ms grace plus 400ms continuous contact");
    }
    contact.now = armHomeStart + 750; prematchTiming.tick(contact);
    require(prematchTiming.state == State::armBackoff0, "preMatch contact completes at its unchanged 750ms minimum");

    Rig driverTiming; driverTiming.setup(); driverTiming.manuallyPositionArm(driverTiming.maxArm);
    driverTiming.step(true, false); driverTiming.step(); driverTiming.step(true, false);
    driverTiming.until(State::pickupEndstop);
    const auto topStarted = driverTiming.in.now;
    driverTiming.until(State::carryingEndstop);
    const auto topContactMs = driverTiming.in.now - topStarted;
    require(topContactMs >= 200 && topContactMs <= 220,
            "driver upper endstop already in contact completes in about 200ms instead of 750ms");
    driverTiming.step(false, true); driverTiming.until(State::returnArmZero);
    driverTiming.arm = 0;
    const auto zeroStarted = driverTiming.in.now;
    driverTiming.until(State::returnLiftBottom);
    const auto zeroContactMs = driverTiming.in.now - zeroStarted;
    require(zeroContactMs >= 200 && zeroContactMs <= 220,
            "driver lower endstop uses the same shorter contact confirmation");
    std::cout << "Endstop confirmation, starting in contact: preMatch 750 ms; driver top "
              << topContactMs << " ms, driver zero " << zeroContactMs << " ms\n";

    require(cfg::liftLowerRotations > cfg::returnLiftRotations,
            "pin pickup is below the R2 passage clearance");
    for (double origin : {-5.0, 0.0, 7.0}) for (int polarity : {-1, 1}) for (bool raised : {false, true}) {
        Rig passage; passage.liftOffset = origin; passage.lift += origin;
        passage.encoderPolarity = polarity; passage.setup();
        passage.step(true); passage.until(State::carrying);
        if (raised) passage.height(1, origin + liftLevel1);
        passage.step(false, true); passage.until(State::returnArmClearance);
        const auto returnStarted = passage.in.now;
        bool overlapping = false, liftClose = false;
        std::uint32_t liftCloseSince = 0;
        while (passage.controller.state == State::returnArmClearance) {
            passage.step();
            if (passage.controller.output.arm && passage.controller.output.lift) {
                overlapping = true;
                require(raised ? passage.controller.output.lift < 0 : passage.controller.output.lift > 0,
                        "R2 concurrently raises or lowers the lift to the same passage height");
            }
            if (std::abs(passage.in.lift - (origin - cfg::returnLiftRotations)) <= cfg::liftToleranceRot) {
                if (!liftClose) { liftClose = true; liftCloseSince = passage.in.now; }
            } else liftClose = false;
            if (std::abs(passage.in.arm * polarity - cfg::armClearanceDeg) > cfg::armReturnClearanceToleranceDeg
                || !liftClose || passage.in.now - liftCloseSince < cfg::settleMs)
                require(passage.controller.state == State::returnArmClearance,
                        "front homing waits for arm clearance and the lift's full settling interval");
        }
        require(overlapping && passage.controller.state == State::returnArmZero
                && std::abs(passage.arm - cfg::armClearanceDeg) <= cfg::armReturnClearanceToleranceDeg
                && std::abs(passage.lift - (origin - cfg::returnLiftRotations)) <= cfg::liftToleranceRot,
                "parallel return joins both settled axes before starting front homing");
        while (passage.controller.state == State::returnArmZero) {
            passage.step();
            require(std::abs(passage.lift - (origin - 3.0)) <= cfg::liftToleranceRot,
                    "the lift remains at quota three throughout the front endstop approach");
        }
        require(passage.controller.state == State::returnLiftBottom && passage.arm == 0,
                "the final bottom descent starts after confirmed front endstop contact");
        while (passage.controller.state == State::returnLiftBottom) {
            passage.step();
            require(passage.motor.coasting() && passage.arm == 0,
                    "the final lift descent preserves COAST at the front endstop");
        }
        require(passage.controller.state == State::prepared && std::abs(passage.lift - (origin + liftLow)) < 0.03,
                "R2 finishes at the configured normal pin pickup position");
        if (!origin && polarity == 1)
            std::cout << "R2 return after departure, " << (raised ? "raised" : "base")
                      << " lift: " << passage.in.now - returnStarted << " ms\n";

        Rig rapid; rapid.liftOffset = origin; rapid.lift += origin; rapid.setup();
        rapid.step(false, false, 1); rapid.step(false, false, 1);
        for (unsigned i = 0; i < 500; ++i) rapid.step();
        require(std::abs(rapid.lift - (origin + liftLow + 2 * cfg::driverLiftStepRotations)) < 0.03,
                "quick B presses accumulate two full steps even before the first one finishes");
    }

    // Either actuator can be the slower one. A jam in one must stop both,
    // including the case where the first helper fails before the second runs.
    for (bool jamArm : {false, true}) {
        Rig jammedReturn; jammedReturn.setup(); jammedReturn.step(true); jammedReturn.until(State::carrying);
        jammedReturn.height(1, liftLevel1);
        jammedReturn.step(false, true); jammedReturn.until(State::returnArmClearance);
        if (jamArm) jammedReturn.maxArm = jammedReturn.arm;
        else jammedReturn.liftFloor = jammedReturn.lift;
        jammedReturn.until(State::stopped);
        require(jammedReturn.controller.calibrated && jammedReturn.motor.holding()
                && !jammedReturn.controller.output.arm && !jammedReturn.controller.output.lift,
                "a jam on either parallel axis stops both outputs without losing calibration");
        for (int i = 0; i < 100; ++i) {
            jammedReturn.step();
            require(jammedReturn.controller.state == State::stopped
                    && !jammedReturn.controller.output.arm && !jammedReturn.controller.output.lift,
                    "a parallel-axis jam cannot restart the other motor on subsequent ticks");
        }
    }

    // Interrupt the phase while both motors are actually powered, rather
    // than only at the idle tick where the parallel phase was entered.
    for (int interruption = 0; interruption < 5; ++interruption) {
        Rig interrupted; interrupted.setup(); interrupted.step(true); interrupted.until(State::carrying);
        interrupted.height(1, liftLevel1);
        interrupted.step(false, true); interrupted.until(State::returnArmClearance);
        for (int i = 0; i < 20 && !(interrupted.controller.output.arm && interrupted.controller.output.lift); ++i)
            interrupted.step();
        require(interrupted.controller.state == State::returnArmClearance
                && interrupted.controller.output.arm && interrupted.controller.output.lift,
                "interruption regression begins with both return motors moving");
        const double pausedArm = interrupted.arm, pausedLift = interrupted.lift;
        if (interruption == 0 || interruption == 1) {
            if (interruption == 0) interrupted.controller.suspend();
            else interrupted.controller.cancel();
            applyArmMotor(interrupted.motor, interrupted.controller.output, interrupted.controller.state, interruption != 0);
            require(!interrupted.controller.output.arm && !interrupted.controller.output.lift,
                    "disable or cancellation immediately stops both parallel outputs");
            interrupted.controller.resume(interrupted.in.now);
            for (int i = 0; i < 30; ++i) interrupted.step();
            require(interrupted.controller.state == State::prepared && interrupted.arm == pausedArm
                    && interrupted.lift == pausedLift && !interrupted.controller.output.arm
                    && !interrupted.controller.output.lift,
                    "driver reentry never resumes a cancelled parallel return");
        } else if (interruption == 2) {
            interrupted.step(true);
            require(interrupted.controller.state == State::pickupWait && interrupted.controller.output.closed
                    && !interrupted.controller.output.arm && !interrupted.controller.output.lift
                    && interrupted.arm == pausedArm && interrupted.lift == pausedLift,
                    "R1 stops both active return motors before its full clamp pause");
            interrupted.until(State::carryingEndstop);
        } else {
            interrupted.in.valid = false;
            interrupted.step();
            require(interrupted.controller.state == State::returnArmClearance && interrupted.motor.holding()
                    && !interrupted.controller.output.arm && !interrupted.controller.output.lift
                    && interrupted.arm == pausedArm && interrupted.lift == pausedLift,
                    "missing telemetry pauses both concurrent motors immediately");
            interrupted.driverAt(interrupted.in.now + (interruption == 3 ? 50 : cfg::sensorRecoveryMs));
            interrupted.in.valid = true;
            interrupted.step();
            if (interruption == 3) {
                require(interrupted.controller.state == State::returnArmClearance,
                        "a brief telemetry pause resumes the same parallel request");
                interrupted.until(State::prepared);
            } else {
                require(interrupted.controller.state == State::stopped && interrupted.controller.calibrated
                        && !interrupted.controller.output.arm && !interrupted.controller.output.lift,
                        "a persistent telemetry loss latches both parallel outputs off after reconnection");
            }
        }
    }

    // A stationary arm already on the front side with the lift at passage
    // height can return directly after the same half-turn departure gate.
    for (double origin : {-5.0, 0.0, 7.0}) for (int polarity : {-1, 1}) {
        Rig directReturn; directReturn.liftOffset = origin; directReturn.lift += origin;
        directReturn.encoderPolarity = polarity; directReturn.autoDepartAfterRelease = false;
        directReturn.setup(); directReturn.step(true); directReturn.until(State::carrying);
        directReturn.in.forwardPodDegrees = 1234;
        directReturn.step(false, true); directReturn.until(State::releaseWait);
        directReturn.lift = origin - cfg::returnLiftRotations;
        directReturn.in.armVelocity = directReturn.in.liftVelocity = 0;
        directReturn.driverAt(directReturn.in.now + cfg::releasePauseMs);
        const double releasedArm = directReturn.arm;
        require(directReturn.controller.state == State::releaseWait,
                "being at passage height does not bypass the half-turn departure gate");
        directReturn.in.forwardPodDegrees = 1234 + 179.99;
        directReturn.step();
        require(directReturn.controller.state == State::releaseWait,
                "the direct return still rejects departure just below half a rotation");
        directReturn.in.forwardPodDegrees = 1234 + 180;
        directReturn.step();
        require(directReturn.controller.state == State::returnArmZero,
                "stationary passage height enables direct front homing without the 144-degree detour");
        const auto directStarted = directReturn.in.now;
        for (int i = 0; i < 1000 && directReturn.controller.state != State::prepared; ++i) {
            directReturn.step();
            require(directReturn.arm <= releasedArm && directReturn.controller.state != State::returnArmClearance,
                    "the direct return never reverses toward 144 degrees");
        }
        require(directReturn.controller.state == State::prepared && directReturn.arm == 0
                && std::abs(directReturn.lift - (origin + liftLow)) <= cfg::liftToleranceRot,
                "the direct return reaches the same open-clamp pickup pose");
        if (!origin && polarity == 1)
            std::cout << "R2 direct return after departure: " << directReturn.in.now - directStarted << " ms\n";
    }
    for (int ineligible = 0; ineligible < 3; ++ineligible) {
        Rig guarded; guarded.autoDepartAfterRelease = false; guarded.setup();
        guarded.step(true); guarded.until(State::carrying);
        guarded.step(false, true); guarded.until(State::releaseWait);
        guarded.lift = -cfg::returnLiftRotations;
        guarded.driverAt(guarded.in.now + cfg::releasePauseMs);
        guarded.in.forwardPodDegrees += cfg::releaseReturnPodDegrees;
        guarded.in.armVelocity = ineligible == 0 ? cfg::stallVelocityRpm + 1 : 0;
        guarded.in.liftVelocity = ineligible == 1 ? cfg::stallVelocityRpm + 1 : 0;
        if (ineligible == 2) guarded.manuallyPositionArm(guarded.maxArm);
        guarded.driverAt(guarded.in.now + 10);
        require(guarded.controller.state == State::returnArmClearance,
                "a moving mechanism or an arm beyond clearance cannot take the direct-return shortcut");
        guarded.until(State::prepared);
    }

    // The encoder origin is arbitrary: startup may tare at any lift height.
    // Exercise the exact production driver path with negative and positive zeros.
    for (double origin : {0.0, -5.0, 2.4, 7.0}) {
        Rig heights; heights.liftOffset = origin; heights.lift += origin; heights.setup();
        const std::array<double, 2> references{heights.controller.liftMeasurements[0], heights.controller.liftMeasurements[1]};
        heights.step(true, false); heights.until(State::carrying);
        const double base = heights.lift;
        for (int i = 0; i < 300; ++i) {
            heights.step();
            require(!heights.controller.output.lift && heights.lift == base,
                    "R1 alone must never start a second lift move or an endstop seek");
        }
        require(std::abs(base - (origin + liftPickup)) < 0.03, "R1 pickup is only 0.2 rotations above calibrated low");
        heights.height(1, origin + liftLevel1);
        heights.height(1, origin + liftLevel2);
        const double raised = heights.lift;
        for (State destination : {State::carryingEndstop, State::carrying}) {
            heights.step(true, false);
            for (int i = 0; i < 1500 && heights.controller.state != destination; ++i) {
                heights.step();
                require(heights.lift == raised && !heights.controller.output.lift,
                        "R1 reverses only the arm at lift level two; it must not lower or rehome the lift");
            }
            require(heights.controller.state == destination, "arm toggle reaches the alternate pose at raised lift height");
        }
        // The physical stop can differ slightly from the calibrated contact.
        heights.liftExtraTop = 0.04;
        heights.height(1, origin + 0.04);
        require(heights.lift == origin + 0.04, "the final B step reaches actual contact instead of a rounded rotation cap");
        heights.height(-1, origin + 0.04 - cfg::driverLiftStepRotations);
        heights.height(-1, origin + 0.04 - 2 * cfg::driverLiftStepRotations);
        heights.height(-1, origin + liftLow);
        const double bottom = heights.lift;
        heights.step(false, false, -1);
        require(!heights.controller.output.lift && heights.lift == bottom, "a lower-level request cannot move below the calibrated bottom");
        heights.height(1, origin + liftLow + cfg::driverLiftStepRotations);
        require(std::abs(heights.lift - bottom - cfg::driverLiftStepRotations) < 0.05,
                "the first B step from the bottom is a full step, not the 0.2 pickup clearance");
        heights.height(1, origin + liftLow + 2 * cfg::driverLiftStepRotations);
        const double releaseHeight = heights.lift;
        heights.step(false, true); heights.until(State::releaseWait);
        require(heights.lift == releaseHeight, "R2 releases at the selected height before the automatic descent");
        heights.until(State::prepared);
        require(std::abs(heights.lift - (origin + liftLow)) < 0.03 && !heights.controller.output.closed,
                "automatic return uses the normal bottom of the manual levels, with clamp open");
        require(heights.controller.liftMeasurements[0] == references[0]
                && heights.controller.liftMeasurements[1] == references[1],
                "R1, height selection, endstop contact and release never rewrite preMatch measurements");
        heights.step(true, false); heights.until(State::carrying);
        require(std::abs(heights.lift - (origin + liftPickup)) < 0.03, "the next pickup resets correctly after release");
    }

    Rig overload; overload.setup(); overload.step(true, false); overload.until(State::carrying);
    overload.forcedLiftPeakCurrent = 2500;
    overload.step(false, false, 1); overload.until(State::stopped);
    const double overloadPosition = overload.lift;
    overload.forcedLiftPeakCurrent = -1;
    for (int i = 0; i < 300; ++i) {
        overload.step();
        require(!overload.controller.output.lift && overload.lift == overloadPosition,
                "lift overload stays stopped after current drops; it must not automatically retry");
    }
    overload.height(-1, liftLow);
    require(overload.controller.calibrated, "a lower-level request recovers from lift overload without recalibrating");

    Rig jammedTop; jammedTop.setup(); jammedTop.step(true, false); jammedTop.until(State::carrying);
    jammedTop.height(1, liftLevel1); jammedTop.height(1, liftLevel2);
    jammedTop.liftCeiling = -0.1;
    jammedTop.step(false, false, 1); jammedTop.until(State::stopped);
    require(std::strstr(jammedTop.controller.notice, "ostacolo") && jammedTop.controller.calibrated,
            "contact below the calibrated top is an obstruction, not a new upper reference");
    for (int i = 0; i < 100; ++i) {
        jammedTop.step(); require(!jammedTop.controller.output.lift, "endstop obstruction must remain stopped");
    }
    jammedTop.height(-1, jammedTop.lift - cfg::driverLiftStepRotations);

    Rig topTimeout; topTimeout.setup(); topTimeout.step(true, false); topTimeout.until(State::carrying);
    topTimeout.height(1, liftLevel1); topTimeout.height(1, liftLevel2);
    topTimeout.liftSpeedScale = 0; topTimeout.forcedLiftCurrent = 100;
    topTimeout.step(false, false, 1); topTimeout.until(State::stopped);
    require(std::strstr(topTimeout.controller.notice, "timeout") && !topTimeout.controller.output.lift,
            "a final step without movement/contact has a deadline and cannot power the lift forever");

    Rig heightDropout; heightDropout.setup(); heightDropout.step(true, false); heightDropout.until(State::carrying);
    heightDropout.step(false, false, 1); heightDropout.in.valid = false;
    for (int i = 0; i < 30; ++i) {
        heightDropout.step(); require(!heightDropout.controller.output.lift, "missing telemetry pauses a selected lift move");
    }
    heightDropout.in.valid = true;
    for (int i = 0; i < 100; ++i) {
        heightDropout.step(); require(!heightDropout.controller.output.lift, "recovered sensors must not restart a long-paused height move");
    }
    heightDropout.height(-1, liftLow);

    Rig interruptedHeight; interruptedHeight.setup(); interruptedHeight.step(true, false); interruptedHeight.until(State::carrying);
    interruptedHeight.step(false, false, 1); interruptedHeight.controller.cancel();
    for (int i = 0; i < 100; ++i) {
        interruptedHeight.step(); require(!interruptedHeight.controller.output.lift, "cancel must also cancel the independent height request");
    }

    for (bool duringDescent : {false, true}) {
        Rig slipped; slipped.start(); slipped.until(State::preLiftLow);
        if (duringDescent) {
            slipped.step();
            require(slipped.controller.output.lift < 0, "extra-low startup descent starts at endstop 2");
        }
        slipped.arm -= cfg::repeatArmToleranceDeg + 1;
        const double stoppedLift = slipped.lift;
        slipped.step();
        require(slipped.controller.state == State::fault && !slipped.controller.calibrated
                && !slipped.controller.output.lift && slipped.lift == stoppedLift
                && slipped.controller.output.closed && slipped.motor.braking && !slipped.motor.holding(),
                "preMatch stops before or during extra-low descent if the arm leaves endstop 2");
    }
    Rig blockedStartup; blockedStartup.start(); blockedStartup.until(State::preLiftLow);
    blockedStartup.liftFloor = liftLow;
    blockedStartup.until(State::fault);
    require(!blockedStartup.controller.calibrated && !blockedStartup.controller.output.lift
            && std::strstr(blockedStartup.controller.error, "Lift bloccato"),
            "a blocked extra-low startup descent stops effort and cannot declare READY");

    for (int polarity : {-1, 1}) for (double load : {8.0, 55.0}) {
        Rig supported; supported.encoderPolarity = polarity;
        supported.start(); supported.until(State::armTop2);
        supported.armLoadPower = load;
        supported.until(State::liftTop1);
        require(supported.motor.holding() && !supported.controller.output.armCoast,
                "the final measured upper contact engages HOLD immediately before descent");
        while (supported.controller.state != State::prepared && supported.controller.state != State::fault) {
            supported.step();
            require(supported.arm == supported.maxArm && supported.motor.holding()
                    && supported.controller.output.closed,
                    "gravity cannot pull the arm off endstop 2 while the startup lift descends");
        }
        require(supported.controller.state == State::prepared && supported.controller.calibrated
                && std::abs(supported.lift - liftPreMatch) < 0.03,
                "loaded startup descent completes at the requested extra-low lift height");
        for (int i = 0; i < 100; ++i) supported.step();
        require(supported.arm == supported.maxArm && supported.motor.holding(),
                "the loaded arm remains at endstop 2 while waiting in READY");
        supported.controller.suspend();
        applyArmMotor(supported.motor, supported.controller.output, supported.controller.state, false);
        require(supported.motor.braking && !supported.motor.holding(),
                "disabled startup support must not apply active motor HOLD");
        supported.controller.resume(supported.in.now);
        supported.step();
        require(supported.motor.holding() && supported.arm == supported.maxArm,
                "driver reentry restores support for the low startup pose");
        supported.step(false, true);
        require(supported.controller.state == State::releaseWait && !supported.controller.output.closed
                && supported.motor.holding() && supported.arm == supported.maxArm,
                "R2 at the low startup pose opens in place and retains support");
        supported.until(State::returnLiftClearance);
        while (supported.controller.state == State::returnLiftClearance) {
            supported.step();
            require(supported.arm == supported.maxArm && supported.motor.holding(),
                    "R2 keeps the loaded arm supported until the lift regains clearance");
        }
        supported.armLoadPower = 0;
        supported.until(State::prepared);
        require(supported.arm == 0 && supported.motor.coasting(),
                "the subsequent normal R2 return still releases its confirmed front endstop");
    }

    for (double origin : {-5.0, 0.0, 7.0}) for (bool upper : {false, true}) {
        Rig autonLift; autonLift.liftOffset = origin; autonLift.lift += origin;
        if (upper) autonLift.preMatch();
        else autonLift.setup();
        autonLift.controller.cancel(); // Autonomous takes over the calibrated mechanisms.
        const bool closed = autonLift.controller.output.closed;
        const std::array<double, 2> references{autonLift.controller.liftMeasurements[0], autonLift.controller.liftMeasurements[1]};
        autonLift.lift = origin + liftLevel1; autonLift.in.lift = autonLift.lift;
        require(autonLift.controller.requestLiftBottom(autonLift.in),
                "autonomous can select the calibrated bottom without a manual button press");
        for (unsigned i = 0; i < 1000 && autonLift.controller.driverLiftMoving(); ++i) autonLift.step();
        const double expectedBottom = origin + (upper ? liftPreMatch : liftLow);
        require(!autonLift.controller.driverLiftMoving() && !autonLift.controller.output.lift
                && std::abs(autonLift.lift - expectedBottom) < 0.03
                && autonLift.controller.output.closed == closed,
                "full lowering uses the extra-low limit only at endstop 2 and preserves the clamp");
        const double bottom = autonLift.lift;
        autonLift.controller.cancelDriverLiftTarget();
        autonLift.controller.adjustDriverLift(1, autonLift.in);
        for (unsigned i = 0; i < 1000 && autonLift.controller.driverLiftMoving(); ++i) autonLift.step();
        require(std::abs(autonLift.lift - bottom - cfg::driverLiftStepRotations) < 0.03,
                "autonomous raises one full configured step from the current lift position");
        require(autonLift.controller.requestLiftBottom(autonLift.in), "autonomous can lower again after its upward step");
        for (unsigned i = 0; i < 1000 && autonLift.controller.driverLiftMoving(); ++i) autonLift.step();
        require(std::abs(autonLift.lift - expectedBottom) < 0.03
                && autonLift.controller.liftMeasurements[0] == references[0]
                && autonLift.controller.liftMeasurements[1] == references[1],
                "autonomous lift moves return to the calibrated bottom without rewriting references");
    }

    RotationReadout rotation;
    Input podSample;
    readForwardPodInput(podSample, static_cast<RotationReadout*>(nullptr));
    require(!std::isfinite(podSample.forwardPodDegrees), "unconfigured pod cannot authorize return");
    rotation.position = -36000;
    readForwardPodInput(podSample, &rotation);
    require(podSample.forwardPodDegrees == -360, "odometry pod centidegrees convert to continuous signed degrees");
    podSample = Input{}; rotation.position = PROS_ERR;
    readForwardPodInput(podSample, &rotation);
    require(!std::isfinite(podSample.forwardPodDegrees) && podSample.valid,
            "missing odometry pod blocks return without preventing clamp release or mechanism calibration");
    rotation.position = 91000;
    ArmReadout armReadout;
    LiftReadout liftReadout;
    auto sample = readMechanismInput(rotation, armReadout, liftReadout, 123, 2);
    require(sample.valid && sample.arm == 130 && sample.lift == -3 && sample.now == 123, "hardware input conversions");
    liftReadout.positions[1] += 0.3;
    sample = readMechanismInput(rotation, armReadout, liftReadout, 124, 2);
    require(sample.valid && sample.warning[0], "valid lift encoder offsets are advisory, not a sensor fault");
    liftReadout.currents[1] = PROS_ERR;
    sample = readMechanismInput(rotation, armReadout, liftReadout, 125, 2);
    require(!sample.valid && std::strcmp(sample.problem, "Lift 2 non disponibile") == 0,
            "a missing second lift motor is not hidden in the average");
    liftReadout.currents[1] = 200; rotation.position = PROS_ERR;
    sample = readMechanismInput(rotation, armReadout, liftReadout, 126, 2);
    require(!sample.valid && std::strcmp(sample.problem, "Rotation braccio assente") == 0, "missing Rotation identified");
    rotation.position = 91000; armReadout.velocity = std::numeric_limits<double>::infinity();
    sample = readMechanismInput(rotation, armReadout, liftReadout, 127, 2);
    require(!sample.valid && std::strcmp(sample.problem, "Motore braccio non disponibile") == 0, "invalid arm telemetry identified");
    std::cout << "Mechanism manual grab sequences passed\n";
}
