#pragma once
#include "robot/mechanism_config.hpp"
#include "robot/arm_progress.hpp"
#include "robot/arm_motion.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace robot::mechanism {
namespace cfg = config::mechanism;
struct Input {
    std::uint32_t now = 0;
    double arm = 0, lift = 0; // continuous arm degrees; direction-corrected motor rotations
    double forwardPodDegrees = std::numeric_limits<double>::quiet_NaN(); // Continuous odometry wheel position.
    double armCurrent = 0, armVelocity = 0, liftCurrent = 0, liftPeakCurrent = 0, liftVelocity = 0;
    int proximity = 0;
    double hue = 0, saturation = 0;
    bool valid = true;
    const char* problem = "sensore/motore non disponibile";
    const char* warning = "";
};
struct Output {
    int arm = 0, lift = 0, intake = 0;
    bool closed = false, armProfileActive = false, armHold = false, armBrake = false;
    bool armCoast = false; // Retain after an accepted endstop contact until the next arm movement.
};
enum class State {
    off, fault, preLiftStart, armZero1, armBackoff0, armZero2, armTop1, armBackoffTop,
    armTop2, preArmClearance, liftTop1, liftBackoff, liftTop2, preLiftLow,
    preArmZero, prepared, collecting, classifyFirst, classifySecond, capture,
    pickupWait, pickupLift, pickupArm, carrying, releaseArm90, releaseWait, returnArmClearance,
    returnLiftLow, returnArmZero, reversing, clearSensor, pickupEndstop, carryingEndstop, stopped,
    returnLiftClearance, returnLiftBottom, releaseClampWait
};
inline const char* phaseLabel(State state) {
    switch (state) {
    case State::preLiftStart: return "Reset lift: anticipo di 1 secondo";
    case State::armZero1: return "Braccio: primo zero";
    case State::armBackoff0: return "Braccio: distacco dallo zero";
    case State::armZero2: return "Braccio: secondo zero";
    case State::armTop1: return "Braccio: primo finecorsa alto";
    case State::armBackoffTop: return "Braccio: distacco dall'alto";
    case State::armTop2: return "Braccio: secondo finecorsa alto";
    case State::preArmClearance: return "Braccio: posizione di passaggio";
    case State::liftTop1: return "Braccio pronto: attesa reset lift";
    case State::liftBackoff: return "Lift: distacco dall'alto";
    case State::liftTop2: return "Lift: secondo finecorsa alto";
    case State::preLiftLow: return "Lift: discesa";
    case State::preArmZero: return "Braccio: ritorno a zero";
    case State::prepared: return "PreMatch completato";
    case State::fault: return "Errore meccanismi";
    default: return "Macro meccanismi";
    }
}
class Controller {
public:
    State state = State::off;
    Output output;
    bool calibrated = false; // Valid arm/lift references, independent of macro ownership.
    int encoderSign = 1; // Learned from the first positive, open-loop backoff.
    bool encoderDirectionKnown = false;
    const char* error = "";
    const char* notice = "";
    bool endstopSelected = false;
    double armTargetDegrees = 0; // Last arm target, retained for fault diagnostics.
    double zeroMeasurements[2]{}, topMeasurements[2]{}, liftMeasurements[2]{};
    void cancel() {
        // Stopping motion does not erase a completed homing measurement. Keep
        // the load supported away from endstops, retaining COAST at a confirmed
        // endstop, and require a new button press to start any macro.
        output.armHold = calibrated || output.armHold;
        state = State::off; releaseRequested = false; preMatchLiftActive = false;
        releasePodTracking = false;
        driverLiftActive = driverLiftTiming = driverLiftPaused = false;
        driverLiftTargetKnown = false;
        sensorPaused = false; notice = "";
        resetDriverIntake();
        output.arm = output.lift = output.intake = 0; output.armProfileActive = output.armBrake = false;
    }
    void suspend() { cancel(); }
    void resume(std::uint32_t now) { if (calibrated) enter(State::prepared, now); }
    void start(const Input& in) {
        cancel(); calibrated = false; output.armHold = output.armCoast = false;
        error = ""; pinSeen = false; present = false; edge = false;
        grabSelected = endstopSelected = false;
        encoderSign = 1; encoderDirectionKnown = false;
        enter(State::preLiftStart, in.now);
        preMatchLiftActive = true;
        enterPreMatchLift(State::liftTop1, in.now);
        output.lift = cfg::liftHomePower; output.armHold = true;
    }
    void collect(const Input& in) {
        grab(in);
    }
    void grab(const Input& in) {
        if (!calibrated) return;
        resetDriverIntake();
        // Leaving endstop 2 cancels any pending descent below baseline. The
        // usual pickup interlock below will raise first if already too low.
        if (driverLiftTargetKnown && driverLiftTarget < liftLow()) {
            driverLiftTargetKnown = false;
            driverLiftActive = driverLiftTiming = driverLiftPaused = false;
        }
        endstopSelected = grabSelected ? !endstopSelected : false;
        grabSelected = true;
        releaseRequested = false;
        releasePodTracking = false;
        output.arm = output.lift = output.intake = 0;
        output.armProfileActive = output.armBrake = false;
        output.armHold = true;
        output.closed = true;
        notice = "";
        // Each R1 edge closes the clamp and starts a full pause, including
        // retargets during motion. A selected driver height resumes afterward
        // with fresh timing/progress observers rather than expiring in the pause.
        driverLiftTiming = driverLiftPaused = driverLiftContact = false;
        liftProgressTracking = liftOverloadTracking = false;
        enter(State::pickupWait, in.now);
    }
    bool release(const Input& in) {
        if (!calibrated) return false;
        if (state == State::prepared || state == State::off || state == State::stopped
            || state == State::returnLiftLow || state == State::returnArmZero) {
            // R2 also works after switching to manual with valid references.
            // Release in place at endstop 2 even below the normal low position.
            // Otherwise reuse the lift interlock before moving the arm to 90.
            output.intake = 0;
            resetDriverIntake();
            trackReleasePod(in);
            driverLiftActive = driverLiftTiming = false;
            if ((in.valid && atArmUpperEndstop(armPosition(in.arm)))
                || in.lift >= liftBase() - cfg::liftToleranceRot)
                beginRelease(in.now, armPosition(in.arm));
            else {
                releaseRequested = true;
                enter(State::pickupLift, in.now);
            }
            return true;
        }
        if (state == State::pickupWait || state == State::pickupLift) {
            // Keep even a brief R2 press, but finish the clamp pause and lift
            // interlock before swinging the arm. No second press is needed.
            releaseRequested = true;
            resetDriverIntake();
            trackReleasePod(in);
            driverLiftActive = driverLiftTiming = false;
            return true;
        }
        if (state == State::pickupArm || state == State::carrying
            || state == State::pickupEndstop || state == State::carryingEndstop
            || state == State::returnArmClearance) {
            // The lift is already raised: preempt either R1 destination.
            trackReleasePod(in);
            driverLiftActive = driverLiftTiming = false;
            beginRelease(in.now, armPosition(in.arm));
            return true;
        }
        return false;
    }
    bool owns() const { return state != State::off; }
    bool releasePending() const { return releaseRequested; }
    void adjustDriverLift(int direction, const Input& in) {
        if (!driverLiftAvailable() || !direction) return;
        if (!in.valid || !std::isfinite(in.lift)) return;
        // Queue full steps from the last requested height, including quick
        // repeated presses. The pickup clearance is not a driver step.
        const double previousTarget = driverLiftTargetKnown ? driverLiftTarget : in.lift;
        const double lower = previousTarget <= liftLow() + cfg::liftToleranceRot && extraLiftLowAvailable(in)
            ? liftLow() - cfg::liftEndstopExtraLowerRotations : liftLow();
        const double nextTarget = std::clamp(previousTarget + (direction > 0 ? 1 : -1)
            * cfg::driverLiftStepRotations, lower, liftTop);
        if (std::abs(nextTarget - previousTarget) > cfg::liftToleranceRot) {
            driverLiftTarget = nextTarget; driverLiftTargetKnown = true;
            driverLiftActive = true; driverLiftTiming = driverLiftContact = false;
            driverLiftPaused = false;
            liftOverloadTracking = liftProgressTracking = stable = false;
        }
    }
    void cancelDriverLiftTarget() {
        driverLiftActive = driverLiftTiming = driverLiftPaused = false;
        driverLiftTargetKnown = false;
        liftProgressTracking = liftOverloadTracking = false;
        output.lift = 0;
    }
    bool driverLiftAvailable() const {
        return calibrated && (state == State::prepared || state == State::off
            || state == State::pickupArm || state == State::pickupEndstop
            || state == State::carrying || state == State::carryingEndstop || state == State::stopped);
    }
    void updateDriverLift(const Input& input) {
        if (!driverLiftAvailable() || !driverLiftActive) return;
        if (!input.valid || !std::isfinite(input.lift) || !std::isfinite(input.liftPeakCurrent)) {
            output.lift = 0; liftOverloadTracking = driverLiftContact = false;
            if (!driverLiftPaused) { driverLiftPaused = true; driverLiftPauseSince = input.now; }
            if (input.now - driverLiftPauseSince >= cfg::sensorRecoveryMs) fail(input.problem);
            return;
        }
        // Height selection also works while the arm is stopped or in manual
        // mode, where tick() does not run its macro sensor-pause handling.
        if (driverLiftPaused) {
            const auto pause = input.now - driverLiftPauseSince;
            if (pause >= cfg::sensorRecoveryMs) { fail("Lift: dati non disponibili"); return; }
            if (driverLiftTiming) driverLiftSince += pause;
            driverLiftPaused = false; liftProgressTracking = stable = false;
        }
        if (!driverLiftTiming) { driverLiftTiming = true; driverLiftSince = input.now; }
        if (input.now - driverLiftSince > cfg::motionTimeoutMs) {
            fail("Lift: timeout altezza"); return;
        }
        // This contact seek is entered only by B selecting the final level.
        // It never changes the calibration references or runs after R1 alone.
        if (driverLiftTarget >= liftTop) {
            const bool contact = input.liftCurrent >= cfg::stallCurrentMa
                && std::abs(input.liftVelocity) <= cfg::stallVelocityRpm;
            if (!contact) driverLiftContact = false;
            else if (!driverLiftContact) { driverLiftContact = true; driverLiftContactSince = input.now; }
            if (driverLiftContact && input.now - driverLiftContactSince >= cfg::liftEndstopContactMs) {
                if (input.lift < liftTop - cfg::repeatLiftToleranceRot) {
                    fail("Lift: ostacolo prima del finecorsa"); return;
                }
                driverLiftTarget = input.lift;
                driverLiftActive = false; output.lift = 0; return;
            }
            if (liftOverloaded(input)) return;
            output.lift = cfg::liftHomePower;
            return;
        }
        if (driverLiftTarget < liftLow() && !extraLiftLowAvailable(input)) {
            fail("Lift basso: serve finecorsa 2"); return;
        }
        if (liftTo(input, driverLiftTarget)) driverLiftActive = false;
    }
    double armPosition(double rawEncoderDegrees) const { return rawEncoderDegrees * encoderSign - zero; }
    void setDriverIntake(bool enabled) { driverIntakeForward = enabled; }
    void updateDriverIntake(const Input& in, bool xHeld, bool xPressed) {
        const bool manual = state == State::off || state == State::prepared;
        const bool valid = in.valid && std::isfinite(in.arm) && std::isfinite(in.lift);
        const bool low = calibrated && valid && state == State::prepared
            && std::abs(armPosition(in.arm)) <= cfg::armToleranceDeg
            && std::abs(in.lift - liftLow()) <= cfg::liftToleranceRot;
        // Restart once upon reaching the complete low pose. A short X press
        // can still stop intake there without the next tick undoing it.
        if (low && !driverIntakeWasLow) driverIntakeForward = true;
        driverIntakeWasLow = low;
        if (!valid || state == State::fault || state == State::stopped
            || (!manual && !calibrated)) {
            driverIntakeTracking = false;
            output.intake = 0;
            return;
        }
        if (xHeld && xPressed) {
            driverIntakeTracking = true;
            driverIntakeSince = in.now;
        }
        if (driverIntakeTracking && !xHeld) {
            if (in.now - driverIntakeSince >= cfg::intakeReverseHoldMs)
                driverIntakeForward = true;
            else if (manual) driverIntakeForward = !driverIntakeForward;
            driverIntakeTracking = false;
        }
        if (state == State::pickupWait) { output.intake = 0; return; }
        if (manual) output.intake = 0;
        if (driverIntakeForward) output.intake = 127;
        if (driverIntakeTracking && xHeld
            && in.now - driverIntakeSince >= cfg::intakeReverseHoldMs)
            output.intake = -127;
    }
    void tick(const Input& rawInput) {
        Input in = rawInput;
        in.arm *= encoderSign;
        output.arm = output.lift = output.intake = 0;
        // A driver fault must keep supporting the load on every subsequent tick.
        // fail() selects HOLD for calibrated operation, passive brake for homing.
        if (state == State::fault) return;
        output.armProfileActive = output.armHold = output.armBrake = false;
        if (state == State::off) return;
        if (state == State::stopped) { output.armHold = true; return; }
        if (!in.valid || !std::isfinite(in.arm) || !std::isfinite(in.lift)) {
            if (!calibrated) { fail(in.problem); return; }
            // Never drive from a stale/invalid sample. A short communication
            // dropout pauses the same request without losing the homing data.
            output.armHold = true; notice = in.problem;
            if (!sensorPaused) { sensorPaused = true; sensorPauseSince = in.now; }
            if (in.now - sensorPauseSince >= cfg::sensorRecoveryMs) fail(in.problem);
            return;
        }
        if (sensorPaused) {
            if (in.now - sensorPauseSince >= cfg::sensorRecoveryMs) { fail(notice); return; }
            // Restart control/settling observers after a short pause. Preserve
            // elapsed motion time, excluding only the pause itself.
            const auto motionSince = since + (in.now - sensorPauseSince);
            enter(state, in.now); since = motionSince; sensorPaused = false;
        }
        notice = in.warning;
        if (releasePodTracking) {
            // A missing sample cannot authorize return. Rebase after recovery
            // so a reconnect/reset cannot look like a completed wheel turn.
            if (!std::isfinite(in.forwardPodDegrees) || !std::isfinite(releasePodStart))
                releasePodStart = in.forwardPodDegrees;
        }
        const auto elapsed = in.now - since;
        const bool timed = state >= State::preLiftStart && state <= State::preArmZero;
        const bool moving = state == State::pickupLift || state == State::pickupArm
            || state == State::releaseArm90 || state == State::returnArmClearance
            || state == State::returnLiftLow || state == State::returnArmZero || state == State::pickupEndstop
            || state == State::returnLiftClearance || state == State::returnLiftBottom;
        if ((timed || moving) && elapsed > cfg::motionTimeoutMs) { fail("timeout movimento"); return; }
        if (preMatchLiftActive) {
            updatePreMatchLift(in);
            if (state == State::fault) return;
        }
        const double low = liftLow();
        switch (state) {
        case State::preLiftStart:
            // Lift homing is already running on its own clock. Start the arm
            // after one second, even if the lift has not finished yet.
            if (elapsed >= cfg::preMatchArmDelayMs) {
                enter(State::armZero1, in.now);
                if (home(in, true, -1)) { zeroMeasurements[0] = in.arm; zero = in.arm; coastArm(); enter(State::armBackoff0, in.now); }
            } else output.armHold = true;
            break;
        case State::armZero1:
            if (home(in, true, -1)) { zeroMeasurements[0] = in.arm; zero = in.arm; coastArm(); enter(State::armBackoff0, in.now); }
            break;
        case State::armBackoff0:
            // Motor-positive is the known way away from the first endstop.
            // Do not use feedback before observing the encoder's actual polarity.
            output.arm = cfg::armBackoffPower;
            if (!armProgressTracking) {
                armProgressTracking = true; armProgressAnchor = in.arm; armProgressSince = in.now;
            }
            if (std::abs(in.arm - armProgressAnchor) >= cfg::armProgressDeg) {
                armProgressAnchor = in.arm; armProgressSince = in.now;
            } else if (in.now - armProgressSince >= cfg::armBlockedMs) {
                fail("Distacco: braccio o encoder fermo"); break;
            }
            if (std::abs(in.arm - zero) >= cfg::armBackoffDeg) {
                encoderSign = in.arm > zero ? 1 : -1;
                encoderDirectionKnown = true;
                zero *= encoderSign;
                zeroMeasurements[0] *= encoderSign;
                output.arm = 0;
                enter(State::armZero2, in.now);
            }
            break;
        case State::armZero2:
            if (home(in, true, -1)) {
                zeroMeasurements[1] = in.arm;
                if (std::abs(in.arm - zeroMeasurements[0]) > cfg::repeatArmToleranceDeg) {
                    fail("zero braccio non ripetibile"); break;
                }
                zero = (in.arm + zeroMeasurements[0]) / 2; coastArm(); enter(State::armTop1, in.now);
            }
            break;
        case State::armTop1:
            if (home(in, true, 1)) { topMeasurements[0] = in.arm; coastArm(); enter(State::armBackoffTop, in.now); }
            break;
        case State::armBackoffTop:
            // A homing backoff needs a measured clearance, not precise settling.
            // Constant low power avoids losing the torque needed to overcome friction.
            output.arm = -cfg::armBackoffPower;
            if (topMeasurements[0] - in.arm >= cfg::armBackoffDeg) {
                output.arm = 0; enter(State::armTop2, in.now); break;
            }
            if (!armProgressTracking) {
                armProgressTracking = true; armProgressAnchor = in.arm; armProgressSince = in.now;
            }
            if (in.arm - armProgressAnchor > cfg::armWrongDirectionDeg) {
                fail("Distacco alto: movimento nel verso opposto"); break;
            }
            if (armProgressAnchor - in.arm >= cfg::armProgressDeg) {
                armProgressAnchor = in.arm; armProgressSince = in.now;
            } else if (in.now - armProgressSince >= cfg::armBlockedMs) {
                fail("Distacco alto: braccio o encoder fermo");
            }
            break;
        case State::armTop2:
            if (home(in, true, 1)) {
                topMeasurements[1] = in.arm;
                if (std::abs(in.arm - topMeasurements[0]) > cfg::repeatArmToleranceDeg) {
                    fail("finecorsa braccio non ripetibile"); break;
                }
                if (std::min(in.arm, topMeasurements[0]) - zero < cfg::minimumArmTravelDeg) {
                    fail("corsa braccio insufficiente per i target"); break;
                }
                coastArm(); enter(State::preArmClearance, in.now);
            }
            break;
        case State::preArmClearance:
            if (armTo(in, cfg::armClearanceDeg)) enter(State::liftTop1, in.now);
            break;
        case State::liftTop1:
            holdArm(cfg::armClearanceDeg);
            if (!preMatchLiftActive) enter(State::preLiftLow, in.now);
            break;
        case State::preLiftLow:
            holdArm(cfg::armClearanceDeg);
            if (liftTo(in, low)) enter(State::preArmZero, in.now);
            break;
        case State::preArmZero:
            if (home(in, true, -1)) {
                zero = in.arm; calibrated = true; output.closed = false;
                driverLiftTarget = low; driverLiftTargetKnown = true;
                coastArm(); enter(State::prepared, in.now);
            }
            break;
        case State::prepared: break;
        case State::collecting:
            // Retained as a harmless compatibility state; R1 now starts grab()
            // directly, so object counting and proximity are no longer used.
            output.intake = 0;
            break;
        case State::capture:
            output.intake = 127;
            if (elapsed >= cfg::captureMs) { output.intake = 0; output.closed = true; enter(State::pickupLift, in.now); }
            break;
        case State::pickupWait:
            output.armHold = true;
            if (elapsed >= cfg::pickupClampPauseMs) {
                if (in.lift < liftBase() - cfg::liftToleranceRot) {
                    driverLiftActive = driverLiftTiming = driverLiftPaused = false;
                    enter(State::pickupLift, in.now);
                } else if (releaseRequested) beginRelease(in.now, in.arm - zero);
                else enter(pickupArmState(), in.now);
            }
            break;
        case State::pickupLift:
            output.armHold = true;
            if (liftTo(in, low + cfg::pickupLiftRotations)) {
                driverLiftTarget = liftBase(); driverLiftTargetKnown = true;
                if (releaseRequested) beginRelease(in.now, in.arm - zero);
                else enter(pickupArmState(), in.now);
            }
            break;
        case State::pickupArm:
            if (armTo(in, cfg::armPickupDeg)) enter(State::carrying, in.now);
            break;
        case State::carrying: holdArm(cfg::armPickupDeg);
            break;
        case State::pickupEndstop:
            armTargetDegrees = (topMeasurements[0] + topMeasurements[1]) / 2 - zero;
            if (!endstopContact && armTargetDegrees - (in.arm - zero) > cfg::armEndstopContactDeg) {
                armTo(in, armTargetDegrees);
                break;
            }
            endstopContact = true;
            if (home(in, true, 1, cfg::armEndstopContactPower)) {
                if (std::abs(in.arm - zero - armTargetDegrees) > cfg::repeatArmToleranceDeg) {
                    fail("Finecorsa alto non raggiunto"); break;
                }
                coastArm();
                enter(State::carryingEndstop, in.now);
            }
            break;
        case State::carryingEndstop:
            coastArm();
            break;
        case State::releaseArm90:
            if (releaseArmFast(in)) enter(State::releaseClampWait, in.now);
            break;
        case State::releaseClampWait:
            output.armHold = true;
            if (elapsed >= cfg::releaseClampPauseMs) {
                output.closed = false;
                enter(State::releaseWait, in.now);
            }
            break;
        case State::releaseWait:
            // Hold where the fast move stopped; do not spend time correcting
            // its deliberately loose release tolerance with the PID.
            output.armHold = true;
            if (elapsed >= cfg::releasePauseMs && releasePodTracking
                && std::isfinite(in.forwardPodDegrees) && std::isfinite(releasePodStart)
                && std::abs(in.forwardPodDegrees - releasePodStart) >= cfg::releaseReturnPodDegrees)
                enter(in.lift < low - cfg::liftToleranceRot
                    ? State::returnLiftClearance : State::returnArmClearance, in.now);
            break;
        case State::returnLiftClearance:
            // Below baseline the arm may stay at endstop 2, but must not swing
            // back until the lift has recovered the usual pickup clearance.
            output.armHold = true;
            if (liftTo(in, liftBase())) {
                driverLiftTargetKnown = false;
                enter(State::returnArmClearance, in.now);
            }
            break;
        case State::returnArmClearance:
            if (armTo(in, cfg::armClearanceDeg)) enter(State::returnLiftLow, in.now);
            break;
        case State::returnLiftLow:
            holdArm(cfg::armClearanceDeg);
            // Set the requested passage height from either direction before
            // allowing the arm to reach its front endstop.
            if (liftTo(in, liftTop - cfg::returnLiftRotations)) enter(State::returnArmZero, in.now);
            break;
        case State::returnArmZero:
            if (home(in, true, -1)) {
                zero = in.arm; output.closed = false; pinSeen = false; present = false;
                grabSelected = endstopSelected = false;
                driverLiftActive = driverLiftTiming = false;
                coastArm();
                enter(State::returnLiftBottom, in.now);
            }
            break;
        case State::returnLiftBottom:
            coastArm();
            if (liftTo(in, low)) {
                driverLiftTarget = low; driverLiftTargetKnown = true;
                enter(State::prepared, in.now);
            }
            break;
        case State::reversing:
            output.intake = -127;
            if (elapsed >= cfg::reverseMs) { output.intake = 127; present = true; enter(State::clearSensor, in.now); }
            break;
        case State::clearSensor:
            output.intake = 127;
            arrival(in);
            if (!present) enter(State::collecting, in.now);
            break;
        default: break;
        }
        // Pauses and lift-only moves retain the endpoint release. A new arm
        // command restores the usual profile braking and intermediate HOLD.
        if (output.arm || output.armProfileActive) output.armCoast = false;
        if (output.armCoast) output.armHold = output.armBrake = false;
    }
private:
    std::uint32_t since = 0, stableSince = 0, edgeSince = 0, arrivalTime = 0;
    bool stable = false, edge = false, present = false, pinSeen = false;
    bool releaseRequested = false;
    bool releasePodTracking = false;
    double releasePodStart = std::numeric_limits<double>::quiet_NaN();
    bool grabSelected = false, sensorPaused = false;
    std::uint32_t sensorPauseSince = 0;
    int releaseDirection = 0;
    bool endstopContact = false;
    double zero = 0, liftTop = 0;
    bool preMatchLiftActive = false;
    State preMatchLiftState = State::liftTop1;
    std::uint32_t preMatchLiftSince = 0;
    ArmMotion armMotion;
    ArmProgress motionProgress;
    double armStillAnchor = 0, armProgressAnchor = 0;
    std::uint32_t armStillSince = 0, armProgressSince = 0;
    bool armStillTracking = false, armProgressTracking = false;
    double liftTarget = 0, liftProgressAnchor = 0;
    std::uint32_t liftProgressSince = 0;
    bool liftProgressTracking = false;
    std::uint32_t liftOverloadSince = 0;
    bool liftOverloadTracking = false;
    double driverLiftTarget = 0;
    bool driverLiftTargetKnown = false;
    bool driverLiftActive = false, driverLiftTiming = false, driverLiftContact = false;
    bool driverLiftPaused = false;
    std::uint32_t driverLiftSince = 0, driverLiftContactSince = 0, driverLiftPauseSince = 0;
    bool driverIntakeForward = false, driverIntakeWasLow = false, driverIntakeTracking = false;
    std::uint32_t driverIntakeSince = 0;
    void resetDriverIntake() {
        driverIntakeForward = driverIntakeWasLow = driverIntakeTracking = false;
    }
    State pickupArmState() const { return endstopSelected ? State::pickupEndstop : State::pickupArm; }
    // Encoder zero is arbitrary. All heights use the same calibrated origin,
    // including negative encoder coordinates after startup at an elevated pose.
    double liftLow() const { return liftTop - cfg::liftLowerRotations; }
    double liftBase() const { return liftLow() + cfg::pickupLiftRotations; }
    bool atArmUpperEndstop(double position) const {
        const double upper = (topMeasurements[0] + topMeasurements[1]) / 2 - zero;
        return calibrated && std::isfinite(position)
            && std::abs(position - upper) <= cfg::repeatArmToleranceDeg;
    }
    bool extraLiftLowAvailable(const Input& in) const {
        return (state == State::carryingEndstop || state == State::stopped)
            && in.valid && atArmUpperEndstop(armPosition(in.arm));
    }
    void trackReleasePod(const Input& in) {
        releasePodTracking = true;
        releasePodStart = in.forwardPodDegrees;
    }
    void beginRelease(std::uint32_t now, double position) {
        resetDriverIntake();
        driverLiftTargetKnown = false;
        const bool atEndstop = state == State::carryingEndstop || atArmUpperEndstop(position);
        if (atEndstop) {
            // R2 at the upper endstop releases in place, then reuses the same
            // pause/clearance/lift/zero return sequence, skipping the 90 move.
            coastArm();
            output.closed = false;
            enter(State::releaseWait, now);
        } else enter(State::releaseArm90, now);
    }
    void enter(State next, std::uint32_t now) {
        // A new destination requested during a data pause starts its own timer.
        // Recovery must not add an older pause onto this newer phase timestamp.
        if (sensorPaused) sensorPauseSince = now;
        state = next; since = now;
        if (!preMatchLiftActive) stable = false;
        if (next == State::releaseArm90 || next == State::releaseWait) releaseRequested = false;
        releaseDirection = 0;
        endstopContact = false;
        armStillTracking = armProgressTracking = false;
        armMotion.reset(); motionProgress.reset();
        if (!driverLiftActive && !preMatchLiftActive) liftProgressTracking = liftOverloadTracking = false;
    }
    void fail(const char* reason) {
        resetDriverIntake();
        output.armHold = calibrated; // Hold the current motor position, not the requested target.
        // Driver stops are immediately retargetable with R1/R2. Only an
        // incomplete/failed homing run needs a new preMatch.
        if (calibrated) { notice = reason; error = ""; state = State::stopped; }
        else { error = reason; state = State::fault; }
        sensorPaused = false;
        preMatchLiftActive = false;
        releaseRequested = false;
        releasePodTracking = false;
        // A lift overload must stay stopped after current drops. Only a new
        // height button edge (or an explicit macro request) may restart it.
        driverLiftActive = driverLiftTiming = driverLiftPaused = false;
        driverLiftTargetKnown = false;
        output.arm = output.lift = output.intake = 0; output.armProfileActive = output.armBrake = false;
    }
    bool dwell(bool condition, std::uint32_t now, std::uint32_t duration) {
        if (!condition) { stable = false; return false; }
        if (!stable) { stable = true; stableSince = now; }
        return now - stableSince >= duration;
    }
    bool home(const Input& in, bool arm, int direction, int armPower = cfg::armHomePower) {
        (arm ? output.arm : output.lift) = direction * (arm ? armPower : cfg::liftHomePower);
        const double current = arm ? in.armCurrent : in.liftCurrent;
        const double velocity = arm ? in.armVelocity : in.liftVelocity;
        bool hit;
        if (arm) {
            // Require effort AND a stationary external encoder, not current alone.
            // Restart the entire confirmation interval after any measurable movement.
            const auto graceMs = calibrated ? cfg::armDriverHomeGraceMs : cfg::homeGraceMs;
            const auto stillMs = calibrated ? cfg::armDriverHomeStillMs : cfg::armHomeStillMs;
            const bool loaded = in.now - since >= graceMs
                && current >= cfg::armHomeCurrentMa && std::abs(velocity) <= cfg::stallVelocityRpm;
            if (!loaded) armStillTracking = false;
            else if (!armStillTracking || std::abs(in.arm - armStillAnchor) > cfg::armStillToleranceDeg) {
                armStillTracking = true; armStillAnchor = in.arm; armStillSince = in.now;
            }
            hit = loaded && armStillTracking && in.now - armStillSince >= stillMs;
        } else {
            hit = dwell(in.now - preMatchLiftSince >= cfg::homeGraceMs && current >= cfg::stallCurrentMa
                && std::abs(velocity) <= cfg::stallVelocityRpm, in.now, cfg::stallMs);
        }
        if (hit) (arm ? output.arm : output.lift) = 0;
        return hit;
    }
    void enterPreMatchLift(State next, std::uint32_t now) {
        preMatchLiftState = next; preMatchLiftSince = now;
        stable = false; liftProgressTracking = liftOverloadTracking = false;
    }
    void updatePreMatchLift(const Input& in) {
        if (in.now - preMatchLiftSince > cfg::motionTimeoutMs) {
            fail("Lift: timeout reset"); return;
        }
        switch (preMatchLiftState) {
        case State::liftTop1:
            if (home(in, false, 1)) {
                liftMeasurements[0] = in.lift; liftTop = in.lift;
                enterPreMatchLift(State::liftBackoff, in.now);
            }
            break;
        case State::liftBackoff:
            if (liftTo(in, liftTop - cfg::liftBackoffRot))
                enterPreMatchLift(State::liftTop2, in.now);
            break;
        case State::liftTop2:
            if (home(in, false, 1)) {
                liftMeasurements[1] = in.lift;
                if (std::abs(in.lift - liftMeasurements[0]) > cfg::repeatLiftToleranceRot) {
                    fail("finecorsa lift non ripetibile"); return;
                }
                liftTop = (in.lift + liftMeasurements[0]) / 2;
                preMatchLiftActive = false;
                stable = false; liftProgressTracking = liftOverloadTracking = false;
            }
            break;
        default: break;
        }
    }
    void holdArm(double target) {
        armTargetDegrees = target;
        output.arm = 0; output.armHold = true;
    }
    void coastArm() {
        output.arm = 0; output.armProfileActive = output.armHold = output.armBrake = false;
        output.armCoast = true;
    }
    bool armTo(const Input& in, double target) {
        armTargetDegrees = target; output.armProfileActive = true;
        // Clearance is a passage before lowering the lift and homing the arm.
        // Accept its wider range from the first approach, so R2 does not spend
        // corrections chasing the normal 2-degree positioning tolerance.
        const double tolerance = !calibrated && state == State::preArmClearance
            ? cfg::armPreMatchClearanceToleranceDeg
            : state == State::returnArmClearance ? cfg::armReturnClearanceToleranceDeg
            : cfg::armToleranceDeg;
        const auto command = armMotion.update(in.now, in.arm - zero, target, tolerance);
        output.arm = command.power; output.armHold = command.hold; output.armBrake = command.brake;
        if (command.exhausted) {
            fail("Braccio: correzioni terminate fuori target"); return false;
        }
        const auto progress = motionProgress.update(in.now, in.arm, target - (in.arm - zero), output.arm,
            calibrated ? cfg::driverBlockedMs : cfg::armBlockedMs);
        if (progress == ArmProgress::Result::opposite) {
            fail("Braccio: movimento opposto al comando"); return false;
        }
        if (progress == ArmProgress::Result::blocked && (!calibrated
            || (in.armCurrent >= cfg::armHomeCurrentMa && std::abs(in.armVelocity) <= cfg::stallVelocityRpm))) {
            fail("Braccio fermo prima del target"); return false;
        }
        return command.arrived;
    }
    bool releaseArmFast(const Input& in) {
        armTargetDegrees = cfg::armReleaseDeg;
        const double errorNow = cfg::armReleaseDeg - (in.arm - zero);
        if (!releaseDirection) releaseDirection = errorNow >= 0 ? 1 : -1;
        // Stop once near or across 90 degrees. No PID settling, reversals or
        // derivative delay: R2 prioritizes speed and accepts a few degrees.
        if (errorNow * releaseDirection <= cfg::armReleaseToleranceDeg) {
            output.armHold = true; return true;
        }
        output.arm = releaseDirection * cfg::armReleasePower;
        const auto progress = motionProgress.update(in.now, in.arm, errorNow, output.arm, cfg::driverBlockedMs);
        if (progress == ArmProgress::Result::opposite || progress == ArmProgress::Result::blocked)
            fail("Braccio fermo: R1/R2 riprova");
        return false;
    }
    bool liftOverloaded(const Input& in) {
        if (in.liftPeakCurrent >= cfg::liftUpOverloadCurrentMa) {
            if (!liftOverloadTracking) {
                liftOverloadTracking = true; liftOverloadSince = in.now;
            } else if (in.now - liftOverloadSince >= cfg::liftUpOverloadMs) {
                fail("Sovraccarico lift durante salita"); return true;
            }
        } else liftOverloadTracking = false;
        return false;
    }
    bool liftTo(const Input& in, double target) {
        const double error = target - in.lift;
        const double gain = error > 0 ? cfg::liftUpKp : cfg::liftKp;
        const bool close = std::abs(error) <= cfg::liftToleranceRot;
        if (!close && error > 0) { if (liftOverloaded(in)) return false; }
        else liftOverloadTracking = false;
        if (target != liftTarget || !liftProgressTracking) {
            liftTarget = target; liftProgressAnchor = in.lift; liftProgressSince = in.now;
            liftProgressTracking = true;
        }
        if (std::abs(in.lift - liftProgressAnchor) >= cfg::liftProgressRotations) {
            liftProgressAnchor = in.lift; liftProgressSince = in.now;
        } else if (!close && in.now - liftProgressSince >= (calibrated ? cfg::driverBlockedMs : cfg::liftBlockedMs)
                   && in.liftCurrent >= cfg::liftBlockedCurrentMa
                   && std::abs(in.liftVelocity) <= cfg::liftBlockedVelocityRpm) {
            fail("Lift bloccato prima della posizione richiesta"); return false;
        }
        if (close) output.lift = 0;
        else if (error > 0) {
            const auto command = std::max(cfg::liftUpMinPower, static_cast<int>(gain * error));
            output.lift = std::clamp(command, -cfg::liftMaxPower, cfg::liftUpMaxPower);
        } else {
            output.lift = static_cast<int>(std::clamp(gain * error,
                -double(cfg::liftMaxPower), double(cfg::liftUpMaxPower)));
        }
        return dwell(close, in.now, cfg::settleMs);
    }
    bool arrival(const Input& in) {
        const bool change = present ? in.proximity <= cfg::proximityExit : in.proximity >= cfg::proximityEnter;
        if (!change) { edge = false; return false; }
        if (!edge) { edge = true; edgeSince = in.now; }
        if (in.now - edgeSince < cfg::objectDebounceMs) return false;
        present = !present; edge = false;
        if (present) arrivalTime = in.now;
        return present;
    }
};
}
