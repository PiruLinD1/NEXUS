#pragma once
#include "robot/mechanism.hpp"

namespace robot::mechanism {
struct DriverResult {
    bool owns = false, releaseAccepted = false;
};

inline DriverResult driverTick(Controller& mechanism, const Input& input,
                               bool grabPressed, bool releasePressed, bool clampClosed,
                               int liftStep = 0, bool clampTogglePressed = false,
                               bool intakeHeld = false, bool intakePressed = false) {
    if (clampTogglePressed) {
        // Manual clamp override wins this entire tick. Cancel pending motion
        // so no later macro phase can undo the requested clamp position.
        mechanism.cancel();
        mechanism.output.closed = !clampClosed;
        return {.owns = true};
    }
    if (mechanism.state == State::prepared || mechanism.state == State::off)
        mechanism.output.closed = clampClosed;
    // R1 wins a simultaneous press: every R1 edge selects its next destination.
    if (grabPressed) mechanism.grab(input);
    DriverResult result;
    // Process R2 before handing control back to the manual bindings.
    result.releaseAccepted = !grabPressed && releasePressed && mechanism.release(input);
    if (!grabPressed && !releasePressed) mechanism.adjustDriverLift(liftStep, input);
    const bool ownedBeforeTick = mechanism.owns() && mechanism.state != State::prepared;
    if (mechanism.owns()) mechanism.tick(input);
    else mechanism.output.lift = 0;
    // Production and the simulation run the same complete driver control path.
    mechanism.updateDriverLift(input);
    mechanism.updateDriverIntake(input, intakeHeld, intakePressed);
    result.owns = ownedBeforeTick || (mechanism.owns() && mechanism.state != State::prepared);
    return result;
}

inline const char* driverLabel(State state, bool calibrated) {
    switch (state) {
    case State::off: return calibrated ? "Manuale: B/GIU/L1/X/R1/R2" : "Serve preMatch";
    case State::prepared: return "Pronto: B/GIU/L1/X/R1/R2";
    case State::pickupWait: return "R1: chiusura pinza";
    case State::pickupLift: return "Presa: salita lift";
    case State::pickupArm: return "R1: braccio 150";
    case State::carrying: return "150: R1 finecorsa";
    case State::pickupEndstop: return "R1: finecorsa alto";
    case State::carryingEndstop: return "Finecorsa: R1 150";
    case State::stopped: return "Fermo: R1/R2 | B lift";
    case State::releaseArm90: return "R2: 90 veloce";
    case State::releaseClampWait: return "R2: attesa apertura";
    case State::releaseWait: return "R2: spostati 0.5 giri";
    case State::returnLiftClearance: return "Ritorno: rialzo lift";
    case State::returnArmClearance: return "Ritorno: braccio";
    case State::returnLiftLow: return "Ritorno: lift quota 3";
    case State::returnArmZero: return "Ritorno: zero";
    case State::returnLiftBottom: return "Ritorno: lift fondo";
    default: return phaseLabel(state);
    }
}
}
