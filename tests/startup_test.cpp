#include "robot/startup.hpp"
#include <cstdlib>
#include <iostream>
using namespace robot::startup;
void require(bool condition, const char* message) {
    if (!condition) { std::cerr << message << '\n'; std::exit(1); }
}
int main() {
    Initialization initialization;
    CallbackGate driver(false, false), autonomous(true, true);
    require(initialization.state() == InitializationState::idle, "startup is not ready before task creation");
    initialization.finish(true);
    require(initialization.state() == InitializationState::idle, "only the startup owner may publish completion");
    require(driver.poll(initialization.state(), false, false, false) == CallbackAction::abort,
            "callback does not wait forever when startup was never started");
    require(initialization.begin(), "first caller starts the persistent worker");
    require(!initialization.begin(), "duplicate call cannot start a second hardware owner");
    require(driver.poll(initialization.state(), false, false, false) == CallbackAction::wait,
            "driver waits before touching startup-owned mechanisms");
    require(autonomous.poll(initialization.state(), false, true, true) == CallbackAction::wait,
            "autonomous waits before constructing a motion sequence");
    require(driver.poll(initialization.state(), true, false, false) == CallbackAction::abort,
            "disable cancels a waiting callback");
    require(driver.poll(initialization.state(), false, true, false) == CallbackAction::abort,
            "autonomous mode cancels a waiting driver callback");
    require(autonomous.poll(initialization.state(), false, false, true) == CallbackAction::abort,
            "driver mode cancels a waiting autonomous callback");
    require(driver.poll(initialization.state(), false, false, true) == CallbackAction::abort,
            "connecting field control cancels a stale practice callback");
    require(autonomous.poll(initialization.state(), false, true, false) == CallbackAction::abort,
            "disconnecting field control cancels a stale autonomous callback");

    Gate gate(100);
    require(!gate.tick(1099, false, false, true, false).startPreMatch, "wait full initial second; ignore early READY");
    require(gate.tick(1100, false, false, false, false).startPreMatch, "automatic preMatch after one second");
    gate.tick(2000, false, false, true, false);
    require(gate.phase == Phase::preMatch, "READY cannot skip preMatch");
    gate.tick(5000, true, false, false, false);
    gate.tick(20000, true, false, false, false);
    require(gate.phase == Phase::waitReady, "navigation waits indefinitely for READY");
    gate.tick(20010, true, false, true, false);
    gate.tick(21009, true, false, false, false);
    require(gate.phase == Phase::delayNavigation, "wait full second after READY");
    gate.tick(21010, true, false, false, false);
    require(gate.phase == Phase::complete, "navigation allowed only after READY plus one second");
    require(driver.poll(initialization.state(), false, false, false) == CallbackAction::wait,
            "READY completes only mechanism setup: callbacks still wait for IMU, chassis, bias and services");
    initialization.finish(true);
    require(initialization.state() == InitializationState::ready, "full startup completion publishes readiness");
    require(driver.poll(initialization.state(), false, false, false) == CallbackAction::proceed,
            "driver enters only after all hardware is published");
    require(autonomous.poll(initialization.state(), false, true, true) == CallbackAction::proceed,
            "autonomous enters only after all hardware is published");
    require(driver.poll(initialization.state(), true, false, false) == CallbackAction::abort,
            "readiness cannot override field disable");
    require(driver.poll(initialization.state(), false, true, false) == CallbackAction::abort,
            "readiness cannot resurrect an old mode callback");
    require(!initialization.begin(), "ready hardware cannot be initialized twice");
    initialization.finish(false);
    require(initialization.state() == InitializationState::ready, "late duplicate completion cannot revoke readiness");
    Initialization failed;
    require(failed.begin(), "failing startup initially owns hardware");
    failed.finish(false);
    require(driver.poll(failed.state(), false, false, false) == CallbackAction::abort,
            "bad configuration or task creation failure exits callback wait");
    require(!failed.begin(), "failed startup does not silently repeat motor setup");
    failed.finish(true);
    require(failed.state() == InitializationState::failed, "failure cannot accidentally publish partial hardware");
    // Normal callbacks remain blocked throughout boot; homing has its own owner.
    Gate boot(0);
    require(autonomous.poll(InitializationState::running, true, true, true) == CallbackAction::abort,
            "field-disabled autonomous remains blocked during startup");
    require(boot.tick(1000, false, false, false, false).startPreMatch,
            "boot preMatch starts when motor power is available");
    require(!boot.tick(1500, false, false, false, false).startPreMatch,
            "ongoing boot cannot start a second homing run");
    require(boot.tick(1600, false, true, false, false).stopPreMatch, "fault still stops startup outputs");
    boot.tick(2000, false, false, true, false);
    require(boot.phase == Phase::fault, "READY does not bypass fault");
    boot.tick(2100, false, false, false, true);
    require(!boot.tick(3099, false, false, false, false).startPreMatch, "retry waits the complete second");
    require(boot.tick(3100, false, false, false, false).startPreMatch, "explicit fault retry repeats full preMatch");
    Gate inhibited(0);
    require(!inhibited.tick(1000, false, false, false, false, false).startPreMatch &&
            inhibited.phase == Phase::waitMotorEnable, "boot starts in disabled without timing an inhibited homing move");
    require(!inhibited.tick(60000, false, false, true, true, false).startPreMatch &&
            inhibited.phase == Phase::waitMotorEnable, "long disable and READY cannot fake homing completion");
    require(inhibited.tick(60010, false, false, false, false, true).startPreMatch,
            "pending boot homing starts automatically when motor power is available");
    require(inhibited.tick(60100, false, false, false, false, false).stopPreMatch &&
            inhibited.phase == Phase::waitMotorEnable, "disable during homing stops output without a false endstop");
    require(inhibited.tick(90000, false, false, false, false, true).startPreMatch,
            "interrupted boot homing restarts with fresh mechanical references");
    inhibited.tick(100000, true, false, false, false, true);
    inhibited.tick(100010, true, false, true, false, false);
    inhibited.tick(101010, true, false, false, false, false);
    require(inhibited.phase == Phase::complete &&
            !inhibited.tick(110000, false, false, false, false, true).startPreMatch,
            "completed prematch never repeats on a later field enable");
    for (unsigned now : {22000u, 30000u, 90000u}) {
        const auto completed = gate.tick(now, false, true, true, true);
        require(gate.phase == Phase::complete && !completed.startPreMatch && !completed.stopPreMatch,
                "completed startup never rehomes after later events or mode changes");
    }
    Gate wrap(0xffffff00u);
    require(wrap.tick(744, false, false, false, false).startPreMatch, "timer handles millis wrap");
    std::cout << "Startup sequencing passed\n";
}
