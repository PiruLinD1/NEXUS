#include "robot/arm_motion.hpp"
#include <cstdlib>
#include <iostream>
#include <limits>

using robot::mechanism::ArmMotion;
namespace cfg = robot::config::mechanism;
void require(bool ok, const char* message) {
    if (!ok) { std::cerr << message << '\n'; std::exit(1); }
}
struct Plant {
    double position, velocity = 0;
    double responseSeconds = 0.05, brakeSeconds = 0.025;
    double load = 0, friction = 0, gain = 2;
    // Optional underdamped internal position hold, capturing the position when
    // HOLD is enabled. This exposes the rebound caused by engaging it at speed.
    bool springHold = false, wasHolding = false;
    double holdPosition = 0;
    void step(const ArmMotion::Command& command, unsigned milliseconds) {
        if (command.hold && !wasHolding) holdPosition = position;
        wasHolding = command.hold;
        for (unsigned i = 0; i < milliseconds; ++i) {
            const double effort = command.power - load;
            const double moving = std::copysign(std::max(0.0, std::abs(effort) - friction), effort) * gain;
            const bool stopping = command.hold || command.brake;
            const double targetSpeed = stopping ? 0 : moving;
            const double tau = stopping ? brakeSeconds : responseSeconds;
            if (springHold && command.hold)
                velocity += (500 * (holdPosition - position) - 14 * velocity) * 0.001;
            else velocity += (targetSpeed - velocity) * (1 - std::exp(-0.001 / tau));
            position += velocity * 0.001;
            if (position < 0 || position > 195) { position = std::clamp(position, 0.0, 195.0); velocity = 0; }
        }
    }
};
unsigned move(Plant& plant, double target, bool jitter = false) {
    ArmMotion motion;
    unsigned elapsed = 0;
    ArmMotion::Command command;
    for (unsigned i = 0; elapsed < 3000; ++i) {
        const double noise = 0.04 * std::sin(i * 0.71);
        command = motion.update(elapsed, plant.position + noise, target);
        require(std::abs(command.power) <= 127, "profile respects motor command range");
        if (command.arrived || command.exhausted) break;
        const unsigned step = jitter ? (i % 3 == 0 ? 20 : i % 3 == 1 ? 5 : 10) : 10;
        plant.step(command, step); elapsed += step;
    }
    if (!command.arrived)
        std::cerr << "Failed move to " << target << ": position " << plant.position << ", velocity " << plant.velocity << '\n';
    require(command.arrived && command.hold, "profile must settle with inertia, load, friction and sample jitter");
    require(std::abs(plant.position - target) <= 2.2, "profile arrival must be within approximately two degrees");
    plant.step(command, 300);
    if (std::abs(plant.position - target) > 2.5)
        std::cerr << "Residual drift at " << target << ": " << plant.position
                  << ", response " << plant.responseSeconds << ", brake " << plant.brakeSeconds
                  << ", load " << plant.load << ", gain " << plant.gain << '\n';
    require(std::abs(plant.position - target) <= 2.5, "residual momentum after arrival must remain within tolerance margin");
    return elapsed;
}
int main() {
    unsigned worst = 0;
    for (double response : {0.025, 0.07, 0.12}) for (double brake : {0.01, 0.04, 0.08, 0.12})
        for (double load : {-10.0, 0.0, 10.0}) for (double gain : {1.5, 2.5}) {
            Plant plant{0}; plant.responseSeconds = response; plant.brakeSeconds = brake;
            plant.load = load; plant.friction = 6; plant.gain = gain;
            worst = std::max(worst, move(plant, cfg::armPickupDeg, true));
            worst = std::max(worst, move(plant, 144, true));
            plant.position = 190;
            worst = std::max(worst, move(plant, 144, true));
            worst = std::max(worst, move(plant, cfg::armPickupDeg, true));
        }
    Plant nominal{0};
    const auto pickupTime = move(nominal, cfg::armPickupDeg);
    require(pickupTime < 1300, "nominal pickup profile must be fast, including its braking phase");

    // Loads above the old 20/30 correction commands used to stall or push
    // the arm backward near its destination. Test both torque directions.
    unsigned loadedWorst = 0;
    for (double load : {-55.0, -35.0, 35.0, 55.0}) {
        Plant loaded{load > 0 ? 0.0 : 190.0};
        loaded.load = load; loaded.friction = 10;
        loadedWorst = std::max(loadedWorst, move(loaded, cfg::armPickupDeg, true));
        loadedWorst = std::max(loadedWorst, move(loaded, cfg::armPickupDeg + (load > 0 ? 14 : -20), true));
    }

    // Remove a heavy payload after assistance has built up, during the same
    // request. Braking must still stop the now much lighter arm near target.
    ArmMotion unloading;
    Plant unloadingPlant{cfg::armPickupDeg - 20}; unloadingPlant.load = 55; unloadingPlant.friction = 10;
    ArmMotion::Command unloadingCommand;
    bool removedLoad = false;
    for (unsigned now = 0; now < 3000; now += 10) {
        unloadingCommand = unloading.update(now, unloadingPlant.position, cfg::armPickupDeg);
        if (!removedLoad && now > 100 && unloadingCommand.power >= 80) {
            unloadingPlant.load = 0; unloadingPlant.friction = 0; removedLoad = true;
        }
        unloadingPlant.step(unloadingCommand, 10);
        if (unloadingCommand.arrived || unloadingCommand.exhausted) break;
    }
    require(removedLoad && unloadingCommand.arrived && unloadingCommand.hold
            && std::abs(unloadingPlant.position - cfg::armPickupDeg) <= 2.2,
            "removing a payload during assistance must preserve braking and arrival precision");

    // Additional effort is always bounded, even if an obstruction never moves.
    ArmMotion immovable;
    ArmMotion::Command saturated;
    for (unsigned now = 0; now <= 1000; now += 10) {
        saturated = immovable.update(now, 120, 130);
        require(saturated.power >= 0 && saturated.power <= 127 && !saturated.arrived,
                "a stalled approach cannot exceed full effort or falsely report arrival");
    }
    require(saturated.power == 127, "load assistance can use full effort when the slow approach cannot move");

    // A new request discards effort learned for an old payload.
    const auto restarted = immovable.update(1010, 120, 124);
    require(restarted.power < 30 && !restarted.arrived, "retargeting resets the load assist and acceleration ramp");

    // A slow mechanical response plus an underdamped motor HOLD reproduces
    // the overshoot/rebound scenario; approach it from both sides.
    Plant rebound{0}; rebound.responseSeconds = 0.10; rebound.brakeSeconds = 0.08;
    rebound.springHold = true;
    move(rebound, cfg::armPickupDeg, true);
    move(rebound, 144, true);
    rebound.position = 190;
    move(rebound, 144, true);

    ArmMotion arrivingFast;
    arrivingFast.update(0, 100, 130);
    const auto fastBrake = arrivingFast.update(10, 127, 130);
    require(fastBrake.brake && !fastBrake.hold && !fastBrake.arrived && fastBrake.power == 0,
            "a fast approach must use passive braking before engaging position HOLD");
    // Passive braking cannot leave a gravity-loaded arm unsupported forever.
    const auto supported = arrivingFast.update(220, 150, 130);
    require(supported.hold && !supported.brake && !supported.arrived,
            "braking has a bounded duration even if the arm does not slow down");

    // Feed repeated measured overshoots, with time to stop between each one.
    // The old controller kept issuing alternating correction commands.
    ArmMotion hunting;
    double displaced = 100;
    int previousSign = 1;
    unsigned reversals = 0;
    ArmMotion::Command huntingCommand;
    for (unsigned now = 0; now < 4000; now += 10) {
        huntingCommand = hunting.update(now, displaced, 130);
        if (huntingCommand.power) {
            const int sign = huntingCommand.power > 0 ? 1 : -1;
            if (sign != previousSign) ++reversals;
            previousSign = sign;
            displaced = sign > 0 ? 136 : 124;
        }
    }
    require(reversals <= 2 && huntingCommand.exhausted && huntingCommand.hold
            && !huntingCommand.arrived && !huntingCommand.power,
            "persistent overshoot must stop corrections, retain HOLD and never report false arrival");
    hunting.reset();
    require(!hunting.update(4010, displaced, 130).exhausted, "a fresh request can retry after bounded corrections");

    // Retarget while momentum still points toward the old destination.
    ArmMotion switching;
    Plant moving{40};
    for (unsigned now = 0; now < 180; now += 10) moving.step(switching.update(now, moving.position, 144), 10);
    const double changedAt = moving.position;
    ArmMotion::Command command;
    for (unsigned now = 180; now < 3000; now += 10) {
        command = switching.update(now, moving.position, 50);
        moving.step(command, 10);
        if (command.arrived) break;
    }
    require(changedAt > 50 && command.arrived && std::abs(moving.position - 50) <= 2.2,
            "a new target can reverse a moving arm and still stop accurately");

    const auto invalid = switching.update(3100, std::numeric_limits<double>::quiet_NaN(), 130);
    require(invalid.hold && !invalid.arrived && invalid.power == 0, "invalid position cannot command motion or claim arrival");
    std::cout << "Arm profile: " << cfg::armPickupDeg << " degrees nominal " << pickupTime << " ms; worst matrix move " << worst
              << " ms; heavy-load move " << loadedWorst << " ms\n";
}
