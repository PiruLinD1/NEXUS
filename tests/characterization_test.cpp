#include "nexus/characterization.hpp"
#include <cstdlib>
#include <iostream>
#include <limits>

using namespace nexus;
using namespace nexus::calibration;
void require(bool condition, const char* message) {
    if (!condition) { std::cerr << message << '\n'; std::exit(1); }
}
bool zero(Voltage voltage) { return voltage.left == 0 && voltage.right == 0; }
struct Rig {
    EstimatorConfig config;
    SensorSample sample;
    CharacterizationRoutine routine;
    CharacterizationOutput out;
    double now = 10;
    Rig() {
        config.forwardOffset = .087; config.lateralOffset = -.063;
        config.forwardScale = 1.12; config.lateralScale = .91; config.gyroScale = 1.03;
        sample.timestamp = now;
        sample.forward = 100; sample.lateral = -20; sample.gyro = 1000;
        sample.forwardValid = sample.lateralValid = sample.gyroValid = sample.gyroRateValid = true;
        // Deliberately unusable drive encoder values: only pods and IMU count.
        sample.left = sample.right = std::numeric_limits<double>::quiet_NaN();
    }
    void start(unsigned trial = 0) {
        require(routine.start(sample, now, trial, config), "a stationary valid trial starts");
        out = routine.output();
    }
    // Independent body displacement -> sensor geometry. No drivetrain dynamics
    // or implementation outputs are used to generate the scripted trajectory.
    void step(double forward = 0, double lateral = 0, double yaw = 0, double dt = .01, bool permit = true) {
        now += dt;
        sample.timestamp = now;
        sample.forward += (forward - config.forwardOffset * yaw) / config.forwardScale;
        sample.lateral += (lateral + config.lateralOffset * yaw) / config.lateralScale;
        sample.gyro += yaw / config.gyroScale;
        sample.gyroRate = yaw / dt / config.gyroScale;
        out = routine.update(sample, now, permit);
    }
    void powered(unsigned trial = 0) {
        start(trial);
        for (int i = 0; i < 50; ++i) {
            require(zero(out.voltage), "settling never applies voltage");
            step();
        }
        require(out.stage == CharacterizationStage::powered, "half a second of stillness starts one pulse");
    }
    void fault(CharacterizationFailure expected) {
        require(out.stage == CharacterizationStage::aborted && out.failure == expected && zero(out.voltage),
                "fault identifies its cause and immediately zeros both voltages");
        step();
        require(out.stage == CharacterizationStage::aborted && out.failure == expected && zero(out.voltage),
                "sensor recovery cannot restart an aborted run");
    }
};
int main() {
    static_assert(static_cast<unsigned>(CharacterizationStage::idle) == 0);
    static_assert(static_cast<unsigned>(CharacterizationStage::settling) == 1);
    static_assert(static_cast<unsigned>(CharacterizationStage::powered) == 2);
    static_assert(static_cast<unsigned>(CharacterizationStage::coast) == 3);
    static_assert(static_cast<unsigned>(CharacterizationStage::complete) == 4);
    static_assert(static_cast<unsigned>(CharacterizationStage::aborted) == 5);
    require(characterizationTrialCount == 18 && !characterizationTrial(18), "three tiers contain six mirrored trials each");
    for (unsigned trial = 0; trial < characterizationTrialCount; ++trial) {
        Rig rig; rig.powered(trial);
        const auto* profile = characterizationTrial(trial);
        const unsigned kind = trial % 6;
        const bool spinning = kind == 2 || kind == 3;
        const double forwardRate = spinning ? 0 : kind == 1 ? -.65 : .65;
        const double yawRate = kind == 2 || kind == 5 ? 1.5 : kind == 3 || kind == 4 ? -1.5 : 0;
        const double outer = std::max(profile->voltage.left, profile->voltage.right);
        const Voltage initial = kind >= 4 ? Voltage{outer, outer} : profile->voltage;
        require(initial.left == rig.out.voltage.left && initial.right == rig.out.voltage.right
                && std::abs(rig.out.voltage.left) <= 6 && std::abs(rig.out.voltage.right) <= 6,
                "the selected tier applies at most six volts without changing the trial");
        require((kind != 0 || (rig.out.voltage.left > 0 && rig.out.voltage.right > 0))
                && (kind != 1 || (rig.out.voltage.left < 0 && rig.out.voltage.right < 0))
                && (kind != 2 || (rig.out.voltage.left > 0 && rig.out.voltage.right < 0))
                && (kind != 3 || (rig.out.voltage.left < 0 && rig.out.voltage.right > 0))
                && (kind < 4 || (rig.out.voltage.left == rig.out.voltage.right && rig.out.voltage.left > 0)),
                "mirrored trials start in the intended direction and curves enter while driving forward");
        const double poweredStarted = rig.now;
        bool steeringStep = false;
        while (rig.out.stage == CharacterizationStage::powered) {
            const bool straightLead = kind >= 4 && rig.now - poweredStarted < .20 - 1e-9;
            rig.step(forwardRate * .01, 0, straightLead ? 0 : yawRate * .01);
            if (kind >= 4 && rig.out.stage == CharacterizationStage::powered) {
                const bool leading = rig.now - poweredStarted < .20 - 1e-9;
                require(leading ? rig.out.voltage.left == outer && rig.out.voltage.right == outer
                        : rig.out.voltage.left == profile->voltage.left && rig.out.voltage.right == profile->voltage.right,
                        "curves make one sharp voltage step after exactly 200 ms of straight acceleration");
                steeringStep = steeringStep || !leading;
            }
        }
        require(kind < 4 || steeringStep, "each curve includes a moving steering step within the original pulse duration");
        require(rig.out.stage == CharacterizationStage::coast && zero(rig.out.voltage)
                && std::abs(rig.now - poweredStarted - profile->poweredSeconds) < .011,
                "each bounded pulse enters an actual zero-voltage coast phase");
        const double poweredDistance = rig.out.distance;
        for (int i = 12; i > 0; --i) {
            const double fraction = i / 12.0;
            rig.step(forwardRate * .01 * fraction, 0, yawRate * .01 * fraction);
            require(rig.out.stage == CharacterizationStage::coast && zero(rig.out.voltage),
                    "continuing coast motion is measured with both outputs zero");
        }
        for (int i = 0; i < 40 && rig.out.stage == CharacterizationStage::coast; ++i) rig.step();
        require(rig.out.stage == CharacterizationStage::complete && zero(rig.out.voltage)
                && rig.out.failure == CharacterizationFailure::none && rig.out.trialId == trial,
                "settled coast completes exactly one selected run");
        require(spinning ? rig.out.distance < 1e-9 : rig.out.distance > poweredDistance,
                "calibrated pod offsets remove pure spin translation and retain real coast displacement");
        require(std::abs(rig.out.angle - yawRate * (profile->poweredSeconds - profile->straightLeadSeconds + .065)) < 1e-9,
                "scaled unwrapped IMU records the full powered and coast angle");
        for (int i = 0; i < 100; ++i) rig.step();
        require(rig.out.stage == CharacterizationStage::complete && zero(rig.out.voltage),
                "completion cannot automatically start another tier or pulse");
        if (!trial) {
            rig.out = rig.routine.abort(CharacterizationFailure::loggingUnavailable);
            rig.fault(CharacterizationFailure::loggingUnavailable);
        }
    }
    {
        Rig rig; rig.powered(2);
        // Device counters can repeat across a UI poll, then deliver the next
        // cumulative increment. This script refreshes both pods and IMU every
        // 20 ms while the runner continues receiving 10 ms timestamps.
        for (int tick = 0; tick < 40; ++tick) rig.step(0, 0, tick % 2 ? .04 : 0);
        require(rig.out.stage == CharacterizationStage::coast && std::abs(rig.out.angle - .8) < 1e-9
                && rig.out.distance < 1e-9 && zero(rig.out.voltage),
                "quantized cumulative sensor refresh retains calibrated spin geometry at a 10 ms UI cadence");
        for (int tick = 0; tick < 40; ++tick) rig.step();
        require(rig.out.stage == CharacterizationStage::complete,
                "normal repeated sensor counters settle without relaxing any motion limits");
    }
    {
        Rig rig; rig.start();
        require(!rig.routine.start(rig.sample, rig.now, 12, rig.config) && rig.routine.output().trialId == 0,
                "an active trial cannot be restarted to extend its powered deadline");
        rig.step(.001); rig.fault(CharacterizationFailure::initialMotion);
    }
    {
        Rig rig; rig.sample.gyroRate = radians(10);
        require(!rig.routine.start(rig.sample, rig.now, 0, rig.config), "initial IMU motion prevents starting");
        rig.out = rig.routine.output(); rig.fault(CharacterizationFailure::initialMotion);
    }
    for (unsigned trial : {0u, 2u, 4u}) {
        Rig rig; rig.powered(trial);
        for (int i = 0; i < 60 && rig.out.stage == CharacterizationStage::powered; ++i) rig.step();
        rig.fault(CharacterizationFailure::stalled);
    }
    for (CharacterizationStage stage : {CharacterizationStage::settling, CharacterizationStage::powered, CharacterizationStage::coast}) {
        Rig rig;
        if (stage == CharacterizationStage::settling) rig.start();
        else {
            rig.powered();
            if (stage == CharacterizationStage::coast)
                while (rig.out.stage == CharacterizationStage::powered) rig.step(.005);
        }
        rig.step(0, 0, 0, .01, false); rig.fault(CharacterizationFailure::notPermitted);
    }
    for (int sensor = 0; sensor < 6; ++sensor) {
        Rig rig; rig.powered();
        if (sensor == 0) rig.sample.forwardValid = false;
        if (sensor == 1) rig.sample.lateralValid = false;
        if (sensor == 2) rig.sample.gyroValid = false;
        if (sensor == 3) rig.sample.forward = std::numeric_limits<double>::quiet_NaN();
        if (sensor == 4) rig.sample.lateral = std::numeric_limits<double>::infinity();
        if (sensor == 5) rig.sample.gyro = std::numeric_limits<double>::quiet_NaN();
        rig.step(); rig.fault(CharacterizationFailure::invalidSensors);
    }
    for (int badConfig = 0; badConfig < 4; ++badConfig) {
        Rig rig;
        if (badConfig == 0) rig.config.forwardScale = 0;
        if (badConfig == 1) rig.config.lateralScale = -1;
        if (badConfig == 2) rig.config.gyroScale = std::numeric_limits<double>::infinity();
        if (badConfig == 3) rig.config.forwardOffset = std::numeric_limits<double>::quiet_NaN();
        require(!rig.routine.start(rig.sample, rig.now, 0, rig.config), "invalid geometry prevents a run");
        rig.out = rig.routine.output(); rig.fault(CharacterizationFailure::invalidConfiguration);
    }
    {
        Rig rig; require(!rig.routine.start(rig.sample, rig.now, 18, rig.config), "invalid trial is rejected");
        rig.out = rig.routine.output(); rig.fault(CharacterizationFailure::invalidTrial);
    }
    for (int clockFault = 0; clockFault < 3; ++clockFault) {
        Rig rig; rig.powered();
        if (clockFault == 0) rig.now -= .01;
        if (clockFault == 1) rig.sample.timestamp -= .01;
        if (clockFault == 2) rig.sample.timestamp += .1;
        rig.out = rig.routine.update(rig.sample, rig.now, true);
        rig.fault(CharacterizationFailure::invalidClock);
    }
    {
        Rig rig; rig.powered(); rig.step(0, 0, 0, .11); rig.fault(CharacterizationFailure::schedulerGap);
    }
    {
        Rig rig; rig.powered();
        for (int i = 0; i < 6; ++i) {
            rig.now += .02;
            rig.out = rig.routine.update(rig.sample, rig.now, true);
        }
        rig.fault(CharacterizationFailure::staleSensors);
    }
    {
        Rig rig; rig.powered(); rig.step(.10); rig.fault(CharacterizationFailure::sensorJump);
    }
    {
        Rig rig; rig.powered(2); rig.step(0, 0, radians(8)); rig.fault(CharacterizationFailure::rotationTooFast);
    }
    {
        Rig rig; rig.powered(12);
        while (rig.out.stage == CharacterizationStage::powered) rig.step(.01);
        while (rig.out.stage == CharacterizationStage::coast) rig.step(.01);
        rig.fault(CharacterizationFailure::translationLimit);
    }
    {
        Rig rig; rig.powered(14);
        while (rig.out.stage == CharacterizationStage::powered) rig.step(0, 0, .10);
        while (rig.out.stage == CharacterizationStage::coast) rig.step(0, 0, .10);
        rig.fault(CharacterizationFailure::rotationLimit);
    }
    {
        Rig rig; rig.powered();
        while (rig.out.stage == CharacterizationStage::powered) rig.step(.001);
        while (rig.out.stage == CharacterizationStage::coast) rig.step(.001);
        rig.fault(CharacterizationFailure::coastTimeout);
    }
    for (unsigned trial : {0u, 2u, 4u}) {
        Rig rig; rig.powered(trial);
        if (trial == 4) for (int i = 0; i < 20; ++i) rig.step(.002);
        for (int i = 0; i < 30 && rig.out.stage == CharacterizationStage::powered; ++i)
            rig.step(trial == 0 ? -.005 : .002, 0, trial == 2 ? -.01 : trial == 4 ? .01 : 0);
        rig.fault(CharacterizationFailure::wrongDirection);
    }
    {
        Rig rig; rig.powered(); rig.out = rig.routine.abort(CharacterizationFailure::loggingUnavailable);
        rig.fault(CharacterizationFailure::loggingUnavailable);
        require(rig.routine.start(rig.sample, rig.now, 1, rig.config), "an explicit new start permits a separate run after a fault");
        require(rig.routine.output().stage == CharacterizationStage::settling && zero(rig.routine.output().voltage),
                "the new run repeats baseline settling before any voltage");
    }
    std::cout << "Characterization: 18 single-pulse trials, corrected pod geometry, coast, and abort guards passed\n";
}
