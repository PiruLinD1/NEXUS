#pragma once

#include "nexus/types.hpp"

namespace nexus {

// Receding-horizon, box-constrained iLQR on a five-state voltage-driven model.
// All storage is owned and bounded: no allocation occurs in start() or update().
class Controller {
public:
    using Clock = double (*)(); // monotonic seconds; optional, e.g. pros::micros()/1e6
    explicit Controller(DynamicsConfig config = {});
    // Owner task only: replaces the physical model and clears any active motion.
    // Runtime clock and budget remain installed; no controller-sized temporary.
    void reconfigure(DynamicsConfig config);
    void start(const DriveState& state, Target target, MotionOptions options = {});
    Voltage update(const Estimate& estimate, double dt, double batteryVolts = 12.0);
    bool settled(const Estimate& estimate, double dt);
    void reset();
    void setClock(Clock clock) { clock_ = clock; }
    void setBudgetMs(double milliseconds) {
        budgetMs_ = std::isfinite(milliseconds) ? std::max(0.1, milliseconds) : 12.0;
    }
    SolverStats stats() const { return stats_; }
    const DynamicsConfig& config() const { return config_; }
    DriveState reference() const;
    double trajectoryDuration() const { return path_[pathCount_ - 1].time; }
    // Same continuous wheel model integrated in <=10ms midpoint substeps.
    DriveState predict(const DriveState& state, Voltage voltage, double dt,
                       double batteryVolts = 12.0) const;

private:
    static constexpr std::size_t nx = 5, nu = 2, horizon = 20, pathCapacity = 129;
    using State = std::array<double, nx>; // x,y,theta,left wheel speed,right wheel speed
    using Input = std::array<double, nu>;
    using Matrix = std::array<std::array<double, nx>, nx>;
    using InputJacobian = std::array<std::array<double, nu>, nx>;
    using Gain = std::array<std::array<double, nx>, nu>;
    struct Knot { Pose pose{}; double v = 0, omega = 0, time = 0; };
    struct Reference { State state{}; Input input{}; bool terminal = false; };
    struct Derivatives { State x{}; Input u{}; Matrix xx{}; std::array<double, nu> uu{}; };

    State encode(const DriveState&) const;
    DriveState decode(const State&) const;
    State integrate(State, const Input&, double dt, Matrix* a = nullptr,
                    InputJacobian* b = nullptr) const;
    void makePath(const DriveState&);
    Reference sample(double time, double scale, double voltage) const;
    double cost(const State&, const Input&, const Reference&, bool terminal,
                double limitScale, Derivatives* derivatives = nullptr) const;
    double rollout(const State&, const std::array<Input, horizon>&,
                   std::array<State, horizon + 1>&, double scale) const;
    bool backward(double regularization, double voltage);
    bool exhausted();

    DynamicsConfig config_{};
    MotionOptions options_{};
    Target target_{};
    Clock clock_ = nullptr;
    double budgetMs_ = 12, computeStart_ = 0, pathTime_ = 0, settledTime_ = 0;
    double horizonDt_ = 0.075, lastScale_ = 1, lastReplan_ = 0, runTime_ = 0;
    bool active_ = false, warm_ = false;
    Voltage previous_{};
    SolverStats stats_{};
    std::size_t pathCount_ = 1;
    std::array<Knot, pathCapacity> path_{};
    std::array<Reference, horizon + 1> refs_{};
    std::array<Input, horizon> controls_{}, trialControls_{}, feedforward_{};
    std::array<State, horizon + 1> states_{}, trialStates_{};
    std::array<Matrix, horizon> a_{};
    std::array<InputJacobian, horizon> b_{};
    std::array<Gain, horizon> feedback_{};
    std::array<Derivatives, horizon + 1> derivatives_{};
};
} // namespace nexus
