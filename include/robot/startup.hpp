#pragma once
#include <atomic>
#include <cstdint>

namespace robot::startup {
enum class InitializationState { idle, running, ready, failed };

// Publishes all startup-owned hardware/configuration only after the complete
// sequence finishes. A partially constructed chassis is not a ready robot.
class Initialization {
public:
    bool begin() {
        auto expected = InitializationState::idle;
        return state_.compare_exchange_strong(expected, InitializationState::running,
                                             std::memory_order_acq_rel);
    }
    void finish(bool success) {
        auto expected = InitializationState::running;
        state_.compare_exchange_strong(expected,
            success ? InitializationState::ready : InitializationState::failed,
            std::memory_order_release, std::memory_order_relaxed);
    }
    InitializationState state() const { return state_.load(std::memory_order_acquire); }
private:
    std::atomic<InitializationState> state_{InitializationState::idle};
};

enum class CallbackAction { wait, proceed, abort };
class CallbackGate {
public:
    CallbackGate(bool autonomous, bool connected) : autonomous_(autonomous), connected_(connected) {}
    CallbackAction poll(InitializationState state, bool disabled, bool autonomous, bool connected) const {
        // A callback waiting for startup must not later enter a different mode.
        if (disabled || autonomous != autonomous_ || connected != connected_)
            return CallbackAction::abort;
        switch (state) {
        case InitializationState::running: return CallbackAction::wait;
        case InitializationState::ready: return CallbackAction::proceed;
        case InitializationState::idle:
        case InitializationState::failed: return CallbackAction::abort;
        }
        return CallbackAction::abort;
    }
private:
    bool autonomous_, connected_;
};

enum class Phase { delayPreMatch, waitMotorEnable, preMatch, waitReady, delayNavigation, complete, fault };
struct Action { bool startPreMatch = false, stopPreMatch = false; };
class Gate {
public:
    explicit Gate(std::uint32_t now) : since(now) {}
    Phase phase = Phase::delayPreMatch;
    // Boot starts immediately; VEXos still inhibits motors while disabled.
    // Never run homing timers against inhibited motors. A completed boot never repeats.
    Action tick(std::uint32_t now, bool prepared, bool failed,
                bool readyPressed, bool retryPressed, bool motorsEnabled = true) {
        Action action;
        switch (phase) {
        case Phase::delayPreMatch:
            if (now - since >= 1000) {
                phase = motorsEnabled ? Phase::preMatch : Phase::waitMotorEnable;
                action.startPreMatch = motorsEnabled;
            }
            break;
        case Phase::waitMotorEnable:
            if (motorsEnabled) { phase = Phase::preMatch; action.startPreMatch = true; }
            break;
        case Phase::preMatch:
            if (failed) { phase = Phase::fault; action.stopPreMatch = true; }
            else if (prepared) phase = Phase::waitReady;
            else if (!motorsEnabled) { phase = Phase::waitMotorEnable; action.stopPreMatch = true; }
            break;
        case Phase::waitReady:
            if (readyPressed) { phase = Phase::delayNavigation; since = now; }
            break;
        case Phase::delayNavigation:
            if (now - since >= 1000) phase = Phase::complete;
            break;
        case Phase::fault:
            if (retryPressed) { phase = Phase::delayPreMatch; since = now; }
            break;
        case Phase::complete: break;
        }
        return action;
    }
private:
    std::uint32_t since;
};
}
