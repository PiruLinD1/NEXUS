#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>

// Platform-independent guard; only the 10ms motor task may mutate it.
// Stops latch for each command. Heartbeats never restart a completed pulse.
struct PulseGuard {
    static bool controllerConnected(std::int32_t status) { return status>0 && status!=INT32_MAX; }
    static constexpr unsigned maxDuration = 2000, maxHeartbeatAge = 120;
    static constexpr int maxVoltage = 3000;
    static constexpr double maxPulseMm = 300, maxSessionMm = 1250;
    bool headingReady = false;
    double heading = 0, initialHeading = 0, pulseHeading = 0;
    void observeHeading(double value) {
        if (!std::isfinite(value)) { fault=true; stop(6); return; }
        if (!headingReady) { initialHeading=value; headingReady=true; }
        heading=value;
        if (std::abs(heading-initialHeading)>400) { fault=true; stop(9); }
    }
    void envelope(double x, double y, double degrees) {
        // Additional conservative position/heading checks; NOT sole travel limits.
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(degrees) ||
            std::abs(x)>150 || std::abs(y)>150) {
            fault=true; stop(9);
        }
    }
    bool active = false, fault = false, haveBaseline = false, heartbeatSeen = false;
    unsigned heartbeat = 0, start = 0, duration = 0, sequence = 0;
    int left = 0, right = 0, reason = 0;
    std::array<double, 6> baseline{}, previous{}, travel{};
    void stop(int why) { active = false; left = right = 0; reason = why; }
    bool fresh(unsigned now, unsigned token) const { return now - token <= 100; }
    void beat(unsigned now, unsigned token) {
        if (fresh(now, token)) { heartbeat = now; heartbeatSeen = true; }
    }
    bool startPulse(unsigned now, unsigned token, unsigned seq, int l, int r,
                    unsigned ms, bool consent, const std::array<double, 6>& enc) {
        if (seq <= sequence) return false;
        sequence = seq; // Rejected commands are consumed, never replayed later.
        if (active || fault || !haveBaseline || !headingReady || !consent || !fresh(now, token) ||
            !heartbeatSeen || now - heartbeat > maxHeartbeatAge ||
            ms == 0 || ms > maxDuration || l < -maxVoltage || l > maxVoltage ||
            r < -maxVoltage || r > maxVoltage || l == 0 || l != -r) return false;
        for (double t : travel) if (t >= maxSessionMm) return false;
        baseline = enc; start = now; duration = ms; left = l; right = r; pulseHeading=heading;
        active = true; reason = 0; return true;
    }
    void tick(unsigned now, bool consent, bool valid, const std::array<double, 6>& enc) {
        for (double e : enc) if (!std::isfinite(e)) valid = false;
        if (!valid) { fault = true; stop(6); return; }
        if (!haveBaseline) { previous = enc; haveBaseline = true; }
        for (unsigned i = 0; i < 6; ++i) {
            travel[i] += std::abs(enc[i] - previous[i]);
            previous[i] = enc[i];
            if (travel[i] >= maxSessionMm) { fault = true; stop(5); }
        }
        if (!active) return;
        if (!consent) { stop(1); return; }
        if (!heartbeatSeen || now - heartbeat > maxHeartbeatAge) { stop(2); return; }
        if (now - start >= duration) { stop(3); return; }
        const double angle=(heading-pulseHeading)*(left>0?1:-1);
        if (angle < -5) { fault=true; stop(11); return; }
        if (angle >= 90) { stop(10); return; }
        for (unsigned i = 0; i < 6; ++i)
            if (std::abs(enc[i] - baseline[i]) >= maxPulseMm) { stop(4); return; }
    }
};
