#pragma once

#include "nexus/types.hpp"

namespace nexus {

// Sensor-only estimator. No motors, tasks, dynamic allocation, or platform API.
// A caller serializes reset/update/estimate; the robot adapter provides its lock.
// Range timestamps are acquisition times in the same clock as SensorSample.
class Estimator {
public:
    explicit Estimator(EstimatorConfig config = {});
    // Replaces the pose prior and sensor baselines, preserving learned bias.
    void reset(Pose pose, double timestamp = 0);
    // Apply verified physical calibration while stopped. Preserve the complete
    // pose prior; the next update establishes fresh sensor baselines.
    void reconfigure(EstimatorConfig config);
    // A separately verified stationary calibration observes only gyro bias.
    // Units: rad/s and (rad/s)^2. Never resets pose uncertainty/confidence.
    bool setGyroBias(double bias, double variance);
    Estimate update(const SensorSample& sample);
    Estimate estimate() const;
    // Stationary field recovery with trusted heading and >=3 independent wall
    // beams (opposing walls plus an orthogonal wall). Five fresh, consistent
    // frames are required. Each correction is bounded; true means converged.
    // Insufficient/ambiguous geometry leaves the pose untouched.
    bool relocalize(const SensorSample& sample);
    const EstimatorConfig& config() const { return config_; }

    static constexpr std::size_t historyCapacity = 64;

private:
    using Covariance = std::array<std::array<double, 4>, 4>;
    struct State {
        Pose pose{};
        OdometryTravel odometry{};
        Covariance p{};
        double bias = 0, v = 0, omega = 0;
        double vSmooth = 0, omegaSmooth = 0;
    };
    struct Step {
        double dt = 0, forward = 0, lateral = 0, heading = 0;
        double stationaryHeading = 0; // Independent wheel yaw in a bounded quiet interval.
        double forwardRotationOffset = 0;
        double slip = 0, biasReading = 0, biasVariance = 0;
        bool forwardPod = false, lateralPod = false, translation = false;
        bool stationary = false, observeBias = false;
        HeadingSource headingSource = HeadingSource::unavailable;
    };
    struct Event {
        double timestamp = 0, distance = 0;
        int confidence = 0;
        bool confidenceAvailable = true;
        std::size_t sensor = 0;
    };
    struct Node {
        State state{};
        Step step{};
        double timestamp = 0;
        std::array<Event, maxRanges> events{};
        std::size_t count = 0;
    };
    struct Corroboration {
        double timestamp = -1, residual = 0;
        int wall = -1;
        unsigned count = 0;
    };
    struct Ray {
        double distance = 0, incidence = 0;
        std::array<double, 4> h{};
        int wall = -1;
        bool valid = false;
    };

    EstimatorConfig config_;
    State state_{};
    Estimate output_{};
    SensorSample previous_{};
    // Last changed reading/baseline per forward, lateral, left, right, gyro.
    std::array<double, 5> lastCounterChange_{};
    bool initialized_ = false;
    // New increments cannot reconstruct the path missed during a long blackout.
    // Only a known pose or a converged absolute recovery restores this prior.
    bool lostPrior_ = false;
    // A bounded recovery may span many range frames. Its intermediate pose
    // remains untrusted until recovery converges or reset supplies a new prior.
    bool recovering_ = false;
    double stationaryTime_ = 0, missingTranslationTime_ = 0, missingHeadingTime_ = 0;
    double stationaryGyro_ = 0, stationaryGyroTime_ = -1;
    double stationaryWheelYaw_ = 0;
    std::array<double, 4> stationaryMinimum_{}, stationaryMaximum_{};
    std::uint8_t stationaryWheelMask_ = 0;
    bool stationaryBoundsValid_ = false;
    std::array<Node, historyCapacity> history_{};
    std::size_t historyStart_ = 0, historySize_ = 0;
    std::array<Corroboration, maxRanges> corroboration_{};
    std::array<double, maxRanges> lastRangeTimestamp_{};
    std::array<double, maxRanges> lastRecoveryRangeTimestamp_{};
    Pose recoveryCandidate_{};
    double recoveryTimestamp_ = -1;
    unsigned recoveryCount_ = 0;

    Node& node(std::size_t logical);
    const Node& node(std::size_t logical) const;
    void clearReplay();
    void noteQuietBreak(std::uint16_t mask, const std::array<double, 4>& spans,
                        double gyroRate, double interval, double timestamp);
    void append(const Step& step, double timestamp);
    void propagate(State& state, const Step& step, double fraction) const;
    void observeBias(State& state, const Step& step) const;
    void replayNode(std::size_t logical);
    State stateAt(std::size_t logical, double timestamp) const;
    Ray raycast(const Pose& pose, std::size_t sensor) const;
    double rangeVariance(double distance, std::size_t sensor) const;
    bool correctRange(State& state, const Event& event, double* residual = nullptr) const;
    void processRange(std::size_t sensor, const RangeReading& reading, double now);
    void publish(double timestamp, const Step& step);
};
} // namespace nexus
