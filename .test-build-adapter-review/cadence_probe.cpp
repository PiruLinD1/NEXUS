#include "nexus/estimator.hpp"
#include <cstdio>
int main() {
    nexus::EstimatorConfig cfg;
    cfg.forwardOffset = .020;
    cfg.lateralOffset = -.070;
    nexus::Estimator e(cfg);
    nexus::SensorSample s;
    s.forwardValid = s.lateralValid = s.gyroValid = true;
    s.leftValid = s.rightValid = true;
    e.update(s);
    s.timestamp = .01;
    s.left = s.right = .027;
    e.update(s); // Pod frame is held while drive counters arrive.
    s.timestamp = .02;
    s.forward = s.left = s.right = .054;
    auto out = e.update(s);
    std::printf("cadence: expected y=54mm; y=%.3fmm; rejection=%u; mask=%u\n",
                out.state.pose.y * 1000, out.rejectedIncrements, out.rejectedIncrementMask);

    for (double lag : {0.0, .01}) {
        nexus::Estimator est(cfg);
        est.setGyroBias(0, nexus::square(nexus::radians(.001)));
        nexus::SensorSample sample;
        sample.forwardValid = sample.lateralValid = sample.gyroValid = true;
        sample.leftValid = sample.rightValid = true;
        est.update(sample);
        double previousY = 0, previousTheta = 0;
        for (int j = 1; j <= 20000; ++j) {
            const double t = j * .001;
            const double y = .3 * std::sin(2 * nexus::pi * t);
            const double theta = nexus::radians(2) * std::sin(2 * nexus::pi * t);
            const double angle = theta - previousTheta;
            const double mid = (theta + previousTheta) / 2;
            const double chord = std::abs(angle) < 1e-10 ? 1 : std::sin(angle / 2) / (angle / 2);
            const double f = (y - previousY) * std::cos(mid) / chord;
            const double l = -(y - previousY) * std::sin(mid) / chord;
            sample.forward += f - cfg.forwardOffset * angle;
            sample.lateral += l + cfg.lateralOffset * angle;
            sample.left += f + cfg.trackWidth / 2 * angle;
            sample.right += f - cfg.trackWidth / 2 * angle;
            previousY = y; previousTheta = theta;
            if (j % 10 == 0) {
                sample.timestamp = t;
                sample.gyro = nexus::radians(2) * std::sin(2 * nexus::pi * std::max(0.0, t - lag));
                est.update(sample);
            }
        }
        sample.gyro = 0;
        for (int j = 0; j < 50; ++j) { sample.timestamp += .01; est.update(sample); }
        const auto result = est.estimate();
        std::printf("20 closed cycles, IMU lag=%.0fms: X=%.3fmm Y=%.3fmm h=%.5fdeg reject=%u\n",
            lag * 1000, result.state.pose.x * 1000, result.state.pose.y * 1000,
            nexus::degrees(result.state.pose.theta), result.rejectedIncrements);
    }
}
