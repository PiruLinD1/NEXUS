#include "nexus/estimator.hpp"
#include <cstdio>
#include <vector>
using namespace nexus;
int main() {
    for (double delay : {0.0, .005, .010, .020, .040}) {
        EstimatorConfig config;
        config.forwardOffset=.020; config.lateralOffset=-.070; config.trackWidth=.287;
        Estimator estimator(config);
        estimator.setGyroBias(0, square(radians(.02)));
        constexpr double amplitude=.250, frequency=1.0, duration=20;
        const double thetaAmplitude=radians(3), k=thetaAmplitude/amplitude;
        auto yAt=[](double t) {return t<=0 || t>=duration ? 0.0 : amplitude*std::sin(2*pi*frequency*t);};
        SensorSample sample;
        sample.forwardValid=sample.lateralValid=sample.leftValid=sample.rightValid=sample.gyroValid=true;
        for(int i=0;i<=2100;++i) {
            double t=i*.010, y=yAt(t), theta=k*y;
            sample.timestamp=t;
            // Analytic integrals for truth x=0, y(t), theta(t)=k*y(t).
            // The raw counters all return to zero after the closed trajectory.
            const double center=std::sin(theta)/k;
            sample.forward=center-config.forwardOffset*theta;
            sample.lateral=(std::cos(theta)-1)/k+config.lateralOffset*theta;
            sample.left=center+config.trackWidth*.5*theta;
            sample.right=center-config.trackWidth*.5*theta;
            sample.gyro=k*yAt(t-delay);
            estimator.update(sample);
        }
        const auto out=estimator.estimate();
        std::printf("gyro delay %5.1f ms: x=%8.3f y=%7.3f mm, h=%.4f deg; rejected=%u; bias=%.6f deg/s; final raw F/L/G %.12f %.12f %.12f\n",
            delay*1000,out.state.pose.x*1000,out.state.pose.y*1000,degrees(out.state.pose.theta),out.rejectedIncrements,
            degrees(out.gyroBias),sample.forward,sample.lateral,sample.gyro);
    }
}
