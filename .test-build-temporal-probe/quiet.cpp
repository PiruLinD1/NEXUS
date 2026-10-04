#include "nexus/estimator.hpp"
#include <cstdio>
using namespace nexus;
int main() {
    for (double noise : {0.0, pi*2*inch/36000}) {
        EstimatorConfig config;
        config.forwardOffset=.020; config.lateralOffset=-.070; config.trackWidth=.287;
        Estimator estimator(config);
        estimator.setGyroBias(0,square(radians(.01)));
        SensorSample sample;
        sample.forwardValid=sample.lateralValid=sample.leftValid=sample.rightValid=sample.gyroValid=true;
        estimator.update(sample);
        for(int i=1;i<=12000;++i) {
            sample.timestamp=i*.01;
            sample.forward=(i%2)*noise;
            sample.lateral=(i%2)*noise;
            estimator.update(sample);
            if(i==2000||i==12000) {
                auto out=estimator.estimate();
                std::printf("pod jitter=%.6f mm t=%.0fs stationary=%d pose %.6f,%.6f mm sigma %.2fmm/%.2fdeg bias_sigma=%.4fdeg/s\n",
                    noise*1000,sample.timestamp,out.stationary,out.state.pose.x*1000,out.state.pose.y*1000,
                    std::sqrt(out.covariance[0][0]+out.covariance[1][1])*1000,degrees(std::sqrt(out.covariance[2][2])),degrees(out.gyroBiasStd));
            }
        }
    }
}
