#include "nexus/estimator.hpp"
#include "lemlib/chassis/odom.hpp"
#include <cstdio>
#include <cstdlib>
extern float prevVertical, prevVertical1, prevVertical2, prevHorizontal, prevHorizontal1, prevHorizontal2, prevImu;

int main() {
    // The same sensor values straddle a setPose request at t=.005. Ground truth
    // differs only in when the initial 3mm motion occurred: t=.002..004 or
    // t=.006..008. Neither implementation receives a sample at the reset time.
    for (bool movementBeforeReset : {true, false}) {
        nexus::EstimatorConfig config;
        config.nativeImuHeading = true;
        config.forwardOffset=.020; config.lateralOffset=-.070;
        nexus::Estimator estimator(config);
        nexus::SensorSample sample;
        sample.forwardValid=sample.lateralValid=sample.gyroValid=true;
        lemlib::TrackingWheel f,l,sub;
        f.offset=.020/nexus::inch;l.offset=-.070/nexus::inch;sub.type=1;
        pros::Imu gyro;
        prevVertical=prevVertical1=prevVertical2=prevHorizontal=prevHorizontal1=prevHorizontal2=prevImu=0;
        lemlib::setSensors({&f,&sub,&l,nullptr,&gyro},{nullptr,nullptr,0,0,0,0});
        lemlib::setPose({0,0,0},true);
        estimator.update(sample);lemlib::update();
        estimator.reset({},.005);lemlib::setPose({0,0,0},true);

        sample.timestamp=.010;sample.forward=.003;f.value=sample.forward/nexus::inch;
        estimator.update(sample);lemlib::update();
        const double expectedAfterReading=movementBeforeReset?0:3;
        const double nexusAfterReading=estimator.estimate().state.pose.y/nexus::millimeter;
        const double lemAfterReading=lemlib::getPose(true).y*25.4;
        // Add a later 30mm excursion, then return to the physical reset point.
        sample.timestamp=.020;sample.forward=.033;f.value=sample.forward/nexus::inch;
        estimator.update(sample);lemlib::update();
        sample.timestamp=.030;sample.forward=movementBeforeReset?.003:0;
        f.value=sample.forward/nexus::inch;
        estimator.update(sample);lemlib::update();
        std::printf("Motion %s reset: first reading truth %.3fmm, Nexus %.6fmm, Lem %.6fmm; physical return truth0, Nexus %.6fmm, Lem %.6fmm\n",
            movementBeforeReset?"BEFORE":"AFTER",expectedAfterReading,nexusAfterReading,lemAfterReading,
            estimator.estimate().state.pose.y/nexus::millimeter,lemlib::getPose(true).y*25.4);
        if(estimator.estimate().rejectedIncrements)std::abort();
    }
    std::puts("This is the unavoidable reset-time acquisition ambiguity, not evidence for the recorded S residual.");
}
