#include "nexus/estimator.hpp"
#include <algorithm>
#include <cstdio>
using namespace nexus;
int main() {
    constexpr double length=.5, turn=pi/6, radius=length/turn;
    std::array<std::array<Pose,202>,2> reference;
    for(double pollDt : {.005,.010,.020,0.0}) for(bool heldGyro : {false,true}) {
        EstimatorConfig cfg;
        cfg.forwardOffset=.020; cfg.lateralOffset=-.070; cfg.trackWidth=.287;
        cfg.maxSensorHold=.010; cfg.stationaryEncoderSpan=.000010;
        Estimator estimator(cfg);
        estimator.setGyroBias(0,square(radians(.02)));
        SensorSample sample;
        sample.forwardValid=sample.lateralValid=sample.leftValid=sample.rightValid=sample.gyroValid=true;
        double largestError=0;
        double timestampPoseDifference=0;
        Pose endpoint;
        const auto progress=[](int frame) {frame=std::clamp(frame,0,200); return (frame<=100 ? frame : 200-frame)/100.;};
        for(int frame=0;frame<=201;++frame) {
            const double fraction=progress(frame), theta=turn*fraction, distance=length*fraction;
            sample.timestamp=pollDt>0 ? frame*pollDt : frame*.010-(frame%2 ? .004 : 0);
            sample.forward=distance-cfg.forwardOffset*theta;
            sample.lateral=cfg.lateralOffset*theta;
            sample.left=distance+cfg.trackWidth/2*theta;
            sample.right=distance-cfg.trackWidth/2*theta;
            sample.gyro=turn*progress(frame-(heldGyro ? 1 : 0));
            const auto pose=estimator.update(sample).state.pose;
            if(pollDt==.010)reference[heldGyro][frame]=pose;
            if(pollDt==0) {
                const auto regular=reference[heldGyro][frame];
                timestampPoseDifference=std::max({timestampPoseDifference,std::abs(pose.x-regular.x),
                    std::abs(pose.y-regular.y),std::abs(pose.theta-regular.theta)});
            }
            const double trueX=radius*(1-std::cos(theta)), trueY=radius*std::sin(theta);
            largestError=std::max(largestError,std::hypot(pose.x-trueX,pose.y-trueY));
            if(frame==100)endpoint=pose;
        }
        const auto e=estimator.estimate();
        const char* timing=pollDt==.005 ? "5ms" : pollDt==.010 ? "10ms" : pollDt==.020 ? "20ms" : "6/14ms";
        std::printf("poll %s gyro %s: endpoint %.6f %.6f mm; returned %.6f %.6f mm, heading %.8f deg; maxError %.6f mm rejected %u\n",
            timing,heldGyro?"previous frame":"current frame",endpoint.x/millimeter,endpoint.y/millimeter,
            e.state.pose.x/millimeter,e.state.pose.y/millimeter,degrees(e.state.pose.theta),largestError/millimeter,e.rejectedIncrements);
        if(pollDt==0)std::printf("  all-frame max pose difference vs regular10ms: %.17g SI\n",timestampPoseDifference);
    }
}
