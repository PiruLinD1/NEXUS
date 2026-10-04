#include "nexus/types.hpp"
#define private public
#include "nexus/estimator.hpp"
#include "../src/nexus/estimator.cpp"
#undef private
#include <cstdio>
using namespace nexus;
int main() {
 Estimator e; Estimator::Step step; step.dt=.1; step.forward=.2;step.lateral=.03;step.heading=1;step.headingSource=HeadingSource::imu;step.forwardPod=step.lateralPod=step.translation=true;step.forwardRotationOffset=e.config().forwardOffset;
 for(double dt: {.01,.05,.1}) {
 step.dt=dt;step.heading=10*dt; step.forward=2*dt; step.lateral=.3*dt;
 auto whole=e.state_; auto split=whole; e.propagate(whole,step,1);e.propagate(split,step,.5);e.propagate(split,step,.5);
 std::printf("dt %g diff pose=%g pxx whole=%g split=%g pyy whole=%g split=%g ptt whole=%g split=%g\n",dt,std::hypot(whole.pose.x-split.pose.x,whole.pose.y-split.pose.y),whole.p[0][0],split.p[0][0],whole.p[1][1],split.p[1][1],whole.p[2][2],split.p[2][2]);
 }
}

