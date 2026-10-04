#include "nexus/chassis.hpp"
#include <cstdio>
#include <cstdlib>
template<class P> void waitFor(P p) { for(int i=0;i<1000 && !p();++i) pros::delay(1); if(!p()) std::exit(2); }
int main() {
    pros::MotorGroup left({1,2,3}), right({4,5,6});
    pros::Rotation forward, lateral;
    pros::Imu imu;
    nexus::HardwareConfig hc;
    auto* c = new nexus::Chassis({left,right,&imu,&forward,&lateral,{}},hc,{},{});
    c->begin();
    waitFor([&]{auto s=c->diagnostics().sensors;return s.leftValid&&s.rightValid&&s.forwardValid;});
    pros::test::pauseEstimator=true;
    waitFor([]{return pros::test::estimatorPaused.load();});
    const double start=c->diagnostics().sensors.left;
    const auto cycle=[&]{
        pros::test::pauseAfterEstimatorCycle=true;pros::test::pauseEstimator=false;
        waitFor([]{return pros::test::pauseEstimator.load()&&pros::test::estimatorPaused.load();});
    };
    for(int i=0;i<3;++i){left.position[i]=right.position[i]=.01;cycle();}
    for(int i=0;i<3;++i)left.position[i]=right.position[i]=0;
    cycle();
    const double travel=c->diagnostics().sensors.left-start;
    std::printf("raw motors returned to0; integrated side delta=%.9f mm\n",travel*1000);
    pros::test::pauseEstimator=false;pros::test::stop();
    return std::abs(travel)<1e-10?0:1;
}
