#include "nexus/types.hpp"
#define private public
#include "nexus/controller.hpp"
#undef private
#define main original_main
#include "../tests/controller_test.cpp"
#undef main

void testLongLatency(unsigned frames) {
    DynamicsConfig dynamics;
    dynamics.commandLatency = .02 * frames;
    Controller controller(dynamics);
    controller.start({}, {{.7,1.1,radians(60)},true,false});
    Estimate estimate; estimate.health=Health::healthy;
    std::array<Voltage,8> commands{};
    bool done=false;
    unsigned iterations=0;
    for (;iterations<700;++iterations) {
        auto voltage=controller.update(estimate,.02);
        auto applied=commands[iterations%frames];
        commands[iterations%frames]=voltage;
        estimate.state=plant(estimate.state,applied,.02,dynamics);
        if(controller.settled(estimate,.02)){done=true;break;}
    }
    std::cout<<"LATENCY " << dynamics.commandLatency << " done="<<done<<" time="<<iterations*.02<<" pos="<<std::hypot(estimate.state.pose.x-.7,estimate.state.pose.y-1.1)<<" heading="<<degrees(wrap(estimate.state.pose.theta-radians(60)))<<"\n";
}
int main() {
    MotionOptions options;
    Controller controller;
    for (auto goal : {Pose{0.9,1.2,radians(90)}, Pose{0,-1,0}, Pose{.06,.1,radians(90)}, Pose{.1,0,0}, Pose{0,-1,radians(180)}, Pose{.0006,.0006,radians(90)}}) {
        controller.start({}, {goal,true,false}, options);
        double worst = 0, headingRateError=0;
        for (std::size_t i=1;i<controller.pathCount_;++i) {
            auto a=controller.path_[i-1], b=controller.path_[i];
            headingRateError=std::max(headingRateError,std::abs((b.pose.theta-a.pose.theta)/(b.time-a.time)-(a.omega+b.omega)/2));
            for (double sign : {-1.0,1.0}) worst=std::max(worst,std::abs((b.v+sign*b.omega*controller.config().trackWidth/2-a.v-sign*a.omega*controller.config().trackWidth/2)/(b.time-a.time)));
        }
        std::cout << "PROFILE " << goal.x << "," << goal.y << " acceleration " << worst << " headingRateError="<<headingRateError<<"\n";
    }
    run("lateral pose", {}, {{.4,0,0},true,false},options);
    run("tiny lateral", {}, {{.01,0,0},true,false},options);
    MotionOptions reversed=options; reversed.reverse=true;
    run("reverse curve", {}, {{-.8,-1.2,radians(65)},true,false},reversed);
    run("reverse ahead", {}, {{0,1,0},false,false},reversed);
    run("turn 180", {}, {{0,0,pi},true,true},options);
    run("zero distance", {}, {{0,0,radians(90)},true,false},options);
    run("rolling lateral", {{0,0,0},.7,0}, {{.4,0,0},true,false},options);
    run("rolling behind", {{0,0,0},.7,0}, {{0,-1,0},true,false},options);
    testLongLatency(2);
    testLongLatency(3);
    testLongLatency(5);
    testLongLatency(7);
    return 0;
}
