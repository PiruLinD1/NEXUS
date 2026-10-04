#define main controller_suite_main
#include "../tests/controller_test.cpp"
#undef main
int main(int argc, char**) {
    if (argc > 1) return controller_suite_main();
    for (int scenario=0; scenario<12; ++scenario) {
        DynamicsConfig model; model.trackWidth=.287; model.maxLateralAcceleration=1.;
        DynamicsConfig actual=model;
        actual.kVLeft*=1.09; actual.kVRight*=.94;
        actual.kALeft*=1.1; actual.kARight*=.85;
        actual.kSLeft=1.0; actual.kSRight=1.2;
        Controller controller(model); controller.setClock(clockSeconds); controller.setBudgetMs(500);
        MotionOptions options; options.maxSpeed=1.5; options.positionTolerance=.01;
        const double sign=scenario%2 ? -1.:1.;
        Target target{{sign*.6,.6,0},true,false};
        if (scenario>=8) target={{0, .9, 0},true,false};
        Estimate e; e.health=Health::healthy;
        e.covariance[0][0]=e.covariance[1][1]=square(.05);
        e.covariance[2][2]=square(radians(6));
        DriveState truth;
        controller.start(truth,target,options);
        Voltage previous{};
        const double grip=scenario<4 ? 0. : scenario<8 ? .08 : .2;
        double elapsed=0,nearAt=-1,extra=0,reverse=0,rotation=0,smoothV=0,smoothW=0,smoothL=0;
        double initialDuration=controller.trajectoryDuration();
        bool pushed=false,done=false;
        const auto filter=[](double measured, double& smooth, double& out) {
            const double f=.02/.07,d=std::exp(-f),a=1-d;
            out+=a*(measured-out)+(measured-smooth)*f*d; smooth+=a*(measured-smooth);
        };
        for (unsigned i=0;i<700;++i) {
            const double remaining=std::hypot(truth.pose.x-target.pose.x,truth.pose.y-target.pose.y);
            if (!pushed && scenario%4>=2 && remaining<.38) {
                truth.pose.x+=sign*.045;truth.pose.theta+=sign*radians(9);pushed=true;
            }
            e.state.pose=truth.pose;
            filter(truth.v,smoothV,e.state.v);filter(truth.omega,smoothW,e.state.omega);filter(truth.lateralV,smoothL,e.state.lateralV);
            const auto u=controller.update(e,.02);
            for(unsigned k=0;k<20;++k) {
                const auto old=truth;
                truth=plant(old,k<10?previous:u,.001,actual);
                truth.lateralV=grip>0 ? old.lateralV+.001*(-old.v*old.omega-old.lateralV/grip):0;
                truth.pose.x+=.001*truth.lateralV*std::cos(old.pose.theta);
                truth.pose.y-=.001*truth.lateralV*std::sin(old.pose.theta);
                rotation+=std::abs(wrap(truth.pose.theta-old.pose.theta));
            }
            previous=u;elapsed+=.02;
            if(std::hypot(truth.pose.x-target.pose.x,truth.pose.y-target.pose.y)<.05 && nearAt<0)nearAt=elapsed;
            if(nearAt>=0) {extra+=std::hypot(truth.v,truth.lateralV)*.02;reverse+=std::max(0.,-truth.v)*.02;}
            auto actualEstimate=e; actualEstimate.state=truth;
            if(controller.settled(actualEstimate,.02)){done=true;break;}
        }
        std::cout<<scenario<<","<<done<<","<<elapsed<<","<<elapsed-nearAt<<","<<extra*1000<<","<<reverse*1000<<","<<degrees(rotation)<<","<<initialDuration<<","<<std::hypot(truth.pose.x-target.pose.x,truth.pose.y-target.pose.y)*1000<<"\n";
    }
}
