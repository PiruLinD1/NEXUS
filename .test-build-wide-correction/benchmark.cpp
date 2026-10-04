#define main controller_suite_main
#include "../tests/controller_test.cpp"
#undef main
int main(int argc, char**) {
    if (argc>1) return controller_suite_main();
    for (unsigned scenario=0;scenario<24;++scenario) {
        DynamicsConfig model;model.trackWidth=.287;model.maxLateralAcceleration=1.;
        DynamicsConfig actual=model;
        actual.kVLeft*=1.05;actual.kVRight*=.96;actual.kSLeft=.9;actual.kSRight=1.1;
        Controller controller(model);controller.setClock(clockSeconds);controller.setBudgetMs(500);
        MotionOptions options;options.positionTolerance=scenario<12?.001:.005;
        const double errors[]={.003,.006,.012,.025,.050,.080};
        const double side=scenario%2?-1.:1.;
        const double error=side*errors[(scenario/2)%6];
        controller.start({},{{0,0,0},true,false},options);
        Estimate e;e.health=Health::healthy;e.covariance[0][0]=e.covariance[1][1]=square(.03);
        DriveState truth{{error,-.003,side*radians(3)},0,0};
        Voltage previous{};double elapsed=0,travel=0,rotation=0,peak=0,sv=0,sw=0,sl=0;
        double grip=.08;int gear=0,switches=0,refGear=0,refSwitches=0;bool done=false;
        const auto filter=[](double value,double& smooth,double& out) {
          const double f=.02/.07,d=std::exp(-f),a=1-d;
          out+=a*(value-out)+(value-smooth)*f*d;smooth+=a*(value-smooth);
        };
        for(unsigned i=0;i<600;++i) {
            e.state.pose=truth.pose;filter(truth.v,sv,e.state.v);filter(truth.omega,sw,e.state.omega);filter(truth.lateralV,sl,e.state.lateralV);
            const auto u=controller.update(e,.02);
            const double rv=controller.reference().v;
            const int rg=rv>.02?1:rv<-.02?-1:0;
            if(rg){if(refGear&&refGear!=rg)++refSwitches;refGear=rg;}
            for(unsigned k=0;k<20;++k){
                const auto old=truth;truth=plant(old,k<10?previous:u,.001,actual);
                truth.lateralV=old.lateralV+.001*(-old.v*old.omega-old.lateralV/grip);
                truth.pose.x+=.001*truth.lateralV*std::cos(old.pose.theta);
                truth.pose.y-=.001*truth.lateralV*std::sin(old.pose.theta);
                travel+=std::hypot(truth.v,truth.lateralV)*.001;
                rotation+=std::abs(wrap(truth.pose.theta-old.pose.theta));
            }
            previous=u;elapsed+=.02;
            peak=std::max(peak,std::hypot(truth.pose.x,truth.pose.y));
            const int g=truth.v>.02?1:truth.v<-.02?-1:0;
            if(g){if(gear&&gear!=g)++switches;gear=g;}
            auto arrival=e;arrival.state=truth;
            if(controller.settled(arrival,.02)){done=true;break;}
        }
        std::cout<<scenario<<","<<done<<","<<elapsed<<","<<switches<<","<<refSwitches<<","<<travel*1000<<","<<degrees(rotation)<<","<<peak*1000<<","<<std::hypot(truth.pose.x,truth.pose.y)*1000<<"\n";
    }
}
