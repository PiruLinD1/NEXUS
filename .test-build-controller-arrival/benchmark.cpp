#define main controller_existing_test_main
#include "../tests/controller_test.cpp"
#undef main
#include <fstream>

int main(int argc, char** argv) {
    const bool dry = argc > 1;
    for (int scenario=0; scenario<6; ++scenario) {
        DynamicsConfig model;
        model.trackWidth=.287; model.commandLatency=.01;
        DynamicsConfig actual=model;
        if(scenario>=1) { actual.kVLeft*=1.12; actual.kVRight*=.9; actual.kALeft*=1.2; actual.kARight*=.9; }
        if(scenario>=2) { actual.kSLeft=1.6; actual.kSRight=1.35; }
        Controller controller(model); controller.setClock(clockSeconds); controller.setBudgetMs(500);
        MotionOptions options; options.maxSpeed=1.2; options.positionTolerance=.01;
        options.timeout=30;
        const Target target{{.6,.6,0},scenario!=5,false};
        Estimate e; e.health=Health::healthy;
        if(scenario>=3) {e.covariance[0][0]=e.covariance[1][1]=.005;e.covariance[2][2]=square(radians(6));}
        controller.start(e.state,target,options);
        double travel=0,backward=0,angle=0,time=0,firstNear=-1;
        Voltage previous{};bool settled=false;
        std::ofstream trace;
        if(argc>2 && scenario==4) {trace.open(argv[2]);trace<<"time,x,y,h,v,w,ul,ur,rx,ry,rh,rv,rw\n";}
        for(int step=0;step<1500;++step) {
            if(scenario==4 && step==55) {e.state.pose.x+=.035;e.state.pose.theta+=radians(4);}
            auto u=controller.update(e,.02,12);
            auto advance=[&](Voltage v) {
                if(dry) {
                    // Coulomb breakaway, versus the controller's smooth tanh.
                    const double vl=e.state.v+e.state.omega*actual.trackWidth/2;
                    const double vr=e.state.v-e.state.omega*actual.trackWidth/2;
                    auto load=[](double voltage,double speed,double ks) {
                        const double friction=std::abs(speed)<.002 ? std::clamp(voltage,-ks,ks) : std::copysign(ks,speed);
                        return voltage-friction;
                    };
                    v={load(v.left,vl,actual.kSLeft),load(v.right,vr,actual.kSRight)};
                    auto frictionless=actual;frictionless.kSLeft=frictionless.kSRight=0;
                    e.state=plant(e.state,v,.01,frictionless);
                } else e.state=plant(e.state,v,.01,actual);
            };
            advance(previous); advance(u); previous=u;
            travel+=std::abs(e.state.v)*.02;
            backward+=std::max(0.,-e.state.v)*.02;
            angle+=std::abs(e.state.omega)*.02;
            time+=.02;
            const double error=std::hypot(e.state.pose.x-.6,e.state.pose.y-.6);
            if(error<.05 && firstNear<0)firstNear=time;
            auto ref=controller.reference();
            if(trace)trace<<time<<','<<e.state.pose.x<<','<<e.state.pose.y<<','<<e.state.pose.theta<<','<<e.state.v<<','<<e.state.omega<<','<<u.left<<','<<u.right<<','<<ref.pose.x<<','<<ref.pose.y<<','<<ref.pose.theta<<','<<ref.v<<','<<ref.omega<<'\n';
            if(controller.settled(e,.02)) {settled=true;break;}
        }
        std::cout<<"scenario="<<scenario<<" dry="<<dry<<" done="<<settled<<" seconds="<<time<<" first50mm="<<firstNear<<" error_mm="<<1000*std::hypot(e.state.pose.x-.6,e.state.pose.y-.6)<<" heading="<<degrees(e.state.pose.theta)<<" travel="<<travel<<" reverse="<<backward<<" angle="<<degrees(angle)<<'\n';
    }
}
