#define main controller_suite_main
#include "../tests/controller_test.cpp"
#undef main
int main() {
    for (double uncertainty : {0., .05, .20}) for (double grip : {0., .05, .2, .5}) for (bool filtered : {false, true}) {
        DynamicsConfig model;
        model.trackWidth = .287;
        model.maxLateralAcceleration = 1.;
        Controller controller(model);
        controller.setClock(clockSeconds);
        controller.setBudgetMs(500);
        MotionOptions options;
        options.maxSpeed = 1.2; options.positionTolerance = .050;
        Estimate e; e.health = Health::healthy;
        e.covariance[0][0] = e.covariance[1][1] = square(uncertainty);
        e.covariance[2][2] = square(radians(10));
        controller.start({}, {{.6,.6,0},false,false}, options);
        double rotation=0, previousTheta=0, elapsed=0;
        DriveState truth;
        double smoothV=0, smoothW=0, smoothL=0;
        const auto filter=[](double measured, double& smooth, double& output) {
            const double interval=.02/.07, decay=std::exp(-interval), alpha=1-decay;
            output+=alpha*(measured-output)+(measured-smooth)*interval*decay;
            smooth+=alpha*(measured-smooth);
        };
        bool done=false;
        while(elapsed<10) {
            auto u=controller.update(e,.02,12);
            auto next=plant(truth,u,.02,model);
            if(grip>0) {
                next.lateralV=truth.lateralV+.02*(-truth.v*truth.omega-truth.lateralV/grip);
                next.pose.x += .02*next.lateralV*std::cos(next.pose.theta);
                next.pose.y -= .02*next.lateralV*std::sin(next.pose.theta);
            }
            truth=next;
            if(filtered) {
                e.state.pose=next.pose;
                filter(truth.v,smoothV,e.state.v);
                filter(truth.omega,smoothW,e.state.omega);
                filter(truth.lateralV,smoothL,e.state.lateralV);
            } else e.state=next;
            rotation += std::abs(wrap(next.pose.theta-previousTheta)); previousTheta=next.pose.theta;
            elapsed+=.02;
            if(controller.settled(e,.02)) {done=true;break;}
        }
        std::cout<<"probe sigma="<<uncertainty<<" grip="<<grip<<" filter="<<filtered<<" settled="<<done<<" t="<<elapsed<<" x="<<e.state.pose.x<<" y="<<e.state.pose.y<<" rotation="<<degrees(rotation)<<'\n';
    }
}
