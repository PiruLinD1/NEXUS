#include "../include/nexus/types.hpp"
#define private public
#include "../include/nexus/controller.hpp"
#undef private
#define main original_controller_test_main
#include "../tests/controller_test.cpp"
#undef main
int main() {
 for(double lat : {.02,.05,.1,.2}) for(double longitudinal : {0.,-.03}) {
  Controller ctrl; MotionOptions o; o.positionTolerance=.01; ctrl.start({},{{lat,longitudinal,0},true,false},o);
  for(bool rev:{false,true}) {ctrl.buildPath({},rev);double travel=0,angle=0;for(size_t i=1;i<ctrl.pathCount_;++i) {travel+=std::hypot(ctrl.path_[i].pose.x-ctrl.path_[i-1].pose.x,ctrl.path_[i].pose.y-ctrl.path_[i-1].pose.y);angle+=std::abs(ctrl.path_[i].pose.theta-ctrl.path_[i-1].pose.theta);} std::cout<<"path lat="<<lat<<" forward="<<longitudinal<<" reverse="<<rev<<" duration="<<ctrl.trajectoryDuration()<<" travel="<<travel<<" turn="<<degrees(angle)<<'\n';}
 }
 for(double tau : {0.,.05,.1,.2,.4}) for(bool finalHeading:{false,true}) {
  DynamicsConfig model; model.commandLatency=.01; Controller ctrl(model); MotionOptions o; o.maxSpeed=1.5;o.positionTolerance=.01;
  Estimate e; e.health=Health::healthy; Target target{{.6,.6,0},finalHeading,false}; ctrl.start(e.state,target,o);
  double lateral=0,time=0,travel=0,back=0,angle=0,firstNear=-1,maxlat=0; bool done=false;unsigned switches=0,replans=0;bool lastReverse=false;double lastReplan=0;Voltage previous{};
  for(unsigned i=0;i<1000;++i) {
   const auto u=ctrl.update(e,.02);
   for(unsigned j=0;j<20;++j) {
    const auto before=e.state; e.state=plant(e.state,j<10?previous:u,.001,model);
    lateral=tau==0?0:lateral+.001*(-before.v*before.omega-lateral/tau);
    maxlat=std::max(maxlat,std::abs(lateral));
    e.state.pose.x+=.001*lateral*std::cos(before.pose.theta);e.state.pose.y-=.001*lateral*std::sin(before.pose.theta);
   }
   previous=u;time+=.02;travel+=std::hypot(e.state.v,lateral)*.02;back+=std::max(0.,-e.state.v)*.02;angle+=std::abs(e.state.omega)*.02;
   if(ctrl.pathReverse_!=lastReverse){++switches;lastReverse=ctrl.pathReverse_;}if(ctrl.lastReplan_!=lastReplan){++replans;lastReplan=ctrl.lastReplan_;}
   if(std::hypot(e.state.pose.x-.6,e.state.pose.y-.6)<.05&&firstNear<0)firstNear=time;
   if(ctrl.settled(e,.02)){done=true;break;}
  }
  std::cout<<"omni tau="<<tau<<" heading="<<finalHeading<<" done="<<done<<" time="<<time<<" first50="<<firstNear<<" err="<<1000*std::hypot(e.state.pose.x-.6,e.state.pose.y-.6)<<" heading="<<degrees(e.state.pose.theta)<<" travel="<<travel<<" back="<<back<<" turn="<<degrees(angle)<<" switches="<<switches<<" replans="<<replans<<" latpeak="<<maxlat<<" latend="<<lateral<<'\n';
 }
}
#include "../src/nexus/controller.cpp"
