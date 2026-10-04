#include "../include/nexus/types.hpp"
#define private public
#include "../include/nexus/controller.hpp"
#undef private
#include <iostream>
#include "../src/nexus/controller.cpp"
using namespace nexus;
int main(){for(double dx:{.01,.02,.05,.1,.2}){
 Controller c;MotionOptions o;o.maxSpeed=1.5;o.positionTolerance=.01;Target goal{{dx,0,0},true,false};c.start({},goal,o);
 auto metric=[&](){double a=0,l=0;for(size_t i=1;i<c.pathCount_;++i){a+=std::abs(c.path_[i].pose.theta-c.path_[i-1].pose.theta);l+=std::hypot(c.path_[i].pose.x-c.path_[i-1].pose.x,c.path_[i].pose.y-c.path_[i-1].pose.y);}return std::array<double,3>{c.trajectoryDuration(),degrees(a),l};};auto direct=metric();std::cout<<"direct x="<<dx<<" duration="<<direct[0]<<" angle="<<direct[1]<<" len="<<direct[2]<<'\n';
 for(double lead:{.06,.1,.15,.2,.3}){c.target_.pose={dx,lead,0};c.buildPath({},false);auto first=metric();c.target_=goal;c.buildPath({{dx,lead,0},0,0},true);auto second=metric();std::cout<<" cusp lead="<<lead<<" duration="<<first[0]+second[0]<<" angle="<<first[1]+second[1]<<" len="<<first[2]+second[2]<<'\n';}
}}

