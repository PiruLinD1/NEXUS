#include "nexus/estimator.hpp"
#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <vector>
using namespace nexus;
constexpr double fineDt = .0001, duration = 20;
constexpr int endTick = 200000;
struct Truth { double x, y, theta, dx, dy; };
Truth truth(int type, double t) {
    const double w = 2*pi;
    const double s = std::sin(w*t), c = std::cos(w*t);
    const double s2 = std::sin(2*w*t), c2 = std::cos(2*w*t);
    switch (type) {
    case 0: return {0, .25*s, 0, 0, .25*w*c};
    case 1: return {.25*s, 0, 0, .25*w*c, 0};
    case 2: return {.15*s, .25*s, 0, .15*w*c, .25*w*c};
    case 3: return {.15*s, .25*s, radians(3)*s, .15*w*c, .25*w*c};
    default: return {.15*s2+.02*(1-c), .25*s,
        radians(3)*s+radians(1)*(1-c2), .30*w*c2+.02*w*s, .25*w*c};
    }
}
struct Delay {const char* name; std::array<int,5> ticks;};
int main(int argc, char** argv) {
    const int sampleTicks=argc>1 ? std::max(1,std::atoi(argv[1])) : 100;
    const double lateralScale=argc>2 ? std::atof(argv[2]) : 1;
    std::printf("sampling interval %.3f ms; lateral scale %.9f\n",sampleTicks*fineDt*1000,lateralScale);
    const char* paths[] = {"forward locked yaw", "lateral locked yaw", "diagonal locked yaw", "diagonal plus yaw", "S plus yaw"};
    const Delay delays[] = {{"synchronous", {0,0,0,0,0}}, {"gyro 5ms", {0,0,0,0,50}},
        {"gyro 10ms", {0,0,0,0,100}}, {"forward 10ms", {100,0,0,0,0}},
        {"lateral 10ms", {0,100,0,0,0}}, {"both pods 10ms", {100,100,0,0,0}},
        {"all sensors 10ms", {100,100,100,100,100}}};
    for (int path=0; path<5; ++path) {
        std::vector<std::array<double,5>> reading(endTick+1);
        double centre = 0, sideways = 0;
        for (int tick=1; tick<=endTick; ++tick) {
            const auto mid = truth(path,(tick-.5)*fineDt);
            centre += (std::sin(mid.theta)*mid.dx + std::cos(mid.theta)*mid.dy)*fineDt;
            sideways += (std::cos(mid.theta)*mid.dx - std::sin(mid.theta)*mid.dy)*fineDt;
            const double theta = truth(path,tick*fineDt).theta;
            reading[tick] = {centre-.020*theta, sideways-.070*theta,
                centre+.287/2*theta, centre-.287/2*theta, theta};
        }
        for (const auto& delay : delays) for (bool tolerant : {false,true}) {
            EstimatorConfig config;
            config.forwardOffset=.020; config.lateralOffset=-.070; config.trackWidth=.287;
            config.maxSensorHold=.010;
            config.lateralScale=lateralScale;
            config.stationaryEncoderSpan=tolerant ? .000010 : 0;
            Estimator estimator(config);
            estimator.setGyroBias(0,square(radians(.02)));
            SensorSample sample;
            sample.forwardValid=sample.lateralValid=sample.leftValid=sample.rightValid=sample.gyroValid=true;
            unsigned movingQuiet=0;
            double worst=0;
            for (int tick=0; tick<=endTick+10000; tick+=sampleTicks) {
                const auto read=[&](int i) {return reading[std::clamp(tick-delay.ticks[i],0,endTick)][i];};
                sample.timestamp=tick*fineDt;
                sample.forward=read(0); sample.lateral=read(1);
                sample.left=read(2); sample.right=read(3); sample.gyro=read(4);
                const auto e=estimator.update(sample);
                const auto real = tick<=endTick ? truth(path,tick*fineDt) : Truth{};
                worst=std::max(worst,std::hypot(e.state.pose.x-real.x,e.state.pose.y-real.y));
                if (tick<endTick && e.stationary) ++movingQuiet;
            }
            const auto e=estimator.estimate();
            std::printf("%-20s %-16s quiet%u: X%9.4f Y%9.4f H%8.4f; maxErr%8.3f mm; movingQuiet%u reject%u bias%g\n",
                paths[path],delay.name,tolerant,e.state.pose.x/millimeter,e.state.pose.y/millimeter,
                degrees(e.state.pose.theta),worst/millimeter,movingQuiet,e.rejectedIncrements,degrees(e.gyroBias));
        }
    }
}
