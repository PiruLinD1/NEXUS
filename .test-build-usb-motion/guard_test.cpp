#include "include/pulse_guard.hpp"
#include <cassert>
#include <limits>
#include <cstdio>
int main() {
    assert(PulseGuard::controllerConnected(1)); assert(PulseGuard::controllerConnected(2));
    assert(!PulseGuard::controllerConnected(0)); assert(!PulseGuard::controllerConnected(INT32_MAX));
    assert(!PulseGuard::controllerConnected(-1));
    std::array<double,6> enc{};
    auto ready = [&] { PulseGuard g; g.tick(1000,true,true,enc); g.beat(1000,1000); return g; };
    auto start = [&](PulseGuard& g) { assert(g.startPulse(1000,1000,1,1200,1200,200,true,enc)); };
    { auto g=ready(); assert(!g.startPulse(1000,1000,1,1200,1200,200,false,enc)); assert(!g.startPulse(1000,1000,1,1200,1200,200,true,enc)); }
    { auto g=ready(); start(g); g.tick(1010,false,true,enc); assert(!g.active && g.reason==1); g.beat(1020,1020); g.tick(1020,true,true,enc); assert(!g.active); }
    { auto g=ready(); start(g); g.tick(1121,true,true,enc); assert(!g.active && g.reason==2); }
    { auto g=ready(); start(g); g.beat(1190,1190); g.tick(1200,true,true,enc); assert(!g.active && g.reason==3); }
    { auto g=ready(); assert(!g.startPulse(1000,899,1,1200,1200,200,true,enc)); }
    { auto g=ready(); assert(!g.startPulse(1000,1000,1,3001,0,100,true,enc)); }
    { auto g=ready(); assert(!g.startPulse(1000,1000,1,1200,1200,2001,true,enc)); }
    { auto g=ready(); start(g); auto e=enc; e[4]=600; g.tick(1010,true,true,e); assert(!g.active && g.reason==4); }
    { auto g=ready(); start(g); g.tick(1010,true,false,enc); assert(!g.active && g.fault); }
    { auto g=ready(); auto e=enc; e[0]=std::numeric_limits<double>::quiet_NaN(); g.tick(1010,true,true,e); assert(g.fault); }
    { auto g=ready(); auto e=enc; for(int j=0;j<91;++j) {e[1]=(j%2)?0:10; g.tick(1010+j,true,true,e);} assert(g.fault); assert(!g.startPulse(1110,1110,1,1000,1000,100,true,e)); }
    { auto g=ready(); assert(g.startPulse(1000,1000,1,3000,3000,1000,true,enc)); for(unsigned t=1010;t<2000;t+=10) {g.beat(t,t);g.tick(t,true,true,enc);assert(g.active);} g.beat(2000,2000);g.tick(2000,true,true,enc);assert(!g.active && g.reason==3); }
    { auto g=ready(); assert(g.startPulse(1000,1000,1,3000,1000,2000,true,enc)); for(unsigned t=1010;t<3000;t+=10) {g.beat(t,t);g.tick(t,true,true,enc);g.envelope(100,500,30);assert(g.active);} g.beat(3000,3000);g.tick(3000,true,true,enc);assert(!g.active && g.reason==3); }
    for(auto p: {std::array<double,3>{451,0,0},{0,801,0},{-51,0,0},{0,-51,0},{0,0,56},{0,0,-11}}) {auto g=ready();start(g);g.envelope(p[0],p[1],p[2]);assert(!g.active && g.fault && g.reason==9);}
    { auto g=ready(); start(g); assert(!g.startPulse(1010,1010,2,1200,1200,200,true,enc)); assert(g.start==1000); }
    { auto g=ready(); g.heartbeatSeen=false; assert(!g.startPulse(1000,1000,1,1000,1000,100,true,enc)); }
    std::puts("PASS: pulse expiry, consent release, stale/absent heartbeat, replay, voltage/duration, each encoder, session path, invalid sensor, no restart");
}
