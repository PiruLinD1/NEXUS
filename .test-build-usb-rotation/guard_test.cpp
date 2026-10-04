#include "include/pulse_guard.hpp"
#include <cassert>
#include <limits>
#include <cstdio>
int main() {
    std::array<double,6> enc{};
    auto ready=[&] { PulseGuard g; g.observeHeading(0); g.tick(1000,true,true,enc); g.beat(1000,1000); return g; };
    auto start=[&](PulseGuard& g,int direction=1) { assert(g.startPulse(1000,1000,1,3000*direction,-3000*direction,2000,true,enc)); };
    { auto g=ready(); assert(!g.startPulse(1000,1000,1,3000,3000,2000,true,enc)); }
    { auto g=ready(); assert(!g.startPulse(1000,1000,1,0,0,2000,true,enc)); }
    { auto g=ready(); assert(!g.startPulse(1000,1000,1,3001,-3001,2000,true,enc)); }
    { auto g=ready(); assert(!g.startPulse(1000,1000,1,3000,-3000,2001,true,enc)); }
    for(int direction:{-1,1}) {
        auto g=ready();start(g,direction);
        g.observeHeading(direction*89.9); g.tick(1010,true,true,enc); assert(g.active);
        g.observeHeading(direction*90.0); g.tick(1020,true,true,enc); assert(!g.active && g.reason==10 && g.left==0 && g.right==0);
        g.beat(1030,1030);g.tick(1030,true,true,enc);assert(!g.active);
        assert(!g.startPulse(1030,1030,1,3000,-3000,2000,true,enc));
    }
    { auto g=ready();start(g);g.observeHeading(-5.1);g.tick(1010,true,true,enc);assert(g.fault && !g.active && g.reason==11); }
    { auto g=ready();start(g);g.tick(1010,false,true,enc);assert(!g.active && g.reason==1); }
    { auto g=ready();start(g);g.tick(1121,true,true,enc);assert(!g.active && g.reason==2); }
    { auto g=ready();start(g);for(unsigned t=1010;t<=3000;t+=10) {g.beat(t,t);g.tick(t,true,true,enc);} assert(!g.active && g.reason==3); }
    { auto g=ready();start(g);auto e=enc;e[3]=300;g.tick(1010,true,true,e);assert(!g.active && g.reason==4); }
    { auto g=ready();start(g);auto e=enc;e[0]=1250;g.tick(1010,true,true,e);assert(g.fault && !g.active && g.reason==5); }
    { auto g=ready();start(g);g.tick(1010,true,false,enc);assert(g.fault && !g.active); }
    { auto g=ready();start(g);g.observeHeading(std::numeric_limits<double>::quiet_NaN());assert(g.fault && !g.active); }
    { auto g=ready();start(g);g.observeHeading(400.1);assert(g.fault && !g.active); }
    for(auto p:{std::array<double,2>{151,0},{-151,0},{0,151},{0,-151}}) {auto g=ready();start(g);g.envelope(p[0],p[1],0);assert(g.fault && !g.active);}
    { auto g=ready();for(unsigned q=0;q<4;++q) {
        unsigned now=1000+q*1000;g.beat(now,now);
        assert(g.startPulse(now,now,q+1,3000,-3000,2000,true,enc));
        for(unsigned i=1;i<=90;++i) {unsigned t=now+i*10;g.beat(t,t);g.observeHeading(q*90+i);g.tick(t,true,true,enc);}
        assert(!g.active && !g.fault && g.reason==10);
    }}
    std::puts("PASS: quarter angle stop both directions, wrong-direction latch, no straight commands, watchdogs, time/travel/envelope limits, four-quarter sequence");
}
