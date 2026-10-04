#include "nexus/estimator.hpp"
#include "lemlib/chassis/odom.hpp"
#include "lemlib/util.hpp"
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
using namespace nexus;
extern float prevVertical, prevVertical1, prevVertical2, prevHorizontal, prevHorizontal1, prevHorizontal2, prevImu;
extern lemlib::Pose odomSpeed, odomLocalSpeed;
static unsigned checks = 0;
static bool nativeMode = false;
void require(bool value, const char* what) { ++checks; if (!value) { std::fprintf(stderr, "FAIL: %s\n", what); std::exit(1); } }
EstimatorConfig config() {
    EstimatorConfig c;
    c.forwardOffset=.020; c.lateralOffset=-.070; c.trackWidth=.287; c.maxSensorHold=.010;
    c.nativeImuHeading=nativeMode; c.stationaryEncoderSpan=.010*millimeter;
    return c;
}
SensorSample base() { SensorSample s; s.forwardValid=s.lateralValid=s.gyroValid=true; s.leftValid=s.rightValid=nativeMode; return s; }

struct Reference {
    lemlib::TrackingWheel vertical, horizontal, substitute;
    pros::Imu imu;
    Reference() {
        vertical.offset=.020/inch; horizontal.offset=-.070/inch;
        substitute.type=1; // Chassis::calibrate supplies this drive-wheel fallback.
        lemlib::setSensors({&vertical,&substitute,&horizontal,nullptr,&imu}, {nullptr,nullptr,0,0,0,0});
    }
    void sensors(const SensorSample& s, bool mapHorizontal=true) {
        vertical.value=s.forward/inch;
        horizontal.value=(mapHorizontal ? -s.lateral : s.lateral)/inch;
        imu.degrees=degrees(s.gyro);
    }
    void reset(const SensorSample& s, Pose pose = {}) {
        sensors(s);
        prevVertical=prevVertical1=vertical.value;
        prevVertical2=0;
        prevHorizontal=prevHorizontal1=horizontal.value;
        prevHorizontal2=0;
        prevImu=lemlib::degToRad(static_cast<float>(imu.degrees));
        lemlib::setPose({float(pose.x/inch),float(pose.y/inch),float(pose.theta)},true);
        odomSpeed=odomLocalSpeed={0,0,0};
    }
    Pose update(const SensorSample& s) {
        sensors(s); lemlib::update(); const auto p=lemlib::getPose(true);
        return {p.x*inch,p.y*inch,p.theta};
    }
};

struct Comparison {
    Estimator nexus{config()};
    Reference lem;
    bool initialized=false;
    double maxPosition=0,maxHeading=0;
    Pose nexusEnd{},lemEnd{};
    Pose initialPose{};
    unsigned frames=0,quietFrames=0;
    Comparison(Pose pose = {}) : initialPose(pose) {
        nexus.reset(pose);
        require(nexus.setGyroBias(0,square(radians(.02)))!=nativeMode,"native policy must reject manual bias while legacy accepts it");
    }
    void update(SensorSample s) {
        // The earlier audit isolates legacy integration by withholding wheel
        // evidence. The native policy audit retains real drive readings and
        // quiet diagnostics, which must have no effect on the integrated angle.
        if(!nativeMode)s.leftValid=s.rightValid=s.gyroRateValid=false;
        if(!initialized) { lem.reset(s,initialPose); initialized=true; }
        nexusEnd=nexus.update(s).state.pose;
        lemEnd=lem.update(s);
        maxPosition=std::max(maxPosition,std::hypot(nexusEnd.x-lemEnd.x,nexusEnd.y-lemEnd.y));
        maxHeading=std::max(maxHeading,std::abs(wrap(nexusEnd.theta-lemEnd.theta)));
        require(nexus.estimate().rejectedIncrements==0,"test/replay contains a rejected Nexus increment");
        if(nativeMode)require(nexus.estimate().gyroBias==0,"native comparison cannot learn a local bias");
        else require(!nexus.estimate().stationary,"legacy pure integration comparison cannot enter quiet");
        quietFrames+=nexus.estimate().stationary;
        ++frames;
    }
    void print(const char* name) const {
        std::printf("%s: frames=%u quiet=%u max position difference=%.9f mm heading=%.9f deg; final Nexus=(%.6f,%.6f) LemLib=(%.6f,%.6f) mm\n",
            name,frames,quietFrames,maxPosition/millimeter,degrees(maxHeading),nexusEnd.x/millimeter,nexusEnd.y/millimeter,lemEnd.x/millimeter,lemEnd.y/millimeter);
    }
    void verify() const {
        require(maxPosition<.000050,"float LemLib vs double Nexus must stay within 0.05 mm in the synthetic replay");
        require(maxHeading<radians(.0001),"synthetic heading difference must remain below 0.0001 degree");
    }
};

void tests() {
    {
        Reference direct; auto s=base(); direct.reset(s);
        s.lateral=.100; direct.sensors(s,false); lemlib::update();
        const double x=lemlib::getPose(true).x*inch;
        require(std::abs(x+.100)<1e-8,"upstream positive horizontal raw projects toward negative world X");
        std::printf("Unadapted upstream horizontal +100mm gives world X=%.6fmm; mapping horizontal=-NexusL is explicit.\n",x/millimeter);
    }
    for(int mode=0;mode<3;++mode) {
        Comparison c; auto s=base(); c.update(s);
        for(int i=1;i<=400;++i) {
            const double travel=(i<=200 ? i : 400-i)*.003;
            s.timestamp=i*.01; s.forward=mode!=1 ? travel : 0; s.lateral=mode!=0 ? travel : 0;
            c.update(s);
        }
        c.print(mode==0?"forward reverse":mode==1?"right left":"diagonal reverse"); c.verify();
    }
    for(bool lag:{false,true}) {
        Comparison c; auto s=base();
        const auto progress=[](int n) {n=std::clamp(n,0,200); return (n<=100 ? n : 200-n)/100.;};
        for(int i=0;i<=201;++i) {
            const double p=progress(i), theta=p*pi/6;
            s.timestamp=i*.01; s.forward=.5*p-.020*theta; s.lateral=-.070*theta;
            s.gyro=progress(i-(lag?1:0))*pi/6; c.update(s);
        }
        c.print(lag?"arc + inverse, gyro previous frame":"arc + inverse, synchronized"); c.verify();
        if(lag)require(std::hypot(c.lemEnd.x,c.lemEnd.y)>.004,"same stale input also prevents LemLib arc closure");
    }
    for(bool lag:{false,true}) {
        Comparison c; auto s=base(); const double k=radians(3)/.25;
        const auto y=[](double t) {return t<=0||t>=20 ? 0.0 : .25*std::sin(2*pi*t);};
        for(int i=0;i<=2100;++i) {
            const double t=i*.01,theta=k*y(t),centre=std::sin(theta)/k;
            s.timestamp=t; s.forward=centre-.020*theta;
            s.lateral=(std::cos(theta)-1)/k-.070*theta; s.gyro=k*y(t-(lag?.01:0)); c.update(s);
        }
        c.print(lag?"20 closed manual cycles, gyro 10ms older":"20 closed manual cycles, synchronized"); c.verify();
    }
    {
        Comparison c; auto s=base(); c.update(s);
        for(int i=1;i<=6000;++i) {
            const double f=.004*std::sin(i*.013),l=.002*std::cos(i*.021),a=.009*std::sin(i*.008);
            s.timestamp=i*.01;s.forward+=f-.020*a;s.lateral+=l-.070*a;s.gyro+=a;c.update(s);
        }
        c.print("varying simultaneous forward/lateral/yaw");c.verify();
    }
    for(double angle:{1e-8,-1e-8,.012,-.012}) {
        Comparison c;auto s=base();c.update(s);
        for(int i=1;i<=1200;++i) {
            s.timestamp=i*.01;s.gyro=i*angle;s.forward=-.020*s.gyro;s.lateral=-.070*s.gyro;c.update(s);
        }
        c.print(angle>0?"clockwise pure turn":"counterclockwise pure turn");c.verify();
    }
    std::printf("LemLib differential: %u checks passed.\n",checks);
}

std::vector<std::string> split(const std::string& line) {
    std::vector<std::string> columns;std::stringstream stream(line);std::string column;
    while(std::getline(stream,column,','))columns.push_back(column);
    return columns;
}
void replay(const char* path, double start = -1, double end = 1e100) {
    std::ifstream file(path);require(bool(file),"cannot open cleaned NXOD CSV");
    std::string line;std::getline(file,line);const auto header=split(line);
    const auto column=[&](const char* name) {const auto it=std::find(header.begin(),header.end(),name);require(it!=header.end(),"missing CSV column");return std::size_t(it-header.begin());};
    const auto time=column("host_s"),f=column("f_mm"),l=column("l_mm"),g=column("imu_deg"),valid=column("valid"),epoch=column("epoch"),seq=column("seq");
    const auto x=column("x_mm"),y=column("y_mm"),heading=column("heading_deg");
    const auto left=column("left_mm"),right=column("right_mm");
    std::unique_ptr<Comparison> c;
    unsigned segments=0,skipped=0;std::uint32_t priorEpoch=0,priorSeq=0;double previousTime=-1;
    double maximumPosition=0,maximumHeading=0;
    const auto finish=[&] {
        if(!c)return;
        c->print("CSV segment");
        maximumPosition=std::max(maximumPosition,c->maxPosition);
        maximumHeading=std::max(maximumHeading,c->maxHeading);
        c.reset();
    };
    while(std::getline(file,line)) {
        const auto row=split(line);require(row.size()==header.size(),"malformed CSV: clean the NXOD log first");
        const double timestamp=std::stod(row[time]);
        if(timestamp<start||timestamp>end)continue;
        if((std::stoul(row[valid])&0x13)!=0x13) {finish();++skipped;continue;}
        auto s=base();s.timestamp=std::stod(row[time]);s.forward=std::stod(row[f])*millimeter;s.lateral=std::stod(row[l])*millimeter;s.gyro=radians(std::stod(row[g]));
        s.left=std::stod(row[left])*millimeter;s.right=std::stod(row[right])*millimeter;
        s.leftValid=bool(std::stoul(row[valid])&4);s.rightValid=bool(std::stoul(row[valid])&8);
        const auto e=static_cast<std::uint32_t>(std::stoul(row[epoch])),q=static_cast<std::uint32_t>(std::stoul(row[seq]));
        if(!c||e!=priorEpoch||q-priorSeq!=1||s.timestamp<=previousTime) {
            finish();
            // A requested subrange starts from the board's recorded pose once.
            // Subsequent frames use only F/L/IMU, never the recorded pose/bias.
            const Pose origin=start<0 ? Pose{} : Pose{std::stod(row[x])*millimeter,std::stod(row[y])*millimeter,radians(std::stod(row[heading]))};
            std::printf("CSV origin at %.6fs: (%.6f,%.6f) mm, %.9f deg\n",s.timestamp,origin.x/millimeter,origin.y/millimeter,degrees(origin.theta));
            c=std::make_unique<Comparison>(origin);++segments;
        }
        c->update(s);priorEpoch=e;priorSeq=q;previousTime=s.timestamp;
    }
    finish();
    std::printf("All CSV segments: max position difference %.9f mm; heading %.9f deg.\n",maximumPosition/millimeter,degrees(maximumHeading));
    std::printf("CSV segments=%u invalid F/L/G frames skipped=%u. Boundaries are rebaselined, never interpolated.\n",segments,skipped);
}
int main(int argc,char** argv) {
    if(argc>1 && std::string(argv[1])=="--native") {nativeMode=true;--argc;++argv;}
    std::printf("Original LemLib v0.5.6 odom.cpp, commit3388145e0d90ee3c7c17d7267a30ef64367df4cb.\nKinematic comparison only: F/L/G same input, no ranges; offsets +20/-70mm, scales1. Lem horizontal=-Nexus lateral; inches converted at API boundary.\n%s\n",
        nativeMode?"Native IMU policy: real drive validity/quiet diagnostics retained; bias disabled by policy.":"Legacy pure integration: bias0 imposed, drive validity withheld to exclude quiet constraints.");
    try {
        if(argc==1)tests();else if(argc==2)replay(argv[1]);
        else if(argc==4)replay(argv[1],std::stod(argv[2]),std::stod(argv[3]));
        else { std::fprintf(stderr,"Usage: lemlib_compare [--native] [clean.csv [start_s end_s]]\n");return 2; }
    } catch(const std::exception& e) {
        std::fprintf(stderr,"Replay error: %s\n",e.what()); return 2;
    }
}
