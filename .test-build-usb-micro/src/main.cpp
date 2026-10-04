// USBMICRO 1. Two bounded small arcs; R1 deadman and USB heartbeat required.
#include "main.h"
#include "pros/apix.h"
#include "lemlib/chassis/odom.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "nexus/estimator.hpp"
#include "pulse_guard.hpp"
#include <atomic>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <memory>
#include <cerrno>

namespace {
constexpr double pi=3.14159265358979323846;
constexpr double podMm=50.8*pi/36000.0, driveMm=82.55*pi*0.6;
std::atomic<bool> forcedStop{false}, modeEnabled{false};
pros::c::queue_t commands=nullptr,records=nullptr;
std::unique_ptr<pros::Task> startup,input,control,logger;
struct Command { char type=0; unsigned token=0,sequence=0,duration=0; int left=0,right=0; };
struct Record {
    unsigned ms=0,cycle=0,seq=0,accepted=0,drops=0,interlocks=0;
    bool consent=false,active=false,fault=false;
    int reason=0,left=0,right=0;
    double f=0,l=0,g=0,gyro=0,enc[6]{},travel=0;
    nexus::Pose nx{}; lemlib::Pose ll{0,0,0};
};
// Blocking USB input is confined to a lower-priority task.
void receive() {
    char line[96];
    for (;;) {
        if(!std::fgets(line,sizeof(line),stdin)) { std::clearerr(stdin); pros::delay(10); continue; }
        Command c; int used=0; bool ok=false;
        if(std::strcmp(line,"STOP\n")==0) { forcedStop=true; continue; }
        if(std::sscanf(line,"H %u %n",&c.token,&used)==1 && used && line[used]=='\0') { c.type='H'; ok=true; }
        else {
            used=0;
            if(std::sscanf(line,"P %u %u %d %d %u %n",&c.token,&c.sequence,&c.left,&c.right,&c.duration,&used)==5 && used && line[used]=='\0') { c.type='P'; ok=true; }
        }
        if(!ok || !std::strchr(line,'\n') || !pros::c::queue_append(commands,&c,0)) forcedStop=true;
    }
}
void emit() {
    Record r; unsigned displayTime=0;
    for (;;) {
        if(!pros::c::queue_recv(records,&r,100)) continue;
        std::printf("UD1,%u,%u,%u,%u,%u,%d,%d,%d,%d,%d,%d,%.3f,%.3f,%.5f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.5f,%.3f,%.3f,%.5f,%u\n",
          r.ms,r.cycle,r.seq,r.accepted,r.drops,int(r.consent),int(r.active),int(r.fault),r.reason,r.left,r.right,
          r.f,r.l,r.g,r.gyro,r.enc[0],r.enc[1],r.enc[2],r.enc[3],r.enc[4],r.enc[5],r.travel,
          r.nx.x*1000,r.nx.y*1000,nexus::degrees(r.nx.theta),r.ll.x*25.4,r.ll.y*25.4,double(r.ll.theta),r.interlocks);
        if(r.ms-displayTime>=200) {
            displayTime=r.ms;
            pros::lcd::print(0,"USBMICRO 1 | max 0.5s");
            pros::lcd::print(1,"%s | consenso %s",r.fault?"BLOCCATO":(r.active?"IMPULSO":"FERMO"),r.consent?"SI":"NO");
            pros::lcd::print(2,"R1 tenuto abilita | B stop");
            pros::lcd::print(3,"NX X%.1f Y%.1f H%.2f",r.nx.x*1000,r.nx.y*1000,nexus::degrees(r.nx.theta));
            pros::lcd::print(4,"LL X%.1f Y%.1f H%.2f",r.ll.x*25.4,r.ll.y*25.4,double(r.ll.theta));
            pros::lcd::print(5,"Ultimo comando %u / OK %u",r.seq,r.accepted);
            pros::lcd::print(6,"Corsa %.0f/250mm | stop %d",r.travel,r.reason);
            pros::lcd::print(7,"Blocchi %u | R1 %d",r.interlocks,(r.interlocks&2)?0:1);
        }
    }
}
[[noreturn]] void fail(const char* why) {
    const int code=errno;
    pros::lcd::print(1,"ERRORE: %s",why);
    pros::lcd::print(2,"errno %d | verifica batteria",code);
    for(;;) { std::printf("UD1_ERROR,%s,errno=%d,battery_mv=%ld\n",why,code,long(pros::battery::get_voltage())); pros::delay(1000); }
}
void run() {
    static pros::MotorGroup left({5,-4,6},pros::MotorGears::blue,pros::MotorUnits::rotations);
    static pros::MotorGroup right({-7,9,-10},pros::MotorGears::blue,pros::MotorUnits::rotations);
    left.move_voltage(0); right.move_voltage(0);
    left.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE); right.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    left.set_current_limit_all(1000); right.set_current_limit_all(1000);
    static pros::Imu imu(1);
    static pros::Rotation forward(2),horizontal(3);
    static pros::Controller pad(pros::E_CONTROLLER_MASTER);
    pros::lcd::print(1,"Calibrazione IMU: FERMO");
    if(imu.reset(true)!=PROS_SUCCESS) fail("IMU1 calibrazione");
    if(imu.set_data_rate(5)!=PROS_SUCCESS) fail("IMU1 frequenza");
    if(forward.set_reversed(false)!=PROS_SUCCESS) fail("pod2 verso");
    if(horizontal.set_reversed(false)!=PROS_SUCCESS) fail("pod3 verso");
    if(forward.set_data_rate(5)!=PROS_SUCCESS) fail("pod2 frequenza");
    if(horizontal.set_data_rate(5)!=PROS_SUCCESS) fail("pod3 frequenza");
    static lemlib::TrackingWheel f(&forward,2.0f,20.0f/25.4f);
    static lemlib::TrackingWheel h(&horizontal,2.0f,-70.0f/25.4f);
    static lemlib::TrackingWheel substitute(&right,3.25f,143.5f/25.4f,360.0f);
    f.reset(); h.reset(); pros::delay(50);
    lemlib::setSensors({&f,&substitute,&h,nullptr,&imu},{&left,&right,287.0f/25.4f,3.25f,360.0f,2.0f});
    lemlib::setPose({0,0,0}); lemlib::init();
    static nexus::EstimatorConfig config;
    config.forwardOffset=.020; config.lateralOffset=-.070; config.trackWidth=.287;
    config.nativeImuHeading=true; config.maxSensorHold=.010; config.stationaryEncoderSpan=.000010;
    static nexus::Estimator estimator(config);
    commands=pros::c::queue_create(16,sizeof(Command)); records=pros::c::queue_create(128,sizeof(Record));
    if(!commands || !records) fail("memoria code");
    input=std::make_unique<pros::Task>(receive,TASK_PRIORITY_DEFAULT-1,TASK_STACK_DEPTH_DEFAULT,"usb-input");
    logger=std::make_unique<pros::Task>(emit,TASK_PRIORITY_DEFAULT-1,TASK_STACK_DEPTH_DEFAULT,"usb-logger");
    control=std::make_unique<pros::Task>([] {
        PulseGuard guard; unsigned last=pros::millis(),accepted=0,drops=0;
        bool released=false,hadG=false; double lastG=0;
        for(;;) {
            const unsigned now=pros::millis(),cycle=now-last; last=now;
            const auto fr=forward.get_position(),hr=horizontal.get_position();
            const double heading=imu.get_rotation(); const auto gyro=imu.get_gyro_rate();
            const auto lm=left.get_position_all(),rm=right.get_position_all();
            std::array<double,6> enc{};
            bool valid=lm.size()==3 && rm.size()==3 && fr!=PROS_ERR && hr!=PROS_ERR &&
                std::isfinite(heading) && !imu.is_calibrating() && cycle<=30;
            if(valid) for(unsigned i=0;i<3;++i) { enc[i]=lm[i]*driveMm; enc[i+3]=rm[i]*driveMm; }
            if(hadG && std::abs(heading-lastG)>5) valid=false;
            lastG=heading; hadG=true;
            const bool connected=PulseGuard::controllerConnected(pad.is_connected());
            const int r1=pad.get_digital(pros::E_CONTROLLER_DIGITAL_R1),b=pad.get_digital(pros::E_CONTROLLER_DIGITAL_B);
            if(connected && r1==0) released=true;
            const unsigned interlocks=(!connected?1u:0u)|(r1!=1?2u:0u)|(b!=0?4u:0u)|(!released?8u:0u)|
                (!modeEnabled.load()?16u:0u)|(pros::competition::is_connected()?32u:0u)|
                (pros::competition::is_disabled()?64u:0u)|(pros::competition::is_autonomous()?128u:0u);
            const bool consent=interlocks==0;
            guard.tick(now,consent,valid,enc);
            const bool stop=forcedStop.exchange(false);
            if(stop) guard.stop(7);
            Command c;
            for(unsigned j=0;j<16 && pros::c::queue_recv(commands,&c,0);++j) {
                if(c.type=='H') guard.beat(now,c.token);
                else if(c.type=='P' && !stop && guard.startPulse(now,c.token,c.sequence,c.left,c.right,c.duration,consent,enc)) accepted=c.sequence;
            }
            nexus::SensorSample sample;
            sample.timestamp=now*.001; sample.forward=fr*podMm*.001; sample.lateral=-hr*podMm*.001;
            sample.gyro=nexus::radians(heading); sample.gyroRate=nexus::radians(-gyro.z);
            sample.forwardValid=fr!=PROS_ERR; sample.lateralValid=hr!=PROS_ERR; sample.gyroValid=std::isfinite(heading);
            sample.gyroRateValid=std::isfinite(gyro.z);
            // Index 1 on each side is a fixed-200-RPM 5.5 W motor (ports 4/9).
            // Keep every motor actuated, but only use equal-ratio blue encoders.
            sample.left=(enc[0]+enc[2])*.0005; sample.right=(enc[3]+enc[5])*.0005;
            sample.leftValid=sample.rightValid=valid;
            const auto nx=estimator.update(sample);
            guard.envelope(nx.state.pose.x*1000,nx.state.pose.y*1000,nexus::degrees(nx.state.pose.theta));
            if(guard.active) { left.move_voltage(guard.left); right.move_voltage(guard.right); }
            else { left.brake(); right.brake(); }
            Record r; r.ms=now; r.cycle=cycle; r.seq=guard.sequence; r.accepted=accepted; r.drops=drops; r.interlocks=interlocks;
            r.consent=consent; r.active=guard.active; r.fault=guard.fault; r.reason=guard.reason; r.left=guard.left; r.right=guard.right;
            r.f=sample.forward*1000; r.l=sample.lateral*1000; r.g=heading; r.gyro=-gyro.z; r.nx=nx.state.pose; r.ll=lemlib::getPose();
            for(unsigned i=0;i<6;++i) { r.enc[i]=enc[i]; r.travel=std::max(r.travel,guard.travel[i]); }
            if(!pros::c::queue_append(records,&r,0)) ++drops;
            pros::delay(10);
        }
    },TASK_PRIORITY_DEFAULT+2,TASK_STACK_DEPTH_DEFAULT,"usb-guard");
}
}
void initialize() {
    pros::c::serctl(SERCTL_DISABLE_COBS,nullptr); std::setvbuf(stdout,nullptr,_IONBF,0);
    pros::lcd::initialize(); pros::lcd::print(0,"USBMICRO 1 | DISARMATO");
    startup=std::make_unique<pros::Task>(run,TASK_PRIORITY_DEFAULT-1,TASK_STACK_DEPTH_DEFAULT,"usb-start");
}
void disabled() { modeEnabled=false; forcedStop=true; }
void competition_initialize() { modeEnabled=false; forcedStop=true; }
void autonomous() { modeEnabled=false; forcedStop=true; }
void opcontrol() { modeEnabled=true; for(;;) pros::delay(100); }
