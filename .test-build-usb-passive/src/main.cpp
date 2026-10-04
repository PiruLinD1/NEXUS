// USBODOM 5: passive sensor recording only. No input/motion command parser.
#include "main.h"
#include "pros/apix.h"
#include <atomic>
#include <cmath>
#include <cstdio>
#include <memory>

namespace {
constexpr double pi=3.14159265358979323846, podMm=50.8*pi/36000.;
pros::c::queue_t records=nullptr;
std::unique_ptr<pros::Task> startup,acquisition,logger;
struct Record {
    unsigned ms=0,cycle=0,seq=0,marker=0,markerOk=0,connected=0,valid=0,drops=0;
    double f=0,l=0,h=0,gx=0,gy=0,gz=0,ax=0,ay=0,az=0,pitch=0,roll=0,enc[6]{};
    unsigned long long stamp[8]{};
};
void emit() {
    Record r; unsigned display=0;
    for(;;) {
        if(!pros::c::queue_recv(records,&r,100)) continue;
        std::printf("UP1,%u,%u,%u,%u,%u,%u,%u,%u,%.0f,%.0f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
          r.ms,r.cycle,r.seq,r.marker,r.markerOk,r.connected,r.valid,r.drops,
          r.f,r.l,r.f*podMm,-r.l*podMm,r.h,r.gx,r.gy,r.gz,r.ax,r.ay,r.az,r.pitch,r.roll);
        for(double v:r.enc) std::printf(",%.9f",v);
        for(auto v:r.stamp) std::printf(",%llu",v);
        std::printf("\n");
        if(r.ms-display>=200) {
            display=r.ms;
            pros::lcd::print(0,"USBODOM 5B | SOLO REGISTRAZIONE");
            pros::lcd::print(1,"Motori: 0 V / liberi");
            const char* states[]={"0/3 PRONTO ALLA PARTENZA","1/3 PARTENZA SALVATA","2/3 CHECKPOINT SALVATO","3/3 ARRIVO SALVATO - FINE"};
            const char* next[]={"A sul controller: salva partenza","Prossimo: vai al checkpoint","Prossimo: torna alla partenza","Resta fermo: acquisisco i dati"};
            const unsigned stage=r.marker<3?r.marker:3;
            pros::lcd::print(2,"%s",states[stage]);
            pros::lcd::print(3,"%s",next[stage]);
            pros::lcd::print(4,"F %.1f L %.1f H %.2f",r.f*podMm,-r.l*podMm,r.h);
            pros::lcd::print(5,"Ciclo %u ms | persi %u",r.cycle,r.drops);
            pros::lcd::print(6,"Sensori %u/31 | pad %s",r.valid,r.connected?"OK":"NO");
            pros::lcd::print(7,"FERMO 2s prima/dopo tasto A");
        }
    }
}
[[noreturn]] void fail(const char* message) {
    pros::lcd::print(1,"ERRORE %s",message);
    for(;;) {std::printf("UP1_ERROR,%s\n",message);pros::delay(1000);}
}
void run() {
    static pros::MotorGroup left({5,-4,6},pros::MotorGears::blue,pros::MotorUnits::rotations);
    static pros::MotorGroup right({-7,9,-10},pros::MotorGears::blue,pros::MotorUnits::rotations);
    left.move_voltage(0);right.move_voltage(0);
    left.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);right.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    static pros::Imu imu(1);
    static pros::Rotation forward(2),lateral(3);
    static pros::Controller pad(pros::E_CONTROLLER_MASTER);
    pros::lcd::print(1,"Calibrazione IMU: FERMO");
    if(imu.reset(true)!=PROS_SUCCESS) fail("calibrazione IMU");
    if(imu.set_data_rate(5)!=PROS_SUCCESS || forward.set_data_rate(5)!=PROS_SUCCESS || lateral.set_data_rate(5)!=PROS_SUCCESS) fail("frequenza sensori");
    if(forward.set_reversed(false)!=PROS_SUCCESS || lateral.set_reversed(false)!=PROS_SUCCESS) fail("verso pod");
    records=pros::c::queue_create(256,sizeof(Record));if(!records) fail("coda");
    logger=std::make_unique<pros::Task>(emit,TASK_PRIORITY_DEFAULT-1,TASK_STACK_DEPTH_DEFAULT,"passive-log");
    acquisition=std::make_unique<pros::Task>([] {
        unsigned last=pros::millis(),seq=0,marker=0,lastMark=0,drops=0;
        bool released=false;
        for(;;) {
            // No controller key or USB input can request a nonzero output.
            left.move_voltage(0);right.move_voltage(0);
            Record r;r.ms=pros::millis();r.cycle=r.ms-last;last=r.ms;r.seq=++seq;
            r.stamp[0]=pros::micros();r.f=forward.get_position();r.stamp[1]=pros::micros();
            r.stamp[2]=pros::micros();r.l=lateral.get_position();r.stamp[3]=pros::micros();
            r.stamp[4]=pros::micros();r.h=imu.get_rotation();const auto g=imu.get_gyro_rate();const auto a=imu.get_accel();
            r.pitch=imu.get_pitch();r.roll=imu.get_roll();r.stamp[5]=pros::micros();
            r.gx=g.x;r.gy=g.y;r.gz=g.z;r.ax=a.x;r.ay=a.y;r.az=a.z;
            r.stamp[6]=pros::micros();const auto le=left.get_position_all(),re=right.get_position_all();r.stamp[7]=pros::micros();
            r.valid=(r.f!=PROS_ERR?1u:0u)|(r.l!=PROS_ERR?2u:0u)|
              (std::isfinite(r.h)&&std::isfinite(g.x)&&std::isfinite(g.y)&&std::isfinite(g.z)&&!imu.is_calibrating()?4u:0u)|
              (std::isfinite(a.x)&&std::isfinite(a.y)&&std::isfinite(a.z)&&std::isfinite(r.pitch)&&std::isfinite(r.roll)?8u:0u);
            bool motorOk=le.size()==3 && re.size()==3;
            if(motorOk) for(unsigned i=0;i<3;++i) {r.enc[i]=le[i];r.enc[i+3]=re[i];motorOk=motorOk&&std::isfinite(le[i])&&std::isfinite(re[i]);}
            if(motorOk)r.valid|=16;
            const int connected=pad.is_connected(),button=pad.get_digital(pros::E_CONTROLLER_DIGITAL_A);
            r.connected=connected>0 && connected!=PROS_ERR;
            if(r.connected && button==0)released=true;
            if(r.connected && button==1 && released) {
                released=false;
                if(marker<3 && r.ms-lastMark>=1000) {++marker;lastMark=r.ms;}
            }
            if(!r.connected)released=false;
            r.marker=marker;r.markerOk=r.connected;r.drops=drops;
            if(!pros::c::queue_append(records,&r,0))++drops;
            pros::delay(10);
        }
    },TASK_PRIORITY_DEFAULT+2,TASK_STACK_DEPTH_DEFAULT,"passive-read");
}
}
void initialize() {
    pros::c::serctl(SERCTL_DISABLE_COBS,nullptr);std::setvbuf(stdout,nullptr,_IONBF,0);
    pros::lcd::initialize();pros::lcd::print(0,"USBODOM 5B | SOLO REGISTRAZIONE");
    startup=std::make_unique<pros::Task>(run,TASK_PRIORITY_DEFAULT-1,TASK_STACK_DEPTH_DEFAULT,"passive-start");
}
void disabled() {}
void competition_initialize() {}
void autonomous() {for(;;)pros::delay(100);}
void opcontrol() {for(;;)pros::delay(100);}
