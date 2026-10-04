// Read-only sensor/configuration audit. Motor commands are constant zero.
// No nonzero-output code, motion parser, IMU calibration or counter taring.
// Reproduce the old blue/rotations configuration, then fix only 4/9 to green.
#include "main.h"
#include "pros/apix.h"
#include <cerrno>
#include <cstdio>
#include <memory>

namespace {
constexpr int ports[]={5,-4,6,-7,9,-10};
std::unique_ptr<pros::Task> reader;
void run() {
    for(int p:ports) {
        pros::c::motor_move_voltage(p,0);
        pros::c::motor_set_brake_mode(p,pros::E_MOTOR_BRAKE_COAST);
    }
    unsigned next=0,stage=0;
    const unsigned started=pros::millis();
    for(;;) {
        for(int p:ports)pros::c::motor_move_voltage(p,0);
        const unsigned now=pros::millis();
        if(stage==0 && now-started>=1000) {
            // Same constructors as the recorded passive test, after PROS init.
            static pros::MotorGroup left({5,-4,6},pros::MotorGears::blue,pros::MotorUnits::rotations);
            static pros::MotorGroup right({-7,9,-10},pros::MotorGears::blue,pros::MotorUnits::rotations);
            stage=1;
            std::printf("AUDITCFG,%u,stage1_all_blue_like_USBODOM5B\n",now);
        }
        if(stage==1 && now-started>=4000) {
            const int a=pros::c::motor_set_gearing(-4,pros::E_MOTOR_GEARSET_18);
            const int b=pros::c::motor_set_gearing(9,pros::E_MOTOR_GEARSET_18);
            stage=2;
            std::printf("AUDITCFG,%u,stage2_ports4and9_green,%d,%d\n",now,a,b);
        }
        if(now>=next) {
            next=now+500;
            for(int p:ports) {
                errno=0;
                const auto gear=pros::c::motor_get_gearing(p);
                const auto units=pros::c::motor_get_encoder_units(p);
                const auto type=pros::c::motor_get_type(p);
                std::uint32_t stamp=0;
                const auto raw=pros::c::motor_get_raw_position(p,&stamp);
                const double pos=pros::c::motor_get_position(p);
                const double speed=pros::c::motor_get_actual_velocity(p);
                const int voltage=pros::c::motor_get_voltage(p);
                const int current=pros::c::motor_get_current_draw(p);
                const double temp=pros::c::motor_get_temperature(p);
                const int error=errno;
                std::printf("AUDIT1,%u,%d,%d,%d,%ld,%lu,%.9f,%.6f,%d,%d,%.3f,%d,%d\n",
                    now,p,int(gear),int(units),long(raw),static_cast<unsigned long>(stamp),pos,speed,voltage,current,temp,error,int(type));
            }
            const auto f=pros::c::rotation_get_position(2),l=pros::c::rotation_get_position(3);
            const auto h=pros::c::imu_get_rotation(1);
            const auto status=pros::c::imu_get_status(1);
            const auto fr=pros::c::rotation_get_reversed(2),lr=pros::c::rotation_get_reversed(3);
            std::printf("AUDITS,%u,%ld,%ld,%.9f,%u,%ld,%ld\n",now,long(f),long(l),h,unsigned(status),long(fr),long(lr));
            pros::lcd::print(0,"USB AUDIT | SOLO LETTURA");
            pros::lcd::print(1,"Motori: 0 V / liberi");
            pros::lcd::print(2,"Cartucce: fase %u / 2",stage);
            pros::lcd::print(3,"F %ld L %ld centigradi",long(f),long(l));
            pros::lcd::print(4,"IMU %.3f deg | stato %u",h,unsigned(status));
            pros::lcd::print(5,"Conteggi e rapporti via USB");
            pros::lcd::print(6,"Nessun movimento programmato");
            pros::lcd::print(7,"Campione %lu ms",static_cast<unsigned long>(now));
        }
        pros::delay(10);
    }
}
}
void initialize() {
    pros::c::serctl(SERCTL_DISABLE_COBS,nullptr);
    std::setvbuf(stdout,nullptr,_IONBF,0);
    pros::lcd::initialize();
    reader=std::make_unique<pros::Task>(run,TASK_PRIORITY_DEFAULT-1,TASK_STACK_DEPTH_DEFAULT,"read-only-audit");
}
void disabled() {}
void competition_initialize() {}
void autonomous() {for(;;)pros::delay(100);}
void opcontrol() {for(;;)pros::delay(100);}
