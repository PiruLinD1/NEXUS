#include "nexus/chassis.hpp"
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <vector>

using namespace nexus;
void check(bool okay, const char* what) {
    if (!okay) { std::cerr << "FAIL " << what << '\n'; std::exit(1); }
}
template<class Predicate> bool eventually(Predicate p, unsigned timeout = 1500) {
    const auto start = pros::millis();
    do { if (p()) return true; pros::delay(1); } while (pros::millis() - start < timeout);
    return p();
}
int main(int argc, char**) {
    pros::MotorGroup left({5,-4,6}), right({-7,9,-10});
    pros::Rotation forward, lateral;
    pros::Imu imu;
    HardwareConfig hw;
    hw.forwardWheelDiameter = hw.lateralWheelDiameter = 2;
    hw.driveWheelPerMotor = .6;
    hw.lateralPodReversed = true;
    EstimatorConfig cfg;
    cfg.nativeImuHeading = true;
    cfg.trackWidth = .287;
    cfg.forwardOffset = .020;
    cfg.lateralOffset = -.070;
    cfg.maxSensorHold = .010;
    cfg.stationaryEncoderSpan = .000010;
    const double podQuantum = pi * .0508 / 36000;
    const double drivePerDegree = pi * 3.25 * inch * .6 / 360;
    double fTotal = 3.456, lTotal = -.789, leftTotal = 0, rightTotal = 0;
    Pose truth;
    auto writeRaw = [&] {
        forward.position = static_cast<int>(std::llround(fTotal / podQuantum));
        lateral.position = -static_cast<int>(std::llround(lTotal / podQuantum));
        imu.rotation = degrees(truth.theta);
        for (unsigned i = 0; i < 3; ++i) {
            left.position[i] = leftTotal / drivePerDegree;
            right.position[i] = rightTotal / drivePerDegree;
        }
    };
    writeRaw();
    auto* chassis = new Chassis({left,right,&imu,&forward,&lateral,{}}, hw, cfg, {});
    chassis->begin();
    check(eventually([&] { auto s=chassis->diagnostics().sensors; return s.forwardValid && s.lateralValid && s.leftValid && s.rightValid && s.gyroValid; }), "initial sensor baseline");
    pros::test::pauseEstimator = true;
    check(eventually([] { return pros::test::estimatorPaused.load(); }), "initial pause");
    auto tick = [&] {
        pros::test::pauseAfterEstimatorCycle = true;
        pros::test::pauseEstimator = false;
        check(eventually([] { return pros::test::pauseEstimator.load() && pros::test::estimatorPaused.load(); }), "step acknowledged");
        return chassis->diagnostics();
    };
    chassis->setPose(0,0,0);
    (void)tick();
    struct Increment { double f,l,a; };
    std::vector<Increment> path;
    double previousAngle=0;
    for (unsigned i=1;i<=64;++i) {
        const double phase=2*pi*i/64;
        const double angle=.5*std::sin(phase);
        path.push_back({.006,.002*std::sin(phase),angle-previousAngle});
        previousAngle=angle;
    }
    const auto outbound=path;
    if (argc > 1) {
        // Close by a different route: straight back to the origin with both
        // body translation components, not the inverse sequence of counters.
        Pose turningEnd;
        for (const auto& d:outbound) {
            const double sinc=std::abs(d.a)<1e-12 ? 1 : 2*std::sin(d.a/2)/d.a;
            const double mid=turningEnd.theta+d.a/2;
            turningEnd.x += sinc*(d.f*std::sin(mid)+d.l*std::cos(mid));
            turningEnd.y += sinc*(d.f*std::cos(mid)-d.l*std::sin(mid));
            turningEnd.theta += d.a;
        }
        for (unsigned i=0;i<64;++i) path.push_back({-turningEnd.y/64,-turningEnd.x/64,0});
    } else {
        for (auto it=outbound.rbegin();it!=outbound.rend();++it) path.push_back({-it->f,-it->l,-it->a});
    }
    double maxPositionError=0, maxHeadingError=0;
    for (const auto& d:path) {
        // Exact constant-twist rigid-body increment, before pod quantization.
        const double sinc=std::abs(d.a)<1e-12 ? 1 : 2*std::sin(d.a/2)/d.a;
        const double mid=truth.theta+d.a/2;
        truth.x += sinc*(d.f*std::sin(mid)+d.l*std::cos(mid));
        truth.y += sinc*(d.f*std::cos(mid)-d.l*std::sin(mid));
        truth.theta += d.a;
        fTotal += d.f-cfg.forwardOffset*d.a;
        lTotal += d.l+cfg.lateralOffset*d.a;
        leftTotal += d.f+cfg.trackWidth*d.a/2;
        rightTotal += d.f-cfg.trackWidth*d.a/2;
        writeRaw();
        const auto data=tick();
        check(data.sensors.forwardValid && data.sensors.lateralValid && data.sensors.leftValid && data.sensors.rightValid && data.sensors.gyroValid,"all channels remain valid");
        check(data.estimate.rejectedIncrementMask==0,"no rejected increment");
        check(data.estimate.headingSource==HeadingSource::imu && data.estimate.gyroBias==0,"native IMU source");
        maxPositionError=std::max(maxPositionError,std::hypot(data.estimate.state.pose.x-truth.x,data.estimate.state.pose.y-truth.y));
        maxHeadingError=std::max(maxHeadingError,std::abs(wrap(data.estimate.state.pose.theta-truth.theta)));
    }
    const auto end=chassis->diagnostics().estimate;
    std::cout<<std::setprecision(12)<<"frames="<<path.size()<<" peak_position_error_mm="<<maxPositionError/millimeter
        <<" peak_heading_error_deg="<<degrees(maxHeadingError)<<" final_x_mm="<<end.state.pose.x/millimeter
        <<" final_y_mm="<<end.state.pose.y/millimeter<<" final_heading_deg="<<degrees(end.state.pose.theta)<<'\n';
    check(maxPositionError<.000050,"complete adapter path stays within 0.05 mm quantization bound");
    check(maxHeadingError<1e-10,"complete adapter preserves native IMU angle");
    pros::test::stop();
}
