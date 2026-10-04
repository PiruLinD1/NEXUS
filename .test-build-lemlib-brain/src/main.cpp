// Isolated hardware A/B test; no NEXUS code or actuator commands.
// LemLib files under src/lemlib and include/lemlib are the unmodified
// v0.5.6 sources at 3388145e0d90ee3c7c17d7267a30ef64367df4cb (MIT).
#include "main.h"
#include "lemlib/chassis/odom.hpp"
#include "lemlib/chassis/trackingWheel.hpp"

#include <cmath>
#include <cstdio>
#include <memory>

namespace {
constexpr double mmPerInch = 25.4;
std::unique_ptr<pros::Task> startupTask;

[[noreturn]] void fail(const char* message) {
    pros::lcd::print(1, "ERRORE: %s", message);
    pros::lcd::print(2, "Controlla sensori, riavvia");
    std::printf("LL06_ERROR,%s\n", message);
    for (;;) pros::delay(100);
}

void runReadOnlyTest() {
    static pros::Imu imu(1);
    static pros::Rotation forwardRotation(2);
    // NEXUS uses reversed port -3 for right-positive lateral travel. The pinned
    // LemLib update projects a positive horizontal counter LEFT, so use +3.
    static pros::Rotation horizontalRotation(3);
    static pros::MotorGroup rightReadings({-7, 9, -10});

    pros::lcd::print(1, "Calibrazione IMU: FERMO");
    if (imu.reset(true) != PROS_SUCCESS) fail("calibrazione IMU");
    if (imu.set_data_rate(5) != PROS_SUCCESS ||
        forwardRotation.set_reversed(false) != PROS_SUCCESS ||
        horizontalRotation.set_reversed(false) != PROS_SUCCESS ||
        forwardRotation.set_data_rate(5) != PROS_SUCCESS ||
        horizontalRotation.set_data_rate(5) != PROS_SUCCESS)
        fail("configurazione sensori");

    static lemlib::TrackingWheel forward(&forwardRotation, 2.0f, 20.0f / 25.4f);
    static lemlib::TrackingWheel horizontal(&horizontalRotation, 2.0f, -70.0f / 25.4f);
    // Upstream update dereferences vertical2 even with one pod + IMU. This is
    // the same real motor-backed substitute Chassis::calibrate would construct.
    // Its constructor sets encoder units to rotations; it never commands motion.
    // Its reading is unused for pose: the pod is selected and the IMU supplies yaw.
    static lemlib::TrackingWheel rightSubstitute(&rightReadings, 3.25f, 143.5f / 25.4f, 360.0f);
    forward.reset();
    horizontal.reset();
    pros::delay(50); // Allow the reset counters to reach the PROS shared buffers.
    if (forwardRotation.get_position() == PROS_ERR || horizontalRotation.get_position() == PROS_ERR ||
        forwardRotation.get_reversed() != 0 || horizontalRotation.get_reversed() != 0 ||
        !std::isfinite(imu.get_rotation())) fail("lettura sensori");

    lemlib::setSensors({&forward, &rightSubstitute, &horizontal, nullptr, &imu},
                      {nullptr, &rightReadings, 287.0f / 25.4f, 3.25f, 360.0f, 2.0f});
    lemlib::setPose({0, 0, 0});
    // Unmodified LemLib task: update(), then pros::delay(10). No sensor wrappers,
    // extra fusion, deadbands, local bias, or resampling between it and PROS.
    lemlib::init();

    std::printf("LL06_CONFIG,commit=3388145e0d90ee3c7c17d7267a30ef64367df4cb,imu=1,forward_rotation=2,horizontal_rotation=3,diameter_in=2.0,forward_offset_mm=20,horizontal_offset_mm=-70,device_rate_ms=5,odom_delay_ms=10,native_calibration=1,no_actuator_commands=1\n");
    unsigned rows = 0;
    for (;;) {
        // These are ordinary upstream getters for display/10Hz telemetry. Raw
        // telemetry reads are separate from the odometry task's own acquisition.
        const auto pose = lemlib::getPose();
        const double rawForward = forward.getDistanceTraveled() * mmPerInch;
        const double rawRight = -horizontal.getDistanceTraveled() * mmPerInch;
        const double rawImu = imu.get_rotation();
        if (++rows % 100 == 1)
            std::printf("LL06_HEADER,ms,x_mm,y_mm,heading_deg,forward_mm,lateral_right_mm,imu_deg\n");
        std::printf("LL06,%lu,%.4f,%.4f,%.6f,%.4f,%.4f,%.6f\n",
                    static_cast<unsigned long>(pros::millis()), pose.x * mmPerInch, pose.y * mmPerInch,
                    static_cast<double>(pose.theta), rawForward, rawRight, rawImu);
        pros::lcd::print(0, "LEMLIB 0.5.6 | 3388145");
        pros::lcd::print(1, "X %.1f mm | Y %.1f mm", pose.x * mmPerInch, pose.y * mmPerInch);
        pros::lcd::print(2, "Heading %.2f deg", static_cast<double>(pose.theta));
        pros::lcd::print(3, "F %.1f | L destra %.1f mm", rawForward, rawRight);
        pros::lcd::print(4, "IMU1 F2 H3 | ruote 2.0in");
        pros::lcd::print(5, "Sposta solo a mano");
        pros::lcd::print(6, "Motori: nessun comando");
        pros::lcd::print(7, "Riavvia app per azzerare");
        pros::delay(100);
    }
}
} // namespace

void initialize() {
    pros::lcd::initialize();
    pros::lcd::print(0, "LEMLIB 0.5.6 | 3388145");
    // Return before the blocking IMU calibration. Priority 7 is below the PROS
    // initialize callback's 8, avoiding the known kernel stale-deadline catch-up.
    startupTask = std::make_unique<pros::Task>(runReadOnlyTest, TASK_PRIORITY_DEFAULT - 1,
                                              TASK_STACK_DEPTH_DEFAULT, "lemlib-readonly");
}

void disabled() {}
void competition_initialize() {}
void autonomous() {}
void opcontrol() {}
