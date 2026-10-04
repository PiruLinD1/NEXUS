#include "robot.hpp"
#include "robot-config.hpp"
#include "robot/driver_input.hpp"
#include "robot/calibration_support.hpp"
#include "robot/mechanism.hpp"
#include "robot/arm_motor.hpp"
#include "robot/mechanism_driver.hpp"
#include "robot/mechanism_input.hpp"
#include "robot/startup.hpp"
#include "liblvgl/lvgl.h"
#include "nexus/calibration.hpp"
#include "nexus/automatic_calibration.hpp"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/error.h"
#include <atomic>
#include <bit>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <limits>

namespace robot {
pros::Controller controller(pros::E_CONTROLLER_MASTER);
namespace {
pros::MotorGroup leftMotors(config::leftPorts, pros::MotorGearset::blue);
pros::MotorGroup rightMotors(config::rightPorts, pros::MotorGearset::blue);
pros::MotorGroup liftMotors(config::liftPorts, pros::MotorGearset::green);
pros::MotorGroup intakeMotors(config::intakePorts, pros::MotorGearset::blue);
pros::Motor arm(config::braccioPort, pros::MotorGearset::green);
pros::Rotation armEncoder(config::mechanism::encoderPort);
mechanism::Controller mechanisms;
pros::adi::DigitalOut clamp(config::clampPort, !config::clampClosedLevel);
std::unique_ptr<pros::Imu> imu;
std::unique_ptr<pros::Rotation> forwardPod, lateralPod;
std::array<std::unique_ptr<pros::Distance>, nexus::maxRanges> ranges;
std::unique_ptr<nexus::Chassis> drive;
std::unique_ptr<pros::Task> displayTask;
std::atomic<bool> clampClosed{false}, intakeEnabled{false};
std::atomic<nexus::MotionStatus> lastAutonStatus{nexus::MotionStatus::idle};
constexpr int firstButton = pros::E_CONTROLLER_DIGITAL_L1;
constexpr std::size_t buttonCount = pros::E_CONTROLLER_DIGITAL_A - firstButton + 1;
detail::ButtonState<buttonCount> buttonState;
bool controllerConnected = false;
std::atomic<int> driverR2Input{0};
std::atomic<const char*> macroNotice{"SU: preMatch"};
bool imuCalibrationFailed = false; // Set before diagnostics task starts.
const char* startupBiasStatus = "IMU assente"; // Immutable after diagnostics task starts.
nexus::EstimatorConfig activeEstimatorConfig;
nexus::DynamicsConfig activeDynamicsConfig;
nexus::calibration::CalibrationProfile activeProfile;
std::atomic<std::uint32_t> activeCalibrationMask{0};
mechanism::Input mechanismInput();
void applyMacroArm();
bool configureArmEncoder();
void startupPreMatch();

bool calibrationOwnsDriver() {
    return detail::calibrationScreenActive() && !pros::competition::is_autonomous();
}

std::size_t buttonIndex(Button button) {
    return static_cast<std::size_t>(static_cast<int>(button) - firstButton);
}
std::array<int, buttonCount> readButtons() {
    std::array<int, buttonCount> values{};
    controllerConnected = detail::controllerIsConnected(controller.is_connected());
    for (std::size_t i = 0; i < buttonCount; ++i)
        values[i] = controllerConnected
            ? controller.get_digital(static_cast<Button>(firstButton + i)) : PROS_ERR;
    driverR2Input = values[buttonIndex(R2)];
    return values;
}

nexus::HardwareConfig hardwareConfig() {
    using namespace config;
    nexus::HardwareConfig cfg;
    cfg.driveWheelDiameter = driveWheelDiameter;
    cfg.driveWheelPerMotor = wheelRPM / motorRPM;
    cfg.forwardWheelDiameter = forwardWheelDiameter;
    cfg.lateralWheelDiameter = lateralWheelDiameter;
    cfg.leftMotorCount = leftPorts.size(); cfg.rightMotorCount = rightPorts.size();
    cfg.forwardPodReversed = forwardPodPort < 0;
    cfg.lateralPodReversed = lateralPodPort < 0;
    cfg.driveGearset = pros::MotorGearset::blue;
    for (std::size_t i = 0; i < wallSensors.size(); ++i) {
        cfg.rangeLatency[i] = wallSensors[i].latencyMs * 0.001;
        const auto& sensor = wallSensors[i];
        cfg.rangeCalibration[i] = {sensor.distanceScale, sensor.distanceOffset * nexus::millimeter,
            sensor.minimumDistance * nexus::millimeter, sensor.maximumDistance * nexus::millimeter};
    }
    return cfg;
}
nexus::EstimatorConfig localizationConfig() {
    using namespace config;
    nexus::EstimatorConfig cfg;
    cfg.trackWidth = trackWidth * nexus::millimeter;
    cfg.forwardOffset = forwardPodX * nexus::millimeter;
    cfg.lateralOffset = lateralPodY * nexus::millimeter;
    // V5 shared sensor values refresh at 10 ms: repeated cached counters may
    // carry two acquisition intervals of real travel in their next increment.
    cfg.maxSensorHold = 0.010;
    cfg.field = {fieldMinX * nexus::millimeter, fieldMaxX * nexus::millimeter,
                 fieldMinY * nexus::millimeter, fieldMaxY * nexus::millimeter};
    for (std::size_t i = 0; i < wallSensors.size(); ++i) {
        const auto& sensor = wallSensors[i];
        cfg.mounts[i] = {sensor.x * nexus::millimeter, sensor.y * nexus::millimeter,
                         nexus::radians(sensor.heading)};
        cfg.rangeNoise[i] = {sensor.distanceStd < 0 ? -1 : sensor.distanceStd * nexus::millimeter,
                            sensor.distanceRelativeStd};
        if (sensor.port) cfg.rangeCount = i + 1;
    }
    return cfg;
}
nexus::DynamicsConfig dynamicsConfig() {
    using namespace config;
    nexus::DynamicsConfig cfg;
    cfg.trackWidth = trackWidth * nexus::millimeter;
    // Initial estimates only. Replace with fitted values from physical CSV logs.
    cfg.kVLeft = kVLeft / nexus::millimeter; cfg.kVRight = kVRight / nexus::millimeter;
    cfg.kALeft = kALeft / nexus::millimeter; cfg.kARight = kARight / nexus::millimeter;
    cfg.kSLeft = kSLeft; cfg.kSRight = kSRight;
    cfg.maxSpeed = maxSpeed * nexus::millimeter;
    cfg.maxAcceleration = maxAcceleration * nexus::millimeter;
    cfg.maxLateralAcceleration = maxLateralAcceleration * nexus::millimeter;
    cfg.commandLatency = commandLatencyMs * 0.001;
    return cfg;
}

bool validPorts() {
    std::array<bool, 22> used{};
    const auto add = [&used](int port) {
        if (port == 0) return true;
        port = std::abs(port);
        if (port > 21 || used[port]) return false;
        used[port] = true;
        return true;
    };
    for (auto ports : {config::leftPorts, config::rightPorts, config::liftPorts, config::intakePorts})
        for (int port : ports) if (port == 0 || !add(port)) return false;
    if (config::braccioPort == 0 || !add(config::braccioPort) || !add(config::imuPort)
        || !add(config::forwardPodPort) || !add(config::lateralPodPort)
        || config::mechanism::encoderPort == 0 || !add(config::mechanism::encoderPort)) return false;
    for (auto sensor : config::wallSensors) if (!add(sensor.port)) return false;
    return true;
}

std::uint64_t fingerprint() {
    std::uint64_t hash = 14695981039346656037ULL;
    const auto add = [&hash](std::uint64_t value) {
        for (unsigned i = 0; i < 8; ++i) {
            hash ^= (value >> (8 * i)) & 255;
            hash *= 1099511628211ULL;
        }
    };
    add(config::robotRevision);
    for (auto ports : {config::leftPorts, config::rightPorts}) for (int port : ports) add(static_cast<std::uint64_t>(port));
    add(config::imuPort); add(static_cast<std::uint64_t>(config::forwardPodPort));
    add(static_cast<std::uint64_t>(config::lateralPodPort));
    for (double value : {config::trackWidth, config::driveWheelDiameter, config::wheelRPM,
         config::motorRPM, config::forwardWheelDiameter, config::lateralWheelDiameter,
         config::forwardPodX, config::lateralPodY}) add(std::bit_cast<std::uint64_t>(value));
    for (const auto& sensor : config::wallSensors) {
        add(sensor.port);
        for (double value : {sensor.x, sensor.y, sensor.heading, sensor.latencyMs})
            add(std::bit_cast<std::uint64_t>(value));
    }
    return hash;
}

void drawDiagnostics(const nexus::Diagnostics& data, unsigned page) {
    const auto& state = data.estimate.state;
    if (page == 3) {
        const auto& odom = data.estimate.odometry;
        pros::lcd::print(0, "ODOM 2 | contributi X/Y in mm");
        pros::lcd::print(1, "Avanz. X %.1f Y %.1f", odom.forwardX / nexus::millimeter,
                         odom.forwardY / nexus::millimeter);
        pros::lcd::print(2, "Later. X %.1f Y %.1f", odom.lateralX / nexus::millimeter,
                         odom.lateralY / nexus::millimeter);
        pros::lcd::print(3, "Offset X %.1f Y %.1f", odom.offsetX / nexus::millimeter,
                         odom.offsetY / nexus::millimeter);
        pros::lcd::print(4, "Totale X %.1f Y %.1f",
                         (odom.forwardX + odom.lateralX + odom.offsetX) / nexus::millimeter,
                         (odom.forwardY + odom.lateralY + odom.offsetY) / nexus::millimeter);
        pros::lcd::print(5, "Da avvio / ultimo setPose");
        pros::lcd::print(6, "Avanz.: proiezione con heading");
        pros::lcd::print(7, "LCD frecce: cambia pagina");
        return;
    }
    if (page == 2) {
        const auto& sensors = data.sensors;
        const auto& estimate = data.estimate;
        pros::lcd::print(0, "Sensori | fermo %s | IMU %s", estimate.stationary ? "SI" : "NO",
                         sensors.gyroValid ? "OK" : "--");
        pros::lcd::print(1, "Pod avanti %.1f mm [%s]", sensors.forward / nexus::millimeter,
                         sensors.forwardValid ? "OK" : "--");
        pros::lcd::print(2, "Pod laterale %.1f mm [%s]", sensors.lateral / nexus::millimeter,
                         sensors.lateralValid ? "OK" : "--");
        pros::lcd::print(3, "Ruote S %.0f%s D %.0f%s mm", sensors.left / nexus::millimeter,
                         sensors.leftValid ? "" : "?", sensors.right / nexus::millimeter,
                         sensors.rightValid ? "" : "?");
        pros::lcd::print(4, "Bias %.3f +/-%.3f deg/s", nexus::degrees(estimate.gyroBias),
                         nexus::degrees(estimate.gyroBiasStd));
        const unsigned mask = estimate.lastRejectedIncrementMask;
        pros::lcd::print(5, "Scarti %lu | ultimo %c%c%c%c%c",
                         static_cast<unsigned long>(estimate.rejectedIncrements),
                         mask & 1 ? 'F' : '-', mask & 2 ? 'L' : '-', mask & 4 ? 'S' : '-',
                         mask & 8 ? 'D' : '-', mask & 16 ? 'G' : '-');
        pros::lcd::print(6, "Stima %.1fms max %.1fms", data.estimatorMs, data.worstEstimatorMs);
        pros::lcd::print(7, "Bias avvio: %s", startupBiasStatus);
        return;
    }
    if (page == 1) {
        pros::lcd::print(0, "IMU: accel +/-4g, gyro 1000deg/s");
        if (data.sensors.accelerationValid)
            pros::lcd::print(1, "Accel %.2f %.2f %.2f g", data.sensors.accelerationG[0],
                             data.sensors.accelerationG[1], data.sensors.accelerationG[2]);
        else pros::lcd::print(1, "Accel non disponibile");
        if (data.sensors.rawGyroValid)
            pros::lcd::print(2, "Gyro %.0f %.0f %.0f deg/s", data.sensors.rawGyroDps[0],
                             data.sensors.rawGyroDps[1], data.sensors.rawGyroDps[2]);
        else pros::lcd::print(2, "Gyro non disponibile");
        pros::lcd::print(3, "Picco asse %.2fg | %.0fdeg/s", data.peakAccelerationG, data.peakGyroRateDps);
        pros::lcd::print(4, "Campioni >=3.8g: %lu", static_cast<unsigned long>(data.accelerationNearLimit));
        pros::lcd::print(5, "Campioni >=950deg/s: %lu", static_cast<unsigned long>(data.gyroNearLimit));
        pros::lcd::print(6, "Bias stimato: %.3f deg/s", nexus::degrees(data.estimate.gyroBias));
        pros::lcd::print(7, "LCD frecce: cambia pagina");
        return;
    }
    const double positionUncertainty = std::sqrt(data.estimate.covariance[0][0] + data.estimate.covariance[1][1]);
    const double headingUncertainty = std::sqrt(data.estimate.covariance[2][2]);
    pros::lcd::print(0, "X %.1f  Y %.1f mm", state.pose.x / nexus::millimeter, state.pose.y / nexus::millimeter);
    const char* headingSource = data.estimate.headingSource == nexus::HeadingSource::imu ? "IMU"
        : data.estimate.headingSource == nexus::HeadingSource::driveEncoders ? "ENC" : "NONE";
    pros::lcd::print(1, "Heading %.2f deg [%s]", nexus::degrees(state.pose.theta), headingSource);
    if (imuCalibrationFailed)
        pros::lcd::print(2, "IMU CAL FAILED | Conf %.2f", data.estimate.confidence);
    else if (data.sensors.gyroValid)
        // Only the display wraps; estimation and logging keep continuous rotation.
        pros::lcd::print(2, "IMU %.2f deg | Conf %.2f", nexus::degrees(nexus::wrap(data.sensors.gyro)), data.estimate.confidence);
    else pros::lcd::print(2, "IMU unavailable | Conf %.2f", data.estimate.confidence);
    const auto padStatus = controller.is_connected();
    const char* pad = detail::controllerIsConnected(padStatus) ? "OK" : padStatus == 0 ? "OFF" : "ERROR";
    const char* mode = pros::competition::is_disabled() ? "DISABLED"
        : pros::competition::is_autonomous() ? "AUTO ENABLED" : "DRIVER ENABLED";
    pros::lcd::print(3, "%s | Pad:%s", mode, pad);
    pros::lcd::print(4, "NMPC %.1fms Max%.1f Slip%.2f", data.solver.computeMs, data.worstSolverMs, data.estimate.slip);
    pros::lcd::print(5, "Incertezza %.0fmm %.1fdeg", positionUncertainty / nexus::millimeter,
                     nexus::degrees(headingUncertainty));
    const auto mask = activeCalibrationMask.load();
    pros::lcd::print(6, "%s", mask == 255 ? "Auton profile: fitted" : mask
                     ? "Auton profile: partial" : "Auton profile: defaults (not tuned)");
    if (!pros::competition::is_autonomous()) {
        const int r2 = driverR2Input.load();
        pros::lcd::print(7, "R2:%s | %s", r2 == 1 ? "1" : r2 == 0 ? "0" : "?", macroNotice.load());
    } else pros::lcd::print(7, "Auton:%s | LCD frecce: IMU", nexus::statusName(lastAutonStatus.load()));
}

void diagnosticsLoop() {
    std::FILE* log = config::logTelemetry ? std::fopen("/usd/nexus-log.csv", "w") : nullptr;
    if (log) std::fprintf(log, "time,x_mm,y_mm,theta,v_mmps,omega,pxx_mm2,pyy_mm2,ptt,confidence,slip,health,left_v,right_v,forward_pod_mm,lateral_pod_mm,left_encoder_mm,right_encoder_mm,gyro,gyro_rate,solver_ms,cost,iterations,status,motor_time,left_terminal_v,right_terminal_v,left_motor_mmps,right_motor_mmps,accel_x_g,accel_y_g,accel_z_g,raw_gyro_x_dps,raw_gyro_y_dps,raw_gyro_z_dps,accel_valid,raw_gyro_valid,heading_source,gyro_bias,accel_peak_g,gyro_peak_dps,accel_near_limit,gyro_near_limit,forward_valid,lateral_valid,left_valid,right_valid,gyro_valid,stationary,gyro_bias_std,rejected_increments,rejected_increment_mask,last_rejected_increment_mask,estimator_ms,estimator_overruns,odom_forward_x_mm,odom_forward_y_mm,odom_lateral_x_mm,odom_lateral_y_mm,odom_offset_x_mm,odom_offset_y_mm\n");
    unsigned frames = 0;
    unsigned page = 0;
    unsigned previousLcdButtons = 0;
    for (;;) {
        const auto data = drive->diagnostics();
        const auto& state = data.estimate.state;
        const auto lcdButtons = pros::lcd::read_buttons();
        if (!detail::calibrationScreenActive()) {
            const auto pressedLcd = lcdButtons & ~previousLcdButtons;
            if (pressedLcd & LCD_BTN_RIGHT) page = (page + 1) % 4;
            else if (pressedLcd & LCD_BTN_LEFT) page = (page + 3) % 4;
            drawDiagnostics(data, page);
        }
        previousLcdButtons = lcdButtons;
        if (log) {
            const auto readSide = [](pros::MotorGroup& motors, std::size_t count) {
                double voltage = 0, speed = 0;
                std::size_t valid = 0;
                for (std::size_t i = 0; i < count; ++i) {
                    const auto measured = motors.get_voltage(static_cast<std::uint8_t>(i));
                    const double rpm = motors.get_actual_velocity(static_cast<std::uint8_t>(i));
                    if (measured != PROS_ERR && std::isfinite(rpm) && std::abs(rpm) <= 1000) {
                        voltage += measured * 0.001;
                        speed += rpm / 60 * nexus::pi * config::driveWheelDiameter * nexus::inch
                                 * config::wheelRPM / config::motorRPM;
                        ++valid;
                    }
                }
                const double invalid = std::numeric_limits<double>::quiet_NaN();
                return std::array<double, 2>{valid ? voltage / valid : invalid, valid ? speed / valid : invalid};
            };
            const double before = static_cast<double>(pros::micros()) * 1e-6;
            const auto left = readSide(leftMotors, config::leftPorts.size());
            const auto right = readSide(rightMotors, config::rightPorts.size());
            const double motorTime = (before + static_cast<double>(pros::micros()) * 1e-6) / 2;
            std::fprintf(log, "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.8f,%.8f,%.8f,%.4f,%.4f,%d,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.3f,%.6f,%u,%d,%.6f,%.4f,%.4f,%.6f,%.6f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%d,%d,%d,%.6f,%.4f,%.2f,%lu,%lu,%d,%d,%d,%d,%d,%d,%.6f,%lu,%u,%u,%.3f,%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                data.estimate.timestamp, state.pose.x / nexus::millimeter, state.pose.y / nexus::millimeter,
                state.pose.theta, state.v / nexus::millimeter, state.omega,
                data.estimate.covariance[0][0] / nexus::square(nexus::millimeter),
                data.estimate.covariance[1][1] / nexus::square(nexus::millimeter), data.estimate.covariance[2][2],
                data.estimate.confidence, data.estimate.slip, static_cast<int>(data.estimate.health),
                data.command.left, data.command.right, data.sensors.forward / nexus::millimeter,
                data.sensors.lateral / nexus::millimeter, data.sensors.left / nexus::millimeter,
                data.sensors.right / nexus::millimeter, data.sensors.gyro, data.sensors.gyroRate,
                data.solver.computeMs, data.solver.finalCost, data.solver.iterations, static_cast<int>(data.status),
                motorTime, left[0], right[0], left[1] / nexus::millimeter, right[1] / nexus::millimeter,
                data.sensors.accelerationG[0], data.sensors.accelerationG[1], data.sensors.accelerationG[2],
                data.sensors.rawGyroDps[0], data.sensors.rawGyroDps[1], data.sensors.rawGyroDps[2],
                data.sensors.accelerationValid, data.sensors.rawGyroValid, static_cast<int>(data.estimate.headingSource),
                data.estimate.gyroBias, data.peakAccelerationG, data.peakGyroRateDps,
                static_cast<unsigned long>(data.accelerationNearLimit), static_cast<unsigned long>(data.gyroNearLimit),
                data.sensors.forwardValid, data.sensors.lateralValid, data.sensors.leftValid,
                data.sensors.rightValid, data.sensors.gyroValid, data.estimate.stationary,
                data.estimate.gyroBiasStd, static_cast<unsigned long>(data.estimate.rejectedIncrements),
                static_cast<unsigned>(data.estimate.rejectedIncrementMask),
                static_cast<unsigned>(data.estimate.lastRejectedIncrementMask), data.estimatorMs,
                static_cast<unsigned long>(data.estimatorOverruns),
                data.estimate.odometry.forwardX / nexus::millimeter, data.estimate.odometry.forwardY / nexus::millimeter,
                data.estimate.odometry.lateralX / nexus::millimeter, data.estimate.odometry.lateralY / nexus::millimeter,
                data.estimate.odometry.offsetX / nexus::millimeter, data.estimate.odometry.offsetY / nexus::millimeter);
            if (++frames % 10 == 0) std::fflush(log);
        }
        pros::delay(50);
    }
}
}

bool initialize() {
    if (drive) return true;
    liftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    intakeMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    liftMotors.set_encoder_units_all(pros::E_MOTOR_ENCODER_ROTATIONS);
    liftMotors.tare_position_all();
    armEncoder.set_data_rate(10);
    if (!validPorts()) { pros::lcd::print(0, "CONFIG ERROR: duplicate/invalid ports"); return false; }
    // Mechanical movements finish before IMU reset, pod setup or odometry tasks.
    startupPreMatch();
    if (config::imuPort) {
        startupBiasStatus = "errore IMU";
        imu = std::make_unique<pros::Imu>(config::imuPort);
        pros::lcd::print(0, "Calibrating IMU: keep still");
        imuCalibrationFailed = imu->reset(true) != 1;
        if (imuCalibrationFailed)
            std::printf("IMU calibration failed: port %u, errno %d\n", static_cast<unsigned>(config::imuPort), errno);
    }
    if (config::forwardPodPort) forwardPod = std::make_unique<pros::Rotation>(config::forwardPodPort);
    if (config::lateralPodPort) lateralPod = std::make_unique<pros::Rotation>(config::lateralPodPort);
    nexus::Hardware hardware{leftMotors, rightMotors, imu.get(), forwardPod.get(), lateralPod.get()};
    for (std::size_t i = 0; i < ranges.size(); ++i) {
        if (config::wallSensors[i].port) {
            ranges[i] = std::make_unique<pros::Distance>(config::wallSensors[i].port);
            hardware.distances[i] = ranges[i].get();
        }
    }
    activeEstimatorConfig = localizationConfig();
    activeDynamicsConfig = dynamicsConfig();
    nexus::calibration::CalibrationProfile profile;
    const auto identity = fingerprint();
    bool recovered = false;
    const auto loaded = nexus::calibration::loadProfile(config::calibrationPath, identity, profile, &recovered);
    const bool calibrated = loaded == nexus::calibration::StorageStatus::ok
        && nexus::calibration::applyProfile(profile, activeEstimatorConfig, activeDynamicsConfig);
    activeProfile.hardwareFingerprint = identity;
    if (calibrated) activeProfile = profile;
    // Cache effective defaults too: later partial fits preserve the current configuration.
    activeProfile.forwardScale = activeEstimatorConfig.forwardScale;
    activeProfile.lateralScale = activeEstimatorConfig.lateralScale;
    activeProfile.gyroScale = activeEstimatorConfig.gyroScale;
    activeProfile.forwardOffset = activeEstimatorConfig.forwardOffset;
    activeProfile.lateralOffset = activeEstimatorConfig.lateralOffset;
    activeProfile.trackWidth = activeEstimatorConfig.trackWidth;
    activeProfile.kSLeft = activeDynamicsConfig.kSLeft; activeProfile.kSRight = activeDynamicsConfig.kSRight;
    activeProfile.kVLeft = activeDynamicsConfig.kVLeft; activeProfile.kVRight = activeDynamicsConfig.kVRight;
    activeProfile.kALeft = activeDynamicsConfig.kALeft; activeProfile.kARight = activeDynamicsConfig.kARight;
    activeCalibrationMask = activeProfile.calibratedMask;
    std::printf("NEXUS hardware fingerprint: %llu; calibrated mask: %lu; backup: %d\n",
        static_cast<unsigned long long>(identity), static_cast<unsigned long>(calibrated ? profile.calibratedMask : 0), recovered);
    drive = std::make_unique<nexus::Chassis>(hardware, hardwareConfig(), activeEstimatorConfig, activeDynamicsConfig);
    drive->begin();
    pros::delay(50);
    if (imu && !imuCalibrationFailed) {
        // Heap storage: the collector is larger than a PROS competition callback stack.
        auto stationary = std::make_unique<nexus::calibration::StationaryCalibration>();
        stationary->reset(activeEstimatorConfig.gyroScale);
        pros::lcd::print(0, "IMU bias: keep still for 3 seconds");
        const auto started = pros::millis();
        double previousSample = -1;
        bool acceptedSamples = true;
        while (pros::millis() - started < 3000) {
            const auto sample = drive->diagnostics().sensors;
            const double now = static_cast<double>(pros::micros()) * 1e-6;
            if (!std::isfinite(sample.timestamp) || sample.timestamp <= 0
                || now < sample.timestamp || now - sample.timestamp > 0.10) {
                acceptedSamples = false; break;
            }
            if (sample.timestamp != previousSample) {
                previousSample = sample.timestamp;
                if (!stationary->add(sample)) { acceptedSamples = false; break; }
            }
            pros::delay(10);
        }
        const auto fit = stationary->finish();
        const auto latest = drive->diagnostics().sensors;
        const double checkedAt = static_cast<double>(pros::micros()) * 1e-6;
        const bool fresh = latest.gyroValid && latest.leftValid && latest.rightValid
            && std::isfinite(latest.timestamp) && checkedAt >= latest.timestamp
            && checkedAt - latest.timestamp <= 0.10;
        const bool applied = acceptedSamples && fresh && fit.quality.accepted
            && drive->setGyroBias(fit.gyroBias, fit.gyroBiasVariance);
        startupBiasStatus = applied ? "applicato" : !fit.quality.accepted
            ? nexus::calibration::fitFailureName(fit.quality.failure)
            : !acceptedSamples ? "dati interrotti" : !fresh ? "dati non recenti" : "applicazione rifiutata";
        std::printf("Stationary startup bias: %s, %.4f deg/s (%s)\n", applied ? "applied" : "skipped",
                    nexus::degrees(fit.gyroBias), startupBiasStatus);
    }
    displayTask = std::make_unique<pros::Task>(diagnosticsLoop, TASK_PRIORITY_DEFAULT - 1, 8192, "nexus-diagnostics");
    detail::startCalibrationService();
    return true;
}
bool ready() { return static_cast<bool>(drive); }
nexus::Chassis& chassis() { return *drive; }
void intake(bool enabled) {
    intakeEnabled = enabled && !pros::competition::is_disabled() && !calibrationOwnsDriver();
    if (intakeEnabled) intakeMotors.move_voltage(12000); else intakeMotors.brake();
}
void pinza(bool closed) {
    if (calibrationOwnsDriver()) return;
    clampClosed = closed;
    clamp.set_value(closed ? config::clampClosedLevel : !config::clampClosedLevel);
}
void lift(int power) {
    if (power != 0 && !pros::competition::is_disabled() && !calibrationOwnsDriver())
        liftMotors.move(std::clamp(power, -127, 127) * config::mechanism::liftDirection);
    else liftMotors.brake();
}
void manualLift(int power) {
    if (power) mechanisms.cancelDriverLiftTarget();
    lift(power);
}
void braccio(int power) {
    if (power != 0 && !pros::competition::is_disabled() && !calibrationOwnsDriver()) {
        mechanisms.output.armCoast = false;
        arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        arm.move(std::clamp(power, -127, 127));
    }
    else arm.brake();
}
void stop() {
    mechanisms.suspend();
    detail::abortCalibration();
    if (drive) drive->cancel();
    else { leftMotors.brake(); rightMotors.brake(); }
    lift(0); applyMacroArm(); intake(false);
}
void resetDriver() {
    stop();
    mechanisms.resume(pros::millis());
    buttonState.reset(readButtons());
    macroNotice = mechanism::driverLabel(mechanisms.state, mechanisms.calibrated);
}
void updateController() {
    buttonState.update(readButtons());
    std::uint32_t mask = 0;
    for (std::size_t i = 0; i < buttonCount; ++i)
        if (buttonState.held(i)) mask |= 1u << i;
    detail::publishCalibrationInput(mask, controllerConnected);
}
bool held(Button button) { return buttonState.held(buttonIndex(button)); }
bool pressed(Button button) { return buttonState.pressed(buttonIndex(button)); }
int buttons(Button forward, Button reverse) {
    return buttonState.power(buttonIndex(forward), buttonIndex(reverse));
}
void arcade(Stick forward, Stick turn, nexus::ArcadeCurves curves) {
    if (!drive || calibrationOwnsDriver()) return;
    const auto readStick = [](Stick stick) {
        if (!controllerConnected) return 0;
        const int value = controller.get_analog(stick);
        return value >= -127 && value <= 127 ? value : 0;
    };
    drive->arcade(readStick(forward), readStick(turn), curves);
}
void toggleIntake() { intake(!intakeEnabled.load()); }
void togglePinza() { pinza(!clampClosed.load()); }

namespace {
bool finishMacroUpdate(bool owns) {
    // Controller LCD writes can be dropped when sent too quickly. Retry the
    // current status periodically, separating status/detail lines in time.
    static std::uint32_t lastWrite = 0;
    static unsigned faultLine = 0;
    const auto now = pros::millis();
    if (controllerConnected && !calibrationOwnsDriver() && now - lastWrite >= 100) {
        lastWrite = now;
        const bool fault = mechanisms.state == mechanism::State::fault;
        const unsigned line = faultLine++ % 3;
        const char* detail = fault ? mechanisms.error : mechanisms.notice;
        const char* message = line == 0 ? (fault ? "PreMatch: SU riprova" : macroNotice.load()) : detail;
        if (line == 2) message = std::strlen(detail) > 19 ? detail + 19 : "";
        controller.print(line, 0, "%-19.19s", message);
    }
    return owns;
}
void applyMacroArm() {
    const bool allowed = !calibrationOwnsDriver() && (!pros::competition::is_disabled()
        || (!drive && !pros::competition::is_connected()));
    mechanism::applyArmMotor(arm, mechanisms.output, mechanisms.state, allowed);
}
bool configureArmEncoder() {
    // Global constructors may run before the Rotation sensor is available.
    // Reapply and verify the requested sign immediately before each homing run.
    const bool reversed = config::mechanism::encoderPort < 0;
    return armEncoder.set_reversed(reversed) == 1
        && armEncoder.set_data_rate(10) == 1
        && armEncoder.get_reversed() == static_cast<int>(reversed);
}
mechanism::Input mechanismInput() {
    auto in = mechanism::readMechanismInput(armEncoder, arm, liftMotors, pros::millis(), config::liftPorts.size());
    mechanism::readForwardPodInput(in, forwardPod.get());
    return in;
}

// Owns the display only during initialize(); callbacks just post an atomic event.
// Delete the overlay before starting the existing LCD diagnostics/calibration UI.
class StartupScreen {
public:
    StartupScreen() {
        lv_lock();
        panel = lv_obj_create(lv_screen_active());
        lv_obj_set_size(panel, 480, 240);
        lv_obj_center(panel);
        lv_obj_remove_flag(panel, LV_OBJ_FLAG_SCROLLABLE);
        title = lv_label_create(panel);
        lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 12);
        label = lv_label_create(panel);
        lv_obj_set_width(label, 430);
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 50);
        button = lv_button_create(panel);
        lv_obj_set_size(button, 200, 60);
        lv_obj_align(button, LV_ALIGN_BOTTOM_MID, 0, -8);
        buttonLabel = lv_label_create(button);
        lv_obj_center(buttonLabel);
        lv_obj_add_flag(button, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_event_cb(button, [](lv_event_t* event) {
            auto* self = static_cast<StartupScreen*>(lv_event_get_user_data(event));
            self->clicked.store(true);
        }, LV_EVENT_CLICKED, this);
        lv_unlock();
    }
    ~StartupScreen() { lv_lock(); lv_obj_delete(panel); lv_unlock(); }
    bool pressed() { return clicked.exchange(false); }
    void show(const char* heading, const char* message, const char* buttonText = nullptr) {
        lv_lock();
        clicked = false;
        lv_label_set_text(title, heading);
        lv_label_set_text(label, message);
        if (buttonText) {
            lv_label_set_text(buttonLabel, buttonText);
            lv_obj_remove_flag(button, LV_OBJ_FLAG_HIDDEN);
        } else lv_obj_add_flag(button, LV_OBJ_FLAG_HIDDEN);
        lv_unlock();
    }
private:
    lv_obj_t *panel, *title, *label, *button, *buttonLabel;
    std::atomic<bool> clicked{false};
};

void startupPreMatch() {
    StartupScreen screen;
    startup::Gate gate(pros::millis());
    auto previousPhase = startup::Phase::complete;
    auto previousMechanism = mechanism::State::off;
    bool previousAllowed = false;
    std::uint32_t lastMechanismDisplay = 0;
    mechanism::Input lastMechanismInput;
    int lastArmCommand = 0;
    bool failedDuringMotion = false;
    // During initialize, local practice can run before opcontrol. Never override
    // a disabled signal from a connected field controller/competition switch.
    while (gate.phase != startup::Phase::complete) {
        const bool allowed = !pros::competition::is_connected() || !pros::competition::is_disabled();
        const bool pressedScreen = screen.pressed();
        const bool abort = gate.phase == startup::Phase::preMatch
            && detail::controllerIsConnected(controller.is_connected()) && controller.get_digital(R2) == 1;
        const auto action = gate.tick(pros::millis(), allowed,
            mechanisms.state == mechanism::State::prepared,
            mechanisms.state == mechanism::State::fault || abort, pressedScreen, pressedScreen);
        if (action.stopPreMatch) {
            // Passive brake releases the active HOLD effort after a jam/abort.
            arm.set_brake_mode(mechanisms.output.armCoast ? pros::E_MOTOR_BRAKE_COAST : pros::E_MOTOR_BRAKE_BRAKE);
            mechanisms.cancel(); liftMotors.brake(); arm.brake(); intakeMotors.brake();
        }
        if (action.startPreMatch) {
            arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            mechanisms.output.closed = clampClosed.load();
            const bool encoderConfigured = configureArmEncoder();
            auto initial = mechanismInput();
            if (!encoderConfigured && initial.valid) initial.problem = "Rotation: setup non riuscito";
            initial.valid = initial.valid && encoderConfigured;
            mechanisms.start(initial);
            if (!initial.valid) mechanisms.tick(initial);
        }
        if (gate.phase == startup::Phase::preMatch) {
            const auto in = mechanismInput();
            const auto phaseBeforeTick = mechanisms.state;
            lastMechanismInput = in;
            lastArmCommand = mechanisms.output.arm;
            const bool wasProfile = mechanisms.output.armProfileActive;
            mechanisms.tick(in);
            if (mechanisms.state == mechanism::State::fault) failedDuringMotion = wasProfile;
            const auto& output = mechanisms.output;
            applyMacroArm();
            if (output.lift) liftMotors.move(output.lift * config::mechanism::liftDirection); else liftMotors.brake();
            intakeMotors.brake(); intakeEnabled = false;
            pinza(output.closed);
            if (in.now - lastMechanismDisplay >= 200 || phaseBeforeTick != mechanisms.state) {
                lastMechanismDisplay = in.now;
                char message[160];
                std::snprintf(message, sizeof(message), "%s\n%s %.1f gradi | %.0f mA | %.1f rpm\nR2: interrompi",
                    mechanism::phaseLabel(mechanisms.state), mechanisms.encoderDirectionKnown ? "Da zero:" : "Grezzi:",
                    mechanisms.encoderDirectionKnown ? mechanisms.armPosition(in.arm) : in.arm,
                    in.armCurrent, in.armVelocity);
                screen.show("PREMATCH", message);
                std::printf("PreMatch %s -> %s: raw arm %.2f deg, %.0f mA, %.1f rpm, power %d; encoder sign %d (known %d); %s\n",
                    mechanism::phaseLabel(phaseBeforeTick), mechanism::phaseLabel(mechanisms.state),
                    in.arm, in.armCurrent, in.armVelocity, output.arm,
                    mechanisms.encoderSign, mechanisms.encoderDirectionKnown, mechanisms.error);
            }
        }
        if (gate.phase != previousPhase || mechanisms.state != previousMechanism || allowed != previousAllowed) {
            switch (gate.phase) {
            case startup::Phase::delayPreMatch:
                screen.show("AVVIO", allowed ? "PreMatch automatico fra 1 secondo"
                    : "Robot disabilitato dal campo.\nIn attesa di abilitazione per il preMatch.");
                break;
            case startup::Phase::preMatch: {
                break;
            }
            case startup::Phase::waitReady:
                screen.show("PREMATCH COMPLETATO", "Posiziona il robot e premi READY.\nPoi tienilo fermo durante la calibrazione.", "READY");
                break;
            case startup::Phase::delayNavigation:
                screen.show("READY", "Calibrazione gyro e odometria fra 1 secondo.\nTieni fermo il robot.");
                break;
            case startup::Phase::fault: {
                char message[220];
                const char* reason = mechanisms.error[0] ? mechanisms.error : "Interrotto con R2";
                if (failedDuringMotion) {
                    std::snprintf(message, sizeof(message), "%s\n%.1f -> %.1f gradi | %.0f mA\nUltimo comando: %d/127",
                        reason, mechanisms.armPosition(lastMechanismInput.arm), mechanisms.armTargetDegrees,
                        lastMechanismInput.armCurrent, lastArmCommand);
                } else std::snprintf(message, sizeof(message), "%s\nLettura %.1f gradi | %.0f mA",
                    reason, lastMechanismInput.arm, lastMechanismInput.armCurrent);
                screen.show("PREMATCH INTERROTTO", message, "RIPROVA");
                break;
            }
            case startup::Phase::complete: break;
            }
            previousPhase = gate.phase; previousMechanism = mechanisms.state; previousAllowed = allowed;
        }
        pros::delay(10);
    }
    liftMotors.brake(); arm.brake(); intakeMotors.brake();
    std::printf("Startup preMatch completed; READY + 1s: initializing navigation\n");
    std::printf("Arm stops: %.2f %.2f / %.2f %.2f deg; lift top %.3f %.3f rot\n",
        mechanisms.zeroMeasurements[0], mechanisms.zeroMeasurements[1],
        mechanisms.topMeasurements[0], mechanisms.topMeasurements[1],
        mechanisms.liftMeasurements[0], mechanisms.liftMeasurements[1]);
}
}
void preMatch() {
    if (!ready() || pros::competition::is_disabled() || calibrationOwnsDriver()) return;
    intake(false); lift(0); braccio(0);
    arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    mechanisms.output.closed = clampClosed.load();
    const bool encoderConfigured = configureArmEncoder();
    auto initial = mechanismInput();
    if (!encoderConfigured && initial.valid) initial.problem = "Rotation: setup non riuscito";
    initial.valid = initial.valid && encoderConfigured;
    mechanisms.start(initial);
    if (!initial.valid) mechanisms.tick(initial);
    macroNotice = mechanism::driverLabel(mechanisms.state, mechanisms.calibrated);
}
bool updateMacros() {
    static bool wasGated = false;
    static bool clampManualOverride = false;
    if (pros::competition::is_disabled() || calibrationOwnsDriver() || !controllerConnected) {
        wasGated = true;
        const bool owned = mechanisms.owns();
        if (pros::competition::is_disabled()) mechanisms.suspend();
        else mechanisms.cancel();
        lift(0); applyMacroArm(); intake(false);
        macroNotice = pros::competition::is_disabled() ? "Robot disabilitato"
            : calibrationOwnsDriver() ? "Calibrazione telaio"
            : !controllerConnected ? "Pad assente: macro ferma"
            : mechanism::driverLabel(mechanisms.state, mechanisms.calibrated);
        return finishMacroUpdate(owned || !controllerConnected);
    }
    if (wasGated) {
        wasGated = false;
        macroNotice = mechanism::driverLabel(mechanisms.state, mechanisms.calibrated);
    }
    const bool clampPressed = pressed(L1);
    if (clampPressed) {
        togglePinza();
        clampManualOverride = true;
    }
    const bool grabPressed = pressed(R1), releasePressed = pressed(R2);
    if ((grabPressed || releasePressed) && !clampPressed) clampManualOverride = false;
    const int liftStep = static_cast<int>(pressed(B)) - static_cast<int>(pressed(DOWN));
    const auto in = mechanismInput();
    const auto before = mechanisms.state;
    const int previousArmCommand = mechanisms.output.arm;
    const auto result = mechanism::driverTick(mechanisms, in, grabPressed, releasePressed,
        clampClosed.load(), liftStep);
    if (clampManualOverride) mechanisms.output.closed = clampClosed.load();
    if (releasePressed) {
        macroNotice = result.releaseAccepted ? "R2: rilascio richiesto"
            : mechanisms.state == mechanism::State::fault ? "R2 bloccato: errore macro"
            : !mechanisms.calibrated ? "R2: serve prematch SU" : "R2: sequenza gia attiva";
        std::printf("R2 read: state %d -> %d, calibrated %d, accepted %d, arm %.1f, target %.1f, command %d; %s\n",
            static_cast<int>(before), static_cast<int>(mechanisms.state),
            mechanisms.calibrated, result.releaseAccepted, mechanisms.armPosition(in.arm),
            mechanisms.armTargetDegrees, mechanisms.output.arm, mechanisms.error);
    }
    lift(mechanisms.output.lift);
    if (!result.owns) return finishMacroUpdate(false);
    applyMacroArm();
    intakeEnabled = mechanisms.output.intake != 0;
    if (mechanisms.output.intake) intakeMotors.move(mechanisms.output.intake);
    else intakeMotors.brake();
    pinza(mechanisms.output.closed);
    if (before != mechanisms.state) {
        macroNotice = mechanism::driverLabel(mechanisms.state, mechanisms.calibrated);
        std::printf("Mechanism state %d: %s\n", static_cast<int>(mechanisms.state), mechanisms.error);
        if (mechanisms.state == mechanism::State::fault) {
            std::printf("Mechanism fault from state %d: arm %.2f -> %.2f deg, previous command %d/127, %.0f mA, %.1f rpm\n",
                static_cast<int>(before), mechanisms.armPosition(in.arm), mechanisms.armTargetDegrees,
                previousArmCommand, in.armCurrent, in.armVelocity);
            macroNotice = mechanisms.error;
            controller.rumble("---");
        } else if (mechanisms.state == mechanism::State::stopped) {
            std::printf("Mechanism paused: %s; R1/R2 can start a new request\n", mechanisms.notice);
        } else if (mechanisms.state == mechanism::State::prepared) {
            std::printf("Arm stops: %.2f %.2f / %.2f %.2f deg; lift top %.3f %.3f rot\n",
                mechanisms.zeroMeasurements[0], mechanisms.zeroMeasurements[1],
                mechanisms.topMeasurements[0], mechanisms.topMeasurements[1],
                mechanisms.liftMeasurements[0], mechanisms.liftMeasurements[1]);
        }
    }
    if (mechanisms.releasePending() && mechanisms.state == mechanism::State::pickupLift)
        macroNotice = "R2: attendo lift";
    else if (mechanisms.state == mechanism::State::pickupLift)
        macroNotice = mechanisms.endstopSelected ? "Lift -> finecorsa" : "Lift -> 150";
    if (mechanisms.state == mechanism::State::fault) macroNotice = mechanisms.error;
    else if (mechanisms.notice[0]) macroNotice = mechanisms.notice;
    else if (mechanisms.state != mechanism::State::pickupLift)
        macroNotice = mechanism::driverLabel(mechanisms.state, mechanisms.calibrated);
    return finishMacroUpdate(true);
}

namespace detail {
CalibrationContext calibrationContext() {
    // Written only at initialization and by the persistent calibration service.
    return {activeEstimatorConfig, activeProfile, imuCalibrationFailed};
}
CalibrationApplyResult applyCalibrationProfile(const nexus::calibration::CalibrationProfile& profile, bool persist) {
    using namespace nexus::calibration;
    CalibrationApplyResult result;
    if (!ready() || pros::competition::is_disabled() || pros::competition::is_autonomous()
        || pros::competition::is_connected() || drive->busy()) return result;
    if (profile.hardwareFingerprint != activeProfile.hardwareFingerprint || !validProfile(profile)) return result;
    auto estimatorConfig = activeEstimatorConfig;
    auto modelConfig = activeDynamicsConfig;
    if (!applyProfile(profile, estimatorConfig, modelConfig)
        || !drive->applyConfiguration(estimatorConfig, modelConfig)) return result;
    activeEstimatorConfig = estimatorConfig;
    activeDynamicsConfig = modelConfig;
    activeProfile = profile;
    activeCalibrationMask = profile.calibratedMask;
    result.applied = true;
    if (persist) result.storage = saveProfile(config::calibrationPath, profile);
    return result;
}
} // namespace detail

Auton::Auton() {
    // Stop first: Sequence must capture the cancellation epoch AFTER this cancel.
    robot::stop();
    if (robot::ready()) sequence_.emplace(robot::chassis());
    lastAutonStatus = sequence_ ? nexus::MotionStatus::running : nexus::MotionStatus::invalidRequest;
}
Auton::~Auton() {
    const auto outcome = result();
    robot::stop();
    lastAutonStatus = outcome.status;
}
void Auton::setPose(double x, double y, double heading) {
    if (sequence_) sequence_->setPose(x, y, heading);
}
void Auton::moveToPoint(double x, double y, nexus::MoveOptions options) {
    if (sequence_) sequence_->moveToPoint(x, y, options);
}
void Auton::moveToPose(double x, double y, double heading, nexus::MoveOptions options) {
    if (sequence_) sequence_->moveToPose(x, y, heading, options);
}
void Auton::turnToHeading(double heading, nexus::MoveOptions options) {
    if (sequence_) sequence_->turnToHeading(heading, options);
}
void Auton::wait(std::uint32_t milliseconds) {
    if (sequence_) sequence_->wait(milliseconds);
}
void Auton::intake(bool enabled) {
    if (sequence_) sequence_->action([enabled] { robot::intake(enabled); });
}
void Auton::pinza(bool closed) {
    if (sequence_) sequence_->action([closed] { robot::pinza(closed); });
}
void Auton::lift(int power) {
    if (sequence_) sequence_->action([power] { if (power) mechanisms.cancel(); robot::lift(power); });
}
void Auton::braccio(int power) {
    if (sequence_) sequence_->action([power] { if (power) mechanisms.cancel(); robot::braccio(power); });
}
nexus::MotionResult Auton::result() const {
    return sequence_ ? sequence_->result() : nexus::MotionResult{nexus::MotionStatus::invalidRequest};
}
} // namespace robot
