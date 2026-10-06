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

// Signature verified in the official VEX V5 SDK 20240802 v5_api.h. PROS's
// linked libpros.a exports this vendor entry point, but its public headers do not.
// This is the CPU0 packet-reception clock, NOT the physical measurement time.
extern "C" std::int32_t vexDeviceGetTimestampByIndex(std::int32_t index);

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
std::unique_ptr<pros::Task> odometryUsbTask;
struct CharacterizationRecord { std::array<double, 36> fields{}; };
pros::c::queue_t characterizationQueue = nullptr;
std::atomic<bool> characterizationTelemetry{false};
std::uint32_t characterizationSequence = 0, characterizationDropped = 0; // Calibration service only.
constexpr const char* characterizationHeader = "NXCHAR_HEADER,version,sequence,run,trial,stage,failure,t_s,sensor_s,cmd_left_v,cmd_right_v,applied_left_v,applied_right_v,battery_v,x_m,y_m,heading_rad,v_mps,lateral_mps,omega_radps,forward_m,lateral_m,gyro_rad,left_wheel_mps,right_wheel_mps,accel_x_g,accel_y_g,accel_z_g,valid_mask,dropped,imu_scale,forward_scale,lateral_scale,forward_offset_m,lateral_offset_m,track_width_m,motor_s";
std::unique_ptr<pros::Task> startupTask;
startup::Initialization initialization;
std::atomic<bool> clampClosed{false}, intakeEnabled{false};
std::atomic<nexus::MotionStatus> lastAutonStatus{nexus::MotionStatus::idle};
constexpr int firstButton = pros::E_CONTROLLER_DIGITAL_L1;
constexpr std::size_t buttonCount = pros::E_CONTROLLER_DIGITAL_A - firstButton + 1;
detail::ButtonState<buttonCount> buttonState;
bool controllerConnected = false;
std::atomic<int> driverR2Input{0};
std::atomic<const char*> macroNotice{"PreMatch all'avvio"};
bool imuCalibrationFailed = false; // Set before diagnostics task starts.
const char* imuCalibrationStatus = "assente"; // Immutable after diagnostics task starts.
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

std::uint32_t odometryPacketClock(std::size_t source) {
    const std::array<int, 3> ports{config::forwardPodPort, config::lateralPodPort, config::imuPort};
    if (source >= ports.size() || ports[source] == 0) return 0;
    const int port = ports[source] < 0 ? -ports[source] : ports[source];
    if (port > 21) return 0;
    return static_cast<std::uint32_t>(vexDeviceGetTimestampByIndex(port - 1));
}

nexus::HardwareConfig hardwareConfig() {
    using namespace config;
    nexus::HardwareConfig cfg;
    cfg.driveWheelDiameter = driveWheelDiameter;
    cfg.driveWheelPerMotor = wheelRPM / motorRPM;
    cfg.forwardWheelDiameter = forwardWheelDiameter;
    cfg.lateralWheelDiameter = lateralWheelDiameter;
    cfg.leftMotorCount = leftPorts.size(); cfg.rightMotorCount = rightPorts.size();
    cfg.leftOdometryMask = useDriveEncodersForOdometry ? leftOdometryMask : 0;
    cfg.rightOdometryMask = useDriveEncodersForOdometry ? rightOdometryMask : 0;
    cfg.forwardPodReversed = forwardPodPort < 0;
    cfg.lateralPodReversed = lateralPodPort < 0;
    cfg.driveGearset = pros::MotorGearset::blue;
    cfg.captureOdometryTrace = config::logOdometryUsb;
    cfg.tracePacketClock = config::logOdometryUsb ? odometryPacketClock : nullptr;
    cfg.requireLateralPodForMotion = config::lateralPodPort != 0;
    cfg.odometryPeriodMs = config::odometryPeriodMs;
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
    // get_rotation() is already produced by the IMU's native calibration.
    // Do not apply a second bias estimate or substitute wheel yaw at rest.
    cfg.nativeImuHeading = true;
    cfg.trackWidth = trackWidth * nexus::millimeter;
    cfg.forwardOffset = forwardPodX * nexus::millimeter;
    cfg.lateralOffset = lateralPodY * nexus::millimeter;
    // Native IMU heading still multiplies every increment by gyroScale.
    cfg.gyroScale = imuScale;
    // V5 shared sensor values refresh at 10 ms: repeated cached counters may
    // carry two acquisition intervals of real travel in their next increment.
    cfg.maxSensorHold = 0.010;
    // Conservative initial quiet band in wheel travel, not a specification of
    // hardware resolution. Raw translation remains integrated inside this band.
    cfg.stationaryEncoderSpan = 0.010 * nexus::millimeter;
    cfg.stationaryMaxInterval = std::max(0.05, 1.5 * config::odometryPeriodMs * 0.001);
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
    add(config::leftOdometryMask); add(config::rightOdometryMask);
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
    if (page == 5) {
        const auto& timing = data.timing;
        pros::lcd::print(0, "TEMPI 5 | pause >%.0fms: %lu", timing.longIntervalLimitMs,
                         static_cast<unsigned long>(timing.longIntervals));
        pros::lcd::print(1, "Ciclo %.1f max %.1f ms", timing.intervalMs, timing.worstIntervalMs);
        pros::lcd::print(2, "Ultima a %.1fs: %.1fms", timing.lastAt, timing.lastIntervalMs);
        pros::lcd::print(3, "Stesso intervallo RTOS: %lums", static_cast<unsigned long>(timing.lastRtosMs));
        pros::lcd::print(4, "Lavoro %.1f | uscita %.1fms", timing.lastWorkMs, timing.lastPublishMs);
        pros::lcd::print(5, "Attesa/esecuzione: %.1fms", timing.lastWaitMs);
        pros::lcd::print(6, "Lettura clock: %.3fms", timing.lastClockReadMs);
        pros::lcd::print(7, "Ultima pausa salvata | ciclo %lums", static_cast<unsigned long>(timing.periodMs));
        return;
    }
    if (page == 4) {
        const auto& quiet = data.estimate.quiet;
        const unsigned mask = quiet.lastBreakMask;
        const char* reason = mask & 256 ? "incremento scartato"
            : mask & 64 ? "IMU/ruote non utilizzabili"
            : mask & 32 ? "intervallo troppo lungo"
            : mask & 16 ? "variazione IMU"
            : mask & 128 ? "cambio sensori validi"
            : mask & 15 ? "escursione encoder" : "nessuna";
        pros::lcd::print(0, "QUIETE | fermo %s", data.estimate.stationary ? "SI" : "NO");
        pros::lcd::print(1, "Ultima causa: %s", reason);
        pros::lcd::print(2, "Span F %.3f L %.3f mm", quiet.wheelSpans[0] / nexus::millimeter,
                         quiet.wheelSpans[1] / nexus::millimeter);
        pros::lcd::print(3, "Span S %.3f D %.3f mm", quiet.wheelSpans[2] / nexus::millimeter,
                         quiet.wheelSpans[3] / nexus::millimeter);
        pros::lcd::print(4, "Soglia %.3fmm | quiete %.0fms",
                         quiet.encoderSpan / nexus::millimeter, quiet.dwell * 1000);
        pros::lcd::print(5, "IMU %.2fdeg/s | dt %.1fms", nexus::degrees(quiet.gyroRate),
                         quiet.sampleInterval * 1000);
        pros::lcd::print(6, "Riavvii %lu | mask %03X", static_cast<unsigned long>(quiet.breaks), mask);
        pros::lcd::print(7, "Ultimo stop: F/L pod S/D ruote");
        return;
    }
    if (page == 3) {
        const auto& odom = data.estimate.odometry;
        pros::lcd::print(0, "ODOM 7 | IMU nativa | X/Y mm");
        pros::lcd::print(1, "Avanz. X %.1f Y %.1f", odom.forwardX / nexus::millimeter,
                         odom.forwardY / nexus::millimeter);
        pros::lcd::print(2, "Later. X %.1f Y %.1f", odom.lateralX / nexus::millimeter,
                         odom.lateralY / nexus::millimeter);
        pros::lcd::print(3, "Offset X %.1f Y %.1f", odom.offsetX / nexus::millimeter,
                         odom.offsetY / nexus::millimeter);
        pros::lcd::print(4, "Totale X %.1f Y %.1f",
                         (odom.forwardX + odom.lateralX + odom.offsetX) / nexus::millimeter,
                         (odom.forwardY + odom.lateralY + odom.offsetY) / nexus::millimeter);
        pros::lcd::print(5, "Angolo min %.2f max %.2f deg", nexus::degrees(odom.minimumHeadingChange),
                         nexus::degrees(odom.maximumHeadingChange));
        const auto& unusable = data.estimate.unusableIncrements;
        pros::lcd::print(6, "Non usati F:%lu L:%lu G:%lu", static_cast<unsigned long>(unusable[0]),
                         static_cast<unsigned long>(unusable[1]), static_cast<unsigned long>(unusable[2]));
        pros::lcd::print(7, "Da reset | USB persi %lu", static_cast<unsigned long>(drive->droppedOdometryTrace()));
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
        if (!config::useDriveEncodersForOdometry)
            pros::lcd::print(3, "Encoder motori: ESCLUSI");
        else pros::lcd::print(3, "Ruote S %.0f%s D %.0f%s mm", sensors.left / nexus::millimeter,
                         sensors.leftValid ? "" : "?", sensors.right / nexus::millimeter,
                         sensors.rightValid ? "" : "?");
        pros::lcd::print(4, "Vel. avanti %.0f laterale %.0f mm/s", state.v / nexus::millimeter,
                         state.lateralV / nexus::millimeter);
        const unsigned mask = estimate.lastRejectedIncrementMask;
        pros::lcd::print(5, "Scarti %lu | ultimo %c%c%c%c%c",
                         static_cast<unsigned long>(estimate.rejectedIncrements),
                         mask & 1 ? 'F' : '-', mask & 2 ? 'L' : '-', mask & 4 ? 'S' : '-',
                         mask & 8 ? 'D' : '-', mask & 16 ? 'G' : '-');
        pros::lcd::print(6, "Stima %.1fms max %.1fms", data.estimatorMs, data.worstEstimatorMs);
        pros::lcd::print(7, "Cal. IMU nativa: %s", imuCalibrationStatus);
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
        pros::lcd::print(6, "Correzione bias locale: OFF");
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
    pros::lcd::print(4, "CTRL 9 %.1fms Max%.1f Slip%.2f", data.solver.computeMs, data.worstSolverMs, data.estimate.slip);
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

// Only the selected 600 RPM motors share this conversion. In particular the
// 200 RPM centre motors must never enter a fit with the 600 RPM wheel ratio.
std::array<double, 2> measuredDriveSide(pros::MotorGroup& motors, std::size_t count, std::uint8_t mask) {
    double voltage = 0, speed = 0;
    unsigned selected = 0, valid = 0;
    for (std::size_t i = 0; i < count && i < 8; ++i) {
        if (!(mask & (1u << i))) continue;
        ++selected;
        const auto measured = motors.get_voltage(static_cast<std::uint8_t>(i));
        const double rpm = motors.get_actual_velocity(static_cast<std::uint8_t>(i));
        if (measured != PROS_ERR && std::abs(measured) <= 14000 && std::isfinite(rpm) && std::abs(rpm) <= 1000) {
            voltage += measured * .001;
            speed += rpm / 60 * nexus::pi * config::driveWheelDiameter * nexus::inch * config::wheelRPM / config::motorRPM;
            ++valid;
        }
    }
    const double invalid = std::numeric_limits<double>::quiet_NaN();
    return selected && valid == selected ? std::array<double, 2>{voltage / valid, speed / valid}
                                         : std::array<double, 2>{invalid, invalid};
}

void odometryUsbLoop(const nexus::EstimatorConfig& startupConfig) {
    // The independent consumer can block on USB without holding any chassis or
    // sensor mutex. A full bounded queue drops trace frames, never sensor input.
    const bool odometryEnabled = config::logOdometryUsb && drive->startOdometryTrace();
    if (config::logOdometryUsb && !odometryEnabled) {
        std::printf("NXOD_ERROR,trace unavailable\n");
    }
    unsigned lines = 0;
    double lastCharacterizationRun = -1;
    std::uint32_t lastControlTrace = 0;
    for (;;) {
        CharacterizationRecord record;
        for (unsigned batch = 0; characterizationQueue && batch < 16 &&
             pros::c::queue_recv(characterizationQueue, &record, 0); ++batch) {
            // A complete line is emitted in one stdio operation, only here.
            if (record.fields[2] != lastCharacterizationRun ||
                static_cast<unsigned>(record.fields[1]) % 50 == 1 || record.fields[4] >= 4)
                std::puts(characterizationHeader);
            lastCharacterizationRun = record.fields[2];
            std::array<char, 1536> line{};
            std::size_t used = static_cast<std::size_t>(std::snprintf(line.data(), line.size(), "NXCHAR"));
            for (std::size_t i = 0; i < record.fields.size(); ++i) {
                const bool integer = i < 6 || i == 27 || i == 28;
                const int written = integer
                    ? std::snprintf(line.data() + used, line.size() - used, ",%.0f", record.fields[i])
                    : std::snprintf(line.data() + used, line.size() - used, ",%.9f", record.fields[i]);
                if (written < 0 || static_cast<std::size_t>(written) >= line.size() - used) { used = 0; break; }
                used += static_cast<std::size_t>(written);
            }
            if (used) std::puts(line.data());
        }
        const bool characterizing = characterizationTelemetry.load();
        nexus::OdometryTrace trace;
        for (unsigned batch = 0; batch < 16 && drive->popOdometryTrace(trace); ++batch) {
            if (characterizing) continue; // Drain old trace without flooding USB during a trial.
            if (lines++ % 100 == 0) {
                std::printf("NXOD_STATUS,dropped,%lu\n", static_cast<unsigned long>(drive->droppedOdometryTrace()));
                std::printf("NXOD_CONFIG,startup,forward_scale=%.9f,lateral_scale=%.9f,gyro_scale=%.9f,forward_offset_mm=%.6f,lateral_offset_mm=%.6f,track_mm=%.6f,native_imu=%u\n",
                    startupConfig.forwardScale, startupConfig.lateralScale, startupConfig.gyroScale,
                    startupConfig.forwardOffset / nexus::millimeter, startupConfig.lateralOffset / nexus::millimeter,
                    startupConfig.trackWidth / nexus::millimeter, static_cast<unsigned>(startupConfig.nativeImuHeading));
                std::printf("NXOD_HEADER,seq,host_s,f_mm,l_mm,left_mm,right_mm,imu_deg,x_mm,y_mm,heading_deg,bias_dps,gx_dps,gy_dps,gz_dps,f_before_ms,l_before_ms,g_before_ms,f_after_ms,l_after_ms,g_after_ms,epoch,dropped,valid,rejected,stationary,heading_source\n");
                std::printf("NXCTRL_HEADER,version,control_s,generation,status,active,target_x_mm,target_y_mm,target_deg,constrain_heading,ref_x_mm,ref_y_mm,ref_deg,v_mmps,lateral_mmps,omega_dps,left_v,right_v,iterations,budget_exceeded,compute_ms\n");
            }
            std::printf("NXOD,%lu,%.6f,%.4f,%.4f,%.4f,%.4f,%.6f,%.4f,%.4f,%.6f,%.6f,%.3f,%.3f,%.3f,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%u,%u,%u,%u\n",
                static_cast<unsigned long>(trace.sequence), trace.timestamp,
                trace.counters[0] / nexus::millimeter, trace.counters[1] / nexus::millimeter,
                trace.counters[2] / nexus::millimeter, trace.counters[3] / nexus::millimeter,
                nexus::degrees(trace.counters[4]), trace.pose.x / nexus::millimeter,
                trace.pose.y / nexus::millimeter, nexus::degrees(trace.pose.theta),
                nexus::degrees(trace.gyroBias), trace.rawGyroDps[0], trace.rawGyroDps[1], trace.rawGyroDps[2],
                static_cast<unsigned long>(trace.packetBeforeMs[0]),
                static_cast<unsigned long>(trace.packetBeforeMs[1]),
                static_cast<unsigned long>(trace.packetBeforeMs[2]),
                static_cast<unsigned long>(trace.packetAfterMs[0]),
                static_cast<unsigned long>(trace.packetAfterMs[1]),
                static_cast<unsigned long>(trace.packetAfterMs[2]),
                static_cast<unsigned long>(trace.epoch), static_cast<unsigned long>(trace.dropped),
                static_cast<unsigned>(trace.validMask), static_cast<unsigned>(trace.rejectedMask),
                static_cast<unsigned>(trace.stationary), static_cast<unsigned>(trace.headingSource));
        }
        if (odometryEnabled && !characterizing && pros::millis() - lastControlTrace >= 100) {
            lastControlTrace = pros::millis();
            const auto data = drive->diagnostics();
            std::printf("NXCTRL,9,%.6f,%lu,%s,%u,%.2f,%.2f,%.2f,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%u,%u,%.3f\n",
                data.controlTimestamp, static_cast<unsigned long>(data.controlGeneration),
                nexus::statusName(data.status), static_cast<unsigned>(data.motionActive),
                data.target.pose.x / nexus::millimeter, data.target.pose.y / nexus::millimeter,
                nexus::degrees(data.target.pose.theta), static_cast<unsigned>(data.target.constrainHeading),
                data.reference.pose.x / nexus::millimeter, data.reference.pose.y / nexus::millimeter,
                nexus::degrees(data.reference.pose.theta), data.estimate.state.v / nexus::millimeter,
                data.estimate.state.lateralV / nexus::millimeter, nexus::degrees(data.estimate.state.omega),
                data.command.left, data.command.right, data.solver.iterations,
                static_cast<unsigned>(data.solver.budgetExceeded), data.solver.computeMs);
        }
        pros::delay(10);
    }
}

void diagnosticsLoop() {
    std::FILE* log = config::logTelemetry ? std::fopen("/usd/nexus-log.csv", "w") : nullptr;
    if (log) std::fprintf(log, "time,x_mm,y_mm,theta,v_mmps,omega,pxx_mm2,pyy_mm2,ptt,confidence,slip,health,left_v,right_v,forward_pod_mm,lateral_pod_mm,left_encoder_mm,right_encoder_mm,gyro,gyro_rate,solver_ms,cost,iterations,status,motor_time,left_terminal_v,right_terminal_v,left_motor_mmps,right_motor_mmps,accel_x_g,accel_y_g,accel_z_g,raw_gyro_x_dps,raw_gyro_y_dps,raw_gyro_z_dps,accel_valid,raw_gyro_valid,heading_source,gyro_bias,accel_peak_g,gyro_peak_dps,accel_near_limit,gyro_near_limit,forward_valid,lateral_valid,left_valid,right_valid,gyro_valid,stationary,gyro_bias_std,rejected_increments,rejected_increment_mask,last_rejected_increment_mask,estimator_ms,estimator_overruns,odom_forward_x_mm,odom_forward_y_mm,odom_lateral_x_mm,odom_lateral_y_mm,odom_offset_x_mm,odom_offset_y_mm,quiet_breaks,quiet_last_break_mask,quiet_span_forward_mm,quiet_span_lateral_mm,quiet_span_left_mm,quiet_span_right_mm,quiet_last_gyro_dps,quiet_last_dt_ms,quiet_last_break_time,quiet_dwell_ms,quiet_encoder_span_mm,cycle_ms,cycle_max_ms,cycle_long_count,cycle_last_time,cycle_last_ms,cycle_last_rtos_ms,cycle_last_work_ms,cycle_last_publish_ms,cycle_last_wait_ms,cycle_last_clock_read_ms\n");
    unsigned frames = 0;
    unsigned page = 0;
    unsigned previousLcdButtons = 0;
    for (;;) {
        const auto data = drive->diagnostics();
        const auto& state = data.estimate.state;
        const auto lcdButtons = pros::lcd::read_buttons();
        if (!detail::calibrationScreenActive()) {
            const auto pressedLcd = lcdButtons & ~previousLcdButtons;
            if (pressedLcd & LCD_BTN_RIGHT) page = (page + 1) % 6;
            else if (pressedLcd & LCD_BTN_LEFT) page = (page + 5) % 6;
            drawDiagnostics(data, page);
        }
        previousLcdButtons = lcdButtons;
        if (log) {
            const double before = static_cast<double>(pros::micros()) * 1e-6;
            const auto left = measuredDriveSide(leftMotors, config::leftPorts.size(), config::leftOdometryMask);
            const auto right = measuredDriveSide(rightMotors, config::rightPorts.size(), config::rightOdometryMask);
            const double motorTime = (before + static_cast<double>(pros::micros()) * 1e-6) / 2;
            std::fprintf(log, "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.8f,%.8f,%.8f,%.4f,%.4f,%d,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.3f,%.6f,%u,%d,%.6f,%.4f,%.4f,%.6f,%.6f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%d,%d,%d,%.6f,%.4f,%.2f,%lu,%lu,%d,%d,%d,%d,%d,%d,%.6f,%lu,%u,%u,%.3f,%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%lu,%u,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.3f,%.3f,%lu,%.6f,%.3f,%lu,%.3f,%.3f,%.3f,%.3f\n",
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
                data.estimate.odometry.offsetX / nexus::millimeter, data.estimate.odometry.offsetY / nexus::millimeter,
                static_cast<unsigned long>(data.estimate.quiet.breaks), static_cast<unsigned>(data.estimate.quiet.lastBreakMask),
                data.estimate.quiet.wheelSpans[0] / nexus::millimeter, data.estimate.quiet.wheelSpans[1] / nexus::millimeter,
                data.estimate.quiet.wheelSpans[2] / nexus::millimeter, data.estimate.quiet.wheelSpans[3] / nexus::millimeter,
                nexus::degrees(data.estimate.quiet.gyroRate), data.estimate.quiet.sampleInterval * 1000,
                data.estimate.quiet.timestamp, data.estimate.quiet.dwell * 1000,
                data.estimate.quiet.encoderSpan / nexus::millimeter,
                data.timing.intervalMs, data.timing.worstIntervalMs,
                static_cast<unsigned long>(data.timing.longIntervals), data.timing.lastAt,
                data.timing.lastIntervalMs, static_cast<unsigned long>(data.timing.lastRtosMs),
                data.timing.lastWorkMs, data.timing.lastPublishMs, data.timing.lastWaitMs,
                data.timing.lastClockReadMs);
            if (++frames % 10 == 0) std::fflush(log);
        }
        pros::delay(50);
    }
}
}

namespace {
bool initializeHardware() {
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
        imu = std::make_unique<pros::Imu>(config::imuPort);
        pros::lcd::print(0, "Calibrating IMU: keep still");
        imuCalibrationFailed = imu->reset(true) != 1;
        imuCalibrationStatus = imuCalibrationFailed ? "errore" : "OK";
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
    std::printf("NEXUS heading: native IMU; local bias correction disabled; native calibration: %s\n",
                imuCalibrationStatus);
    displayTask = std::make_unique<pros::Task>(diagnosticsLoop, TASK_PRIORITY_DEFAULT - 1, 8192, "nexus-diagnostics");
    characterizationQueue = pros::c::queue_create(64, sizeof(CharacterizationRecord));
    odometryUsbTask = std::make_unique<pros::Task>([traceConfig = activeEstimatorConfig] { odometryUsbLoop(traceConfig); },
        TASK_PRIORITY_DEFAULT - 1, 8192, "nexus-usb-trace");
    detail::startCalibrationService();
    return true;
}
} // namespace

bool startInitialization() {
    if (!initialization.begin()) return initialization.state() != startup::InitializationState::failed;
    // PROS 4.2.1 keeps a daemon deadline from before its initialize wait. Returning
    // promptly avoids a priority-14 catch-up loop after a long PREMATCH/READY wait.
    // Priority 7 is below the initialize callback's 8: this worker cannot start
    // the blocking sequence before the callback returns to the kernel.
    startupTask = std::make_unique<pros::Task>([] {
        initialization.finish(initializeHardware());
    }, TASK_PRIORITY_DEFAULT - 1, 16384, "robot-startup");
    if (static_cast<pros::task_t>(*startupTask) == nullptr) {
        initialization.finish(false);
        pros::lcd::print(0, "STARTUP ERROR: task unavailable");
        return false;
    }
    return true;
}
bool ready() { return initialization.state() == startup::InitializationState::ready; }
bool waitUntilReady(bool autonomousMode) {
    const auto initialStatus = pros::competition::get_status();
    const startup::CallbackGate gate(autonomousMode, (initialStatus & COMPETITION_CONNECTED) != 0);
    for (;;) {
        const auto status = pros::competition::get_status();
        switch (gate.poll(initialization.state(), (status & COMPETITION_DISABLED) != 0,
                          (status & COMPETITION_AUTONOMOUS) != 0,
                          (status & COMPETITION_CONNECTED) != 0)) {
        case startup::CallbackAction::proceed: return true;
        case startup::CallbackAction::abort: return false;
        case startup::CallbackAction::wait: pros::delay(10); break;
        }
    }
}
nexus::Chassis& chassis() { return *drive; }
void intake(bool enabled) {
    intakeEnabled = enabled && !pros::competition::is_disabled() && !calibrationOwnsDriver();
    mechanisms.setDriverIntake(intakeEnabled.load());
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
    // The boot-only preMatch owns mechanisms until ready. Do not race it or
    // inspect drive while that pointer is still being constructed.
    if (!ready()) return;
    mechanisms.suspend();
    detail::abortCalibration();
    if (drive) drive->cancel();
    else { leftMotors.brake(); rightMotors.brake(); }
    lift(0); applyMacroArm(); intake(false);
}
void resetDriver() {
    if (!ready()) return;
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
    if (!ready() || calibrationOwnsDriver()) return;
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
        const char* message = line == 0 ? (fault ? "Errore meccanismo" : macroNotice.load()) : detail;
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

// Owns the display only during startup; callbacks just post an atomic event.
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
    detail::ButtonState<1> readyButton;
    auto previousPhase = startup::Phase::complete;
    auto previousMechanism = mechanism::State::off;
    std::uint32_t lastMechanismDisplay = 0;
    mechanism::Input lastMechanismInput;
    int lastArmCommand = 0;
    bool failedDuringMotion = false;
    // The UI/sequence starts at boot, including disabled. VEXos inhibits motor
    // power in disabled, so homing must wait instead of timing out or accepting
    // the unmoving motor as an endstop. Completed startup never runs again.
    while (gate.phase != startup::Phase::complete) {
        const bool pressedScreen = screen.pressed();
        const bool padConnected = detail::controllerIsConnected(controller.is_connected());
        // Require a fresh A press after entering READY. A held through homing
        // or controller reconnection must not acknowledge the new screen.
        readyButton.update({gate.phase == startup::Phase::waitReady && padConnected
            ? controller.get_digital(A) : PROS_ERR});
        const bool abort = gate.phase == startup::Phase::preMatch
            && padConnected && controller.get_digital(R2) == 1;
        const auto action = gate.tick(pros::millis(),
            mechanisms.state == mechanism::State::prepared,
            mechanisms.state == mechanism::State::fault || abort,
            pressedScreen || readyButton.pressed(0), pressedScreen,
            !pros::competition::is_disabled());
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
            // Startup owns mechanisms; normal callbacks wait until it completes.
            mechanism::applyArmMotor(arm, output, mechanisms.state, true);
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
        if (gate.phase != previousPhase || mechanisms.state != previousMechanism) {
            switch (gate.phase) {
            case startup::Phase::delayPreMatch:
                screen.show("AVVIO CTRL 9", "PreMatch automatico fra 1 secondo");
                break;
            case startup::Phase::waitMotorEnable:
                screen.show("PREMATCH IN ATTESA", "DISABLED: VEXos blocca i motori.\nAbilita o scollega il field controller.\nIl prematch partira automaticamente.");
                break;
            case startup::Phase::preMatch: {
                break;
            }
            case startup::Phase::waitReady:
                screen.show("PREMATCH COMPLETATO", "Posiziona il robot: premi A sul controller\noppure READY. Poi tienilo fermo.", "READY");
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
            previousPhase = gate.phase; previousMechanism = mechanisms.state;
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
        clampClosed.load(), liftStep, false, held(X), pressed(X));
    if (clampManualOverride) mechanisms.output.closed = clampClosed.load();
    if (releasePressed) {
        macroNotice = result.releaseAccepted ? "R2: rilascio richiesto"
            : mechanisms.state == mechanism::State::fault ? "R2 bloccato: errore macro"
            : !mechanisms.calibrated ? "R2: prematch mancante" : "R2: sequenza gia attiva";
        std::printf("R2 read: state %d -> %d, calibrated %d, accepted %d, arm %.1f, target %.1f, command %d; %s\n",
            static_cast<int>(before), static_cast<int>(mechanisms.state),
            mechanisms.calibrated, result.releaseAccepted, mechanisms.armPosition(in.arm),
            mechanisms.armTargetDegrees, mechanisms.output.arm, mechanisms.error);
    }
    lift(mechanisms.output.lift);
    intakeEnabled = mechanisms.output.intake != 0;
    if (mechanisms.output.intake) intakeMotors.move(mechanisms.output.intake);
    else intakeMotors.brake();
    if (!result.owns) return finishMacroUpdate(false);
    applyMacroArm();
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
void setCharacterizationTelemetry(bool enabled) { characterizationTelemetry = enabled; }

bool publishCharacterization(std::uint32_t run, std::uint32_t trial,
        std::uint32_t stage, std::uint32_t failure, const nexus::Diagnostics& data) {
    if (!characterizationQueue) return false;
    const double now = static_cast<double>(pros::micros()) * 1e-6;
    const auto left = measuredDriveSide(leftMotors, config::leftPorts.size(), config::leftOdometryMask);
    const auto right = measuredDriveSide(rightMotors, config::rightPorts.size(), config::rightOdometryMask);
    const double motorTime = (now + static_cast<double>(pros::micros()) * 1e-6) / 2;
    const auto batteryMv = pros::battery::get_voltage();
    const double battery = batteryMv == PROS_ERR ? std::numeric_limits<double>::quiet_NaN() : batteryMv * .001;
    const auto& s = data.sensors;
    const auto& e = data.estimate;
    const unsigned rejected = e.rejectedIncrementMask;
    const unsigned valid = (s.forwardValid && !(rejected & 1) ? 1u : 0u)
        | (s.lateralValid && !(rejected & 2) ? 2u : 0u)
        | (s.gyroValid && !(rejected & 16) ? 4u : 0u)
        | (s.accelerationValid ? 8u : 0u)
        | (std::isfinite(left[0]) && std::isfinite(left[1]) ? 16u : 0u)
        | (std::isfinite(right[0]) && std::isfinite(right[1]) ? 32u : 0u)
        | (std::isfinite(battery) && battery > 0 && battery < 15 ? 64u : 0u);
    const auto& c = activeEstimatorConfig;
    CharacterizationRecord record{{1, static_cast<double>(++characterizationSequence),
        static_cast<double>(run), static_cast<double>(trial), static_cast<double>(stage), static_cast<double>(failure),
        now, s.timestamp, data.command.left, data.command.right, left[0], right[0], battery,
        e.state.pose.x, e.state.pose.y, e.state.pose.theta, e.state.v, e.state.lateralV, e.state.omega,
        s.forward, s.lateral, s.gyro, left[1], right[1],
        s.accelerationG[0], s.accelerationG[1], s.accelerationG[2],
        static_cast<double>(valid), static_cast<double>(characterizationDropped),
        c.gyroScale, c.forwardScale, c.lateralScale, c.forwardOffset, c.lateralOffset, c.trackWidth, motorTime}};
    if (pros::c::queue_append(characterizationQueue, &record, 0)) return true;
    ++characterizationDropped;
    return false;
}

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
    if (robot::ready()) {
        robot::stop();
        sequence_.emplace(robot::chassis());
    }
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
void Auton::moveToPose(double x, double y, double heading, nexus::MoveOptions options, LiftMove liftMove) {
    if (!sequence_) return;
    if (liftMove == LiftMove::none) { sequence_->moveToPose(x, y, heading, options); return; }
    if (!beginLift(liftMove)) { sequence_->abort(nexus::MotionStatus::sensorFault); finishLift(); return; }
    sequence_->moveToPose(x, y, heading, options, [this] { return updateLift(); });
    finishLift();
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
bool Auton::beginLift(LiftMove move) {
    bool accepted = false;
    if (sequence_) sequence_->action([&] {
        const auto in = mechanismInput();
        if (!mechanisms.driverLiftAvailable() || !in.valid) return;
        mechanisms.cancelDriverLiftTarget();
        if (move == LiftMove::bottom) accepted = mechanisms.requestLiftBottom(in);
        else if (move == LiftMove::upStep) {
            mechanisms.adjustDriverLift(1, in);
            accepted = true;
        }
    });
    return accepted && updateLift();
}
bool Auton::updateLift() {
    bool healthy = false;
    if (sequence_) sequence_->action([&] {
        mechanisms.updateDriverLift(mechanismInput());
        robot::lift(mechanisms.output.lift);
        applyMacroArm();
        healthy = mechanisms.calibrated && mechanisms.state != mechanism::State::fault
            && mechanisms.state != mechanism::State::stopped;
    });
    return healthy;
}
void Auton::finishLift() {
    while (sequence_ && mechanisms.driverLiftMoving()) {
        if (!updateLift()) { sequence_->abort(nexus::MotionStatus::sensorFault); break; }
        sequence_->wait(10);
    }
    mechanisms.cancelDriverLiftTarget();
    robot::lift(0);
}
void Auton::liftToBottom() {
    if (!sequence_) return;
    if (!beginLift(LiftMove::bottom)) { sequence_->abort(nexus::MotionStatus::sensorFault); finishLift(); return; }
    finishLift();
}
void Auton::moveThrough(std::initializer_list<nexus::Waypoint> points, double heading, nexus::MoveOptions options, LiftMove liftMove) {
    if (!sequence_) return;
    if (liftMove == LiftMove::none) { sequence_->moveThrough(points, heading, options); return; }
    if (!beginLift(liftMove)) { sequence_->abort(nexus::MotionStatus::sensorFault); finishLift(); return; }
    sequence_->moveThrough(points, heading, options, [this] { return updateLift(); });
    finishLift();
}
void Auton::braccio(int power) {
    if (sequence_) sequence_->action([power] { if (power) mechanisms.cancel(); robot::braccio(power); });
}
nexus::MotionResult Auton::result() const {
    return sequence_ ? sequence_->result() : nexus::MotionResult{nexus::MotionStatus::invalidRequest};
}
} // namespace robot
