#include "nexus/calibration.hpp"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using namespace nexus::calibration;
constexpr double metresPerMillimetre = 0.001;
constexpr const char* wheelUnits = "# units: voltage=V,velocity=mm/s,acceleration=mm/s^2";
std::string trim(const std::string& text) {
    const auto first = text.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return {};
    return text.substr(first, text.find_last_not_of(" \t\r\n") - first + 1);
}
std::vector<std::vector<double>> readCsv(const std::string& path, const std::string& header,
                                       std::size_t columns, const char* requiredUnits = nullptr) {
    std::ifstream file(path);
    if (!file) throw std::runtime_error("Cannot open " + path);
    std::vector<std::vector<double>> result;
    std::string line;
    std::size_t lineNumber = 0;
    bool headerFound = false;
    bool unitsFound = requiredUnits == nullptr;
    while (std::getline(file, line)) {
        ++lineNumber; line = trim(line);
        if (line.empty()) continue;
        if (line[0] == '#') {
            if (requiredUnits && line == requiredUnits) unitsFound = true;
            continue;
        }
        if (!headerFound) {
            if (!unitsFound) throw std::runtime_error(path + ": expected unit declaration before header: " + requiredUnits);
            if (line != header) throw std::runtime_error(path + ": expected CSV header " + header);
            headerFound = true; continue;
        }
        std::stringstream stream(line);
        std::string token;
        std::vector<double> row;
        while (std::getline(stream, token, ',')) {
            token = trim(token);
            char* end = nullptr; errno = 0;
            const double value = std::strtod(token.c_str(), &end);
            if (token.empty() || end == token.c_str() || *end != '\0' || errno == ERANGE)
                throw std::runtime_error(path + ": invalid number on line " + std::to_string(lineNumber));
            row.push_back(value);
        }
        if (row.size() != columns || line.back() == ',')
            throw std::runtime_error(path + ": wrong column count on line " + std::to_string(lineNumber));
        result.push_back(row);
    }
    if (!headerFound || result.empty()) throw std::runtime_error(path + ": no measurements");
    return result;
}
const char* failureName(FitFailure failure) {
    switch (failure) {
    case FitFailure::none: return "accepted";
    case FitFailure::insufficientData: return "insufficient valid samples";
    case FitFailure::insufficientExcitation: return "insufficient excitation / coverage";
    case FitFailure::illConditioned: return "unobservable / ill-conditioned model";
    case FitFailure::implausible: return "physically implausible parameters";
    case FitFailure::excessiveResidual: return "excessive model error / outliers";
    case FitFailure::invalidData: return "sensor / timestamp fault";
    case FitFailure::moving: return "unexpected translation";
    case FitFailure::inconsistentDirections: return "CW / CCW mismatch";
    case FitFailure::capacityExceeded: return "capture too long";
    }
    return "unknown";
}
void report(const std::string& name, const FitQuality& quality, const char* residualUnit,
            double residualScale = 1.0) {
    std::cout << name << ": " << failureName(quality.failure) << "; samples=" << quality.samples
              << "; outliers=" << quality.outliers << "; rms=" << quality.rms * residualScale
              << ' ' << residualUnit << "; inlier_rms=" << quality.inlierRms * residualScale
              << ' ' << residualUnit << "; condition=" << quality.condition << '\n';
}
std::vector<WheelSample> readWheel(const std::string& path) {
    std::vector<WheelSample> samples;
    for (const auto& row : readCsv(path, "voltage,velocity,acceleration", 3, wheelUnits))
        samples.push_back({row[0], row[1] * metresPerMillimetre,
                          row[2] * metresPerMillimetre, true});
    return samples;
}
std::uint64_t fingerprint(const std::string& value) {
    if (value.empty() || value[0] == '-') throw std::runtime_error("Hardware fingerprint must be nonzero unsigned integer");
    std::size_t parsed = 0;
    const auto id = std::stoull(value, &parsed, 0);
    if (parsed != value.size() || id == 0) throw std::runtime_error("Invalid hardware fingerprint");
    return static_cast<std::uint64_t>(id);
}
CalibrationProfile initialProfile(std::uint64_t id, const char* baseline) {
    CalibrationProfile profile; profile.hardwareFingerprint = id;
    if (baseline && loadProfile(baseline, id, profile) != StorageStatus::ok)
        throw std::runtime_error("Baseline profile unreadable, corrupt, or from another robot");
    return profile;
}
void usage() {
    std::cerr << "Usage:\n"
        "  nexus_calibrate fit-wheel left.csv right.csv output.cfg hardware_id [baseline.cfg]\n"
        "  nexus_calibrate fit-travel forward|lateral travel.csv output.cfg hardware_id [baseline.cfg]\n"
        "  nexus_calibrate fit-rotation rotation.csv output.cfg hardware_id [baseline.cfg]\n"
        "  nexus_calibrate fit-distance distance.csv\n"
        "CSV/report units: millimetres, mm/s, mm/s^2, seconds, radians, volts.\n"
        "Wheel CSVs require: # units: voltage=V,velocity=mm/s,acceleration=mm/s^2\n"
        "Travel/rotation CSVs require explicit _mm/_rad headers; see docs/calibration.md.\n"
        "Hardware ID: decimal or 0x hexadecimal. Binary profiles retain internal SI units.\n"
        "fit-distance prints per-sensor configuration only; it does not write a binary profile.\n"
        "Never run identification in the control loop. See docs/calibration.md.\n";
}
} // namespace

int main(int argc, char** argv) {
    try {
        if (argc < 2) { usage(); return EXIT_FAILURE; }
        std::cout << std::setprecision(10);
        const std::string command = argv[1];
        CalibrationProfile profile;
        std::string output;
        if (command == "fit-distance" && argc == 3) {
            std::vector<ReferenceSample> samples;
            for (const auto& row : readCsv(argv[2], "measured_mm,reference_mm", 2))
                samples.push_back({row[0] * metresPerMillimetre, row[1] * metresPerMillimetre});
            const auto fit = fitDistance(samples);
            report("distance", fit.quality, "mm", 1000.0);
            if (!fit.quality.accepted) return EXIT_FAILURE;
            std::cout << "distance_scale=" << fit.scale << " distance_offset_mm=" << fit.offset * 1000.0
                      << " measured_min_mm=" << fit.measuredMin * 1000.0
                      << " measured_max_mm=" << fit.measuredMax * 1000.0 << '\n'
                      << "WallSensor configuration: .distanceScale = " << fit.scale
                      << ", .distanceOffset = " << fit.offset * 1000.0
                      << ", .minimumDistance = " << fit.measuredMin * 1000.0
                      << ", .maximumDistance = " << fit.measuredMax * 1000.0 << " // lengths in mm\n"
                      << "Validate on independent distances, target surfaces and angles. "
                         "Fit residuals do not establish runtime uncertainty or accuracy outside the measured domain.\n";
            return EXIT_SUCCESS;
        } else if (command == "fit-wheel" && (argc == 6 || argc == 7)) {
            output = argv[4]; profile = initialProfile(fingerprint(argv[5]), argc == 7 ? argv[6] : nullptr);
            const auto left = fitWheel(readWheel(argv[2])), right = fitWheel(readWheel(argv[3]));
            report("left", left.quality, "V"); report("right", right.quality, "V");
            if (!left.quality.accepted || !right.quality.accepted) return EXIT_FAILURE;
            profile.kSLeft = left.kS; profile.kVLeft = left.kV; profile.kALeft = left.kA;
            profile.kSRight = right.kS; profile.kVRight = right.kV; profile.kARight = right.kA;
            profile.calibratedMask |= leftDynamicsField | rightDynamicsField;
            std::cout << "left kS=" << left.kS << " V kV=" << left.kV * metresPerMillimetre
                      << " V/(mm/s) kA=" << left.kA * metresPerMillimetre << " V/(mm/s^2)\n"
                      << "right kS=" << right.kS << " V kV=" << right.kV * metresPerMillimetre
                      << " V/(mm/s) kA=" << right.kA * metresPerMillimetre << " V/(mm/s^2)\n";
        } else if (command == "fit-travel" && (argc == 6 || argc == 7)) {
            const std::string axis = argv[2];
            if (axis != "forward" && axis != "lateral") throw std::runtime_error("Axis must be forward or lateral");
            output = argv[4]; profile = initialProfile(fingerprint(argv[5]), argc == 7 ? argv[6] : nullptr);
            std::vector<ReferenceSample> samples;
            for (const auto& row : readCsv(argv[3], "measured_mm,reference_mm", 2))
                samples.push_back({row[0] * metresPerMillimetre, row[1] * metresPerMillimetre});
            const auto fit = fitTravelScale(samples); report(axis, fit.quality, "mm", 1000.0);
            if (!fit.quality.accepted) return EXIT_FAILURE;
            if (axis == "forward") {
                if (profile.forwardScale != fit.value) profile.calibratedMask &= ~forwardOffsetField;
                profile.forwardScale = fit.value; profile.calibratedMask |= forwardScaleField;
            } else {
                if (profile.lateralScale != fit.value) profile.calibratedMask &= ~lateralOffsetField;
                profile.lateralScale = fit.value; profile.calibratedMask |= lateralScaleField;
            }
            std::cout << axis << " scale=" << fit.value << '\n';
        } else if (command == "fit-rotation" && (argc == 5 || argc == 6)) {
            output = argv[3]; profile = initialProfile(fingerprint(argv[4]), argc == 6 ? argv[5] : nullptr);
            std::vector<RotationSample> samples;
            for (const auto& row : readCsv(argv[2],
                    "reference_angle_rad,gyro_angle_rad,forward_travel_mm,lateral_travel_mm,left_travel_mm,right_travel_mm", 6))
                samples.push_back({row[0], row[1], row[2] * metresPerMillimetre,
                                   row[3] * metresPerMillimetre, row[4] * metresPerMillimetre,
                                   row[5] * metresPerMillimetre});
            const auto fits = fitRotation(samples, profile.forwardScale, profile.lateralScale);
            report("gyro scale", fits.gyroScale.quality, "rad");
            report("track width", fits.trackWidth.quality, "mm", 1000.0);
            std::uint32_t newMask = 0;
            if (fits.gyroScale.quality.accepted) {
                // On-robot geometry is relative to the configured gyro scale.
                // The v1 profile does not record fit provenance, so preserve
                // geometry only when this run independently remeasures it.
                if (profile.gyroScale != fits.gyroScale.value)
                    profile.calibratedMask &= ~(forwardOffsetField | lateralOffsetField | trackWidthField);
                profile.gyroScale = fits.gyroScale.value; newMask |= gyroScaleField;
            }
            if (fits.trackWidth.quality.accepted) { profile.trackWidth = fits.trackWidth.value; newMask |= trackWidthField; }
            // Pod scale and offset cannot be separated using pure rotation alone.
            if ((profile.calibratedMask & forwardScaleField) && fits.forwardOffset.quality.accepted) {
                profile.forwardOffset = fits.forwardOffset.value; newMask |= forwardOffsetField;
                report("forward offset", fits.forwardOffset.quality, "mm", 1000.0);
            } else std::cout << "forward offset unchanged: needs accepted travel scale and rotation fit\n";
            if ((profile.calibratedMask & lateralScaleField) && fits.lateralOffset.quality.accepted) {
                profile.lateralOffset = fits.lateralOffset.value; newMask |= lateralOffsetField;
                report("lateral offset", fits.lateralOffset.quality, "mm", 1000.0);
            } else std::cout << "lateral offset unchanged: needs accepted travel scale and rotation fit\n";
            if (!newMask) return EXIT_FAILURE;
            profile.calibratedMask |= newMask;
            std::cout << "gyro_scale=" << profile.gyroScale << " track_width_mm=" << profile.trackWidth * 1000.0
                      << " forward_offset_mm=" << profile.forwardOffset * 1000.0
                      << " lateral_offset_mm=" << profile.lateralOffset * 1000.0 << '\n';
        } else { usage(); return EXIT_FAILURE; }
        const auto status = saveProfile(output, profile);
        if (status != StorageStatus::ok) throw std::runtime_error("Could not save validated profile (status " +
            std::to_string(static_cast<int>(status)) + ")");
        std::cout << "Saved " << output << "; hardware_id=" << profile.hardwareFingerprint
                  << "; calibrated_mask=" << profile.calibratedMask << '\n';
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "Calibration failed: " << error.what() << '\n'; return EXIT_FAILURE;
    }
}
