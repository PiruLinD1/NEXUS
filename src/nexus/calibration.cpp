#include "nexus/calibration.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>

namespace nexus { namespace calibration {
namespace {
using Row = std::array<double, 3>;
using Matrix = std::array<Row, 3>;
bool bounded(double x, double lo, double hi) { return std::isfinite(x) && x >= lo && x <= hi; }
double median(std::vector<double> values) {
    if (values.empty()) return 0;
    const std::size_t n = values.size();
    std::nth_element(values.begin(), values.begin() + n / 2, values.end());
    const double upper = values[n / 2];
    if (n % 2) return upper;
    return 0.5 * (upper + *std::max_element(values.begin(), values.begin() + n / 2));
}
double robustSigma(const std::vector<double>& residuals, double floor) {
    const double center = median(residuals);
    std::vector<double> deviations;
    deviations.reserve(residuals.size());
    for (double r : residuals) deviations.push_back(std::abs(r - center));
    return std::max(floor, 1.4826 * median(deviations));
}
void assessResiduals(FitQuality& quality, const std::vector<double>& residuals,
                     double floor, double maximumRms, double maximumOutlierFraction) {
    quality.samples = residuals.size();
    const double threshold = 4.5 * robustSigma(residuals, floor);
    double sum = 0, inlierSum = 0;
    quality.outliers = 0;
    quality.maxResidual = 0;
    for (double r : residuals) {
        sum += r * r;
        quality.maxResidual = std::max<double>(quality.maxResidual, std::abs(r));
        if (std::abs(r) > threshold) ++quality.outliers;
        else inlierSum += r * r;
    }
    const std::size_t inliers = residuals.size() - quality.outliers;
    quality.rms = std::sqrt(sum / std::max<std::size_t>(1, residuals.size()));
    quality.inlierRms = std::sqrt(inlierSum / std::max<std::size_t>(1, inliers));
    quality.accepted = inliers > 0 && quality.inlierRms <= maximumRms &&
        static_cast<double>(quality.outliers) / std::max<std::size_t>(1, residuals.size()) <= maximumOutlierFraction;
    quality.failure = quality.accepted ? FitFailure::none : FitFailure::excessiveResidual;
}
bool solve(Matrix a, Row b, Row& result) {
    for (std::size_t i = 0; i < 3; ++i) {
        std::size_t pivot = i;
        for (std::size_t j = i + 1; j < 3; ++j)
            if (std::abs(a[j][i]) > std::abs(a[pivot][i])) pivot = j;
        if (std::abs(a[pivot][i]) < 1e-12) return false;
        std::swap(a[pivot], a[i]); std::swap(b[pivot], b[i]);
        for (std::size_t j = i + 1; j < 3; ++j) {
            const double ratio = a[j][i] / a[i][i];
            for (std::size_t k = i; k < 3; ++k) a[j][k] -= ratio * a[i][k];
            b[j] -= ratio * b[i];
        }
    }
    for (int i = 2; i >= 0; --i) {
        double value = b[static_cast<std::size_t>(i)];
        for (int j = i + 1; j < 3; ++j)
            value -= a[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] * result[static_cast<std::size_t>(j)];
        result[static_cast<std::size_t>(i)] = value / a[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)];
    }
    return true;
}
double informationCondition(Matrix a) {
    // Jacobi eigensolver for a real symmetric 3x3 information matrix.
    for (unsigned iteration = 0; iteration < 24; ++iteration) {
        std::size_t p = 0, q = 1;
        for (std::size_t i = 0; i < 3; ++i) for (std::size_t j = i + 1; j < 3; ++j)
            if (std::abs(a[i][j]) > std::abs(a[p][q])) { p = i; q = j; }
        if (std::abs(a[p][q]) < 1e-12) break;
        const double angle = 0.5 * std::atan2(2 * a[p][q], a[q][q] - a[p][p]);
        const double c = std::cos(angle), s = std::sin(angle);
        const double pp = c*c*a[p][p] - 2*c*s*a[p][q] + s*s*a[q][q];
        const double qq = s*s*a[p][p] + 2*c*s*a[p][q] + c*c*a[q][q];
        for (std::size_t k = 0; k < 3; ++k) if (k != p && k != q) {
            const double kp = c*a[k][p] - s*a[k][q];
            const double kq = s*a[k][p] + c*a[k][q];
            a[k][p] = a[p][k] = kp; a[k][q] = a[q][k] = kq;
        }
        a[p][p] = pp; a[q][q] = qq; a[p][q] = a[q][p] = 0;
    }
    const double smallest = std::min(a[0][0], std::min(a[1][1], a[2][2]));
    const double largest = std::max(a[0][0], std::max(a[1][1], a[2][2]));
    return smallest <= 1e-12 ? std::numeric_limits<double>::infinity() : largest / smallest;
}
ScalarFit fitScalar(const std::vector<ReferenceSample>& input, double minimumInput,
                    double lower, double upper, double residualFloor, double maxRms) {
    ScalarFit result;
    std::vector<ReferenceSample> samples;
    std::size_t positive = 0, negative = 0;
    for (const auto& sample : input) {
        if (!bounded(sample.measured, -1e5, 1e5) || !bounded(sample.reference, -1e5, 1e5) ||
            std::abs(sample.measured) < minimumInput) continue;
        samples.push_back(sample);
        if (sample.measured > 0) ++positive; else ++negative;
    }
    result.quality.samples = samples.size();
    if (samples.size() < 6) return result;
    if (positive < 2 || negative < 2) { result.quality.failure = FitFailure::insufficientExcitation; return result; }
    std::vector<double> weights(samples.size(), 1), residuals(samples.size(), 0);
    for (unsigned iteration = 0; iteration < 12; ++iteration) {
        double information = 0, rhs = 0;
        for (std::size_t i = 0; i < samples.size(); ++i) {
            information += weights[i] * square(samples[i].measured);
            rhs += weights[i] * samples[i].measured * samples[i].reference;
        }
        if (information <= 1e-12) { result.quality.failure = FitFailure::illConditioned; return result; }
        result.value = rhs / information;
        for (std::size_t i = 0; i < samples.size(); ++i)
            residuals[i] = samples[i].reference - result.value * samples[i].measured;
        const double delta = 1.5 * robustSigma(residuals, residualFloor);
        for (std::size_t i = 0; i < samples.size(); ++i)
            weights[i] = std::abs(residuals[i]) > delta ? delta / std::abs(residuals[i]) : 1.0;
    }
    result.quality.condition = 1;
    assessResiduals(result.quality, residuals, residualFloor, maxRms, 0.25);
    if (result.quality.accepted) {
        // Outliers must not be the only evidence for one travel direction.
        const double threshold = 4.5 * robustSigma(residuals, residualFloor);
        positive = negative = 0;
        for (std::size_t i = 0; i < samples.size(); ++i) if (std::abs(residuals[i]) <= threshold) {
            if (samples[i].measured > 0) ++positive; else ++negative;
        }
        if (positive + negative < 6 || positive < 2 || negative < 2) {
            result.quality.accepted = false;
            result.quality.failure = FitFailure::insufficientExcitation;
        }
    }
    if (!bounded(result.value, lower, upper)) {
        result.quality.accepted = false; result.quality.failure = FitFailure::implausible;
    }
    return result;
}
bool wheelPlausible(double kS, double kV, double kA) {
    return bounded(kS, 0, 3) && bounded(kV, 0.05, 100) && bounded(kA, 0.002, 20);
}
constexpr std::size_t fileSize = 124;
using Bytes = std::array<unsigned char, fileSize>;
constexpr char magic[] = "VXMOCAL1";
std::uint32_t crc32(const unsigned char* bytes, std::size_t size) {
    std::uint32_t crc = 0xffffffffu;
    for (std::size_t i = 0; i < size; ++i) {
        crc ^= bytes[i];
        for (unsigned bit = 0; bit < 8; ++bit) crc = (crc >> 1) ^ ((crc & 1u) ? 0xedb88320u : 0u);
    }
    return ~crc;
}
void putInteger(Bytes& bytes, std::size_t& offset, std::uint64_t value, unsigned length) {
    for (unsigned i = 0; i < length; ++i) bytes[offset++] = static_cast<unsigned char>(value >> (i * 8));
}
std::uint64_t getInteger(const Bytes& bytes, std::size_t& offset, unsigned length) {
    std::uint64_t value = 0;
    for (unsigned i = 0; i < length; ++i) value |= static_cast<std::uint64_t>(bytes[offset++]) << (i * 8);
    return value;
}
Bytes encode(const CalibrationProfile& p) {
    static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559, "IEEE754 doubles required");
    Bytes bytes{};
    std::memcpy(bytes.data(), magic, 8);
    std::size_t offset = 8;
    putInteger(bytes, offset, 1, 4); putInteger(bytes, offset, p.hardwareFingerprint, 8);
    putInteger(bytes, offset, p.calibratedMask, 4);
    const std::array<double, 12> values{{p.forwardScale, p.lateralScale, p.gyroScale, p.forwardOffset,
        p.lateralOffset, p.trackWidth, p.kSLeft, p.kVLeft, p.kALeft, p.kSRight, p.kVRight, p.kARight}};
    for (double value : values) {
        std::uint64_t bits = 0; std::memcpy(&bits, &value, sizeof(bits));
        putInteger(bytes, offset, bits, 8);
    }
    putInteger(bytes, offset, crc32(bytes.data(), fileSize - 4), 4);
    return bytes;
}
StorageStatus readFile(const std::string& path, std::uint64_t expectedHardware, CalibrationProfile& output) {
    FILE* file = std::fopen(path.c_str(), "rb");
    if (!file) return StorageStatus::ioError;
    Bytes bytes{};
    const auto length = std::fread(bytes.data(), 1, bytes.size(), file);
    const int trailing = std::fgetc(file);
    const bool ioFailed = std::ferror(file) != 0;
    const int closeStatus = std::fclose(file);
    if (ioFailed || closeStatus != 0) return StorageStatus::ioError;
    if (length != fileSize || trailing != EOF || std::memcmp(bytes.data(), magic, 8) != 0)
        return StorageStatus::checksumMismatch;
    std::size_t crcOffset = fileSize - 4;
    if (getInteger(bytes, crcOffset, 4) != crc32(bytes.data(), fileSize - 4)) return StorageStatus::checksumMismatch;
    std::size_t offset = 8;
    if (getInteger(bytes, offset, 4) != 1) return StorageStatus::unsupportedVersion;
    CalibrationProfile p;
    p.hardwareFingerprint = getInteger(bytes, offset, 8);
    p.calibratedMask = static_cast<std::uint32_t>(getInteger(bytes, offset, 4));
    std::array<double*, 12> values{{&p.forwardScale, &p.lateralScale, &p.gyroScale, &p.forwardOffset,
        &p.lateralOffset, &p.trackWidth, &p.kSLeft, &p.kVLeft, &p.kALeft, &p.kSRight, &p.kVRight, &p.kARight}};
    for (double* value : values) {
        const std::uint64_t bits = getInteger(bytes, offset, 8); std::memcpy(value, &bits, sizeof(bits));
    }
    if (p.hardwareFingerprint != expectedHardware || expectedHardware == 0) return StorageStatus::wrongHardware;
    if (!validProfile(p)) return StorageStatus::invalidProfile;
    if (p.calibratedMask == 0) return StorageStatus::uncalibrated;
    output = p;
    return StorageStatus::ok;
}
bool removeIfExists(const std::string& path) { return std::remove(path.c_str()) == 0 || errno == ENOENT; }
} // namespace

const char* fitFailureName(FitFailure failure) {
    switch (failure) {
    case FitFailure::none: return "ok";
    case FitFailure::insufficientData: return "too few samples";
    case FitFailure::insufficientExcitation: return "turn both directions";
    case FitFailure::illConditioned: return "unobservable fit";
    case FitFailure::implausible: return "implausible parameters";
    case FitFailure::excessiveResidual: return "slip/noisy readings";
    case FitFailure::invalidData: return "sensor/timestamp fault";
    case FitFailure::moving: return "unexpected translation";
    case FitFailure::inconsistentDirections: return "CW/CCW mismatch";
    case FitFailure::capacityExceeded: return "capture too long";
    }
    return "unknown calibration fault";
}

WheelFit fitWheel(const std::vector<WheelSample>& input, const FitLimits& limits) {
    WheelFit result;
    if (limits.minSamples < 6 || !bounded(limits.maxVoltage, 1, 24) ||
        !bounded(limits.minSpeed, 0.001, 2) || !bounded(limits.maxCondition, 1, 1e8) ||
        !bounded(limits.maxInlierRms, 0.001, 5) || !bounded(limits.maxOutlierFraction, 0, 0.4)) {
        result.quality.failure = FitFailure::implausible; return result;
    }
    std::vector<WheelSample> samples;
    std::size_t positive = 0, negative = 0;
    double v2 = 0, a2 = 0, minimumSpeed = 1e9, maximumSpeed = 0;
    for (const auto& sample : input) {
        if (!sample.valid || !bounded(sample.voltage, -limits.maxVoltage, limits.maxVoltage) ||
            !bounded(sample.velocity, -10, 10) || !bounded(sample.acceleration, -100, 100) ||
            std::abs(sample.velocity) < limits.minSpeed) continue;
        samples.push_back(sample);
        v2 += square(sample.velocity); a2 += square(sample.acceleration);
        minimumSpeed = std::min<double>(minimumSpeed, std::abs(sample.velocity));
        maximumSpeed = std::max<double>(maximumSpeed, std::abs(sample.velocity));
        if (sample.velocity > 0) ++positive; else ++negative;
    }
    result.quality.samples = samples.size();
    if (samples.size() < limits.minSamples) return result;
    const double vScale = std::sqrt(v2 / samples.size()), aScale = std::sqrt(a2 / samples.size());
    if (positive < 5 || negative < 5 || aScale < 0.15 || maximumSpeed - minimumSpeed < 0.15) {
        result.quality.failure = FitFailure::insufficientExcitation; return result;
    }
    std::vector<Row> rows;
    rows.reserve(samples.size());
    for (const auto& sample : samples)
        rows.push_back(Row{{sample.velocity > 0 ? 1.0 : -1.0, sample.velocity / vScale, sample.acceleration / aScale}});
    std::vector<double> weights(samples.size(), 1), residuals(samples.size(), 0);
    Row coefficients{};
    for (unsigned iteration = 0; iteration < 12; ++iteration) {
        Matrix information{}; Row rhs{};
        for (std::size_t i = 0; i < samples.size(); ++i) for (std::size_t r = 0; r < 3; ++r) {
            rhs[r] += weights[i] * rows[i][r] * samples[i].voltage;
            for (std::size_t c = 0; c < 3; ++c) information[r][c] += weights[i] * rows[i][r] * rows[i][c];
        }
        result.quality.condition = informationCondition(information);
        if (result.quality.condition > limits.maxCondition || !solve(information, rhs, coefficients)) {
            result.quality.failure = FitFailure::illConditioned; return result;
        }
        for (std::size_t i = 0; i < samples.size(); ++i)
            residuals[i] = samples[i].voltage - (rows[i][0]*coefficients[0] + rows[i][1]*coefficients[1] + rows[i][2]*coefficients[2]);
        const double delta = 1.5 * robustSigma(residuals, 0.02);
        for (std::size_t i = 0; i < samples.size(); ++i)
            weights[i] = std::abs(residuals[i]) > delta ? delta / std::abs(residuals[i]) : 1.0;
    }
    result.kS = coefficients[0]; result.kV = coefficients[1] / vScale; result.kA = coefficients[2] / aScale;
    assessResiduals(result.quality, residuals, 0.02, limits.maxInlierRms, limits.maxOutlierFraction);
    if (result.quality.accepted) {
        const double threshold = 4.5 * robustSigma(residuals, 0.02);
        positive = negative = 0;
        a2 = 0; minimumSpeed = 1e9; maximumSpeed = 0;
        for (std::size_t i = 0; i < samples.size(); ++i) if (std::abs(residuals[i]) <= threshold) {
            if (samples[i].velocity > 0) ++positive; else ++negative;
            a2 += square(samples[i].acceleration);
            minimumSpeed = std::min<double>(minimumSpeed, std::abs(samples[i].velocity));
            maximumSpeed = std::max<double>(maximumSpeed, std::abs(samples[i].velocity));
        }
        const std::size_t inliers = positive + negative;
        if (inliers < limits.minSamples || positive < 5 || negative < 5 ||
            a2 < square(0.15) * inliers || maximumSpeed - minimumSpeed < 0.15) {
            result.quality.accepted = false;
            result.quality.failure = FitFailure::insufficientExcitation;
        }
    }
    if (!wheelPlausible(result.kS, result.kV, result.kA)) {
        result.quality.accepted = false; result.quality.failure = FitFailure::implausible;
    }
    return result;
}

ScalarFit fitTravelScale(const std::vector<ReferenceSample>& samples) {
    return fitScalar(samples, 0.20, 0.5, 1.5, 0.002, 0.025);
}
DistanceFit fitDistance(const std::vector<ReferenceSample>& input) {
    DistanceFit result;
    std::vector<ReferenceSample> samples;
    for (const auto& sample : input)
        if (bounded(sample.measured, 0.020, 2.000) && bounded(sample.reference, 0.020, 2.000))
            samples.push_back(sample);
    result.quality.samples = samples.size();
    if (samples.size() < 12) return result;
    std::sort(samples.begin(), samples.end(), [](const ReferenceSample& a, const ReferenceSample& b) {
        return a.measured < b.measured;
    });
    const auto excited = [](const std::vector<ReferenceSample>& data) {
        if (data.size() < 12) return false;
        const double span = data.back().measured - data.front().measured;
        if (span < 0.500) return false;
        std::array<std::size_t, 3> bins{};
        for (const auto& sample : data) {
            const auto bin = std::min<std::size_t>(2, static_cast<std::size_t>(
                3 * (sample.measured - data.front().measured) / span));
            ++bins[bin];
        }
        return bins[0] >= 3 && bins[1] >= 3 && bins[2] >= 3;
    };
    if (!excited(samples)) { result.quality.failure = FitFailure::insufficientExcitation; return result; }

    // A bounded Theil-Sen seed prevents a few large distance errors from setting
    // the initial residual scale. Work/storage for the pair slopes stays bounded
    // even when the offline input contains a long stationary capture.
    constexpr std::size_t maximumSeedSamples = 65;
    const std::size_t seedCount = std::min(maximumSeedSamples, samples.size());
    std::vector<double> slopes;
    slopes.reserve(seedCount * (seedCount - 1) / 2);
    for (std::size_t i = 0; i < seedCount; ++i) {
        const auto& a = samples[i * (samples.size() - 1) / (seedCount - 1)];
        for (std::size_t j = i + 1; j < seedCount; ++j) {
            const auto& b = samples[j * (samples.size() - 1) / (seedCount - 1)];
            if (b.measured - a.measured >= 0.10)
                slopes.push_back((b.reference - a.reference) / (b.measured - a.measured));
        }
    }
    if (slopes.empty()) { result.quality.failure = FitFailure::illConditioned; return result; }
    result.scale = median(slopes);
    std::vector<double> residuals(samples.size());
    for (std::size_t i = 0; i < samples.size(); ++i)
        residuals[i] = samples[i].reference - result.scale * samples[i].measured;
    result.offset = median(residuals);

    double center = 0, spread = 0;
    for (const auto& sample : samples) center += sample.measured;
    center /= samples.size();
    for (const auto& sample : samples) spread += square(sample.measured - center);
    spread = std::sqrt(spread / samples.size());
    std::vector<double> weights(samples.size(), 1.0);
    for (unsigned iteration = 0; iteration < 20; ++iteration) {
        for (std::size_t i = 0; i < samples.size(); ++i)
            residuals[i] = samples[i].reference - result.scale * samples[i].measured - result.offset;
        const double delta = 1.5 * robustSigma(residuals, 0.002);
        double w = 0, wx = 0, wy = 0, wxx = 0, wxy = 0;
        for (std::size_t i = 0; i < samples.size(); ++i) {
            const double r = std::abs(residuals[i]);
            weights[i] = r > delta ? delta / r : 1.0;
            const double x = (samples[i].measured - center) / spread;
            w += weights[i]; wx += weights[i] * x; wy += weights[i] * samples[i].reference;
            wxx += weights[i] * x * x; wxy += weights[i] * x * samples[i].reference;
        }
        const double discriminant = std::hypot(w - wxx, 2 * wx);
        const double smallest = 0.5 * (w + wxx - discriminant);
        result.quality.condition = smallest > 1e-12 ?
            0.5 * (w + wxx + discriminant) / smallest : std::numeric_limits<double>::infinity();
        const double determinant = w * wxx - wx * wx;
        if (result.quality.condition > 100 || determinant <= 1e-12) {
            result.quality.failure = FitFailure::illConditioned; return result;
        }
        result.scale = (w * wxy - wx * wy) / determinant / spread;
        result.offset = (wy - result.scale * spread * wx) / w - result.scale * center;
    }
    for (std::size_t i = 0; i < samples.size(); ++i)
        residuals[i] = samples[i].reference - result.scale * samples[i].measured - result.offset;
    assessResiduals(result.quality, residuals, 0.002, 0.025, 0.20);
    if (result.quality.accepted) {
        const double threshold = 4.5 * robustSigma(residuals, 0.002);
        std::vector<ReferenceSample> inliers;
        for (std::size_t i = 0; i < samples.size(); ++i)
            if (std::abs(residuals[i]) <= threshold) inliers.push_back(samples[i]);
        if (!excited(inliers)) {
            result.quality.accepted = false; result.quality.failure = FitFailure::insufficientExcitation;
        } else {
            result.measuredMin = inliers.front().measured; result.measuredMax = inliers.back().measured;
        }
    }
    if (!bounded(result.scale, 0.8, 1.2) || !bounded(result.offset, -0.100, 0.100)) {
        result.quality.accepted = false; result.quality.failure = FitFailure::implausible;
    }
    return result;
}
RotationFit fitRotation(const std::vector<RotationSample>& samples, double forwardScale, double lateralScale) {
    RotationFit result;
    std::vector<ReferenceSample> gyro, forward, lateral, drive;
    std::size_t forwardArcs = 0, lateralArcs = 0;
    for (const auto& sample : samples) {
        if (!bounded(sample.referenceAngle, -100, 100) || std::abs(sample.referenceAngle) < 0.30) continue;
        if (sample.gyroValid) gyro.push_back({sample.gyroAngle, sample.referenceAngle});
        const bool driveValid = sample.driveValid && bounded(sample.leftTravel, -1e5, 1e5) &&
            bounded(sample.rightTravel, -1e5, 1e5);
        const double center = driveValid ? 0.5 * (sample.leftTravel + sample.rightTravel) : 0;
        const double differential = driveValid ? sample.leftTravel - sample.rightTravel : 0;
        // Use the same pure-turn check as the on-robot collector. A small,
        // measured center drift is observable and must not become pod offset.
        // With no drive measurements, the caller must establish a pure turn.
        const bool arc = driveValid && std::abs(2 * center) >
            std::max(0.002, 0.06 * std::abs(differential));
        if (sample.forwardValid && bounded(forwardScale, 0.5, 1.5) &&
            bounded(sample.forwardTravel * forwardScale, -1e5, 1e5)) {
            if (arc) ++forwardArcs;
            else forward.push_back({sample.referenceAngle, center - sample.forwardTravel * forwardScale});
        }
        if (sample.lateralValid && bounded(lateralScale, 0.5, 1.5) &&
            bounded(sample.lateralTravel * lateralScale, -1e5, 1e5)) {
            if (arc) ++lateralArcs;
            else lateral.push_back({sample.referenceAngle, sample.lateralTravel * lateralScale});
        }
        if (driveValid) drive.push_back({sample.referenceAngle, differential});
    }
    result.gyroScale = fitScalar(gyro, 0.30, 0.8, 1.2, 0.003, 0.015);
    result.forwardOffset = fitScalar(forward, 0.30, -1, 1, 0.002, 0.01);
    result.lateralOffset = fitScalar(lateral, 0.30, -1, 1, 0.002, 0.01);
    result.trackWidth = fitScalar(drive, 0.30, 0.10, 1.0, 0.003, 0.015);
    const auto includeArcs = [](ScalarFit& fit, std::size_t rejected) {
        fit.quality.samples += rejected;
        fit.quality.outliers += rejected;
        if (rejected && (fit.quality.outliers > 0.25 * fit.quality.samples ||
                         fit.quality.failure == FitFailure::insufficientData)) {
            fit.quality.accepted = false;
            fit.quality.failure = FitFailure::moving;
        }
    };
    includeArcs(result.forwardOffset, forwardArcs);
    includeArcs(result.lateralOffset, lateralArcs);
    return result;
}

bool validProfile(const CalibrationProfile& p) {
    return p.hardwareFingerprint != 0 && (p.calibratedMask & ~255u) == 0 &&
        bounded(p.forwardScale, 0.5, 1.5) && bounded(p.lateralScale, 0.5, 1.5) &&
        bounded(p.gyroScale, 0.8, 1.2) && bounded(p.forwardOffset, -1, 1) &&
        bounded(p.lateralOffset, -1, 1) && bounded(p.trackWidth, 0.10, 1.0) &&
        wheelPlausible(p.kSLeft, p.kVLeft, p.kALeft) && wheelPlausible(p.kSRight, p.kVRight, p.kARight);
}
bool applyProfile(const CalibrationProfile& p, EstimatorConfig& e, DynamicsConfig& d) {
    if (!validProfile(p) || p.calibratedMask == 0) return false;
    if (p.calibratedMask & forwardScaleField) e.forwardScale = p.forwardScale;
    if (p.calibratedMask & lateralScaleField) e.lateralScale = p.lateralScale;
    if (p.calibratedMask & gyroScaleField) e.gyroScale = p.gyroScale;
    if (p.calibratedMask & forwardOffsetField) e.forwardOffset = p.forwardOffset;
    if (p.calibratedMask & lateralOffsetField) e.lateralOffset = p.lateralOffset;
    if (p.calibratedMask & trackWidthField) e.trackWidth = d.trackWidth = p.trackWidth;
    if (p.calibratedMask & leftDynamicsField) { d.kSLeft = p.kSLeft; d.kVLeft = p.kVLeft; d.kALeft = p.kALeft; }
    if (p.calibratedMask & rightDynamicsField) { d.kSRight = p.kSRight; d.kVRight = p.kVRight; d.kARight = p.kARight; }
    return true;
}
StorageStatus saveProfile(const std::string& path, const CalibrationProfile& p) {
    if (path.empty() || !validProfile(p)) return StorageStatus::invalidProfile;
    if (p.calibratedMask == 0) return StorageStatus::uncalibrated;
    const std::string temporary = path + ".tmp", backup = path + ".bak";
    const Bytes bytes = encode(p);
    FILE* file = std::fopen(temporary.c_str(), "wb");
    if (!file) return StorageStatus::ioError;
    const bool writeSucceeded = std::fwrite(bytes.data(), 1, bytes.size(), file) == bytes.size();
    const bool flushSucceeded = std::fflush(file) == 0;
    const bool closeSucceeded = std::fclose(file) == 0;
    if (!writeSucceeded || !flushSucceeded || !closeSucceeded) { removeIfExists(temporary); return StorageStatus::ioError; }
    CalibrationProfile verified;
    if (readFile(temporary, p.hardwareFingerprint, verified) != StorageStatus::ok) {
        removeIfExists(temporary); return StorageStatus::ioError;
    }
    // Keep a known-good backup. Never replace it with a corrupt primary file.
    CalibrationProfile existing;
    const bool primaryValid = readFile(path, p.hardwareFingerprint, existing) == StorageStatus::ok;
    if (primaryValid) {
        if (!removeIfExists(backup) || std::rename(path.c_str(), backup.c_str()) != 0) {
            removeIfExists(temporary); return StorageStatus::ioError;
        }
    } else if (!removeIfExists(path)) { removeIfExists(temporary); return StorageStatus::ioError; }
    if (std::rename(temporary.c_str(), path.c_str()) != 0) {
        if (primaryValid) std::rename(backup.c_str(), path.c_str());
        removeIfExists(temporary); return StorageStatus::ioError;
    }
    return StorageStatus::ok;
}
StorageStatus loadProfile(const std::string& path, std::uint64_t expectedHardware,
                          CalibrationProfile& p, bool* recoveredBackup) {
    if (recoveredBackup) *recoveredBackup = false;
    if (path.empty()) return StorageStatus::ioError;
    const StorageStatus primaryStatus = readFile(path, expectedHardware, p);
    if (primaryStatus == StorageStatus::ok) return primaryStatus;
    if (readFile(path + ".bak", expectedHardware, p) == StorageStatus::ok) {
        if (recoveredBackup) *recoveredBackup = true;
        return StorageStatus::ok;
    }
    return primaryStatus;
}
}} // namespace nexus::calibration
