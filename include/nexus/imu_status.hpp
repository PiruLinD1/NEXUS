#pragma once
#include <cstdint>

namespace nexus::detail {
// PROS imu_get_status passes through VEX flags: bit 0 is calibration,
// bits 1..3 encode mounting orientation. A ready IMU need not report zero.
// The error sentinel is 0xff. Keep SDK-independent for regression tests.
constexpr bool imuStatusReady(std::uint32_t status) {
    return status < 0xffu && (status & 1u) == 0;
}
} // namespace nexus::detail
