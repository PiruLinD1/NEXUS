#pragma once
// Small behavioral test double for the adapter's PROS boundary. Queues copy
// payloads and workers are real threads; this is not a V5 timing simulator.
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <initializer_list>
#include <limits>
#include <mutex>
#include <thread>
#include <vector>

#define PROS_ERR (std::numeric_limits<std::int32_t>::max())
#define PROS_ERR_F (INFINITY)
#define PROS_SUCCESS 1
#define TASK_PRIORITY_DEFAULT 8

namespace pros {
namespace test {
inline std::atomic<bool> running{true}, disabled{false}, autonomous{false}, connected{false};
inline std::atomic<bool> pauseCommands{false};
inline std::atomic<bool> pauseEstimator{false}, estimatorPaused{false};
inline std::atomic<bool> pauseAfterEstimatorCycle{false};
inline thread_local bool estimatorWorker = false;
inline std::atomic<unsigned> commandEnqueues{0};
inline std::atomic<std::uint32_t> failQueueCreationOfSize{0};
inline const auto origin = std::chrono::steady_clock::now();
// PROS exposes separate RTOS and VEX clocks. Tests may advance the high-res
// clock independently, without changing the scheduler's elapsed milliseconds.
inline std::atomic<std::uint64_t> highResolutionOffsetMicros{0};
inline std::uint64_t elapsedMicros() {
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - origin).count());
}
inline std::vector<std::thread*> workers;
struct Shutdown {};
inline void check() { if (!running.load()) throw Shutdown{}; }
inline void stop() {
    running = false;
    for (auto* worker : workers) if (worker->joinable()) worker->join();
}
}
inline std::uint64_t micros() {
    return test::elapsedMicros() + test::highResolutionOffsetMicros.load();
}
inline std::uint32_t millis() { return static_cast<std::uint32_t>(test::elapsedMicros() / 1000); }
inline void delay(std::uint32_t milliseconds) {
    test::check(); std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds)); test::check();
}
namespace competition {
inline bool is_disabled() { return test::disabled; }
inline bool is_autonomous() { return test::autonomous; }
inline bool is_connected() { return test::connected; }
}
namespace battery { inline std::int32_t get_voltage() { return 12000; } }
class Mutex {
    std::mutex mutex_;
public:
    bool take() { mutex_.lock(); return true; }
    bool give() { mutex_.unlock(); return true; }
};
class Task {
    std::thread thread_;
public:
    template<class F> Task(F function, std::uint32_t, std::uint32_t, const char* name)
        : thread_([function, name] {
            test::estimatorWorker = std::strcmp(name, "nexus-estimator") == 0;
            try { function(); } catch (const test::Shutdown&) {}
        }) {
        test::workers.push_back(&thread_);
    }
    ~Task() { if (thread_.joinable()) thread_.join(); }
    static void delay_until(std::uint32_t* previous, std::uint32_t interval) {
        if (test::estimatorWorker && test::pauseAfterEstimatorCycle.exchange(false))
            test::pauseEstimator = true;
        if (test::estimatorWorker && test::pauseEstimator) {
            test::estimatorPaused = true;
            while (test::pauseEstimator) delay(1);
            test::estimatorPaused = false;
        }
        *previous += interval;
        const auto remaining = static_cast<std::int32_t>(*previous - millis());
        delay(remaining > 0 ? static_cast<std::uint32_t>(remaining) : 0);
    }
};
enum class MotorGearset { blue, green };
enum motor_encoder_units_e_t { E_MOTOR_ENCODER_DEGREES };
enum motor_brake_mode_e_t { E_MOTOR_BRAKE_COAST };
class MotorGroup {
public:
    std::array<std::atomic<bool>, 8> connected{};
    std::array<std::atomic<double>, 8> position{};
    std::array<std::atomic<unsigned>, 8> gearingWrites{}, unitWrites{}, brakeWrites{};
    mutable std::array<std::atomic<unsigned>, 8> positionReads{};
    std::atomic<int> voltage{0};
    std::size_t count;
    explicit MotorGroup(std::initializer_list<std::int8_t> ports) : count(ports.size()) {
        for (std::size_t i = 0; i < count; ++i) connected[i] = true;
    }
    double get_position(std::uint8_t index) const {
        ++positionReads[index];
        return connected[index] ? position[index].load() : PROS_ERR_F;
    }
    int set_gearing(MotorGearset, std::uint8_t index) {
        if (!connected[index]) return PROS_ERR;
        ++gearingWrites[index]; return 1;
    }
    int set_encoder_units(motor_encoder_units_e_t, std::uint8_t index) {
        if (!connected[index]) return PROS_ERR;
        ++unitWrites[index]; return 1;
    }
    int set_brake_mode(motor_brake_mode_e_t, std::uint8_t index) {
        if (!connected[index]) return PROS_ERR;
        ++brakeWrites[index]; return 1;
    }
    int set_encoder_units_all(motor_encoder_units_e_t value) {
        for (std::size_t i = 0; i < count; ++i) set_encoder_units(value, static_cast<std::uint8_t>(i)); return 1;
    }
    int set_brake_mode_all(motor_brake_mode_e_t value) {
        for (std::size_t i = 0; i < count; ++i) set_brake_mode(value, static_cast<std::uint8_t>(i)); return 1;
    }
    int move_voltage(int value) { voltage = value; return 1; }
    int move(int value) { return move_voltage(value * 12000 / 127); }
    int brake() { return move_voltage(0); }
};
constexpr unsigned E_IMU_STATUS_CALIBRATING = 1, E_IMU_STATUS_ERROR = 0xff;
struct Vector { double x, y, z; };
class Imu {
public:
    std::atomic<bool> connected{true};
    std::atomic<double> rotation{0};
    std::array<std::atomic<double>, 3> acceleration{}, rate{};
    mutable std::atomic<int> dataRate{10};
    std::uint32_t get_status() const { return connected ? 4 : E_IMU_STATUS_ERROR; }
    int set_data_rate(int value) const {
        if (!connected) return PROS_ERR;
        dataRate = value; return 1;
    }
    double get_rotation() const { return connected ? rotation.load() : PROS_ERR_F; }
    Vector get_gyro_rate() const { return {rate[0], rate[1], rate[2]}; }
    Vector get_accel() const { return {acceleration[0], acceleration[1], acceleration[2]}; }
};
class Rotation {
    mutable std::mutex stateMutex_;
    std::int64_t directionOffset_ = 0;
public:
    std::atomic<bool> connected{true}, reversed{false};
    // Unreversed physical counter, for injecting movement into the fixture.
    std::atomic<std::int32_t> position{0};
    std::atomic<unsigned> directionWrites{0};
    mutable std::atomic<int> dataRate{10};
    int get_position() const {
        std::lock_guard lock(stateMutex_);
        if (!connected) return PROS_ERR;
        return static_cast<std::int32_t>(static_cast<std::int64_t>(position.load()) *
                                        (reversed ? -1 : 1) + directionOffset_);
    }
    int get_reversed() const {
        std::lock_guard lock(stateMutex_);
        return connected ? static_cast<int>(reversed.load()) : PROS_ERR;
    }
    int set_reversed(bool value) {
        std::lock_guard lock(stateMutex_);
        if (!connected) return PROS_ERR;
        // VEX preserves the current cumulative position when changing the
        // direction at runtime; only subsequent increments change sign.
        // https://www.vexforum.com/t/rotation-sensor-bug-workaround-on-vexos-1-1-0/96577/4
        if (reversed != value)
            directionOffset_ += 2 * static_cast<std::int64_t>(position.load()) * (reversed ? -1 : 1);
        reversed = value; ++directionWrites; return 1;
    }
    int set_data_rate(int value) const {
        std::lock_guard lock(stateMutex_);
        if (!connected) return PROS_ERR;
        dataRate = value; return 1;
    }
    // A reboot loses configuration and initializes a fresh counter. This is
    // deliberately separate from a runtime direction setter; connectivity is
    // controlled independently to also model a reboot between two polls.
    void simulate_reboot(std::int32_t initialPosition) {
        std::lock_guard lock(stateMutex_);
        position = initialPosition;
        directionOffset_ = 0;
        reversed = false;
        dataRate = 10;
    }
};
class Distance {
public:
    std::atomic<int> distance{1000}, confidence{63};
    int get() const { return distance; }
    int get_confidence() const { return confidence; }
};
namespace c {
struct Queue {
    std::mutex mutex;
    std::condition_variable changed;
    std::deque<std::vector<unsigned char>> items;
    std::uint32_t capacity, size;
    Queue(std::uint32_t capacityValue, std::uint32_t sizeValue) : capacity(capacityValue), size(sizeValue) {}
};
using queue_t = Queue*;
inline queue_t queue_create(std::uint32_t length, std::uint32_t size) {
    if (size != 0 && size == test::failQueueCreationOfSize.load()) return nullptr;
    return new Queue(length, size);
}
inline bool queue_append(queue_t queue, const void* item, std::uint32_t) {
    test::check();
    std::lock_guard lock(queue->mutex);
    if (queue->items.size() >= queue->capacity) return false;
    queue->items.emplace_back(queue->size);
    std::memcpy(queue->items.back().data(), item, queue->size);
    if (queue->capacity == 16) ++test::commandEnqueues;
    queue->changed.notify_all(); return true;
}
inline bool receive(queue_t queue, void* item, std::uint32_t timeout, bool remove) {
    test::check();
    if (remove && queue->capacity == 16 && test::pauseCommands) { delay(1); return false; }
    std::unique_lock lock(queue->mutex);
    if (queue->items.empty() && timeout)
        queue->changed.wait_for(lock, std::chrono::milliseconds(timeout), [&] { return !queue->items.empty(); });
    if (remove && queue->capacity == 16 && test::pauseCommands) return false;
    if (queue->items.empty()) return false;
    std::memcpy(item, queue->items.front().data(), queue->size);
    if (remove) queue->items.pop_front();
    return true;
}
inline bool queue_peek(queue_t queue, void* item, std::uint32_t timeout) { return receive(queue, item, timeout, false); }
inline bool queue_recv(queue_t queue, void* item, std::uint32_t timeout) { return receive(queue, item, timeout, true); }
inline void queue_reset(queue_t queue) { std::lock_guard lock(queue->mutex); queue->items.clear(); }
}
}
