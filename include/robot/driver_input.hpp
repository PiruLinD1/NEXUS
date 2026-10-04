#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace robot::detail {
// PROS passes through the VEX connection status, not a normalized 0/1 bool.
// Accept positive connection states, but never treat PROS_ERR as connected.
inline bool controllerIsConnected(std::int32_t status) {
    return status > 0 && status != std::numeric_limits<std::int32_t>::max();
}

// One controller snapshot per driver tick. Invalid reads never count as presses.
template <std::size_t Count> class ButtonState {
public:
    void reset(const std::array<int, Count>& values) {
        known_.fill(false);
        update(values);
    }
    void update(const std::array<int, Count>& values) {
        for (std::size_t i = 0; i < Count; ++i) {
            const bool valid = values[i] == 0 || values[i] == 1;
            const bool down = valid && values[i] == 1;
            pressed_[i] = valid && known_[i] && down && !held_[i];
            held_[i] = down;
            known_[i] = valid;
        }
    }
    bool held(std::size_t index) const { return index < Count && held_[index]; }
    bool pressed(std::size_t index) const { return index < Count && pressed_[index]; }
    int power(std::size_t forward, std::size_t reverse) const {
        return 127 * (static_cast<int>(held(forward)) - static_cast<int>(held(reverse)));
    }
private:
    std::array<bool, Count> known_{}, held_{}, pressed_{};
};
} // namespace robot::detail
