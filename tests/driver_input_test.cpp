#include "robot/driver_input.hpp"
#include <cstdlib>
#include <iostream>
#include <limits>

namespace {
void require(bool condition, const char* message) {
    if (!condition) { std::cerr << "FAILED: " << message << '\n'; std::exit(1); }
}
}

int main() {
    using robot::detail::controllerIsConnected;
    require(controllerIsConnected(1), "connection status 1 enables driver input");
    require(controllerIsConnected(2), "connection status 2 must not disable all driver input");
    require(!controllerIsConnected(0), "disconnected controller does not enable input");
    require(!controllerIsConnected(std::numeric_limits<std::int32_t>::max()),
            "PROS_ERR is not a connected controller despite being positive");
    require(!controllerIsConnected(-1), "negative invalid status does not enable input");

    robot::detail::ButtonState<12> buttons;
    std::array<int, 12> values{};
    buttons.reset(values);
    values[0] = 1;
    buttons.update(values);
    require(buttons.held(0) && buttons.pressed(0), "press triggers both held and rising edge");
    require(buttons.pressed(0), "one press can be read by multiple bindings in the same tick");
    require(buttons.power(0, 1) == 127, "forward button drives at positive power");
    buttons.update(values);
    require(buttons.held(0) && !buttons.pressed(0), "holding a toggle does not retrigger it");
    values[1] = 1;
    buttons.update(values);
    require(buttons.power(0, 1) == 0, "opposing buttons held together stop mechanism");
    values[0] = 0;
    buttons.update(values);
    require(buttons.power(0, 1) == -127, "reverse button drives at negative power");
    values[1] = 0;
    buttons.update(values);
    require(buttons.power(0, 1) == 0, "releasing both buttons stops mechanism");
    values[0] = 1;
    buttons.update(values);
    require(buttons.pressed(0), "a new press after release toggles again");

    // Changing a toggle to any other button must preserve entry behavior.
    values.fill(1);
    buttons.reset(values);
    for (std::size_t i = 0; i < values.size(); ++i)
        require(buttons.held(i) && !buttons.pressed(i), "driver entry does not toggle already held buttons");
    values.fill(0);
    buttons.update(values);
    values.fill(1);
    buttons.update(values);
    for (std::size_t i = 0; i < values.size(); ++i)
        require(buttons.pressed(i), "all remapped buttons detect fresh presses after entry");

    values.fill(std::numeric_limits<int>::max()); // PROS_ERR / disconnected controller.
    buttons.update(values);
    for (std::size_t i = 0; i < values.size(); ++i)
        require(!buttons.held(i) && !buttons.pressed(i), "controller errors are not button presses");
    require(buttons.power(0, 1) == 0, "invalid readings command zero power");
    values.fill(1);
    buttons.update(values);
    require(buttons.held(0) && !buttons.pressed(0), "reconnect with held button does not trigger toggle");
    values.fill(0);
    buttons.update(values);
    values[0] = 1;
    buttons.update(values);
    require(buttons.pressed(0), "fresh press works after controller reconnect");
    require(!buttons.held(12) && !buttons.pressed(static_cast<std::size_t>(-1)),
            "invalid button indices are ignored");
    std::cout << "Driver input: connection states 1/2, disconnection/errors, held, toggles, remapping, opposing inputs and reconnect passed.\n";
}
