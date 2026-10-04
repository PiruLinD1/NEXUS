#pragma once
namespace pros {
class Task { public: template<class F> explicit Task(F&&) {} };
inline void delay(unsigned) {}
class Imu { public: double degrees = 0; double get_rotation() { return degrees; } };
}
