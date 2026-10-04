#pragma once
// These three helper definitions follow the pinned upstream util.hpp/util.cpp.
// Copyright (c) 2024 Liam Teale and LemLib contributors; see ../../LICENSE.
namespace lemlib {
constexpr float radToDeg(float rad) { return rad * 180 / 3.14159265358979323846; }
constexpr float degToRad(float deg) { return deg * 3.14159265358979323846 / 180; }
inline float ema(float current, float previous, float smooth) {
    return (current * smooth) + (previous * (1 - smooth));
}
}
