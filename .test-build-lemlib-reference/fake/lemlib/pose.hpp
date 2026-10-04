#pragma once
// Minimal host definition of upstream Pose. Operations used by odom.cpp retain
// the upstream float arithmetic, including operator* preserving theta.
// Copyright (c) 2024 Liam Teale and LemLib contributors; see ../../LICENSE.
namespace lemlib {
struct Pose {
    float x, y, theta;
    Pose(float x, float y, float theta = 0) : x(x), y(y), theta(theta) {}
    Pose operator*(const float& scale) const { return Pose(x * scale, y * scale, theta); }
};
}
