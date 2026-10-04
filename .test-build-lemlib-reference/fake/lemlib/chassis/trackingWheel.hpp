#pragma once
namespace lemlib {
struct TrackingWheel {
    float value = 0, offset = 0;
    int type = 0;
    float getDistanceTraveled() { return value; }
    float getOffset() { return offset; }
    int getType() { return type; }
};
}
