#pragma once

// ---- Lidar point ----
#include <cstdint>

struct PCpoint
{
    float x;
    float y;
    float z;
    float intensity; //  0-255
    float range;    // meters
    long long time; // timestamp in nanoseconds
    uint32_t ring;
};

