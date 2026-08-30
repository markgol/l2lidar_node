//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: quaternion.h
//
//  Purpose:
//  Define the point cloud structure for a point
//
//  V0.3.6  2026-01-24  Added IMU orientation to point cloud
//  V1.3.4  2026-06-15  Moved structure defintions to here
//
//--------------------------------------------------------
#pragma once

// ---- Lidar point ----
#include <cstdint>

struct PCpoint
{
    float x; //meters
    float y; //meters
    float z; //meters
    float intensity; //  0-255
    float range;    // meters
                    // V1.3.5 or earlier are L2 raw units
                    // V1..3.6 or later are calibrated units
    float raw_range;  // meters, uncalibrated range reported by L2
    long long time; // timestamp in nanoseconds
    uint32_t ring;
};

