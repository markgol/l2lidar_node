//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: quaternion.h
//
//  Purpose:
//  Add Quaternion calculation methods used in the application software
//
//  V0.3.6  2026-01-24  Added IMU orientation to point cloud
//  V1.3.4  2026-06-15  Added RemoveYaw() for pose correction
//                        using only roll, pitch
//                      Added normalizeQuaternion()
//                      Moved structure defintions to here
//                      Added EulerAnglesRad2Deg(), EulerAnglesDeg2Rad
//
//--------------------------------------------------------

//--------------------------------------------------------
// GPL-3.0 license
//
// This file is part of L2diagnsotic.
//
// L2diagnsotic is free software : you can redistribute it and /or modify it under
// the terms of the GNU General Public License as published by the Free Software Foundation,
// either version 3 of the License, or (at your option) any later version.
//
// L2diagnsotic is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with L2diagnsotic.
// If not, see < https://www.gnu.org/licenses/>.
//--------------------------------------------------------

#pragma once
#include <cmath>

//--------------------------------------------------------
// defines that don't rely on includes
//--------------------------------------------------------
#define PI 3.14159265358979323846
#define RAD_TO_DEG (180.0 / PI)
#define DEG_TO_RAD (PI / 180.0)

//--------------------------------------------------------
// structure definitions
//--------------------------------------------------------
struct Quaternion
{
    double w, x, y, z;
};

struct EulerAngles
{
    double roll;
    double pitch;
    double yaw;
};

//--------------------------------------------------------
//
//--------------------------------------------------------
inline void EulerAnglesRad2Deg(EulerAngles& e)
{
    e.roll  *= RAD_TO_DEG;
    e.pitch *= RAD_TO_DEG;
    e.yaw   *= RAD_TO_DEG;
    return;
}

//--------------------------------------------------------
//
//--------------------------------------------------------
inline void EulerAnglesDeg2Rad(EulerAngles& e)
{
    e.roll  *= DEG_TO_RAD;
    e.pitch *= DEG_TO_RAD;
    e.yaw   *= DEG_TO_RAD;
    return;
}

//--------------------------------------------------------
//  rotateByQuaternion
//--------------------------------------------------------
inline void rotateByQuaternion(const Quaternion& q,
                                 float& x, float& y, float& z)
{
    // v' = q * v * q_conjugate
    const float vx = x, vy = y, vz = z;

    // q * v
    const float qw = -q.x*vx - q.y*vy - q.z*vz;
    const float qx =  q.w*vx + q.y*vz - q.z*vy;
    const float qy =  q.w*vy + q.z*vx - q.x*vz;
    const float qz =  q.w*vz + q.x*vy - q.y*vx;

    // (q * v) * q_conjugate
    x = -qw*q.x + qx*q.w - qy*q.z + qz*q.y;
    y = -qw*q.y + qy*q.w - qz*q.x + qx*q.z;
    z = -qw*q.z + qz*q.w - qx*q.y + qy*q.x;
}

//--------------------------------------------------------
//  removeYaw
//--------------------------------------------------------
inline Quaternion removeYaw(const Quaternion& q)
{
    // Extract yaw, pitch, roll
    const float sinroll_cospitch = 2.0f * (q.w*q.x + q.y*q.z);
    const float cosroll_cospitch = 1.0f - 2.0f * (q.x*q.x + q.y*q.y);
    const float roll = std::atan2(sinroll_cospitch, cosroll_cospitch);

    const float sinpitch = 2.0f * (q.w*q.y - q.z*q.x);
    const float pitch = std::abs(sinpitch) >= 1.0f ?
                            std::copysign(static_cast<float>(PI) / 2.0f, sinpitch) :
                            std::asin(sinpitch);

    // Reconstruct quaternion with yaw = 0
    const float cr = std::cos(roll * 0.5f);
    const float sr = std::sin(roll * 0.5f);
    const float cp = std::cos(pitch * 0.5f);
    const float sp = std::sin(pitch * 0.5f);

    Quaternion qrollpitch;
    qrollpitch.w = cr * cp;
    qrollpitch.x = sr * cp;
    qrollpitch.y = cr * sp;
    qrollpitch.z = -sr * sp;

    return qrollpitch;
}

//--------------------------------------------------------
//  normalizeQuaternion
//--------------------------------------------------------
inline void normalizeQuaternion(Quaternion& q)
{
    // magnitude is sum of squares
    const float magnitude =
        q.w * q.w +
        q.x * q.x +
        q.y * q.y +
        q.z * q.z;
    if (magnitude > 0.0f) {
        // scale by inverse of magnitude 1/sqrt(magnitude)
        const float invMagnitude = 1.0f / std::sqrt(magnitude);
        q.w *= invMagnitude;
        q.x *= invMagnitude;
        q.y *= invMagnitude;
        q.z *= invMagnitude;
    } else {
        // magnitude not valid
        // set to roll, pitch, yaw = 0,0,0
        q.w = 1.0f;
        q.x = q.y = q.z = 0.0f;
    }
}

//--------------------------------------------------------
//
//--------------------------------------------------------
inline EulerAngles QuaternionToEuler( const Quaternion& q,
                                      bool outputDegrees = false)
{
    // Normalize quaternion
    double norm = std::sqrt(
        q.x * q.x +
        q.y * q.y +
        q.z * q.z +
        q.w * q.w);

    if (norm <= 0.0)
    {
        return {0.0, 0.0, 0.0};
    }

    double x = q.x / norm;
    double y = q.y / norm;
    double z = q.z / norm;
    double w = q.w / norm;

    EulerAngles e;

    // Roll (X-axis rotation)
    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    e.roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (Y-axis rotation)
    double sinp = 2.0 * (w * y - z * x);
    if (std::abs(sinp) >= 1.0)
        e.pitch = std::copysign(PI / 2.0, sinp);
    else
        e.pitch = std::asin(sinp);

    // Yaw (Z-axis rotation)
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    e.yaw = std::atan2(siny_cosp, cosy_cosp);

    if (outputDegrees)
    {
        EulerAnglesRad2Deg(e);
    }

    return e;
}

