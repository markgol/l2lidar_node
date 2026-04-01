//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: quaternion.h
//
//  Purpose:
//  Determine correct operation of the Unitreee L2 Lidar hardware
//  and software.  Establish platform independent software protocols
//  for using the L2 Lidar with its Ethernet interface.
//
//  Background:
//  Unitree provides undoucmented software files in the form:
//      include files
//      example application files
//      .a Archive Library
//
//  The source files rely on an Archive library using POSIX I/O
//  No source exists for the archive Library making it diffcult
//  to debug or port usage of the L2 Lidar for other platforms.
//  The hardware has 2 mutually exclusive communication interfaces:
//      Ethernet using UDP
//      Serial UART
//  The serial UART is limited in speed and does not operate at
//  the full sensor speed of 64K/sec sample points.
//
//  Solution:
//  This software skeleton was created using directed ChatGPT AI
//  conversation targetting a QT Creator development platform.
//  It reads UPD packets from the L2, caterorizes them, performs
//  error detection for bad packets (lost), display subsample
//  of packets.
//
//  V0.3.6  2026-01-24  Added IMU orientation to point cloud
//
//--------------------------------------------------------

#pragma once

// struct for rotation alogrithm
struct Quaternion
{
    float w, x, y, z;
};


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
