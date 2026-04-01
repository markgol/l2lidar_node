//**********************************************************************
//  Copyright (c) 2020-2024, Unitree Robotics.Co.Ltd. All rights reserved.
//
// BSD 3-Clause License
//
// Copyright (c) 2024, Unitree Robotics
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// **********************************************************************

//------------------------------------------------
//
//  The following modifcations/clarifications have been made to the original
//  source file from unitree
//
//  date: 2025-01-24
//  Author: Mark Stegall
//  no claim is made to any portion of this source file
//
//      Corrected @notes packet size in bytes
//          LidarImuData structure size is 56
//          LidarImuDataPacket size is 80
//          LidarIpAddressConfig 20
//          LidarIpAddressConfigPacket 44
//          LidarPointData 1020
//          Lidar2DPointDataPacket 1044
//          Lidar2DPointData 5512
//          Lidar2DPointDataPacket 5536
//          udated note on crc32, it is calculated on just data
//              excluding header and tail
//          Added workmode bit definitions
//          Added set workmode cmd type 2002
//          Added MAX_3DPOINTS_PER_FRAME
//          Added MAX_2DPOINTS_PER_FRAME
//      Added LidarTimeStampData structure to be consistent
//          with other data packet structures
//      Added LIDAR COMMAND PACKET definitions
//      Added LIDAR PARAMETER DATA PACKET definitions
//      Removed unused #includes
//
//------------------------------------------------
//
// When including this file you must do the following:
//  THIS IS REQUIRED
//  #pragma pack(push, 1)
//  #include "unitree_lidar_protocolL2.h"
//  #pragma pack(pop)
//
//------------------------------------------------

#ifndef UNITREE_LIDAR_PROTOCOL_HEADER
#define UNITREE_LIDAR_PROTOCOL_HEADER

#include <stdint.h>

///////////////////////////////////////////////////////////////////////////////
// MACRO DEFINITION
///////////////////////////////////////////////////////////////////////////////

#define FRAME_HEADER_ARRAY_0 0x55
#define FRAME_HEADER_ARRAY_1 0xAA
#define FRAME_HEADER_ARRAY_2 0x05
#define FRAME_HEADER_ARRAY_3 0x0A

#define FRAME_TAIL_ARRAY_0 0x00
#define FRAME_TAIL_ARRAY_1 0xFF

#define LIDAR_USER_CMD_PACKET_TYPE 100
#define LIDAR_ACK_DATA_PACKET_TYPE 101
#define LIDAR_POINT_DATA_PACKET_TYPE 102
#define LIDAR_2D_POINT_DATA_PACKET_TYPE 103
#define LIDAR_IMU_DATA_PACKET_TYPE 104
#define LIDAR_VERSION_PACKET_TYPE 105
#define LIDAR_TIME_STAMP_PACKET_TYPE 106
#define LIDAR_WORK_MODE_CONFIG_PACKET_TYPE 107
#define LIDAR_IP_ADDRESS_CONFIG_PACKET_TYPE 108
#define LIDAR_MAC_ADDRESS_CONFIG_PACKET_TYPE 109

#define LIDAR_COMMAND_PACKET_TYPE 2000
#define LIDAR_PARAM_DATA_PACKET_TYPE 2001
#define LIDAR_PARAM_WORK_MODE_TYPE 2002    // sets the workmode (requires restart)

#define CMD_RESET_TYPE 1
#define CMD_PARAM_SAVE 2
#define CMD_PARAM_GET 3
#define CMD_VERSION_GET 4
#define CMD_STANDBY_TYPE 5
#define CMD_LATENCY_TYPE 6
#define CMD_CONFIG_RESET 7

#define USER_CMD_RESET_TYPE 1
#define USER_CMD_STANDBY_TYPE 2     // value 0: start; value 1: standby
#define USER_CMD_VERSION_GET 3
#define USER_CMD_LATENCY_TYPE 4
#define USER_CMD_CONFIG_RESET 5
#define USER_CMD_CONFIG_GET 6
#define USER_CMD_CONFIG_AUTO_STANDBY 7

#define ACK_SUCCESS 1
#define ACK_CRC_ERROR 2
#define ACK_HEADER_ERROR 3
#define ACK_BLOCK_ERROR 4
#define ACK_WAIT_ERROR 5    // data is not ready

// WORKMODE bit definitions
#define WORK_MODE_WFOV          1  // 0: Standard FOV (180°), 1: Wide-angle FOV (192°)
#define WORK_MODE_2D            2  // 0: 3D measurement mode, 2D measurement mode
#define WORK_MODE_IMU_DISABLE   4  // 0: Enable IMU, 1: Disable IMU
#define WORK_MODE_SERIAL        8  // 0: Ethernet mode, 1: Serial mode
#define WORK_MODE_STARTUP_WAIT  16 // 0: Power on and start automatically,
                                   // 1: Power on and wait for start command without rotation

// buffer sizes
#define MAX_3DPOINTS_PER_FRAME  300
#define MAX_2DPOINTS_PER_FRAME  1800
///////////////////////////////////////////////////////////////////////////////
// FRAME HEADER & FRAME TAIL
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Frame Header
 * @note 12 bytes
 */
typedef struct
{
    uint8_t header[4];          // Head: 0x55 0xAA 0x05 0x0A
    uint32_t packet_type;       // packet type
    uint32_t packet_size;       // packet size - total bytes of the whole packet
}FrameHeader;

/**
 * @brief Frame Tail
 * @note 12 bytes
 */
typedef struct
{
    uint32_t crc32;               // crc check of data (excludes header and footer)
    uint32_t msg_type_check;      // msg ack for lidar
    uint8_t reserve[2];           // reserve
    uint8_t tail[2];              // Tail: 0x00 0xFF
}FrameTail;

///////////////////////////////////////////////////////////////////////////////
// TIMESTAMP & DATA INFO
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Time stamp
 * @note 8 bytes
 */
typedef struct
{
    uint32_t sec;       // time stamp of second
    uint32_t nsec;      // time stamp of nsecond
}TimeStamp;

/**
 * @brief Data Info
 * @note 16 bytes
 */
typedef struct
{
    uint32_t seq;           // packet sequence id, consecutively increasing
    uint32_t payload_size;  // Packet Size
    TimeStamp stamp;        // timestamp
}DataInfo;

/**
 * @brief Lidar calib param
 */
typedef struct
{
    float a_axis_dist;                      // unit: m
    float b_axis_dist;                      // unit: m
    float theta_angle_bias;                 // unit: rad
    float alpha_angle_bias;                 // unit: rad
    float beta_angle;                       // unit: rad
    float xi_angle;                         // unit: rad
    float range_bias;                       // unit: mm
    float range_scale;                      // unit: 1
}LidarCalibParam;

/**
 * @brief Lidar Inside State
 */
typedef struct
{
    uint32_t sys_rotation_period;       // Up motor rotation period
    uint32_t com_rotation_period;       // Down motor rotation period
    float dirty_index;
    float packet_lost_up;
    float packet_lost_down;
    float apd_temperature;
    float apd_voltage;
    float laser_voltage;
    float imu_temperature;
}LidarInsideState;

///////////////////////////////////////////////////////////////////////////////
// LIDAR POINT DATA PACKET
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Lidar Point Data
 * @note 1020 bytes
 */
typedef struct 
{
    // Packet Info
    DataInfo info;

    // Lidar inside state
    LidarInsideState state;
    
    // Lidar calib param
    LidarCalibParam param;

    // Line info
    float com_horizontal_angle_start;   // Horizontal Start Angle
    float com_horizontal_angle_step;    // Horizontal Angle Step
    float scan_period;                  // Scan period [second]
    float range_min;                    // Minimum range value [m]
    float range_max;                    // Maximum range value [m]
    float angle_min;                    // First Angle [rad]
    float angle_increment;              // Angle Step [rad]
    float time_increment;               // Time step [second]
    uint32_t point_num;                 // Point Number
    uint16_t ranges[MAX_3DPOINTS_PER_FRAME];      // Point Distance [mm]
    uint8_t intensities[MAX_3DPOINTS_PER_FRAME];  // Point Reflect [0-255]
}LidarPointData;

/**
 * @brief Lidar Point Data Packet
 * @note 1044 bytes
 */
typedef struct
{
    FrameHeader header;
    LidarPointData data;
    FrameTail tail;
}LidarPointDataPacket;

///////////////////////////////////////////////////////////////////////////////
// LIDAR 2D POINT DATA PACKET
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Lidar 2D Point Data
 * @note 5512 bytes
 */
typedef struct 
{
    // Packet Info
    DataInfo info;

    // Lidar inside state
    LidarInsideState state;
    
    // Lidar calib param
    LidarCalibParam param;

    // Line info
    float scan_period;                  // scan period [second]
    float range_min;                    // minimum range value [m]
    float range_max;                    // maximum range value [m]
    float angle_min;                    // First Angle [rad]
    float angle_increment;              // Angle Step [rad]
    float time_increment;               // point time step
    uint32_t point_num;                 // Point Number
    uint16_t ranges[MAX_2DPOINTS_PER_FRAME];     // Point Distance Data
    uint8_t intensities[MAX_2DPOINTS_PER_FRAME]; // Point Reflect Data
}Lidar2DPointData;

/**
 * @brief Lidar Point Data Packet
 * @note 5536 bytes
 */
typedef struct
{
    FrameHeader header;
    Lidar2DPointData data;
    FrameTail tail;
}Lidar2DPointDataPacket;


///////////////////////////////////////////////////////////////////////////////
// TIME STAMP PACKET
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Lidar Time Stamp
 * @note 8 bytes
 * This is to be consistent with other packet data structures
 */
typedef struct
{
    TimeStamp data;
}LidarTimeStampData;

/**
 * @brief Lidar Time Stamp Packet
 * @note 32 bytes
 */
typedef struct
{
    FrameHeader header;
    LidarTimeStampData data;
    FrameTail tail;
}LidarTimeStampPacket;

///////////////////////////////////////////////////////////////////////////////
// IMU DATA PACKET
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Lidar IMU Data
 * @note 56 bytes
 */
typedef struct 
{
    DataInfo info;
    float quaternion[4];             // Quaternion Array.
    float angular_velocity[3];       // Three-axis angular velocity values.
    float linear_acceleration[3];    // Three-axis acceleration values.
}LidarImuData;

/**
 * @brief Lidar IMU Data Packet
 * @note 80 bytes
 */
typedef struct
{
    FrameHeader header;
    LidarImuData data;
    FrameTail tail;
}LidarImuDataPacket;

///////////////////////////////////////////////////////////////////////////////
// ACK PACKET
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief ACK
 * @note Lidar will respond with an ack packet if it receive a packet from user
 * @note 16 bytes
 */
typedef struct 
{
    uint32_t packet_type;   // packet type received by lidar
    uint32_t cmd_type;      // cmd type received by lidar
    uint32_t cmd_value;     // cmd value received by lidar
    uint32_t status;        // execute result
}LidarAckData;

/**
 * @brief ACK command Packet
 * @note 40 bytes
 */
typedef struct
{
    FrameHeader header;
    LidarAckData data;
    FrameTail tail;
}LidarAckDataPacket;

///////////////////////////////////////////////////////////////////////////////
// LIDAR VERSION PACKET
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Lidar version
 * @note 80 bytes
*/
typedef struct 
{
    uint8_t hw_version[4];  // hardware version
    uint8_t sw_version[4];  // software version
    uint8_t name[24];       // device name
    uint8_t date[8];        // device compile date
    uint8_t reserve[40];    // Product name string
}LidarVersionData;

/**
 * @brief Lidar version
 * @note 104 bytes
*/
typedef struct
{
    FrameHeader header;
    LidarVersionData data;
    FrameTail tail;
}LidarVersionDataPacket;

/////////////////////////////////////////////////////////////////////////
// IP CONFIG PACKET
/////////////////////////////////////////////////////////////////////////

/**
 * @brief Lidar IP Config
 * @note 20 bytes
 */
typedef struct 
{
    uint8_t lidar_ip[4];                   // UDP local ip
    uint8_t user_ip[4];                    // UDP remote ip
    uint8_t gateway[4];                    // Gate way
    uint8_t subnet_mask[4];                // Subnet mask
    uint16_t lidar_port;                   // UDP local port
    uint16_t user_port;                    // UDP remote port
}LidarIpAddressConfig;

/**
 * @brief Lidar IP config packet
 * @note 44 bytes
 */
typedef struct
{
    FrameHeader header;
    LidarIpAddressConfig data;
    FrameTail tail;
}LidarIpAddressConfigPacket;

/////////////////////////////////////////////////////////////////////////
// MAC ADDRESS CONFIG PACKET
/////////////////////////////////////////////////////////////////////////

/**
 * @brief Lidar MAC address Config
 * @note 8 bytes
 */
typedef struct 
{
    uint8_t mac[6];
    uint8_t reserve[2];
}LidarMacAddressConfig;

/**
 * @brief Lidar MAC address config packet
 * @note 32 bytes
 */
typedef struct
{
    FrameHeader header;
    LidarMacAddressConfig data;
    FrameTail tail;
}LidarMacAddressConfigPacket;

/////////////////////////////////////////////////////////////////////////
// WORK MODE PACKET
/////////////////////////////////////////////////////////////////////////

/**
 * @brief Lidar Work Mode
* @note 4 bytes
*/
typedef struct
{
    uint32_t mode;

    // Bit 0: Switch between standard FOV and wide-angle FOV
    // 0: Standard FOV (180°)
    // 1: Wide-angle FOV (192°)

    // Bit 1: Switch between 3D and 2D measurement modes
    // 0: 3D measurement mode
    // 1: 2D measurement mode

    // Bit 2: Enable or disable IMU
    // 0: Enable IMU
    // 1: Disable IMU

    // Bit 3: Switch between Ethernet mode and serial mode
    // 0: Ethernet mode
    // 1: Serial mode

    // Bit 4: Switch between lidar power-on default start mode
    // 0: Power on and start automatically
    // 1: Power on and wait for start command without rotation

}LidarWorkModeConfig;

/**
 * @brief Lidar Work Mode Packet
* @note 28 bytes
*/
typedef struct
{
    FrameHeader header;
    LidarWorkModeConfig data;
    FrameTail tail;
}LidarWorkModeConfigPacket;

/////////////////////////////////////////////////////////////////////////
// USER CONTROL COMMAND PACKET
/////////////////////////////////////////////////////////////////////////

/**
 * @brief Lidar User Control Command
 * @note 8 bytes
 */
typedef struct 
{
    uint32_t cmd_type; //   0:null, 1:standby
    uint32_t cmd_value;
}LidarUserCtrlCmd;

/**
 * @brief Lidar User Control Command Packet
 * @note 32 bytes
 */
typedef struct
{
    FrameHeader header;
    LidarUserCtrlCmd data;
    FrameTail tail;
}LidarUserCtrlCmdPacket;

/////////////////////////////////////////////////////////////////////////
// LIDAR PARAMETER DATA PACKET
/////////////////////////////////////////////////////////////////////////

/**
 * @brief Lidar Parameter Data
 * @note 172 bytes
 */
typedef struct
{
    uint8_t Unkown1[4]; //uknown
    uint16_t Short1;
    uint16_t Short2;
    uint16_t Short3;
    uint16_t Short4;

    uint32_t workmode;
    uint32_t ParamInt1;
    uint32_t ParamInt2;
    float ParamFloat1;

    float RangeMax;
    float RangeMin;
    float ParamFloat2;

    LidarCalibParam Calib3DstdFOV;
    LidarCalibParam Calib3DwideFOV;
    LidarCalibParam Calib2DstdFOV;
    LidarCalibParam Calib2DwideFOV;
    uint8_t Unkown2[4]; //unknown

}LidarParamData;

/**
 * @brief Lidar Parameter Data Packet
 * @note 196 bytes
 */
typedef struct
{
    FrameHeader header;
    LidarParamData data;
    FrameTail tail;
}LidarParamDataPacket;

#endif

