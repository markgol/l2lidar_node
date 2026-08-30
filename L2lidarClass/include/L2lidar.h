//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: L2lidar.h
//
//  Purpose:
//
//  To provide necesary software interfaces to control and
//  receive and correct data for the Unitree L2 LiDAR.
//
//  This is to support applications such as diagnotic, point cloud viewer
//  and ROS2 interfaces to the L2
//
//  Background:
//      Unitree provides marginally documented software files
//      in the form:
//          include files (open source)
//          example application files (open source)
//          .a Archive Library (proprietary)
//
//      The source files rely on an Archive library using POSIX I/O
//      No source exists for the archive Library making it diffcult
//      to debug or port usage of the L2 Lidar for other platforms.
//
//      The hardware has 2 mutually exclusive communication interfaces:
//          Ethernet using UDP
//          Serial UART
//
//  Observations:
//      The L2 has both documented and undocumented packet used to transmit
//      and receive data with the L2 and a host computer.  Builtin calibration
//      calibration values are not inconsistent at minimizing the distortion
//      in the converted point cloud.  The IMU does not return reliable
//      angles.  The L2 has considerable gyroscopic induced vibration that
//      requires careful mounting in order to minimize its affect on both
//      point cloud and IMU data.  The L2 has a timebase that has been
//      measured at approx 1/2 realtime.
//
//  Current status:
//      Implementation of class verified using UDP interface only.
//      Serial UART impementation is being explored but not included.
//      Working on integration and use of this class in support of ROS2
//      as substitute for Unitree's SDK proprietary archive library.
//      Implements a calibration strategy to correct range data and
//      override builtin calibration values.
//
//  V0.1.0  2025-12-27  compilable skeleton created by ChatGPT
//  V0.2.0  2026-01-02  Documentation, start of debugging
//                      CRC32normal() added to unitree_lidar_utilies.h
//                      implementation of LidarDecoder
//  V0.2.1  2026-01-05  Changed LidarDecoder.h and cpp to L2lidar
//                      Changed class name from LidarDecoer to L2lidar
//                      Added USER commands to control L2 lidar
//                      Updated @notes for unitree_lidar_protocols.h
//                      Consolidated all UDP operations into this class
//                      CRC32normal() normal removed from unitree_lidar_utilies.h
//  V.2.2   2026-01-08  Added Mutex access to packet copies
//  V0.3.4  2026-01-23  Changed processingDatagram() to process multiple
//                      UDP datagrams into one L2 Lidar packet
//  V0.3.6  2026-01-26  Added quaternion spatial correction routine
//                      Added Serial UART support
//  V0.3.7  2026-01-28  Documentation updates
//                      Minor bug corrections
//                      Added Set UPD configuration in the L2
//                      Added send Latency command packet
//                      Added requestLatencyMeasurement(), note this is rtt latency
//                          This is non-blocking.
//  V0.3.8  2026-01-29  Refined latency mesurements and class interface to them
//  V0.3.9  2026-01-30  Added
//                          SyncL2clock() // syncs to the host timestamp
//                          SyncL2clock(TimeStamp)
//                          EnableL2TimeCorrection(enableflag);
//                          SetL2TimeScale(Scale)
//                          GetL2TimeScale()
//  V0.3.10 2026-02-01  Added Get L2 Parameters
//                      Added GetWorkmode()
//                      Added enable latency measurement flag
//  V0.3.11 2026-02-04  Added void ConvertL2data2pointcloud()
//                      to return just actual point cloud
//                      frame instead of entire unprocessed packet
//  V0.4.1  2026-02-11  Added Set MAC command
//                      Added decode for the 3 config packets, MAC, workmode, IPaddress
//                      Sorted alphabetically in groups for public class members
//  V0.4.2  2026-02-13  Added error string for communication connect failure
//                      or send error
//  V0.4.3  2026-02-16  Added more logic to the timestamping of the point cloud data
//                          mL2EnableSyncHost && mEnableL2TimeStampFix
//                              true  use L2 timestamping for each cloud point
//                              false use system time for each cloud point
//                      Dump first 100 frames of IMU and point cloud after connect
//                          It takes a some time for the first sync to host to occur
//                          So it will return wrong time in the initial IMU and point
//                          cloud packets
//                      Added more packet stats to help track if app is keeping up
//                      with packet rate
//                      Added range(m) to point cloud data.  It is already present
//                      in the raw point cloud packet.  It saves recomputing it later
//                      in a user app.  PCpoint.h has been changed to include this field.
//  V1.0.0  2026-02-20  Separated L2lidar class from the L2diagnostic app and l2lidar_ros2 app
//                      This is the initial release of the standalone L2lidar class
//                      Changed the unitree_lidar_utilities.h,  the parse function for the
//  V1.1.0  2026-02-22  Corrected ConvertL2data2pointcloud() to generate more accurate timestamps.
//                      Changed the timestamp units in the cloud point array returned by
//                      ConvertL2data2pointcloud().
//                      PCpoint structure member time change from float to long long
//                      PCpoint structure member time units are now nanoseconds since Epoch
//
//  V1.2.0  2026-04-19  Added range calibration overrides to:
//                          parseFromPacketToPointCloud()
//                          parseFromPacketPointCloud2D()
//  V1.3.0  2026-05-12  Changed the SetL2TimeScale() and GetL2TimeScale() to use long long numerator,
//                      long long denominator for time scaling instead of a double.
//                      Changed time corrections to use only long long arithmetic instead of
//                      double.  This preserves precision of the timestamps with
//                      minimal numerical loss
//  V1.3.2  2026-05-24  Corrected fixed IMU to point cloud packet timing constraint.
//                      This is now a settable parameter.
//  V1.3.3  2026-05-30 Corrected bug in timestamp correction introduced in V1.3.0
//                      The IMU timestamp was incorrectly being calculated when
//                      the fix time stamp was enabled.
//  V1.3.4  2026-06-15  Added adjust RollPitch only flag to IMUadjust correction to:
//                          ConvertL2data2pointcloud()
//                      Removed ConvertL2data2pointcloud() use of System Now time
//                      Added SetUseSystemNowTimestamps() and GetUseSystemNowTimestamps().
//                          if true it substitutes systemnow time for the packet timestamp
//                            when a packet is recieved and decoded.
//                          It is recommended that sync to host and timestamp correction be used instead.
//                          This is added to emulate what the L2 archive library does with timestamps.
//                      Added Quaternion normalization before pose correction.
//                      Refactored quaternion and euler methods into quaternion.h
//                      Corrected initialization and reinitialization of time stamp correction
//                          for various start, restart, connect and disconnect conditions
//                          using conditional logic based on the settings of host to L2 timestamp syncing
//                          and time stamp correction flags.
//                      SetL2TimeScale() changed to a bool return.  Flags invalid settings of
//                          numerator, denominator.  Timestamp correction will not be performed
//                          until valid values are set.
//                          Invalid: numerator<=denominator, numerator<=0, denominator<=0,
//                                   numberator/denominator<1.5, numerator/denominator>3.0
//                      Corrected FixTimestamp to handle various startup conditions correctly
//                          particularly after L2disconnect and L2connect when sync to host setting
//                          have changed.
//  V1.3.5  2026-06-21  Added optional parameter for gateway IP address and subnet mask in setL2UDPconfig()
//  V1.3.6  2026-07-11  Changed PCpoint to include both calibrated range and raw range value.
//                      Changed return of range value from parseFromPacketToPointCloud()
//                          and parseFromPacketPointCloud2D() to be actual range from L2.
//                          The raw range value from the L2 is returned in a PCpoint field called raw_range.
//                      Complimentary changes also made in the unitree_lidar_utilitiesL2.h file
//                      Added SelectiveParseFromPacketToPointCloud() for 3d scans
//                          This is not part of the original unitree_lidar_utilities.h SDK
// V2.0.0RC1 2026-08-18 Adding range calibration class to be used in the L2lidar class
//                          This will align with L2diagnostics V2.0.0
//                          It will only include the application of the range correction methods
//                          It does not include the creation and calibration procedures
//                            that generates the calibration dataset used in the application of
//                            range correction methods.
//                      Updated some of the private class variables to start with m...
//                      Update to the API involving the calibration override settings
//                          Moved these into the L2lidarClass
//                          Override internal L2 biases for:
//                              Range Bias, Range Scale
//                              ThetaAngle Bias, Alpha Angle Bias,
//                              Beta angle, Xi angle
//                          API interface for the 3d point cloud parser
//                              ConvertL2data2pointcloud()
//                          Updates for 2d scan mode conversion no longer actively supported
//  V2.0.1  2026-08-24  Implemented application of the alpha angle LUT
//                      Added Alpha Angle step size override
//                      Removed unused code
//  V2.1.0  2026-08-27  Changed calibration file so that range correction
//                          optional.  This allows just metadata to be saved
//                          which includes the overrride biases.
//                      Corrected logic error for EnableAlphaAngleCorrection
//                      Renamed LoadRangeCalibration() to LoadCalibration to reflect
//                          optional use of Range Correction
//
//--------------------------------------------------------

//--------------------------------------------------------
//--------------------------------------------------------
// This uses modified version of the following Unitree L2 open sources:
//      unitree_lidar_protocol.h
//      unitree_lidar_utilities
// They have been modifed from the original sources
// to correct for errors, missing definitions and
// inconsistencies. These have been minor in most
// instances.
//
// The orignal source can be found at:
//      https://github.com/unitreerobotics/unilidar_sdk2
//      Copyright (c) 2024, Unitree Robotics
//      under License: BSD 3-Clause License (see files)
//
// Corrections/additions have been made to these 2 files
//--------------------------------------------------------

//--------------------------------------------------------
// GPL-3.0 license
//
// This file is part of L2LidarClass.
//
// L2LidarClass is free software : you can redistribute it and /or modify it under
// the terms of the GNU General Public License as published by the Free Software Foundation,
// either version 3 of the License, or (at your option) any later version.
//
// L2LidarClass is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with L2diagnsotic.
// If not, see < https://www.gnu.org/licenses/>.
//--------------------------------------------------------


//--------------------------------------------------------
//
//  No ui (user interface) elements are contained in this class
//
//  class L2lidar
//
//  There is no UART support at this time.  Sample code
//  exists that is commented out based on the QSerialPort class
//--------------------------------------------------------

#pragma once

// The Qt dependencies
#include <QByteArray>
#include <QElapsedTimer>
#include <QMutex>
#include <QObject>
#include <QUdpSocket>
#include <QHostAddress>
#include <QTimer>
#include <QVector>

// other dependencies
#include <unordered_map>
#include "quaternion.h"
#include "PCpoint.h"
#include "L2calibration.h"

// this is required, DO NOT REMOVE
#pragma pack(push, 1)
#include "unitree_lidar_protocolL2.h"
#pragma pack(pop)
// This is typically needed by the parent classes
#include "unitree_lidar_utilitiesL2.h"

typedef struct {
    double lastMeasurement;
    double Average;
    double Variance;
    double min;
    double max;
} Latency;

using Frame = QVector<PCpoint>;

//--------------------------------------------------------
//  L2lidar class definitions
//--------------------------------------------------------
class L2lidar : public QObject {
    Q_OBJECT
public:
    explicit L2lidar(QObject* parent = nullptr);

    // Accessors for external data acces in other threads

    // latest packet information requests

    const LidarAckData ack() const {
        QMutexLocker locker(&PacketMutex);
        return latestACKdata_;
    }

    uint32_t GetL2Workmode() const {
        QMutexLocker locker(&PacketMutex);
        return latestWorkmode_;
    }

    const LidarImuDataPacket imu() {
        QMutexLocker locker(&PacketMutex);
        totalIMUretrieved_++;
        return latestImuPacket_;
    }

    const LidarIpAddressConfig IPaddress() const {
        QMutexLocker locker(&PacketMutex);
        return latestIPaddress_;
    }

    const LidarParamDataPacket L2ParamsPacket() const {
        QMutexLocker locker(&PacketMutex);
        return latestL2ParamsPacket_;
    }

    // This has never been observed but is here just in case
    const LidarMacAddressConfig MAC() const {
        QMutexLocker locker(&PacketMutex);
        return latestMACdata_;
    }

    const Lidar2DPointDataPacket Pcl2Dpacket() {
        QMutexLocker locker(&PacketMutex);
        totalPCretrieved_++;
        return latest2DdataPacket_;
    }

    const LidarPointDataPacket Pcl3Dpacket() {
        QMutexLocker locker(&PacketMutex);
        totalPCretrieved_++;
        return latest3DdataPacket_;
    }

    const LidarTimeStampData timestamp() const {
        QMutexLocker locker(&PacketMutex);
        return latestTimestamp_;
    }

    const LidarVersionData version() const {
        QMutexLocker locker(&PacketMutex);
        return latestVersion_;
    }

    const QString GetLastUDPError() {return stringErrorCOMM;}

    // packet stats from L2 (only updates when L2 socket connected)
    // These are not actually critical, only for reporting stats
    void ClearCounts(); // clears the packet totals
    const Latency GetLatency() const {return latestLatency_;}
	
	// packets received from the L2
    uint64_t lostPackets() const { return lostPackets_; }
    uint64_t total2D() const { return total2Dpackets_;}
    uint64_t total3D() const { return total3Dpackets_;}
    uint64_t totalACK() const { return totalACKpackets_;}
    uint64_t totalIMU() const { return totalIMUpackets_;}
    uint64_t totalPackets() const { return totalPackets_; }
    uint64_t totalOther() const { return lostPackets_; }
	
	// packets retrieved by the app
    uint64_t totalIMUretrieved() const { return totalIMUretrieved_; }
    uint64_t totalPCretrieved() const { return totalPCretrieved_; }

    // number fo scan processed for point cloud
    uint64_t NumPointsConverted() const { return mNumPointedConverted; }
    void ClearNumPointsConverted() { mNumPointedConverted = 0;}

    // L2 commands

    bool GetL2Params(void);
    bool GetWorkMode(void);
    bool LidarGetVersion(void);
    bool LidarReset(void);
    bool LidarStartRotation(void);
    bool LidarStopRotation(void);
    bool sendLatencyID(uint32_t SeqeunceID);
    bool SetL2MAC(LidarMacAddressConfig MACsettings); // requires reset or power cycle after setting
    // This set the stored UDP configuration on the L2
    // a power cycle is required after this for it to take effect
    bool setL2UDPconfig(QString hostIP, uint32_t hostPort,
                        QString LidarIP, uint32_t LidarPort,
                        QString gateway = "0.0.0.0",
                        QString subnet = "255.255.255.0");
    bool SetWorkMode(uint32_t mode);  // requires reset or power cycle after setting

    // L2 Timstamp correction and controls
    void SetUseSystemNowTimestamps(bool enable) {mUseSystemTimestamp = enable;}
    bool GetUseSystemNowTimestamps() {return mUseSystemTimestamp;}

    void EnableL2TimeCorrection(bool enableflag) {mEnableL2TimeStampFix = enableflag; }
    bool GetL2TimeScale(long long& ScaleNumerator, long long& ScaleDenominator)
            {ScaleNumerator = mL2ScaleTimeNum;
             ScaleDenominator = mL2ScaleTimeDen;
             return mBadTSscalar;}
    bool SetL2TimeScale(long long ScaleNumerator, long long ScaleDenominator)
    {
        mL2ScaleTimeNum = ScaleNumerator; mL2ScaleTimeDen = ScaleDenominator;
        double Num = (double) ScaleNumerator;
        double Den = (double) ScaleDenominator;

        if((ScaleNumerator <=0) |
            (ScaleDenominator <=0) |
            (ScaleNumerator <= ScaleDenominator) |
            ((Num/Den) < 1.5) |
            ((Num/Den) > 3.0)) {

            mBadTSscalar = true;
            return false;
        }
        mBadTSscalar = false;
        return true;
    }

    // L2 timestamp syncing to host (timer driven)
    void EnableL2TSsync(bool enable);
    void SetL2TSsyncRate(uint32_t Rate);

    // L2 calibration override (RangeScale and RangeBias)
    void EnableCalibrationOVR(bool Override) {  // true: use calibration overrides
        mOverideCalibration = Override;         // false: use internal L2 calibration
    }

    bool IsCalibrationOVRenabled() { // true: using clibration overrides
        return mOverideCalibration ; // false: using internal L2 calibration
    }

    void SetRangeScaleOVR(double Scale) {mRangeScaleOVR = Scale;}
    void SetRangeBiasOVR(int32_t Offset) {mRangeBiasOVR = Offset;}
    void SetThetaAngleBiasOVR(double angle) {mThetaBiasOVR = angle;} // in degrees
    void SetAlphaAngleBiasOVR(double angle) {mAlphaBiasOVR = angle;} // in degrees
    void SetAlphaAngleStepOVR(double angle) {mAlphaAngleStepOVR = angle;} // in degrees
    void SetBetaAngleOVR(double angle) {mBetaOVR = angle;} // in degrees
    void SetXiAngleOVR(double angle) {mXiOVR = angle;} // in degrees

    double GetRangeScaleOVR() {return mRangeScaleOVR;}
    int32_t GetRangeBiasOVR() {return mRangeBiasOVR;}
    double GetThetaAngleBiasOVR() {return mThetaBiasOVR;} // in degrees
    double GetAlphaAngleBiasOVR() {return mAlphaBiasOVR;} // in degrees
    double GetAlphaAngleStepOVR() {return mAlphaAngleStepOVR;} // in degrees
    double GetBetaAngleOVR() {return mBetaOVR;} // in degrees
    double GetXiAngleOVR() {return mXiOVR;} // in degrees

    // L2 scan paramters
    void EnableFlattenScan(bool p) {mFlattenScanEnabled = p;}
    bool IsFlattenScanEnabled() {return mFlattenScanEnabled ;}

    // double mStartScanAngle {0.0};
    double GetStartScanAngle() {return mStartScanAngle;} // in degrees
    void SetStartScanAngle(double p) {mStartScanAngle = p;}

    // double mScanAngleWidth {360};
    double GetScanAngleWidth() {return mScanAngleWidth;} // in degrees
    void SetScanAngleWidth(double p) {mScanAngleWidth = p;}

    // L2 non-linear range calibration
    void EnableRangeCorrection(bool Correction) { // true: apply non-linear range correction
        mEnableRangeCorrection  = Correction;     // false: use just linear correction (RangeBias, RangeScale)
    }

    // load range calibration file
    bool LoadCalibration(const std::string& filename);

    // L2 range correction clear
    void ClearRangeCorrection();
    // L2 range correction loaded
    bool IsRangeCorrectionLoaded() {return mRangeCorrectionLoaded;}

    // L2 non-linear range calibration
    // true: apply non-linear range correction
    // false: use just linear correction
    bool IsRangeCorrectionEnabled() {return mEnableRangeCorrection ;}

    const std::vector<std::string> GetCalibrationWarnings() {
        return mCalibration.GetWarnings();
    };

    const std::vector<std::string> GetCalibrationErrors() {
        return mCalibration.GetErrors();
    };

    const CalibrationInfo& GetCalibrationInfo() const noexcept
    {
        return mCalibration.GetCalibrationInfo();
    };
    // L2 alpha angle LUT
    bool IsAlphaAngleLUTloaded() {return mAlphaAngleLUTloaded;}
    bool IsAlphaAngleLUTenabled() {return mEnableAlphaAngleCorrection ;}
    void EnableAlphaAngleLUT(bool p) {mEnableAlphaAngleCorrection = p;}
    void ClearAlphaAngleLUT();
    const std::vector<double>& GetAlphaAngleLUT() const noexcept;

    // IMU adjustment to point cloud
    // true: apply IMU adjustment
    // false: do not apply IMU correction
    bool IsIMUadjustEnabled() {return mIMUadjust ;}
    void EnableIMUadjust(bool p) {mIMUadjust = p;}

    // IMU adjustment to point cloud
    // true: apply IMU adjustment
    // false: do not apply IMU correction
    bool IsAdjustRollPitchOnlyEnabled() {return mAdjustRollPitchOnly ;}
    void EnableAdjustRollPitchOnly(bool p) {mAdjustRollPitchOnly = p;}

    double GetIMUPCtimeConstraint() {return mIMUPCtimeConstraint;}
    void SetIMUPCtimeConstraint(double p) {mIMUPCtimeConstraint = p;}

    // latency measurement
    void EnableLatencyMeasure(bool enable);

    // Time sync of L2 to host
    bool SyncL2Clock() ;    // sync L2 to current system time
    bool SyncL2Clock(TimeStamp timestamp) ;    // sync L2 to TimeStamp

    // this is only to set the UDP parameters in the class
    // It DOES NOT change the L2 configuration settings
    void LidarSetCmdConfig(QString srcIP, uint32_t srcPort,
                           QString dstIP, uint32_t dstPort);

    // (Dis)connectL2
    bool IsL2connected() const { return mConnected;}
    bool ConnectL2();  // bind to create, bind socket, connect callback for decode
    void DisconnectL2();   // close socket

    // convert point frame data from L2 to point cloud
    bool ConvertL2data2pointcloud(Frame& frame, bool Frame3D);

    // get/set minRange, MaxRange, minTrustedRange
    void SetMinRange_mm(double p) {mMinRange_mm = p;}
    void SetMaxRange_mm(double p) {mMaxRange_mm = p;}
    void SetMinTrustedRange_mm(double p) {mMinTrustedRange_mm = p;}
    double getMinRange_mm() {return mMinRange_mm;}
    double getMaxRange_mm() {return mMinRange_mm;}
    double getMinTrustedRange_mm() {return mMinTrustedRange_mm;}

signals:
    void ackReceived();
    void imuReceived();
    void IPreceived();
    void MACReceived();
    void L2ParamsReceived();
    void PCL2DReceived();
    void PCL3DReceived();
    void timestampReceived();
    void versionReceived();
    void WorkmodeReceived();

private: // functions
    // Generic Send/receive packets
    bool SendPacket(uint8_t *Buffer,uint32_t Len);
    void processDatagram(const QByteArray& datagram);

    // This is the readyread Qt callback for processing
    // UDP packets that have been recieved
    void readUDPpendingDatagrams();
    bool SendUDPpacket(uint8_t *Buffer,uint32_t Len);

    // UART packets
    bool SendUARTpacket(uint8_t *Buffer,uint32_t Len);
    //void readUARTpendingDatagrams();

    // UDP packet decoders
    void decode3D(const QByteArray& datagram, uint64_t Offset);
    void decode2D(const QByteArray& datagram, uint64_t Offset);
    void decodeImu(const QByteArray& datagram, uint64_t Offset);
    void decodeVersion(const QByteArray& datagram, uint64_t Offset);
    void decodeL2Params(const QByteArray& datagram, uint64_t Offset);
    void decodeMAC(const QByteArray& datagram, uint64_t Offset);
    void decodeWorkmode(const QByteArray& datagram, uint64_t Offset);
    void decodeIPaddress(const QByteArray& datagram, uint64_t Offset);
    void decodeAck(const QByteArray& datagram, uint64_t Offset);
    void handleRaw(uint32_t packetType,
                   const QByteArray& datagram, uint64_t Offset);

    // latency
    bool requestRTTLatencyMeasurement();
    void StartLatency();
    void StopLatency();

    // L2 time base corrections
    void SyncClock() {SyncL2Clock();} // triggered by TimerSyncTimer

    // helper functions
    void setPacketHeader(FrameHeader *FrameHeader, uint32_t packet_type,
                         uint32_t packet_size);
    void setPacketTail(FrameHeader *FrameTale);

    void UpdateEWMAStats(double alpha,
                     double Xnew,
                     double& Xmean,
                     double& Xvariance
                     );

    void FixTimeStamp(TimeStamp& stamp);
    void SetSystemTimeStamp(TimeStamp& stamp);

    inline bool inAngularWindow(double theta,
                            const double StartAngle,
                                const double AngleWidth);

    inline void SelectiveParseFromPacketToPointCloud(
        unilidar_sdk2::PointCloudUnitree &cloud,
        const LidarPointDataPacket &packet,
        bool use_system_timestamp = false
        );

private: // variables
    L2calibration mCalibration;

    // mutex for critical packet access while copying packet
    mutable QMutex  PacketMutex;

    // Communicatopns selector
    bool UseSerial {false}; // false -  use UDP
                            // true - use UART

    // UDP socket
    QUdpSocket L2socket;

    // Serial UART
    QString SerialPort {"com27"};
    // serial port settings are fixed and can not be changed
    // 4M buadrate, 8 bit, even partity, 1 stop, no flow control ?

    // Packet buffer
    QByteArray PacketBuffer;
    bool IncompletePacket {false};  // if true needs more UDP datagrams
                                    // to complete packet

    // Latest decoded values
    // Accessing these should use mutex lock, PacketMutex
    LidarImuDataPacket  latestImuPacket_{};
    LidarVersionData    latestVersion_{};
    LidarTimeStampData  latestTimestamp_{};
    Lidar2DPointDataPacket latest2DdataPacket_{};
    LidarPointDataPacket latest3DdataPacket_{};
    LidarParamDataPacket latestL2ParamsPacket_{};
    LidarMacAddressConfig latestMACdata_{};
    LidarAckData latestACKdata_{};
    LidarIpAddressConfig latestIPaddress_ {};

    // Packet counters, these do not have a mutex lock
    // and should not be relied on for downstream processing
    // They are intended to be only informative
    uint64_t totalPackets_{0};
    uint64_t lostPackets_{0};
    uint64_t totalIMUpackets_{0};
    uint64_t totalACKpackets_{0};
    uint64_t total3Dpackets_{0};
    uint64_t total2Dpackets_{0};
    uint64_t totalOther_{0};
    uint64_t totalIMUretrieved_{0};
    uint64_t totalPCretrieved_{0};

    // number of points processed for point cloud
    uint64_t mNumPointedConverted {0};

    // QudpSocket parmameters
    // These should only be a reflection of L2
    // UDP ethernet interface. They do not set
    // ethernet configuration on the L2
    QString src_ip {"192.168.1.2"}; // factory default
    QString dst_ip {"192.168.1.62"}; // factory default
    uint32_t src_port {6201}; // factory default
    uint32_t dst_port {6101}; // factory default

    // Latency measurement variables
    QElapsedTimer latencyTimer;
    QTimer LatencyTimer;
    std::unordered_map<uint32_t, qint64> latencyMap; // SeqID → send time (ns)
    // latest latency measurements
    uint32_t SequenceID {100};
    Latency latestLatency_ {-1.0,0.0,-1.0, 999.99,-1.0};
    bool mEnableLatency {true};

    // enable L2 timestamp correction
    QTimer TimerSyncTimer;
    // This only enables correction algorithm
    // It does not enable timer based updates to
    // syncing of the L2 timestamp
    bool mEnableL2TimeStampFix {false};
    // last known timestamp sync
    // This is used as offset along with scale to correct
    // the L2 timestamp
    long long mLastTimestamp {-1}; // units are nanoseconds
    // L2 Fw Version 2.8.11.1, compile date: 2025-07-30
    // This is known to have a timestamp which is slow by a factor 1/2 actual time
    // As an example the bench measurement on one L2 the actual factor is 0.4983629.
    // This was measured after 15 minutes of warmup when temperature
    // drift stabilized.  It appears that the IMU temperature may be
    // controlled at ~60C.  This results in using a scaler = (1/0.498363):
    //      mL2ScaleTimeNum = 200567
    //      mL2ScaleTimeDen = 100000
    // Individual measurements should be made on each L2
    long long mL2ScaleTimeNum {2};
    long long mL2ScaleTimeDen {1};
    bool mBadTSscalar {false};

    // sync L2 to host controls
    bool mL2EnableSyncHost = false;
    uint32_t mL2TSsyncRate = {0}; // stop timer

    // Ignore packet timestamps, return SystemNow tiemstamps instead
    bool mUseSystemTimestamp {false};

    int skipIMUpackets {100}; // countdown to good frames
    int skipPCpackets {100};// countdown to good frames

    // workmode
    uint32_t latestWorkmode_ {256}; // 256 is invalid

    // L2 connected
    QString stringErrorCOMM {};
    bool mConnected {false}; // set true when connected to L2

    // L2 IMU adjust point cloud
    bool mIMUadjust {false};
    bool mAdjustRollPitchOnly {true};
    double mIMUPCtimeConstraint {0.07};

    // L2 calibration
    double mRangeScaleOVR {.001};
    int32_t mRangeBiasOVR {-500};
    double mAlphaBiasOVR {1.5};
    double mAlphaAngleStepOVR {0.602};
    double mThetaBiasOVR {120.0};
    double mBetaOVR {0.0};
    double mXiOVR {0.0};
    bool mOverideCalibration {false};

    // L2 scan parameters
    bool mFlattenScanEnabled {false}; // only allow capture of point clouds close to xz plane
    double mStartScanAngle {0.0};
    double mScanAngleWidth {360};

    // L2 min ranges, max ranges
    double mMinRange_mm{0.0};
    double mMaxRange_mm {65535.0};
    double mMinTrustedRange_mm {-1.0};

    // L2 Range non-linear range correction
    bool mEnableRangeCorrection {false};
    std::vector<double> mRangeCorrectionLUT;
    bool mRangeCorrectionLoaded {false};

    // L2 Alpha angle LUT
    bool mEnableAlphaAngleCorrection {false};
    bool mAlphaAngleLUTloaded {false};
    std::vector<double> mAlphaAngleLUT;
};
