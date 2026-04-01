//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: LidarDecoder.h
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
//  conversation targeting a QT Creator development platform.
//  It reads UPD packets from the L2, categorizes them, performs
//  error detection for bad packets (lost), live point cloud display.
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
//                          mL2EnableSyncHost && enableL2TimeStampFix
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
//  V1.0.0  2026-02-20  Separated L2lidar class from the L2diagnostic app and l2lidar_node app
//                      This is the initial release of the standalone L2lidar class
//                      Changed the unitree_lidar_utilities.h,  the parse function for the
//  V1.1.0  2026-02-22  Corrected ConvertL2data2pointcloud() to generate more accurate timestamps.
//                      Changed the timestamp units in the cloud point array returned by
//                      ConvertL2data2pointcloud().
//                      PCpoint structure member time change from float to long long
//                      PCpoint structure member time units are now nanoseconds since Epoch
//
//--------------------------------------------------------

//--------------------------------------------------------
// This uses the following Unitree L2 sources modules:
//      unitree_lidar_protocol.h
//      unitree_lidar_utilities
// The orignal source can be found at:
//      https://github.com/unitreerobotics/unilidar_sdk2
//      under License: BSD 3-Clause License (see files)
//
// Corrections/additions have been made to these 2 files
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
    // such as a timer based GUI

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
                        QString LidarIP, uint32_t LidarPort);
    bool SetWorkMode(uint32_t mode);  // requires reset or power cycle after setting

    // L2 Timstamp correction and controls
    void EnableL2TimeCorrection(bool enableflag) {enableL2TimeStampFix = enableflag; }
    double GetL2TimeScale() {return mL2ScaleTimeStamp;}
    void SetL2TimeScale(double Scale) {mL2ScaleTimeStamp = Scale;}

    // L2 timestamp syncing to host (timer driven)
    void EnableL2TSsync(bool enable);
    void SetL2TSsyncRate(uint32_t Rate);

    // latency measurement
    void EnableLatencyMeasure(bool enable);

    // Time sync of L2 to host
    bool SyncL2Clock() ;    // sync L2 to current system time
    bool SyncL2Clock(TimeStamp timestamp) ;    // sync L2 to TimeStamp

    // this is only to set the UDP parameters in the class
    // It DOES NOT change the L2 configuration settings
    void LidarSetCmdConfig(QString srcIP, uint32_t srcPort,
                           QString dstIP, uint32_t dstPort);


    bool ConnectL2();  // bind to create, bind socket, connect callback for decode
    void DisconnectL2();   // close socket

    // convert point frame data from L2 to point cloud
    bool ConvertL2data2pointcloud(Frame& frame, bool Frame3D, bool IMUadjust);

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
    void decodeTimestamp(const QByteArray& datagram, uint64_t Offset);
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

private: // variables
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
    bool enableL2TimeStampFix {false};
    // last known timestamp sync
    // This is used as offset along with scale to correct
    // the L2 timestamp
    double mLastTimestamp {0};
    // L2 Fw Version 2.8.11.1, compile date: 2025-07-30
    // is known to have a timestamp which is slow by
    // a factor 2.0
    double mL2ScaleTimeStamp {2.0};
    bool mL2EnableSyncHost = false;
    uint32_t mL2TSsyncRate = {0}; // stop timer

    int skipIMUpackets {100}; // countdown to good frames
    int skipPCpackets {100};// countdown to good frames

    // workmode
    uint32_t latestWorkmode_ {256}; // 256 is invalid

    // L2 connected
    QString stringErrorCOMM {};
    bool mConnected {false}; // set true when connected to L2
};
