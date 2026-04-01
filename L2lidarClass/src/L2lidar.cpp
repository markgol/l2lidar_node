//--------------------------------------------------------
//
//  L2Diagnostic
//  Author: Mark Stegall
//  Module: L2lidar.cpp
//
//  Purpose:
//
//  The L2lidar class consists of 2 files:
//      L2lidar.cpp
//      L2lidar.h
//
//  This is to provide necesary software interfaces to control and
//  receive data for the Unitree L2 LiDAR.
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
//      The serial UART on various platforms can be limited in speed and
//      may not operate at the full sensor speed of 64K/sec sample points.
//
//  Current status:
//      Implementation of class verified using UDP interface only.
//      Serial UART impementation is being explored but not included.
//      Working on integration and use of this class in support of ROS2
//      as substitute for Unitree's SDK proprietary archive library.
//
//  Planned offical release will be V0.4.0
//
//
//  V0.1.0  2025-12-27  compilable skeleton created by ChatGPT
//  V0.2.0  2026-01-02  Documentation, start of debugging
//                      CRC32normal() added to unitree_lidar_utilies.h
//                      implementation of LidarDecoder
//  V0.2.1  2026-01-05  Changed LidarDecoder.h and cpp to L2lidar
//                      Changed class name from LidarDecoder to L2lidar
//                      Added L2 start rotation, stop rotation,
//                              reset and get Version info
//                      Added crc32() check on incoming packets
//                      Added crc32() calculation on outgoing packets
//                      Moved all socket calls to this class
//                      removed CRC32normal()from unitree_lidar_utilies.h
//                      Full PCL packet is saved because the
//                          unitree utility to convert PCL cloud
//                          requires it
//  V0.3.4  2026-01-23  Changed processingDatagram() to process multiple
//                      UDP datagrams into one L2 Lidar packet
//  V0.3.5  2026-01-24  Correction to last2D packet
//  V0.3.6  2026-01-26  Added skeleton for Serial UART only
//                      (std QSerialPort has issues with
//                       4M buadrate and 250K byte/sec)
//  V0.3.7  2026-01-26  Primarily documentation updates
//                      Added setting UDP settings on L2
//                      Minor bug corrections
//  V0.3.7  2026-01-29  Added send latency ID command
//                      Minor bug corrections
//                      Added Set UPD configuration in the L2
//                      Added requestLatencyMeasurement(), note this is rtt latency
//                          This is non-blocking.
//  V0.3.9  2026-01-30  Added SyncL2clock(), SyncL2clock(TimeStamp)
//  V0.3.10 2026-02-01  Added Get L2 Parameters
//                      Added GetWorkmode()
//                      Added enable latency measurement flag
//  V0.3.10 2026-02-03  Changed get L2 params cmd_value to 3
//                      to match observed behaviour
//  V0.3.11 2026-02-04  Added void ConvertL2data2pointcloud()
//                      to return just actual point cloud
//                      frame instead of entire unprocessed packet
//  V0.4.1  2026-02-11  Remove Qdebug statements
//                      Added MAC packet decode
//                      Set MAC command
//                      Added sending a datagram to unblock
//                      the Qt readyRead signal
//                      Added decode for the 3 config packets, MAC, workmode, IPaddress
//                      Sorted alphabetically in groups for public class members
//  V0.4.2  2026-02-13  Bug correction, connectL2() mConnected was after all intialization
//                      instead of just after the socket connection.  Moved to just after
//                      socket connection where it belongs.
//                      Added error string for communication connect failure
//                      or send error
//  V0.4.3  2026-02-16  Added more logic to the timestamping of the point cloud data
//  V0.4.3  2026-02-16  Added more logic to the timestamping of the point cloud data
//                          mL2EnableSyncHost && enableL2TimeStampFix
//                              true  use L2 timestamping for each cloud point
//                              false use system time for each cloud point
//                      If adjust cloud packet using IMU is enabled
//                      then if the point cloud packet is different from
//                      IMU packet time by more than 50 msec the point cloud
//                      packet will be dropped.
//                      Added range(m) to point cloud data.  It is already present
//                      in the raw point cloud packet.  It saves recomputing it later
//                      in a user app.  PCpoint.h has been changed to include this field.
//  V1.0.0  2026-02-20  Separated L2lidar class from the L2diagnostic app and l2lidar_node app
//                      This is the initial release of the standalone L2lidar class
//  V1.1.0  2026-02-22  Added [[maybe_unused]] to params for HandleRaw() packet decode
//                      Corrected ConvertL2data2pointcloud() to generate more accurate timestamps.
//                      Changed the timestamp units in the cloud point array returned by
//                      ConvertL2data2pointcloud().
//                      PCpoint structure member time change from float to long long
//                      PCpoint structure member time units are now nanoseconds since Epoch
//
//--------------------------------------------------------

//--------------------------------------------------------
// This uses the following Unitree L2 open sources:
//      unitree_lidar_protocol.h
//      unitree_lidar_utilities
// They have been modifed from the original sources
// to correct for errors, missing definitions and
// inconsistencies. These have been minor in most
// instances.
//
// Copyright (c) 2024, Unitree Robotics
// The orignal source can be found at:
//      https://github.com/unitreerobotics/unilidar_sdk2
//      under License: BSD 3-Clause License (see files)
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

//--------------------------------------------------------
//
//  The packets sent from the L2 are recieved on different port
//  then the command packets that are sent to the L2
//
//  No ui (user interface) elements are contained in this class
//
//  class L2lidar
//
//--------------------------------------------------------

#include "L2lidar.h"

//--------------------------------------------------------------------
// L2lidar class constructor
//
//  There is no complex constructor required
//
//--------------------------------------------------------------------
L2lidar::L2lidar(QObject* parent)
    : QObject(parent)
{
    PacketBuffer.clear(); // make sure buffered starts cleared
    latencyTimer.start();
}


//====================================================================
//  start of received packet handling section
//  This handles the receipt of packets from the L2
//  This includes error checking of the packets
//  When certain packets are decoded a signal is emitted
//  to conneted subscribers
//  The packet types or conditions that emit signals are:
//      3D point cloud packets
//      2D point cloud packets
//      ACK packet
//      VERSION packet
//      Time stamp updates from IMU, point cloud or timestamp packets
//====================================================================

//--------------------------------------------------------
//  readUDPpendingDatagrams()
//
//  It is a Qt non-blocking I/O service called when data
//  is received on the ethernet interface.
//  This is a callback function that receives the UDP
//  datagram packets.
//
//  It is connected to the readyread Qt thread in
//  this ConnectL2() in this class.
//
//  It is not used externally from the class
//
//--------------------------------------------------------
void L2lidar::readUDPpendingDatagrams()
{
    // only process incoming UPD packets
    while (L2socket.hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(static_cast<int>(L2socket.pendingDatagramSize()));
        QHostAddress sender;
        quint16 senderPort;

        // read next Datagram
        L2socket.readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);

        // packets decoder also updates the specific packet ID count totals and packet loss counts
        // csv file processing also happens in the decoder
        processDatagram(datagram);
    }
}

//--------------------------------------------------------
//  readUARTpendingDatagrams()
//  This is a callback function that receives the UDP
//  data packets.  It is a Qt non-blocking I/O service called
//  when data in received on the UART interface
//  This is just skeleton fragment, to reserve for
//  future implementation
//--------------------------------------------------------
//void L2lidar::readUARTpendingDatagrams()
//{
    // // only process incoming UPD packets
    // while (L2SerialPort.hasPendingDatagrams()) {
    //     QByteArray datagram;
    //     datagram.resize(static_cast<int>(L2SerialPort.pendingDatagramSize()));
    //     QHostAddress sender;
    //     quint16 senderPort;

    //     // read next Datagram
    //     L2SerialPort.readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);

    //     // packets decoder also updates the specific packet ID count totals and packet loss counts
    //     // csv file processing also happens in the decoder
    //     processDatagram(datagram);
    // }
//}

//--------------------------------------------------------
//  processDatagram()
//  processing the payload package from the datagram
//  note: more than one packet may be in the datagram
//  Each packet must be processed.
//
//  processing requirements:
//  All but one of the L2 Lidar packets can be contained
//  in one UDP datagram.  Some UDP datagrams contain more
//  than one L2 Lidar packet.
//  Only the L2 Lidar packet for 2D scan data spans multiple
//  UDP datagrams (4 datagrams)
//
//  This means that a processing buffer needs to be appended
//  so that it can span multiple UDP datagrams.
//
//--------------------------------------------------------
void L2lidar::processDatagram(const QByteArray& datagram)
{
    uint64_t Length;
    uint64_t Offset {0};



    // check if additional UDP datagrams needed for complete packet
    if(!IncompletePacket) {
        // if last packet was complete then this is first
        // datagram in new packet
        PacketBuffer.clear();
    }

    PacketBuffer.append(datagram);

    // Get the Datagram size
    Length = PacketBuffer.size();

    // first check if this is too small to be valid UPD frame
    if (Length < sizeof(FrameHeader) + sizeof(FrameTail)) {
        totalPackets_++;
        lostPackets_++;
        return;
    }

    // make sure first UDP packet starts with correct Header
    // since we will need packet size from header to determine
    // if multiple UPD packets are in the datagram.

    do {
        // point to current packet
        const auto* header =
            reinterpret_cast<const FrameHeader*>(PacketBuffer.constData() + Offset);

        // check header frame correct format
        if (header->header[0] != FRAME_HEADER_ARRAY_0 ||
            header->header[1] != FRAME_HEADER_ARRAY_1 ||
            header->header[2] != FRAME_HEADER_ARRAY_2 ||
            header->header[3] != FRAME_HEADER_ARRAY_3) {

            totalPackets_++;
            lostPackets_++;
            return;
        }

        Length = PacketBuffer.size();
        if(header->packet_size > Length) {
            // The PacketBuffer does not have enough data for this
            // L2 Lidar packet
            // This will add the next UDP datagram
            IncompletePacket = true;
            return;
        }

        // enough data in packet buffer to process packet
        IncompletePacket = false;

        // Verify Tail and CRC
        auto* tail = reinterpret_cast<const FrameTail*>(
                        PacketBuffer.constData() + ((Offset + header->packet_size) - sizeof(FrameTail)));
	
	    // check tail frame correct format
        // 2D packets do not have consistent tail code

        bool GoodTailCode =
            ((tail->tail[0] == FRAME_TAIL_ARRAY_0) &&
             (tail->tail[1] == FRAME_TAIL_ARRAY_1));

        if (!GoodTailCode) {
            totalPackets_++;
            lostPackets_++;
            return;
        }

        // perform crc just on the data without hreader or tail
        uint32_t crc = unilidar_sdk2::crc32(reinterpret_cast<const uint8_t*>(PacketBuffer.constData()+Offset+sizeof(FrameHeader)),
                               header->packet_size - (sizeof(FrameHeader) + sizeof(FrameTail)));

        if (crc != tail->crc32) {
            totalPackets_++;
            lostPackets_++;
            return;
        }

        switch (header->packet_type) {
            case LIDAR_IMU_DATA_PACKET_TYPE:
                if(skipIMUpackets <= 0) {
                    decodeImu(PacketBuffer,Offset);
                } else {
                    skipIMUpackets--;
                }
                Offset += header->packet_size;
                break;

            case LIDAR_POINT_DATA_PACKET_TYPE:
                if(skipPCpackets <= 0) {
                    decode3D(PacketBuffer,Offset);
                } else {
                    skipPCpackets--;
                }
                Offset += header->packet_size;
                break;

            case LIDAR_2D_POINT_DATA_PACKET_TYPE:
                if(skipPCpackets <= 0) {
                    decode2D(PacketBuffer,Offset);
                } else {
                    skipPCpackets--;
                }
                Offset += header->packet_size;
                break;

            case LIDAR_VERSION_PACKET_TYPE:
                decodeVersion(PacketBuffer,Offset);
                Offset += header->packet_size;
                break;

            case LIDAR_TIME_STAMP_PACKET_TYPE:
                decodeTimestamp(PacketBuffer,Offset);
                Offset += header->packet_size;
                break;

            case LIDAR_PARAM_DATA_PACKET_TYPE:
                decodeL2Params(PacketBuffer,Offset);
                Offset += header->packet_size;
                break;

            case LIDAR_MAC_ADDRESS_CONFIG_PACKET_TYPE:
                decodeMAC(PacketBuffer,Offset);
                Offset += header->packet_size;
                break;

            case LIDAR_WORK_MODE_CONFIG_PACKET_TYPE:
                decodeWorkmode(PacketBuffer,Offset);
                Offset += header->packet_size;
                break;

            case LIDAR_IP_ADDRESS_CONFIG_PACKET_TYPE:
                decodeIPaddress(PacketBuffer,Offset);
                Offset += header->packet_size;
                break;

            case LIDAR_ACK_DATA_PACKET_TYPE:
                decodeAck(PacketBuffer,Offset);
                Offset += header->packet_size;
                break;

            default:
                handleRaw(header->packet_type, PacketBuffer,Offset);
                Offset += header->packet_size;
                break;
        }
    } while(Offset < Length);
}

//--------------------------------------------------------------------
//  3D point cloud packet Decoder
//--------------------------------------------------------------------
void L2lidar::decode3D(const QByteArray& datagram, uint64_t Offset)
{
    const auto* header =
        reinterpret_cast<const FrameHeader*>(datagram.constData() + Offset);

    if ((size_t)header->packet_size != sizeof(LidarPointDataPacket)) {
        lostPackets_++;
        return;
    }

    totalPackets_++;
    total3Dpackets_++;
    const auto* pkt =
        reinterpret_cast<const LidarPointDataPacket*>(datagram.constData()+Offset);

    double t1;
    double t2;

    // critical section
    PacketMutex.lock();

    latest3DdataPacket_.header = pkt->header;
    latest3DdataPacket_.data = pkt->data;
    latest3DdataPacket_.tail = pkt->tail;

    // correct timestamp if needed
    // initally for test only changed latest
    // after testing also change packet
    if(enableL2TimeStampFix) {
        t1 = (double)latest3DdataPacket_.data.info.stamp.sec +
             ((double)latest3DdataPacket_.data.info.stamp.nsec*1.0e-9);

        t2 = ((t1-mLastTimestamp) * mL2ScaleTimeStamp) + mLastTimestamp;

        // convert to seconds, nanoseconds
        latestTimestamp_.data.sec = (uint32_t) t2;
        t2 = t2 - (double)latestTimestamp_.data.sec;
        latestTimestamp_.data.nsec = (uint32_t)(t2*1.0e9);

        // update time stamp in the packet
        latest3DdataPacket_.data.info.stamp.sec = latestTimestamp_.data.sec;
        latest3DdataPacket_.data.info.stamp.nsec = latestTimestamp_.data.nsec;

    } else {
        latestTimestamp_.data.sec = latest3DdataPacket_.data.info.stamp.sec;
        latestTimestamp_.data.nsec = latest3DdataPacket_.data.info.stamp.nsec;
    }

    PacketMutex.unlock();
    // end of critical section

    // send out notice of latest L2 time stap
    emit timestampReceived();
    // send out notice that a 3D point cloud packet received
    emit PCL3DReceived();
}

//--------------------------------------------------------------------
//  2D point cloud packet Decoder
//--------------------------------------------------------------------
void L2lidar::decode2D(const QByteArray& datagram, uint64_t Offset)
{
    const auto* header =
        reinterpret_cast<const FrameHeader*>(datagram.constData() + Offset);

    if ((size_t)header->packet_size != sizeof(Lidar2DPointDataPacket)) {
        lostPackets_++;
        return;
    }
    totalPackets_++;
    total2Dpackets_++;
    const auto* pkt =
        reinterpret_cast<const Lidar2DPointDataPacket*>(datagram.constData()+Offset);

    double t1;
    double t2;

    // critical section
    PacketMutex.lock();

    latest2DdataPacket_.header = pkt->header;
    latest2DdataPacket_.data = pkt->data;
    latest2DdataPacket_.tail = pkt->tail;

    // correct timestamp if needed
    // initally for test only changed latest
    // after testing also change packet
    if(enableL2TimeStampFix) {
        t1 = (double)latest2DdataPacket_.data.info.stamp.sec +
             ((double)latest2DdataPacket_.data.info.stamp.nsec*1.0e-9);

        t2 = ((t1-mLastTimestamp) * mL2ScaleTimeStamp) + mLastTimestamp;

        // convert to seconds, nanoseconds
        latestTimestamp_.data.sec = (uint32_t) t2;
        t2 = t2 - (double)latestTimestamp_.data.sec;
        latestTimestamp_.data.nsec = (uint32_t)(t2*1.0e9);

        // update time stamp in the packet
        latest2DdataPacket_.data.info.stamp.sec = latestTimestamp_.data.sec;
        latest2DdataPacket_.data.info.stamp.nsec = latestTimestamp_.data.nsec;
    } else {
        latestTimestamp_.data.sec = latest2DdataPacket_.data.info.stamp.sec;
        latestTimestamp_.data.nsec = latest2DdataPacket_.data.info.stamp.nsec;
    }

    PacketMutex.unlock();
    // end of critical section



    // send out notice of latest L2 time stap
    emit timestampReceived();
    // send out notice that a 2D point cloud packet received
    emit PCL2DReceived();
}

//--------------------------------------------------------------------
//  IMU packet Decoder
//--------------------------------------------------------------------
void L2lidar::decodeImu(const QByteArray& datagram, uint64_t Offset)
{
    const auto* header =
        reinterpret_cast<const FrameHeader*>(datagram.constData() + Offset);

    if ((size_t)header->packet_size != sizeof(LidarImuDataPacket)) {
        lostPackets_++;
        return;
    }
    totalPackets_++;
    totalIMUpackets_++;
    const auto* pkt =
        reinterpret_cast<const LidarImuDataPacket*>(datagram.constData()+Offset);

    double t1;
    double t2;

    // critical section
    PacketMutex.lock();
    latestImuPacket_.header = pkt->header;
    latestImuPacket_.data = pkt->data;
    latestImuPacket_.tail = pkt->tail;

    // correct timestamp if needed
    // initally for test only changed latest
    // after testing also change packet
    if(enableL2TimeStampFix) {
        t1 = (double)pkt->data.info.stamp.sec +
             ((double)pkt->data.info.stamp.nsec*1.0e-9);
        t2 = ((t1-mLastTimestamp) * mL2ScaleTimeStamp) +mLastTimestamp;

        // convert to seconds, nanoseconds
        latestTimestamp_.data.sec = (uint32_t) t2;
        t2 = t2 - (double)latestTimestamp_.data.sec;
        latestTimestamp_.data.nsec = (uint32_t)(t2*1.0e9);

        // update time stamp in the packet
        latestImuPacket_.data.info.stamp.sec = latestTimestamp_.data.sec;
        latestImuPacket_.data.info.stamp.nsec = latestTimestamp_.data.nsec;
    } else {
        latestTimestamp_.data.sec = pkt->data.info.stamp.sec;
        latestTimestamp_.data.nsec = pkt->data.info.stamp.nsec;
    }

    PacketMutex.unlock();
    // end of critical section

    // send out notice of latest L2 time stap
    emit timestampReceived();
    // send out notice that a IMU packet received
    emit imuReceived();
}

//--------------------------------------------------------------------
//  VERSION packet Decoder
//--------------------------------------------------------------------
void L2lidar::decodeVersion(const QByteArray& datagram, uint64_t Offset)
{
    const auto* header =
        reinterpret_cast<const FrameHeader*>(datagram.constData() + Offset);

    if ((size_t)header->packet_size != sizeof(LidarVersionDataPacket)) {
        lostPackets_++;
        return;
    }
    totalOther_++;
    totalPackets_++;

    const auto* pkt =
        reinterpret_cast<const LidarVersionDataPacket*>(datagram.constData()+Offset);

    // critical section
    PacketMutex.lock();
    latestVersion_ = pkt->data;
    PacketMutex.unlock();
    // end of critical section

    // send out notice that a VERSION packet received
    emit versionReceived();
}

//--------------------------------------------------------------------
// TIMESTAMP Decoder
// Currently no timestamp packet has been observed being sent
// by the L2
//--------------------------------------------------------------------
void L2lidar::decodeTimestamp(const QByteArray& datagram, uint64_t Offset)
{
    const auto* header =
        reinterpret_cast<const FrameHeader*>(datagram.constData() + Offset);

    if ((size_t)header->packet_size != sizeof(LidarTimeStampPacket)) {
        lostPackets_++;
        return;
    }

    totalOther_++;
    totalPackets_++;

    const auto* pkt =
        reinterpret_cast<const LidarTimeStampPacket*>(datagram.constData()+Offset);

    // critical section
    PacketMutex.lock();
    latestTimestamp_ = pkt->data;
    PacketMutex.unlock();
    // end of critical section

    // send out notice that a TIMESTAMP packet received
    emit timestampReceived();
}

//--------------------------------------------------------------------
// MAC Decoder
//--------------------------------------------------------------------
void L2lidar::decodeMAC(const QByteArray& datagram, uint64_t Offset)
{
    const auto* header =
        reinterpret_cast<const FrameHeader*>(datagram.constData() + Offset);

    if ((size_t)header->packet_size != sizeof(LidarMacAddressConfigPacket)) {
        lostPackets_++;
        return;
    }

    totalOther_++;
    totalPackets_++;

    const auto* pkt =
        reinterpret_cast<const LidarMacAddressConfigPacket*>(datagram.constData()+Offset);

    // critical section
    PacketMutex.lock();
    latestMACdata_ = pkt->data;
    PacketMutex.unlock();
    // end of critical section

    // send out notice that a MAC config packet received
    emit MACReceived();
}

//--------------------------------------------------------------------
// workmode Decoder
//--------------------------------------------------------------------
void L2lidar::decodeWorkmode(const QByteArray& datagram, uint64_t Offset)
{
    const auto* header =
        reinterpret_cast<const FrameHeader*>(datagram.constData() + Offset);

    if ((size_t)header->packet_size != sizeof(LidarWorkModeConfigPacket)) {
        lostPackets_++;
        return;
    }

    totalOther_++;
    totalPackets_++;

    const auto* pkt =
        reinterpret_cast<const LidarWorkModeConfigPacket*>(datagram.constData()+Offset);

    // critical section
    PacketMutex.lock();
    latestWorkmode_ = pkt->data.mode;
    PacketMutex.unlock();
    // end of critical section

    // send out notice that a workmode config packet received
    emit WorkmodeReceived();
}

//--------------------------------------------------------------------
// IP address Decoder
//--------------------------------------------------------------------
void L2lidar::decodeIPaddress(const QByteArray& datagram, uint64_t Offset)
{
    const auto* header =
        reinterpret_cast<const FrameHeader*>(datagram.constData() + Offset);

    if ((size_t)header->packet_size != sizeof(LidarIpAddressConfigPacket)) {
        lostPackets_++;
        return;
    }

    totalOther_++;
    totalPackets_++;

    const auto* pkt =
        reinterpret_cast<const LidarIpAddressConfigPacket*>(datagram.constData()+Offset);

    // critical section
    PacketMutex.lock();
    latestIPaddress_ = pkt->data;
    PacketMutex.unlock();
    // end of critical section

    // send out notice that a IP address config packet received
    emit IPreceived();
}

//--------------------------------------------------------------------
// L2 PARAMS Decoder
// This is for investigating the parameters packet
//--------------------------------------------------------------------
void L2lidar::decodeL2Params(const QByteArray& datagram, uint64_t Offset)
{
    const auto* header =
        reinterpret_cast<const FrameHeader*>(datagram.constData() + Offset);

    if ((size_t)header->packet_size != sizeof(LidarParamDataPacket)) {
        lostPackets_++;
        return;
    }

    totalPackets_++;

    const auto* pkt =
        reinterpret_cast<const LidarParamDataPacket*>(datagram.constData()+Offset);

    // critical section
    PacketMutex.lock();

    latestL2ParamsPacket_.header = pkt->header;
    latestL2ParamsPacket_.data = pkt->data;
    latestL2ParamsPacket_.tail = pkt->tail;
    latestWorkmode_ = pkt->data.workmode;
    PacketMutex.unlock();
    // end of critical section
    emit WorkmodeReceived();
    emit L2ParamsReceived();
    return;
}

//--------------------------------------------------------------------
// ACK Decoder
//--------------------------------------------------------------------
void L2lidar::decodeAck(const QByteArray& datagram, uint64_t Offset)
{
    const auto* header =
        reinterpret_cast<const FrameHeader*>(datagram.constData() + Offset);

    if ((size_t)header->packet_size != sizeof(LidarAckDataPacket)) {
        lostPackets_++;
        return;
    }
    totalPackets_++;
    totalACKpackets_++;

    const auto* pkt =
        reinterpret_cast<const LidarAckDataPacket*>(datagram.constData()+Offset);

    uint32_t seq = pkt->data.cmd_value;

    if(mEnableLatency) {
        // ----- RTT latency calculation -----
        auto it = latencyMap.find(seq);
        if (it != latencyMap.end()) {
            qint64 now = latencyTimer.nsecsElapsed();
            double latencyMs = (now - it->second) / 1e6;
            latencyMap.erase(it);

            // save results if valid
            latestLatency_.lastMeasurement = latencyMs;
            if(latestLatency_.Average<0.0) {
                // first point is new stats
                latestLatency_.Average = latencyMs;
                latestLatency_.Variance = 0.0;
                latestLatency_.max = latencyMs;
                latestLatency_.min = latencyMs;
            } else {
                // compute stats
                PacketMutex.lock();
                if(latencyMs>latestLatency_.max)
                    latestLatency_.max = latencyMs;
                if(latencyMs<latestLatency_.min)
                    latestLatency_.min = latencyMs;
                // first order exponential moving average
                const double Aplha {.05};
                double Xmean = latestLatency_.Average;
                double Xvariance = latestLatency_.Variance;

                UpdateEWMAStats(Aplha,latencyMs,Xmean,Xvariance);

                latestLatency_.Average = Xmean;
                latestLatency_.Variance = Xvariance;
                PacketMutex.unlock();
            }
        }
    // ----------------------------------
    }

    // critical section
    PacketMutex.lock();
    latestACKdata_ = pkt->data;
    PacketMutex.unlock();
    // end of critical section

    // send out notice that a ACK packet received
    emit ackReceived();
}

//--------------------------------------------------------------------
// Raw Packet Handler
// This is called when the packet type is NOT:
//          LIDAR_IMU_DATA_PACKET_TYPE
//          LIDAR_POINT_DATA_PACKET_TYPE
//          LIDAR_2D_POINT_DATA_PACKET_TYPE
//          LIDAR_VERSION_PACKET_TYPE
//          LIDAR_TIME_STAMP_PACKET_TYPE
//          LIDAR_PARAM_DATA_PACKET_TYPE
//          LIDAR_MAC_ADDRESS_CONFIG_PACKET_TYPE
//          LIDAR_WORK_MODE_CONFIG_PACKET_TYPE
//          LIDAR_IP_ADDRESS_CONFIG_PACKET_TYPE
//          LIDAR_ACK_DATA_PACKET_TYPE
//--------------------------------------------------------------------
void L2lidar::handleRaw([[maybe_unused]]uint32_t packetType,
                        [[maybe_unused]]const QByteArray& datagram,
                        [[maybe_unused]]uint64_t Offset)
{
    //const auto* header =
    //   reinterpret_cast<const FrameHeader*>(datagram.constData() + Offset);

    totalOther_++;
    totalPackets_++;
}
//====================================================================
// end of received packet handling section
//====================================================================

//--------------------------------------------------------------------
//  ClearCounts()
//  Packet count statistics are accumulated
//  This reset the stats
//--------------------------------------------------------------------
void L2lidar::ClearCounts()
{
    totalPackets_ = 0;
    lostPackets_ = 0;
    totalIMUpackets_ = 0;
    totalACKpackets_ = 0;
    total3Dpackets_ = 0;
    total2Dpackets_ = 0;
    lostPackets_ = 0;
    totalIMUretrieved_ = 0;
    totalPCretrieved_ = 0;
}

//====================================================================
//  L2 commmands
//  These are commands sent to the L2
//  They are used to:
//      change the state of the L2
//      configure L2
//      reset the L2
//      request information from the L2 (such as Version info)
//      set sequence ID using for latency checks
//
//  note: when constructing the commad packets the 'tail' portion
//   of the packet has 2 fields that are undocumented
//   and their actual purpose is unknown
//   These fields have been set to match what has been observed
//   in commands sent by Unitree L2 software.
//   Some experimentation indicates these fields may not be used
//   the L2b ut are included here just in case.
//====================================================================

//--------------------------------------------------------------------
//  GetL2Params
//  This sends a request to the L2 for a Parameters packet
//  The parameters packet are not defined
//  This is part of diagnsotic to undertand what they might be
//--------------------------------------------------------------------
bool L2lidar::GetL2Params(void)
{
    // USER_CMD_VERSION_GET

    // set header
    LidarUserCtrlCmdPacket cmd;
    setPacketHeader(&cmd.header,LIDAR_COMMAND_PACKET_TYPE,
                    sizeof(LidarUserCtrlCmdPacket));

    // set data
    cmd.data.cmd_type = CMD_PARAM_GET;
    cmd.data.cmd_value = 3;  // observed usage

    // set tail
    cmd.tail.crc32 = unilidar_sdk2::crc32((uint8_t *) &cmd.data, sizeof(cmd.data));

    cmd.tail.msg_type_check = 0;
    cmd.tail.reserve[0] = 0x0;
    cmd.tail.reserve[1] = 0x0;
    cmd.tail.tail[0] = 0x00;
    cmd.tail.tail[1] = 0xff;

    // send packet
    if(!SendPacket((uint8_t *) &cmd, sizeof(LidarUserCtrlCmdPacket))) {
        return false;
    }

    return true;
}

//--------------------------------------------------------------------
//  GetWorkMode
//  The L2 parasm packet is the only know source of the current L2 workmode
//  The decodeL2Params will emit the signal that the latestWorkmode_
//  settings has been received.
//--------------------------------------------------------------------
bool L2lidar::GetWorkMode()
{
    // USER_CMD_VERSION_GET

    // set header
    LidarUserCtrlCmdPacket cmd;
    setPacketHeader(&cmd.header,LIDAR_USER_CMD_PACKET_TYPE,
                    sizeof(LidarUserCtrlCmdPacket));

    // set data
    cmd.data.cmd_type = USER_CMD_CONFIG_GET;
    int value = 0;
    cmd.data.cmd_value = value;  // value guess

    // set tail
    cmd.tail.crc32 = unilidar_sdk2::crc32((uint8_t *) &cmd.data, sizeof(cmd.data));

    cmd.tail.msg_type_check = 0;
    cmd.tail.reserve[0] = 0x0;
    cmd.tail.reserve[1] = 0x0;
    cmd.tail.tail[0] = 0x00;
    cmd.tail.tail[1] = 0xff;

    // send packet
    if(!SendPacket((uint8_t *) &cmd, sizeof(LidarUserCtrlCmdPacket))) {
        return false;
    }

    return true;
}

//--------------------------------------------------------------------
//  LidarGetVersion
//  This sends a request to the L2 for a version packet
//  The are 3 posible results:
//      The command is completely ignored (This can happen early
//          in the startup.
//      An ACK packet is sent with the status: ACK_WAIT_ERROR. This
//          occurs when the L2 is not fully initializes.  It will
//          not be fully initialize until the L2 is no longer
//          in standby and the first point cloud packet is sent.
//          If a standby command is sent after the L2 is full initialized
//          then a version packet will still be sent.
//      A VERSION packet is sent (this class will send a signal
//          to any connected subscribers).  An ACK packet will
//          also be sent by the L2 with the status ACK_SUCCESS
//--------------------------------------------------------------------
bool L2lidar::LidarGetVersion(void)
{
    // USER_CMD_VERSION_GET

    // set header
    LidarUserCtrlCmdPacket cmd;
    setPacketHeader(&cmd.header,LIDAR_USER_CMD_PACKET_TYPE,
                    sizeof(LidarUserCtrlCmdPacket));

    // set data
    cmd.data.cmd_type = USER_CMD_VERSION_GET;
    cmd.data.cmd_value = 0;  // value guess

    // set tail
    cmd.tail.crc32 = unilidar_sdk2::crc32((uint8_t *) &cmd.data, sizeof(cmd.data));

    cmd.tail.msg_type_check = 0;
    cmd.tail.reserve[0] = 0xff; // known value from recorded command
    cmd.tail.reserve[1] = 0x7f; // known value from recorded command
    cmd.tail.tail[0] = 0x00;
    cmd.tail.tail[1] = 0xff;

    // send packet
    if(!SendPacket((uint8_t *) &cmd, sizeof(LidarUserCtrlCmdPacket))) {
        return false;
    }

    return true;
}

//--------------------------------------------------------------------
//  LidarReset
//  This causes the L2 to restart (should be equivalent to power cycle)
//  Some workmode changes are only effective after a restart
//  Note:
//   The L2 restart is immediate and does not send and ACK packet
//   that confirms receipt of the command
//--------------------------------------------------------------------
bool L2lidar::LidarReset(void)
{
    // USER_CMD_RESET_TYPE

    // set header
    LidarUserCtrlCmdPacket cmd;
    setPacketHeader(&cmd.header,LIDAR_USER_CMD_PACKET_TYPE,
                    sizeof(LidarUserCtrlCmdPacket));

    // set data
    cmd.data.cmd_type = USER_CMD_RESET_TYPE;
    cmd.data.cmd_value = 1;

    // set tail
    cmd.tail.crc32 = unilidar_sdk2::crc32((uint8_t *) &cmd.data, sizeof(cmd.data));
    cmd.tail.msg_type_check = 0;
    cmd.tail.reserve[0] = 0xff; // known value from recorded command
    cmd.tail.reserve[1] = 0x7f; // known value from recorded command
    cmd.tail.tail[0] = 0x00;
    cmd.tail.tail[1] = 0xff;

    // send packet
    if(!SendPacket((uint8_t *) &cmd, sizeof(LidarUserCtrlCmdPacket))) {
        return false;
    }


    return true;
}

//--------------------------------------------------------------------
//  LidarStartRotation
//  sends a run command to the L2
//  if the L2 was in standby then it should start scanning
//  it can take >20 seconds to come up to speed and start
//  sending point cloud data
//--------------------------------------------------------------------
bool L2lidar::LidarStartRotation(void)
{
    // USER_CMD_STANDBY_TYPE, value = 0

    // set header
    LidarUserCtrlCmdPacket cmd;
    setPacketHeader(&cmd.header,LIDAR_USER_CMD_PACKET_TYPE,
                    sizeof(LidarUserCtrlCmdPacket));

    // set data
    cmd.data.cmd_type = USER_CMD_STANDBY_TYPE;
    cmd.data.cmd_value = 0;
    
    // set tail
    cmd.tail.crc32 = unilidar_sdk2::crc32((uint8_t *) &cmd.data, sizeof(cmd.data));
    cmd.tail.msg_type_check = 0;
    cmd.tail.reserve[0] = 0xff; // known value from recorded command
    cmd.tail.reserve[1] = 0x7f; // known value from recorded command
    cmd.tail.tail[0] = 0x00;
    cmd.tail.tail[1] = 0xff;

    // send packet
    if(!SendPacket((uint8_t *) &cmd, sizeof(LidarUserCtrlCmdPacket))) {
        return false;
    }

    return true;
}

//--------------------------------------------------------------------
//  LidarStopRotation
//  Send a standby command to the L2
//  This causes the L2 to stop the motors and go into low power mode
//  Issues have been seen with not being able to bring the L2
//  out of standby mode without a power cycle
//--------------------------------------------------------------------
bool L2lidar::LidarStopRotation(void)
{
    // USER_CMD_STANDBY_TYPE, value = 1

    // set header
    LidarUserCtrlCmdPacket cmd;
    setPacketHeader(&cmd.header,LIDAR_USER_CMD_PACKET_TYPE,
                    sizeof(LidarUserCtrlCmdPacket));

    // set data
    cmd.data.cmd_type = USER_CMD_STANDBY_TYPE;
    cmd.data.cmd_value = 1;  // 1 puts the L2 into standby

    // set tail
    cmd.tail.crc32 = unilidar_sdk2::crc32((uint8_t *) &cmd.data, sizeof(cmd.data));
    cmd.tail.msg_type_check = 0;
    cmd.tail.reserve[0] = 0xff; // known value from recorded command
    cmd.tail.reserve[1] = 0x7f; // known value from recorded command
    cmd.tail.tail[0] = 0x00;
    cmd.tail.tail[1] = 0xff;

    // send packet
    if(!SendPacket((uint8_t *) &cmd, sizeof(LidarUserCtrlCmdPacket))) {
        return false;
    }

    return true;
}

//--------------------------------------------------------------------
//  sendLatencyID
//  sets packet sequence ID
//  This is used in measuring the latency of packets over the UDP interface
//  Note: Command packets and User command packets use the same packet structure
//--------------------------------------------------------------------
bool L2lidar::sendLatencyID(uint32_t SeqID)
{
    // set header
    LidarUserCtrlCmdPacket cmd;
    setPacketHeader(&cmd.header,LIDAR_COMMAND_PACKET_TYPE,
                    sizeof(LidarUserCtrlCmdPacket));

    // set data
    cmd.data.cmd_type = CMD_LATENCY_TYPE;
    cmd.data.cmd_value = SeqID;

    // set tail
    cmd.tail.crc32 = unilidar_sdk2::crc32((uint8_t *) &cmd.data, sizeof(cmd.data));
    cmd.tail.msg_type_check = 0;
    cmd.tail.reserve[0] = 0;
    cmd.tail.reserve[1] = 0;
    cmd.tail.tail[0] = 0x00;
    cmd.tail.tail[1] = 0xff;

    // send packet
    if(!SendPacket((uint8_t *) &cmd, sizeof(LidarUserCtrlCmdPacket))) {
        return false;
    }

    latencyMap[SeqID] = latencyTimer.nsecsElapsed();
    return true;
}

//--------------------------------------------------------------------
//  SetL2MAC
//
//--------------------------------------------------------------------
bool L2lidar::SetL2MAC(LidarMacAddressConfig MACsettings)
{
    // set header
    LidarMacAddressConfigPacket cmd;
    setPacketHeader(&cmd.header,LIDAR_MAC_ADDRESS_CONFIG_PACKET_TYPE,
                    sizeof(LidarMacAddressConfigPacket));

    // set data
    cmd.data.mac[0] = MACsettings.mac[0];
    cmd.data.mac[1] = MACsettings.mac[1];
    cmd.data.mac[2] = MACsettings.mac[2];
    cmd.data.mac[3] = MACsettings.mac[3];
    cmd.data.mac[4] = MACsettings.mac[4];
    cmd.data.mac[5] = MACsettings.mac[5];
    cmd.data.reserve[0] = MACsettings.reserve[0];
    cmd.data.reserve[1] = MACsettings.reserve[1];

    // set tail
    cmd.tail.crc32 = unilidar_sdk2::crc32((uint8_t *) &cmd.data, sizeof(cmd.data));
    cmd.tail.msg_type_check = 0x0;
    cmd.tail.reserve[0] = 0x0;
    cmd.tail.reserve[1] = 0x0;
    cmd.tail.tail[0] = 0x00;
    cmd.tail.tail[1] = 0xff;

    // send packet
    if(!SendPacket((uint8_t *) &cmd, sizeof(LidarMacAddressConfigPacket))) {
        return false;
    }

    return true;
}

//--------------------------------------------------------------------
//  setL2UDPconfig
//  This commands set the UDP configuration used by the L2 to send
//  and receive packets through the Ehternet connection to the L2
//  Note:
//      The L2 does not use DHCP. It is really configured for
//      peer to peer operation.  This means the host Ethernet conenction
//      should also be manually configured.
//      It does not appear that the L2 uses jumbo packets.
//
//      The factory default when it is shipped are:
//          Lidar IP: 192.168.1.62
//          Lidar port: 6101 (this is the port L2 listens for packets)
//          Host IP: 192.168.1.2
//          Host port: 6201 (this is the port the L2 uses to send packets)
//
//      If you change these parameters you will also likely need to change
//      the port settings on your host to match.
//      These settings take effect on the next power cycle of the L2
//
//--------------------------------------------------------------------
bool L2lidar::setL2UDPconfig(QString hostIP, uint32_t hostPort,
                             QString LidarIP, uint32_t LidarPort)
{
    // convert IP string to numbers
    int L2ip[4] {0,0,0,0};
    int Hip[4] {0,0,0,0};

    // make sure valid ip
    // extract Lidar ip
    QStringList parts = LidarIP.split('.');
    bool result;

    L2ip[0] = parts[0].toUInt(&result);
    if(!result) return false;
    if(L2ip[0]<0 || L2ip[0]>255) return false;

    L2ip[1] = parts[1].toUInt(&result);
    if(!result) return false;
    if(L2ip[1]<0 || L2ip[1]>255) return false;

    L2ip[2] = parts[2].toUInt(&result);
    if(!result) return false;
    if(L2ip[2]<0 || L2ip[2]>255) return false;

    L2ip[3] = parts[3].toUInt(&result);
    if(!result) return false;
    if(L2ip[3]<0 || L2ip[3]>255) return false;

    // extract hist ip
    parts = hostIP.split('.');
    Hip[0] = parts[0].toUInt(&result);
    if(!result) return false;
    if(Hip[0]<0 || Hip[0]>255) return false;

    Hip[1] = parts[1].toUInt(&result);
    if(!result) return false;
    if(Hip[1]<0 || Hip[1]>255) return false;

    Hip[2] = parts[2].toUInt(&result);
    if(!result) return false;
    if(Hip[2]<0 || Hip[2]>255) return false;

    Hip[3] = parts[3].toUInt(&result);
    if(!result) return false;
    if(Hip[3]<0 || Hip[3]>255) return false;

    // Set lidar ip address
    LidarIpAddressConfigPacket config;

    config.data.lidar_ip[0] = L2ip[0];
    config.data.lidar_ip[1] = L2ip[1];
    config.data.lidar_ip[2] = L2ip[2];
    config.data.lidar_ip[3] = L2ip[3];

    config.data.user_ip[0] = Hip[0];
    config.data.user_ip[1] = Hip[1];
    config.data.user_ip[2] = Hip[2];
    config.data.user_ip[3] = Hip[3];

    config.data.lidar_port = LidarPort;
    config.data.user_port = hostPort;

    config.data.gateway[0] = 0;
    config.data.gateway[1] = 0;
    config.data.gateway[2] = 0;
    config.data.gateway[3] = 0;

    config.data.subnet_mask[0] = 255;
    config.data.subnet_mask[1] = 255;
    config.data.subnet_mask[2] = 255;
    config.data.subnet_mask[3] = 0;

    setPacketHeader(&config.header, LIDAR_IP_ADDRESS_CONFIG_PACKET_TYPE, sizeof(LidarIpAddressConfigPacket));

    // set tail
    config.tail.crc32 = unilidar_sdk2::crc32((uint8_t *) &config.data, sizeof(config.data));
    config.tail.msg_type_check = 0x00000000;
    config.tail.reserve[0] = 0x50; // known value from recorded command
    config.tail.reserve[1] = 0xda; // known value from recorded command
    config.tail.tail[0] = 0x00;
    config.tail.tail[1] = 0xff;

    // send packet
    if(!SendPacket((uint8_t *) &config, sizeof(LidarIpAddressConfigPacket))) {
        return false;
    }

    return true;
}

//--------------------------------------------------------------------
//  SetWorkMode
//  This commands only sets the workmode in the L2.  It does not restart
//  the L2.  This must be done separately for certain workmode changes
//  Note:
//      Immediately effective workmode settings that have observed are:
//          Std/Wide FOV
//          IMU disable/enable
//      Effective after restart or reset
//          2D/3D mode
//          serial/UPD mode
//          start automatically or wait for start command after power on
//
//  NOTE: This uses an undocumented command to perform this function
//  It was discovered using Wireshark to see how the
//  Unitree software sends commands
//  This is how the Unitree software sets workmode
//  The define LIDAR_PARAM_WORK_MODE_TYPE
//  was added to be consistent with the documented commands
//
//--------------------------------------------------------------------
bool L2lidar::SetWorkMode(uint32_t mode)
{
    // set header
    LidarWorkModeConfigPacket cmd;
    setPacketHeader(&cmd.header,LIDAR_PARAM_WORK_MODE_TYPE,
                    sizeof(LidarWorkModeConfigPacket));

    // set data
    cmd.data.mode = mode;

    // set tail
    cmd.tail.crc32 = unilidar_sdk2::crc32((uint8_t *) &cmd.data, sizeof(cmd.data));
    cmd.tail.msg_type_check = 0x00007fff;
    cmd.tail.reserve[0] = 0x5b; // known value from recorded command
    cmd.tail.reserve[1] = 0x5f; // known value from recorded command
    cmd.tail.tail[0] = 0x00;
    cmd.tail.tail[1] = 0xff;

    // send packet
    if(!SendPacket((uint8_t *) &cmd, sizeof(LidarWorkModeConfigPacket))) {
        return false;
    }

    return true;
}

//--------------------------------------------------------------------
//  LidarSetConfig
//  This command does not communicate with the L2
//  It saves the IP setup required to connect to the
//  L2 through UDP.
//  This will be extended to include the serial com port
//  if UART communications when is implemented
//  This must be set before the connectL2() method is used.
//--------------------------------------------------------------------
void L2lidar::LidarSetCmdConfig(QString srcIP, uint32_t srcPort,
                                QString dstIP, uint32_t dstPort)
{
    src_ip = srcIP;
    src_port = srcPort;
    dst_ip = dstIP;
    dst_port = dstPort;
    return;
}

//--------------------------------------------------------------------
//  SyncL2Clock()
//  SyncL2Clock(TimeStamp timestamp)
//  Sets the L2 clock either to the current system time or
//  the passed timestamp parameter
//--------------------------------------------------------------------
bool L2lidar::SyncL2Clock()
{
    // set header
    LidarTimeStampPacket pkt;
    setPacketHeader(&pkt.header,LIDAR_TIME_STAMP_PACKET_TYPE,
                    sizeof(LidarTimeStampPacket));

    // set data
    TimeStamp Now;
    unilidar_sdk2::getSystemTimeStamp(Now);
    pkt.data.data.sec = Now.sec;
    pkt.data.data.nsec = Now.nsec;

    // set tail
    pkt.tail.crc32 = unilidar_sdk2::crc32((uint8_t *) &pkt.data, sizeof(pkt.data));
    pkt.tail.msg_type_check = 0;
    pkt.tail.reserve[0] = 0;
    pkt.tail.reserve[1] = 0;
    pkt.tail.tail[0] = 0x00;
    pkt.tail.tail[1] = 0xff;

    // send packet
    if(!SendPacket((uint8_t *) &pkt, sizeof(LidarTimeStampPacket))) {
        return false;
    }

    mLastTimestamp = (double)Now.sec + (Now.nsec * 1.0e-9);

    return true;
}

bool L2lidar::SyncL2Clock(TimeStamp timestamp)
{
    // set header
    LidarTimeStampPacket pkt;
    setPacketHeader(&pkt.header,LIDAR_TIME_STAMP_PACKET_TYPE,
                    sizeof(LidarTimeStampPacket));

    // set data
    pkt.data.data.sec = timestamp.sec;
    pkt.data.data.nsec = timestamp.nsec;

    // set tail
    pkt.tail.crc32 = unilidar_sdk2::crc32((uint8_t *) &pkt.data, sizeof(pkt.data));
    pkt.tail.msg_type_check = 0;
    pkt.tail.reserve[0] = 0;
    pkt.tail.reserve[1] = 0;
    pkt.tail.tail[0] = 0x00;
    pkt.tail.tail[1] = 0xff;

    // send packet
    if(!SendPacket((uint8_t *) &pkt, sizeof(LidarTimeStampPacket))) {
        return false;
    }

    mLastTimestamp = (double)timestamp.sec + (timestamp.nsec * 1.0e-9);

    return true;
}

//--------------------------------------------------------------------
//  setPacketHeader
//  This is just a helper functin to help construct a packet to be
//  sent to the L2.
//--------------------------------------------------------------------
void L2lidar::setPacketHeader(FrameHeader *FrameHeader, uint32_t packet_type, uint32_t packet_size)
{
    FrameHeader->header[0] = 0x55;
    FrameHeader->header[1] = 0xaa;
    FrameHeader->header[2] = 0x05;
    FrameHeader->header[3] = 0x0a;

    FrameHeader->packet_type = packet_type;
    FrameHeader->packet_size = packet_size;
    return;
}

//--------------------------------------------------------------------
// SendUDPPacket
//--------------------------------------------------------------------
bool L2lidar::SendUDPpacket(uint8_t *Buffer,uint32_t Len)
{
    QByteArray byteArray(reinterpret_cast<const char*>(Buffer), Len);

    if(L2socket.state()!= QAbstractSocket::BoundState) {
        return false;
    }
    // write to target ip:port
    qint64 bytesWritten = L2socket.writeDatagram(byteArray,QHostAddress(dst_ip),dst_port);
    if( bytesWritten == -1) {
        stringErrorCOMM = L2socket.errorString();
        return false;
    }
    return true;
}

//--------------------------------------------------------------------
//  SendUARTPacket
//  This is not yet implememted.  It is here just as part of the skeleton
//  required when it is implemented
//--------------------------------------------------------------------
bool L2lidar::SendUARTpacket([[maybe_unused]]uint8_t *Buffer,[[maybe_unused]]uint32_t Len)
{
    return false;
}

//--------------------------------------------------------------------
//  SendPacket
//  Sends a packet to the L2 communication interace
//  This does not construct or check the buffer being sent.
//--------------------------------------------------------------------
bool L2lidar::SendPacket(uint8_t *Buffer,uint32_t Len)
{
    bool result;

    if(!UseSerial) {
        result = SendUDPpacket(Buffer,Len);
    } else {
        result = SendUARTpacket(Buffer,Len);
    }

    return result;
}

//====================================================================
//  L2 connect/disconnect
//  These are the equivalent to opening or closing the L2 communications
//  To open communications with the L2 you must:
//      LidarSetCmdConfig()
//      ConnectL2()
//
//  To close you should:
//      DisconnectL2()
//
//  Note: The interface is automatically closed when the L2lidar class
//  is deleted.
//====================================================================

//--------------------------------------------------------------------
//  ConnectL2
//  Currently only UDP Ethernet communications is supported
//  The UART implementation is only a placeholder for future implementation
//  You should call LidarSetCmdConfig() before calling this.
//  If you do not the factory default settings are used.
//
//  NOTE: If you are using WSL2 on Windows 11 then you are also likely
//  to have configured a virtual ethernet port. Both your actual physical
//  Ethernet port and the Virutal Ethernet ports should be manually
//  configured.  You can not set both to be the same IP address.
//--------------------------------------------------------------------
bool L2lidar::ConnectL2()
{
    if(!UseSerial) {
        // Receive packets from L2 UDP
        if (!L2socket.bind(QHostAddress(src_ip),src_port)) {
            stringErrorCOMM = L2socket.errorString();
            return false;
        }

        mConnected = true;

        skipIMUpackets = 100;
        skipPCpackets = 100;

        // Connect readyRead signal
        connect(&L2socket, &QUdpSocket::readyRead, this, &L2lidar::readUDPpendingDatagrams);

        // readyread may not trigger until after a datagram is first sent
        // on the socket. Unless one of the timer driven processes
        // like latency measurement, or timesync is enabled or
        // a command is sent then nothing will ever be received
        // This is an issue with Qt's implementation of readReady()
        // Sending a packet doesn't even need to be received by the L2
        // It just needs to be sent.
        GetWorkMode(); // dummy request to unblock readyRead

        // setup for latency measurements
        StartLatency();

        // setup host sync timer
        if(mL2EnableSyncHost && mL2TSsyncRate>0){
            connect(&TimerSyncTimer, &QTimer::timeout,
                    this, &L2lidar::SyncClock);
            TimerSyncTimer.start(mL2TSsyncRate);
        } else {
            TimerSyncTimer.stop();
        }

        return true;

    } else {
        // Receive packets from L2 UART
        // problem with QSerialPort support for 4M baudrate
        // need to find alternative UART support package
        // that will support across multiple platforms
        //
        // L2serial.setPortName(SerialPort);
        // if (!L2serial.open(QIODevice::ReadWrite)) {
        //     return false;
        // }

        // L2serial.setBaudRate(QSerialPort::Baud115200);
        // L2serial.setDataBits(QSerialPort::Data8);
        // L2serial.setParity(QSerialPort::NoParity);
        // L2serial.setStopBits(QSerialPort::OneStop);
        // L2serial.setFlowControl(QSerialPort::NoFlowControl);

        // Connect readyRead signal
        // connect(&L2serial, &QUdpSocket::readyRead, this, &L2lidar::readUARTpendingDatagrams);

        // setup for latency measurements
        latestLatency_.Average = -1.0; // invalid
        latestLatency_.Variance = 0.0;   // invalid
        latestLatency_.lastMeasurement = -1.0;  // inva;od
        latestLatency_.min = 999.99; // invlaid
        latestLatency_.max = -1.0; // invlaid

        connect(&LatencyTimer, &QTimer::timeout,
                this, &L2lidar::requestRTTLatencyMeasurement);

        LatencyTimer.start(1500); // first timeout is 1.5 sec
                                // to allow everything to statup

        // not yet implemented
        mConnected = false;
        return false;
    }

}

//--------------------------------------------------------------------
//  StartLatency
//--------------------------------------------------------------------
void L2lidar::StartLatency()
{
    // setup for latency measurements
    latestLatency_.Average = -1.0; // invalid
    latestLatency_.Variance = 0.0;   // invalid
    latestLatency_.lastMeasurement = -1.0;  // inva;od
    latestLatency_.min = 999.99; // invlaid
    latestLatency_.max = -1.0; // invlaid

    connect(&LatencyTimer, &QTimer::timeout,
            this, &L2lidar::requestRTTLatencyMeasurement);

    LatencyTimer.start(1000); // first timeout is 1 sec
    // to allow everything to statup
}

//--------------------------------------------------------------------
//  StopLatency
//--------------------------------------------------------------------
void L2lidar::StopLatency()
{
    disconnect(&LatencyTimer, &QTimer::timeout,
            this, &L2lidar::requestRTTLatencyMeasurement);

    LatencyTimer.stop(); // first timeout is 1 sec
    // to allow everything to statup
}

//--------------------------------------------------------------------
//  DisconnectL2
//--------------------------------------------------------------------
void L2lidar::DisconnectL2()
{
    if(!UseSerial){
        // close UDP
        L2socket.close();

        // end latency measurements
        LatencyTimer.stop();
        latestLatency_.Average = -1.0; // invalid
        latestLatency_.Variance = 0;   // invalid
        latestLatency_.lastMeasurement = -1.0;  // inva;od
        latestLatency_.min = 999.99; // invlaid
        latestLatency_.max = -1.0; // invlaid

        StopLatency();

        // end timebase syncing
        TimerSyncTimer.stop();
        disconnect(&TimerSyncTimer, &QTimer::timeout,
                this, &L2lidar::SyncClock);

    } else {
        // close UART
        //L2serial.close();

        // end latency measurements
        LatencyTimer.stop();
        latestLatency_.Average = -1.0; // invalid
        latestLatency_.Variance = 0;   // invalid
        latestLatency_.lastMeasurement = -1.0;  // inva;od
        latestLatency_.min = 999.99; // invlaid
        latestLatency_.max = -1.0; // invlaid

        disconnect(&LatencyTimer, &QTimer::timeout,
                   this, &L2lidar::requestRTTLatencyMeasurement);
    }
    mConnected = false;
}

//====================================================================
//
//  Support functions
//
//====================================================================

void L2lidar::SetL2TSsyncRate(uint32_t Rate)
{
    // nothing to do
    if(mL2TSsyncRate==Rate) return;

    mL2TSsyncRate = Rate;

    if(!mConnected) return;

    if(mL2EnableSyncHost && Rate > 0) {
            TimerSyncTimer.stop();
            TimerSyncTimer.setInterval(Rate);
            TimerSyncTimer.start();
    }
}

void L2lidar::EnableL2TSsync(bool enable)
{
    // no change, nothing to do
    if(enable==mL2EnableSyncHost) return;

    mL2EnableSyncHost = enable;

    if(!mConnected) return;

    if(enable) {
        if(mL2TSsyncRate>0) {
            TimerSyncTimer.setInterval(mL2TSsyncRate);
            TimerSyncTimer.start();
        }
    } else {
        TimerSyncTimer.stop();
    }

}

//--------------------------------------------------------------------
//  requestLatencyMeasurement
//  This is timer driven (rate 1/4 second)
//  The latency stats are measured over time
//  as long as the L2 is connected and opened
//--------------------------------------------------------------------
bool L2lidar::requestRTTLatencyMeasurement()
{
    if(!mEnableLatency)
        return false;

    // the first timeout is 1 second
    // after the connect
    // subsequent timeouts are 1/4 second
    // This allows all the activities after connect
    // to complete and not affect the measurement
    if(LatencyTimer.interval() > 250) {
        LatencyTimer.stop();
        LatencyTimer.setInterval(250);
        LatencyTimer.start();
    }
    uint32_t seq = ++SequenceID;
    if(SequenceID >10000) SequenceID = 100;
    latencyMap[seq] = latencyTimer.nsecsElapsed();
    return sendLatencyID(seq);
}

//--------------------------------------------------------------------
//  EnableLatencyMeasure
//  This enables/disables the latency measurement
//--------------------------------------------------------------------
void L2lidar::EnableLatencyMeasure(bool enable)
{
    mEnableLatency = enable;
}


//--------------------------------------------------------------------
//  UpdateEWMAStats
//  This is in updating the latency stats
//  This is frist order exponetial filter
//  that doesn't require saving 'n' samples
//  It is only an estimator using Alpha as a 'time' constant
//--------------------------------------------------------------------
void L2lidar::UpdateEWMAStats(double alpha,
                     double Xnew,
                     double& Xmean,
                     double& Xvariance
                     )
{
    double delta = Xnew - Xmean;

    // Update mean (EWMA)
    Xmean += alpha * delta;

    // Update variance (Welford-style EWMA)
    Xvariance = (1.0 - alpha) * Xvariance
                + alpha * delta * (Xnew - Xmean);
}

//--------------------------------------------------------------------
//  ConvertL2data2pointcloud(Frame& frame, bool Frame3D, bool IMUadjust)
//  This returns the latest point cloud Frame
//      frame is (QVector<PCpoint>)
//      Frame3D true process latest 3D packet
//              false process latest 2D packet
//      IMUadjust   true  Applies pose from IMU to point cloud data
//                  false Do not applies IMU pose correction
//      return  true if successful
//              false if point cloud packet does not exit
//              false if IMUAdjust is true and not valid IMU data
//
//  Change in the unitree_lidar_utilities.h to both PointUnitree and
//  PointCloudUnitree related to timestamps.
//--------------------------------------------------------------------
bool L2lidar::ConvertL2data2pointcloud(Frame& frame, bool Frame3D, bool IMUadjust)
{
    LidarImuDataPacket Imu;
    double time;
    bool adjustWithIMU {false};
    Quaternion Quat;

    // Retrieve packet
    unilidar_sdk2::PointCloudUnitree cloud;

    bool UseSystemTime = !(mL2EnableSyncHost && enableL2TimeStampFix);

    if(Frame3D) {
        // get latest 3D packet
        LidarPointDataPacket packet = Pcl3Dpacket();
        if(packet.header.header[0] == (uint8_t)0){
            // there is no latest packet
            return false;
        }

        unilidar_sdk2::parseFromPacketToPointCloud(
            cloud, packet, UseSystemTime, 0, 100);
        time = (double)packet.data.info.stamp.sec + (double)packet.data.info.stamp.nsec * 1.0e-9;
    } else {
        // get latest 2D packet
        Lidar2DPointDataPacket packet = Pcl2Dpacket();
        if(packet.header.header[0] == (uint8_t)0){
            // there is no latest packet
            return false;
        }
        // if !mL2EnableSyncHost then use system time (now)
        unilidar_sdk2::parseFromPacketPointCloud2D(
            cloud, packet, UseSystemTime, 0, 100);
        time = (double)packet.data.info.stamp.sec + (double)packet.data.info.stamp.nsec * 1.0e-9;
    }

    if(IMUadjust) {
        // check if latest IMU packet is within 10 msec
        double IMUtime;
        Imu = imu();
        // do not count this as retrieval
        totalIMUretrieved_--;
        if(Imu.header.header[0] == (uint8_t)0){
            // there is no latest packet
            return false;
        }

        IMUtime = (double)Imu.data.info.stamp.sec +(double)Imu.data.info.stamp.nsec * 1e-9;
        if(abs(time-IMUtime) < 0.08) {
            adjustWithIMU = true;
            Quat.w = Imu.data.quaternion[0];
            Quat.x = Imu.data.quaternion[1];
            Quat.y = Imu.data.quaternion[2];
            Quat.z = Imu.data.quaternion[3];
        } else {
            // if IMU adjustment requested and time differnce to high
            // then drop packet
            return false;
        }
    }

    // note:  The changes to the unitree structures was to allow better
    // time stamp precision.
    // This necesitated a change to the
    frame.reserve(cloud.points.size());
    long long adjustedTime {0};  // adjusted time is in nanoseconds since epoch

    adjustedTime = cloud.stamp;

    // p.time is a double containing relative time to the start of frame in seconds
    // The time field in frame is long long and is nanoseconds since epoch
    for (auto& p : cloud.points)
    {
        long long actualtime;

        if(adjustWithIMU) {
            rotateByQuaternion(Quat,p.x,p.y,p.z);
        }
        actualtime = (long long) (p.time*1.0e9+0.5) + adjustedTime;

        frame.push_back({
            p.x,
            p.y,
            p.z,
            p.intensity,
            p.range,
            actualtime,
            p.ring
        });
    }

    return true;
}
