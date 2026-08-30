L2lidar Class  – Framework
===============================================================

**Author:** Mark Stegall  
**License:** GPL-3.0  
**Platform:** Qt6.11.2 (Qt Creator), C++  
**Hardware:** Unitree L2 LiDAR (Ethernet / UDP)

--------

**L2lidar** is the core class that implements a complete UDP communications pipeline, packet reconstruction, decoding, and synchronization logic without any UI dependencies. It is designed to be embedded into GUI or headless applications (Qt timers, ROS bridges, visualization tools, etc.).

It is intended to be used by applications that need to interface to the Untiree L2 4D LiDAR hadrware.

Exampe Apps that use the L2lidar class:

**L2Diagnostic** is a platform-independent C++ framework for validating and diagnosing the operation of the Unitree L2 LiDAR sensor over its Ethernet (UDP) interface.  It relies on the L2idar class to control and recieve data from the L2.

**l2lidar_ros2** is a platform-independent C++ framework for a ROS2 publishing node for IMU and point cloud data from the L2.



**Goal**

The project was developed to replace reliance on undocumented vendor libraries and POSIX-dependent archive binaries provided by Unitree Robotics, enabling transparent packet decoding, timestamp correction, latency measurement, and point-cloud extraction in a portable Qt environment.



* * *

Motivation
----------

Unitree’s official SDK distributes:

* Header files

* Example applications

* Precompiled `.a` archive libraries

However:

* The archive library source is unavailable

* It relies on POSIX I/O

* Debugging and porting are difficult

* Protocol behavior is undocumented

This project reconstructs and documents the L2 protocol behavior while providing:

* Robust packet handling

* Error detection and packet loss statistics

* Timestamp synchronization to host time

* Quaternion-based spatial correction

* Extracted point cloud frames for downstream processing

* * *

Key Features
------------

**UDP Packet Reconstruction**

* Combines multiple datagrams into complete L2 packets

* Detects lost and malformed packets

**Protocol Decoding**

* 3D point cloud packets

* 2D scan packets

* IMU packets

* Version, timestamp, MAC, IP, work mode, and parameter packets

* ACK packets

**Point Cloud Conversion**

* `ConvertL2data2pointcloud()` produces clean `Frame` (vector of `PCpoint`)

* Optional IMU-based quaternion spatial correction

* Override of internal Range calibration 

* Includes precomputed range (meters) for each point

**Time Synchronization**

* Sync L2 clock to host system time

* Adjustable timestamp scale correction

* Host-driven periodic resynchronization

**Field notes (fw 2.8.11.1, build 20205-07-15 measured)**

* The slow-clock factor is not exactly 2: one unit measured ~1.988 at cold start, drifting to ~1.991 over a 20 minute warm-up (issue #3)

* The first cloud packet(s) after stream start carry `dirty_index` = 100.0 (initialization value; nothing is emitted during the 15 to 30 s motor spin-up); settled clean-unit values are roughly 0.7 median with spikes to about 8

* **Erratum** for the vendor reference doc (Unitree_README_Verbatim.md, kept verbatim from Unitree): Its sample output labels the IMU quaternion `(x, y, z, w)`, but the wire order is `(w, x, y, z)`; see issue #2 for the verification

**Latency Measurement**

* RTT latency using sequence IDs

* EWMA statistics (average, variance, min, max)

* Non-blocking implementation

**Command & Control**

- Start/stop rotation

- Reset device

- Set work mode

- Configure UDP IP/port

- Set MAC address

- Retrieve firmware version and parameters

**Calibration overrides and range correction**

- Override of internal calibration settings for point cloud conversion

- Piecewise cubic spline correction for range values

- Allows usage of a calibration file

**Thread-Safe Data Access**

* All decoded packets protected by `QMutex`

* Safe access from GUI or worker threads

**Qt-Native Design**

* Uses `QUdpSocket`, `QTimer`, `QObject` signals

* No UI code included

* Ready for integration into Qt or ROS2 applications

* * *

See L2lidar class Vx.y.z.pdf for more details
---------------------
