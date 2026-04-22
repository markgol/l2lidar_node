l2lidar_node
============

**updated 2026-04-21**
============

Overview
============

l2lidar_ros2 is a standalone ROS 2 Jazzy driver node for the **Unitree L2 4D LiDAR** sensor.  It provides a high-performance interface between the Unitree L2 hardware and ROS 2 by leveraging a Qt 6.10 UDP backend (`L2lidar` class) for deterministic packet handling, timestamp synchronization, and decoding.

This package publishes synchronized **3D point cloud** and **IMU data** using standard ROS 2 message types and is intended for robotics perception, mapping, and localization applications.

The node runs without any Qt GUI components and is designed to be launched independently and visualized using RViz2.

This application utlitizes the L2lidarClass for the driver interface. This driver is also available separately at:

https://github.com/markgol/L2lidarClass



NOTE: This replaces the L2lidar_ros2 (depracated) github repo: https://github.com/markgol/l2lidar_ros2

* * *

Features
--------

* Native ROS 2 Jazzy node (C++20)

* Qt 6.10 UDP backend (Core + Network only, no GUI)

* Publishes:
  
  * `/points` — `sensor_msgs/PointCloud2`
  
  * `/imu/data` — `sensor_msgs/Imu`

* Deterministic IMU and point cloud synchronization

* Per-point timestamps supported

* Host ↔ LiDAR timebase synchronization

* Static TF transform between LiDAR and IMU frames

* RViz2 visualization support (distance/range coloring)

* Designed for Ubuntu 24.04 + ROS 2 Jazzy

* Target platforms: Raspberry Pi 5 (ARM64) and x86_64

* * *

Architecture
------------

Unitree L2 LiDAR (UDP Ethernet)

        |

L2lidar (Qt 6.10 backend)

        |

l2lidar_node

        |

        +--> /points      (PointCloud2)

        +--> /imu/data    (Imu)

        +--> /tf_static

                base_link -> l2lidar_frame (names set in config file)

                l2lidar_frame -> l2lidar_imu (names set in config file)



The node uses Qt’s networking and event system for UDP packet reception and ROS 2 publishers for message dissemination. No Qt GUI or ROS GUI dependencies are used.

****

## EXECUTABLES

The exectuable for gcc_64 (Ubuntu x86_64) has been tested under Windows 11 through WSL2 running Ubuntu24.04.

The exectuable for aarch64 (gcc_arm64) has been tested under Ubuntu 24.04 on a RPI5.

There is no executable to run under Windows 11 since Windows 11 does not directly support ROS2.

If you are only going to use the executables they cna be found at:

https://github.com/markgol/l2lidar_node/tree/main/executables

* * *

Topics
------

| Topic        | Message Type                     | Description                                                   |
| ------------ | -------------------------------- | ------------------------------------------------------------- |
| `/points`    | `sensor_msgs/PointCloud2`        | 3D point cloud with intensity, time, and optional range field |
| `/imu/data`  | `sensor_msgs/Imu`                | Orientation, angular velocity, and linear acceleration        |
| `/tf_static` | `geometry_msgs/TransformStamped` | Static transform between LiDAR and IMU frames                 |

* * *

Parameters
----------

| Parameter                   | Type   | Default                            | Description                                                 |
| --------------------------- | ------ | ---------------------------------- | ----------------------------------------------------------- |
| `l2_ip`                     | string | 192.168.1.62<br/>(factory default) | LiDAR IP address                                            |
| `l2_port`                   | int    | 6101<br/>(factory default)         | LiDAR UDP port                                              |
| `host_ip`                   | string | 192.168.1.2<br/>(factory default)  | Host IP address                                             |
| `host_port`                 | int    | 6201<br/>(factory default)         | Host UDP port                                               |
| frame3d                     | bool   | true                               | point cloud data is 3D not 2D                               |
| imu_adjust                  | bool   | true                               | Apply IMU pose correction to cloud points before publishing |
| `enable_l2_time_correction` | bool   | `true`                             | Enable LiDAR timestamp correction                           |
| `enable_l2_host_sync`       | bool   | `true`                             | Enable host → LiDAR time sync                               |
| `l2_sync_rate_ms`           | int    | `50`                               | Sync rate in milliseconds                                   |
| `enable_latency_measure`    | bool   | `false`                            | Enable latency measurement                                  |
| `frame_id`                  | string | `l2lidar_frame`                    | Point cloud frame ID                                        |
| `imu_frame_id`              | string | `l2lidar_imu`                      | IMU frame ID                                                |
| robot_id                    | string | base_link                          | Robot origin frame                                          |
| robot_x                     | float  | 0.0                                | x offset from lidar position                                |
| robot_y                     | float  | 0.0                                | y offset from lidar position                                |
| robot_z                     | float  | 0.0                                | z offset from lidar position                                |
| `enable_IMU_publishing`     | bool   | `false`                            | true - publish IMU data                                     |
| aggregateNframes            | int    | 38                                 | NUmber of L2 frames to aggregate for publishing             |
| EnableCalRangeOVR           | bool   | false                              | Override internal L2 Range calibration                      |
| calRangeScale               | float  | 0.000978                           | Range Scale override value                                  |
| calRangeBias                | float  | -365.625                           | Range Bias override value                                   |
| watchdog_timeout_ms         | int    | 35000                              | max time without data from L2 in msec                       |

* * *

Build Requirements
------------------

* Ubuntu 24.04

* ROS 2 Jazzy

* Qt 6.10.2 or newer (Core + Network only)

* CMake ≥ 3.22

* C++20

* colcon

* * *

Installation
------------

### 1. Install ROS 2 Jazzy

Follow the official ROS 2 Jazzy installation instructions for Ubuntu 24.04.

Ensure ROS is sourced:

`source /opt/ros/jazzy/setup.bash`

* * *

### 2. Install Qt 6.10.2

Install Qt 6.10.2 using the Qt Online Installer:

`/opt/Qt/6.10.2/gcc_64`

Make sure Qt6 Core and Network modules are installed.

* * *

### 3. Create workspace

(Edit to match your ROS2 workspace folder and repo source)

`mkdir -p ~/ros2_ws/src cd ~/ros2_ws/srcgit clone <your_repo_url> l2lidar_node`

* * *

### 4. Build

(Edit to match your ROS2 workspace folder)

`cd ~/ros2_ws source /opt/ros/jazzy/setup.bashcolcon build --packages-select l2lidar_node

Then source:

`source install/setup.bash`

* * *

Running the Node
----------------

You should edit this to point to where you have the yaml configuration file.

`run l2lidar_node l2lidar_node --ros-args --params-file \home\robot\SoftwareDev\ros2_ws\src\l2lidar_node\bin\gcc_64\config/l2lidar_node.yaml`

Or using a launch file:

`ros2 launch l2lidar_node l2lidar.launch.py`

Or from terminal in folder with exectuable:

`./l2lidar_node --params-file ./config/l2lidar_node.yaml`

This assumes are you in the folder with the following files:
    l2lidar_node

    libQt6Core.so.6.10.2

    libQt6Network.so.6.10.2

    config/l2lidar_node.yaml

* * *

RViz2 Visualization
-------------------

Start RViz2:

`rviz2`

Load the provided configuration:

`rviz2 -d share/l2lidar_node/rviz/l2lidar.rviz`

Recommended settings:

* Fixed Frame: `l2lidar_frame`

* PointCloud2:
  
  * Topic: `/points`
  
  * Color Transformer: `Channel`
  
  * Channel Name: `range` or `time`
  
  * Autoscale: `false`
  
  * Min: `0.0`
  
  * Max: `5.0`

* IMU: via TF visualization

* * *

Coordinate Frames
-----------------

There are 3 coordinate frames used: imu, lidar, robot

The orientation (x,y,z axis) are the same in all 3 frames.

Static transform is published:

`l2lidar_frame  -->  l2lidar_imu

`base_link --> l2lidar_frame

The l2lidar_frame --> l2lidar_imu is set in the source to match the Unitree L2 published spec.

The base_lik --> l2lidar_frame is set in the config yaml file and represents the offset from the robot base to the L2 robot location.  The mouting surface of the L2 is not 0.0, 0.0, 0.0.  See the Unitree L2 published specification for the offsets.

* * *

Shutdown Behavior
-----------------

The node supports:

* Clean shutdown on connection failure

* Signal-safe shutdown (SIGINT / SIGTERM)

* Watchdog timeout handling

* Proper Qt and ROS2 event loop exit

* * *

Debugging
---------

Run under debugger from QtCreator:

* Configure kit with Qt 6.10

* Source ROS2 environment

* Run target: `l2lidar_node`

* Set breakpoints in:
  
  * `onImuReceived()`
  
  * `onPointCloudReceived()`

* * *

Known Limitations
-----------------

* No GUI configuration (command-line only)

* Static TF only (no dynamic motion TF yet)

* RViz IMU display plugin is not available in Jazzy; visualization is via TF and point cloud only

* Requires Qt 6.10 due to UDP reliability fixes (Qt 6.4 is not supported)

* * *

Design Goals
------------

* Deterministic timing

* Zero packet loss

* Minimal dependencies

* No GUI coupling

* High throughput (≈250 Hz IMU, ≈216 Hz point cloud frames)

* Clean shutdown

* Host synced timestamps

* * *

Version
-------

**0.1.0** – Initial functional driver with synchronized IMU and point cloud publishing.  This is only the intial release and does not include a prebuilt executable.  That is planned for the 0.2.0 release

**0.2.0** - Added aggregation ofL2 frames for publishing

This is needed to align point cloud publishing to the requirements for LIO-SAM methodology

Changed point time from float to double.

Changed point time to eliminate truncation errors.

The L2lidar class sources moved to their own directories.

The L2lidar class was updated to improve computational accuracy and time stamp handling

**0.2.1** - Included parameters in the config.yaml frame3d and imu_adjust in the implementation

This allows the user to specify that 3D frames or 2D frames are to be published.  It also allows the user to specify the pose (rotation) correction is to be applied before the point cloud data is published.

**0.2.2** - added static transform publishing, renamed from l2lidar_ros2 to l2lidar_node

**0.2.3** - Made publishing IMU data optional, changed ROS2 QOS publisher settins to SensorData

This specifies the static fixed transforms.  We already know the l2idar_frame -> l2lidar_imu.  This also adds the transform robot origin frame (base_link) -> l2lidar_frame.  This implies the L2 is at a fixed location on robot.

**0.3.0** - Added override of L2 Range calibration parameters

Added dynamic settings for certain parameters:

| Parameter        | type  | range            |
| ---------------- | ----- | ---------------- |
| imu_adjust       | bool  | true, false      |
| aggregateNframes | int   | 0 - 4000         |
| calRangeScale    | float | 0.002 - 0.000250 |
| calRangeScale    | float | 0.0 to -1000.0   |

Note: When using the ROS2 param set commands float values must have a decimal point or an type error will be generated.  As an example: 100 must be 100.0.

* * *

Licenses
-------

l2lidar_node license, see license file: l2lidar_node LICENSE.txt

Qt license, see license file: Qt LICENSE LGPL.txt

Unitree license, see license file: Unitree BSD-3 LICENSE.txt

* * *

Maintainer
----------

https://github.com/markgol/l2lidar_node  
Support and contact via GitHub repository issues.
