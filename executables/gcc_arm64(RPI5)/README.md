l2lidar_ros2
============

**updated 2026-02-23**

Overview
============

l2lidar_ros2` is a standalone ROS 2 Jazzy driver node for the **Unitree L2 4D LiDAR** sensor.  
It provides a high-performance interface between the Unitree L2 hardware and ROS 2 by leveraging a Qt 6.10 UDP backend (`L2lidar` class) for deterministic packet handling, timestamp synchronization, and decoding.

This package publishes synchronized **3D point cloud** and **IMU data** using standard ROS 2 message types and is intended for robotics perception, mapping, and localization applications.

The node runs without any Qt GUI components and is designed to be launched independently and visualized using RViz2.

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

l2lidar_ros2 node

        |

        +--> /points      (PointCloud2)

        +--> /imu/data    (Imu)

        +--> /tf_static



The node uses Qt’s networking and event system for UDP packet reception and ROS 2 publishers for message dissemination. No Qt GUI or ROS GUI dependencies are used.

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

| Parameter                   | Type   | Default                            | Description                                     |
| --------------------------- | ------ | ---------------------------------- | ----------------------------------------------- |
| `l2_ip`                     | string | 192.168.1.62<br/>(factory default) | LiDAR IP address                                |
| `l2_port`                   | int    | 6101<br/>(factory default)         | LiDAR UDP port                                  |
| `host_ip`                   | string | 192.168.1.2<br/>(factory default)  | Host IP address                                 |
| `host_port`                 | int    | 6201<br/>(factory default)         | Host UDP port                                   |
| `enable_l2_time_correction` | bool   | `true`                             | Enable LiDAR timestamp correction               |
| `enable_l2_host_sync`       | bool   | `true`                             | Enable host → LiDAR time sync                   |
| `l2_sync_rate_ms`           | int    | `50`                               | Sync rate in milliseconds                       |
| `enable_latency_measure`    | bool   | `false`                            | Enable latency measurement                      |
| `frame_id`                  | string | `l2lidar_frame`                    | Point cloud frame ID                            |
| `imu_frame_id`              | string | `l2lidar_imu`                      | IMU frame ID                                    |
| `pointcloud_queue_size`     | int    | `10`                               | Point cloud publisher queue size                |
| `imu_queue_size`            | int    | `10`                               | IMU publisher queue size                        |
| aggregateNframes            | int    | 38                                 | NUmber of L2 frames to aggregate for publishing |

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

`mkdir -p ~/ros2_ws/src cd ~/ros2_ws/srcgit clone <your_repo_url> l2lidar_ros2`

* * *

### 4. Build

(Edit to match your ROS2 workspace folder)

`cd ~/ros2_ws source /opt/ros/jazzy/setup.bashcolcon build --packages-select l2lidar_ros2`

Then source:

`source install/setup.bash`

* * *

Running the Node
----------------

`ros2 run l2lidar_ros2 l2lidar_node`

Or using a launch file:

`ros2 launch l2lidar_ros2 l2lidar.launch.py`

* * *

RViz2 Visualization
-------------------

Start RViz2:

`rviz2`

Load the provided configuration:

`rviz2 -d share/l2lidar_ros2/rviz/l2lidar.rviz`

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

Static transform is published:

`l2lidar_frame  -->  l2lidar_imu`

This can be adjusted in code if the physical offset is known.

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

* Hardware-accurate timestamps

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

* * *

Licenses
-------

l2lidar_ros2 license, see license file: l2lidar_ros2 LICENSE.txt

Qt license, see license file: Qt LICENSE LGPL.txt

Unitree license, see license file: Unitree BSD-3 LICENSE.txt

* * *

Maintainer
----------

https://github.com/markgol/l2lidar_ros2  
Support and contact via GitHub repository issues.
