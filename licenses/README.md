l2lidar_node
============

**updated 2026-06-13**
============

Overview
============

l2lidar_ros2 is a standalone ROS 2 Jazzy driver node for the **Unitree L2 4D LiDAR** sensor.  It provides a high-performance interface between the Unitree L2 hardware and ROS 2 by leveraging a Qt 6.10 UDP backend (`L2lidar` class) for deterministic packet handling, timestamp synchronization, and decoding.

This package publishes **3D point cloud** and **IMU data** using standard ROS 2 message types and is intended for robotics perception, mapping, and localization applications.

The node runs without any Qt GUI components and is designed to be launched independently and visualized using RViz2.

This application utlitizes the L2lidarClass for the driver interface. This driver is also available separately at:

https://github.com/markgol/L2lidarClass



NOTE: This replaces the L2lidar_ros2 (depracated) github repo: https://github.com/markgol/l2lidar_ros2

* * *

Features
--------

* Native ROS 2 Jazzy node (C++20)

* Qt 6.10 UDP backend (Core + Network only, no GUI, minimal number of shared libraries needed (5)  for porting)

* Publishes:
  
  * `/points` — `sensor_msgs/PointCloud2`
  
  * `/imu/data` — `sensor_msgs/Imu`
  
  * static transforms
    
    * frame_id -> imu_frame_id (l2 lidar frame -> l2 imu frame)
    
    * robot_id -> frame_id (optional, example would be base_link -> l2lidar_frame)

* Per-point timestamps supported ( recorded relative to frame time stamp)

* Host to LiDAR time base synchronization

* supports standby startup mode for the L2

* RViz2 visualization support (distance/range coloring)

* Designed for Ubuntu 24.04 + ROS 2 Jazzy

* Target platforms: Raspberry Pi 5 (ARM64), Jetson Orin Nano (ARM64) and x86_64

* * *

Architecture
------------

Unitree L2 LiDAR (UDP Ethernet)

        |

L2lidar (Qt 6.10 backend)

        |

l2lidar_node

        |

        +--> /points      (PointCloud2, topic ID set in the config file)

        +--> /imu/data    (Imu, topic ID set in the config file)

        +--> /tf_static

                base_link -> l2lidar_frame (only if enabled, names set in config file)

                l2lidar_frame -> l2lidar_imu (names set in config file)



The node uses Qt’s networking and event system for UDP packet reception and ROS 2 publishers for message dissemination. No Qt GUI or ROS GUI dependencies are used.

****

## EXECUTABLES

The exectuable for gcc_64 (Ubuntu x86_64) has been tested under Windows 11 through WSL2 running Ubuntu24.04.

The exectuable for gcc_arm64(Ubuntu ARM64) has been tested under Ubuntu 24.04 on a RPI5 and Jetson Orin Nano

These can now run without the installation of Qt.

There is no executable to run under Windows 11 since Windows 11 does not directly support ROS2 yet.

If you are only going to use the executables they can be found at:

https://github.com/markgol/l2lidar_node/tree/main/executables

* * *

Topics
------

| Topic        | Message Type                     | Description                                                                                     |
| ------------ | -------------------------------- | ----------------------------------------------------------------------------------------------- |
| `/points`    | `sensor_msgs/PointCloud2`        | 3D point cloud with intensity, time, and optional range field                                   |
| `/imu/data`  | `sensor_msgs/Imu`                | Orientation, angular velocity, and linear acceleration                                          |
| `/tf_static` | `geometry_msgs/TransformStamped` | Static transform between LiDAR and IMU frames<br/>optional static transform robot base to LiDar |

* * *

Services
--------

| Service    | Type                   | Description                                                                                                                                                                                                                                                                                           |
| ---------- | ---------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `~/enable` | `std_srvs/srv/SetBool` | Live start / stop of the L2's motor and data stream without restarting the node. `data: true` calls `LidarStartRotation()` (cloud rate ramps up over ~20 s; IMU resumes within ~1 s). `data: false` calls `LidarStopRotation()` and pauses the watchdog so the deliberate gap is not read as a fault. |

Example:

```bash
ros2 service call /l2lidar_node/enable std_srvs/srv/SetBool "{data: false}"
ros2 service call /l2lidar_node/enable std_srvs/srv/SetBool "{data: true}"
```

* * *

Parameters
----------

| Parameter                   | Type   | Default                            | Description                                                                     |
| --------------------------- | ------ | ---------------------------------- | ------------------------------------------------------------------------------- |
| `l2_ip`                     | string | 192.168.1.62<br/>(factory default) | LiDAR IP address                                                                |
| `l2_port`                   | int    | 6101<br/>(factory default)         | LiDAR UDP port                                                                  |
| `host_ip`                   | string | 192.168.1.2<br/>(factory default)  | Host IP address                                                                 |
| `host_port`                 | int    | 6201<br/>(factory default)         | Host UDP port                                                                   |
| frame3d                     | bool   | true                               | point cloud data is 3D not 2D                                                   |
| imu_adjust                  | bool   | true                               | Apply IMU pose correction to cloud points before publishing (Dynamic)           |
| `enable_l2_time_correction` | bool   | `true`                             | Enable LiDAR timestamp correction                                               |
| `enable_l2_host_sync`       | bool   | `true`                             | Enable host → LiDAR time sync                                                   |
| `l2_sync_rate_ms`           | int    | `50`                               | Sync rate in milliseconds                                                       |
| `enable_latency_measure`    | bool   | `false`                            | Enable latency measurement                                                      |
| disable_base_link_pub       | bool   | false                              | Disbables publishing of the static TF for robot_id -> frame_id if true          |
| `frame_id`                  | string | `l2lidar_frame`                    | Point cloud frame ID                                                            |
| `imu_frame_id`              | string | `l2lidar_imu`                      | IMU frame ID                                                                    |
| robot_id                    | string | base_link                          | Robot origin frame                                                              |
| robot_x                     | float  | 0.0                                | x offset from lidar position                                                    |
| robot_y                     | float  | 0.0                                | y offset from lidar position                                                    |
| robot_z                     | float  | 0.0                                | z offset from lidar position                                                    |
| `enable_IMU_publishing`     | bool   | `false`                            | true - publish IMU data                                                         |
| accel_x_covar               | double | 0.01                               | noise variance for x axis accelerometer                                         |
| accel_y_covar               | double | 0.01                               | noise variance for y axis accelerometer                                         |
| accel_z_covar               | double | 0.01                               | noise variance for z axis accelerometer                                         |
| gyro_x_covar                | double | 0.000025                           | noise variance for x axis gyroscope                                             |
| gyro_y_covar                | double | 0.000025                           | noise variance for y axis gyroscope                                             |
| gyro_z_covar                | double | 0.0000002                          | noise variance for z axis gyroscope                                             |
| aggregateNframes            | int    | 38                                 | Number of L2 frames to aggregate for publishing (Dynamic)                       |
| EnableCalRangeOVR           | bool   | false                              | Override internal L2 Range calibration (Dynamic)                                |
| calRangeScale               | float  | 0.000978                           | Range Scale override value (Dynamic)                                            |
| calRangeBias                | float  | -365.625                           | Range Bias override value (Dynamic)                                             |
| watchdog_timeout_ms         | int    | 35000                              | max time without data from L2 in msec                                           |
| point_cloud_topic_id        | string | /points                            | Topic ID for point cloud publishing                                             |
| imu_topic_id                | string | /imu/data                          | Topic IF for IMU publishing                                                     |
| standby_on_powerup_enabled  | bool   | false                              | Turn off watchdog timer at startup so L2 starting in standy mode won't timeout. |

Parameters indentified as (Dynamic) can be set using ros2 command like:

```
ros2 param set l2lidar_node EnableCalRangeOVR true
ros2 param set l2lidar_node calRangeBias -525.5
ros2 param set l2lidar_node imu_adjust false
```

Note: realtime overrides of parameters are not persistent. If you want persistence you need to change the config yaml file.

* * *

Build Requirements
------------------

* Ubuntu 24.04

* ROS 2 Jazzy

* Qt 6.10.2 or newer (Core + Network only)

* CMake ≥ 3.22

* C++20 or later

* colcon (minimum)

* * *

Installation
------------

### 1. Install ROS 2 Jazzy

Follow the official ROS 2 Jazzy installation instructions for Ubuntu 24.04.

Ensure ROS is sourced:

`source /opt/ros/jazzy/setup.bash`

* * *

### 2. Install of Qt 6.10.2 (optional if using prebuilt exectuable)

Install Qt 6.10.2 using the Qt Online Installer:

`/opt/Qt/6.10.2/gcc_64`

Make sure Qt6 Core and Network modules are installed.

* * *

### 3. Create workspace

(Edit to match your ROS2 workspace folder and repo source)

```
mkdir -p ~/ros2_ws/src cd ~/ros2_ws/src
git clone <your_repo_url> l2lidar_node
```

* * *

### 4. Build

(Edit to match your ROS2 workspace folder)

```
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select l2lidar_node
```

Then source:

```
`source install/setup.bash`
```

### 5. Prebuilt exectuables

You still must have ros2 jazzy installed.  This is assuming you are running Ubuntu 24.04.
They are in the excutables directories:
excutables/
    gcc_arm64 (for the ARM64 or aarch64 hardware platforms such as the Raspberry PI or Nvidia Jetson Orin)
    gcc_64 (for the x86_64 hardware platforms)

These contain a copy of the executable and required Qt libraries that are needed to run the l2lidar_node app as tar.gz files.

They should be extracted to the appropriate folder/subfolders of your target installation folder.

No specific install steps are required.  If you are setting up a bash file to run the node remember to include sourcing the ros2 installation, your DDS if not defaulted, and your ROS domain ID if not defaulted.  This may also already be done in the .bashrc file for the user and looks like:

```
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/jazzy/setup.bash
```

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
    lib/libQt6* shared library files
    config/l2lidar_node.yaml
    rviz/rvizl2lidar.rviz

* * *

RViz2 Visualization
-------------------

Start RViz2:

`rviz2`

Load the provided configuration:

`rviz2 -d rviz/rvizl2lidar.rviz`

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

The optional base_lik --> l2lidar_frame is set in the config yaml file and represents the offset from the robot base to the L2 robot location.  The center of the mouting surface of the L2 is 0.0, 0.0, 0.0.  of the lidar data.  It includes the 44.5mm offset from the mounting surface to the lidar scan plane.  This does not corresond to any of the mounting hole.  Note: the mouting holes are offset by 22.5 degrees from the x and y axis origins. 

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

* The is no GUI, you must use the .yaml configuration file to set parameters

* Static TF only (no dynamic motion TF)

* Note: RViz IMU display plugin is not available in ros2 Jazzy; visualization is via TF and point cloud only

* Requires Qt 6.10 or later due to UDP reliability fixes (Qt 6.4 is not supported). This isonly required if your are compiling and building the app

* * *

Design Goals
------------

* Deterministic timing

* Zero packet loss

* Minimal dependencies

* No GUI coupling

* High throughput (≈250 Hz IMU, ≈216 Hz point cloud frames, 30HZ host->L2 timesync)

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

| Parameter         | type  | range            |
| ----------------- | ----- | ---------------- |
| imu_adjust        | bool  | true, false      |
| aggregateNframes  | int   | 0 - 4000         |
| EnableCalRangeOVR | bool  | true,false       |
| calRangeScale     | float | 0.002 - 0.000250 |
| calRangeScale     | float | 0.0 to -1000.0   |

Note: When using the ROS2 param set commands float values must have a decimal point or an type error will be generated.  As an example: 100 must be 100.0.

**0.3.2** - Added services to start/stop rotation of L2, contributed by https://github.com/pondersome
Updated cmakelist.txt file to allow proper linkage related to Qt 6.10 
Updated to L2lidarClass V1.3.0

**0.3.3** - Corrected statement on the mounting plane and the origin of the point cloud frame. There are no software changes in this revision.

**0.3.4** - Updated to V1.3.3 of the L2lidarClass, this corrects a bug in the timestamp correction from the L2.
Added enable/disable of IMU publishing

**0.3.5** - Added enable/disable of TF base_link (robot_id to frame_id) publishing.
Added configuration parameters for accelerometer and gyroscopic covariances.
Added initialization of the accelerometer and gyroscopic covariances for the IMU messages.

**0.3.6** - No changes to the source code.  Changes to the CMakeList.txt file which build a install directory for the distributable app. This change allows the executable disto to be run without the installation of Qt.

**0.3.7** - Code cleanup: renaming l2lidar_node class members variable to end with _ , comments updates. Disable/enable watchdog timeout for supporting L2 startup in standby.  Added topic IDs to be set in the config .yaml file. Documentation cleanup and updated to reflect current operation.  Added the license folder that was lost in V0.3.6.

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
