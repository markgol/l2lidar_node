l2lidar_node
============

**updated 2026-09-01**
============

Overview
============

l2lidar_ros2 is a standalone ROS 2 Jazzy driver node for the **Unitree L2 4D LiDAR** sensor.  It provides a high-performance interface between the Unitree L2 hardware and ROS 2 by leveraging a Qt 6.11 UDP backend (`L2lidar` class) for deterministic packet handling, timestamp synchronization, and decoding.

This package publishes **3D point cloud** and **IMU data** using standard ROS 2 message types and is intended for robotics perception, mapping, and localization applications.

The node runs without any Qt GUI components and is designed to be launched independently and visualized using RViz2.

This application utlitizes the L2lidarClass for the driver interface. This driver is also available separately at:

https://github.com/markgol/L2lidarClass



NOTE: This replaces the L2lidar_ros2 (depracated) github repo: https://github.com/markgol/l2lidar_ros2

* * *

Features
--------

* Native ROS 2 Jazzy node (C++20)

* Qt 6.11 UDP backend (Core + Network only, no GUI, minimal number of shared libraries needed (5)  for porting)

* Publishes:
  
  * `/points` — `sensor_msgs/PointCloud2`
  
  * `/imu/data` — `sensor_msgs/Imu`
  
  * static transform: `cloud_frame -> imu_frame` (intrinsic L2 IMU offset, single hop). URDF or a `static_transform_publisher` owns the extrinsic placement above `cloud_frame`. See Coordinate Frames below.

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
                cloud_frame -> imu_frame (intrinsic L2 IMU offset; gated by publish_tf)

        +-->/min_trusted_range (ranges less than this are not reliable and may have significant error in range)

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

| Topic              | Message Type                     | Description                                                                                                                            |
| ------------------ | -------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------- |
| `/points`          | `sensor_msgs/PointCloud2`        | 3D point cloud with intensity, time, and optional range field                                                                          |
| `/imu/data`        | `sensor_msgs/Imu`                | Orientation, angular velocity, and linear acceleration                                                                                 |
| `/tf_static`       | `geometry_msgs/TransformStamped` | Static transform between LiDAR and IMU frames<br/>optional static transform robot base to LiDar                                        |
| /min_trusted_range | sensor_msgs/Float64              | Ranges under this value have been determined as not reliable and may have significant range errors.  If -1 then this has not been set. |

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

| Parameter                   | Type   | Default                              | Description                                                                                                                                                                                                                                                                                                                              |
| --------------------------- | ------ | ------------------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `l2_ip`                     | string | 192.168.1.62<br/>(factory default)   | LiDAR IP address                                                                                                                                                                                                                                                                                                                         |
| `l2_port`                   | int    | 6101<br/>(factory default)           | LiDAR UDP port                                                                                                                                                                                                                                                                                                                           |
| `host_ip`                   | string | 192.168.1.2<br/>(factory default)    | Host IP address                                                                                                                                                                                                                                                                                                                          |
| `host_port`                 | int    | 6201<br/>(factory default)           | Host UDP port                                                                                                                                                                                                                                                                                                                            |
| frame3d                     | bool   | true                                 | point cloud data is 3D not 2D                                                                                                                                                                                                                                                                                                            |
| imu_adjust                  | bool   | true                                 | Apply IMU pose correction to cloud points before publishing (Dynamic)                                                                                                                                                                                                                                                                    |
| imuRollPitchOnly            | bool   | true                                 | restirct pose correction to roll, pitch only (yaw=0.0) (Dynamic)                                                                                                                                                                                                                                                                         |
| `enable_l2_time_correction` | bool   | `true`                               | Enable LiDAR timestamp correction                                                                                                                                                                                                                                                                                                        |
| `enable_l2_host_sync`       | bool   | `true`                               | Enable host → LiDAR time sync                                                                                                                                                                                                                                                                                                            |
| `l2_sync_rate_ms`           | int    | `50`                                 | Sync rate in milliseconds                                                                                                                                                                                                                                                                                                                |
| timeScaleNum                | int64  | 2                                    | numerator for time correction scalar                                                                                                                                                                                                                                                                                                     |
| timeScaleDenom              | int64  | 1                                    | denominator for time correction scalar                                                                                                                                                                                                                                                                                                   |
| UseSystemTimeTS             | bool   | false                                | Replace IMU and point cloud timestamp with system now timestamp (Dynamic)                                                                                                                                                                                                                                                                |
| `enable_latency_measure`    | bool   | `false`                              | Enable latency measurement                                                                                                                                                                                                                                                                                                               |
| `l2_name`                   | string | `l2lidar`                            | Prefix for default-derived frame names                                                                                                                                                                                                                                                                                                   |
| `cloud_frame`               | string | `""` (resolves to `${l2_name}_link`) | Primary reference frame. Published as `header.frame_id` on `/points`, and the URDF / static_transform_publisher pin point for the device. IMU frame is auto-derived (strip trailing `_link` if present, append `_imu`). Override to embed a robot namespace for multi-robot / multi-device deployments. See **Coordinate Frames** below. |
| `publish_tf`                | bool   | `true`                               | Emit the intrinsic `cloud_frame → imu_frame` static transform. Set `false` when URDF places both frames independently.                                                                                                                                                                                                                   |
| `enable_IMU_publishing`     | bool   | `false`                              | true - publish IMU data                                                                                                                                                                                                                                                                                                                  |
| accel_x_covar               | double | 0.01                                 | noise variance for x axis accelerometer                                                                                                                                                                                                                                                                                                  |
| accel_y_covar               | double | 0.01                                 | noise variance for y axis accelerometer                                                                                                                                                                                                                                                                                                  |
| accel_z_covar               | double | 0.01                                 | noise variance for z axis accelerometer                                                                                                                                                                                                                                                                                                  |
| gyro_x_covar                | double | 0.000025                             | noise variance for x axis gyroscope                                                                                                                                                                                                                                                                                                      |
| gyro_y_covar                | double | 0.000025                             | noise variance for y axis gyroscope                                                                                                                                                                                                                                                                                                      |
| gyro_z_covar                | double | 0.0000002                            | noise variance for z axis gyroscope                                                                                                                                                                                                                                                                                                      |
| roll_covar                  | double | 4.9e-9                               | variance for roll (from quaternion pose)                                                                                                                                                                                                                                                                                                 |
| pitch_covar                 | double | 4.9e-9                               | variance for pitch (from quaternion pose)                                                                                                                                                                                                                                                                                                |
| yaw_covar                   | double | 10                                   | variance for yaw (from quaternion pose)                                                                                                                                                                                                                                                                                                  |
| aggregateNframes            | int    | 38                                   | Number of L2 frames to aggregate for publishing (Dynamic)                                                                                                                                                                                                                                                                                |
| StartScanAngle              | double | 0.0                                  | Process point cloud scans only within the StartScanAngle(degrees CCW) and ScanAngleWidth(degrees CCW). If 360.0 process all point cloud scans.                                                                                                                                                                                           |
| ScanAngleWidth              | double | 360.0                                | Process point cloud scans only within the StartScanAngle(degrees CCW) and ScanAngleWidth(degrees CCW). If 360.0 process all point cloud scans.                                                                                                                                                                                           |
| FlattenScan                 | double | false                                | Flatten the 3d point cloud to a 2d point cloud in 3d space located at StartScanAngle+ScanAngleWidth/2                                                                                                                                                                                                                                    |
| EnableCalibrationOVR        | bool   | false                                | Override internal L2 Range calibration (Dynamic)                                                                                                                                                                                                                                                                                         |
| RangeScale                  | double | 0.000978                             | Range Scale override value (Dynamic)                                                                                                                                                                                                                                                                                                     |
| RangeBias                   | double | -365.625                             | Range Bias override value (Dynamic)                                                                                                                                                                                                                                                                                                      |
| MinRange_mm                 | double | 150.0                                | Do not process point cloud point with a range less than Min_Range_mm (in millilters)                                                                                                                                                                                                                                                     |
| MaxRange_mm                 | double | 40000.0                              | Do not process point cloud point with a range greater than Min_Range_mm (in millilters)                                                                                                                                                                                                                                                  |
| AlphaAngleStep              | double | 0.602                                | Step size of the elevation scan angle for the fast scan (degrees)                                                                                                                                                                                                                                                                        |
| AlphaAngleBias              | double | 1.15                                 | Offset of the elevation scan angle for the fast scan (degrees)                                                                                                                                                                                                                                                                           |
| BetaAngle                   | double | 0.25                                 | laser alignment (degrees)                                                                                                                                                                                                                                                                                                                |
| XiAngle                     | double | 0.20                                 | laser alignment (degrees)                                                                                                                                                                                                                                                                                                                |
| ThetaAngleBias              | double | 120.0                                | Offset for slow scan angle start to align where 0.0 deg Azimuth is on the L2 mounted on the platform. (degrees)                                                                                                                                                                                                                          |
| MinTrustedRange_mm          | double | 150.0                                | This is only used for publishing a minimum trusted range.  It is not used in any point processing in this node.                                                                                                                                                                                                                          |
| EnableRangeCorrection       | bool   | false                                | Enable Range correction for processing cloud points                                                                                                                                                                                                                                                                                      |
| EnableAlphaAngleLUT         | bool   | false                                | Enable Alpha Angle LUT correction for processing cloud points                                                                                                                                                                                                                                                                            |
| CalibrationFile             | string | ""                                   | Name of calibration file to load. If "" then no calibration file                                                                                                                                                                                                                                                                         |
| watchdog_timeout_ms         | int    | 35000                                | max time without data from L2 in msec                                                                                                                                                                                                                                                                                                    |
| point_cloud_topic_id        | string | /points                              | Topic ID for point cloud publishing                                                                                                                                                                                                                                                                                                      |
| imu_topic_id                | string | /imu/data                            | Topic IF for IMU publishing                                                                                                                                                                                                                                                                                                              |
| standby_on_powerup_enabled  | bool   | false                                | Turn off watchdog timer at startup so L2 starting in standy mode won't timeout.                                                                                                                                                                                                                                                          |

Parameters indentified as (Dynamic) can be set using ros2 command like:

```
ros2 param set l2lidar_node EnableCalRangeOVR true
ros2 param set l2lidar_node calRangeBias -525.5
ros2 param set l2lidar_node calRangeScale 0.000989
ros2 param set l2lidar_node imu_adjust false
ros2 param set l2lidar_node imuRollPitchOnly true
ros2 param set l2lidar_node UseSystemTimeTS false
```

Note: realtime overrides of parameters are not persistent. If you want persistence you need to change the config yaml file or specify a calibration file in the config yaml file.

* * *

Build Requirements
------------------

* Ubuntu 24.04

* ROS 2 Jazzy

* Qt 6.11.2 or newer (Core + Network only)

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

### 2. Install of Qt 6.11.2 (optional if using prebuilt exectuable)

Install Qt 6.11.2 using the Qt Online Installer (you will need a Qt account):

```
https://my.qt.io/download
```

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

```
run l2lidar_node l2lidar_node --ros-args --params-file \home\robot\SoftwareDev\ros2_ws\src\l2lidar_node\bin\gcc_64\config/l2lidar_node.yaml
```

Or using a launch file:

```
ros2 launch l2lidar_node l2lidar.launch.py
```

Or from terminal in folder with exectuable:

```
./l2lidar_node --params-file ./config/l2lidar_node.yaml
```

This assumes are you in the folder with the following files:
    l2lidar_node
    lib/libQt6* shared library files
    config/l2lidar_node.yaml
    rviz/rvizl2lidar.rviz

* * *

RViz2 Visualization
-------------------

Start RViz2:

```
rviz2
```

Load the provided configuration:

```
rviz2 -d rviz/rvizl2lidar.rviz
```

Recommended settings:

* Fixed Frame: `l2lidar_link`

* PointCloud2:
  
  * Topic: `/points`
  
  * **Reliability Policy: `Best Effort`** — the publisher uses `SensorDataQoS`, so a default `Reliable` subscription will not receive any messages. The shipped `rviz/rvizl2lidar.rviz` layout is already configured this way; only relevant if you build a layout from scratch or import a config that defaults to Reliable.
  
  * You may see a one-shot warning at rviz startup of the form *"New subscription discovered on topic '/points', requesting incompatible QoS. No messages will be sent to it. Last incompatible policy: RELIABILITY_QOS_POLICY"*. This is a known rviz2 artifact — rviz briefly creates a default-QoS (Reliable) discovery subscription before the saved layout's Best Effort cloud-display subscription takes over. It is benign; once the display is fully loaded the cloud streams normally. The warning is harmless and can be ignored.
  
  * Color Transformer: `Channel`
  
  * Channel Name: `range` or `time`
  
  * Autoscale: `false`
  
  * Min: `0.0`
  
  * Max: `5.0`

* IMU: via TF visualization

* * *

Coordinate Frames
-----------------

### Overview

The L2 is a **single-frame device**. Its point cloud and IMU streams share a single physical reference: the geometric center of the bottom mounting surface. Per the Unitree SDK documentation (`L2lidarClass/READMore.md`):

> *"The origin of the LiDAR point cloud coordinate system is located at the center of the bottom mounting surface of the LiDAR."*

> *"The origin of the IMU coordinate system in the LiDAR point cloud coordinate system is (in meters): [-0.007698, -0.014655, 0.00667]."*

The driver exposes this directly:

- **`cloud_frame`** — the L2's primary reference. Published as `header.frame_id` on `/points`. Also where URDF (or a static_transform_publisher) should pin the device onto your robot.
- **`imu_frame`** — auto-derived from `cloud_frame` (strip trailing `_link` if present, append `_imu`). Published as `header.frame_id` on `/imu/data`. The driver emits one static TF between them with the spec offsets above.

This collapses to a single user-visible frame name (`cloud_frame`) — same approach as `velodyne_pointcloud` and `livox_ros_driver2`, which also have no mount-to-cloud offset. (`realsense_ros` and `ouster_ros` use multiple frames because their devices have non-zero optical-to-mount offsets.)

### Physical positioning

The cloud frame origin is the **geometric center of the bottom base plate** — not at any of the four mounting holes. The mounting holes form a square pattern inscribed in a Ø51 mm circle centered on the base, rotated 22.5° from the cable-exit axis. The +Z axis is perpendicular to the bottom surface, pointing away from it (toward where the L2 scans). +X is opposite the cable exit; +Y completes the right-hand rule.

### Frame naming and resolution

`cloud_frame` defaults to empty string. At startup the node resolves it:

- If empty: `cloud_frame = ${l2_name}_link` (default: `l2lidar_link`)
- If set: that exact value wins

`imu_frame` is always derived from the resolved `cloud_frame`:

| `cloud_frame`                  | derived `imu_frame`      |
| ------------------------------ | ------------------------ |
| `l2lidar_link` (default)       | `l2lidar_imu`            |
| `front_lidar_link`             | `front_lidar_imu`        |
| `bot1/lidar/l2lidar_link`      | `bot1/lidar/l2lidar_imu` |
| `my_lidar` (no `_link` suffix) | `my_lidar_imu`           |

The `_link` suffix follows REP-105 / standard ROS convention for a device's physical-attachment reference frame.

### URDF integration

**Typical case (full URDF on the host platform)**:

```xml
<link name="l2lidar_link"/>

<joint name="l2lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="l2lidar_link"/>
  <origin xyz="0.025 0 0.34" rpy="3.14159 -1.5708 0"/>
</joint>
```

Launch the node with defaults; URDF owns the extrinsic placement, the node emits the intrinsic cloud→imu TF. Resulting TF tree:

```
base_link
 └── l2lidar_link            (URDF — also the cloud frame, per Unitree spec)
      └── l2lidar_imu        (node — intrinsic offset, hardcoded per Unitree spec)
```

**Without URDF** (single-sensor setup or simple integration): use a one-line `static_transform_publisher`:

```
ros2 run tf2_ros static_transform_publisher \
  --x 0.025 --y 0 --z 0.34 --roll 3.14159 --pitch -1.5708 --yaw 0 \
  --frame-id base_link --child-frame-id l2lidar_link
```

Same TF tree, no URDF needed.

### Multi-robot / multi-device frame isolation

When multiple robots share a TF graph, the default frame names would collide between robots. Override `cloud_frame` to embed a robot-namespace prefix:

```yaml
cloud_frame: "bot1/lidar/l2lidar_link"
# imu_frame auto-derives as "bot1/lidar/l2lidar_imu"
```

A second robot would set `cloud_frame: "bot2/lidar/l2lidar_link"` and the two graphs coexist without collision. The same pattern works for multiple L2s on a single robot (`front_l2_link` / `rear_l2_link`).

### Suppressing the TF emission

If the host's URDF places the IMU frame directly (e.g., with its own intrinsic offset characterization), set `publish_tf: false` to suppress the node's emission. The PointCloud2 and IMU `header.frame_id` fields are still written normally; only the static TF broadcast is suppressed.

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

* Configure kit with Qt 6.11

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

* Requires Qt 6.11 or later due to UDP reliability fixes (Qt 6.4 is not supported). This isonly required if your are compiling and building the app

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

Example URDFs
-------------

Two trivial example URDFs and matching launch files are shipped under `urdf/examples/` and `launch/` so you can see the L2 mounting pattern end-to-end without bringing your own robot. Both define the same minimal "robot" — a 200 mm × 200 mm × 200 mm base cube with a 100 mm × 100 mm × 100 mm cube on top — and differ only in how the L2 is mounted on the small cube.

| Example                              | Mount                                    | L2 cloud direction | Use case                                                                                                |
| ------------------------------------ | ---------------------------------------- | ------------------ | ------------------------------------------------------------------------------------------------------- |
| `l2_top_mount_example.launch.py`     | top face of the small cube               | world +Z (up)      | benchtop tests, overhead-scanning setups, easiest URDF to read                                          |
| `l2_forward_mount_example.launch.py` | front face of the small cube, cable down | world +X (forward) | typical mobile-robot mounting; same `rpy=(π, -π/2, 0)` convention used by mast-mounted real deployments |

Run either:

```bash
ros2 launch l2lidar_node l2_top_mount_example.launch.py
ros2 launch l2lidar_node l2_forward_mount_example.launch.py
```

Each launch brings up:

- `robot_state_publisher` with the example URDF
- `l2lidar_node` with the stock config (factory network defaults)
- `rviz2` with the `rvizl2lidar_example.rviz` layout (Fixed Frame = `base_link`, so the cube robot sits flat in both variants — only the L2 attachment differs visually). The shipped `rvizl2lidar.rviz` layout (Fixed Frame = `l2lidar_link`) remains the right choice for the no-URDF standalone launch since `l2lidar_link` is the only frame published.

If your L2 isn't on the factory subnet, override the network params from the command line — both example launches accept `l2_ip`, `l2_port`, `host_ip`, `host_port` as launch args:

```bash
ros2 launch l2lidar_node l2_top_mount_example.launch.py \
    l2_ip:=10.42.0.62 host_ip:=10.42.0.2
```

The example URDFs are plain (not xacro), use the bundled L2 mesh from `meshes/l2_lidar.stl`, and are the simplest possible starting point if you're adapting the L2 onto your own URDF. Compare the two files side-by-side to see exactly what changes between mounting orientations — only the `top_to_l2` joint's `<origin>` differs.

* * *

Migration from V0.3.x to V0.5 (V2.0.x) for TFs
-----------------------------

V0.5 simplifies the static-TF surface and aligns the parameter naming with ROS conventions used by `realsense_ros`, `ouster_ros`, `livox_ros_driver2`, and `velodyne_pointcloud`. Seven legacy parameters are replaced with three new ones; the node now emits a single intrinsic static TF instead of two. URDF (or a single static_transform_publisher) takes ownership of the extrinsic placement.

### What changed at the parameter surface

| Removed (V0.3.x)        | Replacement (V0.5)                                                                        |
| ----------------------- | ----------------------------------------------------------------------------------------- |
| `robot_id`              | URDF (or a `static_transform_publisher`) owns the parent of `cloud_frame`                 |
| `robot_x`               | URDF / STF                                                                                |
| `robot_y`               | URDF / STF                                                                                |
| `robot_z`               | URDF / STF                                                                                |
| `frame_id`              | `cloud_frame` (semantically equivalent, different default name)                           |
| `imu_frame_id`          | no replacement — auto-derived from `cloud_frame` (strip trailing `_link`, append `_imu`)  |
| `disable_base_link_pub` | no replacement — the flag is gone because the TF is gated and is no longer emitted at all |

### Step-by-step recipe (existing V0.3.x users)

1. **Edit YAML**: replace the seven legacy params with `l2_name` / `cloud_frame` / `publish_tf`. In most cases you only need `l2_name` if the default name (`l2lidar`) doesn't suit, or you can leave all three at defaults.

2. **Declare a URDF link** named `<your_l2_name>_link` (or whatever you set `cloud_frame` to), pinned at the position + rotation that matches your previous `robot_x/y/z` and orientation. If you don't have a URDF, use a single `static_transform_publisher` instead (one CLI line — see Coordinate Frames above).

3. **Topics are unchanged**. Anything subscribing to `/points` or `/imu/data` keeps working without modification — only `header.frame_id` values change, and only if your old `frame_id` / `imu_frame_id` weren't already `l2lidar_link` / `l2lidar_imu`.

### Worked example

A V0.3.x config that placed the L2 25 cm above and 10 cm forward of `base_link`:

```yaml
# V0.3.x (before)
frame_id: "l2lidar"
imu_frame_id: "l2lidar_imu"
robot_id: "base_link"
robot_x: 0.10
robot_y: 0.00
robot_z: 0.25
```

Migrates to:

```yaml
# V0.5 (after)
l2_name: "l2lidar"     # or leave default "l2lidar"
cloud_frame: ""        # resolves to "lidar3d_link"
publish_tf: true
```

Plus, in your URDF (or as a one-line `static_transform_publisher`):

```xml
<joint name="l2lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="l2lidar_link"/>
  <origin xyz="0.10 0 0.25" rpy="0 0 0"/>
</joint>
```

one-line static transform:

```
ros2 run tf2_ros static_transform_publisher \
  --x 0.10 \
  --y 0.0 \
  --z 0.25 \
  --roll 0.0 \
  --pitch 0.0 \
  --yaw 0.0 \
  --frame-id base_link \
  --child-frame-id l2lidar_link
```

### `disable_base_link_pub` users

`disable_base_link_pub:` in the config yaml file has been depracated starting in V0.5.0.
You can just delete the `disable_base_link_pub` line from your YAML.

* * *

Versions
-------



### Current Version

**2.1.0** – Added multiple calibration overrides to builtin clibration parameters for converting cloud point to x,y,z point cloud coordinates. 
Added support for a calibration file which contains 3 sections; calibration override parameters along with meta data, range correction model, and an Alpha angle LUT.
Added publication of topic /min_trusted_range.



### Version history

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

**0.3.8** - Added dynamic config params for roll, pitch only pose correction and use system now timestamp for IMU and point cloud timestamps.  Added config params for roll, pitch, yaw covariance values. Updated to L2lidarClass V1.3.4.

**0.5.0** – Single-frame geometry refactor (breaking change). Replaced the seven legacy frame / placement parameters with three new ones (`l2_name`, `cloud_frame`, `publish_tf`). Auto-derived IMU frame from `cloud_frame`. Collapsed two static TFs into one intrinsic transform; URDF now owns the extrinsic placement. See **Migration from V0.3.x to V0.5** above. Version bumped from prior `0.3.7` (CMakeLists.txt) and `0.2.3` (package.xml) — both unified at `0.5.0`.

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
