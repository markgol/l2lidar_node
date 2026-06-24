//--------------------------------------------------------
//
//  L2lidar_node
//  Author: Mark Stegall
//  Module: l2lidar_node.hpp
// Contributor: https://github.com/pondersome
//
//	Purpose:
//		The l2lidar_node app is a ROS2 package which provides an interface
//      between ROS2 and the Untiree L2 4D LiDAR module.
//      The L2 provides point cloud data and IMU data as it scans.
//      The scans are 3D (x,y,z) with intensity, rannge and ring (always 1).
//      It generates point cloud data 300 points at a time (an L2 frame).
//      The L2 uses a UPD Ethernet interface to send data.
//      This app uses the L2lidar class	software package to provide
//      the backend interface to the L2.
//		This is class is structured to be compatible with the formats
//		and interfaces needed for support in the ROS2 packages.
//
//		This ROS2 package publishes the point cloud data and IMU data
//		for ROS2 subscribers.
//
//		- Publishes (default:
//			/points	(sensor_msgs/PointCloud2)
//			/imu/data (sensor_msgs/Imu)
//			Static TF transform support
//
//	Implementation
//		This is the ROS2 driver for the Unitree L2 4d LiDAR.
//      This implements a publisher node for IMU and point cloud data.
//      ROS2 nodes sources:
//          src/main.cpp
//          src/l2lidar_node.cpp
//          include/l2lidar_node.hpp
//
//      L2 driver sources are in their own folder
//          L2lidarClass
//              src/L2lidar.cpp
//              include/L2lidar.h
//              include/PCpoint.h
//              include/quaternion.h
//              include/unitree_lidar_protocolL2.h
//              include/untiree_lidar_utilitiesL2.h
//
//      Restrictions
//      The sources require Qt6.10.2 or higher.
//      This only uses Qt6 Core and Qt6 Networking.
//      This node is standalone. It does not incorporate any Gui elements.
//      DO NOT USE Qt or ROS2 Gui elements or similar resources in this node.
//
//		Target:	Ubuntu 24.04 systems with ROS2 Jazzy installed
//		Initial target hardware is RPI5 (ARM64)
//		A x86_64 imeplentation will be done after the RPI5 version
//		completed.
//
//		V0.1.0	2026-02-16	Initial package skeleton
//      V0.2.0 	2026-02-21	Added aggregation of L2 frames for publishing
//										This is needed to align point cloud publishing
//                          					aligned otthe requirements for LIO-SAM methodology
//		V0.2.1	2026-03-06	Parameterized frame3d and imu_adjust
//      V0.2.2  2026-03-12  Added static robot TF
//      V0.2.3  2026-04-12  Added enable/disable IMU publishing
//                          changed QOS for publishers to SensorDataQoS()
//      V0.3.0  2026-04-21  Added internal Range calibration override
//	    V0.3.2  2026-05-13  Added service to stop/start rotation, contributed by https://github.com/pondersome
//                          Changed point time from float to double
//                          changed the sensor_msgs::PointCloud2Iterator<float> iter_t(cloud, "time")
//                          backed to float
//                          to a float relative time point (it should be reconstructed into
//                          an int64 in the subscriber node, using the message timestamp+relative time)
//                          Added config yaml parameters for setting the L2 timescale correction scale
//      V0.3.3  2026-05-19  Documentation change only on L2 point cloud origin.
//      V0.3.4  2026-05-31  Updated to L2lidarClass V1.3.3, corrects bugs in timestamp correction introduced
//                          in the V1.3.0 L2lidarClass
//      V0.3.5  2026-06-03  Added enable/disable of TF publishing of base_link.
//                          Added config parameters for accel and gyro covariances
//                          Added initialization of the accel and gyro covariances to the IMU message
//      V0.3.6  2026-06-12  Changes to CMakeList.txt file to create distribution file (subfolder dist
//                          in the build folder for platform and build (release or debug).
//      V0.3.7  2026-06-13  Added config file param for topic IDs for point cloud and imu data.
//                          Code cleanup. Changing class member variable to end with an underscore to
//                          idenfity class memebr variables versus local variables.
//                          Added startup condition to support L2 power on in standby mode.
//                          This l2lidar_node will not timeout in this case but requires a start service command
//                          to send a command to the L2 to bring it out of standby.  If the L2 automatically starts
//                          automatically then the IMU and Point cloud packets woudl still be published but l2lidar_node
//                          would never timeout.  This is implelemented using the config param standby_on_powerup_enabled
//      V0.3.8  2026-06-20  Updated to L2lidarClass V1.3.4
//                          Add config param for roll,ptich only with IMUadjust
//                          Add config param for UseSystemNow timestamps
//                          Add covariance matrix for IMU quaternion
//		V0.5.0 2026-06-24	Updated to L2lidarClass V1.3.5
//						Single-frame geometry refactor (breaking change).
//						Replaced the seven legacy frame / placement parameters with three new ones (`l2_name`, `cloud_frame`, `publish_tf`).
//						Auto-derived IMU frame from `cloud_frame`.
//						Collapsed two static TFs into one intrinsic transform; URDF now owns the extrinsic placement.
//						See **Migration from V0.3.x to V0.5** in the README.md.
//						CMakeLists.txt changed to copy additional files to executable standalone folders.
//						package.xml version updated to V0.5.0
//
//      V1.0.0  2026-0x-xx  This will be the first production release.
//
//  Note: class member variables end with an _
//--------------------------------------------------------

//--------------------------------------------------------
// GPL-3.0 license
//
// This file is part of l2lidar_node.
//
// l2lidar_node is free software : you can redistribute it and /or modify it under
// the terms of the GNU General Public License as published by the Free Software Foundation,
// either version 3 of the License, or (at your option) any later version.
//
// l2lidar_node is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// See the GNU General Public License for more details.
// You should have received a copy of the GNU General Public License along with L2diagnsotic.
// If not, see < https://www.gnu.org/licenses/>.
//--------------------------------------------------------

#pragma once

#include <std_srvs/srv/set_bool.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <QCoreApplication>
#include <QObject>
#include <QTimer>
#include <atomic>

#include "L2lidar.h"

class L2LidarNode : public QObject, public rclcpp::Node
{
    Q_OBJECT

public:
    explicit L2LidarNode(int argc, char **argv);
    ~L2LidarNode();

private slots:
    void onImuReceived();
    void onPointCloudReceived();
    void spinOnce();
    void watchdogCheck();

    // This is standard ROS2 parameter callback
    rcl_interfaces::msg::SetParametersResult onParamChange(
        const std::vector<rclcpp::Parameter> &params);

private:
    void publishStaticTransform();
    void shutdownNode(const std::string &reason);

    L2lidar lidar_;

    // This is standard ROS2 parameter callback
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr cb_params_handle_;
    rcl_interfaces::msg::SetParametersResult paramFail(const std::string &msg);
        rcl_interfaces::msg::SetParametersResult paramSuccess();

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

    // Live enable/disable service. data=true starts rotation, data=false
    // enters standby. Watchdog is also paused while in standby so a
    // deliberate stop doesn't trigger a respawn loop.
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_;

    // V0.5: single-frame topology   see README "Coordinate Frames".
    // cloud_frame_ is the L2's primary reference, where URDF should pin the
    // device, and what published PointCloud2 / IMU messages name in their
    // header.frame_id. imu_frame_ is auto-derived from cloud_frame_ at
    // startup (strip trailing "_link" if present, append "_imu"). publish_tf_
    // gates emission of the intrinsic cloud_frame_ ? imu_frame_ static TF.
    std::string l2_name_; // prefix for default-derived frame names
    std::string cloud_frame_; // primary reference frame, also published PointCloud2 frame_id
    std::string imu_frame_; // derived from cloud_frame_; published IMU frame_id
    bool publish_tf_ {true};

    bool UseSystemTimeTS_ {false};
    bool time_corr_{true}, host_sync_{true};
    int64_t timeScaleNum_ {2};
    int64_t timeScaleDenom_ {1};

    int aggregateNframes_{38};

    bool enable_IMU_publishing_ {false};
    double accel_x_covar_ {0.01};
    double accel_y_covar_ {0.01};
    double accel_z_covar_ {0.01};
    double gyro_x_covar_ {0.000025};
    double gyro_y_covar_ {0.000025};
    double gyro_z_covar_ {0.0000001};
    double roll_covar_ {5.0e-9};
    double pitch_covar_ {5.0e-9};
    double yaw_covar_ {9.0};

    bool frame3d_;

    bool imu_adjust_ {false};
    bool imuRollPitchOnly_ {true};

    // calibration override
    // These do not have to be globals
    // This is for possible future expansion
    // for live parameter updates
    bool EnableCalRangeOVR_ {false};
    double calRangeScale_ {0.000978};
    double calRangeBias_ {-365.625};
	
    // watchdog
    QTimer watchdog_timer_;

    QElapsedTimer last_imu_time_;
    QElapsedTimer last_pc_time_;

    int watchdog_timeout_ms_;

    QTimer spin_timer_;

};
