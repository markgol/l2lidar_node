//--------------------------------------------------------
//
//  L2lidar_node
//  Author: Mark Stegall
//  Module: l2lidar_node.hpp
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
//		- Publishes:
//			/l2lidar/points	(sensor_msgs/PointCloud2)
//			/l2lidar/imu (sensor_msgs/Imu)
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
//
#pragma once

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

    std::string frame_id_; // lidar sensor frame
    std::string imu_frame_id_; // imu sensor frame
    std::string robot_id_; // robot frame
    float robot_x_;
    float robot_y_;
    float robot_z_;

    bool time_corr{true}, host_sync{true};

    // Possible future expansion
    // for live parameter updates
    int aggregateNframes{38};

    bool enable_IMU_publishing_ {false};

    bool frame3d;

    // Possible future expansion
    // for live parameter updates
    bool imu_adjust;

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
