//--------------------------------------------------------
//
//  L2lidar_node
//  Author: Mark Stegall
//  Module: l2lidar.launch.py
//
//	Purpose:
//		This ROS2 package provides an interface between ROS2 and the
//		Untiree L2 4D LiDAR module.  The L2 provides point cloud data
//		 and IMU data as it scans.  The scans are 3D intensity.  It generates
//		point cloud data 300 points at a time (a frame).  The L2 uses an
//		UPD Ethernet interface to send data.  This uses the L2lidar class 
//		software package to provide the backend interface to the L2.
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
//		Target:	Ubuntu 24.04 systems with ROS2 Jazzy installed
//
//		V0.1.0	2026-02-15	Initial package skeleton
//              V0.2.2  2026-03-31      Functional release
//
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                os.getenv('AMENT_PREFIX_PATH').split(':')[0],
                'share/l2lidar_node/config/l2lidar.yaml'
            ),
            description='Path to L2 LiDAR parameter file'
        ),

        Node(
            package='l2lidar_node',
            executable='l2lidar_node',
            name='l2lidar_node',
            output='screen',
            parameters=[params_file],
            respawn=True,
            respawn_delay=3.0
        )
    ])
