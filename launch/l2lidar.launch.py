"""l2lidar_node launch (no rviz).

Author: Mark Stegall
Module: l2lidar.launch.py

Purpose:
  This ROS 2 package provides an interface between ROS 2 and the Unitree
  L2 4D LiDAR module. The L2 provides point cloud data and IMU data as
  it scans. The scans are 3D intensity. It generates point cloud data
  300 points at a time (a frame). The L2 uses a UDP Ethernet interface
  to send data. This uses the L2lidar class software package to provide
  the backend interface to the L2.

  This ROS 2 package publishes the point cloud data and IMU data for
  ROS 2 subscribers.

Publishes:
  /points     (sensor_msgs/PointCloud2)
  /imu/data   (sensor_msgs/Imu)
  /tf_static  cloud_frame -> imu_frame (intrinsic L2 IMU offset)

Launch args:
  params_file  path to the YAML params file (default: the package's
               shipped config/l2lidar_node.yaml).
  l2_ip / l2_port / host_ip / host_port  override the four network
               settings from the params file without editing YAML. Useful
               for testing against an L2 that's not on the factory
               192.168.1.0/24 subnet, e.g.:
                 ros2 launch l2lidar_node l2lidar.launch.py \\
                     l2_ip:=10.42.0.62 host_ip:=10.42.0.2

Target: Ubuntu 24.04 systems with ROS 2 Jazzy installed.

History:
  V0.1.0  2026-02-15  Initial package skeleton
  V0.2.2  2026-03-31  Functional release
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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
                'share/l2lidar_node/config/l2lidar_node.yaml'
            ),
            description='Path to L2 LiDAR parameter file'
        ),

        # Network params surfaced as launch args so users can point the node
        # at a non-factory subnet without hand-editing YAML. Defaults match
        # the stock l2lidar_node.yaml. Any explicitly-passed launch arg
        # overrides the YAML (later params win in the parameters= list).
        DeclareLaunchArgument(
            'l2_ip', default_value='192.168.1.62',
            description='L2 device IP address.'),
        DeclareLaunchArgument(
            'l2_port', default_value='6101',
            description='L2 device UDP port.'),
        DeclareLaunchArgument(
            'host_ip', default_value='192.168.1.2',
            description='Host IP address (the interface receiving cloud + '
                        'IMU data from the L2). Must be a real local '
                        'interface; the node fails to start otherwise.'),
        DeclareLaunchArgument(
            'host_port', default_value='6201',
            description='Host UDP port.'),

        Node(
            package='l2lidar_node',
            executable='l2lidar_node',
            name='l2lidar_node',
            output='screen',
            parameters=[
                params_file,
                {
                    # LaunchConfiguration substitutions are strings until the
                    # node reads them; the node declares l2_port / host_port
                    # as int, so wrap them in ParameterValue with value_type
                    # to coerce. l2_ip / host_ip stay as strings.
                    'l2_ip':     LaunchConfiguration('l2_ip'),
                    'l2_port':   ParameterValue(LaunchConfiguration('l2_port'),   value_type=int),
                    'host_ip':   LaunchConfiguration('host_ip'),
                    'host_port': ParameterValue(LaunchConfiguration('host_port'), value_type=int),
                },
            ],
            respawn=True,
            respawn_delay=3.0
        )
    ])
