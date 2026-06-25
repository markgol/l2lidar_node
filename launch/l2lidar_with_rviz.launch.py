"""l2lidar_node + rviz2 standalone launch (no URDF).

Brings up the L2 driver and rviz2 with the shipped layout
(rviz/rvizl2lidar.rviz, Fixed Frame = l2lidar_link).

Launch args:
  l2_ip / l2_port / host_ip / host_port  override the four network
    settings from the shipped params file without editing YAML. Useful
    for testing against an L2 that's not on the factory 192.168.1.0/24
    subnet, e.g.:
      ros2 launch l2lidar_node l2lidar_with_rviz.launch.py \\
          l2_ip:=10.42.0.62 host_ip:=10.42.0.2
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory('l2lidar_node')

    rviz_config = os.path.join(
        pkg_share,
        'rviz',
        'rvizl2lidar.rviz'
    )

    return LaunchDescription([

        # Network override launch args. Defaults match the stock YAML;
        # any passed launch arg wins via the parameters= list override.
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

        # L2 LiDAR node
        Node(
            package='l2lidar_node',
            executable='l2lidar_node',
            name='l2lidar_node',
            output='screen',
            parameters=[
                os.path.join(pkg_share, 'config', 'l2lidar_node.yaml'),
                {
                    'l2_ip':     LaunchConfiguration('l2_ip'),
                    'l2_port':   ParameterValue(LaunchConfiguration('l2_port'),   value_type=int),
                    'host_ip':   LaunchConfiguration('host_ip'),
                    'host_port': ParameterValue(LaunchConfiguration('host_port'), value_type=int),
                },
            ],
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])
