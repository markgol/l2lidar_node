"""Example launch: Unitree L2 forward-facing on a trivial "robot".

Launches:
  - robot_state_publisher with urdf/examples/l2_forward_mount.urdf, which
    declares a 200mm base cube + 100mm top cube + the L2 mounted on the
    front face of the top cube, cable-down forward-looking convention
    (cloud +Z points world +X, cable points world -Z).
  - l2lidar_node with the stock config (factory network defaults).
    Override the network parameters via launch args if your L2 isn't on
    the factory subnet, e.g.:
        ros2 launch l2lidar_node l2_forward_mount_example.launch.py \\
            l2_ip:=10.42.0.62 host_ip:=10.42.0.2
  - rviz2 with rviz/rvizl2lidar_example.rviz (Fixed Frame = base_link,
    so the cube robot sits flat with the L2 visibly attached to its
    front face; PointCloud2 Reliability = Best Effort). The cloud
    extends ahead of the robot in the +X direction.

This is the typical mobile-robot mounting orientation (forward scanning,
cable routed downward into the body). The orientation math
rpy=(π, -π/2, 0) is the same convention used in mast-mounted deployments
like the Grunt platform.

  ros2 launch l2lidar_node l2_forward_mount_example.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('l2lidar_node')

    urdf_path = os.path.join(pkg_share, 'urdf', 'examples', 'l2_forward_mount.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    config_yaml = os.path.join(pkg_share, 'config', 'l2lidar_node.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'rvizl2lidar_example.rviz')

    # Network params surfaced as launch args so users can point the example
    # at a non-factory subnet without hand-editing config. Defaults match the
    # stock l2lidar_node.yaml; passing any of them overrides the YAML.
    l2_ip_arg = DeclareLaunchArgument(
        'l2_ip', default_value='192.168.1.62',
        description="L2 device IP address.")
    l2_port_arg = DeclareLaunchArgument(
        'l2_port', default_value='6101',
        description="L2 device UDP port.")
    host_ip_arg = DeclareLaunchArgument(
        'host_ip', default_value='192.168.1.2',
        description="Host IP address (the interface receiving cloud + IMU "
                    "data from the L2). Must be a real local interface; "
                    "the node fails to start otherwise.")
    host_port_arg = DeclareLaunchArgument(
        'host_port', default_value='6201',
        description="Host UDP port.")

    return LaunchDescription([
        l2_ip_arg,
        l2_port_arg,
        host_ip_arg,
        host_port_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='l2lidar_node',
            executable='l2lidar_node',
            name='l2lidar_node',
            output='screen',
            parameters=[
                config_yaml,
                {
                    # Later params override earlier; these are the launch-arg
                    # overrides that win against the stock YAML.
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
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
