from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory('l2lidar_node')

    rviz_config = os.path.join(
        pkg_share,
        'config',
        'rvizl2lidar.rviz'
    )

    return LaunchDescription([

        # L2 LiDAR node
        Node(
            package='l2lidar_node',
            executable='l2lidar_node',
            name='l2lidar_node',
            output='screen',
            parameters=[os.path.join(pkg_share, 'config', 'l2lidar.yaml')]
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
	
