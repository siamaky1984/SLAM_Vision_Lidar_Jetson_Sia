# MIT License – see LICENSE in repo root
"""Launch RViz2 with the project's default display config."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('slam_bringup')
    rviz_config = os.path.join(pkg_share, 'rviz', 'slam_default.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        rviz_node,
    ])
