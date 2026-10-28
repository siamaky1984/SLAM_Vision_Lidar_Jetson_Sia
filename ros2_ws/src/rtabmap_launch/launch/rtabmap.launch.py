# MIT License – see LICENSE in repo root
"""Launch rtabmap_ros node with project config (monocular + 2D lidar)."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rtabmap_launch')
    config_file = os.path.join(pkg_share, 'config', 'rtabmap.yaml')

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        parameters=[config_file],
        remappings=[
            ('rgb/image', '/camera/image_raw'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('scan', '/scan'),
            ('odom', '/odometry/filtered'),
        ],
        arguments=['--delete_db_on_start'],   # Remove for persistent mapping
        output='screen',
    )

    return LaunchDescription([
        rtabmap_node,
    ])
