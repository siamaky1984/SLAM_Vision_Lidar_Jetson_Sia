# MIT License – see LICENSE in repo root
"""Launch ORB-SLAM3 monocular node with project config.

NOTE: The actual executable name depends on which ORB-SLAM3 ROS 2 wrapper
you built.  Common options:
  - zang09/ORB_SLAM3_ROS2  →  executable='mono',  package='orb_slam3_ros2'
  - thien94/orb_slam3_ros2 →  executable='mono',  package='orb_slam3_ros2'

Adjust `package` and `executable` below to match your build.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('orb_slam3_wrapper')
    config_file = os.path.join(pkg_share, 'config', 'orb_slam3.yaml')

    orb_slam3_node = Node(
        # zang09/ORB_SLAM3_ROS2 installs package name as "orbslam3"
        package='orbslam3',             # upstream package name
        executable='mono',              # monocular executable
        name='orb_slam3_mono_node',
        parameters=[config_file],
        remappings=[
            ('camera/image_raw', '/camera/image_raw'),
        ],
        output='screen',
    )

    return LaunchDescription([
        orb_slam3_node,
    ])
