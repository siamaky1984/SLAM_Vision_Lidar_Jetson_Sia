# MIT License – see LICENSE in repo root
"""Launch v4l2_camera node with project config."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('camera_launch')
    config_file = os.path.join(pkg_share, 'config', 'camera.yaml')

    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video0',
        description='V4L2 video device path',
    )

    v4l2_camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera_node',
        namespace='camera',
        parameters=[
            config_file,
            {'video_device': LaunchConfiguration('camera_device')},
        ],
        remappings=[
            ('image_raw', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
        ],
        output='screen',
    )

    return LaunchDescription([
        camera_device_arg,
        v4l2_camera_node,
    ])
