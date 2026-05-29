# MIT License – see LICENSE in repo root
"""Launch rplidar_ros node with project config."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rplidar_launch')
    config_file = os.path.join(pkg_share, 'config', 'rplidar.yaml')

    serial_port_arg = DeclareLaunchArgument(
        'lidar_serial',
        default_value='/dev/ttyUSB0',
        description='RPLIDAR serial port',
    )

    baudrate_arg = DeclareLaunchArgument(
        'lidar_baudrate',
        default_value='115200',
        description='RPLIDAR baud rate',
    )

    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[
            config_file,
            {
                'serial_port': LaunchConfiguration('lidar_serial'),
                'serial_baudrate': LaunchConfiguration('lidar_baudrate'),
            },
        ],
        output='screen',
        respawn=True,
        respawn_delay=5.0,  # Increased delay to allow hardware recovery
    )

    return LaunchDescription([
        serial_port_arg,
        baudrate_arg,
        rplidar_node,
    ])
