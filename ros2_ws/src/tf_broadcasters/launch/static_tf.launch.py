# MIT License – see LICENSE in repo root
"""Publish static transforms: base_link → camera_link, base_link → laser.

Adjust the xyz / rpy values to match your physical mounting.
Convention: x=forward, y=left, z=up (REP-103).
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # ── base_link → camera_link ──
    # Example: camera mounted 5 cm forward, 10 cm up, facing forward
    base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=[
            '--x',     '0.05',
            '--y',     '0.0',
            '--z',     '0.10',
            '--roll',  '0.0',
            '--pitch', '0.0',
            '--yaw',   '0.0',
            '--frame-id',       'base_link',
            '--child-frame-id', 'camera_link',
        ],
        output='screen',
    )

    # ── base_link → laser ──
    # Example: lidar mounted 3 cm behind centre, 12 cm up, no rotation
    base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=[
            '--x',     '-0.03',
            '--y',     '0.0',
            '--z',     '0.12',
            '--roll',  '0.0',
            '--pitch', '0.0',
            '--yaw',   '3.14159',   # π if lidar connector faces backward
            '--frame-id',       'base_link',
            '--child-frame-id', 'laser',
        ],
        output='screen',
    )

    # ── camera_link → camera (alias for rtabmap compatibility) ──
    camera_link_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_to_camera_tf',
        arguments=[
            '--x',     '0.0',
            '--y',     '0.0',
            '--z',     '0.0',
            '--roll',  '0.0',
            '--pitch', '0.0',
            '--yaw',   '0.0',
            '--frame-id',       'camera_link',
            '--child-frame-id', 'camera',
        ],
        output='screen',
    )

    return LaunchDescription([
        base_to_camera,
        base_to_laser,
        camera_link_to_camera,
    ])
