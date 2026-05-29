# MIT License – see LICENSE in repo root
"""Top-level launch: brings up camera, lidar, ORB-SLAM3, RTAB-Map, and TF.

Usage:
    ros2 launch slam_bringup slam_fusion.launch.py
    ros2 launch slam_bringup slam_fusion.launch.py camera_device:=/dev/video2
    ros2 launch slam_bringup slam_fusion.launch.py use_rviz:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Launch arguments ──
    camera_device_arg = DeclareLaunchArgument(
        'camera_device', default_value='/dev/video0',
        description='V4L2 video device path')

    lidar_serial_arg = DeclareLaunchArgument(
        'lidar_serial', default_value='/dev/ttyUSB0',
        description='RPLIDAR serial port')

    lidar_baudrate_arg = DeclareLaunchArgument(
        'lidar_baudrate', default_value='115200',
        description='RPLIDAR baud rate')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='Launch RViz alongside the pipeline')

    # ── Sub-launches ──

    # 1. Static TF (base_link → camera_link, base_link → laser)
    static_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('tf_broadcasters'), 'launch', 'static_tf.launch.py'
            ])
        ),
    )

    # 2. Camera
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('camera_launch'), 'launch', 'camera.launch.py'
            ])
        ),
        launch_arguments={
            'camera_device': LaunchConfiguration('camera_device'),
        }.items(),
    )

    # 2.5. Static odom → base_link transform (placeholder for when ORB-SLAM3 is disabled)
    from launch_ros.actions import Node
    static_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_link_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen',
    )

    # 3. RPLIDAR
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rplidar_launch'), 'launch', 'rplidar.launch.py'
            ])
        ),
        launch_arguments={
            'lidar_serial': LaunchConfiguration('lidar_serial'),
            'lidar_baudrate': LaunchConfiguration('lidar_baudrate'),
        }.items(),
    )

    # 4. ORB-SLAM3 monocular (visual odometry → /odom)
    # orb_slam3_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([
    #             FindPackageShare('orb_slam3_wrapper'), 'launch', 'orb_slam3_mono.launch.py'
    #         ])
    #     ),
    # )

    # 5. RTAB-Map (graph SLAM → /map)
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rtabmap_launch'), 'launch', 'rtabmap.launch.py'
            ])
        ),
    )

    # 6. RViz (optional)
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('slam_bringup'), 'launch', 'rviz.launch.py'
            ])
        ),
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        # Arguments
        camera_device_arg,
        lidar_serial_arg,
        lidar_baudrate_arg,
        use_rviz_arg,
        # Launches (order matters for startup sequencing)
        static_tf_launch,
        static_odom_tf,  # Static odom frame (placeholder)
        camera_launch,
        rplidar_launch,
        # orb_slam3_launch,
        rtabmap_launch,
        rviz_launch,
    ])
