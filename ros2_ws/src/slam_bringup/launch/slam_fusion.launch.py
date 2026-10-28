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

    use_orb_slam3_arg = DeclareLaunchArgument(
        'use_orb_slam3', default_value='false',
        description='Launch ORB-SLAM3 (scale-aligned to lidar ICP odom)')

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

    # 2.5. ICP Odometry from lidar scans (always on: metric odom input to the EKF)
    from launch_ros.actions import Node
    
    # Use RTAB-Map's ICP odometry to track motion from laser scans
    # This publishes /odom topic and odom→base_link TF
    icp_odom_node = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_tf': False,   # EKF owns odom→base_link
            'wait_for_transform': 0.2,
            'Icp/PointToPlane': 'true',
            'Icp/VoxelSize': '0.05',
            'Icp/MaxCorrespondenceDistance': '0.1',
            'Icp/PM': 'true',  # Use Point Matcher
            'Icp/PMOutlierRatio': '0.95',
            'Odom/Strategy': '0',  # Frame-to-frame
            'Odom/ResetCountdown': '1',
            'Odom/ScanKeyFrameThr': '0.9',
        }],
        remappings=[
            ('scan', '/scan'),
        ],
        output='screen',
    )

    # 2.6. EKF: fuses ICP odom (+ scaled mono VO velocity) → /odometry/filtered + odom→base_link
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[PathJoinSubstitution([
            FindPackageShare('slam_bringup'), 'config', 'ekf.yaml'])],
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

    # 4. ORB-SLAM3 monocular (arbitrary-scale VO → scaled velocity into the EKF; no TF)
    orb_slam3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('orb_slam3_wrapper'), 'launch', 'orb_slam3_mono.launch.py'
            ])
        ),
        condition=IfCondition(LaunchConfiguration('use_orb_slam3')),
    )

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
        use_orb_slam3_arg,
        # Launches (order matters for startup sequencing)
        static_tf_launch,
        icp_odom_node,  # Metric lidar odometry (EKF input; TF disabled)
        ekf_node,       # Sole owner of odom→base_link
        camera_launch,
        rplidar_launch,
        orb_slam3_launch,
        rtabmap_launch,
        rviz_launch,
    ])
