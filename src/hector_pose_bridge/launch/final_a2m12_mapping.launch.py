import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    serial_port = LaunchConfiguration('serial_port')
    launch_rviz = LaunchConfiguration('launch_rviz')

    hector_share = get_package_share_directory('hector_mapping')
    bridge_share = get_package_share_directory('hector_pose_bridge')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')

    hector_launch = os.path.join(
        hector_share,
        'launch',
        'hector_local_odom.launch.py'
    )

    slam_config = os.path.join(
        bridge_share,
        'config',
        'slam_toolbox_hector.yaml'
    )

    slam_launch = os.path.join(
        slam_toolbox_share,
        'launch',
        'online_async_launch.py'
    )

    # ---------------------------------------------------------
    # 1. RPLIDAR A2M12
    # ---------------------------------------------------------
    lidar = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'serial_baudrate': 256000,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Sensitivity',
        }]
    )

    # ---------------------------------------------------------
    # 2. Static TF: base_link -> laser
    # ---------------------------------------------------------
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        output='screen',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser',
        ]
    )

    # ---------------------------------------------------------
    # 3. A2M12 structure filter
    # Uses current FINAL defaults:
    # 0.30-4.00 m >= 15 mm
    # 4.00-8.00 m >= 100 mm
    # ---------------------------------------------------------
    structure_filter = Node(
        package='a2m12_structure_filter',
        executable='structure_filter_node',
        name='a2m12_structure_filter',
        output='screen',
    )

    # ---------------------------------------------------------
    # 4. Hector local pose estimation
    # Uses the tested Hector launch from hector_test_ws
    # ---------------------------------------------------------
    hector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hector_launch),
        launch_arguments={
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'scan_topic': '/scan_hector_clean',
        }.items()
    )

    # ---------------------------------------------------------
    # 5. Pose / scan synchronization bridge
    # Produces:
    #   odom -> base_link
    #   /scan_slam
    # ---------------------------------------------------------
    sync_bridge = Node(
        package='hector_pose_bridge',
        executable='sync_scan_bridge',
        name='hector_sync_scan_bridge',
        output='screen',
    )

    # ---------------------------------------------------------
    # 6. SLAM Toolbox
    # Uses FINAL slam_toolbox_hector.yaml
    # ---------------------------------------------------------
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch),
        launch_arguments={
            'slam_params_file': slam_config,
            'use_sim_time': 'false',
        }.items()
    )

    # ---------------------------------------------------------
    # 7. Optional RViz
    # ---------------------------------------------------------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0'
        ),

        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true'
        ),

        # Driver and TF first
        lidar,
        static_tf,

        # Give LiDAR time to begin publishing
        TimerAction(
            period=2.0,
            actions=[structure_filter]
        ),

        # Wait for filtered scans
        TimerAction(
            period=4.0,
            actions=[hector]
        ),

        # Wait for Hector pose
        TimerAction(
            period=6.0,
            actions=[sync_bridge]
        ),

        # Start global SLAM after TF + synchronized scan exist
        TimerAction(
            period=8.0,
            actions=[slam_toolbox]
        ),

        TimerAction(
            period=10.0,
            actions=[rviz]
        ),
    ])
