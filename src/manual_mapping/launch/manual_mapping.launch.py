from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    manual_mapping_share = get_package_share_directory('manual_mapping')

    rf2o_config = os.path.join(
        manual_mapping_share,
        'config',
        'rf2o_real.yaml'
    )

    slam_config = os.path.join(
        manual_mapping_share,
        'config',
        'mapper_real.yaml'
    )

    slam_toolbox_launch = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
        output='screen'
    )

    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='CLaserOdometry2DNode',
        parameters=[rf2o_config],
        output='screen'
    )

    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch),
        launch_arguments={
            'slam_params_file': slam_config,
            'use_sim_time': 'false'
        }.items()
    )

    return LaunchDescription([
        static_tf_node,
        rf2o_node,
        slam_node
    ])
