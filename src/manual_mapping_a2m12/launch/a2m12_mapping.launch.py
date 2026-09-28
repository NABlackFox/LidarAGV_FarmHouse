import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('manual_mapping_a2m12')

    rf2o_config = os.path.join(pkg, 'config', 'rf2o_a2m12.yaml')
    mapper_config = os.path.join(pkg, 'config', 'mapper_a2m12.yaml')

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','base_link','laser'],
    )

    rf2o = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        parameters=[rf2o_config],
        output='screen',
    )

    slam_launch = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch', 'online_sync_launch.py'
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch),
        launch_arguments={
            'slam_params_file': mapper_config,
            'use_sim_time': 'false',
        }.items(),
    )

    return LaunchDescription([static_tf, rf2o, slam])
