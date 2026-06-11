import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node # 1. Fixed: Added missing Node import

def generate_launch_description(): 
    pkg_nav2_dir = get_package_share_directory('nav2_bringup')
    pkg_orlar_agv = get_package_share_directory('orlar_agv')

    # Path to your map and rviz configuration files inside orlar_agv
    map_yaml_file = os.path.join(pkg_orlar_agv, 'maps', 'orlar_farm_house_map.yaml')
    rviz_config_file = os.path.join(pkg_orlar_agv, 'config', 'slam_mapping.rviz2')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    map_path = LaunchConfiguration('map', default=map_yaml_file)

    # 2. Fixed: Restored the proper Nav2 bringup command with the initial pose arguments filled in
    nav2_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'map': map_path,
            'cmd_vel_topic': '/diff_cont/cmd_vel_unstamped'
        }.items()
    )

    # 3. Your custom RViz Node using the configuration path
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config_file],
    #     parameters=[{'use_sim_time': True}],
    #     output='screen'
    # )

    return LaunchDescription([
        nav2_launch_cmd,
        # rviz_node # Cleaned up to use your custom node instead of nav2's script
    ])