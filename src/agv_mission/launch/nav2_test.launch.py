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
    pkg_agv_mission = get_package_share_directory('agv_mission')

    # Path to your map and rviz configuration files inside orlar_agv
    map_yaml_file = os.path.join(pkg_orlar_agv, 'maps', 'orlar_farm_house_map.yaml')
    rviz_config_file = os.path.join(pkg_agv_mission, 'config', 'nav2_cost_map.rviz')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    map_path = LaunchConfiguration('map', default=map_yaml_file)

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_orlar_agv, 'launch', 'launch_sim.launch.py')
        )
    )

    nav2_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'map': map_path,
        }.items()
    )

    set_pose_node = Node(
        package='agv_mission',          
        executable='set_init_amcl_pose',
        name='set_init_amcl_pose',
        parameters=[{
            'x': 0.0,
            'y': 0.0,
            'theta': 0.0,  # Facing forward
            'cov': 0.25
        }],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}], # Ensures RViz uses sim time
        output='screen'
    )

    return LaunchDescription([
        sim_launch,
        nav2_launch_cmd,
        set_pose_node,
        rviz_node
    ])