import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node 

def generate_launch_description(): 
    pkg_nav2_dir = get_package_share_directory('nav2_bringup')
    pkg_orlar_agv = get_package_share_directory('orlar_agv')
    pkg_agv_mission = get_package_share_directory('agv_mission')

    map_yaml_file = os.path.join(pkg_orlar_agv, 'maps', 'orlar_farm_house_map.yaml')
    rviz_config_file = os.path.join(pkg_agv_mission, 'config', 'nav2_cost_map.rviz')
    nav2_params_file = os.path.join(pkg_agv_mission, 'config', 'nav2_params.yaml')
    keepout_mask_file = os.path.join(pkg_agv_mission, 'filters', 'keepout_mask.yaml')

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
            'params_file': nav2_params_file,
        }.items()
    )

    filter_mask_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        output='screen',
        emulate_tty=True,
        parameters=[
            nav2_params_file,
            {'yaml_filename': keepout_mask_file} # Dynamic path injection
        ],
        remappings=[('/map', '/keepout_filter_mask')]
    )

    costmap_filter_info_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        emulate_tty=True,
        parameters=[nav2_params_file]
    )

    filter_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_filters',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['filter_mask_server', 'costmap_filter_info_server']
        }]
    )

    set_pose_node = Node(
        package='agv_mission',          
        executable='set_init_amcl_pose',
        name='set_init_amcl_pose',
        parameters=[{
            'x': 0.0,
            'y': 0.0,
            'theta': 0.0,  
            'cov': 0.25
        }],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}], 
        output='screen'
    )

    return LaunchDescription([
        sim_launch,
        nav2_launch_cmd,
        filter_mask_server,
        costmap_filter_info_server,
        filter_lifecycle_manager,
        set_pose_node,
        rviz_node
    ])