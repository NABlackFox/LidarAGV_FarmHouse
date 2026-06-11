import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Package directories
    pkg_orlar_agv = get_package_share_directory('orlar_agv')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    # Paths to your Slam Toolbox Params and RViz Configuration
    slam_params_file = os.path.join(pkg_orlar_agv, 'config', 'mapper_params_online_async.yaml')
    rviz_config_file = os.path.join(pkg_orlar_agv, 'config', 'slam_mapping.rviz')

    # Simulation Base Components
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_orlar_agv, 'launch', 'rsp.launch.py')]),
        launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_orlar_agv, 'launch', 'joystick.launch.py')]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    twist_mux_params = os.path.join(pkg_orlar_agv, 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    default_world = os.path.join(pkg_orlar_agv, 'worlds', 'orlar_table_world.sdf')    
    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    spawn_entity = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-topic', 'robot_description', '-name', 'my_bot', '-z', '0.1'],
        output='screen'
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    bridge_params = os.path.join(pkg_orlar_agv, 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}']
    )

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"]
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'params_file': slam_params_file,
            'use_sim_time': 'true'
        }.items()
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Launch everything with a single command!
    return LaunchDescription([
        rsp,
        joystick,
        twist_mux,
        world_arg,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        ros_gz_bridge,
        ros_gz_image_bridge,
        slam_toolbox,
        rviz2_node
    ])