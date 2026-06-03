import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'orlar_agv'

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                launch_arguments={'gz_args': '-r empty.sdf'}.items()
             )

    create_entity = Node(
        package='ros_gz_sim', 
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'orlar_agv',
            '-z', '0.05'
        ],
        output='screen'
    )

    # NEW: Add the Parameter Bridge Node
    # This maps ROS 2 topics over to Gazebo Sim topics
    ros_gz_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        # cmd_vel: ROS2 -> Gazebo (] means ROS2 to Gazebo)
        '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        # odom: Gazebo -> ROS2
        '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        # clock: Gazebo -> ROS2
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        # tf: Gazebo -> ROS2
        '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
    ],
    output='screen'
    )

    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        create_entity,
        ros_gz_bridge, # <-- Don't forget to add it to the return list!
    ])