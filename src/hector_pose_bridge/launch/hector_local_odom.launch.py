from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    hector_mapping = Node(
        package='hector_mapping',
        executable='hector_mapping_node',
        namespace='hector_local',
        name='hector_mapping_local',
        output='screen',

        remappings=[
            ('scan', '/scan_hector_clean'),
        ],

        parameters=[{
            'use_sim_time': False,

            # Local Hector coordinate system
            'map_frame': 'odom',
            'odom_frame': 'odom',
            'base_frame': 'base_link',

            # Hector is used only for local LiDAR pose estimation
            'pub_map_odom_transform': False,
            'pub_map_scanmatch_transform': False,

            # Best-performing Hector configuration
            'map_update_distance_thresh': 0.20,
            'map_update_angle_thresh': 0.03,
            'update_factor_free': 0.35,
            'update_factor_occupied': 0.75,
        }]
    )

    return LaunchDescription([
        hector_mapping
    ])
