#!/usr/bin/env python3
"""
image_capture.launch.py
========================
Launches the image_capture_service node, which exposes the
/top_camera/take_picture (std_srvs/Trigger) service for capturing a
single still photo of the table surface from the top-mounted camera.

Usage
-----
    ros2 launch orlar_agv image_capture.launch.py

    # optionally override where images are saved / what format is used
    ros2 launch orlar_agv image_capture.launch.py \
        save_dir:=/home/user/table_pictures image_format:=jpg

Then trigger a capture from anywhere with:
    ros2 service call /top_camera/take_picture std_srvs/srv/Trigger {}
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    image_topic = LaunchConfiguration('image_topic')
    save_dir = LaunchConfiguration('save_dir')
    image_format = LaunchConfiguration('image_format')
    filename_prefix = LaunchConfiguration('filename_prefix')

    image_capture_node = Node(
        package='camera',
        executable='image_capture_service',
        name='image_capture_service',
        output='screen',
        parameters=[{
            'image_topic': image_topic,
            'save_dir': save_dir,
            'image_format': image_format,
            'filename_prefix': filename_prefix,
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'image_topic',
            default_value='/top_camera/image_raw',
            description='Raw image topic published by the top camera sensor'
        ),
        DeclareLaunchArgument(
            'save_dir',
            default_value='pics/orlar_table',
            description='Directory where captured pictures are saved'
        ),
        DeclareLaunchArgument(
            'image_format',
            default_value='png',
            description="Image file format: 'png' or 'jpg'"
        ),
        DeclareLaunchArgument(
            'filename_prefix',
            default_value='table_capture',
            description='Filename prefix for saved pictures'
        ),

        image_capture_node,
    ])
