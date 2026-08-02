from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    image_topic = LaunchConfiguration('image_topic')
    save_dir = LaunchConfiguration('save_dir')
    image_format = LaunchConfiguration('image_format')
    capture_interval = LaunchConfiguration('capture_interval')

    interval_image_capture_node = Node(
        package='camera',
        executable='interval_image_capture_service',
        name='interval_image_capture_service',
        output='screen',
        parameters=[{
            'image_topic': image_topic,
            'save_dir': save_dir,
            'image_format': image_format,
            'capture_interval': capture_interval,
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'image_topic',
            default_value='/top_camera/image_raw',
            description='Raw image topic'
        ),
        DeclareLaunchArgument(
            'save_dir',
            default_value='pics/orlar_interval',
            description='Directory where interval pictures are saved'
        ),
        DeclareLaunchArgument(
            'image_format',
            default_value='png',
            description="Image file format: 'png' or 'jpg'"
        ),
        DeclareLaunchArgument(
            'capture_interval',
            default_value='5.0',
            description='Interval in seconds between image captures'
        ),
        interval_image_capture_node,
    ])