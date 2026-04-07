from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_path = get_package_share_directory('person_tracker_ros')

    default_config = os.path.join(
        pkg_path,
        'config',
        'person_tracker.yaml'
    )

    mode = LaunchConfiguration('mode')
    config = LaunchConfiguration('config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='webcam'
        ),

        DeclareLaunchArgument(
            'config',
            default_value=default_config
        ),

        Node(
            package='person_tracker_ros',
            executable='person_tracker_node',
            name='person_tracker_node',
            output='screen',
            parameters=[
                config,              
                {'mode': mode},       
            ],
        )
    ])