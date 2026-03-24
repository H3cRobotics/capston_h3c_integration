from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('security_audio_system')
    param_file = os.path.join(pkg_share, 'config', 'system.yaml')

    return LaunchDescription([
        Node(
            package='security_audio_system',
            executable='sound_event_manager_node',
            name='sound_event_manager_node',
            output='screen',
            parameters=[param_file]
        ),
        Node(
            package='security_audio_system',
            executable='clip_transfer_node',
            name='clip_transfer_node',
            output='screen',
            parameters=[param_file]
        )
    ])