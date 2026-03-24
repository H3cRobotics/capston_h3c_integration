from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('security_audio_frontend')
    param_file = os.path.join(pkg_share, 'config', 'frontend.yaml')

    return LaunchDescription([
        Node(
            package='security_audio_frontend',
            executable='audio_frontend_node',
            name='audio_frontend_node',
            output='screen',
            parameters=[param_file]
        ),
        Node(
            package='security_audio_frontend',
            executable='respeaker_doa_node',
            name='respeaker_doa_node',
            output='screen',
            parameters=[param_file]
        ),
    ])