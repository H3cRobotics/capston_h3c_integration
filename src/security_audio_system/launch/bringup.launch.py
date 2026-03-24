from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    frontend_share = get_package_share_directory('security_audio_frontend')
    classifier_share = get_package_share_directory('security_audio_classifier')
    system_share = get_package_share_directory('security_audio_system')

    frontend_param = os.path.join(frontend_share, 'config', 'frontend.yaml')
    classifier_param = os.path.join(classifier_share, 'config', 'classifier.yaml')
    system_param = os.path.join(system_share, 'config', 'system.yaml')

    server_url_arg = DeclareLaunchArgument(
        'server_url',
        default_value='http://192.168.0.16:8000'
    )

    server_url = LaunchConfiguration('server_url')

    return LaunchDescription([
        server_url_arg,

        Node(
            package='security_audio_frontend',
            executable='audio_frontend_node',
            name='audio_frontend_node',
            output='screen',
            parameters=[frontend_param]
        ),
        Node(
            package='security_audio_frontend',
            executable='respeaker_doa_node',
            name='respeaker_doa_node',
            output='screen',
            parameters=[frontend_param]
        ),
        Node(
            package='security_audio_classifier',
            executable='yamnet_classifier_node',
            name='yamnet_classifier_node',
            output='screen',
            parameters=[classifier_param]
        ),
        Node(
            package='security_audio_system',
            executable='sound_event_manager_node',
            name='sound_event_manager_node',
            output='screen',
            parameters=[system_param]
        ),
        Node(
            package='security_audio_system',
            executable='clip_transfer_node',
            name='clip_transfer_node',
            output='screen',
            parameters=[system_param, {'server_url': server_url}]
        ),
        Node(
            package='security_audio_system',
            executable='sound_event_monitor_node',
            name='sound_event_monitor_node',
            output='screen',
            parameters=[system_param]
        ),
    ])