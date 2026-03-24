from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('security_audio_classifier')
    param_file = os.path.join(pkg_share, 'config', 'classifier.yaml')

    return LaunchDescription([
        Node(
            package='security_audio_classifier',
            executable='yamnet_classifier_node',
            name='yamnet_classifier_node',
            output='screen',
            parameters=[param_file]
        )
    ])