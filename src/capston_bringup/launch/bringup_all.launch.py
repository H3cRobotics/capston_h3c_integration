from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    patrol_vision_dir = get_package_share_directory('patrol_vision')
    patrol_bridge_dir = get_package_share_directory('patrol_bridge')
    security_audio_system_dir = get_package_share_directory('security_audio_system')

    server_ip_arg = DeclareLaunchArgument(
        'server_ip',
        default_value='192.168.0.16',
        description='A single IP address to construct all server URLs.'
    )
    
    # Deriving URLs from the single server_ip
    server_ip = LaunchConfiguration('server_ip')
    server_url = ['http://', server_ip, ':8000']
    signaling_url = ['http://', server_ip, ':8001']

    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(patrol_vision_dir, 'launch', 'system.launch.py')),
        launch_arguments={
            'server_url': server_url,
            'signaling_url': signaling_url
        }.items()
    )

    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(patrol_bridge_dir, 'launch', 'bringup.launch.py')),
        launch_arguments={
            'server_url': server_url
        }.items()
    )

    audio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(security_audio_system_dir, 'launch', 'bringup.launch.py')),
        launch_arguments={
            'server_url': server_url
        }.items()
    )

    return LaunchDescription([
        server_ip_arg,
        vision_launch,
        bridge_launch,
        audio_launch
    ])
