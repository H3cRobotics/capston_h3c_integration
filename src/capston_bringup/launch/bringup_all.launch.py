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
    patrol_yolo_system_dir = get_package_share_directory('patrol_yolo')
    capston_bringup_dir = get_package_share_directory('capston_bringup')

    server_ip_arg = DeclareLaunchArgument(
        'server_ip',
        default_value='192.168.0.16',
        description='A single IP address to construct all server URLs.'
    )

    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/camera/color/image_raw', # 리얼센스로 변경되면 /camera/camera/color/image_raw
        description='Image topic used by patrol vision nodes.'
    )

    yolo_mode_arg = DeclareLaunchArgument(
        'yolo_mode',
        default_value='realsense' # 리얼센스로 변경되면 'realsense'
    )
    
    # Deriving URLs from the single server_ip
    server_ip = LaunchConfiguration('server_ip')
    image_topic = LaunchConfiguration('image_topic')
    yolo_mode = LaunchConfiguration('yolo_mode')

    server_url = ['http://', server_ip, ':8000']
    signaling_url = ['http://', server_ip, ':8001']

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(capston_bringup_dir, 'launch', 'realsense_wrapper.launch.py')
        )
    )
    
    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(patrol_vision_dir, 'launch', 'system.launch.py')),
        launch_arguments={
            'server_url': server_url,
            'signaling_url': signaling_url,
            'image_topic': image_topic
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

    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(patrol_yolo_system_dir, 'launch', 'person_tracker.launch.py')),
        launch_arguments={
            'server_ip': server_ip,
            'mode': yolo_mode,
        }.items()
        # 이벤트 전송 동일하게 8000 포트
        # 모드 변경 및 설정 변경 8091 포트 사용 중 (동일 ip)
    )

    return LaunchDescription([
        server_ip_arg,
        image_topic_arg,
        yolo_mode_arg,
        vision_launch,
        yolo_launch,
        bridge_launch,
        audio_launch,
        realsense_launch
    ])
