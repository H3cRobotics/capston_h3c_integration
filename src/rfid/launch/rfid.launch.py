from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 터미널에서 입력받을 인자(Argument)의 변수를 생성합니다.
    server_url_arg = LaunchConfiguration('server_url')

    # 2. 런치 인자를 선언하고, 아무것도 입력하지 않았을 때의 기본값을 설정합니다.
    declare_server_url_cmd = DeclareLaunchArgument(
        'server_url',
        default_value='http://192.168.0.16:8000',
        description='Base URL for the server'
    )

    # 3. 노드를 실행할 때, 위에서 받은 값을 'server_base_url' 파라미터에 넣어줍니다.
    secondary_auth_node = Node(
        package='rfid',
        executable='secondary_auth_node',
        name='secondary_auth_node',
        output='screen',
        parameters=[{
            'server_base_url': server_url_arg
        }]
    )

    # 4. 선언한 인자와 노드를 모두 실행 목록에 담아 반환합니다.
    return LaunchDescription([
        declare_server_url_cmd,
        secondary_auth_node
    ])