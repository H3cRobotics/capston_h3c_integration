from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    server_base_url_arg = DeclareLaunchArgument(
        'server_base_url',
        default_value='http://192.168.0.221:8000',
        description='Server base URL',
    )

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB2',
        description='USB-TTL serial port connected to PDIST120',
    )

    update_period_sec_arg = DeclareLaunchArgument(
        'update_period_sec',
        default_value='60.0',
        description='Battery update period in seconds',
    )

    empty_voltage_arg = DeclareLaunchArgument(
        'empty_voltage',
        default_value='21.5',
        description='Voltage treated as 0 percent',
    )

    full_voltage_arg = DeclareLaunchArgument(
        'full_voltage',
        default_value='25.0',
        description='Voltage treated as 100 percent',
    )

    enable_server_post_arg = DeclareLaunchArgument(
        'enable_server_post',
        default_value='true',
        description='Enable HTTP POST to server',
    )

    enable_topic_publish_arg = DeclareLaunchArgument(
        'enable_topic_publish',
        default_value='true',
        description='Enable /robot/battery topic publish',
    )

    battery_node = Node(
        package='battery',
        executable='battery_node',
        name='battery_node',
        output='screen',
        parameters=[
            {
                'server_base_url': LaunchConfiguration('server_base_url'),
                'serial_port': LaunchConfiguration('serial_port'),
                'update_period_sec': ParameterValue(
                    LaunchConfiguration('update_period_sec'),
                    value_type=float,
                ),
                'empty_voltage': ParameterValue(
                    LaunchConfiguration('empty_voltage'),
                    value_type=float,
                ),
                'full_voltage': ParameterValue(
                    LaunchConfiguration('full_voltage'),
                    value_type=float,
                ),
                'enable_server_post': ParameterValue(
                    LaunchConfiguration('enable_server_post'),
                    value_type=bool,
                ),
                'enable_topic_publish': ParameterValue(
                    LaunchConfiguration('enable_topic_publish'),
                    value_type=bool,
                ),
                'battery_topic': '/robot/battery',
                'server_endpoint': '/robot/battery',
            }
        ],
    )

    return LaunchDescription([
        server_base_url_arg,
        serial_port_arg,
        update_period_sec_arg,
        empty_voltage_arg,
        full_voltage_arg,
        enable_server_post_arg,
        enable_topic_publish_arg,
        battery_node,
    ])

