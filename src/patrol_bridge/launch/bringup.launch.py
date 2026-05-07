from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    server_url = LaunchConfiguration('server_url')

    # Pose sender
    pose_topic = LaunchConfiguration('pose_topic')
    status_topic = LaunchConfiguration('status_topic')
    post_period_sec = LaunchConfiguration('post_period_sec')

    # Goal sender
    goal_topic = LaunchConfiguration('goal_topic')
    next_place_topic = LaunchConfiguration('next_place_topic')

    # Teacher
    default_patrol_enabled = LaunchConfiguration('default_patrol_enabled')
    place_prefix = LaunchConfiguration('place_prefix')

    # CAN
    can_interface = LaunchConfiguration('can_interface')
    teach_can_id = LaunchConfiguration('teach_can_id')
    debounce_sec = LaunchConfiguration('debounce_sec')
    http_timeout_sec = LaunchConfiguration('http_timeout_sec')
    queue_size = LaunchConfiguration('queue_size')

    # Command Bridge
    waypoints_topic = LaunchConfiguration('waypoints_topic')
    command_topic = LaunchConfiguration('command_topic')
    reload_waypoints_topic = LaunchConfiguration('reload_waypoints_topic')
    command_poll_period_sec = LaunchConfiguration('command_poll_period_sec')

    # Control evaluation
    control_eval_csv_dir = LaunchConfiguration('control_eval_csv_dir')

    return LaunchDescription([
        DeclareLaunchArgument('server_url', default_value='http://192.168.0.16:8000'),

        DeclareLaunchArgument('pose_topic', default_value='/robot_pose'),
        DeclareLaunchArgument('status_topic', default_value='/robot_status'),
        DeclareLaunchArgument('post_period_sec', default_value='0.5'),

        DeclareLaunchArgument('goal_topic', default_value='/goal_pose_2d'),
        DeclareLaunchArgument('next_place_topic', default_value='/next_place_id'),

        DeclareLaunchArgument('default_patrol_enabled', default_value='True'),
        DeclareLaunchArgument('place_prefix', default_value='P'),

        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('teach_can_id', default_value='258'),  # 0x102
        DeclareLaunchArgument('debounce_sec', default_value='1.0'),
        DeclareLaunchArgument('http_timeout_sec', default_value='3.0'),
        DeclareLaunchArgument('queue_size', default_value='10'),

        DeclareLaunchArgument('waypoints_topic', default_value='/patrol/waypoints_json'),
        DeclareLaunchArgument('command_topic', default_value='/patrol/command'),
        DeclareLaunchArgument('reload_waypoints_topic', default_value='/patrol/reload_waypoints'),
        DeclareLaunchArgument('command_poll_period_sec', default_value='1.0'),

        DeclareLaunchArgument(
            'control_eval_csv_dir',
            default_value='/home/choisuhyun/scene_ad_for_patrol_robot/control_eval_logs'
        ),

        Node(
            package="patrol_bridge",
            executable="robot_pose_sender",
            name="robot_pose_sender",
            parameters=[{
                "server_url": server_url,
                "pose_topic": pose_topic,
                "status_topic": status_topic,
                "post_period_sec": post_period_sec,
            }],
        ),

        Node(
            package="patrol_bridge",
            executable="robot_goal_sender",
            name="robot_goal_sender",
            parameters=[{
                "server_url": server_url,
                "goal_topic": goal_topic,
                "next_place_topic": next_place_topic,
            }],
        ),

        Node(
            package="patrol_bridge",
            executable="can_teacher_node",
            name="can_teacher_node",
            parameters=[{
                "server_url": server_url,
                "robot_pose_topic": pose_topic,
                "can_interface": can_interface,
                "teach_can_id": teach_can_id,
                "default_patrol_enabled": default_patrol_enabled,
                "place_prefix": place_prefix,
                "debounce_sec": debounce_sec,
                "http_timeout_sec": http_timeout_sec,
                "queue_size": queue_size,
            }],
        ),

        Node(
            package="patrol_bridge",
            executable="patrol_command_bridge",
            name="patrol_command_bridge",
            parameters=[{
                "server_url": server_url,
                "waypoints_topic": waypoints_topic,
                "command_topic": command_topic,
                "reload_waypoints_topic": reload_waypoints_topic,
                "command_poll_period_sec": command_poll_period_sec,
            }],
        ),

        Node(
            package="patrol_bridge",
            executable="cal_errer_node",
            name="cal_errer_node",
            parameters=[{
                "goal_topic": goal_topic,
                "pose_topic": pose_topic,
                "next_place_topic": next_place_topic,
                "status_topic": status_topic,
                "csv_dir": control_eval_csv_dir,
            }],
            output="screen",
        ),
    ])