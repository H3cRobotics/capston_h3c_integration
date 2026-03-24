from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    server_url = "http://192.168.0.16:8000"

    return LaunchDescription([

        Node(
            package="patrol_bridge",
            executable="robot_pose_sender",
            name="robot_pose_sender",
            parameters=[
                {
                    "server_url": server_url,
                    "pose_topic": "/robot_pose",
                    "status_topic": "/robot_status",
                    "post_period_sec": 0.5,
                }
            ],
        ),

        Node(
            package="patrol_bridge",
            executable="robot_goal_sender",
            name="robot_goal_sender",
            parameters=[
                {
                    "server_url": server_url,
                    "goal_topic": "/goal_pose_2d",
                    "next_place_topic": "/next_place_id",
                }
            ],
        ),

        Node(
            package="patrol_bridge",
            executable="teacher_node",
            name="teacher_node",
            parameters=[
                {
                    "server_url": server_url,
                    "robot_pose_topic": "/robot_pose",
                    "teach_trigger_topic": "/teach_trigger",
                    "default_patrol_enabled": True,
                    "place_prefix": "P",
                }
            ],
        ),

        Node(
            package="patrol_bridge",
            executable="can_teach_trigger",
            name="can_teach_trigger",
            parameters=[
                {
                    "can_interface": "can0",
                    "teach_can_id": 0x102,
                    "debounce_sec": 5.0,
                    "teach_trigger_topic": "/teach_trigger",
                }
            ],
        ),
        
        Node(
				    package="patrol_bridge",
				    executable="patrol_command_bridge",
				    name="patrol_command_bridge",
				    parameters=[
				        {
				            "server_url": server_url,
				            "waypoints_topic": "/patrol/waypoints_json",
				            "command_topic": "/patrol/command",
				            "reload_waypoints_topic": "/patrol/reload_waypoints",
				            "command_poll_period_sec": 1.0,
				        }
				    ],
				),
    ])