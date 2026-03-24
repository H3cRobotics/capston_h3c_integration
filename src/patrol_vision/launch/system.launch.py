from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            # 웹캠 카메라 publish node
            package="patrol_vision",
            executable="camera_publisher",
            name="camera_publisher",
            output="screen",
        ),

        Node(
            package="patrol_vision",
            executable="capture_sender",
            name="capture_sender",
            output="screen",
            parameters=[
                {
                    "server_url": "http://192.168.0.221:8000",
                    "signaling_url": "http://192.168.0.221:8001",
                    "n_frames": 5,
                    "sample_dt": 0.2,
                    "capture_timeout_s": 5.0,
                    "post_timeout_s": 10.0,
                }
            ],
        ),

        Node(
            package="patrol_vision",
            executable="patrol_http_bridge",
            name="patrol_http_bridge",
            output="screen",
        ),
    ])