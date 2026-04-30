from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            namespace='camera',
            name='camera',
            output='screen',
            parameters=[{
                'camera_name': 'camera',
                'camera_namespace': 'camera',

                'enable_color': True,
                'enable_depth': True,
                'enable_sync': True,

                'align_depth.enable': True,
                'depth_module.depth_profile': '1280x720x30',
                'rgb_camera.color_profile': '1280x720x30',

                'depth_module.enable_auto_exposure': True,
                'rgb_camera.enable_auto_exposure': True,
            }]
        )
    ])