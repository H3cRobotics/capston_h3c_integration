from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    realsense_dir = get_package_share_directory('realsense2_camera')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(realsense_dir, 'launch', 'rs_launch.py')
            ),
            launch_arguments={
                'rgb_camera.color_profile': '1280x720x30',
                'depth_module.depth_profile': '1280x720x30',
                'enable_depth': 'true',
                'align_depth.enable': 'true',
                'rgb_camera.enable_auto_exposure': 'true',
            }.items()
        )
    ])