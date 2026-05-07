from setuptools import setup
from glob import glob
import os

package_name = 'patrol_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'requests', 'python-can'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='ROS2 bridge nodes for robot pose, teach trigger, and patrol API integration.',
    license='Apache-2.0',
    entry_points={
			'console_scripts': [
			    'robot_pose_sender = patrol_bridge.robot_pose_sender:main',
			    'robot_goal_sender = patrol_bridge.robot_goal_sender:main', 
			    'can_teacher_node = patrol_bridge.can_teacher_node:main',
			    'dummy_pose_input = patrol_bridge.dummy_pose_input:main',
			    'patrol_command_bridge = patrol_bridge.patrol_command_bridge:main',
                'dummy_patrol_server_node = patrol_bridge.dummy_patrol_server_node:main',

                # control evaluation
                'cal_errer_node = patrol_bridge.cal_errer_node:main',

                
			],
    },
)