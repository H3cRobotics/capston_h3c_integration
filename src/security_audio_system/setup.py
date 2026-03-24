from setuptools import setup

package_name = 'security_audio_system'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/system.launch.py', 'launch/bringup.launch.py']),
        ('share/' + package_name + '/config', ['config/system.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='System nodes for sound event management and transfer',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sound_event_manager_node = security_audio_system.sound_event_manager_node:main',
            'clip_transfer_node = security_audio_system.clip_transfer_node:main',
            'sound_event_monitor_node = security_audio_system.sound_event_monitor_node:main',
            'dummy_pose_input_node = security_audio_system.dummy_pose_input_node:main',
        ],
    },
)