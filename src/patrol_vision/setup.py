from setuptools import setup
from glob import glob
import os

package_name = 'patrol_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='patrol robot nodes',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'camera_publisher = patrol_vision.camera_publisher:main',
            'capture_sender = patrol_vision.capture_sender:main',
            'patrol_http_bridge = patrol_vision.patrol_http_bridge:main',
            'send_test = patrol_vision.send_test:main',
        ],
    },
)