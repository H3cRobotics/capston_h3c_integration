from setuptools import setup
from glob import glob
import os

package_name = 'person_tracker_ros'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='choisuhyun',
    maintainer_email='example@example.com',
    description='Person detection and tracking node for webcam / RealSense',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'person_tracker_node = patrol_yolo.person_tracker_node:main',
        ],
    },
)