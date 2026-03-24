from setuptools import setup

package_name = 'security_audio_frontend'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/frontend.launch.py']),
        ('share/' + package_name + '/config', ['config/frontend.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Frontend node for audio capture and trigger',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'audio_frontend_node = security_audio_frontend.audio_frontend_node:main',
            'respeaker_doa_node = security_audio_frontend.respeaker_doa_node:main',
        ],
    },
)