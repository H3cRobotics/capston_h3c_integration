from setuptools import setup

package_name = 'security_audio_classifier'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/classifier.launch.py']),
        ('share/' + package_name + '/config', ['config/classifier.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='YAMNet classifier node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'yamnet_classifier_node = security_audio_classifier.yamnet_classifier_node:main',
        ],
    },
)