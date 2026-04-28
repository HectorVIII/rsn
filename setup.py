from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'rsn'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='huitao',
    maintainer_email='jszdhyjs@gmail.com',
    description='Voice-guided surgical instrument handover demo with xArm and ZED.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'demo_coordinator = rsn.demo_coordinator:main',
            'xarm_controller_node = rsn.xarm_controller_node:main',
            'zed_hand_node = rsn.zed_hand_node:main',
            'voice_command_node = rsn.voice_command_node:main',
            'instrument_detection_node = rsn.instrument_detection_node:main',
        ],
    },
)
