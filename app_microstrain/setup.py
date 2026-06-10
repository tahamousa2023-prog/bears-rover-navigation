from setuptools import setup
from glob import glob
import os

package_name = 'app_microstrain'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob('launch/*.py')),
        ('share/' + package_name + '/config',
            glob('config/*.yml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BEARS Navigation Team',
    maintainer_email='navigation@bears-space.de',
    description='CV7-AHRS IMU driver package for BEARS rover',
    license='MIT',
    entry_points={
        'console_scripts': [
            'cv7_logger = app_microstrain.scripts.cv7_ros2_logger:main',
        ],
    },
)
