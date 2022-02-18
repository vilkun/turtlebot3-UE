import os
from glob import glob
from setuptools import setup

package_name = 'turtlebot3_tests'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (f'lib/{package_name}', glob(f'{package_name}/[!_]**')),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/config', glob('config/*.yaml')),
        (f'share/{package_name}/test', glob('test/[!_]**')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='duc',
    maintainer_email='ducanh.than@rapyuta-robotics.com',
    description='ROS2 tests for turtlebot3',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoints_follower = turtlebot3_tests.waypoints_follower:main',
        ],
    },
)
