from setuptools import setup
import os
from glob import glob

package_name = 'littleblue_autonomy'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.rviz')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mark',
    maintainer_email='mark@todo.todo',
    description='Lane following autonomy with Nav2 obstacle avoidance for IGVC',
    license='MIT',
    entry_points={
        'console_scripts': [
            'lane_follower_node = littleblue_autonomy.lane_follower_node:main',
            'data_recorder_node = littleblue_autonomy.data_recorder_node:main',
            'safety_monitor_node = littleblue_autonomy.safety_monitor_node:main',
            'cmd_vel_to_joy_node = littleblue_autonomy.cmd_vel_to_joy_node:main',
        ],
    },
)
