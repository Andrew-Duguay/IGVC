from setuptools import setup
import os
from glob import glob

package_name = 'littleblue_vision'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mark',
    maintainer_email='mark@todo.todo',
    description='Vision processing for IGVC: lane detection and obstacle projection',
    license='MIT',
    entry_points={
        'console_scripts': [
            'lane_detector_node = littleblue_vision.lane_detector_node:main',
            'obstacle_projector_node = littleblue_vision.obstacle_projector_node:main',
            'lane_candidate_node = littleblue_vision.lane_detector_candidates:main',
            'lane_benchmark_node = littleblue_vision.lane_benchmark_node:main',
            'lane_accumulator_node = littleblue_vision.lane_accumulator_node:main',
            'lidar_obstacle_node = littleblue_vision.lidar_obstacle_node:main',
        ],
    },
)
