from setuptools import setup
import os
from glob import glob

package_name = 'startup_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'udev'), glob('udev/*.rules')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.todo',
    description='Master bringup package for launching the robot',
    license='MIT',
    # Scripts installed into lib/<pkg> so `ros2 run startup_robot <name>.py`
    # and Node(package='startup_robot', executable='<name>.py') both work.
    scripts=['scripts/world_pose_from_ekf.py'],
)
