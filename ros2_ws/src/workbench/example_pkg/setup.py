from setuptools import setup
import os
from glob import glob

package_name = 'example_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # uncomment if you plan to put RViz or YAML files here, otherwise ignore
        # (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.rviz')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andy',
    maintainer_email='andy@todo.todo',
    description='example package for teleop_twist_keyboard',
    license='MIT',
)
