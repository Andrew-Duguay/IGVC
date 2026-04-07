from setuptools import setup

package_name = "yolo_trt_ros2"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mark",
    maintainer_email="mark@example.com",
    description="YOLO TensorRT ROS2 node for Jetson",
    license="MIT",
    entry_points={
        "console_scripts": [
            "yolo_trt_node = yolo_trt_ros2.yolo_trt_node:main",
        ],
    },
)

