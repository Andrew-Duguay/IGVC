from setuptools import setup, find_packages

setup(
    name="astar-nav",
    version="0.1.0",
    description="A* path planning with costmaps and vision utilities",
    author="Barakat",
    python_requires=">=3.8",
    packages=find_packages(),
    install_requires=[
        "numpy",
        "opencv-python",
        "matplotlib"
    ],
)
