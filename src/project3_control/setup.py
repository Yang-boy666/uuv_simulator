from glob import glob
from setuptools import find_packages, setup


package_name = "project3_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="zhuchao",
    maintainer_email="zhuchao@example.com",
    description="Pipe following control stack for a BlueROV2-class UUV.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "pipe_guidance = project3_control.pipe_guidance:main",
            "velocity_controller = project3_control.velocity_controller:main",
            "thruster_allocator = project3_control.thruster_allocator:main",
            "trajectory_publisher = project3_control.trajectory_publisher:main",
        ],
    },
)
