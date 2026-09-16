from glob import glob
from setuptools import find_packages, setup


package_name = "project3_sensors"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="zhuchao",
    maintainer_email="zhuchao@example.com",
    description="Project 3 DVL, depth and SVIn navigation adapters.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "sensor_simulator = project3_sensors.sensor_simulator:main",
            "camera_info_publisher = project3_sensors.camera_info_publisher:main",
            "dead_reckoning_guard = project3_sensors.dead_reckoning_guard:main",
            "svin_odom_adapter = project3_sensors.svin_odom_adapter:main",
            "navigation_evaluator = project3_sensors.navigation_evaluator:main",
        ],
    },
)
