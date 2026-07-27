import os
from glob import glob

from setuptools import setup


package_name = "mirobot_driver"


setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="kjy",
    maintainer_email="kjy@todo.todo",
    description=(
        "Minimal ROS 2 serial driver for WLKATA Mirobot joint, XYZ, "
        "and pump control."
    ),
    license="MIT",
    entry_points={
        "console_scripts": [
            "driver = mirobot_driver.driver:main",
            "joint_command = mirobot_driver.joint_command:main",
            "xyz_command = mirobot_driver.xyz_command:main",
        ],
    },
)
