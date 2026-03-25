import os
import sys
from setuptools import find_packages, setup
from setuptools.command.develop import develop as _develop


package_name = "robotsky_sim"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rocketsky",
    maintainer_email="759094438@qq.com",
    description="TODO: Package description",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "robot_sim = robotsky_sim.robot_sim:main",
        ],
    },
)
