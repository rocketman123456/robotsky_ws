import os
from glob import glob

from setuptools import find_packages, setup

package_name = "robotsky_teleop"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch", "*.launch.py"))),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    # Symlink-install uses egg-link without dist-info; importlib.metadata then breaks console_scripts.
    zip_safe=True,
    maintainer="rocketsky",
    maintainer_email="759094438@qq.com",
    description="Xbox joy to cmd_vel teleop",
    license="Apache-2.0",
    # Use scripts= so the launcher does not call load_entry_point('robotsky-teleop', ...).
    # scripts=[os.path.join("scripts", "xbox_cmd_vel")],
    entry_points={
        "console_scripts": [
            "xbox_cmd_vel = robotsky_teleop.xbox_cmd_vel_node:main",
        ],
    },
)
