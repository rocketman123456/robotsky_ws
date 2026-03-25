import os
import subprocess
import sys
from glob import glob

from setuptools import find_packages, setup
from setuptools.command.develop import develop as _develop
from setuptools.command.easy_install import easy_install as _easy_install

# System setuptools (e.g. Debian): develop subclasses easy_install and already defines
# uninstall. Pip-style setuptools (>=64): develop is a thin wrapper — no uninstall.
# Duplicating "uninstall" in user_options/boolean_options breaks distutils option
# parsing so --no-deps is not applied (colcon then errors: no such option 'no_deps').
_LEGACY_DEVELOP = issubclass(_develop, _easy_install)

package_name = "robotsky_rl_controller"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        # Only install regular files. `model/*` may contain directories
        # (timestamp folders), which breaks colcon's symlink_data step.
        (os.path.join('share', package_name, 'model'),
            [p for p in glob('model/**.pt') if os.path.isfile(p)]),
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
            "rl_controller = robotsky_rl_controller.rl_controller_node:main",
        ],
    },
)
