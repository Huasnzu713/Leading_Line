# -*- coding: utf-8 -*-
import os
from glob import glob

from setuptools import setup

PACKAGE_NAME = "leading_line"

setup(
    name=PACKAGE_NAME,
    version="0.3.0",
    packages=[
        PACKAGE_NAME,
        f"{PACKAGE_NAME}.algo",
        f"{PACKAGE_NAME}.recognition",
        f"{PACKAGE_NAME}.recognition.qr",
        f"{PACKAGE_NAME}.comm",
    ],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        # launch / config
        (os.path.join("share", PACKAGE_NAME, "launch"), glob("launch/*.py")),
        (os.path.join("share", PACKAGE_NAME, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", PACKAGE_NAME, "config"), glob("config/*.yaml")),
    ],
    install_requires=[
        "setuptools",
        "PyYAML",
        "numpy",
        "opencv-python",
        "pyzbar",
    ],
    zip_safe=True,
    maintainer="Leading Line Dev",
    maintainer_email="dev@leading-line.local",
    description="ROS2 Humble vision perception + PC monitor bridge for the Ackermann car.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "perception_node = leading_line.perception_node:main",
            "pc_monitor_bridge = leading_line.pc_monitor_bridge:main",
            "vehicle_pipeline = leading_line.vehicle_pipeline:main",
        ],
    },
)
