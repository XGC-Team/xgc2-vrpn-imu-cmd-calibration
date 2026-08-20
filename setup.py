#!/usr/bin/env python3

from setuptools import find_packages, setup

setup(
    name="vrpn_imu_cmd_calibration",
    version="0.2.0",
    package_dir={"": "src"},
    packages=find_packages("src"),
)
