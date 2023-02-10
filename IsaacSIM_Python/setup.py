"""Installation script for the 'cayde_robot' python package."""

from __future__ import absolute_import
from __future__ import print_function
from __future__ import division

from setuptools import setup, find_packages

import os

# Minimum dependencies required prior to installation
INSTALL_REQUIRES = []

# Installation operation
setup(
    name="IsaacSIM_Python",
    author="LW",
    version="1.0.0",
    description="Unsupport RPMFlow",
    keywords=["RPM"],
    include_package_data=True,
    install_requires=INSTALL_REQUIRES,
    packages=find_packages("."),
    classifiers=["Natural Language :: English", "Programming Language :: Python :: 3.7, 3.8"],
    zip_safe=False,
)

# EOF
