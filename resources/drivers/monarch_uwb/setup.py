#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
    packages=['monarch_uwb', 'monarch_uwb_ros'],
    package_dir={'monarch_uwb': 'common/src/monarch_uwb',
                 'monarch_uwb_ros': 'ros/src/monarch_uwb_ros'}
)

setup(**d)
