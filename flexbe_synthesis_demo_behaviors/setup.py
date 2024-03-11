#!/usr/bin/env python
"""Setup package."""

from setuptools import setup

PACKAGE_NAME = 'flexbe_synthesis_demo_behaviors'

setup(
    name=PACKAGE_NAME,
    version='0.0.1',
    packages=[PACKAGE_NAME],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David Conner',
    maintainer_email='robotics@cnu.edu',
    description='Demonstration behaviors for Wolf-Goat-Corn-Farmer demonstration',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'example_behavior_sm = flexbe_synthesis_demo_behaviors.example_behavior_sm',
        ],
    },
)
