# Copyright 2026 Christopher Newport University
# Capable Humanitarian Robotics and Intelligent Systems Lab (CHRISLAB)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Setup the package."""

from glob import glob
import os

from setuptools import setup

PACKAGE_NAME = 'flexbe_river_crossing_visualization'

image_files = glob(os.path.join('images', '*.png'))
sound_files = glob(os.path.join('sounds', '*.mp3'))

setup(
    name=PACKAGE_NAME,
    version='0.0.1',
    packages=[PACKAGE_NAME],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        (os.path.join('share', PACKAGE_NAME, 'images'), image_files),
        (os.path.join('share', PACKAGE_NAME, 'sounds'), sound_files),
    ],
    include_package_data=True,
    install_requires=['setuptools'],
    zip_safe=True,
    author='Joshua Luzier, David Conner',
    maintainer='David Conner',
    maintainer_email='robotics@cnu.edu',
    description='Simple visualization for Wolf-Goat-Corn-Farmer Synthesis demonstration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'river_crossing = flexbe_river_crossing_visualization.river_crossing:main',
        ],
    },
)
