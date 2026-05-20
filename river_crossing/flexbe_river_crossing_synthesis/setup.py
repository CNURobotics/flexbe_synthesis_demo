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
"""Setup for flexbe_river_crossing_synthesis."""

from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'flexbe_river_crossing_synthesis'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'pipeline'), glob('pipeline/*.yaml')),
        (
            os.path.join('share', package_name, 'capabilities'),
            glob('capabilities/*.yaml'),
        ),
        (os.path.join('share', package_name, 'specs'), glob('specs/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Joshua Luzier, David Conner',
    maintainer='David Conner',
    maintainer_email='robotics@cnu.edu',
    description='Launch files, pipeline configurations, GR(1) specifications, capability definitions, '
                'and request clients for the river-crossing synthesis demonstration '
                '(WGCF and two-rivers scenarios across full-spec, capabilities, and capabilities-parsed variants).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'request_wgcf_full_spec = flexbe_river_crossing_synthesis.request_wgcf_full_spec:main',
            'request_wgcf_capabilities = flexbe_river_crossing_synthesis.request_wgcf_capabilities:main',
            'request_wgcf_capabilities_parsed = flexbe_river_crossing_synthesis.request_wgcf_capabilities_parsed:main',
            'request_two_rivers = flexbe_river_crossing_synthesis.request_two_rivers:main',
            'request_two_rivers_parsed = flexbe_river_crossing_synthesis.request_two_rivers_parsed:main',
            'request_two_rivers_capabilities = flexbe_river_crossing_synthesis.request_two_rivers_capabilities:main',
            'request_two_rivers_capabilities_parsed = flexbe_river_crossing_synthesis.request_two_rivers_capabilities_parsed:main',
        ],
    },
)
