"""Setup the package."""

import os
from glob import glob

from setuptools import setup

PACKAGE_NAME = 'flexbe_synthesis_demo_visualization'

image_files = glob(os.path.join('images', '*.png'))
sound_files = glob(os.path.join('sounds', '*.mp3'))

# print("image files: ", image_files)
# print("sound files: ", sound_files)

setup(
    name=PACKAGE_NAME,
    version='0.0.0',
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
    maintainer='neec23',
    maintainer_email='robotics@cnu.edu',
    description='Simple visualization for Wolf-Goat-Corn-Farmer Synthesis demonstration',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'river_crossing = flexbe_synthesis_demo_visualization.river_crossing:main',
        ],
    },
)
