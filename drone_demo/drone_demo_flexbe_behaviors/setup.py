"""Setup for drone_demo_flexbe_behaviors."""

from glob import glob

from setuptools import find_packages, setup


package_name = 'drone_demo_flexbe_behaviors'


setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/manifest', glob('manifest/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David Conner',
    maintainer_email='robotics@cnu.edu',
    description='FlexBE behavior helpers for the flexible-drones synthesis demo.',
    license='Apache-2.0',
    tests_require=['pytest'],
)
