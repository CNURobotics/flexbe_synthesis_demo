#!/usr/bin/env python3
# Copyright (c) 2024, Sebastian Castro, The AI Institute
# SPDX-License-Identifier: BSD-3-Clause
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""Run FlexBE PyRoboSim demo worlds."""

import os
import threading

from ament_index_python.packages import get_package_share_directory
from pyrobosim.core import WorldYamlLoader
from pyrobosim.gui import start_gui
from pyrobosim_ros.ros_interface import WorldROSWrapper
import rclpy


def create_ros_node():
    """Initialize the ROS node."""
    rclpy.init()
    node = WorldROSWrapper(state_pub_rate=0.1, dynamics_rate=0.01)
    node.declare_parameter('problem_number', 1)

    # Set the world file.
    problem_number = node.get_parameter('problem_number').value
    node.get_logger().info(f'Starting problem number {problem_number}')
    world_file = os.path.join(
        get_package_share_directory('pyrobosim_flexbe_problems'),
        'worlds',
        f'world{problem_number}.yaml',
    )
    world = WorldYamlLoader().from_file(world_file)
    node.set_world(world)

    return node


def main():
    node = create_ros_node()

    # Start ROS node in separate thread
    ros_thread = threading.Thread(target=lambda: node.start(wait_for_gui=True))
    ros_thread.start()

    # Start GUI in main thread
    start_gui(node.world)


if __name__ == '__main__':
    main()
