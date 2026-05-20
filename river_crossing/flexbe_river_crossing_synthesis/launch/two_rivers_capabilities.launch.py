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

"""Launch file for the two-rivers capability-based synthesis demo."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def create_synthesis_node():
    """Create the synthesis node for the Two Rivers capabilities launch."""
    return Node(
        package='flexbe_synthesis_core',
        executable='flexbe_synthesis_server',
        name='flexbe_synthesis',
        output='screen',
        parameters=[
            {
                'preprocesses_filepath': PathJoinSubstitution(
                    [
                        FindPackageShare('flexbe_river_crossing_synthesis'),
                        'pipeline',
                        'preprocesses_def.yaml',
                    ]
                ),
                'processes_filepath': PathJoinSubstitution(
                    [
                        FindPackageShare('flexbe_river_crossing_synthesis'),
                        'pipeline',
                        'capability_processes_def.yaml',
                    ]
                ),
                'preprocesses_data_filepath': PathJoinSubstitution(
                    [
                        FindPackageShare('flexbe_river_crossing_synthesis'),
                        'pipeline',
                        'preprocesses_data.yaml',
                    ]
                ),
                'processes_data_filepath': PathJoinSubstitution(
                    [
                        FindPackageShare('flexbe_river_crossing_synthesis'),
                        'pipeline',
                        'processes_data.yaml',
                    ]
                ),
                'system_name': LaunchConfiguration('system_name'),
                'global_mappings_path': LaunchConfiguration('global_mappings_path'),
                'capabilities_path': LaunchConfiguration('capabilities_path'),
                'spec_path': LaunchConfiguration('spec_path'),
            }
        ],
    )


def generate_launch_description():
    """Generate launch description for the Two Rivers capabilities demo."""
    declared_arguments = [
        DeclareLaunchArgument(
            'system_name',
            default_value='two_rivers_capabilities',
            description='Name of system capabilities',
        ),
        DeclareLaunchArgument(
            'global_mappings_path',
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare('flexbe_synthesis_generic'),
                    'mappings',
                    'global_mappings.yaml',
                ]
            ),
            description='Path to global outcome mapping file',
        ),
        DeclareLaunchArgument(
            'capabilities_path',
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare('flexbe_river_crossing_synthesis'),
                    'capabilities',
                    'two_rivers_parsed.yaml',
                ]
            ),
            description='Path to capabilities file',
        ),
        DeclareLaunchArgument(
            'spec_path',
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare('flexbe_river_crossing_synthesis'),
                    'specs',
                    'two_rivers_env.yaml'
                ]
            ),
            description='Path to specifications file',
        ),
    ]

    return LaunchDescription(declared_arguments + [create_synthesis_node()])
