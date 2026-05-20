# Copyright 2024 Christopher Newport University
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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def create_synthesis_node():
    return Node(
        package='flexbe_synthesis_core',
        executable='flexbe_synthesis_server',
        name='flexbe_synthesis',
        output='screen',
        parameters=[{
            # Load the pipeline information
            'preprocesses_filepath': PathJoinSubstitution([
                FindPackageShare('pyrobosim_flexbe_synthesis'),
                'pipeline',
                'preprocesses_def.yaml'
            ]),
            'processes_filepath': PathJoinSubstitution([
                FindPackageShare('pyrobosim_flexbe_synthesis'),
                'pipeline',
                'processes_def.yaml'
            ]),
            # Load any custom data required by pipeline
            'preprocesses_data_filepath': PathJoinSubstitution([
                FindPackageShare('pyrobosim_flexbe_synthesis'),
                'pipeline',
                'preprocesses_data.yaml'
            ]),
            'processes_data_filepath': PathJoinSubstitution([
                FindPackageShare('pyrobosim_flexbe_synthesis'),
                'pipeline',
                'processes_data.yaml'
            ]),

            # Common parameters used by FlexBE Synthesis Manager that
            # we wish to configure at launch time
            'system_name': LaunchConfiguration('system_name'),
            'global_mappings_path': LaunchConfiguration('global_mappings_path'),
            'custom_mappings_path': LaunchConfiguration('custom_mappings_path'),
            'capabilities_path': LaunchConfiguration('capabilities_path'),
            'spec_path': LaunchConfiguration('spec_path'),
        }]
    )


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'system_name',
            default_value='p0_full_spec_demo',
            description='Name of system capabilities'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'global_mappings_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('flexbe_synthesis_generic'),
                'mappings',
                'global_mappings.yaml'
            ]),
            description='Path to global outcome mapping file'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'capabilities_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('pyrobosim_flexbe_synthesis'),
                'capabilities',
                'pyrobosim.yaml'
            ]),
            description='Path to capabilities file'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'spec_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('pyrobosim_flexbe_synthesis'),
                'specs',
                'p0_full_spec.yaml'
            ]),
            description='Path to specifications file'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'custom_mappings_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('pyrobosim_flexbe_synthesis'),
                'mappings',
                'custom_mappings.yaml'
            ]),
            description='Path to custom outcome mapping file'
        )
    )
    return LaunchDescription(declared_arguments + [create_synthesis_node()])
