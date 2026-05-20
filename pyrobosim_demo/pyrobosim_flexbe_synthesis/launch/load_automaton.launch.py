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
from os import path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def _scenario_spec_filename(prefix):
    scenario = prefix.removesuffix('_demo')
    if scenario.endswith('_capabilities_parsed'):
        return f'{scenario.removesuffix("_parsed")}.yaml'
    return f'{scenario}.yaml'


def _scenario_capabilities_filename(prefix):
    scenario = prefix.removesuffix('_demo')
    if '_capabilities' in scenario:
        world = scenario.split('_', 1)[0]
        return f'{world}_capabilities.yaml'
    return 'pyrobosim.yaml'


def _param_or_default(context, name, default):
    value = LaunchConfiguration(name).perform(context)
    return value if value else default


def create_synthesis_node(context, *args, **kwargs):
    prefix = LaunchConfiguration('prefix').perform(context)
    package_share = get_package_share_directory('pyrobosim_flexbe_synthesis')
    generic_share = get_package_share_directory('flexbe_synthesis_generic')

    system_name = _param_or_default(context, 'system_name', prefix)
    spec_path = _param_or_default(
        context,
        'spec_path',
        path.join(package_share, 'specs', _scenario_spec_filename(prefix)),
    )
    capabilities_path = _param_or_default(
        context,
        'capabilities_path',
        path.join(
            package_share,
            'capabilities',
            _scenario_capabilities_filename(prefix),
        ),
    )
    automaton_path = _param_or_default(
        context,
        'automaton_path',
        path.join(package_share, 'slugs', f'{prefix}_automaton.yaml'),
    )

    return [Node(
        package='flexbe_synthesis_core',
        executable='flexbe_synthesis_server',
        name='flexbe_synthesis',
        output='screen',
        parameters=[{
            # Load the pipeline information
            'preprocesses_filepath': path.join(
                package_share,
                'pipeline',
                'preprocesses_def.yaml',
            ),
            'processes_filepath': path.join(
                package_share,
                'pipeline',
                'load_processes_def.yaml',
            ),
            # Load any custom data required by pipeline
            'preprocesses_data_filepath': path.join(
                package_share,
                'pipeline',
                'preprocesses_data.yaml',
            ),
            'processes_data_filepath': path.join(
                package_share,
                'pipeline',
                'processes_data.yaml',
            ),

            # Common parameters used by FlexBE Synthesis Manager that
            # we wish to configure at launch time
            'system_name': system_name,
            'global_mappings_path': _param_or_default(
                context,
                'global_mappings_path',
                path.join(generic_share, 'mappings', 'global_mappings.yaml'),
            ),
            'custom_mappings_path': _param_or_default(
                context,
                'custom_mappings_path',
                path.join(package_share, 'mappings', 'custom_mappings.yaml'),
            ),
            'capabilities_path': capabilities_path,
            'spec_path': spec_path,
            'automaton_path': automaton_path,
        }]
    )]


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'prefix',
            default_value='p0_full_spec_demo',
            description='Scenario prefix used to derive load-pipeline paths'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'system_name',
            default_value='',
            description='Name of system capabilities; defaults to prefix'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'global_mappings_path',
            default_value='',
            description='Path to global outcome mapping file'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'capabilities_path',
            default_value='',
            description='Path to capabilities file; defaults from prefix'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'spec_path',
            default_value='',
            description='Path to specifications file; defaults from prefix'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'custom_mappings_path',
            default_value='',
            description='Path to custom outcome mapping file'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'automaton_path',
            default_value='',
            description='Path to pre-built automaton YAML; defaults from prefix'
        )
    )
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=create_synthesis_node)])
