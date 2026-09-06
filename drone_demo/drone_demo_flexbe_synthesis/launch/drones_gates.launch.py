"""Launch the synthesis server for the portable four-gate drone demo."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


PACKAGE_NAME = 'drone_demo_flexbe_synthesis'


def _package_file(directory, filename):
    """Return a package-share substitution for one installed demo file."""
    return PathJoinSubstitution([FindPackageShare(PACKAGE_NAME), directory, filename])


def generate_launch_description():
    """Start a capability-driven server configured for the four-gate route."""
    return LaunchDescription([
        DeclareLaunchArgument('system_name', default_value='drone_gates_demo'),
        DeclareLaunchArgument(
            'global_mappings_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('flexbe_synthesis_generic'),
                'mappings',
                'global_mappings.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'custom_mappings_path',
            default_value=_package_file('mappings', 'custom_mappings.yaml'),
        ),
        DeclareLaunchArgument(
            'capabilities_path',
            default_value=_package_file('capabilities', 'drones_gates.yaml'),
        ),
        DeclareLaunchArgument(
            'spec_path',
            default_value=_package_file('specs', 'drones_gates.yaml'),
        ),
        Node(
            package='flexbe_synthesis_core',
            executable='flexbe_synthesis_server',
            name='flexbe_synthesis',
            output='screen',
            parameters=[{
                'preprocesses_filepath': _package_file('pipeline', 'preprocesses_def.yaml'),
                'processes_filepath': _package_file('pipeline', 'capability_processes_def.yaml'),
                'preprocesses_data_filepath': _package_file('pipeline', 'preprocesses_data.yaml'),
                'processes_data_filepath': _package_file('pipeline', 'processes_data.yaml'),
                'system_name': LaunchConfiguration('system_name'),
                'global_mappings_path': LaunchConfiguration('global_mappings_path'),
                'custom_mappings_path': LaunchConfiguration('custom_mappings_path'),
                'capabilities_path': LaunchConfiguration('capabilities_path'),
                'spec_path': LaunchConfiguration('spec_path'),
            }],
        ),
    ])
