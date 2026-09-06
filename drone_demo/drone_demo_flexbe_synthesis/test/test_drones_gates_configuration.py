# Copyright 2026 Christopher Newport University
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Structural checks for the portable four-gate synthesis model."""

from pathlib import Path

from drone_demo_flexbe_synthesis.request_gates_flight import GATES_FLIGHT_DEFAULTS
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _capabilities():
    with (PACKAGE_ROOT / 'capabilities' / 'drones_gates.yaml').open() as stream:
        return yaml.safe_load(stream)


def test_default_request_bootstraps_the_four_gate_route():
    """Defaults select the documented route, bootstrap, and goal."""
    assert GATES_FLIGHT_DEFAULTS['initial_conditions'] == [
        'begin_game_a',
        'gate1=0',
        'gate2=11',
        'gate3=6',
        'gate4=13',
    ]
    assert GATES_FLIGHT_DEFAULTS['goals'] == ['log_finished']
    assert GATES_FLIGHT_DEFAULTS['outcomes'] == ['finished', 'failed']


def test_enumerated_launch_uses_the_request_system_name():
    """Changing the encoding must not change the system/cache identity."""
    launch_source = (
        PACKAGE_ROOT / 'launch' / 'drones_gates_enumerated.launch.py'
    ).read_text()

    expected = (
        "DeclareLaunchArgument('system_name', "
        f"default_value='{GATES_FLIGHT_DEFAULTS['system_name']}')"
    )
    assert expected in launch_source


def test_recovery_landing_must_complete_before_disarm_path_ends():
    """Recovery completion is modeled explicitly rather than as a capability name."""
    model = _capabilities()
    capabilities = model['capabilities']

    assert 'recovery_landed' in model['memory']['sys_props']
    assert 'recovery_landed' in capabilities['recover_land']['postconditions']
    assert 'recovery_landed' in capabilities['log_recovery_finished']['preconditions']
    assert 'recover_land' not in capabilities['log_recovery_finished']['preconditions']


def test_all_flight_interfaces_come_from_flexible_drones_packages():
    """The model uses official flexible-drones states, except gate conversion."""
    model = _capabilities()
    interfaces = {
        capability['interface']
        for capability in model['capabilities'].values()
    }

    assert 'GateTrajectoryBoundaryState' in interfaces
    assert {
        'ArmState',
        'TakeoffState',
        'GoToState',
        'UploadTrajectoryState',
        'ExecuteTrajectoryState',
        'LandState',
    }.issubset(interfaces)
    assert not any('Crazyflie' in interface for interface in interfaces)
    assert not any(interface.startswith('Drone') for interface in interfaces)


def test_drone_namespace_is_a_behavior_parameter_not_synthesis_userdata():
    """The route request does not choose a drone namespace or add userdata."""
    model = _capabilities()
    interfaces = model['capabilities']

    assert 'drone' not in model['variable_mappings']['sys_props']
    for name in ('upload_trajectory', 'arm', 'takeoff', 'go_home', 'execute_trajectory', 'land'):
        assert interfaces[name]['parameters']['namespace'] == 'self.drone_name'
        assert 'drone_name' not in interfaces[name].get('userdata_in', {})


def test_trajectory_approval_requires_operator_autonomy():
    """Executing requires High autonomy and cancelling requires Full autonomy."""
    approval = _capabilities()['capabilities']['approve_trajectory']

    assert approval['autonomy'] == {
        'execute': 2,
        'cancel': 3,
    }
