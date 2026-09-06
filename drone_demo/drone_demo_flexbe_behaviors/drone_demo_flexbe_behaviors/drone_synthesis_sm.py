#!/usr/bin/env python3
# Copyright 2026 Christopher Newport University
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Starter behavior for the portable drone synthesis demonstration."""

from flexbe_core import Autonomy, Behavior, initialize_flexbe_core, OperatableStateMachine
from flexbe_states.log_state import LogState


class drone_synthesisSM(Behavior):
    """Provide startup parameters and userdata for the synthesized container."""

    def __init__(self, node):
        super().__init__()
        self.name = 'drone_synthesis'
        self.add_parameter('drone_name', 'cf1')
        self.add_parameter('gate_radius', 0.9)
        self.add_parameter('gate_distance', 1.5)
        initialize_flexbe_core(node)

    def create(self):
        """Create the temporary container replaced by the WebUI synthesis result."""
        state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
        state_machine.userdata.group_mask = 0
        state_machine.userdata.takeoff_height = 1.0
        state_machine.userdata.takeoff_duration_sec = 2.0
        state_machine.userdata.frame = 0
        state_machine.userdata.home_position = [0.0, 0.0, 1.0]
        state_machine.userdata.go_home_duration_sec = 2.0
        state_machine.userdata.timescale = 1.0
        state_machine.userdata.reversed = False
        state_machine.userdata.relative = False
        state_machine.userdata.landing_height = 0.05
        state_machine.userdata.landing_duration_sec = 2.0
        state_machine.userdata.gate_radius = self.gate_radius
        state_machine.userdata.gate_distance = self.gate_distance

        synthesized_container = OperatableStateMachine(outcomes=['finished', 'failed'],
            input_keys=['group_mask', 'takeoff_height', 'takeoff_duration_sec', 'frame',
                         'home_position', 'go_home_duration_sec', 'timescale', 'reversed',
                         'relative', 'landing_height', 'landing_duration_sec',
                         'gate_radius', 'gate_distance'])

        with synthesized_container:
            OperatableStateMachine.add(
                'Temp',
                LogState(text='Temporary state to be replaced by synthesis', severity=2),
                transitions={'done': 'finished'},
                autonomy={'done': Autonomy.Off}
            )

        with state_machine:
            OperatableStateMachine.add(
                'DroneSynthesis',
                synthesized_container,
                transitions={'finished': 'finished', 'failed': 'failed'},
                autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
                remapping={'group_mask': 'group_mask',
                           'takeoff_height': 'takeoff_height',
                           'takeoff_duration_sec': 'takeoff_duration_sec',
                           'frame': 'frame',
                           'home_position': 'home_position',
                           'go_home_duration_sec': 'go_home_duration_sec',
                           'timescale': 'timescale',
                           'reversed': 'reversed',
                           'relative': 'relative',
                           'landing_height': 'landing_height',
                           'landing_duration_sec': 'landing_duration_sec',
                           'gate_radius': 'gate_radius',
                           'gate_distance': 'gate_distance'}
            )

        return state_machine
