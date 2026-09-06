#!/usr/bin/env python3
# Copyright 2026 Christopher Newport University
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Convert a compact course gate identifier into a planner boundary message."""

from flexbe_core import EventState, Logger

from flexible_drones_msgs.msg import TrajectoryBoundary


class GateTrajectoryBoundaryState(EventState):
    """Build a planner boundary for ``home`` or a gate identifier such as ``A1R``.

    -- gate_id           string   ``home`` or a gate identifier: ``A1`` through ``D2R``.
    -- cruise_velocity   float    Signed x velocity magnitude at a gate.
    -- home_position     list     ``[x, y, z]`` position used for the home boundary.

    #> boundary          object   ``flexible_drones_msgs/msg/TrajectoryBoundary``.

    <= done                       A valid boundary was created.
    <= failed                     The gate identifier or home position was invalid.
    """

    def __init__(self, gate_id, cruise_velocity=0.5, home_position=(0.0, 0.0, 1.0)):
        super().__init__(outcomes=['done', 'failed'], output_keys=['boundary'])
        self._gate_id = str(gate_id)
        self._cruise_velocity = float(cruise_velocity)
        self._home_position = tuple(home_position)
        self._boundary = None

    def on_enter(self, userdata):
        self._boundary = None
        try:
            self._boundary = self._make_boundary(
                self._gate_id,
                self._cruise_velocity,
                self._home_position,
            )
        except (TypeError, ValueError) as exc:
            Logger.logwarn(f'[{self._name}] Invalid gate boundary: {exc}')

    def execute(self, userdata):
        if self._boundary is None:
            return 'failed'
        userdata.boundary = self._boundary
        return 'done'

    @staticmethod
    def _make_boundary(gate_id, cruise_velocity, home_position):
        gate = str(gate_id).strip().upper()
        boundary = TrajectoryBoundary()

        if gate == 'HOME':
            if len(home_position) != 3:
                raise ValueError('home_position must contain exactly x, y, and z')
            boundary.frame_id = 'map'
            boundary.pose.position.x = float(home_position[0])
            boundary.pose.position.y = float(home_position[1])
            boundary.pose.position.z = float(home_position[2])
            boundary.pose.orientation.w = 1.0
            return boundary

        if len(gate) not in (2, 3) or gate[0] not in 'ABCD' or gate[1] not in '12':
            raise ValueError(f"gate_id must be 'home' or A1 through D2R, got {gate_id!r}")
        if len(gate) == 3 and gate[2] != 'R':
            raise ValueError(f"gate suffix must be 'R', got {gate_id!r}")

        velocity = float(cruise_velocity)
        if len(gate) == 3:
            velocity *= -1.0
            boundary.pose.orientation.z = 1.0
        boundary.pose.orientation.w = 1.0 if velocity >= 0.0 else 0.0
        boundary.frame_id = f'gate_{gate[0]}_target_{gate[1]}'
        boundary.twist.linear.x = velocity
        return boundary
