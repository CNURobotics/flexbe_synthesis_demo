#!/usr/bin/env python3
# Copyright 2026 Christopher Newport University
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Build avoidance obstacle cylinders for the gates not on a route segment."""

from flexbe_core import EventState, Logger


class GateObstacleAvoidanceState(EventState):
    """Build cylinders for the gates not on this route segment.

    The trajectory planner always treats every spawned gate's URDF collision
    geometry as a default obstacle; a segment's own start/target gate must
    clear its own thin default pipe geometry via a small enough
    ``obstacle_safety_margin`` planner launch parameter, not by excluding it
    here. This state instead reproduces gates_demo_sm.py's ``_get_obstacles``:
    for gates other than this segment's own endpoints, send a single, larger
    cylinder per gate to SetTrajectoryObstaclesState. That both keeps the
    drone away from gates it is not flying through this segment and replaces
    each such gate's many small merged pipe capsules with one simple cylinder,
    substantially reducing the trajectory optimizer's obstacle count.

    -- start_gate_id     string   ``home`` or a gate identifier: ``A1`` through ``D2R``.
    -- target_gate_id    string   ``home`` or a gate identifier: ``A1`` through ``D2R``.

    ># gate_distance      float   Distance from the course origin to each gate center.
    ># avoidance_radius   float   Radius of each avoidance cylinder.

    #> obstacles          list    Cylinder obstacle dicts for SetTrajectoryObstaclesState.

    <= done                       Built a (possibly empty) avoidance obstacle list.
    <= failed                     A gate identifier was invalid.
    """

    def __init__(self, start_gate_id='home', target_gate_id='home'):
        super().__init__(outcomes=['done', 'failed'],
                          input_keys=['gate_distance', 'avoidance_radius'],
                          output_keys=['obstacles'])
        self._start_gate_id = str(start_gate_id)
        self._target_gate_id = str(target_gate_id)
        self._obstacles = None

    def on_enter(self, userdata):
        self._obstacles = None
        try:
            self._obstacles = self._avoidance_obstacles(
                self._start_gate_id,
                self._target_gate_id,
                userdata.gate_distance,
                userdata.avoidance_radius,
            )
        except (TypeError, ValueError) as exc:
            Logger.logwarn(f'[{self._name}] Invalid gate avoidance: {exc}')

    def execute(self, userdata):
        if self._obstacles is None:
            return 'failed'
        userdata.obstacles = self._obstacles
        return 'done'

    @staticmethod
    def _avoidance_obstacles(start_gate_id, target_gate_id, gate_distance, avoidance_radius):
        distance = float(gate_distance)
        positions = {
            'A': (distance, 0.0),
            'B': (0.0, distance),
            'C': (-distance, 0.0),
            'D': (0.0, -distance),
        }
        radius = float(avoidance_radius)

        excluded = set()
        for gate_id in (start_gate_id, target_gate_id):
            gate = str(gate_id).strip().upper()
            if gate == 'HOME':
                continue
            if len(gate) not in (2, 3) or gate[0] not in positions:
                raise ValueError(f"gate_id must be 'home' or A1 through D2R, got {gate_id!r}")
            excluded.add(gate[0])

        obstacles = []
        for letter in ('A', 'B', 'C', 'D'):
            if letter in excluded:
                continue
            x, y = positions[letter]
            obstacles.append({'type': 'cylinder', 'x': x, 'y': y, 'radius': radius, 'height': 0.0})
        return obstacles
