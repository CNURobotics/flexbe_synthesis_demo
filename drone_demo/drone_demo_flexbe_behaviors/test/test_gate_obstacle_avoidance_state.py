# Copyright 2026 Christopher Newport University
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Tests for per-segment other-gate obstacle avoidance."""

from drone_demo_flexbe_behaviors.gate_obstacle_avoidance_state import (
    GateObstacleAvoidanceState,
)
import pytest


def test_home_to_gate_avoids_the_other_three_gates():
    """The target gate is excluded; the other three gates get avoidance cylinders."""
    obstacles = GateObstacleAvoidanceState._avoidance_obstacles('home', 'A1', 1.75, 0.9)

    assert obstacles == [
        {'type': 'cylinder', 'x': 0.0, 'y': 1.75, 'radius': 0.9, 'height': 0.0},
        {'type': 'cylinder', 'x': -1.75, 'y': 0.0, 'radius': 0.9, 'height': 0.0},
        {'type': 'cylinder', 'x': 0.0, 'y': -1.75, 'radius': 0.9, 'height': 0.0},
    ]


def test_gate_to_gate_excludes_both_endpoints():
    """Both segment endpoints are excluded; the remaining two gates are avoided."""
    obstacles = GateObstacleAvoidanceState._avoidance_obstacles('B2', 'D1', 1.75, 0.9)

    assert obstacles == [
        {'type': 'cylinder', 'x': 1.75, 'y': 0.0, 'radius': 0.9, 'height': 0.0},
        {'type': 'cylinder', 'x': -1.75, 'y': 0.0, 'radius': 0.9, 'height': 0.0},
    ]


def test_shared_gate_letter_excludes_only_that_gate():
    """A reverse-crossing pair sharing the same gate letter excludes one gate."""
    obstacles = GateObstacleAvoidanceState._avoidance_obstacles('C1', 'C2R', 1.75, 0.9)

    assert obstacles == [
        {'type': 'cylinder', 'x': 1.75, 'y': 0.0, 'radius': 0.9, 'height': 0.0},
        {'type': 'cylinder', 'x': 0.0, 'y': 1.75, 'radius': 0.9, 'height': 0.0},
        {'type': 'cylinder', 'x': 0.0, 'y': -1.75, 'radius': 0.9, 'height': 0.0},
    ]


def test_home_to_home_avoids_all_four_gates():
    """Neither endpoint is a gate, so all four gates get avoidance cylinders."""
    obstacles = GateObstacleAvoidanceState._avoidance_obstacles('home', 'home', 1.75, 0.9)

    assert len(obstacles) == 4


def test_invalid_gate_identifier_is_rejected():
    """Gate identifiers outside the supported course are rejected."""
    with pytest.raises(ValueError, match='gate_id'):
        GateObstacleAvoidanceState._avoidance_obstacles('E3', 'home', 1.75, 0.9)
