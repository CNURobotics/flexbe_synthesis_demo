# Copyright 2026 Christopher Newport University
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Tests for compact gate identifier conversion."""

from drone_demo_flexbe_behaviors.gate_trajectory_boundary_state import (
    GateTrajectoryBoundaryState,
)
import pytest


def test_forward_gate_uses_target_frame_and_forward_velocity():
    """A forward gate produces the upstream target-frame convention."""
    boundary = GateTrajectoryBoundaryState._make_boundary('b2', 0.5, (0.0, 0.0, 1.0))

    assert boundary.frame_id == 'gate_B_target_2'
    assert boundary.twist.linear.x == 0.5
    assert boundary.pose.orientation.w == 1.0
    assert boundary.pose.orientation.z == 0.0


def test_reverse_gate_reverses_velocity_and_orientation():
    """A reverse gate retains its target frame with reverse crossing data."""
    boundary = GateTrajectoryBoundaryState._make_boundary('C1R', 0.5, (0.0, 0.0, 1.0))

    assert boundary.frame_id == 'gate_C_target_1'
    assert boundary.twist.linear.x == -0.5
    assert boundary.pose.orientation.w == 0.0
    assert boundary.pose.orientation.z == 1.0


def test_home_uses_map_frame_and_requested_position():
    """The home boundary is an explicit map-frame pose."""
    boundary = GateTrajectoryBoundaryState._make_boundary('home', 0.5, (1.0, 2.0, 1.5))

    assert boundary.frame_id == 'map'
    assert boundary.pose.position.x == 1.0
    assert boundary.pose.position.y == 2.0
    assert boundary.pose.position.z == 1.5


def test_invalid_gate_identifier_is_rejected():
    """Gate identifiers outside the supported course are rejected."""
    with pytest.raises(ValueError, match='gate_id'):
        GateTrajectoryBoundaryState._make_boundary('E3', 0.5, (0.0, 0.0, 1.0))
