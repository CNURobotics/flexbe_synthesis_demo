#!/usr/bin/env python

# Copyright 2026 Christopher Newport University
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

"""Tests for the is_at_goal console script module."""

from pyrobosim_flexbe_problems import is_at_goal
from pyrobosim_flexbe_problems.world_state import WorldState


def test_is_at_goal_uses_packaged_world_state():
    """Verify the console script imports the packaged WorldState helper."""
    assert is_at_goal.WorldState is WorldState
