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

"""Behavioral tests for river-crossing FlexBE states."""

import importlib
import sys
from types import ModuleType


class UserData:
    """Small userdata stand-in that supports FlexBE-style keyed access."""

    def __init__(self, left=None, middle=None, right=None):
        """Initialize bank lists."""
        self.left = list(left or [])
        self.middle = list(middle or [])
        self.right = list(right or [])

    def __getitem__(self, key):
        """Return a bank list by name."""
        return getattr(self, key)


def _install_state_stubs(monkeypatch):
    """Install minimal ROS and FlexBE stubs needed by the state modules."""
    flexbe_core = ModuleType('flexbe_core')

    class FakeClock:
        """Clock stub returning itself as the current time."""

        nanoseconds = 0

        def now(self):
            """Return a fake timestamp."""
            return self

    class FakeNode:
        """Node stub exposing a clock."""

        def get_clock(self):
            """Return the fake clock."""
            return FakeClock()

    class EventState:
        """Minimal EventState replacement."""

        _node = FakeNode()

        def __init__(self, outcomes=None, input_keys=None, output_keys=None):
            """Initialize state metadata."""
            self._manual_transition_requested = 'canceled'
            self._name = self.__class__.__name__

        def on_exit(self, userdata):
            """Return no additional outcome."""
            return None

    class Logger:
        """No-op FlexBE logger replacement."""

        @staticmethod
        def logerr(message):
            """Ignore error logs."""

        @staticmethod
        def loginfo(message):
            """Ignore info logs."""

        @staticmethod
        def logwarn(message):
            """Ignore warning logs."""

    flexbe_core.EventState = EventState
    flexbe_core.Logger = Logger

    proxy = ModuleType('flexbe_core.proxy')

    class ProxyPublisher:
        """Publisher stub that records published messages."""

        def __init__(self, topics):
            """Initialize topic map."""
            self.topics = topics
            self.published = []

        @classmethod
        def initialize(cls, node):
            """Accept the FlexBE node placeholder."""

        def publish(self, topic, msg):
            """Record a published message."""
            self.published.append((topic, msg))

    proxy.ProxyPublisher = ProxyPublisher

    msgs = ModuleType('flexbe_river_crossing_msgs')
    msg_module = ModuleType('flexbe_river_crossing_msgs.msg')

    class WGCFStatus:
        """Message stub accepting arbitrary fields."""

        def __init__(self, **kwargs):
            """Store message fields as attributes."""
            self.__dict__.update(kwargs)

    msg_module.WGCFStatus = WGCFStatus
    msgs.msg = msg_module

    rclpy = ModuleType('rclpy')
    duration = ModuleType('rclpy.duration')

    class Duration:
        """Duration stub exposing nanoseconds."""

        def __init__(self, seconds=0.0):
            """Convert seconds to nanoseconds."""
            self.nanoseconds = int(seconds * 1000000000)

    duration.Duration = Duration

    monkeypatch.setitem(sys.modules, 'flexbe_core', flexbe_core)
    monkeypatch.setitem(sys.modules, 'flexbe_core.proxy', proxy)
    monkeypatch.setitem(sys.modules, 'flexbe_river_crossing_msgs', msgs)
    monkeypatch.setitem(sys.modules, 'flexbe_river_crossing_msgs.msg', msg_module)
    monkeypatch.setitem(sys.modules, 'rclpy', rclpy)
    monkeypatch.setitem(sys.modules, 'rclpy.duration', duration)


def _import_state_modules(monkeypatch):
    """Import state modules against local stubs."""
    _install_state_stubs(monkeypatch)
    for module_name in (
        'flexbe_river_crossing_flexbe_states.move_state',
        'flexbe_river_crossing_flexbe_states.move_state_2r',
    ):
        sys.modules.pop(module_name, None)

    move_state = importlib.import_module('flexbe_river_crossing_flexbe_states.move_state')
    move_state_2r = importlib.import_module('flexbe_river_crossing_flexbe_states.move_state_2r')
    return move_state, move_state_2r


def test_move_state_delivers_passengers_to_opposite_bank(monkeypatch):
    """Single-river state removes passengers on enter and delivers them on done."""
    move_state, _ = _import_state_modules(monkeypatch)
    state = move_state.MoveState(item='goat')
    userdata = UserData(left=['farmer', 'goat', 'wolf'], right=[])

    state.on_enter(userdata)

    assert userdata.left == ['wolf']
    assert state._passengers == ['farmer', 'goat']
    assert state._origin == 'left'
    assert state._destination == 'right'

    state._return = 'done'
    state.on_exit(userdata)

    assert userdata.left == ['wolf']
    assert userdata.right == ['farmer', 'goat']
    assert state._passengers == []


def test_move_state_2r_auto_route_returns_empty_boat_left(monkeypatch):
    """Two-river state auto-routes an empty middle-bank boat back to the left bank."""
    _, move_state_2r = _import_state_modules(monkeypatch)
    state = move_state_2r.MoveState2R(item=None)
    userdata = UserData(left=[], middle=['farmer'], right=[])

    state.on_enter(userdata)

    assert userdata.middle == []
    assert state._origin == 'middle'
    assert state._destination == 'left'
    assert state._passengers0 == ['farmer']

    state._return = 'done'
    state.on_exit(userdata)

    assert userdata.left == ['farmer']
    assert userdata.middle == []
    assert userdata.right == []


def test_move_state_2r_auto_route_carries_middle_item_right(monkeypatch):
    """Two-river state auto-routes a middle-bank item across the right river."""
    _, move_state_2r = _import_state_modules(monkeypatch)
    state = move_state_2r.MoveState2R(item='goat')
    userdata = UserData(left=[], middle=['farmer', 'goat'], right=[])

    state.on_enter(userdata)

    assert userdata.middle == []
    assert state._origin == 'middle'
    assert state._destination == 'right'
    assert state._passengers1 == ['farmer', 'goat']

    state._return = 'done'
    state.on_exit(userdata)

    assert userdata.left == []
    assert userdata.middle == []
    assert userdata.right == ['farmer', 'goat']


def test_move_state_2r_rejects_non_adjacent_explicit_route(monkeypatch):
    """Two-river state rejects explicit routes that skip the middle bank."""
    _, move_state_2r = _import_state_modules(monkeypatch)
    state = move_state_2r.MoveState2R(item='goat', destination='right')
    userdata = UserData(left=['farmer', 'goat'], middle=[], right=[])

    state.on_enter(userdata)

    assert state._return == 'failed'
    assert userdata.left == ['farmer', 'goat']
    assert userdata.middle == []
    assert userdata.right == []
    assert state._passengers0 == []
    assert state._passengers1 == []
