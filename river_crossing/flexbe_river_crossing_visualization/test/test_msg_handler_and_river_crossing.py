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

"""Behavioral tests for visualization message handling and rendering."""

import importlib
import sys
from types import ModuleType, SimpleNamespace


def _install_rclpy_stub(monkeypatch):
    """Install a small rclpy stand-in for MsgHandler tests."""
    state = {'ok': False, 'init_calls': 0, 'shutdown_calls': 0}
    rclpy = ModuleType('rclpy')

    def ok():
        """Return whether the fake ROS context is active."""
        return state['ok']

    def init(args=None):
        """Record fake ROS initialization."""
        state['init_calls'] += 1
        state['ok'] = True

    def shutdown():
        """Record fake ROS shutdown."""
        state['shutdown_calls'] += 1
        state['ok'] = False

    def try_shutdown():
        """Record fake ROS shutdown attempts."""
        shutdown()

    def spin(node):
        """Do not block in tests."""

    rclpy.ok = ok
    rclpy.init = init
    rclpy.shutdown = shutdown
    rclpy.try_shutdown = try_shutdown
    rclpy.spin = spin
    rclpy.executors = SimpleNamespace(ExternalShutdownException=RuntimeError)

    class FakeLogger:
        """Logger stub that records warnings."""

        def __init__(self):
            """Initialize warning storage."""
            self.warnings = []

        def warn(self, message):
            """Record a warning message."""
            self.warnings.append(message)

    fake_logger = FakeLogger()
    logging_module = ModuleType('rclpy.logging')
    logging_module.get_logger = lambda name: fake_logger
    rclpy.logging = logging_module

    node_module = ModuleType('rclpy.node')

    class Node:
        """Minimal rclpy Node replacement."""

        def __init__(self, node_name):
            """Store node name."""
            self.node_name = node_name
            self.destroyed = False

        def create_subscription(self, msg_type, topic_name, callback, queue_size):
            """Return subscription metadata."""
            return (msg_type, topic_name, callback, queue_size)

        def destroy_node(self):
            """Mark the node destroyed."""
            self.destroyed = True

    node_module.Node = Node

    monkeypatch.setitem(sys.modules, 'rclpy', rclpy)
    monkeypatch.setitem(sys.modules, 'rclpy.logging', logging_module)
    monkeypatch.setitem(sys.modules, 'rclpy.node', node_module)
    return state


def _install_visualization_stubs(monkeypatch):
    """Install pygame and message stubs for importing the visualization module."""
    pygame = ModuleType('pygame')
    pygame.__path__ = []
    locals_module = ModuleType('pygame.locals')
    locals_module.QUIT = 'QUIT'
    locals_module.KEYDOWN = 'KEYDOWN'
    locals_module.MOUSEMOTION = 'MOUSEMOTION'
    pygame.locals = locals_module

    monkeypatch.setitem(sys.modules, 'pygame', pygame)
    monkeypatch.setitem(sys.modules, 'pygame.locals', locals_module)

    msgs = ModuleType('flexbe_river_crossing_msgs')
    msg_module = ModuleType('flexbe_river_crossing_msgs.msg')

    class WGCFStatus:
        """Visualization message stub."""

    msg_module.WGCFStatus = WGCFStatus
    msgs.msg = msg_module
    monkeypatch.setitem(sys.modules, 'flexbe_river_crossing_msgs', msgs)
    monkeypatch.setitem(sys.modules, 'flexbe_river_crossing_msgs.msg', msg_module)


def _import_msg_handler(monkeypatch):
    """Import MsgHandler against fake rclpy."""
    _install_rclpy_stub(monkeypatch)
    sys.modules.pop('flexbe_river_crossing_visualization.msg_handler', None)
    return importlib.import_module('flexbe_river_crossing_visualization.msg_handler')


def _import_river_crossing(monkeypatch):
    """Import RiverCrossing against fake pygame and ROS modules."""
    _install_rclpy_stub(monkeypatch)
    _install_visualization_stubs(monkeypatch)
    for module_name in (
        'flexbe_river_crossing_visualization.msg_handler',
        'flexbe_river_crossing_visualization.river_crossing',
    ):
        sys.modules.pop(module_name, None)

    return importlib.import_module('flexbe_river_crossing_visualization.river_crossing')


def test_msg_handler_queues_callback_data(monkeypatch):
    """Message handler queues callback data and yields it through the data generator."""
    msg_handler = _import_msg_handler(monkeypatch)
    handler = msg_handler.MsgHandler('test_node', '/status', object, args=['--ros-args'])
    msg = object()

    handler.callback(msg)

    assert not handler.is_empty
    assert next(handler.data_generator()) is msg
    assert handler.is_empty

    handler.shutdown()


def test_river_crossing_applies_latest_ros_message(monkeypatch):
    """Visualization copies message fields into its display state."""
    river_crossing = _import_river_crossing(monkeypatch)
    msg = SimpleNamespace(
        left=['wolf'],
        middle=['farmer'],
        right=['goat'],
        passengers0=[],
        passengers1=[],
        position0=1.0,
        position1=0.25,
    )

    class Handler:
        """Message handler stub with one queued message."""

        is_empty = False

        def data_generator(self):
            """Yield the queued message."""
            yield msg

    played = []
    game = river_crossing.RiverCrossing.__new__(river_crossing.RiverCrossing)
    game.msg_handler = Handler()
    game.game_over = True
    game.left_bank = []
    game.middle_bank = []
    game.right_bank = []
    game.passengers0 = ['farmer']
    game.passengers1 = []
    game.sounds = SimpleNamespace(pfft=SimpleNamespace(play=lambda: played.append('pfft')))

    game.get_current_state()

    assert game.left_bank == ['wolf']
    assert game.middle_bank == ['farmer']
    assert game.right_bank == ['goat']
    assert game.boat0_position == 1.0
    assert game.boat1_position == 0.25
    assert game.passengers0 == []
    assert game.passengers1 == []
    assert not game.game_over
    assert played == ['pfft']


def test_river_crossing_update_draws_banks_and_boats(monkeypatch):
    """Update draws banks and boats from current state."""
    river_crossing = _import_river_crossing(monkeypatch)
    calls = []
    fake_pygame = SimpleNamespace(
        locals=SimpleNamespace(QUIT='QUIT'),
        mouse=SimpleNamespace(set_visible=lambda visible: calls.append(('mouse', visible))),
        transform=SimpleNamespace(scale=lambda background, size: ('scaled', background, size)),
        display=SimpleNamespace(update=lambda: calls.append(('display',))),
    )
    monkeypatch.setattr(river_crossing, 'pygame', fake_pygame)

    class Screen:
        """Screen stub that records blits and conversion."""

        def blit(self, image, position):
            """Record blit calls."""
            calls.append(('blit', image, position))

        def convert_alpha(self):
            """Record alpha conversion."""
            calls.append(('convert_alpha',))

    class Images:
        """Image manager stub that records draw calls."""

        two_rivers_mode = True
        background = 'background'
        characters = {'farmer': object(), 'goat': object(), 'wolf': object()}

        def draw_character(self, screen, bank, character):
            """Record character draw calls."""
            calls.append(('character', bank, character))

        def draw_boat(self, screen, position0, position1, passengers0, passengers1):
            """Record boat draw calls."""
            calls.append(('boat', position0, position1, passengers0, passengers1))

    game = river_crossing.RiverCrossing.__new__(river_crossing.RiverCrossing)
    game.handle_events = lambda: None
    game.get_current_state = lambda: None
    game.screen = Screen()
    game.images = Images()
    game.all_chars = set(game.images.characters.keys())
    game.screensize = (640, 480)
    game.left_bank = ['wolf']
    game.middle_bank = ['farmer']
    game.right_bank = ['goat']
    game.passengers0 = ['corn']
    game.passengers1 = []
    game.boat0_position = 0.5
    game.boat1_position = 0.0
    game.game_over = False

    game.update()

    assert ('character', 'left', 'wolf') in calls
    assert ('character', 'middle', 'farmer') in calls
    assert ('character', 'right', 'goat') in calls
    assert ('boat', 0.5, 0.0, ['corn'], []) in calls
    assert ('display',) in calls
