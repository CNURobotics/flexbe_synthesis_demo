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

"""Import-only tests for PyRoboSim FlexBE states."""

import os
from pathlib import Path
import tempfile

from flexbe_testing.py_tester import PyTester


class TestPyRoboSimFlexBEStateImports(PyTester):
    """Run FlexBE import-only checks for each state."""

    @classmethod
    def setUpClass(cls):
        """Set the package and test folder for FlexBE test files."""
        log_dir = Path(tempfile.gettempdir()) / 'pyrobosim_flexbe_states_test_logs'
        log_dir.mkdir(parents=True, exist_ok=True)
        os.environ.setdefault('ROS_LOG_DIR', str(log_dir))

        PyTester._package = 'pyrobosim_flexbe_states'
        PyTester._tests_folder = 'test'
        PyTester.setUpClass()

    def test_check_door_state_import(self):
        """Import CheckDoorState."""
        self.run_test('check_door_state_import')

    def test_detect_local_objects_state_import(self):
        """Import DetectLocalObjectsState."""
        self.run_test('detect_local_objects_state_import')

    def test_detect_objects_state_import(self):
        """Import DetectObjectsState."""
        self.run_test('detect_objects_state_import')

    def test_door_action_state_import(self):
        """Import DoorActionState."""
        self.run_test('door_action_state_import')

    def test_follow_path_state_import(self):
        """Import FollowPathState."""
        self.run_test('follow_path_state_import')

    def test_monitor_battery_state_import(self):
        """Import MonitorBatteryState."""
        self.run_test('monitor_battery_state_import')

    def test_navigate_action_state_import(self):
        """Import NavigateActionState."""
        self.run_test('navigate_action_state_import')

    def test_next_room_state_import(self):
        """Import NextRoomState."""
        self.run_test('next_room_state_import')

    def test_pick_action_state_import(self):
        """Import PickActionState."""
        self.run_test('pick_action_state_import')

    def test_place_action_state_import(self):
        """Import PlaceActionState."""
        self.run_test('place_action_state_import')

    def test_plan_path_state_import(self):
        """Import PlanPathState."""
        self.run_test('plan_path_state_import')
