# Copyright 2026 Christopher Newport University
# Capable Humanitarian Robotics and Intelligent Systems Lab (CHRISLAB)
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

"""Smoke tests for checked-in WGCF demo launch files."""

import importlib.util
from pathlib import Path

from launch import LaunchDescription
import pytest

LAUNCH_DIR = Path(__file__).resolve().parents[1] / 'launch'
LAUNCH_FILES = tuple(sorted(LAUNCH_DIR.glob('*.launch.py')))


def _load_launch_module(launch_path: Path):
    module_name = f'test_launch_{launch_path.stem.replace(".", "_")}'
    spec = importlib.util.spec_from_file_location(module_name, launch_path)
    assert spec is not None
    assert spec.loader is not None

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('launch_path', LAUNCH_FILES, ids=lambda path: path.name)
def test_launch_description_can_be_generated(launch_path, monkeypatch, tmp_path):
    """Import each launch file and build its launch description."""
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_log'))
    module = _load_launch_module(launch_path)

    launch_description = module.generate_launch_description()

    assert isinstance(launch_description, LaunchDescription)
    assert len(launch_description.entities) > 0
