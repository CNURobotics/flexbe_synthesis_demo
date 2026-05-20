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

"""Pytest configuration for PyRoboSim synthesis tests."""

import os

import pytest


def pytest_addoption(parser):
    """Register PyRoboSim synthesis test options."""
    parser.addoption(
        '--pyrobosim-synthesis-timeout-s',
        action='store',
        default=None,
        help='Synthesis timeout, in seconds, for PyRoboSim synthesis demo-pair tests.',
    )
    parser.addoption(
        '--pyrobosim-slugs-timeout-s',
        action='store',
        default=None,
        help='Deprecated alias for --pyrobosim-synthesis-timeout-s.',
    )


@pytest.fixture
def pyrobosim_synthesis_timeout_s(pytestconfig):
    """Return the synthesis timeout used by synthesis demo-pair tests."""
    raw_timeout = (
        pytestconfig.getoption('--pyrobosim-synthesis-timeout-s')
        or pytestconfig.getoption('--pyrobosim-slugs-timeout-s')
        or os.environ.get('PYROBOSIM_SYNTHESIS_TIMEOUT_S')
        or os.environ.get('PYROBOSIM_SLUGS_TIMEOUT_S')
        or '10.0'
    )
    timeout_s = float(raw_timeout)
    if timeout_s <= 0:
        raise ValueError('pyrobosim synthesis timeout must be greater than zero')
    return timeout_s
