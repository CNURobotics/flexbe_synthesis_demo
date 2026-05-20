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

"""End-to-end checks for documented river-crossing synthesis demo pairs."""

from dataclasses import dataclass
import multiprocessing
import os
from pathlib import Path
import queue as queue_module
import shutil
import traceback

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover - exercised only outside ROS test envs.
    get_package_share_directory = None

from flexbe_river_crossing_synthesis.request_two_rivers import (
    TWO_RIVERS_DEFAULTS,
)
from flexbe_river_crossing_synthesis.request_two_rivers_capabilities import (
    TWO_RIVERS_CAPABILITIES_DEFAULTS,
)
from flexbe_river_crossing_synthesis.request_two_rivers_capabilities_parsed import (
    TWO_RIVERS_CAPABILITIES_PARSED_DEFAULTS,
)
from flexbe_river_crossing_synthesis.request_two_rivers_parsed import (
    TWO_RIVERS_PARSED_DEFAULTS,
)
from flexbe_river_crossing_synthesis.request_wgcf_capabilities import (
    WGCF_CAPABILITIES_DEFAULTS,
)
from flexbe_river_crossing_synthesis.request_wgcf_capabilities_parsed import (
    WGCF_CAPABILITIES_PARSED_DEFAULTS,
)
from flexbe_river_crossing_synthesis.request_wgcf_full_spec import WGCF_DEFAULTS
from flexbe_synthesis_core.synthesis_manager import FlexBESynthesisActionServer
from flexbe_synthesis_msgs.action import FlexBESynthesis
from flexbe_synthesis_msgs.msg import SynthesisErrorCode
import pytest
import rclpy

PACKAGE_ROOT = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class DemoPair:
    """A documented launch/request pair for the river-crossing synthesis demo."""

    launch_name: str
    server_params: dict
    request_defaults: dict
    expected_state_count: int


class _GoalHandle:
    """Minimal action goal-handle used to call the server callback directly."""

    def __init__(self, request):
        self.request = request
        self.succeeded = False
        self.aborted = False
        self.canceled_called = False
        self.feedback = []

    @property
    def is_cancel_requested(self):
        return False

    def publish_feedback(self, feedback):
        self.feedback.append(feedback)

    def succeed(self):
        self.succeeded = True

    def abort(self):
        self.aborted = True

    def canceled(self):
        self.canceled_called = True


def _package_path(*parts):
    return str(PACKAGE_ROOT.joinpath(*parts))


def _generic_mapping_path(file_name):
    if get_package_share_directory is not None:
        return str(
            Path(get_package_share_directory('flexbe_synthesis_generic'))
            / 'mappings'
            / file_name
        )

    source_root = PACKAGE_ROOT.parents[2] / 'synthesis_sandbox' / 'flexbe_synthesis_generic'
    return str(source_root / 'mappings' / file_name)


def _request_goal(defaults):
    goal = FlexBESynthesis.Goal()
    goal.request.initial_conditions = defaults.get('initial_conditions', [])
    goal.request.goals = defaults['goals']
    goal.request.sm_outcomes = defaults['outcomes']
    goal.request.system_name = defaults['system_name']
    goal.request.spec_name = defaults['spec_name']
    goal.request.synthesis_timeout_s = defaults.get('synthesis_timeout_s', 0.0)
    return goal


def _server_params(system_name, capabilities_file, spec_file, processes_file):
    return {
        'system_name': system_name,
        'global_mappings_path': _generic_mapping_path('global_mappings.yaml'),
        'capabilities_path': _package_path('capabilities', capabilities_file),
        'spec_path': _package_path('specs', spec_file),
        'processes_filepath': _package_path('pipeline', processes_file),
        'processes_data_filepath': _package_path('pipeline', 'processes_data.yaml'),
        'preprocesses_filepath': _package_path('pipeline', 'preprocesses_def.yaml'),
        'preprocesses_data_filepath': _package_path('pipeline', 'preprocesses_data.yaml'),
    }


DEMO_PAIRS = (
    DemoPair(
        'wgcf_synthesis.launch.py',
        _server_params(
            'wgcf_full_spec_demo',
            'wgcf.yaml',
            'wgcf.yaml',
            'processes_def.yaml',
        ),
        WGCF_DEFAULTS,
        expected_state_count=24,
    ),
    DemoPair(
        'wgcf_capabilities.launch.py',
        _server_params(
            'wgcf_capabilities_demo',
            'wgcf.yaml',
            'wgcf_env_spec.yaml',
            'capability_processes_def.yaml',
        ),
        WGCF_CAPABILITIES_DEFAULTS,
        expected_state_count=17,
    ),
    DemoPair(
        'wgcf_capabilities_parsed.launch.py',
        _server_params(
            'wgcf_capabilities_parsed_demo',
            'wgcf.yaml',
            'wgcf_env_spec.yaml',
            'capability_processes_def_parsed.yaml',
        ),
        WGCF_CAPABILITIES_PARSED_DEFAULTS,
        expected_state_count=17,
    ),
    DemoPair(
        'two_rivers.launch.py',
        _server_params(
            'two_rivers',
            'two_rivers.yaml',
            'two_rivers.yaml',
            'processes_def.yaml',
        ),
        TWO_RIVERS_DEFAULTS,
        expected_state_count=33,
    ),
    DemoPair(
        'two_rivers_parsed.launch.py',
        _server_params(
            'two_rivers_parsed',
            'two_rivers_parsed.yaml',
            'two_rivers_parsed.yaml',
            'processes_def.yaml',
        ),
        TWO_RIVERS_PARSED_DEFAULTS,
        expected_state_count=33,
    ),
    DemoPair(
        'two_rivers_capabilities.launch.py',
        _server_params(
            'two_rivers_capabilities',
            'two_rivers_parsed.yaml',
            'two_rivers_env.yaml',
            'capability_processes_def.yaml',
        ),
        TWO_RIVERS_CAPABILITIES_DEFAULTS,
        expected_state_count=23,
    ),
    DemoPair(
        'two_rivers_capabilities_parsed.launch.py',
        _server_params(
            'two_rivers_capabilities_parsed',
            'two_rivers_parsed.yaml',
            'two_rivers_env.yaml',
            'capability_processes_def_parsed.yaml',
        ),
        TWO_RIVERS_CAPABILITIES_PARSED_DEFAULTS,
        expected_state_count=23,
    ),
)


def _ros_args(server_params):
    args = ['--ros-args']
    for key, value in server_params.items():
        if value == '':
            continue
        args.extend(['-p', f'{key}:={value}'])
    return args


def _test_synthesis_home(system_name):
    base_home = os.environ.get('FLEXBE_SYNTHESIS_HOME')
    if not base_home:
        base_home = str(Path.home() / '.flexbe_synthesis')
    return str(Path(base_home) / f'test_{system_name}')


def _execute_demo_pair(demo_pair, ros_log_dir, queue):
    """
    Run a demo pair in a fresh process and return compact result data.

    Synthesis outputs go under FLEXBE_SYNTHESIS_HOME/test_<system_name> so
    parallel test workers cannot share the workspace cache. ROS logs go to
    ros_log_dir (a pytest temp directory).
    """
    os.environ['ROS_LOG_DIR'] = str(ros_log_dir)
    os.environ['FLEXBE_SYNTHESIS_HOME'] = _test_synthesis_home(
        demo_pair.request_defaults['system_name']
    )
    server = None
    try:
        rclpy.init(args=_ros_args(demo_pair.server_params))
        server = FlexBESynthesisActionServer(
            f"test_{demo_pair.launch_name.replace('.', '_')}"
        )
        goal_handle = _GoalHandle(_request_goal(demo_pair.request_defaults))
        result = server.execute_callback(goal_handle)
        queue.put(
            {
                'error_code': result.error_code.value,
                'messages': list(result.messages),
                'state_count': len(result.states),
                'succeeded': goal_handle.succeeded,
                'aborted': goal_handle.aborted,
                'canceled_called': goal_handle.canceled_called,
            }
        )
    except BaseException:  # noqa: BLE001 - propagate child-process failures.
        queue.put({'exception': traceback.format_exc()})
    finally:
        if server is not None:
            server.destroy()
        if rclpy.ok():
            rclpy.shutdown()


def _run_demo_pair(demo_pair, tmp_path):
    """
    Execute one synthesis pair in an isolated child process.

    Synthesis outputs are grouped below the normal synthesis home by system name
    so overnight runs remain inspectable.
    """
    if shutil.which('slugs') is None:
        pytest.skip('Slugs binary is not installed')

    ctx = multiprocessing.get_context('spawn')
    queue = ctx.Queue()
    process = ctx.Process(
        target=_execute_demo_pair,
        args=(demo_pair, tmp_path / 'ros_logs', queue),
    )
    process.start()
    process.join(timeout=30)

    if process.is_alive():
        process.terminate()
        process.join()
        pytest.fail(f'{demo_pair.launch_name} did not finish within 30 seconds')

    assert process.exitcode == 0, \
        f'{demo_pair.launch_name} child exited with code {process.exitcode}'
    try:
        return queue.get(timeout=5)
    except queue_module.Empty:
        pytest.fail(f'{demo_pair.launch_name}: child exited but wrote nothing to queue')


@pytest.mark.parametrize(
    'demo_pair',
    DEMO_PAIRS,
    ids=lambda pair: pair.launch_name,
)
def test_synthesis_server_request_pair_succeeds(demo_pair, tmp_path):
    """Verify each documented launch/request pair synthesizes successfully."""
    result = _run_demo_pair(demo_pair, tmp_path)

    assert 'exception' not in result, result.get('exception')
    assert result['error_code'] == SynthesisErrorCode.SUCCESS
    assert result['messages'] == [], result['messages']
    assert result['state_count'] == demo_pair.expected_state_count
    assert result['succeeded']
    assert not result['aborted']
    assert not result['canceled_called']
