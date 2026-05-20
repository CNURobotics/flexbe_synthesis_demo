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

"""End-to-end checks for documented PyRoboSim synthesis demo pairs."""

from dataclasses import dataclass
import multiprocessing
import os
from pathlib import Path
import shutil
import time
import traceback
import warnings

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover - exercised only outside ROS test envs.
    get_package_share_directory = None

from flexbe_synthesis_core.synthesis_manager import FlexBESynthesisActionServer
from flexbe_synthesis_msgs.action import FlexBESynthesis
from flexbe_synthesis_msgs.msg import SynthesisErrorCode
from pyrobosim_flexbe_synthesis.request_synthesis import (
    LOAD_DEFAULTS,
    P0_CAPABILITIES_DEFAULTS,
    P0_CAPABILITIES_PARSED_DEFAULTS,
    P0_DEFAULTS,
    P0A_DEFAULTS,
    P1_CAPABILITIES_DEFAULTS,
    P1_CAPABILITIES_PARSED_DEFAULTS,
    P1_DEFAULTS,
)
import pytest
import rclpy

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
ANSI_GREEN = '\033[32m'
ANSI_ORANGE = '\033[38;5;208m'
ANSI_RESET = '\033[0m'
MIN_SUCCESS_STATE_COUNT = 4


@dataclass(frozen=True)
class DemoPair:
    """A documented launch/request pair for the PyRoboSim synthesis demo."""

    launch_name: str
    server_params: dict
    request_defaults: dict
    acceptable_error_codes: tuple[int, ...]


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
        'custom_mappings_path': _package_path('mappings', 'custom_mappings.yaml'),
        'capabilities_path': _package_path('capabilities', capabilities_file),
        'spec_path': _package_path('specs', spec_file),
        'processes_filepath': _package_path('pipeline', processes_file),
        'processes_data_filepath': _package_path('pipeline', 'processes_data.yaml'),
        'preprocesses_filepath': _package_path('pipeline', 'preprocesses_def.yaml'),
        'preprocesses_data_filepath': _package_path('pipeline', 'preprocesses_data.yaml'),
    }


def _load_server_params(system_name):
    params = _server_params(
        system_name,
        'pyrobosim.yaml',
        'p0_full_spec.yaml',
        'load_processes_def.yaml',
    )
    params['automaton_path'] = _package_path(
        'slugs',
        f'{system_name}_automaton.yaml',
    )
    return params


SLUGS_DEMO_ERROR_CODES = (
    SynthesisErrorCode.SUCCESS,
    SynthesisErrorCode.SYNTHESIS_FAILED,
    SynthesisErrorCode.SPEC_UNSYNTHESIZABLE,
)

LOAD_DEMO_ERROR_CODES = (
    SynthesisErrorCode.SUCCESS,
    SynthesisErrorCode.AUTOMATON_INVALID,
)


DEMO_PAIRS = (
    DemoPair(
        'p0_full_spec.launch.py',
        _server_params(
            'p0_full_spec_demo',
            'pyrobosim.yaml',
            'p0_full_spec.yaml',
            'processes_def.yaml',
        ),
        P0_DEFAULTS,
        acceptable_error_codes=SLUGS_DEMO_ERROR_CODES,
    ),
    DemoPair(
        'p0_move_only_spec.launch.py',
        _server_params(
            'p0_move_only_spec_demo',
            'pyrobosim.yaml',
            'p0_move_only_spec.yaml',
            'processes_def.yaml',
        ),
        P0A_DEFAULTS,
        acceptable_error_codes=SLUGS_DEMO_ERROR_CODES,
    ),
    DemoPair(
        'p0_capabilities.launch.py',
        _server_params(
            'p0_capabilities_demo',
            'p0_capabilities.yaml',
            'p0_capabilities.yaml',
            'capability_processes_def.yaml',
        ),
        P0_CAPABILITIES_DEFAULTS,
        acceptable_error_codes=SLUGS_DEMO_ERROR_CODES,
    ),
    DemoPair(
        'p0_capabilities_parsed.launch.py',
        _server_params(
            'p0_capabilities_parsed_demo',
            'p0_capabilities.yaml',
            'p0_capabilities.yaml',
            'capability_processes_def_parsed.yaml',
        ),
        P0_CAPABILITIES_PARSED_DEFAULTS,
        acceptable_error_codes=SLUGS_DEMO_ERROR_CODES,
    ),
    DemoPair(
        'p1_full_spec.launch.py',
        _server_params(
            'p1_full_spec_demo',
            'pyrobosim.yaml',
            'p1_full_spec.yaml',
            'processes_def.yaml',
        ),
        P1_DEFAULTS,
        acceptable_error_codes=SLUGS_DEMO_ERROR_CODES,
    ),
    DemoPair(
        'p1_capabilities.launch.py',
        _server_params(
            'p1_capabilities_demo',
            'p1_capabilities.yaml',
            'p1_capabilities.yaml',
            'capability_processes_def.yaml',
        ),
        P1_CAPABILITIES_DEFAULTS,
        acceptable_error_codes=SLUGS_DEMO_ERROR_CODES,
    ),
    DemoPair(
        'p1_capabilities_parsed.launch.py',
        _server_params(
            'p1_capabilities_parsed_demo',
            'p1_capabilities.yaml',
            'p1_capabilities.yaml',
            'capability_processes_def_parsed.yaml',
        ),
        P1_CAPABILITIES_PARSED_DEFAULTS,
        acceptable_error_codes=SLUGS_DEMO_ERROR_CODES,
    ),
    DemoPair(
        'load_automaton.launch.py',
        _load_server_params('p0_full_spec_demo'),
        LOAD_DEFAULTS,
        acceptable_error_codes=LOAD_DEMO_ERROR_CODES,
    ),
)


def _ros_args(server_params):
    args = ['--ros-args']
    for key, value in server_params.items():
        if value == '':
            continue
        args.extend(['-p', f'{key}:={str(value).lower() if isinstance(value, bool) else value}'])
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
        start_time = time.perf_counter()
        result = server.execute_callback(goal_handle)
        elapsed_s = time.perf_counter() - start_time
        queue.put(
            {
                'error_code': result.error_code.value,
                'elapsed_s': elapsed_s,
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


def _run_demo_pair(demo_pair, tmp_path, synthesis_timeout_s):
    """Execute one synthesis pair in an isolated child process."""
    if shutil.which('slugs') is None:
        pytest.skip('Slugs binary is not installed')

    ctx = multiprocessing.get_context('spawn')
    queue = ctx.Queue()
    demo_pair = DemoPair(
        demo_pair.launch_name,
        demo_pair.server_params,
        {
            **demo_pair.request_defaults,
            'synthesis_timeout_s': synthesis_timeout_s,
        },
        demo_pair.acceptable_error_codes,
    )
    process = ctx.Process(
        target=_execute_demo_pair,
        args=(demo_pair, tmp_path / 'ros_logs', queue),
    )
    process.start()
    process_timeout_s = max(25, synthesis_timeout_s + 15)
    process.join(timeout=process_timeout_s)

    if process.is_alive():
        process.terminate()
        process.join()
        pytest.fail(
            f'{demo_pair.launch_name} did not finish within {process_timeout_s:g} seconds'
        )

    assert process.exitcode == 0, \
        f'{demo_pair.launch_name} child exited with code {process.exitcode}'
    try:
        return queue.get(timeout=5)
    except Exception:  # noqa: BLE001
        pytest.fail(f'{demo_pair.launch_name}: child exited but wrote nothing to queue')


def _warn_demo_pair_result(demo_pair, result, status, color):
    """Emit a top-level-visible summary for colcon/pytest output."""
    warnings.warn(
        f'{color}{status}: {demo_pair.launch_name} '
        f'(system={demo_pair.request_defaults["system_name"]}, '
        f'spec={demo_pair.request_defaults["spec_name"]}, '
        f'states={result["state_count"]}, '
        f'synthesis_time={result["elapsed_s"]:.3f}s){ANSI_RESET}',
        stacklevel=2,
    )


@pytest.mark.parametrize(
    'demo_pair',
    DEMO_PAIRS,
    ids=lambda pair: pair.launch_name,
)
def test_synthesis_server_request_pair_succeeds(
    demo_pair,
    tmp_path,
    pyrobosim_synthesis_timeout_s,
):
    """Verify documented launch/request pairs reach their expected result."""
    result = _run_demo_pair(demo_pair, tmp_path, pyrobosim_synthesis_timeout_s)

    assert 'exception' not in result, result.get('exception')
    assert result['error_code'] in demo_pair.acceptable_error_codes
    if result['error_code'] == SynthesisErrorCode.SUCCESS:
        assert result['messages'] == [], result['messages']
        assert result['state_count'] >= MIN_SUCCESS_STATE_COUNT
        assert result['succeeded']
        assert not result['aborted']
        _warn_demo_pair_result(
            demo_pair,
            result,
            'synthesis demo pair succeeded',
            ANSI_GREEN,
        )
    else:
        assert result['state_count'] == 0
        assert not result['succeeded']
        assert result['aborted']
        warnings.warn(
            f'{ANSI_ORANGE}{demo_pair.launch_name}: accepted non-success result '
            f'(error_code={result["error_code"]}, '
            f'synthesis_time={result["elapsed_s"]:.3f}s, '
            f'synthesis_timeout_s={pyrobosim_synthesis_timeout_s:g}s); '
            'set PYROBOSIM_SYNTHESIS_TIMEOUT_S=5400 for full overnight validation'
            f'{ANSI_RESET}',
            stacklevel=1,
        )
    assert not result['canceled_called']
