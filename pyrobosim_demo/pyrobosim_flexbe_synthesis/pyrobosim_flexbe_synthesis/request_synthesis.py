#!/usr/bin/env python3

# Copyright 2024 Christopher Newport University
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

"""Request synthesis for the PyRoboSim demo via action client."""

from flexbe_synthesis_core.request_synthesis import FlexBESynthesisActionClient
import rclpy

_DEFAULT_GOALS = ['bread0=3', 'butter0=3']
_DEFAULT_OUTCOMES = ['finished']
_DEFAULT_SYNTHESIS_TIMEOUT_S = 0.0
_SYSTEM_SPEC_NAMES = {
    'p0_full_spec_demo': 'p0_full_spec',
    'p0_move_only_spec_demo': 'p0_move_only_spec',
    'p1_full_spec_demo': 'p1_full_spec',
    'p0_capabilities_demo': 'p0_capabilities',
    'p1_capabilities_demo': 'p1_capabilities',
    'p0_capabilities_parsed_demo': 'p0_parsed',
    'p1_capabilities_parsed_demo': 'p1_parsed',
}


def _request_defaults(defaults):
    """Add PyRoboSim request defaults common to all demo variants."""
    return {
        **defaults,
        'synthesis_timeout_s': _DEFAULT_SYNTHESIS_TIMEOUT_S,
    }


P0_DEFAULTS = _request_defaults({
    'goals': _DEFAULT_GOALS,
    'outcomes': _DEFAULT_OUTCOMES,
    'system_name': 'p0_full_spec_demo',
    'spec_name': 'p0_full_spec',
})

P0A_DEFAULTS = _request_defaults({
    'goals': _DEFAULT_GOALS,
    'outcomes': _DEFAULT_OUTCOMES,
    'system_name': 'p0_move_only_spec_demo',
    'spec_name': 'p0_move_only_spec',
})

P1_DEFAULTS = _request_defaults({
    'goals': _DEFAULT_GOALS,
    'outcomes': _DEFAULT_OUTCOMES,
    'system_name': 'p1_full_spec_demo',
    'spec_name': 'p1_full_spec',
})

P0_CAPABILITIES_DEFAULTS = _request_defaults({
    'goals': _DEFAULT_GOALS,
    'outcomes': _DEFAULT_OUTCOMES,
    'system_name': 'p0_capabilities_demo',
    'spec_name': 'p0_capabilities',
})

P1_CAPABILITIES_DEFAULTS = _request_defaults({
    'goals': _DEFAULT_GOALS,
    'outcomes': _DEFAULT_OUTCOMES,
    'system_name': 'p1_capabilities_demo',
    'spec_name': 'p1_capabilities',
})

P0_CAPABILITIES_PARSED_DEFAULTS = _request_defaults({
    'goals': _DEFAULT_GOALS,
    'outcomes': _DEFAULT_OUTCOMES,
    'system_name': 'p0_capabilities_parsed_demo',
    'spec_name': 'p0_parsed',
})

P1_CAPABILITIES_PARSED_DEFAULTS = _request_defaults({
    'goals': _DEFAULT_GOALS,
    'outcomes': _DEFAULT_OUTCOMES,
    'system_name': 'p1_capabilities_parsed_demo',
    'spec_name': 'p1_parsed',
})

LOAD_DEFAULTS = _request_defaults({
    'goals': _DEFAULT_GOALS,
    'outcomes': _DEFAULT_OUTCOMES,
    'system_name': 'p0_full_spec_demo',
    'spec_name': 'p0_full_spec',
})

PYROBOSIM_DEFAULTS = P0_DEFAULTS


class PyRoboSimSynthesisActionClient(FlexBESynthesisActionClient):
    """PyRoboSim request client with a synthesis timeout request parameter."""

    def __init__(self, defaults=None):
        """Declare PyRoboSim-specific request parameters."""
        super().__init__(defaults=defaults)
        default_timeout = (defaults or {}).get(
            'synthesis_timeout_s',
            _DEFAULT_SYNTHESIS_TIMEOUT_S,
        )
        if not self.has_parameter('synthesis_timeout_s'):
            self.declare_parameter('synthesis_timeout_s', default_timeout)
        self.goal_msg.request.synthesis_timeout_s = self._double_param(
            'synthesis_timeout_s'
        )

        # Auto-derive spec_name from system_name when system_name is overridden
        # but spec_name is not (i.e., spec_name still matches the default).
        d = defaults or {}
        actual_system = self._string_param('system_name')
        actual_spec = self._string_param('spec_name')
        default_system = d.get('system_name', '')
        default_spec = d.get('spec_name', '')
        if actual_system != default_system and actual_spec == default_spec:
            self.goal_msg.request.spec_name = _SYSTEM_SPEC_NAMES.get(
                actual_system,
                actual_system.removesuffix('_demo'),
            )


def run_pyrobosim_synthesis_client(args, defaults=None):
    """Initialize rclpy, send a PyRoboSim synthesis request, and spin until done."""
    rclpy.init(args=args)
    action_client = PyRoboSimSynthesisActionClient(defaults=defaults)
    try:
        if action_client.connect_and_send():
            while rclpy.ok() and not action_client.request_complete:
                rclpy.spin_once(action_client, timeout_sec=0.1)
    except (RuntimeError, ValueError) as exc:
        print(f'Error: {exc}')
    finally:
        if rclpy.ok():
            action_client.destroy_node()
            rclpy.shutdown()
    print('Done!')


def main(args=None):
    """Run PyRoboSim synthesis request client."""
    run_pyrobosim_synthesis_client(args, defaults=PYROBOSIM_DEFAULTS)


if __name__ == '__main__':
    main()
