#!/usr/bin/env python3

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

"""Request synthesis for the Two Rivers capabilities demo via action client."""

from flexbe_synthesis_core.request_synthesis import run_synthesis_client

TWO_RIVERS_CAPABILITIES_DEFAULTS = {
    'goals': ['log_finished'],
    'outcomes': ['finished'],
    'system_name': 'two_rivers_capabilities',
    'spec_name': 'two_rivers',
}


def main(args=None):
    """Run Two Rivers capabilities request client."""
    run_synthesis_client(args, defaults=TWO_RIVERS_CAPABILITIES_DEFAULTS)


if __name__ == '__main__':
    main()
