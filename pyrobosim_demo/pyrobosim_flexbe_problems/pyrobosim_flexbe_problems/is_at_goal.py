# Copyright (c) 2024, Sebastian Castro, The AI Institute
# SPDX-License-Identifier: BSD-3-Clause
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

"""Check whether a problem goal has been reached."""

import sys

from pyrobosim_flexbe_problems.world_state import WorldState


def get_goal_state(problem_number):
    """Return the goal state for a problem number."""
    if problem_number == 1:
        return (('objects.snacks0.parent', 'table_tabletop'),)
    elif problem_number == 2:
        return (
            ('objects.waste0.parent', 'dumpster_disposal'),
            ('objects.waste1.parent', 'dumpster_disposal'),
            ('locations.dumpster.is_open', False),
        )
    elif problem_number == 3:
        return (
            ('objects.bread0.parent', 'table_tabletop'),
            ('objects.butter0.parent', 'table_tabletop'),
            ('locations.fridge.is_open', False),
            ('locations.pantry.is_open', False),
        )
    elif problem_number == 4:
        return get_goal_state(2) + get_goal_state(3)
    else:
        raise ValueError(f'No goal state for problem number: {problem_number}')


def main():
    ws = WorldState()
    expected_state = get_goal_state(ws.get_problem_number())
    is_at_goal = ws.check_state(expected_state)
    print(f'Is at goal: {is_at_goal}')
    sys.exit(0)
