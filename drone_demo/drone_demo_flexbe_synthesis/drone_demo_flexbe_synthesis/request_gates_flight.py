#!/usr/bin/env python3
"""Request synthesis for the portable four-gate drone demonstration."""

from flexbe_synthesis_core.request_synthesis import run_synthesis_client


GATES_FLIGHT_DEFAULTS = {
    'initial_conditions': [
        'begin_game_a',
        'gate1=0',
        'gate2=11',
        'gate3=6',
        'gate4=13',
    ],
    'goals': ['log_finished'],
    'outcomes': ['finished', 'failed'],
    'system_name': 'drone_gates_demo',
    'spec_name': 'drones_gates',
    'synthesis_timeout_s': 300.0,
}


def main(args=None):
    """Run the four-gate synthesis request client."""
    run_synthesis_client(args, defaults=GATES_FLIGHT_DEFAULTS)


if __name__ == '__main__':
    main()
