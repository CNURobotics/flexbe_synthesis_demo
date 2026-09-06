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

"""Generate Two Rivers experiment matrix YAML for the Slugs batch harness."""

from __future__ import annotations

import argparse
from pathlib import Path

import yaml


def package_root() -> Path:
    """Return the flexbe_river_crossing_synthesis package source root."""
    return Path(__file__).resolve().parents[1]


def _reordering_treatments():
    """Return the CUDD reordering policies used for the current release sweeps."""
    return [
        {
            'reordering_enabled': False,
            'reordering_threshold': None,
        },
        {
            'reordering_enabled': True,
            'reordering_threshold': 250,
        },
    ]


def ordering_rows(full=False, random_seeds=40):
    """Return ordering and reordering treatments for Two Rivers."""
    rows = [
        {
            'ordering_mode': ordering_mode,
            'ordering_seed': None,
            **reordering,
        }
        for ordering_mode in ('alphabetic', 'domain')
        for reordering in _reordering_treatments()
    ]
    if full:
        rows.extend(
            {
                'ordering_mode': 'random',
                'ordering_seed': seed,
                **reordering,
            }
            for reordering in _reordering_treatments()
            for seed in range(random_seeds)
        )
    else:
        rows.extend(
            {
                'ordering_mode': 'random',
                'ordering_seed': 0,
                **reordering,
            }
            for reordering in _reordering_treatments()
        )
    return rows


def _full_spec_row(
    *,
    system_name,
    spec_path,
    capabilities_path,
    encoding,
    ordering,
    n_trials,
):
    """Build one hand-written full-spec baseline row."""
    return {
        'example': 'two_rivers',
        'system_name': system_name,
        'spec_name': 'two_rivers',
        'spec_path': str(spec_path),
        'capabilities_path': str(capabilities_path),
        'encoding': encoding,
        'liveness': 'na',
        'pending': False,
        'full_spec': True,
        'initial_conditions': [],
        'goals': [],
        'sm_outcomes': ['finished'],
        'n_trials': n_trials,
        **ordering,
    }


_ENCODING_DIGIT = {'one-hot': '2', 'enumerated': '3'}


def _capability_row(root, *, encoding, liveness, pending, ordering, n_trials):
    """Build one capability-generated Two Rivers matrix row."""
    digit = _ENCODING_DIGIT[encoding]
    return {
        'example': 'two_rivers',
        'system_name': (
            'two_rivers_capabilities_parsed'
            if encoding == 'enumerated' else 'two_rivers_capabilities'
        ),
        'spec_name': 'two_rivers',
        'spec_path': str(root / 'specs' / 'two_rivers_env.yaml'),
        'capabilities_path': str(root / 'capabilities' / 'two_rivers_parsed.yaml'),
        'encoding': encoding,
        'liveness': liveness,
        'pending': pending,
        'column_indicator': f"{digit}{liveness}{'P' if pending else ''}",
        'initial_conditions': [],
        'goals': ['log_finished'],
        'audit_goals': ['finished'],
        'sm_outcomes': ['finished'],
        'n_trials': n_trials,
        **ordering,
    }


def build_matrix(full=False, random_seeds=40, smoke=False, n_trials=5):
    """Build Two Rivers matrix rows."""
    root = package_root()
    rows = []
    orderings = ordering_rows(full=full, random_seeds=random_seeds)

    for encoding in ('one-hot', 'enumerated'):
        for liveness in ('S', 'F'):
            for pending in (False, True):
                for ordering in orderings:
                    rows.append(
                        _capability_row(
                            root,
                            encoding=encoding,
                            liveness=liveness,
                            pending=pending,
                            ordering=ordering,
                            n_trials=n_trials,
                        )
                    )
                    if smoke:
                        rows.append(
                            _full_spec_row(
                                system_name='two_rivers',
                                spec_path=root / 'specs' / 'two_rivers.yaml',
                                capabilities_path=root / 'capabilities' / 'two_rivers.yaml',
                                encoding='full_spec',
                                ordering=ordering,
                                n_trials=n_trials,
                            )
                        )
                        rows.append(
                            _full_spec_row(
                                system_name='two_rivers_parsed',
                                spec_path=root / 'specs' / 'two_rivers_parsed.yaml',
                                capabilities_path=root / 'capabilities' / 'two_rivers_parsed.yaml',
                                encoding='full_spec_parsed',
                                ordering=ordering,
                                n_trials=n_trials,
                            )
                        )
                        return {'rows': rows}

    for ordering in orderings:
        rows.append(
            _full_spec_row(
                system_name='two_rivers',
                spec_path=root / 'specs' / 'two_rivers.yaml',
                capabilities_path=root / 'capabilities' / 'two_rivers.yaml',
                encoding='full_spec',
                ordering=ordering,
                n_trials=n_trials,
            )
        )
        rows.append(
            _full_spec_row(
                system_name='two_rivers_parsed',
                spec_path=root / 'specs' / 'two_rivers_parsed.yaml',
                capabilities_path=root / 'capabilities' / 'two_rivers_parsed.yaml',
                encoding='full_spec_parsed',
                ordering=ordering,
                n_trials=n_trials,
            )
        )
    return {'rows': rows}


def main():
    """Run the Two Rivers matrix generator CLI."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', help='Matrix YAML output path.')
    parser.add_argument(
        '--full',
        action='store_true',
        help='Generate the full ordering matrix: 8 capability cells plus '
             '2 hand-written baselines, off + threshold_250 policies, '
             'and 40 random seeds.',
    )
    parser.add_argument(
        '--smoke',
        action='store_true',
        help='Generate one capability row plus both full-spec baseline rows.',
    )
    parser.add_argument(
        '--random-seeds',
        type=int,
        default=40,
        help='Number of random ordering seeds for --full.',
    )
    parser.add_argument(
        '--n-trials',
        type=int,
        default=5,
        help='Exact repeated trials per matrix row.',
    )
    args = parser.parse_args()

    matrix = build_matrix(
        full=args.full,
        random_seeds=args.random_seeds,
        smoke=args.smoke,
        n_trials=args.n_trials,
    )
    output = Path(args.output).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(yaml.safe_dump(matrix, sort_keys=False), encoding='utf-8')
    print(f"Wrote {len(matrix['rows'])} Two Rivers matrix row(s) to '{output}'")


if __name__ == '__main__':
    main()
